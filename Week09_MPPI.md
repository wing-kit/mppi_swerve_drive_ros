### Week 9 — Advanced Control: Model Predictive Path-Integral (MPPI)

Instructor: [Your Name]

Course: Robotics and Computer Science — Planning and Control

Duration: 1 week (lecture + lab)


### Learning objectives

- Understand the intuition and mathematics behind sampling-based stochastic optimal control for nonlinear systems.
- Learn the Model Predictive Path-Integral (MPPI) control algorithm and how it differs from classical MPC.
- Read and interpret MPPI controller implementations for a 4WIDS (Four-Wheel Independent Drive and Steering) vehicle.
- Compare MPPI variants: MPPI-3D(a), MPPI-3D(b), MPPI-4D, and MPPI-H (hybrid switching controller).
- Run Gazebo demos and analyze performance/safety trade-offs; modify parameters to see the impact.


### Required readings and resources

- Project README of the provided repository (architecture, setup, launch commands).
- IROS 2024 paper (Aoki et al.) “Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation.”
  - PDF: [IROS 2024 paper PDF](https://mizuhoaoki.github.io/media/papers/IROS2024_paper_mizuhoaoki.pdf)
  - Arxiv: [Arxiv entry](https://arxiv.org/abs/2409.08648)
  - IEEE Xplore: [IROS 2024 article on IEEE Xplore](https://ieeexplore.ieee.org/document/10802359)


## 1. Conceptual background: sampling-based control for nonlinear systems

Model Predictive Control (MPC) formulates a finite-horizon optimal control problem that is re-solved online at each control step. Classical MPC uses linearization or nonlinear programming and often requires differentiable dynamics and costs. Sampling-based control, by contrast, replaces gradient-based optimization with Monte Carlo sampling of control sequences and weighting by trajectory cost. This is particularly attractive when:

- Dynamics are nonlinear and difficult to linearize effectively.
- Costs are nonconvex, discontinuous, or include black-box components (e.g., collision penalties from maps).
- We can exploit parallelism (CPU/GPU) to evaluate many rollouts quickly.

Path-Integral control (PI) connects stochastic optimal control with statistical physics: control updates resemble importance-weighted averages over noise-perturbed rollouts, with temperature-like parameter λ governing exploration/exploitation. MPPI makes this practical for real-time model predictive settings by:

- Sampling K control sequences of length T around a nominal sequence u_opt.
- Propagating system dynamics for each sequence to evaluate the cumulative state-control cost.
- Computing importance weights via an exponential transform of costs (Boltzmann distribution).
- Updating the nominal sequence by the weighted average of injected noise.
- Applying only the first control input to the system (receding horizon), then repeating each control interval.

Key advantages:
- Naturally handles nonlinear dynamics and non-differentiable costs.
- Parallelizable (e.g., OpenMP or GPU acceleration) across rollouts.
- Flexible cost design (map-derived collision, distance error, yaw alignment, command-change penalties, etc.).

Trade-offs:
- Sampling burden (K×T rollouts per step) — computational cost must meet real-time deadlines.
- Hyperparameters (noise scales σ, λ, α, exploration fraction) require tuning.
- No deterministic optimality guarantees; performance depends on sampling coverage and cost shaping.


## 2. MPPI algorithm: steps and mapping to the codebase

We study implementations for a 4WIDS robot with variants sampling in different control spaces. The general algorithmic skeleton is identical across variants. Using `mppi_3d` (3D input space) as a concrete reference, the core logic is in `solveMPPI`:

Pseudocode (generic MPPI):
```python
# Given observed_state x0, maps (collision, distance error, ref yaw), goal
# Current nominal control sequence u_opt_seq (length T)

if near_goal(x0, goal, tol):
    return zeros_control()

# Optionally refresh noise tensor if reduce_computation == False
noises = generate_noise_matrix(sigma, K, T, UDIM)

# Parallel over K samples
for k in 0..K-1:
    x = x0
    cost[k] = 0
    for t in 0..T-1:
        u_k_t = exploit_explore(u_opt_seq[t], noises[k][t], exploration)
        x = f(x, u_k_t, dt)  # rollout dynamics
        cost[k] += stage_cost(x, u_k_t, prev_u, maps, goal, params)
        cost[k] += lambda*(1 - alpha) * u_opt_seq[t]^T * Sigma^{-1} * u_k_t
    cost[k] += terminal_cost(x, goal, params)

# Importance weights (Boltzmann)
min_cost = min(cost)
eta = sum(exp(-(cost[k]-min_cost)/lambda))
for k: w[k] = exp(-(cost[k]-min_cost)/lambda)/eta

# Update nominal control sequence by noise superposition
u_new = u_opt_seq
for t in 0..T-1:
    for k in 0..K-1:
        u_new[t] += w[k] * noises[k][t]

# Optional Savitzky-Golay smoothing on u_new[0]
u_new[0] = SGFilter(u_log, u_new)

# Clip controls to bounds
clip(u_new)

# Recompute optimal state trajectory and state_cost for reporting
traj = rollout(x0, u_new)

return u_new[0]
```

Code mapping (MPPI-3D):
- Core class: `src/control/mppi_3d/src/mppi_3d_core.cpp`, method `controller::MPPICore::solveMPPI(...)`.
- Dynamics: `target_system::calcNextState(...)` in `mppi_3d_setting.hpp`.
- Costs: `controller::stage_cost(...)` and `controller::terminal_cost(...)` in `mppi_3d_setting.hpp`.
- Noise: `generateNoiseMatrix(...)` uses `std::normal_distribution` with per-input σ.
- Weights: `calcWeightsOfSamples(...)` computes Boltzmann weights.
- Filtering: Savitzky–Golay (`initSaviskyGolayFilter`, `applySaviskyGolayFilter`).
- Parallelism: OpenMP pragmas over samples and noise generation.

The MPPI-4D and MPPI-H code mirrors this skeleton, differing primarily in control dimensionality (4D vs 3D) and in hybrid mode selection logic.


## 3. System model and control spaces

### 3.1 State and control (3D variants)

- State: `(x, y, yaw)` abbreviated as `XYYaw`.
- Control: `(vx, vy, omega)` in the body frame.
- Discrete-time dynamics (from `mppi_3d_setting.hpp`):

```cpp
next_state.x = x + vx*cos(yaw)*dt - vy*sin(yaw)*dt;
next_state.y = y + vx*sin(yaw)*dt + vy*cos(yaw)*dt;
next_state.yaw = yaw + omega*dt;
```

This model supports holonomic lateral velocity `vy` and angular rate `omega`. The 8-DoF wheel actuation is recovered for logging/visualization via a kinematic conversion (`convertControlSpace3DToControlSpace8D`).

### 3.2 Control (4D variant)

MPPI-4D samples directly in a 4D wheel command space:

- `fl_steer` (front-left steer angle), `rr_steer` (rear-right steer angle),
- `fl_vel` (front-left wheel speed), `rr_vel` (rear-right wheel speed).

These are converted to `(vx, vy, omega)` via geometric relationships (`mppi_4d_setting.hpp`). This sampling space better reflects actuator feasibility and can be safer (fewer aggressive body-frame commands), though it can be slower due to the constrained mapping from wheel inputs to body velocities.

### 3.3 Dynamics integration and clamping

Both 3D and 4D pipelines clamp control inputs to predefined bounds before integration. After updating the nominal sequence, MPPI clips each `u_new[t]` to admissible ranges to ensure safety and consistency with the robot’s limits.


## 4. Cost design in this repository

At each rollout step t, the stage cost combines:

- Velocity tracking along the path direction: projects `(vx, vy)` onto the reference yaw direction at the current position, penalizing squared deviation from a reference forward speed `ref_velocity`.
- Yaw alignment cost: penalizes squared difference between current yaw and reference yaw from the map.
- Collision penalty: reads a “collision_cost” layer (from local costmap) at the robot’s `(x, y)` to penalize proximity to obstacles.
- Distance error penalty: reads the “distance_error” layer (from the reference costmap generator) interpolated linearly.
- Command change penalties: L2 penalties on changes in `(vx, vy, omega)` and on changes in the 8-DoF wheel-space commands derived from the control input.

Terminal cost encourages the terminal state to be closer to the goal (unless already within goal tolerance).

All of the above are parameterized in each variant’s YAML and in code parameter structs. Crucial hyperparameters include:

- `weight_velocity_error`, `weight_angular_error`, `weight_collision_penalty`, `weight_distance_error_penalty`, `weight_terminal_state_penalty`.
- Command change penalties: `weight_cmd_change` (3D) and `weight_vehicle_cmd_change` (8D wheel changes).
- `ref_velocity` sets the target forward progression rate.

The repository provides a costmap generator (`reference_costmap_generator`) that converts the global planner’s path into:

- `distance_error_map`: distance to path centerline.
- `ref_yaw_map`: preferred heading at each map cell.

These maps enable path-following without explicit waypoints inside the MPPI horizon, while collision costs come from the local costmap.


## 5. Hyperparameters and stochasticity

Key MPPI hyperparameters and their roles:

- `num_samples (K)`: number of rollouts per solve; more rollouts improve robustness but increase computation.
- `prediction_horizon (T)` and `step_len_sec (dt)`: horizon length and integration timestep; longer horizons capture more foresight but increase compute.
- `param_lambda (λ)`: temperature parameter; smaller values make weights concentrate on low-cost rollouts (exploitation), larger values spread weights (exploration).
- `param_alpha (α)`: governs the cross term with the previous nominal control in the cost shaping; higher α reduces reliance on the cross term.
- `sigma`: per-input standard deviations for sampling noise; larger σ expands search but can destabilize.
- `param_exploration`: fraction of rollouts sampled without centering around the current nominal sequence (`exploration-only` rollouts), improving global search.
- `use_sg_filter`: whether to apply Savitzky–Golay smoothing to the first control action to reduce jitter.
- `reduce_computation`: if true, reuse one noise matrix across steps to save sampling time (at some loss in stochastic diversity).

Parallelism: The code uses OpenMP to accelerate noise generation and rollout evaluation across samples. Real-time performance can vary with the hardware and ROS scheduling, as noted in the README.


## 6. Variants in this repository (3D(a), 3D(b), 4D, H)

All variants share the MPPI skeleton, differing in the control space and parameters.

### 6.1 MPPI-3D

Both 3D variants operate in `(vx, vy, omega)`. The core is in `src/control/mppi_3d`. The two YAMLs primarily differ in noise scales `sigma` and are annotated in the top-level README with their intended behaviors:

- 3D(a): “driving faster but dangerous sometimes.”
- 3D(b): “relatively safe but driving slower.”

Selected parameter contrasts (from `config/mppi_3d_a.yaml` vs `config/mppi_3d_b.yaml`):

- `sigma` (3D(a)): `[1.00, 1.00, 0.78]`
- `sigma` (3D(b)): `[0.55, 0.55, 0.96]`

Interpretation: 3D(a) injects larger translational noise in `vx, vy`, leading to more aggressive lateral and forward exploration; 3D(b) increases `omega` noise while reducing translational noise, typically producing safer, more orientation-aware motion with lower lateral excursions.

Other cost weights and structural parameters are identical in the provided YAMLs. Both use SG filtering and the same horizon and sampling counts by default.

### 6.2 MPPI-4D

The 4D variant samples in the actuator space `(fl_steer, rr_steer, fl_vel, rr_vel)` and maps to `(vx, vy, omega)` internally for dynamics and costs. This is closer to physical constraints and thus can be safer but slightly conservative (the top-level README tags it “safe but relatively slow”). Key parameters (from `config/mppi_4d.yaml`):

- `param_lambda`: `100.0` (lower than the 3D configs’ `200.0`), putting relatively stronger emphasis on the best rollouts.
- `sigma`: `[0.78, 0.78, 1.00, 1.00]` for the four control dimensions.
- `use_sg_filter`: `false` by default in 4D (could be enabled in experiments).

The rest of the cost weights mirror the 3D configs; command-change penalties are computed both in body-frame and in the reconstructed 8-DoF wheel command space.

### 6.3 MPPI-H (Hybrid)

MPPI-H switches between MPPI-3D and MPPI-4D online based on tracking error, aiming to balance speed and safety dynamically. The core is in `src/control/mppi_h`:

- `mppi_h_core.cpp` selects mode using thresholds on distance error from the path and yaw error relative to the reference yaw map:
  - If both distance error and yaw error are below thresholds, choose Mode 1 (3D).
  - Otherwise, choose Mode 2 (4D).

- Sequence handoff between modes: the currently active controller shares its optimized `(vx, vy, omega)` sequence with the other one, so that when modes switch, the receiving controller starts from a sensible nominal trajectory:
  - Mode 1 to Mode 2: `getOptimalVxVyOmegaSequence()` from 3D → `setOptimalVxVyOmegaSequence()` in 4D.
  - Mode 2 to Mode 1: the symmetric handoff.

- Thresholds live in `config/mppi_h.yaml` under `mode_selector`: default `yaw_error_threshold = 0.5 rad`, `dist_error_threshold = 0.5 m`.

This hybrid switching realizes the insight of the IROS 2024 paper: in “easy” tracking regimes, operate in 3D space for agility; in “hard” regimes (large tracking error), switch to 4D sampling in actuator space for safety and better recovery.


## 7. ROS/Gazebo integration and data flow

Although the repository contains both ROS 1 and ROS 2 artifacts, follow the repository README for your platform. Key components involved in a full navigation stack:

- Gazebo world and 4WIDS vehicle spawner (`world_handler`), publishing ground-truth odometry and laser scans.
- Global planner (`move_base/NavfnROS`) to generate a global path.
- Reference costmap generator: subscribes to the global path and produces `distance_error_map` and `ref_yaw_map` for the controller’s costs.
- Local planner (MPPI variant), subscribing to odometry, global path, local costmap, distance error map, and reference yaw map; publishing `cmd_vel` and visualization/evaluation topics.

Data flow at each control interval (controller nodes `mppi_3d`, `mppi_4d`, or `mppi_h`):

1. Receive latest `Odometry` → `observed_state`.
2. Receive `Path` → extract `goal_state` from last pose.
3. Receive `OccupancyGrid` → convert to `GridMap` “collision_cost”.
4. Receive `GridMap` distance and ref-yaw maps.
5. Run `solveMPPI` to compute optimal `(vx, vy, omega)` for the next step.
6. Publish `cmd_vel` and evaluation topics; publish visualizations of optimal and sampled trajectories.

The controller publishes the following helpful diagnostics:
- `/mppi/calc_time` (ms) — computational load.
- `/mppi/optimal_traj` and `/mppi/sampled_traj` — visualization markers.
- `/mppi/eval_info` — contains `state_cost`, pose, command, and 8-DoF wheel commands derived from the optimal input.


## 8. Code walkthrough by component

This walkthrough highlights the files you should skim when studying or modifying the controller.

### 8.1 MPPI-3D core (sampling in body-frame space)

- `mppi_3d_core.hpp/.cpp`: MPPI class independent of ROS.
  - Constructor sets K, T, dimensions, initializes noise scales `sigma_[t][u]`, and optionally the SG filter coefficients.
  - `solveMPPI(...)` implements the full sampling loop, weight computation, update, smoothing, clamping, and trajectory recomputation.
  - `getOptimalVehicleCommand()` converts the chosen 3D control to 8-DoF wheel-space values for logging.
- `mppi_3d_setting.hpp`: Defines the state/update rules and the cost functions.
  - `stage_cost(...)`: velocity alignment to ref path, yaw alignment, collision penalty, distance error penalty, control-change penalties.
  - `terminal_cost(...)`: encourages proximity to goal when far.

### 8.2 MPPI-4D core (sampling in actuator space)

- `mppi_4d_core.hpp/.cpp`: parallel to 3D but operating on 4D controls.
- `mppi_4d_setting.hpp`: provides (i) 4D→3D conversion, (ii) dynamics using the 3D equivalent, and (iii) stage/terminal costs. Additional clamping in 4D ensures steer and wheel speed bounds.

### 8.3 MPPI-H core (hybrid switching)

- `mppi_h_core.hpp/.cpp`: holds two core instances (3D and 4D) and a mode selector.
- `selectMode(...)`: compares live distance error and yaw error against thresholds from `CommonParam`.
- Inter-controller sequence handoff maintains continuity through mode switches.

### 8.4 ROS nodes and interfaces

- `mppi_3d.cpp`, `mppi_4d.cpp`, `mppi_h.cpp`: each node
  - Declares/loads parameters (including topic names and controller settings).
  - Subscribes to odometry, global path, local costmap, distance error map, ref yaw map.
  - Sets up a wall timer at `control_interval` that triggers `calcControlCommand()`.
  - Publishes `cmd_vel`, diagnostics, and RViz markers.

### 8.5 Reference costmap generator

- `reference_costmap_generator`: converts the global path and occupancy map to `distance_error_map` and `ref_yaw_map` (visualizations included in its README). These maps are crucial for the controller’s costs.


## 9. Lab: Explore MPPI variants and Gazebo demos

We recommend running the demos using the environment setup instructions provided in the repository README. Two typical paths exist:

### 9.1 Environment setup

Follow the repo’s instructions for your OS/ROS version. The demos and `roslaunch` instructions below use the ROS 1 Noetic pathway (as per the top-level README). For ROS 2 Humble, refer to the README to build and launch the currently-ported packages and adapt with `ros2 launch` equivalents when available.

Key make targets (see README):

```bash
# Docker (Noetic) — build and run
make setup_docker_noetic
make run_docker_noetic

# Build inside container (or natively if on Noetic)
make build_noetic
```

### 9.2 Launch Gazebo and manual teleop (baseline)

Start a world and spawn the 4WIDS robot:

```bash
roslaunch launch/gazebo_world.launch gazebo_world_name:=maze
```

World options: `empty`, `empty_garden`, `cylinder_garden`, `maze`.

Verify topics:
- `/groundtruth_odom` should stream odometry.
- `/laser_link/scan` and `/fwids/joint_states` should be active.

### 9.3 Autonomous navigation with MPPI

Each variant is launched by selecting `local_planner` in the navigation launch file. In separate terminals (after sourcing your workspace):

```bash
# MPPI-3D(a): faster but occasionally risky
roslaunch launch/navigation.launch local_planner:=mppi_3d_a

# MPPI-3D(b): safer but slower
roslaunch launch/navigation.launch local_planner:=mppi_3d_b

# MPPI-4D: safe but relatively slow
roslaunch launch/navigation.launch local_planner:=mppi_4d

# MPPI-H: hybrid mode, recommended by the authors
roslaunch launch/navigation.launch local_planner:=mppi_h
```

Monitor in RViz:
- Optimal trajectory (`/mppi/optimal_traj`), sampled top-100 trajectories (`/mppi/sampled_traj`).
- Overlay text with controller name.

Monitor evaluation and timing:

```bash
rostopic echo /mppi/eval_info
rostopic echo /mppi/calc_time
```

Tip: Performance may vary due to asynchronous simulation and multi-threaded sampling.

### 9.4 Experiments: tuning and analysis

Perform the following experiments and record observations:

- Vary `sigma` (noise std per input): increase translational noise → more aggressive exploration; increase `omega` noise → more pivoting and heading corrections.
- Adjust `param_lambda`: smaller λ tends to exploit the lowest-cost rollouts; larger λ maintains exploration and robustness.
- Change `param_exploration`: raise from 0.1 to 0.3 and check if recovery from large deviations improves (at potential stability cost).
- Enable/disable SG filtering (`use_sg_filter`) and change window/degree to quantify command smoothness vs responsiveness.
- Compare `num_samples` (K = 1000, 2000, 3000+) vs `calc_time` and path quality.
- Switch worlds (`empty_garden`, `cylinder_garden`, `maze`) to test obstacle-rich scenarios.

For MPPI-H specifically:
- Tighten thresholds (`yaw_error_threshold`, `dist_error_threshold`) to make the controller prefer 4D more often — observe safety, speed, and stability.
- Loosen thresholds to prefer 3D — check speed-up and any collision risks.
- Plot mode switches over time (by adding simple logging of `getControllerName()` and thresholds).

Deliverables (suggested):
- Screenshots of RViz with optimal and sampled trajectories in at least two worlds.
- Plots of `/mppi/calc_time` vs `num_samples`.
- Short video or GIF of MPPI-H switching behavior in a challenging world.


## 10. Practical notes and pitfalls

- Map consistency: Ensure the frame conventions are correct for `collision_costmap`, `distance_error_map`, and `ref_yaw_map`. The nodes convert `OccupancyGrid`/`GridMap` into the layers expected by the cost functions.
- Goal handling: When within `xy_goal_tolerance` and `yaw_goal_tolerance`, the solver returns zero velocity; verify your tolerances to avoid oscillation near the goal.
- Clamping: Always consider physical limits; check the clamping ranges in 4D steering and wheel speeds.
- Sampling reuse: `reduce_computation=true` reuses the noise matrix; use with care when you need stochastic richness for recovery.
- Parallelism: OpenMP threads are used. Verify CPU threading limits in your environment for consistent timing.
- Simulation variability: Real-time performance differs across machines; record calc times when comparing variants.


## 11. From theory to implementation: key design choices in this repo

1) Cost shaping toward safe navigation: The cost terms are simple but effective — path alignment (direction and yaw), distance error, and occupancy-based collision cost; combined with penalties on control changes and wheel command changes, they stabilize motion and discourage abrupt actuation.

2) Sampling space matters: 3D body-frame sampling yields highly agile behavior but risks unsafe lateral moves; 4D actuator sampling biases the search to feasible wheel-space actions, trading speed for safety. MPPI-H turns this into a state-dependent decision.

3) Sequence handoff: MPPI-H’s exchange of optimal sequences between controllers reduces the disruption of mode switches and accelerates convergence after a switch.

4) Smoothing and receding horizon: The SG filter on the first control input reduces jitter without eliminating responsive behavior; the controller still replans at each step.

5) Robustness via exploration: A small fraction of purely exploratory rollouts helps escape poor local minima, especially after disturbances or large tracking deviations.


## 12. Suggested extensions and project ideas

- Dynamic obstacle costs: Add a time-varying dynamic obstacle cost layer (from perception) and compare outcomes across 3D/4D/H.
- Risk-sensitive tuning: Vary λ and σ to emulate risk-averse vs risk-seeking behaviors and quantify collision rates and time-to-goal.
- Learned components: Replace/refine map-derived costs using learned traversability or learned ref-velocity profiles.
- GPU acceleration: Port rollout and cost evaluation to GPU (CUDA) for higher K and T at real-time rates.
- Curvature-aware reference velocity: Modulate `ref_velocity` by path curvature or lookahead to smooth cornering.


## 13. Checklists (what to verify when things go wrong)

- Are all five subscriptions active? Odometry, global path, local costmap, distance error map, and reference yaw map.
- Are the frames consistent in RViz? Is the robot pose aligned with the map frame assumed by cost layers?
- Is `calc_time` consistently below `control_interval`? If not, reduce K, reduce T, or enable `reduce_computation`.
- Do you see mode switches in MPPI-H when large tracking errors occur? If not, adjust thresholds.
- Are command limits realistic for the Gazebo model? If commands saturate often, consider adjusting clamping ranges or weights.


## 14. Assessment and discussion prompts

1) Why does sampling in actuator space tend to be safer for a 4WIDS platform than sampling in body-frame velocities? Provide a geometric argument based on wheel steering and velocity composition.

2) If you double λ and halve σ simultaneously, what qualitative changes do you expect in behavior and why?

3) Propose an additional stage cost term that could reduce oscillatory lateral motion without greatly slowing progress.

4) In MPPI-H, would adding hysteresis to the mode selector (different enter/exit thresholds) be beneficial? Describe a design and expected effect.

5) Compare the computational footprints (calc time distributions) across 3D(a), 3D(b), 4D, and H for the `maze` world. Provide plots and commentary.


## 15. References

- Aoki, M., Honda, K., Okuda, H., Suzuki, T. (2024). “Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation.” In IROS 2024. [PDF](https://mizuhoaoki.github.io/media/papers/IROS2024_paper_mizuhoaoki.pdf)
- Repository project page with media and links: [Project website](https://mizuhoaoki.github.io/projects/iros2024)


## Appendix: Where to find things in the code

- MPPI-3D core: `src/control/mppi_3d/src/mppi_3d_core.cpp`; settings and costs: `include/mppi_3d/mppi_3d_setting.hpp`.
- MPPI-4D core: `src/control/mppi_4d/src/mppi_4d_core.cpp`; settings and costs: `include/mppi_4d/mppi_4d_setting.hpp`.
- MPPI-H hybrid: `src/control/mppi_h/src/mppi_h_core.cpp` (mode selection and sequence handoff).
- ROS nodes: `src/control/*/src/*.cpp` (e.g., `mppi_3d.cpp`, `mppi_4d.cpp`, `mppi_h.cpp`).
- Configurations:
  - `src/control/mppi_3d/config/mppi_3d_a.yaml`
  - `src/control/mppi_3d/config/mppi_3d_b.yaml`
  - `src/control/mppi_4d/config/mppi_4d.yaml`
  - `src/control/mppi_h/config/mppi_h.yaml`
- Gazebo worlds: `src/simulation/gazebo/world_handler/world/*.world`, and world launcher: `src/simulation/gazebo/world_handler/launch/launch_gazebo_world_with_fwids.launch`
- Navigation launcher with local planner argument: `launch/navigation.launch`.

Happy experimenting!

