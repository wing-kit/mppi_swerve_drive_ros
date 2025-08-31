## Course Overview: Introduction to Path Planning and Control in Robotics

This section orients you to what we will learn, how we will learn it, and the tools you will use. It also explains why path planning and control are central pillars of robotics and shows you where the course will take you—both intellectually and practically—using a Model Predictive Path Integral (MPPI) controller from this repository as our hands-on foundation.

### Introduction to Robotics

- **What a robot is**
  - **Robot**: A programmable physical system that senses the world, decides what to do, and acts upon it.
  - Typical components:
    - **Sensing**: Cameras, LIDAR, IMUs, encoders.
    - **Planning**: Path/trajectory planning, task sequencing.
    - **Control**: Tracking desired motion precisely and safely.
    - **Actuation**: Motors, servos, wheels, propellers.
- **The Sense–Plan–Act loop (in words)**
  - “Diagram”: Imagine three boxes in a loop.
    - Box 1: “Sense (Perception): estimate state of robot and environment.”
    - Box 2: “Plan (Decision): compute safe, feasible trajectory.”
    - Box 3: “Act (Control): track trajectory with feedback.”
  - Arrows flow from Sense → Plan → Act → back to Sense as new data arrives.
- **Where this course sits**
  - We stand at the interface of “Plan” and “Act,” focusing on algorithms that:
    - Generate motion through complex environments (planning).
    - Track motion robustly despite uncertainty and dynamics (control).

### Why Path Planning and Control Matter

- **Safety and feasibility**
  - Planning ensures paths avoid collisions and respect kinematics/dynamics (e.g., nonholonomic constraints).
  - Control ensures the robot follows the planned path despite disturbances, slippage, or modeling error.
- **Performance and robustness**
  - Good planners find paths quickly, even in large maps or with dynamic obstacles.
  - Good controllers trade off precision, energy, smoothness, and responsiveness.
- **Real-world examples**
  - **Warehouse robot**: Plan collision-free paths among shelves and workers; control to track the path smoothly while carrying payloads.
  - **Self-driving car**: Plan lane changes and merges; control to stabilize speed and steering under wind, slopes, and traffic.
  - **Quadcopter**: Plan 3D trajectories through windows; control thrust and orientation to handle gusts.
- **Planning vs. control vs. trajectory optimization**
  - Planning answers “Where to go?” over maps and constraints.
  - Control answers “How to follow the planned trajectory?” under dynamics and disturbances.
  - Trajectory optimization (e.g., MPPI) merges both by directly optimizing control sequences subject to dynamics and cost.

### Course Format and Duration

- **Duration**: 14 weeks.
- **Meetings**:
  - 2 lectures per week (75 minutes each).
  - 1 lab per week (120 minutes), hands-on with ROS, Gazebo, and this repository.
- **Assessments**:
  - 5–6 labs (individual, with structured rubrics).
  - 2 short quizzes (concept checks on core algorithms).
  - 1 midterm (planning + control fundamentals).
  - Final project (team-based, integrating MPPI with sensing and planning).
- **Pedagogical style**:
  - Concept-first lectures with concrete examples.
  - Labs that bring theory to life using the MPPI controller as a baseline.
  - Incremental complexity, from 2D differential-drive robots to higher-dimensional systems.

### Learning Objectives

By the end of the course, you will be able to:

- **Concepts**
  - Explain the Sense–Plan–Act paradigm and the roles of planning and control.
  - Compare graph-based, sampling-based, and optimization-based planning methods.
  - Describe feedback control fundamentals (PID, LQR) and constraints in robotic systems.
  - Articulate stochastic control ideas and why MPPI is useful for nonlinear systems.
- **Algorithms and implementation**
  - Implement grid/graph planners (Dijkstra/A*) and sampling-based planners (RRT/RRT*).
  - Formulate cost functions and constraints for trajectory optimization.
  - Use, configure, and extend an MPPI controller for different robot models.
- **Tools and systems**
  - Use ROS for messaging and node orchestration; use Gazebo for simulation.
  - Interface planners and controllers via topics/services and standard message types.
  - Debug, tune, and benchmark planning and control pipelines in simulation.
- **Evaluation and communication**
  - Design experiments, collect results, and analyze performance/safety trade-offs.
  - Write clear technical reports and give concise demos.

### Prerequisites

- **Mathematics**
  - Linear algebra (vectors, matrices, eigenvalues).
  - Calculus (derivatives, gradients, Taylor expansions).
  - Basic probability (distributions, expectation, variance).
- **Computer science**
  - Data structures and algorithms (graphs, search, complexity).
  - Proficiency in Python and familiarity with C++.
  - Linux command line, Git, and VS Code or equivalent.
- **Recommended background**
  - Basic control systems (transfer functions, stability) is helpful but not required; we will review essentials.
  - Familiarity with ODEs and numerical integration.

### Required Resources

- **Textbooks (recommended, not all required)**
  - Steven M. LaValle, “Planning Algorithms” (freely available online).
  - Sebastian Thrun, Wolfram Burgard, Dieter Fox, “Probabilistic Robotics.”
  - B. Siciliano et al., “Robotics: Modelling, Planning and Control” (for dynamics/control background).
- **Software**
  - Linux environment (Ubuntu is recommended; WSL2 on Windows is acceptable).
  - ROS 2 (Humble or Iron recommended for long-term support).
  - Gazebo (Classic or the modern Gazebo; we will specify per lab).
  - Python 3.10+; optionally C++17+ (depending on your chosen project path).
  - Visualization tools: RViz2, Matplotlib/PlotJuggler.
- **This repository**
  - Contains:
    - A working MPPI controller implementation (our primary lab baseline).
    - Example robot models and configuration files.
    - Lab scaffolds, test maps, and utility scripts.
    - Starter planners (or stubs) that you will extend.
  - Expect to:
    - Run the MPPI controller out-of-the-box in Gazebo for a baseline demo.
    - Modify cost terms, dynamics models, and sampling strategies.
    - Integrate MPPI with your planner for end-to-end autonomy.
- **Online resources**
  - ROS 2 Tutorials and ROS Concepts (nodes, topics, services, parameters).
  - Gazebo tutorials for world creation, sensors, and robot models.
  - Open-source MPPI references and literature for deeper dives.
  - Instructor-provided notes, slides, and annotated code snippets.

### How We Will Use MPPI in This Course

- **MPPI at a glance**
  - Model Predictive Path Integral (MPPI) control is a sampling-based model predictive control method.
  - It rolls out many noisy control sequences, evaluates a cost over system dynamics, and computes a control update as a cost-weighted average.
  - Strengths: handles nonlinear dynamics and nonconvex costs; amenable to parallelization; flexible and modular.
- **What is in this repository**
  - An MPPI controller with:
    - Pluggable cost functions (goal reaching, collision penalties, smoothness).
    - Dynamics models for common platforms (e.g., differential drive; extendable to unicycle, car-like, quadrotor).
    - Interfaces to ROS topics for state estimation and control commands.
    - Config files for horizons, sampling variances, and cost weights.
- **Your learning path with MPPI**
  - Lab 1–2: Run MPPI in simulation to reach a single goal; visualize trajectories in RViz2; interpret cost terms.
  - Lab 3–4: Integrate a simple planner (A*) with MPPI; MPPI refines/track trajectories while avoiding obstacles.
  - Lab 5–6: Extend MPPI:
    - Add dynamic obstacle cost terms (time-indexed collision checks).
    - Tune sampling and horizon to balance reactivity and smoothness.
    - Swap dynamics models (e.g., differential drive to car-like).
  - Final project: Use MPPI as your control substrate and integrate perception/planning for an end-to-end demo.

“Diagram in words”: A star-shaped “Planner” node proposes a path; a gear-shaped “MPPI” node sits downstream, rolling out candidate controls over your dynamics and choosing the action with a weighted policy update; a thruster-shaped “Robot” executes and publishes new state; arrows loop back for receding horizon control.

### High-Level Weekly Topics (Subject to minor adjustments)

- **Weeks 1–2**: Robotics foundations; kinematics, dynamics; the Sense–Plan–Act loop; intro to ROS/Gazebo; run MPPI baseline.
- **Weeks 3–4**: Configuration space, occupancy grids; graph search (Dijkstra, A*), heuristics, motion primitives.
- **Weeks 5–6**: Sampling-based planning (PRM, RRT, RRT*); kinodynamic variants; feasibility vs. optimality.
- **Week 7**: Trajectory optimization overview; MPC vs. MPPI; cost design; constraints; warm starts.
- **Week 8**: Feedback control; PID, LQR; linearization; tracking controllers; controller–planner interfaces.
- **Week 9**: Uncertainty; stochastic optimal control; belief-space planning; MPPI under uncertainty.
- **Week 10**: Dynamic obstacles; time-parameterized costs; prediction and reactive planning.
- **Week 11**: Multi-robot coordination; shared spaces; collision avoidance at scale.
- **Week 12**: Integration week; planning–control stacks; benchmarking and ablation studies.
- **Weeks 13–14**: Final projects, demos, and presentations.

### Example Scenario Walkthroughs

- **Differential-drive robot in a maze**
  - Planner: A* over a 2D occupancy grid produces a collision-free path.
  - Controller: MPPI refines and tracks a curvature-limited trajectory, penalizing sharp turns and proximity to walls.
  - Outcome: Smooth traversal with consistent clearance margins.
- **Car-like robot (nonholonomic) in a parking lot**
  - Planner: Hybrid A* with feasible motion primitives.
  - Controller: MPPI with car-like dynamics model; cost for lane center, heading alignment, jerk.
  - Outcome: Realistic turns and stable backing maneuvers.
- **Quadrotor waypoint mission**
  - Planner: PRM or kinodynamic RRT*, 3D path with altitude constraints.
  - Controller: MPPI with simplified quadrotor dynamics; altitude and attitude regularization.
  - Outcome: Robust waypoint tracking with disturbance rejection.

“Diagram in words”: Envision a 2D grid with start at bottom-left and goal at top-right. A* finds a jagged piecewise-linear path hugging free space. MPPI overlays multiple transparent candidate trajectories that “wiggle” around the A* path, selecting a smooth, safe one to track.

### Course Policies and Collaboration

- **Collaboration**
  - Discuss concepts and debugging strategies freely.
  - Code you submit must be your own (or your team’s for team assignments).
  - Cite sources for any adapted ideas or snippets.
- **Reproducibility**
  - Labs and projects must include configuration files and scripts to reproduce results.
  - Maintain clear experiment logs (seed values, horizons, noise scales, cost weights).
- **Professional conduct**
  - Write readable code and commit often with meaningful messages.
  - Use issues/PRs if working in teams for clarity and reviewability.

### Getting Started with Tools

- **Environment setup options**
  - Recommended: Ubuntu (native or via WSL2).
  - Alternative: Course-provided Docker image for consistent ROS/Gazebo/colcon toolchains.
- **ROS 2 essentials**
  - Concepts: nodes, topics, services, parameters, launch files.
  - Messages you’ll likely use: odometry, transforms (TF), laser scans, cmd_vel or control inputs.
  - Debug helpers: `rqt_graph`, `rviz2`, `ros2 topic echo`, `ros2 param list`.
- **Gazebo essentials**
  - Worlds: prebuilt maps for indoor/outdoor tests.
  - Models: differential-drive base, car-like base, quadrotor (as provided).
  - Sensors: LIDAR, depth camera, IMU.
- **Repository structure (typical)**
  - `mppi_controller/`: core MPPI code.
  - `robots/`: URDF/SDF models, dynamics configs.
  - `configs/`: cost weights, horizons, noise scales, planner-controller wiring.
  - `worlds/`: Gazebo worlds and maps.
  - `labs/`: starter notebooks/scripts and instructions.
  - `scripts/`: utilities for plotting, logging, evaluation.

- **First run checklist**
  - Clone the repository.
  - Install ROS 2 (Humble or Iron) and Gazebo per instructions.
  - Build and source your workspace.
  - Launch a minimal simulation and verify MPPI publishes control messages.

- **Common commands (illustrative)**
```bash
# Build the workspace
colcon build --symlink-install

# Source overlay (new terminal)
source install/setup.bash

# List ROS topics to verify runtime graph
ros2 topic list

# Launch MPPI controller with a sample world (example)
ros2 launch mppi_controller demo_world.launch.py
```

- **Troubleshooting mindset**
  - If the robot is oscillatory: reduce control noise scale, increase smoothing costs, shorten horizon.
  - If it collides: increase obstacle/clearance costs, refine map resolution, improve planner seeds.
  - If it’s sluggish: reduce horizon or increase control frequency; tune weights to favor progress.

“Diagram in words”: A tuning dashboard with three sliders labeled “Goal Progress,” “Smoothness,” and “Clearance.” Moving one slider affects trajectory shape; the art is to balance all three.

### How Labs Connect Concepts to Practice

- **Lab 1: MPPI warm start**
  - Run MPPI on a point-mass or differential-drive robot to reach a goal in an empty world.
  - Deliverables: trajectory plots, discussion of how noise scale and horizon affect behavior.
- **Lab 2: Obstacles and costs**
  - Add map-based collision costs; tune clearance vs. path length; visualize cost heatmaps.
  - Deliverables: side-by-side runs, ablation study of cost terms.
- **Lab 3: Planning seeds**
  - Implement A* on a grid; interface its output with MPPI; measure tracking errors vs. no-seed.
- **Lab 4: Dynamics swap**
  - Change from differential-drive to car-like dynamics; update constraints; retune parameters.
- **Lab 5: Dynamic obstacles**
  - Introduce moving obstacles (time-indexed costs); evaluate time-to-collision vs. safety margins.
- **Lab 6: Robustness study**
  - Inject disturbances (e.g., simulated wind); compare stability across parameter sets; report.

### What Success Looks Like

- **Technical**
  - You can pick and justify a planner for a given map and robot.
  - You can configure and extend MPPI to meet task constraints.
  - You produce repeatable experiments with clear metrics (e.g., success rate, time, clearance, control effort).
- **Professional**
  - Your code is modular, tested, and documented.
  - Your reports are concise, with plots, tables, and well-chosen metrics.
  - You communicate design decisions and trade-offs clearly.

### A Mental Model to Carry Forward

- **The hierarchy**
  - Planning provides a feasible, goal-directed path under environment constraints.
  - MPPI sits in the control layer (or trajectory optimization layer), turning high-level intent into actionable, dynamically consistent controls in real time.
- **The loop you’ll internalize**
  - Sense → Plan/Update → Optimize (MPPI) → Act → Repeat.
- **The craft**
  - Cost shaping is a language: progress, safety, smoothness, and energy must be balanced.
  - Dynamics fidelity matters: the more realistic your model, the better your predictions—up to your computational budget.

### Frequently Asked Questions

- **Do I need prior ROS experience?**
  - No. We’ll teach the basics in the first two weeks and reinforce them in labs.
- **Will the MPPI controller “just work” for any robot?**
  - Not without adaptation. You’ll learn how to adjust dynamics models and costs for each platform.
- **Is there heavy math?**
  - Yes, but applied. We focus on intuition and implementation, with math supporting design and tuning.

### Final Note

This course combines theory with immediate practice. The MPPI controller in this repository is your sandbox: a tangible, modifiable foundation that makes the abstract concrete. By the end, you will not only understand planning and control—you’ll have built and tuned systems that plan and control in real time.

- “Diagram in words”: A layered cake. Bottom layer: “Dynamics & Actuation.” Middle layer: “Control (MPPI).” Top layer: “Planning & Decision.” Icing around the cake: “Perception & State Estimation.” Candle on top: “Task Goal.” The celebration is getting that candle to the right place—safely, efficiently, and reliably.

- “Diagram in words”: A racetrack with cones (obstacles). The planner draws a rough chalk line around the track; MPPI paints a smooth racing line, constantly adjusting to wind and grip changes; the controller steers with confidence lap after lap.

---

- **Key takeaways**
  - You’ll learn core planning and control methods and apply them in ROS/Gazebo.
  - The MPPI controller from this repository is the backbone of hands-on labs and projects.
  - Expect a mix of theory, coding, tuning, and rigorous evaluation leading to an integrated final project.

