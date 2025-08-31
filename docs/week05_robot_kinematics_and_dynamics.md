### Week 5: Robot Kinematics and Dynamics — Differential vs. Swerve Drive; Forward/Inverse Kinematics

Instructor notes for a 2–3 hour session plus a lab. This week builds a rigorous bridge from geometric thinking to implementable kinematics and a light-touch dynamics perspective, with a strong emphasis on the swerve-drive implementation context in this repository.

---

## Learning objectives

- Understand and contrast nonholonomic differential-drive kinematics with near-holonomic swerve (4-wheel independent steering, 4WIDS) kinematics.
- Derive forward and inverse kinematics for both platforms at the quasi-static level.
- Connect kinematics to dynamics at a high level and recognize when quasi-static approximations are valid.
- Implement swerve-drive kinematics in Python/NumPy and run small simulations.
- Situate the math within this repository’s swerve-drive context (MPPI-based local control and documentation).

## Required reading

- Choset et al., “Principles of Robot Motion,” Chapter 3 (Kinematics and Dynamics fundamentals).
- Repository documentation: `README.md` (project overview, MPPI swerve focus) and `docs/week02_graph_based_path_planning_I.md` (global planners interfacing to swerve local control). The repo name and docs are swerve-centric, and the local controller is MPPI; see `src/control/` for controllers and `src/operation/joy_controller/` for teleop configuration mentioned in `README.md`.

---

## 1) Coordinate frames and planar twists (quick review)

We consider planar motion on SE(2) with robot body frame {B} at the chassis center and world frame {W}. A planar twist is

\[ v_B = [v_x, v_y, \omega]^T, \]

where \(v_x, v_y\) are translational velocities in {B} and \(\omega\) is yaw rate about +z. The relationship between body and world frame translational velocities uses the yaw angle \(\theta\):

\[ \begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix} = R(\theta) \begin{bmatrix} v_x \\ v_y \end{bmatrix}, \quad \dot{\theta} = \omega, \quad R(\theta)=\begin{bmatrix}\cos\theta & -\sin\theta\\ \sin\theta & \cos\theta\end{bmatrix}. \]

Robot-centric control often provides \(v_B\) directly. Field-centric control uses a global heading estimate to rotate user commands to {B} first.

---

## 2) Differential drive kinematics (nonholonomic)

Consider a differential drive with two coaxial wheels separated by track width \(b\). Let right/left wheel linear speeds be \(v_R, v_L\). Assuming pure rolling, no lateral slip:

- Forward kinematics (body twist from wheel speeds):

\[ v_x = \frac{v_R + v_L}{2},\quad v_y = 0,\quad \omega = \frac{v_R - v_L}{b}. \]

The instantaneous center of curvature (ICC) lies along the lateral axis at distance \(R = v_x/\omega\) when \(\omega \neq 0\). The nonholonomic no-sideways constraint is \(v_y = 0\) in the body frame.

- Inverse kinematics (wheel speeds from desired body twist):

\[ v_R = v_x + \frac{b}{2}\,\omega,\quad v_L = v_x - \frac{b}{2}\,\omega, \]

with the feasibility condition \(v_y = 0\). Desired \(v_y \neq 0\) cannot be achieved without violating the rolling constraint.

- Dynamics (brief): Each wheel actuates torque \(\tau\) generating longitudinal force \(F = \tau/r\) (wheel radius \(r\)). Longitudinal acceleration couples through mass/inertia and contact friction. The quasi-static kinematic equations above hold when accelerations are small and no-slip is a fair approximation.

ASCII diagram of ICC and wheel geometry (top view):

```
      ^ +y_B
      |
      |      ICC (to left when v_L < v_R)
      o------x  (robot center at 'x')
   [Left]   b   [Right]
    (L) <-------> (R)

   v_y = 0 constraint; forward v_x along +x_B
```

Implications:

- Nonholonomic: feasible velocities restricted by a non-integrable constraint, making planning/control state-dependent.
- Turning radius \(R\) finite unless \(v_R = -v_L\) (spin-in-place).

---

## 3) Swerve drive (4WIDS) kinematics (near-holonomic)

Swerve modules can independently steer and drive. A 4-module swerve places wheels at chassis corners. Denote module positions in the body frame as \(\mathbf{r}_i = [x_i, y_i]^T\) for modules \(i \in \{\text{FL, FR, RL, RR}\}\). For a desired body twist \(v_B = [v_x, v_y, \omega]^T\), the ideal ground contact velocity at module \(i\) is

\[ \mathbf{v}_i = \begin{bmatrix} v_x \\ v_y \end{bmatrix} + \omega \begin{bmatrix} -y_i \\ x_i \end{bmatrix} = \mathbf{v}_{\text{trans}} + \omega\,\mathbf{k}\times\mathbf{r}_i. \]

To realize this with a steerable wheel, set the wheel’s steering angle to the direction of \(\mathbf{v}_i\) and wheel speed to its magnitude:

\[ \theta_i = \operatorname{atan2}(v_{i,y}, v_{i,x}), \quad s_i = \lVert \mathbf{v}_i \rVert. \]

Because each module aligns its rolling direction to \(\theta_i\), lateral slip is minimized, and the base can achieve arbitrary planar twists within bounds—approximately holonomic.

ASCII vector diagram for one module (top view):

```
   r_i = [x_i, y_i]
   omega x r_i  = [-omega*y_i, +omega*x_i]
   v_trans      = [v_x, v_y]
   v_i = v_trans + (omega x r_i)

   angle(v_i) -> steering direction theta_i
   |v_i|      -> wheel speed s_i
```

### 3.1 Geometry of a rectangular chassis

Let half-lengths be \(L\) (forward/back) and \(W\) (left/right). Then:

\[ \mathbf{r}_{\text{FL}} = [ +L, +W]^T,\quad \mathbf{r}_{\text{FR}} = [ +L, -W]^T,\]
\[ \mathbf{r}_{\text{RL}} = [ -L, +W]^T,\quad \mathbf{r}_{\text{RR}} = [ -L, -W]^T. \]

Given \(v_B\), compute \(\mathbf{v}_i\) and then \((\theta_i, s_i)\) for each module.

### 3.2 Angle optimization and speed scaling

Practical swerve controllers flip a module by \(\pi\) if it reduces steering motion. If the desired angle change exceeds \(\pi/2\), set \(\theta_i' = \theta_i + \pi\) and drive the wheel backwards with speed \(s_i' = -s_i\). Additionally, normalize all \(s_i\) by the maximum across modules to fit actuator speed limits.

### 3.3 Inverse kinematics (estimating chassis twist)

When all module steering angles \(\theta_i\) and speeds \(s_i\) are known, we can estimate \(v_B\) by solving a least-squares system:

\[ \mathbf{v}_i = \begin{bmatrix} v_x - \omega y_i \\ v_y + \omega x_i \end{bmatrix} \approx s_i \begin{bmatrix} \cos\theta_i \\ \sin\theta_i \end{bmatrix}. \]

Stack for \(i \in \{\text{FL, FR, RL, RR}\}\) to form an overdetermined linear system in \(v_x, v_y, \omega\), then solve via pseudoinverse. This is useful for odometry and model validation.

### 3.4 Dynamics (brief)

Each module has two actuators: steer (angle) and drive (tangential speed). Dynamics include steering inertia, drive inertia, and contact friction. Controllers (e.g., MPPI) reason about these in their cost/rollouts. Our lab will focus on the quasi-static mapping from chassis twist to per-module commands, the bedrock of higher-level control.

---

## 4) Forward and inverse kinematics: derivations

### 4.1 Differential drive derivation

Rolling without slipping implies lateral velocity at the wheel-ground contact is zero. For a body twist \([v_x, v_y, \omega]\), the lateral velocity at the axle center is \(v_y + \omega x\). For a symmetric differential drive aligned with the body x-axis and wheels at \(x=0, y=\pm b/2\), the lateral velocity must be zero, implying \(v_y = 0\). The longitudinal velocity at left/right wheels becomes \(v_x \mp (b/2)\omega\). Equating to the actual wheel linear speeds \(v_L, v_R\) yields the forward/inverse formulae above.

### 4.2 Swerve derivation

For a rigid body, the velocity at a point \(\mathbf{r}_i\) is \(\mathbf{v}_{\text{trans}} + \omega\,\mathbf{k}\times\mathbf{r}_i\). A steerable wheel aligns its rolling direction with \(\mathbf{v}_i\); thus the required wheel direction and speed are the direction and magnitude of \(\mathbf{v}_i\). This is a direct application of rigid-body kinematics in the plane.

Inverse: Given \(\theta_i\) and \(s_i\), we reconstruct \(\mathbf{v}_i\) as \(s_i[\cos\theta_i, \sin\theta_i]^T\) and solve the linear system for \(v_x, v_y, \omega\).

---

## 5) Differential vs. swerve: practical trade-offs

- Differential drive:
  - Nonholonomic; cannot move sideways; simpler mechanics and control.
  - Often more robust in rough terrain; fewer actuators (two drive motors).
  - Planning/control incorporate the side-slip constraint; turns follow arcs or spins.

- Swerve drive:
  - Nearly holonomic in plane; can translate and rotate simultaneously in arbitrary directions.
  - Mechanically complex; four steer actuators + four drive actuators; calibration and control more demanding.
  - Excellent agility for confined spaces and precise alignment tasks.

In this repository, we assume a swerve base and use MPPI for local control/tracking. Global planners (grid/A*) from Week 2 feed waypoints that MPPI follows while honoring dynamics and actuator limits.

---

## 6) Tying to this repository (what to look at)

- Project focus: This is an MPPI controller for a swerve drive robot (`README.md`: “MPPI Controller for a Swerve Drive Robot”).
- Controllers: See `src/control/` (e.g., `mppi_3d/`, `mppi_4d/`) for model predictive control code paths and configuration hooks. While the exact kinematic helper functions may be embedded per controller, the kinematic mapping described here is the contract the controller expects.
- Teleoperation: `src/operation/joy_controller/` with configuration noted in `README.md` (e.g., `mppi_swerve_drive_ros/src/operation/joy_controller/config/joy.yaml`). Joystick inputs are typically mapped to body-frame \(v_x, v_y, \omega\) before swerve kinematics.
- Planning docs: `docs/week02_graph_based_path_planning_I.md` explicitly connects Dijkstra/A* global paths to the swerve base via local control. This week’s notes provide the kinematic layer between path and wheel commands.

Takeaway: when you hand MPPI a desired body twist trajectory, it internally samples/control-updates in the space of admissible inputs. The swerve kinematic mapping then converts the chosen instantaneous twist to module commands.

---

## 7) Lab: Model swerve-drive kinematics in NumPy

In this lab you will implement forward/inverse kinematics for swerve, add angle optimization and speed scaling, and run a tiny simulator that integrates body motion. You’ll also validate that inverse recovers the commanded twist when there is no noise.

### 7.1 Setup

- Ensure Python 3.9+ with NumPy installed:

```bash
python -m pip install numpy
```

Optional: Matplotlib for quick plots.

### 7.2 Reference geometry and helpers

```python
import math
import numpy as np

def rotation_matrix(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])

def body_twist_to_module_velocity(vx: float, vy: float, omega: float, r_i: np.ndarray) -> np.ndarray:
    # v_i = [vx, vy] + omega * [-y_i, x_i]
    return np.array([vx - omega * r_i[1], vy + omega * r_i[0]])

def vector_to_angle_speed(v_i: np.ndarray) -> tuple[float, float]:
    angle = math.atan2(v_i[1], v_i[0])
    speed = float(np.linalg.norm(v_i))
    return angle, speed

def optimize_angle(angle_desired: float, angle_current: float) -> tuple[float, int]:
    # Return possibly flipped angle and direction multiplier (+1 or -1)
    # Choose shortest rotation; if > 90 deg, flip by pi and reverse drive
    delta = (angle_desired - angle_current + math.pi) % (2 * math.pi) - math.pi
    if abs(delta) > math.pi / 2:
        return (angle_desired + math.pi) % (2 * math.pi), -1
    return angle_desired % (2 * math.pi), +1
```

### 7.3 Forward kinematics (swerve mapping)

```python
from dataclasses import dataclass
from typing import Dict

@dataclass
class SwerveModuleCommand:
    angle: float  # radians, [0, 2pi)
    speed: float  # meters/second, signed after optimization

@dataclass
class SwerveChassis:
    L: float  # half-length (forward/back)
    W: float  # half-width (left/right)
    max_wheel_speed: float

    def module_positions(self) -> Dict[str, np.ndarray]:
        return {
            "FL": np.array([+self.L, +self.W]),
            "FR": np.array([+self.L, -self.W]),
            "RL": np.array([-self.L, +self.W]),
            "RR": np.array([-self.L, -self.W]),
        }

def swerve_forward_kinematics(vx: float, vy: float, omega: float,
                              chassis: SwerveChassis,
                              angle_ref: Dict[str, float] | None = None) -> Dict[str, SwerveModuleCommand]:
    # Compute raw module vectors
    r_map = chassis.module_positions()
    raw_vectors = {k: body_twist_to_module_velocity(vx, vy, omega, r) for k, r in r_map.items()}

    # Convert to angles & speeds
    angle_speed = {k: vector_to_angle_speed(v) for k, v in raw_vectors.items()}

    # Optional angle optimization relative to prior/reference angles
    commands: Dict[str, SwerveModuleCommand] = {}
    for k, (ang, spd) in angle_speed.items():
        direction = +1
        if angle_ref is not None and k in angle_ref:
            ang_opt, mult = optimize_angle(ang, angle_ref[k])
            ang, direction = ang_opt, mult
        commands[k] = SwerveModuleCommand(angle=ang % (2 * math.pi), speed=direction * spd)

    # Speed scaling to respect max wheel speed
    max_spd = max(abs(c.speed) for c in commands.values())
    if max_spd > chassis.max_wheel_speed > 1e-6:
        scale = chassis.max_wheel_speed / max_spd
        for c in commands.values():
            c.speed *= scale
    return commands
```

### 7.4 Inverse kinematics (estimate chassis twist)

```python
def swerve_inverse_kinematics(commands: Dict[str, SwerveModuleCommand], chassis: SwerveChassis) -> np.ndarray:
    # Build linear system A * [vx, vy, omega]^T ≈ b from v_i = [vx - ω y_i, vy + ω x_i]
    # and v_i ≈ s_i [cos θ_i, sin θ_i]
    A_rows = []
    b_rows = []
    r_map = chassis.module_positions()
    for k, cmd in commands.items():
        x_i, y_i = r_map[k]
        cth, sth = math.cos(cmd.angle), math.sin(cmd.angle)
        vix, viy = cmd.speed * cth, cmd.speed * sth
        # vix = vx - ω y_i
        A_rows.append([1.0, 0.0, -y_i])
        b_rows.append(vix)
        # viy = vy + ω x_i
        A_rows.append([0.0, 1.0, +x_i])
        b_rows.append(viy)
    A = np.array(A_rows)
    b = np.array(b_rows)
    x_hat, *_ = np.linalg.lstsq(A, b, rcond=None)
    return x_hat  # [vx, vy, omega]
```

### 7.5 Tiny simulator for validation

```python
@dataclass
class Pose2D:
    x: float
    y: float
    theta: float

def integrate_pose(pose: Pose2D, vx: float, vy: float, omega: float, dt: float) -> Pose2D:
    # Convert body translation to world frame, then integrate
    R = rotation_matrix(pose.theta)
    v_world = R @ np.array([vx, vy])
    return Pose2D(
        x=pose.x + float(v_world[0]) * dt,
        y=pose.y + float(v_world[1]) * dt,
        theta=pose.theta + omega * dt,
    )

def demo():
    chassis = SwerveChassis(L=0.4, W=0.3, max_wheel_speed=3.0)
    pose = Pose2D(0.0, 0.0, 0.0)
    angle_ref = {k: 0.0 for k in ["FL", "FR", "RL", "RR"]}

    # Command a diagonal translate with slow spin
    vx_cmd, vy_cmd, omega_cmd = 1.0, 0.5, 0.3
    dt = 0.02
    for step in range(200):  # 4 seconds
        cmds = swerve_forward_kinematics(vx_cmd, vy_cmd, omega_cmd, chassis, angle_ref)
        # Update angle refs to encourage smooth steering
        for k, c in cmds.items():
            angle_ref[k] = c.angle

        # Estimate back the chassis twist
        vx_est, vy_est, om_est = swerve_inverse_kinematics(cmds, chassis)
        pose = integrate_pose(pose, vx_est, vy_est, om_est, dt)

    print(f"Final pose ~ x={pose.x:.2f}, y={pose.y:.2f}, th={pose.theta:.2f} rad")

if __name__ == "__main__":
    demo()
```

Expected behavior: With no noise and ideal mapping, the inverse recovers the commanded twist and the integrated pose approximates the analytic result.

### 7.6 Exercises

1) Verify that scaling `max_wheel_speed` changes the scaling applied to module speeds but leaves direction consistent. Plot wheel speeds over time under varying \(\omega\).

2) Implement field-centric input: given world-frame \([v_x^W, v_y^W]\) and heading \(\theta\), compute body-frame \([v_x, v_y] = R(-\theta)[v_x^W, v_y^W]\) prior to `swerve_forward_kinematics`.

3) Add angle-velocity limits: impose a maximum steering rate and simulate how angle optimization reduces large swings.

4) Noise study: perturb module angles/speeds and quantify the inverse-kinematics error in \(v_B\).

5) Compare to a differential-drive mapping by setting \(v_y=0\) and treating each side’s pair of wheels as aligned; reason about why swerve generalizes this constraint.

---

## 8) Dynamics perspective (brief but important)

Kinematics specifies instantaneous relationships under no-slip assumptions. Dynamics determines how quickly those velocities are achieved and how interaction forces behave.

- Differential drive dynamics: Two motors generate longitudinal forces through wheels; lateral slip is suppressed by geometry. Angular acceleration depends on differential forces and chassis inertia \(I_z\).

- Swerve dynamics: Eight actuators (4 steer, 4 drive). Steering dynamics introduce additional states and rate limits. During aggressive maneuvers, transient slip can occur, and controllers like MPPI account for acceleration and friction within predictive rollouts.

When are quasi-static kinematics adequate? For low accelerations, moderate speeds, and when the controller closes the loop quickly. In the repo, MPPI operates at high rate and can compensate for model mismatch, but the kinematic mapping remains the necessary inner transform between chassis twist and module commands.

---

## 9) Worked examples and checks

### 9.1 Swerve: pure rotation

Set \(v_x=v_y=0\). Then \(\mathbf{v}_i = \omega[-y_i, x_i]^T\). Opposite corners have equal magnitudes and directions differ by \(\pi/2\). All wheel speeds scale with distance to center; larger \(\sqrt{L^2+W^2}\) yields higher speeds. This matches intuition: wheels trace circular arcs around the center.

### 9.2 Swerve: pure translation

Set \(\omega=0\). Then all \(\mathbf{v}_i\) are identical with direction \(\operatorname{atan2}(v_y, v_x)\); all wheels steer to that direction, speeds equal to the translation magnitude.

### 9.3 Differential: arc motion

For \(v_R \neq v_L\) and both nonzero, \(\omega = (v_R - v_L)/b\) and \(R = v_x/\omega\). When \(v_R = -v_L\), \(R = 0\), and the robot spins in place.

---

## 10) Implementation guidance for this repo

- Mapping interface: Controllers produce \(v_B\). The swerve mapping computes per-module \((\theta_i, s_i)\). Ensure angles are continuous across updates via optimization and respect actuator speed/rate limits.
- ROS integration: Map \(v_B\) from teleop or planner to module commands. The `src/operation/joy_controller/` config in this repo (see `README.md`) typically sets scaling from joystick axes to \(v_B\). Controllers in `src/control/` will expect consistent units (m/s, rad/s, meters for geometry).
- Testing: Use the provided NumPy lab to validate your geometric assumptions before integrating into control loops. Compare the inverse-kinematics reconstruction against the commanded twist.

---

## 11) Discussion prompts

- Why is swerve often treated as holonomic at the kinematic level? What are the practical departures from holonomy in real systems?
- Compare planning implications: how does a nonholonomic base like differential drive induce state-dependent reachable sets, versus a near-holonomic swerve?
- What trade-offs arise from angle optimization (\(+\pi\) flips)? When does it help or hurt control smoothness?
- In what regimes do you expect MPPI to outperform simpler local controllers for swerve tracking?

---

## 12) Checklist

- Derived differential forward/inverse kinematics and ICC interpretation.
- Derived swerve forward mapping and inverse estimation; added angle optimization and speed scaling.
- Implemented NumPy simulation to validate mappings and pose integration.
- Connected mappings to this repo’s MPPI swerve control and teleop configuration.
- Read Choset Ch. 3 and repository docs.

---

## 13) Appendix: concise formulas

Differential drive (track \(b\)):

\[ v_x = (v_R + v_L)/2,\quad v_y = 0,\quad \omega = (v_R - v_L)/b. \]
\[ v_R = v_x + (b/2)\,\omega,\quad v_L = v_x - (b/2)\,\omega. \]

Swerve (module at \(\mathbf{r}_i=[x_i, y_i]^T\)):

\[ \mathbf{v}_i = \begin{bmatrix} v_x - \omega y_i \\ v_y + \omega x_i \end{bmatrix},\quad \theta_i = \operatorname{atan2}(v_{i,y}, v_{i,x}),\quad s_i = \lVert \mathbf{v}_i \rVert. \]

Inverse (least squares): Stack \(v_{i,x}, v_{i,y}\) equations to solve for \([v_x, v_y, \omega]^T\).

---

## 14) Further reading and practice

- Choset, Lynch, Hutchinson et al., “Principles of Robot Motion,” MIT Press.
- LaValle, “Planning Algorithms,” for deeper nonholonomic planning chapters.
- Practical control design notes on swerve (module calibration, angle wrapping, synchronization) from robotics competition teams and academic labs; compare designs and conventions with the mapping used here.

End of Week 5 notes.

