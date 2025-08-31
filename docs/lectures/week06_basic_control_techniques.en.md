### Week 6: Basic Control Techniques — Feedback and PID for Trajectory Following

#### Overview
This week introduces feedback control and the Proportional–Integral–Derivative (PID) controller, the workhorse of low-level robot control. We will connect PID to trajectory following and show how to integrate it with path planning. In lab, you will tune PID gains for a simulated differential-drive robot in Gazebo to follow a reference trajectory while handling disturbances and noise. By the end, you should be able to design, tune, and reason about PID loops and interpret their impact on stability and tracking performance.

#### Learning Objectives
- Define feedback control and explain its benefits over open-loop control.
- Decompose a PID controller into P, I, D components and explain each role.
- Implement discrete-time PID with anti-windup and derivative filtering.
- Apply PID to trajectory following for a differential-drive robot (linear and angular velocity control).
- Tune PID (heuristic/manual, Ziegler–Nichols, relay/ultimate gain) and evaluate performance.
- Reason about closed-loop stability, steady-state error, overshoot, and robustness.
- Interface a path planner with a PID-based low-level controller in a robotics stack.

#### Required Reading
- Camacho, E. F., & Bordons, C. (Model Predictive Control), Chapters 1–2. These chapters ground the concepts of feedback and control objectives that we build upon. While the book focuses on MPC, the early material on feedback systems, objectives, and constraints is highly relevant for PID as a baseline controller.


## 1) Feedback Control Essentials

Feedback control uses measurements of the system state or output to compute control inputs that reduce the error between a reference and the actual output.

- Terminology:
  - Reference/Setpoint r(t): desired output (e.g., desired pose or velocity).
  - Output y(t): measured output (e.g., measured pose or velocity from odometry).
  - Error e(t) = r(t) − y(t): the quantity we aim to minimize.
  - Control input u(t): actuation command (e.g., wheel torque or commanded velocity).
  - Disturbances d(t): unmodeled effects, friction, slopes, sensor noise, delays.

- Open-loop vs. closed-loop:
  - Open-loop uses a planned input without measuring the output; it cannot reject disturbances or model errors.
  - Closed-loop uses the error to adapt inputs; it is robust to changes and disturbances, at the cost of complexity and potential stability issues if tuned poorly.

- Discrete-time implementation:
  - Robots are controlled digitally at a fixed control period Δt. All continuous expressions must be discretized.
  - Sensor noise, latency, and quantization can degrade performance.
  - Saturation and rate limits on actuators must be respected.

- Stability and performance:
  - Stability: bounded inputs produce bounded outputs; errors converge (or remain bounded) rather than diverge.
  - Performance: speed of response, overshoot, steady-state error, disturbance rejection, noise sensitivity.


## 2) PID Controllers: Components and Roles

The PID controller computes control u(t) from error e(t):

u(t) = Kp · e(t) + Ki · ∫ e(τ) dτ + Kd · de(t)/dt

- Proportional (P):
  - Acts on current error. Larger error yields a stronger correction.
  - Increases responsiveness but can cause overshoot and oscillation if too high.
  - Pure P leaves nonzero steady-state error in presence of constant disturbances.

- Integral (I):
  - Accumulates error over time to eliminate steady-state error.
  - Too aggressive I leads to integrator windup: large overshoot and sluggish recovery when actuators saturate.

- Derivative (D):
  - Acts on rate of change of error; predicts where error is heading.
  - Reduces overshoot and improves damping.
  - Sensitive to measurement noise; typically requires filtering.

Common Forms:
- Parallel/ideal form (above): separate gains Kp, Ki, Kd.
- Series/ISA form: Kc [1 + 1/(Ti s) + (Td s)/(α Td s + 1)], with filter α on D.
- Velocity/incremental form: computes Δu(k) from changes in error; useful when actuators expect command increments.

Practical Considerations:
- Derivative filtering: Replace de/dt with filtered version, e.g., a first-order low-pass filter on error or on measured output y(t).
- Anti-windup: Pause or back-calculate the integrator when actuator saturates.
- Output limits and rate limits: Clamp u and optionally du/dt to protect hardware.
- Setpoint weighting: Use different weights on r for P and D to reduce derivative kick.
- Bumpless transfer: Smoothly apply controller when switching modes or references.


## 3) Discrete-Time PID Implementation

Let k index samples with period Δt. A robust discrete formulation with anti-windup and derivative filtering:

Notation:
- e_k = r_k − y_k
- Integral state I_k
- Filtered derivative state D_k (first-order filter with time constant τ_d)

Update:
- I_k = I_{k−1} + Ki · e_k · Δt, with anti-windup conditional update or back-calculation
- D_k = (α · D_{k−1}) + (1 − α) · ((e_k − e_{k−1})/Δt), where α = τ_d/(τ_d + Δt)
- u_k = Kp · e_k + I_k + Kd · D_k
- u_k clamped to [u_min, u_max]; if clamped, apply anti-windup

Anti-windup (clamping):
- If u_k at limit and sign(e_k) drives integral further into saturation, freeze I_k update for that step.

Anti-windup (back-calculation):
- I_k = I_k + Kaw · (u_sat_k − u_raw_k), where Kaw is anti-windup gain, u_sat_k is clamped output.

Derivative on measurement:
- Use D on −y instead of on e to avoid derivative kick at setpoint changes: D ≈ −d y/dt.

Time-varying Δt:
- If timer jitter exists, compute using measured Δt each iteration to maintain stability.


## 4) Tuning Methods

Heuristic/Manual Tuning:
1) Start with Ki = 0, Kd = 0. Increase Kp until you get a fast response without sustained oscillation.
2) Add Kd to damp oscillations and reduce overshoot.
3) Add Ki to remove steady-state error. Increase carefully; monitor for slow oscillations and windup.
4) Iterate: adjust Kp, Kd, Ki to balance rise time, overshoot, and settling time.

Ziegler–Nichols (Ultimate Gain) Method:
1) Set Ki = 0, Kd = 0.
2) Increase Kp until sustained oscillations occur. Record ultimate gain Ku and oscillation period Pu.
3) Use ZN rules for tuning (classic):
   - P: Kp = 0.5 Ku
   - PI: Kp = 0.45 Ku, Ki = 1.2 Kp / Pu
   - PID: Kp = 0.6 Ku, Ki = 2 Kp / Pu, Kd = Kp · Pu / 8

Tyreus–Luyben and Cohen–Coon provide alternative rules with less overshoot. Always refine after initial estimates.

Relay/Autotune:
- Replace controller temporarily with a relay to induce limit cycles; measure amplitude and period to estimate Ku, Pu.
- Safer than ramping Kp in some systems.

Data-Driven/Optimization:
- Fit gains by minimizing integral error metrics (IAE, ISE, ITAE) on step or trajectory tracking data.
- Tools can automate this; ensure constraints on overshoot and actuator limits.

Practical Tips:
- Always respect output limits; configure anti-windup.
- Filter D; consider measurement low-pass filters.
- Tune at the operating point (speed/load). Gains may need scheduling across regimes.
- Revisit tuning once the full stack (perception/planning) is integrated; added delays and noise change optimal gains.


## 5) Stability and Robustness

Qualitative View:
- Increase Kp speeds response but reduces phase margin; too large leads to oscillation/instability.
- Ki removes bias but introduces a pole at the origin (or z=1 in discrete time); increases risk of low-frequency oscillations.
- Kd adds phase lead, improving damping but increasing high-frequency noise sensitivity.

Classic Frequency-Domain Notions:
- Gain margin and phase margin quantify robustness. Maintain comfortable margins to accommodate modeling errors, delays, and sampling.
- Time delay and sampling reduce phase margin; compensate with smaller Kp and higher damping (Kd) or redesign.

Discrete-Time Stability:
- Closed-loop poles must lie inside the unit circle in the z-plane.
- Excessive integral action pushes poles toward z=1, causing slow, oscillatory settling.

Nonlinearities:
- Saturation causes integrator windup and limit cycles; use anti-windup.
- Dead zones and backlash can create bias; consider feedforward and backlash compensation.

Validation:
- Step tests, disturbance injection, and frequency sweeps (chirps) help assess stability margins experimentally.


## 6) PID for Trajectory Following

Problem: Given a reference trajectory (positions and possibly velocities), compute velocity commands that drive the robot to follow it.

Common Approaches:
- Position-space PID: Control x and y errors (or lateral and heading errors) with PIDs, then map to v and ω.
- Velocity-space PID: When the planner provides v_ref and ω_ref, regulate tracking errors on v and ω directly via PIDs.
- Geometric controllers (Pure Pursuit, Stanley) complement PID; hybrid approaches are common.

Differential-Drive Framework:
- Robot state: pose (x, y, θ) from odometry/AMCL; control inputs: linear velocity v and angular velocity ω.
- Kinematics: ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω.

Error Definitions:
- Transform reference pose into robot frame or compute cross-track error e_ct and heading error e_θ.
- For waypoints, choose a lookahead point on the path to avoid chattering.

Control Structure:
- Outer loop: position errors → desired v_ref, ω_ref (geometric or PID on pose errors).
- Inner PIDs: track v_ref, ω_ref by commanding motor controllers; or directly compute v, ω from pose errors with PID blocks.

Feedforward:
- If the planner provides nominal v_ff and ω_ff, command u = u_ff + PID(error). This speeds response and reduces required gains.

Saturation and Safety:
- Clamp v, ω to robot limits; use acceleration limits to avoid wheel slip and actuator stress.


## 7) Lab: Tune PID for a Simulated Robot in Gazebo

Goal: Tune PID gains so a differential-drive robot follows a circular and a waypoint-based trajectory with minimal steady-state error, limited overshoot, and robustness to disturbances.

Deliverables:
- A ROS 2 node that follows a trajectory using PID.
- A recording or plots of tracking error over time for two trajectories.
- A brief write-up describing your tuning process and final gains.

### 7.1 Setup

Assumptions:
- ROS 2 (e.g., Humble) with Gazebo Classic or Gazebo (Ignition/Garden+) installed.
- A differential-drive robot simulation (e.g., TurtleBot3) or your own SDF/URDF with `gazebo_ros2_control` and wheel odometry. For TB3:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

If you use your own model, ensure topics:
- Odometry: `/odom` (nav_msgs/Odometry)
- Velocity command: `/cmd_vel` (geometry_msgs/Twist)

Optional: Minimal SDF snippet for a diff-drive with ROS 2 plugin (conceptual; adapt paths and parameters to your environment):

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="dd_robot">
    <link name="base_link">
      <inertial><mass>10</mass></inertial>
      <collision name="col"><geometry><box><size>0.3 0.2 0.1</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.3 0.2 0.1</size></box></geometry></visual>
    </link>
    <!-- Wheels and joints omitted for brevity -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/</namespace>
        <remapping>cmd_vel:=/cmd_vel</remapping>
        <remapping>odom:=/odom</remapping>
      </ros>
      <publish_tf>true</publish_tf>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.16</wheel_separation>
      <wheel_diameter>0.066</wheel_diameter>
      <ode> 
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </plugin>
  </model>
</sdf>
```

Note: Adjust plugin name and parameters for your Gazebo/ROS 2 distribution. If using Garden/Fortress (Ignition), use ign-gazebo and the ros_gz bridge.

### 7.2 PID Controller and Follower Node (ROS 2, Python)

The node subscribes to `/odom`, computes pose error to a reference trajectory, and publishes `/cmd_vel` using two PIDs: one for linear velocity (v) and one for angular velocity (ω). It includes anti-windup and derivative filtering.

```python
# pid_follower.py
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


@dataclass
class PidGains:
    kp: float
    ki: float
    kd: float


class DiscretePid:
    def __init__(self, gains: PidGains, dt: float, output_limits: Tuple[float, float],
                 derivative_tau: float = 0.05, anti_windup_k: float = 0.0, derivative_on_measurement: bool = True):
        self.gains = gains
        self.dt = dt
        self.u_min, self.u_max = output_limits
        self.derivative_tau = max(derivative_tau, 1e-6)
        self.anti_windup_k = anti_windup_k
        self.derivative_on_measurement = derivative_on_measurement

        self.integral_state = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.derivative_state = 0.0  # filtered derivative
        self.last_time = None

    def reset(self):
        self.integral_state = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.derivative_state = 0.0
        self.last_time = None

    def update(self, setpoint: float, measurement: float) -> float:
        now = time.time()
        if self.last_time is None:
            self.last_time = now
        dt = max(now - self.last_time, 1e-6)
        self.last_time = now

        error = setpoint - measurement

        # Derivative term (filtered)
        alpha = self.derivative_tau / (self.derivative_tau + dt)
        if self.derivative_on_measurement:
            raw_d = -(measurement - self.prev_measurement) / dt
        else:
            raw_d = (error - self.prev_error) / dt
        self.derivative_state = alpha * self.derivative_state + (1.0 - alpha) * raw_d

        # Integral term
        self.integral_state += self.gains.ki * error * dt

        # Unclamped control
        u_raw = self.gains.kp * error + self.integral_state + self.gains.kd * self.derivative_state

        # Clamp to limits
        u = max(self.u_min, min(self.u_max, u_raw))

        # Anti-windup
        if self.anti_windup_k > 0.0:
            self.integral_state += self.anti_windup_k * (u - u_raw)
        else:
            # Simple clamping anti-windup: freeze integrator if saturating in same direction as error
            if (u == self.u_max and error > 0.0) or (u == self.u_min and error < 0.0):
                self.integral_state -= self.gains.ki * error * dt

        self.prev_error = error
        self.prev_measurement = measurement
        return u


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


class TrajectoryFollowerPID(Node):
    def __init__(self):
        super().__init__('trajectory_follower_pid')

        # Parameters
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('v_limits', [0.0, 0.4])
        self.declare_parameter('w_limits', [-1.5, 1.5])
        self.declare_parameter('v_gains', [0.8, 0.0, 0.05])
        self.declare_parameter('w_gains', [3.0, 0.0, 0.1])
        self.declare_parameter('derivative_tau', 0.05)
        self.declare_parameter('anti_windup_k', 0.1)
        self.declare_parameter('use_feedforward', True)
        self.declare_parameter('trajectory', 'circle')  # 'circle' or 'waypoints'

        dt = float(self.get_parameter('dt').value)
        dv = self.get_parameter('v_limits').value
        dw = self.get_parameter('w_limits').value
        kv = self.get_parameter('v_gains').value
        kw = self.get_parameter('w_gains').value
        tau = float(self.get_parameter('derivative_tau').value)
        kaw = float(self.get_parameter('anti_windup_k').value)
        self.use_ff = bool(self.get_parameter('use_feedforward').value)
        self.mode = str(self.get_parameter('trajectory').value)

        self.pid_v = DiscretePid(PidGains(*kv), dt, (float(dv[0]), float(dv[1])), tau, kaw, derivative_on_measurement=True)
        self.pid_w = DiscretePid(PidGains(*kw), dt, (float(dw[0]), float(dw[1])), tau, kaw, derivative_on_measurement=True)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(dt, self.on_timer)

        self.pose = (0.0, 0.0, 0.0)
        self.last_time = self.get_clock().now()
        self.t0 = self.get_clock().now()
        self.idx = 0
        self.waypoints = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x, y, yaw)

    def reference(self, t: float) -> Tuple[float, float, float, float, float]:
        """Return reference pose and feedforward (x_ref, y_ref, yaw_ref, v_ff, w_ff)."""
        if self.mode == 'circle':
            R = 1.0
            v_ff = 0.2
            w_ff = v_ff / R
            x_r = R * math.cos(w_ff * t)
            y_r = R * math.sin(w_ff * t)
            yaw_r = wrap_angle(w_ff * t + math.pi/2.0)
            return x_r, y_r, yaw_r, v_ff, w_ff
        else:
            # Waypoint tracking with constant feedforward speed
            v_ff = 0.2
            x, y, yaw = self.pose
            tx, ty = self.waypoints[self.idx]
            dx = tx - x
            dy = ty - y
            if math.hypot(dx, dy) < 0.1:
                self.idx = (self.idx + 1) % len(self.waypoints)
                tx, ty = self.waypoints[self.idx]
                dx = tx - x
                dy = ty - y
            yaw_r = math.atan2(dy, dx)
            return tx, ty, yaw_r, v_ff, 0.0

    def on_timer(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        x, y, yaw = self.pose
        x_r, y_r, yaw_r, v_ff, w_ff = self.reference(t)

        # Compute pose error in robot frame
        dx = x_r - x
        dy = y_r - y
        # Rotate error into robot frame
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        e_yaw = wrap_angle(yaw_r - yaw)

        # Map pose errors to desired v, w setpoints
        # Simple design: v_set targets forward error ex, w_set targets lateral and heading errors
        v_set = self.pid_v.gains.kp * ex  # proportional pre-shaping; PID loop will correct residuals
        w_set = self.pid_w.gains.kp * (e_yaw + 0.5 * ey)

        if self.use_ff:
            v_ref = v_ff + v_set
            w_ref = w_ff + w_set
        else:
            v_ref = v_set
            w_ref = w_set

        # For this example, we regulate v and w directly with PIDs measuring zero (since setpoints are references)
        # Alternatively, subscribe to body-frame velocity estimates and close the loop on measured v, w
        v_cmd = self.pid_v.update(v_ref, 0.0)
        w_cmd = self.pid_w.update(w_ref, 0.0)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = TrajectoryFollowerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Notes:
- The node above closes the loop on v and ω internally without measuring them; for better fidelity, estimate body-frame velocities from `/odom` and use those as measurements to the `DiscretePid.update` calls.
- The structure shows setpoint pre-shaping using P on pose errors plus feedforward from the reference.
- Adjust `v_limits` and `w_limits` to match your robot.

Example parameters file to tweak gains at runtime:

```yaml
trajectory_follower_pid:
  ros__parameters:
    dt: 0.02
    v_limits: [0.0, 0.4]
    w_limits: [-2.0, 2.0]
    v_gains: [0.8, 0.05, 0.02]
    w_gains: [3.0, 0.05, 0.1]
    derivative_tau: 0.05
    anti_windup_k: 0.1
    use_feedforward: true
    trajectory: circle
```

Launch example:

```bash
ros2 run your_package pid_follower --ros-args --params-file params.yaml
```

### 7.3 Tuning Procedure and Checklist

1) Verify topics: `/odom` updates at expected rate; `/cmd_vel` drives the robot in Gazebo.
2) Start with conservative gains: small Kp, Ki = 0, Kd = small. Set output limits to robot specs.
3) Circle trajectory:
   - Increase linear Kp until the robot approximates the circle without instability.
   - Increase angular Kp to reduce heading error; balance with linear Kp.
   - Add Kd to reduce overshoot at turns; verify noise doesn’t dominate.
   - Add small Ki to remove steady-state bias (e.g., constant slip or drift). Confirm anti-windup.
4) Waypoints:
   - Validate convergence to each waypoint without oscillations.
   - Increase lookahead or weight ey in w_set if you see cross-track oscillation.
5) Disturbance tests:
   - Add external pushes or friction changes; confirm recovery without integrator runaway.
6) Record metrics:
   - RMS cross-track error, peak overshoot in heading, settling time, command saturation duty.

Target performance (suggested):
- RMS cross-track error < 0.08 m on circle at 0.2 m/s
- Peak heading error < 12°
- Settling time after a waypoint < 2.5 s

### 7.4 Troubleshooting

- Oscillation at high frequency: Kp too high or D too low; add derivative filtering; reduce Kp.
- Slow drift/steady-state error: Increase Ki slightly; check for saturation and anti-windup.
- Large overshoot: Reduce Kp or increase Kd; verify output rate limits.
- Noise-sensitive derivative: Increase derivative_tau; consider derivative on measurement; add sensor filtering.
- Integrator windup at corners: Ensure anti-windup is active; reduce Ki; consider setpoint ramping.
- Chattering near waypoints: Add lookahead; include ey term in angular control; reduce Kp.


## 8) Integrating PID with Path Planning

Role separation:
- Planner (global): computes a feasible path given a map, obstacles, and kinematic constraints.
- Local planner/trajectory generator: converts the path to time-parameterized references (v_ref, ω_ref) and a lookahead strategy.
- Controller (PID): tracks the references while rejecting disturbances and modeling errors.

Interfaces:
- The planner publishes a sequence of poses or a spline; the local planner publishes v_ref(t), ω_ref(t), and sometimes curvature κ.
- The PID controller consumes these references and odometry. Feedforward uses v_ref, ω_ref; feedback corrects residual error.

Best practices:
- Smooth references: Ensure continuity in position, heading, and curvature to avoid large derivative kicks.
- Enforce velocity/acceleration limits: Profile the path to respect robot capabilities so PID doesn’t saturate persistently.
- Gain scheduling: Use different gains at low/high speeds; higher Kd at higher speeds for damping.
- Fault tolerance: If planner fails or perception drops, fall back to safe stop; reset integrators on mode switch (bumpless transfer).

Example mapping:
- From path planner to PID follower:
  - For each cycle, compute a target pose at lookahead distance L on the planned path.
  - Compute pose error in robot frame and set v_ref, ω_ref via feedforward curvature κ: v_ff = profile(t), ω_ff = κ · v_ff.
  - PID adjusts around feedforward to cancel disturbances.


## 9) Beyond PID: When to Consider Alternatives

- Strong nonlinearities or constraints: Use Model Predictive Control (MPC) or nonlinear controllers that respect input/state limits.
- Highly coupled multivariable systems: MIMO control (LQR/LQG, MPC) may outperform decentralized PIDs.
- High-speed path tracking with large curvature changes: Pure Pursuit, Stanley, or nonlinear feedback linearization can be more robust.
Still, PID remains valuable as a baseline and often inside higher-level schemes (e.g., MPC generating setpoints, inner PID tracking).


## 10) Quick Reference and Exercises

Key formulas:
- Error: e_k = r_k − y_k
- Discrete PID: u_k = Kp e_k + I_k + Kd D_k
- I_k = I_{k−1} + Ki e_k Δt with anti-windup
- D_k = α D_{k−1} + (1 − α) Δe/Δt, α = τ_d/(τ_d + Δt)

Exercises:
1) Implement a PID with back-calculation anti-windup. Compare to clamping on a saturation test.
2) Tune PID for a circular trajectory at 0.1 m/s and 0.3 m/s. How do gains change with speed?
3) Add a low-pass filter to measured velocities and quantify its effect on tracking and delay.
4) Use relay autotuning to estimate Ku and Pu in simulation. Compare Ziegler–Nichols gains to your manual tuning.
5) Replace the pose-error pre-shaping with a Pure Pursuit outer loop; retain inner PIDs for v, ω. Compare cross-track RMS.


## 11) Grading Rubric (Suggested)

- Correctness (40%): PID implemented with anti-windup and derivative filtering; stable tracking on both tasks.
- Tuning Quality (30%): Reasonable gains, documented process, and metrics meet targets.
- Integration (20%): Clean ROS 2 node interfaces; parameters; safe saturation handling.
- Analysis (10%): Insightful discussion of stability, noise sensitivity, and planner–controller interplay.


## 12) Bibliography and Pointers

- Camacho, E. F., & Bordons, C.: Model Predictive Control. Chapters 1–2 for feedback basics.
- Åström, K. J., & Murray, R. M.: Feedback Systems. Free online text with fundamentals on stability and design.
- Åström, K. J., & Hägglund, T.: PID Controllers: Theory, Design, and Tuning.
- Franklin, G. F., Powell, J. D., & Emami-Naeini, A.: Feedback Control of Dynamic Systems.
- ROS 2 Control and Gazebo tutorials for differential-drive robots.


---
End of Week 6 notes.

