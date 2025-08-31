### Week 1 — Introduction to Robotics and Path Planning
Course: Introduction to Path Planning and Control in Robotics  
Audience: Computer Science undergraduates

### Learning goals
- Understand what constitutes a robot, with a focus on mobile robots.
- Explain swerve drive platforms and derive basic kinematic relationships.
- Formulate a path planning problem (robot, environment, objectives, constraints).
- Define configuration space and how obstacles map to the robot’s space of motion.
- Set up a Python and ROS development environment to run simple simulations.
- Connect these ideas to this repo’s swerve drive context to ground theory in practice.

---

## 1. What is a robot? Why path planning?
- **Robot**: A physical agent that can sense, process, and act upon the environment. Typical loop: perception → planning → control → actuation.
- **Mobile robots**: Robots that move in space (2D or 3D). Contrast with manipulators (fixed base arms). We’ll focus on ground mobile robots this week.
- **Path planning**: The computational problem of finding a feasible path from start to goal that obeys constraints and avoids collisions.
- **Control**: Converting a path or trajectory into motor commands in real time while handling disturbances and uncertainty.

Text-based conceptual picture:
```
[Sensors] -> [State Estimation] -> [Planning] -> [Control] -> [Actuators]
                             ^                                 |
                             |                                 v
                         [World / Environment / Robot Dynamics]
```

Why this matters:
- Robotics systems must navigate safely and efficiently.
- Planning separates “what to do” (high-level) from “how to actuate” (low-level), enabling modularity.

Recommended reading:
- LaValle, Planning Algorithms, Chapters 1–2 (overview and foundational notions). See the official free text via the author’s site (search “LaValle Planning Algorithms PDF”).

---

## 2. Overview of mobile robots
- **Differential drive**: Two wheels; control via left/right wheel velocities. Simple, nonholonomic.
- **Ackermann (car-like)**: Steering angle + speed; nonholonomic with minimum turning radius.
- **Omnidirectional (mecanum/omni wheels, swerve)**: Capable of independent translation and rotation in the plane; often treated as holonomic for planning at the kinematic level.
- **Tracked robots**: Continuous tracks; often modeled similarly to differential drive but with different contact mechanics.

Assumptions we’ll often use at this stage:
- Planar motion (SE(2): x, y, θ).
- Low-speed, no wheel slippage (pure rolling).
- Quasi-static kinematics (ignore complex dynamics initially).

---

## 3. Swerve drive: intuition and kinematics
Swerve modules allow each wheel to both steer (change its wheel heading) and drive (change its wheel speed). This gives a robot the ability to translate in any planar direction while rotating arbitrarily—approximately holonomic behavior.

Text-based diagram (top-down):
```
        Front
   +-----------------+
   |   (FL)    (FR)  |
   |    o        o   |   o = swerve module (steer + drive)
   |                 |
   |   (RL)    (RR)  |
   |    o        o   |
   +-----------------+
        Rear
```

Notation:
- Desired chassis twist in the robot frame: vx (m/s), vy (m/s), ω (rad/s).
- Position of wheel i relative to robot center: (rxi, ryi).
- Wheel i velocity vector in the plane:
  - vix = vx − ω·ryi
  - viy = vy + ω·rxi
- Wheel i speed: si = sqrt(vix^2 + viy^2)
- Wheel i steering angle: αi = atan2(viy, vix)

Practical notes:
- If any si exceeds the maximum wheel speed, scale all si uniformly to maintain direction while respecting limits.
- To minimize steering rotation, you can invert wheel direction by adding π to αi and negating si if that yields a smaller angular change.
- Field-centric control uses a global frame (e.g., from IMU/odometry) to rotate joystick inputs into robot frame before applying kinematics.

Pseudocode to compute swerve commands:
```python
def compute_swerve_modules(vx, vy, omega, wheel_positions, max_wheel_speed):
    # wheel_positions: list of (rx, ry) per module relative to robot center
    modules = []
    max_speed = 0.0
    for (rx, ry) in wheel_positions:
        vix = vx - omega * ry
        viy = vy + omega * rx
        speed = (vix**2 + viy**2) ** 0.5
        angle = math.atan2(viy, vix)
        modules.append({"speed": speed, "angle": angle})
        max_speed = max(max_speed, speed)

    if max_speed > max_wheel_speed:
        scale = max_wheel_speed / max_speed
        for m in modules:
            m["speed"] *= scale

    # Optional: apply angle-optimization (flip by π if it reduces steering travel)
    for m in modules:
        m["angle"], m["speed"] = optimize_module_angle(m["angle"], m["speed"])
    return modules
```

How this repo ties in:
- We will use the swerve drive context in this repo to instantiate the above mapping and to visualize planned motions executed by a simulated swerve base. Later labs will interface planners to swerve kinematics and, eventually, controllers.

---

## 4. Path planning fundamentals
At a high level, a motion planning problem is:
- **Input**:
  - Robot model (geometry and kinematics/dynamics).
  - Environment (obstacles, map).
  - Initial configuration qstart and goal configuration qgoal.
  - Constraints (kinematic, nonholonomic, dynamic, safety).
  - Objective (e.g., minimize path length, time, energy, risk).
- **Output**: A feasible path (or trajectory) from qstart to qgoal that avoids collisions and respects constraints.

Key terms (LaValle Ch. 1–2):
- **State vs configuration**:
  - Configuration q describes the robot’s geometric degrees of freedom needed to determine occupancy (e.g., planar base: q = (x, y, θ)).
  - State x may include velocities, sensor states, etc. When using purely geometric planning, we often work in configuration space (C-space).
- **C-space**:
  - A space in which each point is a robot configuration.
  - Obstacles in the workspace become forbidden regions in C-space.
  - Free configurations form C_free; forbidden set is C_obs; all configurations form C.
- **Holonomic vs nonholonomic**:
  - Holonomic: all DOFs can be independently controlled (subject to bounds).
  - Nonholonomic: differential constraints reduce instantaneous reachable velocities (e.g., car-like robots).

---

## 5. Configuration space (C-space) and obstacles
Idea: Convert a robot-with-obstacles problem into a point-in-C-space problem.

- If the robot is a disk of radius r and obstacles are polygons in 2D, then:
  - Inflate obstacles by r (Minkowski sum with a disk of radius r).
  - Treat the robot as a point.
  - Plan a path for the point that avoids the inflated obstacles.
- For a rigid polygonal robot, generalize via Minkowski sum of the robot with obstacles (rotations may make C-space 3D: x, y, θ).

ASCII picture:
```
Workspace:
  Robot (circle) navigating around a box

     ######
     #    #
     #    #
     ######     O  Robot

C-space (point robot):
  The box becomes a larger "grown" box

     ########
     #      #
     #      #
     ########    o  Point representing robot's (x,y)
```

Formal pieces:
- C: Set of all configurations.
- C_obs: Set of configurations where robot collides with obstacles.
- C_free = C \ C_obs.
- Path planning reduces to finding a continuous map σ: [0,1] → C_free, with σ(0)=qstart, σ(1)=qgoal.

This week we focus on geometric planning in 2D for intuition. Next weeks add dynamics/kinematics constraints.

---

## 6. Discrete vs continuous planning
- **Discrete planning**: Build a graph/roadmap over C_free and search (BFS, Dijkstra, A*). Common when map is a grid or you can sample waypoints.
  - Pros: Simple, many guarantees on discrete graphs, widely used.
  - Cons: Grid resolution affects quality; high dimensionality is expensive.
- **Continuous planning**: Directly plan in continuous C. Techniques include potential fields, sampling-based planners (PRM, RRT), and variational/optimization methods.
  - Pros: Handles continuous spaces naturally; scalable via sampling.
  - Cons: Requires careful collision checking; can have probabilistic guarantees instead of deterministic.

We will start with a grid-based map and BFS/A* for simplicity, then connect to swerve control.

---

## 7. Simple algorithms you can implement now

### 7.1 Occupancy grid and BFS pathfinding (2D)
- Represent the environment as a 2D grid; 0 = free, 1 = occupied.
- Use BFS for shortest-path in number of grid steps (4- or 8-connected).
- BFS guarantees shortest path in an unweighted grid.

Pseudocode:
```python
from collections import deque

def bfs_grid(grid, start, goal, neighbors="8"):
    # grid: 2D list; 0 = free, 1 = obstacle
    # start, goal: (row, col)
    # neighbors: "4" or "8" for connectivity
    rows, cols = len(grid), len(grid[0])
    nbrs4 = [(-1,0), (1,0), (0,-1), (0,1)]
    nbrs8 = nbrs4 + [(-1,-1), (-1,1), (1,-1), (1,1)]
    steps = nbrs4 if neighbors == "4" else nbrs8

    frontier = deque([start])
    came_from = {start: None}
    visited = set([start])

    while frontier:
        r, c = frontier.popleft()
        if (r, c) == goal:
            break
        for dr, dc in steps:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                if (nr, nc) not in visited:
                    visited.add((nr, nc))
                    came_from[(nr, nc)] = (r, c)
                    frontier.append((nr, nc))

    if goal not in came_from:
        return None  # no path

    # Reconstruct path
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path
```

Small example (text grid):
```
S . . # .
. # . # .
. # . . G
. . . # .
```
- S = start, G = goal, . = free, # = obstacle.
- BFS returns a set of grid cells forming the shortest path.

Discussion:
- Use A* with Euclidean or Manhattan heuristic for faster search on larger grids.
- Discretization artifacts can create jagged paths; smoothing comes later.

### 7.2 From grid path to swerve velocities (high-level)
- Convert cell centers to waypoints in meters.
- At each timestep, compute the desired velocity vector towards the next waypoint; optionally add a heading policy (e.g., face direction of travel or face goal).
- Feed desired vx, vy, ω to swerve kinematics to get per-module speed and angle.

Pseudocode (high-level):
```python
def follow_waypoints(robot_pose, waypoints, max_speed, lookahead=0.2):
    # robot_pose: (x, y, theta)
    # waypoints: list of (x, y)
    # return desired (vx, vy, omega) in robot frame
    target = pick_lookahead_point(robot_pose, waypoints, lookahead)
    ex, ey = target[0] - robot_pose[0], target[1] - robot_pose[1]
    # rotate world error into robot frame
    vx_des =  cos(robot_pose[2]) * ex + sin(robot_pose[2]) * ey
    vy_des = -sin(robot_pose[2]) * ex + cos(robot_pose[2]) * ey
    speed = (vx_des**2 + vy_des**2) ** 0.5
    if speed > max_speed:
        scale = max_speed / speed
        vx_des *= scale
        vy_des *= scale

    # Heading policy: point towards motion direction (or fixed goal heading)
    theta_ref = math.atan2(ey, ex)
    e_theta = wrap_to_pi(theta_ref - robot_pose[2])
    omega_des = clamp(kp_theta * e_theta, -omega_max, omega_max)
    return vx_des, vy_des, omega_des
```

This bridges discrete planning (waypoints) to continuous control inputs for a swerve base.

---

## 8. Formal problem formulation (LaValle’s lens)
A basic geometric planning problem:
- **C**: configuration space (e.g., SE(2) for planar robots).
- **qstart, qgoal ∈ C_free**: start and goal configurations.
- **C_obs ⊂ C**: obstacle region.
- **C_free = C \ C_obs**.
- **Objective**: find a continuous path σ: [0,1] → C_free with σ(0) = qstart, σ(1) = qgoal.
- **Cost functional** (optional this week): path length L(σ), curvature penalties, clearance.

When adding kinematic/differential constraints (nonholonomic), the feasible velocities at each configuration are limited; then planning is often done in state space with a system model ẋ = f(x, u).

For swerve:
- Often approximated as holonomic at the kinematic level: you can command any planar twist (vx, vy, ω) within bounds.
- This simplifies planning since differential constraints are mild compared to car-like robots.

---

## 9. Lab setup: development environment
We’ll use Python for quick algorithm prototyping and ROS for robot middleware, messaging, and simulation/integration.

### 9.1 OS assumptions
- Linux is recommended (Ubuntu 22.04 LTS or 24.04 LTS). The user environment is Linux, which is ideal.
- If you must use Windows or macOS, consider WSL2 (Windows) or containerization.

### 9.2 Python environment
- Install Python 3.10+ and pip.
- Create a virtual environment to isolate dependencies.
- Install common scientific packages.

Commands:
```bash
# Install Python and tools (Ubuntu)
sudo apt update && sudo apt install -y python3 python3-pip python3-venv

# Create and activate a virtual environment
python3 -m venv ~/ppc_venv
source ~/ppc_venv/bin/activate

# Upgrade pip and install packages
pip install --upgrade pip
pip install numpy matplotlib scipy networkx jupyter
```

Optional but useful:
```bash
pip install shapely pillow tqdm
```

Test:
```bash
python -c "import numpy, matplotlib; print('Python OK')"
```

### 9.3 Jupyter for quick experiments
```bash
pip install jupyterlab
jupyter lab  # or jupyter notebook
```
- Create a notebook for Week 1: “W1_Planning_Basics.ipynb”.
- Use it to prototype BFS and swerve kinematic helpers.

### 9.4 ROS 2 installation (Humble for Ubuntu 22.04; Jazzy for Ubuntu 24.04)
Follow the official ROS 2 installation guide for your distro (see ROS 2 docs for “Installation”).
- For Ubuntu 22.04 (Humble Hawksbill), the minimal steps are:

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Source setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:
```bash
ros2 run demo_nodes_cpp talker | cat
# In another terminal:
ros2 run demo_nodes_cpp listener | cat
```

### 9.5 Create a ROS 2 workspace
```bash
mkdir -p ~/ppc_ws/src
cd ~/ppc_ws
colcon build | cat
echo "source ~/ppc_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 9.6 IDE and linters
- VS Code with the Python and ROS extensions.
- Linters/formatters: `ruff`, `black`, `clang-format` (if using C++), installed as needed.

### 9.7 Connecting to this repo
- Clone this repository into `~/ppc_ws/src` or another workspace directory you prefer.
- If a `CMakeLists.txt`/`package.xml` exists (ROS 2 package), `colcon build` will include it; otherwise, use the Python path or your venv for non-ROS code.
- We will use the swerve modules/examples in this repo as the running example for planners you write this week and next.

---

## 10. Worked examples

### 10.1 Inflate obstacles to build C-space for a disk robot
Given:
- Robot radius r.
- Binary image map M (1 = obstacle, 0 = free).
- Compute inflated obstacles via binary dilation with a disk of radius r.

Pseudocode:
```python
def inflate_obstacles(binary_map, radius_cells):
    # binary_map: 2D array (1=obst, 0=free); radius_cells: inflation in grid cells
    inflated = zeros_like(binary_map)
    for r in range(rows(binary_map)):
        for c in range(cols(binary_map)):
            if binary_map[r][c] == 1:
                for dr in range(-radius_cells, radius_cells+1):
                    for dc in range(-radius_cells, radius_cells+1):
                        if dr*dr + dc*dc <= radius_cells*radius_cells:
                            rr, cc = r + dr, c + dc
                            if 0 <= rr < rows(binary_map) and 0 <= cc < cols(binary_map):
                                inflated[rr][cc] = 1
    return inflated
```
- This approximates Minkowski sum in grid space.

### 10.2 BFS on an occupancy grid, then visualize
- Use BFS pseudocode above to compute a path.
- Plot grid and path using `matplotlib` in Python.

Minimal plotting example:
```python
import numpy as np
import matplotlib.pyplot as plt

def plot_grid_path(grid, path):
    grid = np.array(grid)
    plt.imshow(grid, cmap="gray_r", origin="lower")
    if path is not None:
        ys = [p[0] for p in path]
        xs = [p[1] for p in path]
        plt.plot(xs, ys, 'r-')
        plt.plot(xs[0], ys[0], 'go', label='start')
        plt.plot(xs[-1], ys[-1], 'bx', label='goal')
        plt.legend()
    plt.title("BFS Path on Occupancy Grid")
    plt.show()
```

### 10.3 Swerve module commands from a desired twist
- Suppose wheel positions for an L x W chassis (centered at origin):
  - FL: (+L/2, +W/2)
  - FR: (+L/2, −W/2)
  - RL: (−L/2, +W/2)
  - RR: (−L/2, −W/2)

Pseudocode:
```python
import math

def optimize_module_angle(angle, speed):
    # No-op for now; later you can implement a "flip" if needed
    return angle, speed

def swerve_commands(vx, vy, omega, L, W, max_wheel_speed):
    wheel_positions = [
        ( L/2.0,  W/2.0),  # FL
        ( L/2.0, -W/2.0),  # FR
        (-L/2.0,  W/2.0),  # RL
        (-L/2.0, -W/2.0),  # RR
    ]
    return compute_swerve_modules(vx, vy, omega, wheel_positions, max_wheel_speed)
```

Experiment:
- Try vx=0.5 m/s, vy=0.0, ω=0.0 → all wheel angles ~0, equal speeds.
- Try vx=0.0, vy=0.5 → angles ~π/2, equal speeds.
- Try vx=0.0, vy=0.0, ω=1.0 → each module angle points tangentially; speeds proportional to distance from center.
- Try mixed vx, vy, ω for combined translation and rotation.

---

## 11. Common pitfalls and tips
- **Ignoring robot size**: Plan collisions happen if you don’t inflate obstacles or account for robot geometry.
- **Grid resolution too coarse**: Paths clip corners; too fine is computationally heavy.
- **Unclear frames**: Always track whether vectors are in robot or world frame. For swerve, inputs are typically robot frame; field-centric converts first.
- **Wheel angle wrap**: Avoid large angle flips by optimizing angles; minimize travel to reduce actuator load.
- **Velocity saturation**: Scale module speeds uniformly to obey wheel limits; keep the heading strategy consistent under scaling.
- **Nonholonomic effects**: Real swerve may not be perfectly holonomic due to friction, lag, or actuator limits; controllers must handle these.

---

## 12. How this connects to LaValle (Ch. 1–2)
- Ch. 1: Motivation, robot types, planning pipeline—maps directly to our overview of mobile robots and the role of planning vs control.
- Ch. 2: Basic mathematical models—configuration space, obstacles, state vs configuration, holonomic constraints. The disk-robot inflation and C-space vocabulary come straight from here.

Suggested reading tasks:
- Identify the definitions of C, C_free, C_obs in your own words.
- Sketch a simple environment and its inflated obstacles for a disk robot.
- List assumptions needed to treat a swerve drive as kinematic and near-holonomic.

---

## 13. Lab: deliverables and what to turn in
By the end of Week 1, submit:
- A short notebook/script demonstrating:
  - Building a small occupancy grid with obstacles.
  - Inflating obstacles for a disk robot of radius r (grid cells).
  - Running BFS (or A*) to obtain a path.
  - Plotting the grid and path via matplotlib.
- A short function that computes swerve module angles and speeds for a given chassis twist (vx, vy, ω) and chassis dimensions (L, W), including speed scaling.
- A 1–2 paragraph reflection: “What does configuration space buy us, and how would planning differ for a car-like robot vs a swerve base?”

Rubric:
- Correctness of BFS/A* implementation and path reconstruction.
- Correctness of C-space inflation (reasonable approximation).
- Correct mapping from chassis twist to module commands.
- Code clarity, comments for nontrivial parts, and plots readability.

---

## 14. Optional extensions (if you have time)
- Replace BFS with A* using Euclidean heuristic; compare node expansions and runtime.
- Path smoothing: fit a polyline with shortcutting or a cubic spline; visualize.
- Simulate a simple “pure pursuit” controller that follows the smoothed path and outputs (vx, vy, ω).
- Add angle optimization to swerve module steering (flip by π when beneficial).
- Field-centric commands: given a global heading estimate, rotate desired velocity vectors into robot frame before swerve mapping.

---

## 15. Quick reference cheatsheet

- **Configuration space**:
  - Robot config q: pose parameters that determine geometry in the workspace.
  - C_obs: configs that cause collision.
  - Inflate obstacles for disk robots: approximate Minkowski sum.

- **Grid BFS**:
  - 4- vs 8-connected neighbors.
  - BFS returns shortest path in edges; use A* for performance on larger grids.

- **Swerve kinematics**:
  - v_i = [vx − ω·ryi, vy + ω·rxi].
  - speed = sqrt(vix^2 + viy^2), angle = atan2(viy, vix).
  - Scale speeds if any exceed limits; consider angle flip optimization.

- **Mapping path to control**:
  - Convert path cells → waypoints in metric space.
  - Compute robot-frame (vx, vy, ω) towards a lookahead target.
  - Feed to swerve module mapping.

- **Development environment**:
  - Python venv with numpy/matplotlib/scipy.
  - Jupyter for quick experiments.
  - ROS 2 (Humble/Jazzy) for middleware and larger integration.
  - Build ROS workspace with colcon; source setup scripts.

---

## 16. Minimal study questions
- What are the differences between configuration and state? Give an example for a planar base.
- Why do we inflate obstacles for disk robots? What does this achieve computationally?
- In a swerve base, how do ω and wheel position affect a module’s speed and angle?
- What are the tradeoffs between 4- and 8-connected grids in BFS?
- How does A* improve on BFS? What makes a heuristic “admissible”?

---

## 17. Looking ahead
- Week 2: Search-based planners (A*, D*, heuristic design), cost maps, and path smoothing; integrate a simple follower with the swerve base in this repo.
- Weeks 3–4: Sampling-based motion planning (PRM, RRT/RRT*), collision checking, and trajectory generation.
- Weeks 5+: Feedback control, trajectory tracking, and robustness to uncertainty.

---

## 18. Short glossary
- **C-space (Configuration space)**: Space of all robot configurations.
- **C_free**: Collision-free subset of C.
- **BFS**: Breadth-first search on a graph; finds shortest path in unweighted graphs.
- **A***: Best-first search guided by a heuristic; optimal with admissible heuristic.
- **Minkowski sum**: Geometric operation used to “grow” obstacles by robot shape.
- **Holonomic**: Controllable in all DOFs instantaneously (idealized).
- **Swerve drive**: Omnidirectional base with independently steerable and driven wheels.

---

## 19. Quick setup checklist
- Python venv created and activated.
- Numpy/matplotlib installed and verified.
- Jupyter working; a Week 1 notebook started.
- ROS 2 installed; demo talker/listener verified.
- Repo cloned; workspace builds (if ROS package), or Python modules import cleanly.
- BFS example runs and produces a plot.
- Swerve module mapping function returns plausible angles/speeds for test twists.

---

- Implement BFS on a grid and obstacle inflation in your notebook.
- Implement the swerve kinematics function and test on sample twists.
- Ensure Python and ROS environments are working to support later labs.

