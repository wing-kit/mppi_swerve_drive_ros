### Week 4: Sampling-Based Path Planning

Instructor notes for a 90–120 minute lecture + 60–90 minute lab.

#### Learning Objectives
- Explain the core idea behind sampling-based planning and why it scales to high-dimensional spaces.
- Implement and visualize a basic RRT that plans a collision-free path in 2D.
- Compare PRM vs. RRT/RRT* in terms of when to use each, guarantees, and performance.
- Describe the role of randomness, sampling distributions, and goal bias; discuss probabilistic completeness and asymptotic optimality.
- Design collision-checking routines and understand their impact on planner performance.
- Connect sampling-based planning to swerve-drive robots operating in cluttered 2D workspaces with SE(2) state.

---

### 1) Motivation and Big Picture
Motion planning asks: given a robot model and a workspace with obstacles, find a path from start to goal that is collision-free and satisfies constraints. Classical grid/graph search struggles in high-dimensional continuous spaces due to discretization explosion. Sampling-based planners avoid explicit discretization of configuration space. They use random samples to probe free space, connect them with simple local planners, and search on an implicit graph.

- Configuration space (C-space): set of all robot configurations q. For planar rigid-body: q ∈ SE(2) = (x, y, θ). For manipulator: q ∈ ℝⁿ of joint angles.
- Obstacles in C-space: configurations that cause collision in workspace. Free space: C_free.
- Local planner: basic steering function used to connect nearby samples (e.g., straight-line in C-space with collision checking).
- Metrics: distances that define “nearest” and “nearby” samples (Euclidean in ℝⁿ; SE(2) with weighted orientation).

Sampling-based methods build graphs incrementally or in batches:
- Probabilistic Roadmaps (PRM): multi-query. Pre-sample milestones in C_free, connect nearby nodes, then answer many queries by graph search on the roadmap.
- Rapidly-exploring Random Trees (RRT): single-query. Grow a tree quickly into unexplored free space to reach the goal. RRT*: an optimal variant that rewires to lower path cost.

Key advantages: dimensional scalability, no explicit C-space construction, and probabilistic guarantees. Downsides: no completeness in finite time, performance can degrade with narrow passages, and quality may require many samples or post-processing.

---

### 2) Randomness in Planning
Randomness is a tool to explore C_free without gridding:

- Uniform random sampling: draws q ~ Uniform(C) and rejects collisions (rejection sampling). Simple and broadly effective.
- Goal bias: with probability p, sample the goal (or near it) to accelerate convergence.
- Low-discrepancy sequences: Halton, Hammersley, Sobol improve coverage with fewer samples (quasi-random).
- Informed sampling: bias around current best path cost (e.g., ellipsoidal sampling in RRT* after first solution).
- Bridge sampling: draw pairs of random points in collision; midpoints tend to lie in narrow passages.

Reproducibility: set RNG seeds during experiments for fair comparisons. Robust systems often combine deterministic tests with randomized planners to reduce brittleness.

Guarantees:
- Probabilistic completeness: If a solution exists, probability that the planner finds one approaches 1 as samples → ∞. PRM, RRT are probabilistically complete given mild assumptions.
- Asymptotic optimality: As samples → ∞, solution cost approaches the global optimum. PRM* and RRT* achieve this by appropriate connection radius/neighbor-count and rewiring.

---

### 3) Collision Checking Essentials
Collision checking often dominates runtime. Good planners are fast local planners.

- Robot models: point robot, disk robot (inflate obstacles by radius), polygonal robot, or bounding volumes. Use conservative approximations for speed.
- Segment checking: given q_a → q_b, interpolate intermediate states and test collisions. Use step sizes tied to geometry (e.g., ≤ robot radius) to avoid misses.
- Spatial acceleration: bounding volume hierarchies (AABB, OBB), spatial hashing, uniform grids. Broad-phase quickly filters obstacles; narrow-phase tests exact geometry.
- Distance fields: precompute signed distance grid; collision check becomes threshold check. Also supports clearance-aware planning and gradient-based smoothing.
- Lazy strategies: assume edges are free; validate only when the edge is on a candidate path. Effective in PRM variants.
- Caching: store previously validated edges/segments; reuse when revisiting similar queries.

Practical tip: Start with simple axis-aligned rectangles and a disk robot with step-sampled segment checks. This is robust for labs and demos.

---

### 4) Probabilistic Roadmaps (PRM)
PRM builds a roadmap of milestones in C_free and connects nearby nodes with a local planner. Good for static environments and multiple queries.

Algorithm (basic PRM):
1. Sample N configurations uniformly in C; keep only collision-free samples as milestones V.
2. For each v ∈ V, find k nearest neighbors (or all within radius r).
3. Attempt to connect v to neighbors with the local planner. If the connecting path is collision-free, add an undirected edge.
4. For a query (q_start, q_goal), connect each to the roadmap (local connections), then run graph search (Dijkstra/A*) to find a path.

Parameters:
- Number of milestones N.
- k-nearest vs. radius r for connections (theoretical results suggest r ∝ (log N / N)^(1/d)).
- Local planner and step size.

Pros:
- Multi-query efficiency once the roadmap is built.
- Scales to moderate-to-high dimensions.
- Paths can be improved by densifying the roadmap.

Cons:
- Roadmap quality depends on sampling and connection radius.
- Narrow passages are challenging without specialized sampling.
- Initial build can be expensive; less suitable for dynamic obstacles unless updated incrementally.

Variants:
- PRM*: asymptotically optimal using radius that shrinks with N.
- Lazy PRM: defer collision checks until needed by a query path.
- Obstacle-based sampling and bridge tests to target narrow passages.

---

### 5) Rapidly-exploring Random Trees (RRT)
RRT is a single-query planner that grows a tree rooted at the start configuration by iteratively extending toward random samples.

Core idea: bias exploration toward unexplored Voronoi regions. The nearest tree node to a random sample tends to be at the frontier, causing rapid expansion.

Basic RRT algorithm:
1. Initialize tree T with root q_start.
2. Repeat for iterations or until goal reached:
   - With probability p_goal, set q_rand ← q_goal; else sample q_rand ← SampleFree().
   - q_near ← Nearest(T, q_rand) using metric ρ.
   - q_new ← Steer(q_near, q_rand, η) where η is max step length.
   - If SegmentFree(q_near, q_new): add q_new to T with parent q_near.
   - If q_new is within goal region: return path by tracing parents.

Parameters and design choices:
- Step size η controls exploration vs. refinement; too small slows progress, too large increases collisions.
- Goal bias p_goal accelerates convergence but can cause premature greediness if too high.
- Metric: in SE(2), weight θ appropriately (e.g., ρ((x,y,θ)) = √(Δx²+Δy² + wθ Δθ²)).
- Termination: iterations, time budget, or first path found.

Pros:
- Very fast in practice; finds a feasible path quickly if one exists.
- Works with differential constraints (kinodynamic RRT) by using a forward-simulation local planner instead of straight-line steering.

Cons:
- First solution can be jagged and far from optimal.
- Quality depends on step size and sampling; may miss narrow passages without luck or specialized sampling.

Post-processing: shortcutting (randomly replace subpaths by straight segments if collision-free) and smoothing (e.g., cubic splines, time-parameterization subject to velocity/acceleration limits).

---

### 6) RRT*: Optimal Variant
RRT* modifies RRT to improve path cost over time by rewiring local neighborhoods.

Key differences from RRT:
1. Near set: instead of only the nearest neighbor, consider all nodes within a radius r_n = γ (log n / n)^(1/d) where n is current node count, dimension d, and γ > γ* ensures asymptotic optimality.
2. Best parent: choose the neighbor that yields minimum cost-to-come for q_new given a collision-free connection.
3. Rewire: for each neighbor q_near in the near set, if going through q_new reduces its cost and the edge is collision-free, change its parent to q_new.

Properties:
- Probabilistically complete and asymptotically optimal; the path cost converges to the optimum as n → ∞.
- Slower per-iteration than RRT due to neighbor search and rewiring.

Practical tips:
- Use spatial indexes (k-d trees) for Nearest and Near queries.
- Use a limited neighbor count for speed if you can’t compute a radius.
- After the first solution, use “informed” sampling inside an ellipse that contains all paths with cost ≤ current best.

---

### 7) Lab: Implement a Basic RRT in Python
Goal: Implement a 2D RRT for a disk robot among rectangular obstacles, visualize the tree and the path.

Environment setup:
- Python 3.9+
- `numpy`, `matplotlib` (install via `pip install numpy matplotlib`)

Suggested milestones:
1. Represent the world: bounds, list of axis-aligned rectangles as obstacles, robot radius r.
2. Implement collision checking: point-in-inflated-rect and segment sampling.
3. Implement RRT primitives: `sample_free`, `nearest`, `steer`, `collision_free`.
4. Run planner, visualize tree growth and final path.
5. Add goal bias and tune step size.
6. Optional: add path shortcutting and export as waypoints.

Starter code (self-contained script):

```python
import math
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import matplotlib.pyplot as plt


@dataclass
class Rectangle:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def inflated(self, r: float) -> "Rectangle":
        return Rectangle(self.x_min - r, self.y_min - r, self.x_max + r, self.y_max + r)

    def contains(self, x: float, y: float) -> bool:
        return (self.x_min <= x <= self.x_max) and (self.y_min <= y <= self.y_max)


@dataclass
class Node:
    x: float
    y: float
    parent: Optional[int]
    cost: float = 0.0


class RRT:
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float],
                 bounds: Tuple[float, float, float, float],
                 obstacles: List[Rectangle], robot_radius: float = 0.0,
                 step_size: float = 10.0, goal_sample_rate: float = 0.05,
                 max_iter: int = 5000):
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.bounds = bounds  # (xmin, ymin, xmax, ymax)
        self.obstacles = [o.inflated(robot_radius) for o in obstacles]
        self.step_size = float(step_size)
        self.goal_sample_rate = float(goal_sample_rate)
        self.max_iter = int(max_iter)
        self.nodes: List[Node] = [Node(self.start[0], self.start[1], parent=None, cost=0.0)]

    def plan(self, goal_threshold: float = 10.0, seed: Optional[int] = None) -> List[Tuple[float, float]]:
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        for _ in range(self.max_iter):
            q_rand = self._sample()
            nearest_index = self._nearest(q_rand)
            q_new = self._steer(np.array([self.nodes[nearest_index].x, self.nodes[nearest_index].y]), q_rand)
            if self._collision_free(np.array([self.nodes[nearest_index].x, self.nodes[nearest_index].y]), q_new):
                self.nodes.append(Node(q_new[0], q_new[1], parent=nearest_index,
                                       cost=self.nodes[nearest_index].cost + np.linalg.norm(q_new - np.array([self.nodes[nearest_index].x, self.nodes[nearest_index].y]))))
                if np.linalg.norm(q_new - self.goal) <= goal_threshold:
                    return self._extract_path(len(self.nodes) - 1)
        return []

    def _sample(self) -> np.ndarray:
        if random.random() < self.goal_sample_rate:
            return self.goal.copy()
        xmin, ymin, xmax, ymax = self.bounds
        for _ in range(100):  # try a few times to get a free sample
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            if self._point_free(np.array([x, y])):
                return np.array([x, y])
        # fallback: return even if not guaranteed free; extension will be checked
        return np.array([x, y])

    def _nearest(self, q: np.ndarray) -> int:
        pts = np.array([[n.x, n.y] for n in self.nodes])
        dists = np.linalg.norm(pts - q[None, :], axis=1)
        return int(np.argmin(dists))

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        direction = q_to - q_from
        dist = np.linalg.norm(direction)
        if dist < 1e-9:
            return q_from.copy()
        step = self.step_size if dist > self.step_size else dist
        return q_from + step * (direction / dist)

    def _point_free(self, q: np.ndarray) -> bool:
        x, y = float(q[0]), float(q[1])
        xmin, ymin, xmax, ymax = self.bounds
        if x < xmin or x > xmax or y < ymin or y > ymax:
            return False
        for obs in self.obstacles:
            if obs.contains(x, y):
                return False
        return True

    def _collision_free(self, q_from: np.ndarray, q_to: np.ndarray, step: float = 2.5) -> bool:
        seg = q_to - q_from
        dist = np.linalg.norm(seg)
        if dist < 1e-9:
            return self._point_free(q_from)
        num = max(2, int(math.ceil(dist / step)))
        for i in range(num + 1):
            alpha = i / num
            q = q_from + alpha * seg
            if not self._point_free(q):
                return False
        return True

    def _extract_path(self, goal_index: int) -> List[Tuple[float, float]]:
        path: List[Tuple[float, float]] = []
        i = goal_index
        while i is not None:
            n = self.nodes[i]
            path.append((n.x, n.y))
            i = n.parent
        path.reverse()
        return path


def plot_world(bounds, obstacles: List[Rectangle]):
    xmin, ymin, xmax, ymax = bounds
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    for o in obstacles:
        xs = [o.x_min, o.x_max, o.x_max, o.x_min, o.x_min]
        ys = [o.y_min, o.y_min, o.y_max, o.y_max, o.y_min]
        plt.plot(xs, ys, 'k-')


def plot_tree(rrt: RRT):
    for i, n in enumerate(rrt.nodes):
        if n.parent is not None:
            p = rrt.nodes[n.parent]
            plt.plot([p.x, n.x], [p.y, n.y], color='tab:blue', linewidth=0.8, alpha=0.6)


def main():
    random.seed(7)
    np.random.seed(7)

    bounds = (0.0, 0.0, 100.0, 100.0)
    obstacles = [
        Rectangle(20, 20, 40, 80),
        Rectangle(60, 20, 80, 80),
        Rectangle(40, 45, 60, 55),  # a thin middle bar
    ]

    start = (10.0, 10.0)
    goal = (90.0, 90.0)

    rrt = RRT(start, goal, bounds, obstacles, robot_radius=2.0,
              step_size=4.0, goal_sample_rate=0.10, max_iter=8000)
    path = rrt.plan(goal_threshold=5.0)

    plt.figure(figsize=(6, 6))
    plot_world(bounds, [o.inflated(2.0) for o in obstacles])
    plot_tree(rrt)
    if path:
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2.0, label='Path')
        plt.scatter([start[0], goal[0]], [start[1], goal[1]], c=['green', 'red'], s=60)
        plt.legend()
        plt.title('RRT Path')
    else:
        plt.title('RRT: no path found')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
```

What to look for during lab:
- Students should tune `step_size`, `goal_sample_rate`, and `goal_threshold` to get reliable performance in the toy worlds.
- Verify that collision checking is robust by adding tight gaps; ensure the planner still succeeds with enough iterations.
- Encourage using different RNG seeds to see variability in behavior.

Stretch goals:
- Add path shortcutting: repeatedly pick indices i < j and try to replace the subpath by a straight segment if collision-free.
- Implement a simple RRT* (near-neighborhood and rewiring) and compare convergence.
- Use low-discrepancy sampling (Halton) for `sample_free` and measure the difference.
- Implement informed sampling after the first solution (elliptical region between start and goal with major axis equal to current best cost).

---

### 8) Connecting to Swerve Drive Scenarios
Swerve-drive robots move omnidirectionally by steering wheel modules, offering near-holonomic motion in the plane. For planning, the state is often q = (x, y, θ) ∈ SE(2); although the robot can achieve any instantaneous planar velocity vector under constraints, orientation affects chassis and module limits and is important for final pose.

Implications for sampling-based planning:
- State space: Plan in SE(2). Use a metric with angle weight, e.g., ρ² = Δx² + Δy² + wθ Δθ² with angle wrap-around.
- Local planner: Straight-line interpolation in SE(2) for (x, y) and shortest-angle interpolation for θ is acceptable for geometric planning. For execution, time-parameterize with velocity/acceleration/jerk limits and module constraints.
- Footprint: Model as a disk or rectangle; inflate obstacles to account for footprint and tracking error. Use segment sampling fine enough for footprint size.
- Dynamic limits: Swerve is not truly unconstrained; wheel speed, steering rate, and chassis acceleration impose bounds. Kinodynamic RRT can incorporate these by forward-simulating feasible controls (e.g., sample commanded chassis velocities v_x, v_y, ω and integrate dynamics for Δt), but this is more complex than geometric RRT.
- Orientation goals: Many tasks (e.g., docking) require specific θ. Define a goal region in SE(2) that allows small tolerance in position and orientation.

Suggested classroom demonstrations:
- Run geometric RRT in SE(2) and show paths that turn θ gradually. Then apply a simple time-parameterization and depict speed profiles that respect module limits.
- Compare RRT vs. grid A*: highlight that RRT explores large open regions quickly, while A* can struggle if the grid is coarse or the heuristic weak.
- Show the effect of goal bias and step size on success time in a cluttered map resembling a field with obstacles (e.g., scoring nodes, protected zones).

---

### 9) Pros and Cons Summary

PRM:
- Pros: multi-query efficiency, easy to parallelize sampling, good in high-dimensional static spaces.
- Cons: initial build cost, difficulty with narrow passages without targeted sampling, less suitable for dynamic scenes without rebuilds.

RRT:
- Pros: fast first-feasible solution, simple, works with dynamics via kinodynamic extensions.
- Cons: non-optimal initial paths, sensitivity to parameters, may oscillate near narrow passages.

RRT*:
- Pros: asymptotically optimal, improves path quality over time, informed variants converge faster after first solution.
- Cons: heavier per-iteration cost (neighbor search + rewiring), needs good data structures.

Collision checking (cross-cutting): often the bottleneck; optimizing it helps all planners.

---

### 10) Reading and Prep
- LaValle, “Planning Algorithms,” Chapter 5 (Sampling-based Motion Planning). Focus on definitions of probabilistic completeness and RRT/PRM algorithms.
- Choset et al., “Principles of Robot Motion,” Chapter 7 (Sampling-Based Motion Planning). Focus on PRM variants and theoretical properties.

Optional reading:
- Karaman and Frazzoli, “Sampling-based Algorithms for Optimal Motion Planning” (IJRR 2011) for RRT*/PRM* foundations.

Guiding questions for students:
1. Why do sampling-based planners scale better to high dimension than grid-based planners?
2. What does probabilistic completeness mean, and why does RRT have it?
3. How does RRT* differ from RRT, and what property does it add?
4. What aspects of collision checking dominate runtime, and how can we mitigate them?
5. In a swerve-drive context, what changes when moving from geometric to kinodynamic planning?

---

### 11) Pseudocode Cheat Sheet

PRM (sketch):
```text
V ← {} ; E ← {}
for i in 1..N:
  q ← SampleFree()
  V ← V ∪ {q}
for each v in V:
  U ← Near(V, v, r) or kNN(V, v, k)
  for u in U:
    if LocalPlanner(v, u) collision-free:
      E ← E ∪ {(v, u)}
# Query
connect q_start, q_goal to V with LocalPlanner
return GraphSearch(V ∪ {q_start, q_goal}, E ∪ connections)
```

RRT (sketch):
```text
T.init(q_start)
for iter in 1..K:
  q_rand ← (goal with prob p) else SampleFree()
  q_near ← Nearest(T, q_rand)
  q_new ← Steer(q_near, q_rand, η)
  if SegmentFree(q_near, q_new):
    T.add_vertex(q_new)
    T.add_edge(q_near, q_new)
    if q_new in goal region: return Path(T, q_new)
return failure
```

RRT* (sketch):
```text
T.init(q_start)
for n in 1..N:
  q_rand ← SampleFree()
  X_near ← Near(T, q_rand, r_n)
  q_min ← argmin_{x ∈ X_near} Cost(x)+CostLine(x, q_rand) s.t. collision-free
  T.add_vertex(q_rand, parent=q_min)
  for x in X_near:
    if Cost(q_rand)+CostLine(q_rand, x) < Cost(x) and collision-free:
      rewire x parent ← q_rand
```

---

### 12) Implementation Notes and Pitfalls
- Choose a step size tied to map scale. A rule of thumb is 2–5% of the smaller map dimension.
- Ensure angle wrap-around in SE(2) metrics; use the minimal difference in θ.
- Inflate obstacles for the robot radius; alternatively, check swept volume.
- Cap iterations and add a time limit to avoid long hangs.
- Visualize often: plot the tree every few iterations to debug exploration.
- For performance, move collision checks to NumPy where possible and use vectorized distance tests.

---

### 13) Assessment Ideas
- In-lab checkoff: demonstrate a successful RRT path in two maps (one with a narrow passage). Discuss parameter tuning.
- Short quiz (pre- or post-class): definitions of probabilistic completeness and asymptotic optimality; algorithm differences between PRM and RRT*.
- Mini-writeup: a paragraph comparing uniform vs. goal-biased sampling on convergence speed for the provided map.

---

### 14) Appendix: Minimal Halton Sequence for Quasi-Random Sampling (Optional)

```python
def halton(index: int, base: int) -> float:
    f = 1.0
    r = 0.0
    i = index
    while i > 0:
        f = f / base
        r = r + f * (i % base)
        i = i // base
    return r

def halton2d(i: int, bounds: Tuple[float, float, float, float]) -> Tuple[float, float]:
    xmin, ymin, xmax, ymax = bounds
    return (xmin + (xmax - xmin) * halton(i, 2),
            ymin + (ymax - ymin) * halton(i, 3))
```

Use this to replace uniform sampling in `sample_free` to improve coverage.

---

End of Week 4 notes.

