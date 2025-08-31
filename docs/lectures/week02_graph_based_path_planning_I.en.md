### Week 2: Graph-Based Path Planning I — Grid Representations and Dijkstra’s Algorithm

Audience: CS undergraduates (robotics/computer science)

Reading before class:
- LaValle, Planning Algorithms, Chapter 3 (Configuration Space)
- Choset et al., Principles of Robot Motion, Chapter 2 (Configuration Space and Graph Search)

Learning objectives
- Understand grid-based configuration-space representations and occupancy maps
- Construct graphs from 2D grids (4- and 8-connected)
- Derive and implement Dijkstra’s algorithm for shortest paths
- Analyze Dijkstra’s time and space complexity
- Apply Dijkstra on a 2D grid with obstacles
- Connect grid/Dijkstra outputs to downstream motion for swerve-drive robots in this repo

---

### 1. Motivation: From continuous space to discrete search
Robots navigate continuous environments but classical search often operates on discrete structures. We discretize the space into a grid and run a shortest-path algorithm. This yields a kinematically agnostic global path that a local controller (e.g., MPPI in this swerve-drive repo) tracks while respecting dynamics.

Why start with grids?
- Simple, uniform representation of free vs occupied space
- Easy to implement and visualize
- Directly maps to standard graph-search algorithms
- Serves as the basis for more advanced planners (A*, D*, PRM/graph variants)

Visual (text-described): A 2D floor plan as a matrix of cells. Black cells indicate obstacles, white cells free space. A start cell S and goal cell G are marked. Edges connect neighboring free cells (4- or 8-connected), forming a graph.

---

### 2. Grid-based representations
Key terms
- Occupancy grid: Discrete cells with binary or probabilistic occupancy.
- Resolution: Physical size per cell (e.g., 0.1 m/cell). Higher resolution captures detail but increases computational cost.
- Connectivity: Neighborhood model defining edges between cells.
  - 4-connected (Manhattan moves: up, down, left, right)
  - 8-connected (adds diagonals)
- Cost model: Typically uniform 1 for orthogonal moves; √2 for diagonals in 8-connected; optionally terrain or inflation costs.

From readings:
- LaValle Ch. 3: Discretization arises after modeling configuration space (C-space). Grid cells represent configurations (x, y) as free or in collision.
- Choset Ch. 2: Distinguishes between configuration space and workspace; introduces adjacency graphs and search fundamentals.

Practical considerations
- Map origin and orientation matter when converting between world coordinates and grid indices.
- Obstacle inflation: Inflate obstacles by robot footprint radius in cells to maintain clearance.
- Boundaries: Treat out-of-bounds as occupied.

Visual (text-described): Show a 10×10 grid with a 2-cell-radius inflation halo around obstacles (gray ring around black obstacle cells), shrinking the navigable corridor.

---

### 3. Building a graph from a grid
Given a boolean 2D array `grid[h][w]`, where `False` is free and `True` is occupied:
- Each free cell is a node
- For each node, add edges to neighboring free cells according to the connectivity
- Edge weight: 1 for orthogonal; √2 for diagonal (8-connected)

Coordinate convention used below
- Row-major indexing `(r, c)` with `r ∈ [0, h), c ∈ [0, w)`
- Start and goal as `(r_s, c_s)` and `(r_g, c_g)`

Edge sets
- 4-connected: neighbors = {(−1, 0), (1, 0), (0, −1), (0, 1)}
- 8-connected: + {(−1, −1), (−1, 1), (1, −1), (1, 1)} with diagonal cost √2

---

### 4. Dijkstra’s algorithm for shortest paths
Goal: Find the minimum-cost path from a source node to all nodes in a graph with nonnegative edge weights; we will extract the source-to-goal path.

Algorithm intuition
- Maintain a tentative distance for each node; initialize start to 0 and others to ∞
- Repeatedly select the node with the smallest tentative distance that has not been finalized
- Relax edges from that node: if going through it lowers a neighbor’s distance, update
- Continue until all nodes processed or the goal is finalized

Data structures
- Min-priority queue (binary heap) keyed by tentative distance
- Distance dictionary `dist[node]`
- Predecessor dictionary `parent[node]` for path reconstruction
- Visited/finalized set `closed`

Pseudocode
- Initialize `dist[s] = 0`, push `(0, s)` into heap; others `∞`
- While heap not empty:
  - Pop `(d, u)`; if `u` in `closed`, continue
  - Add `u` to `closed`; if `u == goal`, stop early
  - For each neighbor `v` of `u` with edge weight `w`:
    - If `d + w < dist[v]`, set `dist[v] = d + w`, `parent[v] = u`, and push `(dist[v], v)`
- Reconstruct path by following `parent` from goal back to start

Correctness
- Works for nonnegative weights due to greedy selection ordering distances by a cut property (see Choset Ch. 2; LaValle Ch. 3 references to shortest-path properties).

---

### 5. Python utilities and Dijkstra implementation
Below is minimal, clear Python code that:
- Represents grid maps and neighbors (4- and 8-connected)
- Runs Dijkstra from start to goal
- Reconstructs the path
- Includes a small obstacle example

```python
from __future__ import annotations
from typing import List, Tuple, Optional, Dict
import math
import heapq

Grid = List[List[bool]]  # True = occupied, False = free
Index = Tuple[int, int]  # (row, col)

ORTHO_NEIGHBORS: List[Tuple[int, int]] = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIAG_NEIGHBORS: List[Tuple[int, int]] = [(-1, -1), (-1, 1), (1, -1), (1, 1)]


def in_bounds(grid: Grid, r: int, c: int) -> bool:
    return 0 <= r < len(grid) and 0 <= c < len(grid[0])


def is_free(grid: Grid, r: int, c: int) -> bool:
    return not grid[r][c]


def neighbors(grid: Grid, node: Index, use_diagonals: bool = False) -> List[Tuple[Index, float]]:
    r, c = node
    nbr_offsets = ORTHO_NEIGHBORS + (DIAG_NEIGHBORS if use_diagonals else [])
    result: List[Tuple[Index, float]] = []
    for dr, dc in nbr_offsets:
        nr, nc = r + dr, c + dc
        if not in_bounds(grid, nr, nc):
            continue
        if not is_free(grid, nr, nc):
            continue
        cost = math.sqrt(2.0) if (dr != 0 and dc != 0) else 1.0
        result.append(((nr, nc), cost))
    return result


def dijkstra(grid: Grid, start: Index, goal: Index, use_diagonals: bool = False) -> Optional[List[Index]]:
    if not is_free(grid, start[0], start[1]) or not is_free(grid, goal[0], goal[1]):
        return None

    dist: Dict[Index, float] = {}
    parent: Dict[Index, Optional[Index]] = {}
    pq: List[Tuple[float, Index]] = []
    closed: set[Index] = set()

    dist[start] = 0.0
    parent[start] = None
    heapq.heappush(pq, (0.0, start))

    while pq:
        d_u, u = heapq.heappop(pq)
        if u in closed:
            continue
        closed.add(u)
        if u == goal:
            break

        for v, w in neighbors(grid, u, use_diagonals):
            if v in closed:
                continue
            alt = d_u + w
            if alt < dist.get(v, math.inf):
                dist[v] = alt
                parent[v] = u
                heapq.heappush(pq, (alt, v))

    if goal not in parent:
        return None

    # Reconstruct path
    path: List[Index] = []
    cur: Optional[Index] = goal
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)
    path.reverse()
    return path


def pretty_print_grid_with_path(grid: Grid, path: Optional[List[Index]] = None, start: Index | None = None, goal: Index | None = None) -> None:
    path_set = set(path) if path else set()
    for r in range(len(grid)):
        row_chars: List[str] = []
        for c in range(len(grid[0])):
            if start is not None and (r, c) == start:
                row_chars.append('S')
            elif goal is not None and (r, c) == goal:
                row_chars.append('G')
            elif (r, c) in path_set:
                row_chars.append('*')
            else:
                row_chars.append('#' if grid[r][c] else '.')
        print(''.join(row_chars))


if __name__ == "__main__":
    # Example 10x10 grid with a wall and a gap
    w, h = 10, 10
    grid: Grid = [[False for _ in range(w)] for _ in range(h)]
    # Add a horizontal wall at row 5 with a gap at column 7
    for c in range(1, 9):
        grid[5][c] = True
    grid[5][7] = False

    start = (2, 2)
    goal = (8, 8)

    path4 = dijkstra(grid, start, goal, use_diagonals=False)
    print("4-connected:")
    pretty_print_grid_with_path(grid, path4, start, goal)

    print()  # spacer

    path8 = dijkstra(grid, start, goal, use_diagonals=True)
    print("8-connected:")
    pretty_print_grid_with_path(grid, path8, start, goal)
```

Expected console visuals (text-described)
- Free cells: `.`; obstacles: `#`; path: `*`; start `S`; goal `G`
- Students can observe that 8-connected paths are shorter and smoother visually than 4-connected paths.

---

### 6. Complexity analysis
Let N be the number of free cells (nodes). For grid graphs, each node has O(1) neighbors (4 or 8).
- Using a binary heap:
  - Extract-min: O(log N) per node (up to N times)
  - Decrease-key modeled by push: O(log N) per edge relaxation
  - Total edges E = O(N)
  - Time: O((N + E) log N) = O(N log N)
  - Space: O(N) for distances, parents, heap

Special cases
- If uniform costs and a 4-connected grid with weight 1, Breadth-First Search (BFS) can be used instead with O(N) time and O(N) space; Dijkstra reduces to BFS in that case.
- With a d-ary heap or Fibonacci heap, asymptotics can improve theoretically, but constants matter in practice.

---

### 7. Example scenarios with obstacles
Scenario A: Narrow passage
- A thick obstacle wall with a single-cell gap creates a bottleneck.
- Dijkstra finds the unique shortest path through the gap.
- Visual (text-described): A long `#####` barrier across the grid with one `.` gap and the path threading through it.

Scenario B: U-shaped obstacle
- The path must go around the U’s arms; diagonal connectivity can meaningfully shorten the route.
- Visual (text-described): A `U` composed of `#` with `S` outside and `G` inside; 8-connected reduces detour steps vs 4-connected.

Scenario C: Inflated obstacles for safety
- Inflate by k cells to maintain clearance for a robot with nonzero radius; the path pushes away from walls.
- Visual (text-described): Obstacles with a gray band of inflated cells; the planned path avoids this band.

Implementation note: For inflation, run a distance transform or simple k-ring dilation on occupied cells and mark inflated cells as occupied in the planning grid.

---

### 8. Connecting to swerve-drive robots in this repo
This repository targets a swerve (4WIDS) robot controlled by MPPI. Dijkstra provides a kinematic-agnostic global path, which the local planner tracks while respecting the swerve’s holonomic capabilities.

Key integration concepts
- Global vs local planning: Use Dijkstra to compute a coarse path over the occupancy grid (global). Feed this path to a local controller (e.g., MPPI) that outputs feasible velocities and wheel module commands.
- Holonomic advantage: Swerve drive can move laterally and rotate independently, making the local tracking of grid paths more forgiving than for nonholonomic bases. Still, ensure reasonable curvature and avoid extremely jagged paths.
- Costmap parameters: In ROS stacks, you will see configs like `TebLocalPlannerROS` and `TrajectoryPlannerROS`. Holonomic settings and footprint sizes affect obstacle inflation and path clearance requirements.
- Practical tip: Post-process the grid path for waypoints downsampling or short-cuts, yielding fewer, smoother waypoints for the swerve’s local controller.

Visual (text-described): Show a piecewise-linear grid shortest path over a costmap and a smoother, down-sampled waypoint sequence overlayed. The swerve robot tracks the smooth path while avoiding newly sensed obstacles.

---

### 9. Worked example: From grid to swerve-friendly waypoints
Given a discrete path `[(r0,c0), (r1,c1), ..., (rk, ck)]`, convert to metric positions if the map resolution is `res` meters per cell and the grid’s origin is `(x0, y0)` (meters) at cell (0,0):
- For each `(r, c)`, compute `(x, y) = (x0 + c * res, y0 + r * res)`
- Optionally smooth and downsample (e.g., keep corners where direction changes)
- Provide the list of `(x, y)` waypoints to the local planner/controller

```python
def grid_path_to_metric(path: List[Index], origin_xy=(0.0, 0.0), resolution: float = 0.1):
    x0, y0 = origin_xy
    return [(x0 + c * resolution, y0 + r * resolution) for (r, c) in path]


def downsample_corners(path: List[Index]) -> List[Index]:
    if not path:
        return path
    def direction(a: Index, b: Index) -> Tuple[int, int]:
        return (b[0] - a[0], b[1] - a[1])

    result = [path[0]]
    last_dir = None
    for i in range(1, len(path)):
        cur_dir = direction(path[i - 1], path[i])
        if cur_dir != last_dir:
            result.append(path[i])
            last_dir = cur_dir
    return result
```

Remark: The MPPI controller in this repo operates in continuous state/control spaces; a waypoint sequence derived from the grid path informs its reference trajectory. Ensure waypoint spacing is not too dense to avoid oscillatory tracking.

---

### 10. Common pitfalls and debugging tips
- Start/goal on obstacle: Validate inputs and fail fast.
- Disconnected regions: Return `None` when unreachable; visualize obstacles and check connectivity.
- Diagonal cutting: With 8-connectivity and large robot radius, ensure diagonals do not slip through corners that are physically blocked after inflation.
- Heaviest cost normalization: If you add terrain costs, verify nonnegativity for Dijkstra’s correctness.
- Performance: For large maps, consider A* with an admissible heuristic to accelerate search, or multi-resolution planning.

---

### 11. In-class exercises
1) Neighbor models
- Write a function that toggles between 4- and 8-connected neighbors and compare path lengths on the same map.
- Explain why the 8-connected path length in grid steps approximates Euclidean distance better (due to diagonal moves costing √2).

2) Inflation
- Implement a simple inflation routine that marks all cells within k-Manhattan distance of any obstacle as occupied. Show how paths move away from walls as k increases.

3) Path post-processing
- Implement `downsample_corners` and compare tracking behavior when using all cells vs only corner breakpoints.

4) Complexity experiment
- Time Dijkstra on grids of size 50×50, 100×100, 200×200 with random obstacle densities (10%, 30%). Plot time vs N and estimate the slope compared to N log N.

---

### 12. Lab: Implement Dijkstra on a 2D grid map
Purpose
- Gain hands-on experience implementing Dijkstra for grid maps with obstacles
- Practice visualization and performance measurement

Deliverables
- A Python module `grid_dijkstra.py` with:
  - Grid representation
  - Dijkstra implementation
  - Command-line interface to load a map file and output a path
- A short report with: path visualization, runtime measurements, and discussion

Starter map format
- Plain text with `.` for free, `#` for obstacle, `S` start, `G` goal
- Example file `maps/example1.txt` (create for yourself):
```
##########
#S.....#.#
#.####.#.#
#.#..#.#.#
#.#..#...#
#.#..#####[
#.#.......
#.#.#####.
#...#...G#
##########
```
(Note: Ensure all rows have equal width; fix typos like stray brackets before use.)

Tasks (step-by-step)
1) Parse the map file into a 2D boolean grid and locate `S` and `G`.
2) Implement neighbor generation for 4-connected and optionally 8-connected.
3) Implement Dijkstra using a binary heap; return the path or report “unreachable”.
4) Visualize the path back on the ASCII grid and also save a simple PNG using matplotlib (optional but encouraged).
5) Add obstacle inflation parameter `k`; rebuild an inflated planning grid and rerun.
6) Convert the path to metric coordinates (assume resolution = 0.1 m) and write waypoints to `waypoints.csv`.
7) Measure runtime vs grid size and obstacle density; tabulate results.

Reference implementation sketch (students will write their own)
```python
#!/usr/bin/env python3
import argparse
import math
import heapq
from typing import List, Tuple, Optional, Dict

Grid = List[List[bool]]
Index = Tuple[int, int]

ORTHO = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIAG = [(-1, -1), (-1, 1), (1, -1), (1, 1)]

def read_ascii_map(path: str):
    grid: Grid = []
    start = goal = None
    with open(path, 'r') as f:
        for r, line in enumerate(f.read().splitlines()):
            row = []
            for c, ch in enumerate(line):
                if ch == '#':
                    row.append(True)
                else:
                    row.append(False)
                if ch == 'S':
                    start = (r, c)
                elif ch == 'G':
                    goal = (r, c)
            grid.append(row)
    if start is None or goal is None:
        raise ValueError("Map must contain S and G")
    return grid, start, goal


def inflate(grid: Grid, k: int) -> Grid:
    if k <= 0:
        return [row[:] for row in grid]
    h, w = len(grid), len(grid[0])
    inflated = [[cell for cell in row] for row in grid]
    for r in range(h):
        for c in range(w):
            if grid[r][c]:
                for dr in range(-k, k + 1):
                    for dc in range(-k, k + 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and abs(dr) + abs(dc) <= k:
                            inflated[rr][cc] = True
    return inflated


def neighbors(grid: Grid, node: Index, diag: bool) -> List[Tuple[Index, float]]:
    r, c = node
    result = []
    for dr, dc in (ORTHO + (DIAG if diag else [])):
        rr, cc = r + dr, c + dc
        if 0 <= rr < len(grid) and 0 <= cc < len(grid[0]) and not grid[rr][cc]:
            cost = math.sqrt(2.0) if (dr != 0 and dc != 0) else 1.0
            result.append(((rr, cc), cost))
    return result


def dijkstra(grid: Grid, start: Index, goal: Index, diag: bool) -> Optional[List[Index]]:
    if grid[start[0]][start[1]] or grid[goal[0]][goal[1]]:
        return None
    dist: Dict[Index, float] = {start: 0.0}
    parent: Dict[Index, Optional[Index]] = {start: None}
    pq = [(0.0, start)]
    closed = set()
    while pq:
        du, u = heapq.heappop(pq)
        if u in closed:
            continue
        closed.add(u)
        if u == goal:
            break
        for v, w in neighbors(grid, u, diag):
            if v in closed:
                continue
            alt = du + w
            if alt < dist.get(v, math.inf):
                dist[v] = alt
                parent[v] = u
                heapq.heappush(pq, (alt, v))
    if goal not in parent:
        return None
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    return list(reversed(path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_file')
    ap.add_argument('--diag', action='store_true', help='use 8-connected neighbors')
    ap.add_argument('-k', '--inflate', type=int, default=0, help='inflation radius in cells')
    args = ap.parse_args()

    grid, start, goal = read_ascii_map(args.map_file)
    grid = inflate(grid, args.inflate)
    path = dijkstra(grid, start, goal, args.diag)
    if path is None:
        print('No path found')
    else:
        print(f'Path length (cells): {len(path)}')
        # Simple ASCII visualization
        g2 = [row[:] for row in grid]
        for r, c in path:
            g2[r][c] = False  # keep free
        # Print
        path_set = set(path)
        for r in range(len(grid)):
            line = []
            for c in range(len(grid[0])):
                ch = '#'
                if not grid[r][c]:
                    ch = '.'
                if (r, c) == start:
                    ch = 'S'
                if (r, c) == goal:
                    ch = 'G'
                if (r, c) in path_set and (r, c) not in (start, goal):
                    ch = '*'
                line.append(ch)
            print(''.join(line))

if __name__ == '__main__':
    main()
```

Evaluation rubric (brief)
- Correctness: Finds valid shortest path on provided maps; handles unreachable cases
- Software quality: Clear functions, typing, edge-case handling
- Visualization: Correctly overlays path; optional PNG bonus
- Analysis: Runtime measurements, N log N discussion, effect of inflation and connectivity

---

### 13. Extensions (optional exploration)
- Replace Dijkstra with A* using Euclidean or octile-distance heuristics; compare expansions and runtime
- Multi-resolution (coarse-to-fine) planning
- Weighted costs (terrain, inflation gradients) and their effect on paths
- Post-processing via line-of-sight shortcuts (string-pulling) before handing off to local planner

---

### 14. Quick quiz (formative assessment)
1) Why is Dijkstra correct for nonnegative edge weights but not necessarily optimal with negative edges?
2) On a 200×200 grid with 30% obstacles and 8-connected neighbors, what is the big-O time of Dijkstra with a binary heap? What dominates the constant factors?
3) Why might a swerve-drive robot benefit from path downsampling before local tracking?

---

### Appendix A. Step-by-step Dijkstra dry run (textual walk-through)
We illustrate Dijkstra on a small 5×7 grid, 4-connected, uniform costs. `#` are obstacles.

Text grid (row, col) with start S = (1,1) and goal G = (3,5):
```
0: . . . . . . .
1: . S . # . . .
2: . # . # . # .
3: . . . . G . .
4: . # . . . . .
```

Dry run highlights (priority queue holds (dist, node)):
- Init: dist[S]=0; PQ=[(0,(1,1))]
- Pop (1,1): relax (0,1),(2,1),(1,0),(1,2). Obstacles block (2,1). Push with dist=1.
- PQ=[(1,(0,1)), (1,(1,0)), (1,(1,2))]
- Pop (0,1): relax new neighbors; enqueue (0,0),(0,2),(1,1 already closed),(−1,1 out). Distances become 2 for new cells.
- Continue expansions level-by-level; first time we pop G we have its optimal distance.

Key invariants to note as you trace:
- When a node u is popped the first time, dist[u] is final (for nonnegative weights).
- Parent pointers create an acyclic tree rooted at S over finalized nodes.

Exercise: Perform the exact sequence of PQ states for this grid and reconstruct the path S→G. Then repeat with 8-connected neighbors and compare path length.

---

### Appendix B. Preventing diagonal corner-cutting (safety for finite-radius robots)
In 8-connected grids, naïve diagonal moves can "cut corners" through blocked gaps. A common rule forbids a diagonal move unless at least one of the adjacent orthogonal cells is free (or, stricter, both are free).

Python neighbor variant with a strict rule (both orthogonals free):
```python
def neighbors_no_corner_cutting(grid: Grid, node: Index, use_diagonals: bool = False) -> List[Tuple[Index, float]]:
    r, c = node
    nbr_offsets = ORTHO_NEIGHBORS + (DIAG_NEIGHBORS if use_diagonals else [])
    result: List[Tuple[Index, float]] = []
    for dr, dc in nbr_offsets:
        nr, nc = r + dr, c + dc
        if not in_bounds(grid, nr, nc):
            continue
        if not is_free(grid, nr, nc):
            continue
        if dr != 0 and dc != 0:
            # Diagonal step: require both adjacent orthogonals to be free
            if not (is_free(grid, r + dr, c) and is_free(grid, r, c + dc)):
                continue
            cost = math.sqrt(2.0)
        else:
            cost = 1.0
        result.append(((nr, nc), cost))
    return result
```

When planning for a disk robot of radius R, combine this rule with obstacle inflation by ⌈R/res⌉ cells to maintain physical clearance.

---

### Appendix C. Benchmarking and profiling guidance
To empirically validate O(N log N) behavior:
- Fix obstacle densities (e.g., 10%, 30%).
- Sweep grid sizes: 50×50, 100×100, 200×200, 400×400.
- For each size/density, generate K random maps, time Dijkstra with/without diagonals, and plot mean runtime vs N.

Skeleton driver (expand in your lab repo):
```python
import random, time, statistics as stats

def random_grid(h, w, p):
    return [[random.random() < p for _ in range(w)] for _ in range(h)]

def bench():
    sizes = [50, 100, 200, 400]
    densities = [0.1, 0.3]
    trials = 5
    for p in densities:
        for n in sizes:
            times = []
            for _ in range(trials):
                g = random_grid(n, n, p)
                s, t = (0, 0), (n-1, n-1)
                g[s[0]][s[1]] = False; g[t[0]][t[1]] = False
                t0 = time.perf_counter()
                _ = dijkstra(g, s, t, use_diagonals=True)
                times.append(time.perf_counter() - t0)
            print(f"n={n:3d}, p={p:.1f}, mean={stats.mean(times):.4f}s, stdev={stats.pstdev(times):.4f}s")

if __name__ == "__main__":
    bench()
```

Discussion prompts:
- Why do measured constants differ between 4- and 8-connected cases?
- How does obstacle density affect the number of relaxed edges before reaching the goal?

---

### Appendix D. Swerve integration steps with this repository
Bridging grid paths to the MPPI-based swerve stack in this repo:

- Generate waypoints: Convert grid cells to metric using map origin/resolution (see Section 9).
- Downsample: Reduce to corner waypoints or apply shortcutting for smoother references.
- Publish or feed to controller: In a ROS 1 Noetic run (see `README.md`), you can launch an autonomous navigation stack:
  - Manual sim world: `roslaunch launch/gazebo_world.launch gazebo_world_name:=maze`
  - MPPI navigation: `roslaunch launch/navigation.launch local_planner:=mppi_h`
- Integration options for this course:
  - Offline: Save waypoints to CSV and load them in a small node that publishes a reference trajectory topic consumed by MPPI.
  - Online: Wrap your Dijkstra in a ROS node that subscribes to a costmap/occupancy grid and publishes waypoints whenever S/G changes.

Operational tips for swerve bases:
- Keep waypoint spacing larger than the chassis footprint to avoid oscillations.
- Avoid tight zig-zags; prefer 8-connected with corner-cutting prevention and mild downsampling.
- If MPPI exhibits corner hugging, increase clearance/obstacle cost and/or inflate obstacles in the planning grid.

Visual (text-described): In `maze` world, overlay a red polyline of downsampled waypoints over the grid-free space. The swerve robot follows the polyline while MPPI slightly rounds corners for smooth steering.

---

### 15. References
- LaValle, S. M. Planning Algorithms. Chapter 3.
- Choset, H., Lynch, K. M., Hutchinson, S., et al. Principles of Robot Motion. Chapter 2.
- Cormen, T. H., Leiserson, C. E., Rivest, R. L., Stein, C. Introduction to Algorithms (Dijkstra’s algorithm).
- ROS Navigation Stack documentation (global vs local planners, costmaps).
