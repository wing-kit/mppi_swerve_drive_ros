## Week 3 — Graph-Based Path Planning II: A* Search, Heuristics, Optimality, and Completeness

### Learning goals

- Understand the A* search algorithm and its relationship to Dijkstra’s algorithm
- Design admissible and consistent heuristics for grids, roadmaps, and kinematic systems
- Reason about optimality and completeness guarantees of A*
- Implement and extend A* to avoid obstacles using costmap inflation and clearance-aware costs
- Evaluate trade-offs among heuristic quality, runtime, memory, and solution quality

---

## 1) From Dijkstra to A*: Why and when to prefer informed search

- **Dijkstra** explores in rings of increasing path cost `g(n)` from the start; it is optimal for nonnegative edge costs but can expand many nodes far from the goal.
- **A*** adds goal-directed guidance via a heuristic `h(n)` estimating the remaining cost to goal. It prioritizes nodes by `f(n) = g(n) + h(n)`, expanding nodes that look promising for reaching the goal cheaply.
- **Intuition**: If `h(n)` is perfect (equal to true remaining cost), A* only follows the optimal path. If `h(n)=0`, A* reduces to Dijkstra.

ASCII grid intuition:

```
Legend: S=start, G=goal, #=obstacle, .=free, *=expanded frontier

     0 1 2 3 4 5 6 7 8
  0  . . . . . . . . G
  1  . # # # . . . . .
  2  . # . # . . . . .
  3  . # . # . . . . .
  4  S . . . . . . . .

With Dijkstra, expansions form rings centered at S. With A*, expansions bend toward G.
```

Key advantage of A*: fewer expansions for the same optimal path, when `h(n)` is informative and admissible.

---

## 2) A* search algorithm

### Core idea

- Maintain an OPEN set (priority queue) ordered by `f = g + h` and a CLOSED set of visited nodes. At each step, pop the node with the lowest `f`. If it’s the goal, reconstruct the path. Otherwise, relax edges to neighbors and update their `g`, `f`, and parent pointers if an improvement is found.

### Pseudocode

```pseudo
function A_star(start, goal, graph, heuristic):
    for each node v in graph:
        g[v] := +∞
        parent[v] := None
    g[start] := 0
    f_start := g[start] + heuristic(start, goal)
    OPEN := priority_queue ordered by f; OPEN.push((f_start, start))
    CLOSED := empty set

    while OPEN not empty:
        (f_current, u) := OPEN.pop_min()
        if u in CLOSED:  # skip stale entries
            continue
        if u == goal:
            return reconstruct_path(parent, goal), g[goal]
        CLOSED.add(u)

        for each (u, v, cost) in neighbors(u):
            if v in CLOSED:
                continue
            tentative_g := g[u] + cost
            if tentative_g < g[v]:
                parent[v] := u
                g[v] := tentative_g
                f_v := g[v] + heuristic(v, goal)
                OPEN.push((f_v, v))

    return failure  # goal unreachable

function reconstruct_path(parent, goal):
    path := []
    node := goal
    while node != None:
        path.prepend(node)
        node := parent[node]
    return path
```

Implementation notes

- Use a binary heap for `OPEN`; allow duplicates and skip stale entries when popped (simplifies decrease-key).
- Use a hash set for `CLOSED` for O(1) membership checks.
- Tie-breaking: Prefer smaller `h` (or larger `g`) to bias toward deeper exploration along promising paths. A common rule is to break ties on larger `g` to expand nodes closer to the goal surface.

Time and space complexity

- In the worst case, O(E log V) due to heap operations. Memory often dominates because OPEN and CLOSED can grow large.
- With an informative heuristic, A* dramatically reduces expansions relative to Dijkstra’s O(E log V) behavior on the same graph.

---

## 3) Heuristics: design, properties, and examples

### Definitions

- **Admissible**: `h(n) ≤ h*(n)` for all nodes, where `h*(n)` is the true minimum remaining cost to the goal. Ensures A* returns an optimal path.
- **Consistent (monotone)**: For all edges `(n, n')` with cost `c(n, n')`, `h(n) ≤ c(n, n') + h(n')`. Equivalent to the triangle inequality. Consistency implies admissibility and ensures `f(n)` is nondecreasing along a path, so nodes need not be reopened.
- **Dominance**: Heuristic `h1` dominates `h2` if `h1(n) ≥ h2(n)` for all `n` and both are admissible. Dominant heuristics expand no more nodes than dominated heuristics.

### Grid-world heuristics

Let `dx = |x - x_goal|`, `dy = |y - y_goal|`, and `c_card` be the cost of 4-connected moves and `c_diag` for diagonal moves.

- Manhattan (4-connected): `h(n) = c_card * (dx + dy)`
  - Admissible and consistent when only axis-aligned moves are allowed and each move has cost `c_card`.
- Euclidean (8-connected with uniform unit cost): `h(n) = sqrt(dx^2 + dy^2)`
  - Admissible; consistent if diagonal move cost is `sqrt(2)` when `c_card=1`.
- Chebyshev (8-connected with `max` metric): `h(n) = max(dx, dy)`
  - Admissible and consistent when `c_card = 1` and `c_diag = 1` (king’s moves in chess).
- Octile (8-connected with `c_card = 1`, `c_diag = sqrt(2)`): `h(n) = (c_card) * (dx + dy) + (c_diag - 2*c_card) * min(dx, dy)`
  - Admissible and consistent for standard grid costs.

Weighted scaling for costmaps

- If traversing near obstacles has a higher cost (inflation), ensure the heuristic never overestimates the true inflated distance. One safe approach: multiply a geometric heuristic by the minimum step cost `c_min` across all free cells: `h'(n) = c_min * h_geom(n)`.

Kinematic systems

- Dubins/Reeds-Shepp vehicles (bounded turn radius, with/without reverse): Use the shortest-path length in the corresponding metric as a heuristic. Precompute or use a library to evaluate; these distances are admissible and typically consistent given uniform controls and costs.

Roadmaps and visibility graphs

- Use straight-line Euclidean distance in configuration space if edge costs reflect Euclidean motion and obstacles do not alter per-unit cost. For anisotropic costs, lower-bound the cost by multiplying Euclidean distance with the minimum per-unit cost.

Example calculation

- Suppose `S=(1,1)`, `G=(7,5)` on a unit grid with 8-connected moves and `c_diag=√2`, `c_card=1`.
  - `dx=6`, `dy=4`.
  - Euclidean: `h_E = sqrt(6^2 + 4^2) = sqrt(52) ≈ 7.21`.
  - Octile: `h_O = (dx + dy) + (√2 - 2) * min(dx, dy) = 10 + (-0.586) * 4 ≈ 7.66`.
  - Chebyshev: `h_C = max(6, 4) = 6`.
  - Dominance (given costs): Octile ≥ Euclidean ≥ Chebyshev, but admissibility depends on the move cost model.

Pitfalls

- Overestimating `h` breaks admissibility and can yield suboptimal paths (unless using weighted A* deliberately).
- Inconsistent heuristics require node re-openings; either handle properly or ensure consistency by construction.
- Setting `h=0` loses the benefit of guidance and reverts to Dijkstra.

---

## 4) Optimality and completeness

### Optimality with admissible and consistent heuristics

- If `h` is admissible, A* is guaranteed to find an optimal solution, assuming finite branching and nonnegative costs.
- If `h` is also consistent, once a node leaves OPEN (i.e., is expanded), its `g` is the shortest-path cost. No re-openings are needed, and `f`-values along any path are nondecreasing.

Proof sketch (admissible optimality)

- Let `C*` be the optimal path cost. A* never expands a node with `f > C*` before the goal because the goal node appears in OPEN with `f(goal)=C*` once discovered. Any node with `f < C*` may be expanded, but admissibility ensures the goal’s `f` is never exceeded prior to expansion.

Proof sketch (consistency ⇒ no reopen)

- For an edge `(u, v)` with cost `c(u, v)`, consistency gives `h(u) ≤ c(u, v) + h(v)`. Rearranging yields `g(u) + h(u) ≤ g(u) + c(u, v) + h(v) = g(v) + h(v)` when `g(v) = g(u) + c(u, v)`. So along a path, `f` never decreases; once the best `g` for `u` is fixed at expansion, any later path to `u` will have `f ≥ f(u)` and cannot improve `g(u)`.

Inconsistent but admissible heuristics

- A* remains optimal if nodes can be reopened when a better `g` is found. This requires checking improvements for nodes in CLOSED and updating them accordingly (or avoiding CLOSED membership altogether and relying on keyed duplicates).

Completeness

- A* is complete if:
  - All edge costs are bounded below by a positive constant `ε > 0`, preventing infinite expansions on zero-cost cycles.
  - The branching factor is finite.
  - The goal is reachable.
- In infinite graphs or continuous spaces discretized online, completeness depends on the sampling or discretization scheme.

Weighted A* and anytime variants

- Weighted A*: `f = g + w*h` with `w > 1` accelerates search but sacrifices optimality; solution cost ≤ `w` times optimal if `h` is admissible.
- Anytime Repairing A* (ARA*): starts with `w>1` to find a quick solution, then reduces `w` and refines the path toward optimality using previous search data.

---

## 5) A* vs. Dijkstra: a concise comparison

- **Guidance**
  - Dijkstra: Uninformed; explores uniformly by `g`.
  - A*: Informed by admissible `h`; explores toward goal.
- **Optimality**
  - Both optimal with nonnegative edge costs (A* requires admissible `h`). Weighted A* may be suboptimal.
- **Efficiency**
  - Dijkstra may expand large regions irrelevant to the goal.
  - A* reduces expansions proportional to heuristic accuracy.
- **Implementation**
  - Nearly identical, with minor differences for `f` computation and tie-breaking.
- **When to use**
  - Dijkstra: baseline, multi-source multi-goal, or when no sensible heuristic exists.
  - A*: path planning with geometric heuristics, roadmaps, or navigation meshes.

---

## 6) Engineering A*: practical considerations

- **Priority queue**: Use a binary heap or pairing heap; avoid decrease-key by allowing duplicates. Keep a `best_g` map to skip stale entries.
- **Closed set**: Hash set keyed by node ID or coordinates; for continuous states, consider spatial hashing.
- **Costmaps**: For grid maps, maintain an occupancy grid and a distance transform for clearance; incorporate clearance as an additive or multiplicative term in traversal costs.
- **Heuristic scaling**: When costs vary spatially (e.g., inflated obstacles), scale heuristic by `c_min` to preserve admissibility.
- **Tie-breaking**: Favor larger `g` or smaller `h` to reduce “plateau” wandering.
- **Path smoothing**: Post-process with shortcuts or splines; if necessary, recheck collision validity after smoothing.
- **Nonholonomic constraints**: Use motion primitives and a kinematically aware heuristic (e.g., Dubins), or transition to hybrid A*.
- **Memory**: For long searches, consider IDA* (iterative deepening on `f`) or memory-bounded A* variants.

---

## 7) Lab: Extend A* for obstacle avoidance (clearance-aware path planning)

### Goal

Enhance a basic grid-based A* to prefer paths with higher clearance from obstacles by inflating obstacles and adding a clearance-aware traversal cost. Support both static and dynamic updates to the map.

### Starter repos for practical integration

- Minimal Python reference for search algorithms: `https://github.com/aimacode/aima-python`
- ROS 2 Navigation (Nav2) with costmaps and planners: `https://github.com/ros-planning/navigation2`
- Open Motion Planning Library (OMPL) for sampling-based planners: `https://github.com/ompl/ompl`
- Optional grid A* implementations for reference in other languages: `https://github.com/qiao/PathFinding.js`

If your course uses a dedicated starter, clone: `https://github.com/your-org/robotics-path-planning-starter` (replace with your actual course repo).

### Background and design

- Real robots should avoid “grazing” obstacles. A standard trick is to inflate occupancy around obstacles so that paths keep a safety buffer.
- Additionally, charge a penalty for being near obstacles to encourage routes through open space.

We will implement two mechanisms:

1) **Binary inflation for safety**: Expand each obstacle by `r_inflate` cells using a distance transform; any cell with distance < `r_inflate` is considered effectively occupied.
2) **Clearance-aware traversal cost**: For free cells with distance `d` to the nearest obstacle, define a traversal cost multiplier, e.g.,

   - Linear: `w(d) = 1 + α * max(0, (r_clear - d) / r_clear)`
   - Exponential: `w(d) = 1 + α * exp(-β d)`

   The step cost from `u` to `v` becomes `cost(u, v) = base_cost(u, v) * 0.5 * (w(d_u) + w(d_v))`.

Admissible heuristic safeguard: let `c_min` be the minimal possible step cost (e.g., when far from obstacles). Use `h(n) = c_min * h_geom(n)` with `h_geom` being Manhattan/Octile/Euclidean appropriate to your move set.

### Tasks

1) Load or generate an occupancy grid (0 = free, 1 = obstacle). Provide a simple map loader or random map generator.
2) Compute the distance transform (e.g., 2D Euclidean DT via brushfire, Felzenszwalb’s method, or OpenCV’s `distanceTransform`).
3) Inflate obstacles by `r_inflate` (in cells or meters mapped to cells). Mark inflated cells as blocked.
4) Define clearance weights `w(d)` with tunable parameters `α, β, r_clear`.
5) Implement A* with:
   - 4-connected and 8-connected neighbor options.
   - Edge cost `cost(u, v)` incorporating clearance-aware weights.
   - Heuristic `h(n) = c_min * h_geom(n)` (Manhattan for 4-connected, Octile for 8-connected).
   - Tie-breaking: prefer larger `g` on equal `f`.
6) Add live map updates (optional): allow toggling obstacles and recomputing the DT incrementally or from scratch.
7) Visualize:
   - Distance transform heatmap.
   - Inflated obstacles overlay.
   - Final path with arrows or line segments.
8) Evaluate:
   - Compare baseline A* (no clearance weighting) vs clearance-aware A* on: path length, minimum clearance along the path, number of node expansions, runtime.

### Pseudocode for clearance-aware cost

```pseudo
function edge_cost(u, v, grid, dist, params):
    base := 1           # or sqrt(2) for diagonals
    d_u := dist[u]
    d_v := dist[v]
    w_u := clearance_weight(d_u, params)
    w_v := clearance_weight(d_v, params)
    return base * 0.5 * (w_u + w_v)

function clearance_weight(d, params):
    α := params.alpha
    β := params.beta
    r_clear := params.r_clear
    if d >= r_clear:
        return 1
    # Linear option
    return 1 + α * max(0, (r_clear - d) / r_clear)
```

Heuristic choice: if the minimum possible edge cost is `base_min` (often 1 for cardinal steps), use `h(n) = base_min * h_geom(n)`.

### Deliverables

- Source code implementing the above extensions.
- A short report (1–2 pages) including:
  - Parameter values (`r_inflate`, `α`, `β`, `r_clear`), maps used.
  - Plots: heatmap of distance transform, path overlays.
  - Quantitative comparison table: baseline vs clearance-aware.
  - Discussion: trade-offs, observed behaviors, failure cases.

### Rubric (20 points)

- Correct A* implementation with OPEN/CLOSED and parent reconstruction (4)
- Distance transform and obstacle inflation applied correctly (4)
- Clearance-aware cost integrated correctly; heuristic remains admissible (4)
- Visualization and evaluation (4)
- Report clarity and analysis quality (4)

### Stretch goals

- Dynamic obstacles: recompute DT incrementally or use time-parameterized planning (space-time A* with `(x, y, t)` states).
- Hybrid A*: incorporate heading and nonholonomic motion primitives; use Dubins/Reeds-Shepp heuristic.
- Anytime variants: ARA* or Weighted A* with decreasing weights.

### Troubleshooting tips

- Expanding too many nodes: Verify heuristic scaling; ensure `h` is consistent with your move costs.
- Paths skim obstacles: Increase `α` or `r_clear`, or raise `r_inflate`.
- No paths found: Check occupancy thresholds; inflated obstacles might block narrow corridors.
- Oscillatory OPEN sizes: Review tie-breaking; prefer larger `g` on ties.

---

## 8) Sample problems and exercises

1) Manual A*
   - Given a 5×5 grid with 4-connected moves and uniform cost, obstacles at `(2,2), (2,3), (3,2)`, start `S=(1,1)`, goal `G=(5,5)`:
     - Compute one or two `h` values (Manhattan/EUCLIDEAN) at `(1,1), (2,1), (3,1)`.
     - Simulate a few A* steps: which node is expanded first? Show `f=g+h` values.

2) Heuristic admissibility
   - Prove Manhattan is admissible for 4-connected grids with unit costs. Is it still admissible if diagonal moves are allowed at unit cost? Explain.

3) Consistency check
   - For 8-connected grids with `c_card=1`, `c_diag=√2`, show Octile is consistent by verifying the triangle inequality for neighbor moves.

4) Weighted A* trade-offs
   - On an obstacle-free 100×100 grid, compare expansions for weights `w ∈ {1.0, 1.2, 1.5, 2.0}` using Euclidean heuristic. Report solution cost and node expansions.

5) Clearance tuning
   - Evaluate paths for `α ∈ {0.5, 1, 2}` and `r_clear ∈ {1, 2, 3}`. Plot minimum clearance along the path versus path length.

6) Re-openings test
   - Construct a grid and a heuristic that is admissible but not consistent. Show that handling node re-openings can improve the final solution, and quantify the overhead.

7) Kinematic heuristic design
   - For a Dubins car with minimum turning radius `R`, propose an admissible heuristic between two oriented poses. Discuss implementation options and computational cost.

---

## 9) Real-world applications

- **Mobile robots in warehouses**: A* on occupancy or navigation meshes for short-horizon pathing between racks; clearance-aware costs help avoid shelf collisions and human workers.
- **Autonomous driving (local planning)**: Hybrid A* in a costmap with kinematic constraints; heuristics derived from Reeds-Shepp distances.
- **Service robots in hospitals**: Dynamic cost inflation around moving staff; A* replanning on updated costmaps.
- **Video games**: Grid or navmesh A* with jump point search for speed; designer-authored costs for terrain difficulty.
- **Drones and UGVs**: 3D grids or layered 2.5D surfaces; heuristics based on Euclidean distance scaled by minimum per-step energy.

---

## 10) Additional implementation tips and patterns

- Use a `best_g` dictionary keyed by node state; only push to OPEN when `tentative_g < best_g.get(v, +∞)`.
- Store parents as compact integers for memory efficiency in large grids.
- Prefer immutable state keys (tuples) and pool objects to reduce GC overhead in Python.
- Profile early: time in `neighbors()` and collision/cost checks usually dominates.
- Unit-test on micro-maps: one with a straight corridor, one with a U-shaped obstacle, one with a 1-cell bottleneck.

---

## 11) Connections to LaValle (Ch. 4)

- Graph search foundations: Definitions of states, actions, costs, and search trees.
- Optimal search and admissible heuristics: Conditions for optimality in informed search.
- Metric and distance functions: Designing heuristics consistent with problem metrics.
- Extensions to motion planning: From grid graphs to C-space roadmaps; ensuring collision-free edges and appropriate heuristics.

Suggested reading flow

- Skim sections on uninformed search to recap Dijkstra.
- Focus on admissible/consistent heuristics and triangle inequality arguments.
- Review proofs of A* optimality, noting assumptions on cost positivity and branching.

---

## 12) Quick-reference formulas

- A*: `f(n) = g(n) + h(n)`
- Manhattan: `h = c_card * (|dx| + |dy|)`
- Euclidean: `h = sqrt(dx^2 + dy^2)`
- Chebyshev: `h = max(|dx|, |dy|)`
- Octile: `h = (dx + dy) + (√2 - 2) * min(dx, dy)` (when `c_card=1`, `c_diag=√2`)
- Weighted A*: `f = g + w*h` (suboptimal, faster)
- Consistency: `h(n) ≤ c(n, n') + h(n')` for all neighbors `n'`

---

## 13) Suggested schedules and checkpoints (Week 3)

- Lecture 1: A* fundamentals; implement baseline A* on a small map.
- Lecture 2: Heuristics and their properties; prove admissibility/consistency; compare to Dijkstra.
- Lab session: Add distance transform, obstacle inflation, and clearance-aware cost; run experiments.
- End of week: Submit code and short report; prepare for hybrid A* in Week 4.

---

## 14) Appendix: A* variants at a glance

- IDA*: Iterative deepening on `f`-bound; low memory, may re-expand many nodes.
- RBFS: Recursive best-first search; memory-bounded, uses backup `f` values.
- EES/EES* and suboptimal variants: Introduce focal lists and bounded suboptimality.
- JPS (Jump Point Search): Prunes symmetric neighbors on uniform-cost grids; large speedups in practice.

---

### References and useful repos

- LaValle, S. M., Planning Algorithms, Chapter 4.
- `https://github.com/aimacode/aima-python` — Reference search algorithms in Python
- `https://github.com/ros-planning/navigation2` — ROS 2 Navigation stack with costmaps and planners
- `https://github.com/ompl/ompl` — Open Motion Planning Library
- `https://github.com/qiao/PathFinding.js` — Grid-based pathfinding in JavaScript (educational reference)
 - `https://github.com/your-org/robotics-path-planning-starter` — Course starter with A* baseline and lab scaffolding

---

### Optional diagram: A* frontier evolution (conceptual)

```
f = g + h with h ≈ true-to-goal distance

S . . . . . . . .
  * * * . . . . .
  * # # * . . . .
  * # . * . . . .
  * # . * . . . .
  * * * * * * * G

Stars indicate nodes selected early by A*; the frontier arcs toward G.
```

---

## 15) Worked example: Step-by-step A* on a 5×5 grid

Setup

- Grid coordinates: `(x, y)` with `x` increasing to the right, `y` increasing downward
- Moves: 4-connected, unit cost per step
- Heuristic: Manhattan distance
- Obstacles: `#` at `(2,2), (2,3), (3,2)`
- Start `S=(1,1)`, Goal `G=(5,5)`

Map view

```
  x→  1 2 3 4 5
y    -----------
1 |  S . . . .
2 |  . # # . .
3 |  . # . . .
4 |  . . . . .
5 |  . . . . G
```

Heuristic values h(x,y) = |x-5| + |y-5|

- h(1,1)=8, h(2,1)=7, h(1,2)=7, h(3,1)=6, h(4,1)=5, ...

Initialization

- `g(S)=0`, `f(S)=g+h=8`. OPEN = {S: f=8}; CLOSED = ∅

Step 1: expand S=(1,1)

- Neighbors: (2,1), (1,2). Both free.
- g(2,1)=1, f=1+7=8; g(1,2)=1, f=1+7=8
- OPEN = {(2,1):8, (1,2):8}; CLOSED={S}

Step 2: tie-break on equal f; choose larger g (both equal) then smaller h. h(2,1)=7, h(1,2)=7; pick one, say (2,1)

- Expand (2,1). Neighbors: (3,1) free; (2,2) blocked; (1,1) in CLOSED; (2,0) invalid.
- g(3,1)=2, f=2+6=8
- OPEN={(1,2):8, (3,1):8}

Step 3: expand (1,2)

- Neighbors: (1,3) free; (2,2) blocked; (0,2) invalid; (1,1) CLOSED
- g(1,3)=2, f=2+6=8
- OPEN={(3,1):8, (1,3):8}

Step 4: expand (3,1)

- Neighbors: (4,1) free; (2,1) CLOSED; (3,0) invalid; (3,2) blocked (since (3,2) is obstacle)
- g(4,1)=3, f=3+5=8
- OPEN={(1,3):8, (4,1):8}

Step 5: expand (1,3)

- Neighbors: (1,4) free; (1,2) CLOSED; (0,3) invalid; (2,3) blocked
- g(1,4)=3, f=3+5=8
- OPEN={(4,1):8, (1,4):8}

Step 6: expand (4,1)

- Neighbors: (5,1) free; (4,2) free; (3,1) CLOSED; (4,0) invalid
- g(5,1)=4, f=4+4=8; g(4,2)=4, f=4+4=8
- OPEN={(1,4):8, (5,1):8, (4,2):8}

Step 7: expand (1,4)

- Neighbors: (1,5) free; (2,4) free; (1,3) CLOSED; (0,4) invalid
- g(1,5)=4, f=4+4=8; g(2,4)=4, f=4+4=8
- OPEN={(5,1):8, (4,2):8, (1,5):8, (2,4):8}

Step 8: expand (5,1)

- Neighbors: (5,2) free; (4,1) CLOSED
- g(5,2)=5, f=5+3=8
- OPEN={(4,2):8, (1,5):8, (2,4):8, (5,2):8}

Step 9: expand (4,2)

- Neighbors: (4,3) free; (3,2) blocked; (5,2) in OPEN with g=5; (4,1) CLOSED
- g(4,3)=5, f=5+3=8
- OPEN={(1,5):8, (2,4):8, (5,2):8, (4,3):8}

Step 10: expand (1,5)

- Neighbors: (2,5) free; (1,4) CLOSED
- g(2,5)=5, f=5+3=8
- OPEN={(2,4):8, (5,2):8, (4,3):8, (2,5):8}

Step 11: expand (2,4)

- Neighbors: (3,4) free; (2,5) in OPEN with g=5; (1,4) CLOSED; (2,3) blocked
- g(3,4)=5, f=5+3=8
- OPEN={(5,2):8, (4,3):8, (2,5):8, (3,4):8}

Step 12: expand (5,2)

- Neighbors: (5,3) free; (5,1) CLOSED; (4,2) CLOSED
- g(5,3)=6, f=6+2=8
- OPEN={(4,3):8, (2,5):8, (3,4):8, (5,3):8}

Step 13: expand (4,3)

- Neighbors: (4,4) free; (3,3) free; (5,3) in OPEN; (4,2) CLOSED
- g(4,4)=6, f=6+2=8; g(3,3)=6, f=6+4=10 (but remains in OPEN with worse f)
- OPEN={(2,5):8, (3,4):8, (5,3):8, (4,4):8, (3,3):10}

Step 14: expand (2,5)

- Neighbors: (3,5) free; (1,5) CLOSED; (2,4) CLOSED
- g(3,5)=6, f=6+2=8
- OPEN={(3,4):8, (5,3):8, (4,4):8, (3,3):10, (3,5):8}

Step 15: expand (3,4)

- Neighbors: (4,4) in OPEN with g=6 (no improvement); (2,4) CLOSED; (3,5) in OPEN with g=6; (3,3) in OPEN with g=6
- No updates.

Step 16: expand (5,3)

- Neighbors: (5,4) free; (4,3) CLOSED; (5,2) CLOSED
- g(5,4)=7, f=7+1=8
- OPEN={(4,4):8, (3,5):8, (3,3):10, (5,4):8}

Step 17: expand (4,4)

- Neighbors: (4,5) free; (5,4) in OPEN with g=7; (3,4) CLOSED; (4,3) CLOSED
- g(4,5)=7, f=7+1=8
- OPEN={(3,5):8, (5,4):8, (3,3):10, (4,5):8}

Step 18: expand (3,5)

- Neighbors: (4,5) in OPEN with g=7; (2,5) CLOSED
- No updates.

Step 19: expand (5,4)

- Neighbors: (5,5)=Goal; set g(5,5)=8, f=8+0=8
- Goal found. Reconstruct path by parents.

Resulting shortest path (one of many with cost 8):

`(1,1) → (2,1) → (3,1) → (4,1) → (5,1) → (5,2) → (5,3) → (5,4) → (5,5)`

Observation: With Manhattan `h`, A* maintained `f=8` in the OPEN for all expansions until reaching the goal, expanding in a wavefront biased toward the goal corner while detouring around obstacles.

---

## 16) Advanced heuristics and speed-ups

- Landmark-based A* (ALT): Precompute distances from selected landmarks to all nodes; use triangle inequalities to form tighter admissible lower bounds `h(n)=max_L |d(L, goal) - d(L, n)|`. Particularly effective on road networks.
- Multi-Heuristic A* (MHA*): Run multiple heuristics in parallel (one anchor admissible, others possibly inadmissible) using a scheduling policy; retains bounded suboptimality while exploiting diverse guidance.
- Potential functions: Run a backward Dijkstra from the goal to compute exact `h*(n)` on static uniform grids once; reuse as heuristic when dynamic changes are rare.
- Jump Point Search (JPS): In uniform-cost 8-connected grids, prune symmetric neighbors by “jumping” along straight lines to critical points; large reductions in expansions.
- Bidirectional A*: Run searches from start and goal meeting in the middle; requires careful admissible/consistent bidirectional heuristics.

Weighted/inadmissible heuristics in practice

- Small weights `w∈[1.0,1.2]` often provide substantial speedups with minimal suboptimality, useful for real-time systems.
- In dynamic environments, use anytime schemes (ARA*) to quickly provide a feasible path and refine when time permits.

---

## 17) Short-answer checks (for discussion/quiz)

1) Why does consistency imply no node re-openings in A*? State the inequality and its effect on `f`-values.
2) Give a concrete example where Euclidean `h` is admissible but not consistent under a non-Euclidean edge-cost model.
3) Describe a tie-breaking rule that reduces plateau wandering and explain why.
4) How does cost inflation around obstacles affect heuristic admissibility, and how do you correct for it?
5) When is Dijkstra preferable to A* in robotics applications?

---

## 18) Quick integration notes and repo links

- Course starter (fork or adapt): `https://github.com/your-org/robotics-path-planning-starter`
- Minimal Python baseline (search code to extend): `https://github.com/aimacode/aima-python`
- ROS 2 Nav2 for deployment on robots: `https://github.com/ros-planning/navigation2`
- OMPL for sampling-based comparisons: `https://github.com/ompl/ompl`
- JS visualizer for classroom demos: `https://github.com/qiao/PathFinding.js`

Suggested workflow

1) Clone the course starter and run the baseline A*.
2) Add distance transform + inflation and clearance-aware costs.
3) Run the evaluation script to compare baseline vs clearance-aware variants.
4) Optional: Export the path to ROS 2 and visualize in RViz.

---

## 19) Extra exercise: Consistency stress test

Design a heuristic that is admissible but not consistent on a grid with direction-dependent costs (e.g., uphill vs downhill). Implement A* with node re-openings and measure:

- Number of re-openings
- Total expansions
- Final solution cost
- Runtime overhead compared to a consistent heuristic

Provide a brief explanation of the observed trade-offs.

---

## 20) Recap and looking ahead

- You can now implement A*, argue about its optimality and completeness, and design admissible/consistent heuristics for varied robotics settings.
- Next week: Hybrid A* and kinodynamic planning, integrating vehicle orientation and curvature constraints directly into the search.

