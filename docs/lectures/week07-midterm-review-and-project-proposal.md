### Week 7 — Midterm Review and Project Proposal

**Focus**: Review of planning algorithms; Introduction to the course project (extending MPPI)

**Learning objectives**
- Consolidate understanding of classical and modern planning/control algorithms (graph search, sampling-based planning, trajectory optimization, MPC/MPPI)
- Prepare for the midterm with targeted theory and coding practice
- Launch the course project: define a feasible, high-impact plan extending an MPPI repository (e.g., custom environments, new cost terms, performance/robustness experiments)

---

## Part I. Planning Algorithms Review

### 1) Problem Formulation
- **State**: x ∈ ℝ^n, **Control/Action**: u ∈ ℝ^m, **Dynamics**: x_{t+1} = f(x_t, u_t) + w_t
- **Cost/Objective**: minimize J = Φ(x_T) + ∑_{t=0}^{T-1} ℓ(x_t, u_t)
- **Constraints**: state constraints (collision-free, joint limits), control limits (actuator bounds), dynamics feasibility
- **Representations**: configuration space vs. task space; occupancy maps, signed distance fields (SDF), kinematic chains

### 2) Graph Search (Discrete Planning)
- **BFS**: shortest path in edges for unweighted graphs
- **Dijkstra**: optimal on nonnegative edge costs
- **A***: f(n) = g(n) + h(n); admissible ⇒ optimal; consistent ⇒ no re-expansions

### 3) Sampling-Based Planning (Continuous Planning)
- **PRM**: multi-query; asymptotically probabilistically complete
- **RRT**: single-query; biased to unexplored space
- **RRT***: rewiring for asymptotic optimality; steering + radius schedule

### 4) Trajectory Optimization
- **CHOMP/TrajOpt**: optimize a trajectory parameterization with collision costs and smoothness terms
- **iLQR/DDP**: second-order methods around a nominal trajectory; linearize dynamics, quadratize cost; backward pass for local control law
- Strengths: smooth, dynamically feasible trajectories; use gradients and constraints explicitly
- Limitations: local minima; sensitive to initialization; requires differentiable models or approximations

### 5) Model Predictive Control (MPC) and MPPI
- **MPC**: repeatedly solve a finite-horizon optimal control problem; apply first control; shift horizon
- **MPPI (Model Predictive Path Integral Control)**: sample-based stochastic optimal control method
  - Roll out K noisy control sequences U + ε; evaluate trajectory costs J_i
  - Compute weights w_i ∝ exp(−J_i / λ), where λ is a temperature parameter
  - Update control sequence via weighted average of noise: u_t ← u_t + ∑_i w_i ε_{i,t}
  - Advantages: derivative-free, parallelizable; handles nonconvex cost landscapes
  - Tuning: temperature λ, noise covariance Σ_ε, horizon H, samples K, control limits, cost shaping

### 6) Constraints, Collisions, and Safety
- Hierarchy of constraints: hard constraints (infeasible states), soft constraints via penalties, barrier/augmented Lagrangian methods
- Collision checking: exact geometry vs. approximations (spheres/boxes), SDF queries; continuous vs. discrete collision checking
- Safety margins: inflate obstacles by robot radius; robustification with uncertainty buffers

### 7) Uncertainty and Belief-Space Planning (glance)
- Process and observation noise; belief-state b_t over x_t
- Planning under uncertainty: POMDPs, belief-space planning, risk measures (CVaR, chance constraints)
- Practical approach: robust costs, penalties on variance, receding horizon with frequent re-planning

### 8) Complexity and Trade-offs
- Graph search: exponential in branching factor and depth; A* guided by heuristic
- Sampling-based: sample count dominates; nearest neighbor search O(log N) with spatial trees
- Trajectory optimization: per-iteration linear/quadratic solves; real-time feasibility via warm starts
- MPPI: O(K·H) rollouts; strong GPU/parallel scaling

### 9) Quick Formulas and Definitions
- A* f(n) = g(n) + h(n); admissible ⇒ optimal; consistent ⇒ no re-expansions (graph)
- RRT* rewiring radius r_n = γ( log n / n )^(1/d) for d-dimensional space (theoretical form)
- MPPI weights: w_i = exp(−J_i / λ) / ∑_j exp(−J_j / λ); control update via weighted noise
- iLQR update: compute local linear dynamics x_{t+1} ≈ A_t x_t + B_t u_t + c_t; backward pass yields K_t, k_t; forward rollout to update trajectory

---

## Part II. Sample Midterm Questions

### A. Theory (short answers and proofs)
1) Define admissible and consistent heuristics. Prove that A* with a consistent heuristic never needs to decrease a node’s g-value in a graph search setting.

2) Explain probabilistic completeness for PRM/RRT. What does “asymptotic optimality” mean in RRT* and what additional mechanism enables it?

3) Contrast iLQR and MPPI in terms of assumptions, gradient use, and typical failure modes. When might MPPI outperform iLQR, and vice versa?

4) You have a nonholonomic robot (differential drive) navigating narrow passages. Compare the suitability of: (a) grid A* with motion primitives, (b) RRT*, (c) TrajOpt, (d) MPPI. Which would you try first and why?

5) Consider MPPI with temperature λ and noise covariance Σ_ε. Describe qualitatively how increasing λ or scaling Σ_ε by α > 1 affects exploration, convergence, and solution quality.

6) Define and compare chance constraints vs. CVaR-based costs in uncertain planning. Give one advantage of each.

7) In CHOMP/TrajOpt, how are collision costs often constructed using SDFs? Why does smoothing help even when collision penalties exist?

8) Provide the time and space complexity of A* in terms of branching factor b, solution depth d, and the accuracy of the heuristic.

9) Give one example of an admissible heuristic for 2D grid navigation with 4-connected moves and unit costs. Is Manhattan distance consistent in this setting? Why?

10) Explain warm-starting in MPC/MPPI and describe two benefits.

### B. Coding (write or read code)
1) Implement A* for a weighted grid. Return the path or report failure. Provide the core loop and data structures.

```python
import heapq

def astar(start, goal, neighbors_fn, cost_fn, heuristic_fn):
    open_heap = []  # (f, g, node, parent)
    heapq.heappush(open_heap, (heuristic_fn(start, goal), 0.0, start, None))
    came_from = {}
    best_g = {start: 0.0}

    while open_heap:
        f, g, node, parent = heapq.heappop(open_heap)
        if node in came_from:
            continue  # already expanded with a better g
        came_from[node] = parent
        if node == goal:
            # reconstruct path
            path = [node]
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        for nbr in neighbors_fn(node):
            step = cost_fn(node, nbr)
            tentative_g = g + step
            if tentative_g < best_g.get(nbr, float('inf')):
                best_g[nbr] = tentative_g
                h = heuristic_fn(nbr, goal)
                heapq.heappush(open_heap, (tentative_g + h, tentative_g, nbr, node))

    return None  # no path
```
 
2) MPPI weight update and control refinement for a single time step (pseudo-Python):

```python
import numpy as np

def mppi_update(u_t, noises_t, costs, temperature_lambda):
    # noises_t: shape (K, m); costs: shape (K,)
    weights_unnorm = np.exp(-(costs - costs.min()) / max(1e-8, temperature_lambda))
    weights = weights_unnorm / (weights_unnorm.sum() + 1e-12)
    delta_u = (weights[:, None] * noises_t).sum(axis=0)
    return u_t + delta_u
```

3) Given an SDF function d(x) that returns signed distance (>0 free, <0 in collision), write a cost term for a point robot that penalizes penetration with a quadratic barrier up to margin μ.

```python
def sdf_collision_cost(x, sdf_fn, margin, weight):
    d = sdf_fn(x)
    if d >= margin:
        return 0.0
    # penalize when within margin; stronger inside obstacles
    return weight * (margin - d) ** 2
```

---

## Part III. Course Project: Extending MPPI

### 1) Goals and Deliverables
- **Goal**: Extend an existing MPPI repository to demonstrate improved capability, robustness, or applicability.
- **Deliverables**:
  - Working code with documentation and reproducible environment
  - Experiments with meaningful baselines and metrics
  - Short paper-style report (4–6 pages) and a 5–8 minute demo/presentation

### 2) Suggested Project Ideas (extend MPPI)
- **Custom environments**
  - Differential-drive robot in clutter with narrow passages (2D SDF, dynamic obstacles)
  - Car-like robot with nonholonomic constraints and tire limits (Kinematic/dynamic bicycle models)
  - Quadrotor navigation with wind disturbances (stochastic dynamics, risk-sensitive MPPI)
  - Planar manipulator (3–6 DoF) moving in a workspace with obstacles (SDF-based costs)
  - Multi-goal tasks with soft deadlines; time-varying goals; moving targets

- **Cost shaping and constraints**
  - Barrier functions for hard constraints; augmented Lagrangian penalties for soft constraints
  - Velocity/acceleration/jerk regularization; path smoothness; control effort
  - Risk-sensitive objectives: CVaR, chance constraints; uncertainty buffers

- **Algorithmic variations**
  - Temperature and noise scheduling; adaptive covariance; covariance learning
  - Hybrid control: combine MPPI with a tracking controller or safety shield (CBF/backup policy)
  - Multi-agent coordination with coupled costs (collision avoidance, formation keeping)
  - Warm-starting with previous optimal sequence; trajectory library seeding
  - GPU acceleration of rollouts; vectorized physics; JAX/NumPy/CUDA backends

- **Comparative studies**
  - MPPI vs. iLQR/DDP vs. CEM (Cross-Entropy Method) vs. PPO/SAC (for learned policies)
  - Robustness under disturbances and model mismatch; computation vs. performance trade-offs
  - Ablations: effect of λ, sample count K, horizon H, covariance Σ_ε, cost weights

- **Tooling and visualization**
  - Logging of rollouts, cost terms, and control updates for post-hoc analysis
  - Interactive visualizations; trajectory replays; SDF heatmaps; real-time plots

### 3) Proposal Template (submit by end of Week 7)
- **Project title**
- **Team members** (names, roles)
- **Problem statement**: what problem/environment are you tackling and why does MPPI suit it?
- **Novelty/extension**: what is new (environment, cost, constraint handling, robustness, performance)?
- **Hypotheses**: 2–3 testable claims (e.g., “Risk-sensitive MPPI reduces collision rate by ≥30% in gusty wind vs. vanilla MPPI at similar compute.”)
- **Methods**: MPPI configuration, dynamics models, cost terms, constraints, implementation details
- **Baselines**: which methods/settings will you compare against and why
- **Datasets/simulators**: what environments, maps, or dynamics will you use; how generated
- **Metrics**: task success, path length, smoothness, energy, collision rate, runtime, regret
- **Experimental plan**: experiments, ablations, hyperparameter sweeps, expected plots
- **Risks and mitigations**: likely blockers; fallback plan; reduced-scope variant
- **Compute/resources**: GPU/CPU needs; timeline; repo structure; division of labor

### 4) Grading Rubric (guideline)
- **Functionality (25%)**: task completion, correctness, constraint satisfaction
- **Experimental depth (25%)**: quality of baselines, metrics, ablations, analysis
- **Engineering quality (20%)**: code clarity, documentation, modularity, reproducibility
- **Innovation (15%)**: novelty of environment/approach and insightfulness
- **Presentation & report (15%)**: clarity, storytelling, visuals, live demo quality

### 5) Milestones and Timeline
- Week 7: Proposal approved; environment scaffolded; baseline MPPI runs
- Week 8: Cost shaping and constraint handling implemented; first experiments
- Week 9: Robustness/ablation studies; performance optimization; mid-project check-in
- Week 10: Final experiments; polish code; prepare report and demo

### 6) Practical Tips for MPPI Projects
- Start simple: verify rollouts, cost accumulation, and control limits on a toy map
- Normalize and clip cost terms; prevent a single term from dominating
- Temperature λ: too small ⇒ over-exploitation/instability; too large ⇒ noisy controls
- Covariance Σ_ε: align with control space scale; consider anisotropic noise per actuator
- Use warm-start from previous optimal controls; low-pass filter updated sequences
- Keep a deterministic evaluation mode for fair comparisons
- Profile bottlenecks (collision checks, dynamics); parallelize and vectorize
- Log everything: costs per term, entropy of weights, rollout stats, wall time

---

## Part IV. Group Brainstorming and Teaming

### 1) Diverge then converge
- Timebox 20–30 minutes to list as many ideas as possible; no judgment
- Cluster ideas by theme (environment type, robustness, performance, tooling)
- Vote on top 2–3, then conduct quick feasibility checks (data, compute, risks)

### 2) Scope control
- Right-size the first deliverable; define a “minimum lovable product” (MLP)
- Prefer one ambitious axis (e.g., novel environment) plus two modest ones (e.g., visualization and ablation)
- Define explicit out-of-scope items to prevent drift

### 3) Roles and responsibilities
- Product/PM: keeps scope, milestones, and risks visible
- Lead engineer: repo structure, performance, CI/reproducibility
- Research lead: experiment design, metrics, baselines
- Everyone: documentation and weekly demos

### 4) Assumptions and risk mapping
- Write down modeling assumptions (dynamics fidelity, sensor noise)
- Maintain a risk table: risk, likelihood, impact, mitigation
- Establish a rollback plan (reduced-scope variant) early

### 5) Communication hygiene
- Weekly stand-ups + mid-week async updates; short meeting notes
- One place for decisions (README/CHANGELOG); one place for tasks (issues/board)
- Treat the report as a living document; write while you build

---

## Part V. Logistics and Submission

### 1) What to submit for the proposal
- A 1–2 page PDF following the template
- Link to a public/private repo with a minimal scaffold (environment stub, MPPI config)
- A short slide (≤5) deck for your 2–3 minute in-class pitch

### 2) Evaluation reminders
- Use fixed random seeds for comparisons; report compute budget
- Provide plots with confidence intervals or box plots
- Share failure cases; analyze when/why methods break

---

End of Week 7 notes.

