### Week 8 — Model Predictive Control (MPC) Basics

- **Course**: Robotics and Control Systems
- **Week**: 8
- **Topic**: Model Predictive Control (MPC) Basics
- **Readings**: Camacho & Bordons, Chapters 3–4
- **Lab**: Simple MPC for linear systems (discrete-time LTI)

### Learning objectives
By the end of this week you should be able to:
- **Define** the standard finite-horizon MPC problem and its decision variables.
- **Explain** the receding horizon principle and why MPC solves a sequence of open-loop problems in closed-loop fashion.
- **Formulate** quadratic programs (QPs) for linear MPC with state and input constraints.
- **Assemble** prediction and constraint matrices and map an MPC specification to a numerical solver.
- **Tune** horizon length and penalty weights to trade off performance and computational load.
- **Implement** a basic linear MPC controller and test it on a simple LTI system.
- **Contrast** deterministic MPC with sampling-based methods such as MPPI (preview for next week).

### 1) Motivation: why MPC?
Classical linear controllers (PID, LQR) achieve good performance for many tasks, but they lack an explicit way to handle hard constraints (e.g., actuator limits, safety envelopes) and multi-step lookahead. MPC addresses both:
- **Multi-step prediction**: it uses a model to forecast future states over a finite horizon.
- **Optimization with constraints**: it chooses a sequence of inputs that minimizes a cost while respecting state and input limits.
- **Receding horizon feedback**: only the first control action is applied, then the optimization is repeated with updated state information. This yields feedback robustness to disturbances and modeling errors.

MPC has become a standard tool in process control, automotive (adaptive cruise, lane keeping), robotics (trajectory tracking), energy, and aerospace. The appeal is its explicit handling of constraints and its anticipatory behavior, provided computation fits within the sample time.

### 2) Discrete-time LTI model and notation
We consider a discrete-time linear time-invariant (LTI) system with sampling period \(T_s\):
\[ x_{k+1} = A x_k + B u_k, \qquad y_k = C x_k + D u_k. \]
- \(x_k \in \mathbb{R}^{n_x}\): state at step \(k\)
- \(u_k \in \mathbb{R}^{n_u}\): input
- \(y_k \in \mathbb{R}^{n_y}\): output

Typical constraints:
\[ x_k \in \mathcal{X} = \{ x : x_{\min} \le x \le x_{\max} \}, \quad u_k \in \mathcal{U} = \{ u : u_{\min} \le u \le u_{\max} \}. \]
Optionally, a rate constraint on input increments \(\Delta u_k = u_k - u_{k-1}\) can be added to reduce chattering or protect actuators.

We denote the prediction horizon by \(N\). The decision variable is the stacked input sequence:
\[ \mathbf{u} = \begin{bmatrix} u_k^\top & u_{k+1}^\top & \cdots & u_{k+N-1}^\top \end{bmatrix}^\top. \]
Sometimes we also include the states as decision variables; here we eliminate states via prediction to obtain a compact QP in inputs only.

### 3) MPC objective and quadratic cost
A standard tracking MPC penalizes output error relative to a reference \(r\) and input effort and/or movement. With \(Q \succeq 0\), \(R \succ 0\), and optional terminal weight \(P \succeq 0\):
\[ J(\mathbf{u}; x_k) = \sum_{i=1}^{N} \left\| y_{k+i} - r_{k+i} \right\|_{Q}^2 + \sum_{i=0}^{N-1} \left\| u_{k+i} - u_\text{ref} \right\|_{R}^2 + \left\| x_{k+N} - x_\text{ref} \right\|_{P}^2. \]
Frequently we set \(u_\text{ref} = 0\) and track a constant reference \(r\). Many applications also penalize input increments:
\[ J_\Delta = \sum_{i=0}^{N-1} \left\| \Delta u_{k+i} \right\|_{S}^2, \quad S \succeq 0. \]
All these choices preserve a quadratic objective when the prediction model is linear, which leads to a quadratic program (QP). Quadratic costs are attractive because they are convex and lead to efficient solvers with strong convergence guarantees.

A practical tip is to scale states and inputs so that cost terms are of similar magnitude; otherwise, one term may numerically dominate, slowing or destabilizing the solver.

### 4) Prediction model over the horizon
We can express the predicted states and outputs in terms of \(x_k\) and the stacked inputs \(\mathbf{u}\).
Define the lifted dynamics (for simplicity, assume \(D=0\)):
\[
\mathbf{x} = \begin{bmatrix} x_{k+1} \\ x_{k+2} \\ \vdots \\ x_{k+N} \end{bmatrix}
= \underbrace{\begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}}_{\mathcal{A}} x_k + \underbrace{\begin{bmatrix}
B & 0 & \cdots & 0 \\
A B & B & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A^{N-1} B & A^{N-2} B & \cdots & B
\end{bmatrix}}_{\mathcal{B}} \mathbf{u}.
\]
Similarly for outputs, \(\mathbf{y} = \mathcal{Y} x_k + \mathcal{U} \mathbf{u}\), where \(\mathcal{Y}\) and \(\mathcal{U}\) collect lifted output dynamics.

Then the cost can be written in the standard QP form:
\[ \min_{\mathbf{u}} \; \tfrac{1}{2} \mathbf{u}^\top H \, \mathbf{u} + f(x_k)^\top \, \mathbf{u} + \text{const}, \]
with
\[ H = 2 (\mathcal{U}^\top \bar{Q} \, \mathcal{U} + \bar{R} + \bar{S}), \qquad f(x_k) = 2 \, \mathcal{U}^\top \bar{Q} (\mathcal{Y} x_k - \mathbf{r}) + \text{terms from }\bar{S}, \]
where \(\bar{Q}, \bar{R}, \bar{S}\) are block-diagonal matrices stacking the stage weights across the horizon, and \(\mathbf{r}\) is the stacked reference. Terminal cost can be folded into \(H, f\) via the last predicted state.

### 5) Constraints over the horizon
State and input constraints lift to linear inequalities in \(\mathbf{u}\):
\[ x_{\min} \le \mathbf{x} \le x_{\max} \;\; \Rightarrow \;\; G_x \, \mathbf{u} \le h_x + E_x x_k, \]
\[ u_{\min} \le \mathbf{u} \le u_{\max} \;\; \Rightarrow \;\; G_u \, \mathbf{u} \le h_u. \]
Input rate constraints can be encoded using a difference operator matrix \(D_\Delta\) so that \(\Delta \mathbf{u} = D_\Delta \mathbf{u} + d_\Delta u_{k-1}\), producing linear constraints. A terminal constraint \(x_{k+N} \in \mathcal{X}_f\) yields another linear inequality if \(\mathcal{X}_f\) is a polytope; choosing \(\mathcal{X}_f\) as a control invariant set aids stability and recursive feasibility.

In compact form the QP becomes:
\[
\begin{aligned}
\min_{\mathbf{u}} \;& \tfrac{1}{2} \mathbf{u}^\top H \mathbf{u} + f(x_k)^\top \mathbf{u} \\
\text{s.t. }\;& G \, \mathbf{u} \le h + E \, x_k, \\
& A_{eq} \, \mathbf{u} = b_{eq} + E_{eq} x_k \quad (\text{optional equalities}).
\end{aligned}
\]
This is a convex QP whenever weights are positive semidefinite and constraints are linear.

### 6) The receding horizon principle
MPC is not a one-shot open-loop plan. At each sampling instant:
1. **Measure/estimate** the current state \(x_k\) (use a state estimator if needed).
2. **Solve** the finite-horizon constrained optimization to obtain \(\mathbf{u}^* = [u_k^{*\top} \cdots u_{k+N-1}^{*\top}]^\top\).
3. **Apply only the first input**: \(u_k = u_k^*\).
4. **Advance the system** to \(k+1\), update the state estimate, shift the horizon, and repeat.

This procedure “recedes” the horizon forward and closes the loop, improving robustness to disturbances and modeling errors. In practice, warm-start the solver with the shifted solution from the previous step to reduce computation.

Algorithm sketch (at each k):
- Input: \(\hat{x}_k\), previous solution \(\mathbf{u}^*_{k-1}\) (for warm start)
- Build/update \(f(x_k)\) and constraint offsets (\(E x_k\))
- Solve QP for \(\mathbf{u}_k^*\)
- Apply \(u_k = u_{k,0}^*\)
- Warm-start candidate for next step: shift \(\mathbf{u}_k^*\) by one step

### 7) Quadratic programming (QP) for MPC
MPC for LTI systems with quadratic costs and linear constraints reduces to a QP:
- **Objective**: quadratic and convex in \(\mathbf{u}\).
- **Constraints**: linear inequalities/equalities.
- **Solvers**: active-set, interior-point, and operator-splitting (e.g., OSQP via ADMM) are commonly used in real-time MPC.

Key solver considerations:
- **Warm start**: shift the previous solution and append the last input to initialize; this dramatically reduces iterations.
- **Real-time feasibility**: choose horizon \(N\) and model dimension so that solution time \(< T_s\).
- **Scaling**: normalize states/inputs so weights produce comparable magnitudes. Poor scaling slows convergence.
- **Soft constraints**: when constraints may be violated due to model mismatch, add slack variables \(\epsilon \ge 0\) with large linear/quadratic penalties to preserve feasibility while discouraging violations.
- **Sparsity**: exploit banded/block structure to reduce memory and time.

Interior-point methods offer robust convergence but require factorization per iteration; operator-splitting (OSQP) pre-factorizes a KKT-like system and can be very fast for fixed-structure problems.

### 8) Stability and feasibility (overview)
Guaranteeing closed-loop stability with MPC typically uses:
- **Terminal cost** \(\|x_{k+N} - x_\text{ref}\|_P^2\) with \(P\) as the LQR solution of the unconstrained infinite-horizon problem; this approximates the tail cost beyond the horizon.
- **Terminal constraint** \(x_{k+N} \in \mathcal{X}_f\) ensuring that beyond the horizon a local stabilizer keeps the state within constraints.
- **Recursive feasibility** through invariant terminal sets and appropriate constraint tightening (robust MPC).

In this introductory week we prioritize implementation and intuition. Formal proofs and robust MPC are covered in advanced topics.

### 9) Worked example: double integrator with constraints
Consider a 1D point mass with position and velocity \(x = [p, v]^\top\), input acceleration \(u\), sampled at \(T_s\):
\[ A = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} \tfrac{1}{2} T_s^2 \\ T_s \end{bmatrix}. \]
Goal: drive \(p\) to reference \(p_{ref}\) with constraints \(|u| \le u_{max}\), \(|v| \le v_{max}\), and optionally \(|\Delta u| \le d_{max}\).

A typical cost over horizon \(N\):
\[ J = \sum_{i=1}^{N} (p_{k+i}-p_{ref})^2 q_p + v_{k+i}^2 q_v + \sum_{i=0}^{N-1} u_{k+i}^2 r + \sum_{i=0}^{N-1} (u_{k+i}-u_{k+i-1})^2 s. \]
Construct \(\mathcal{A}, \mathcal{B}\), stack constraints, and solve the QP. The controller anticipates future motion and smoothly regulates with explicit respect to limits, avoiding excessive overshoot.

Observations:
- Increasing \(q_p\) tightens position tracking but can cause more aggressive inputs.
- Larger \(r\) reduces actuation amplitude; a small \(s\) smooths inputs and mitigates chattering.
- If \(v_{max}\) is tight, the optimizer will trade tracking speed for safety, leading to longer settling times.

### 10) Implementation pattern: assembling the QP
At each step \(k\):
- **Inputs**: current state estimate \(\hat{x}_k\), reference trajectory \(\mathbf{r}\), previous input \(u_{k-1}\) if \(\Delta u\) is penalized/limited.
- **Build prediction matrices**: \(\mathcal{A}, \mathcal{B}, \mathcal{Y}, \mathcal{U}\) for horizon \(N\).
- **Build cost**: compute \(H\) and \(f\) from \(Q, R, S, P\), including terminal terms.
- **Build constraints**: state/input bounds, rate limits, terminal set. Assemble \(G, h, E\).
- **Solve QP**: with warm start from last iteration.
- **Apply**: first control input, store solution for next warm start.

Numerical tips:
- Precompute and cache structure for fixed \(A, B, C\) and \(N\); update only parts that depend on \(x_k\) and \(\mathbf{r}\).
- Use sparse matrices. MPC matrices are highly banded and block-structured.
- If solving time is tight, reduce \(N\), simplify constraints, or adopt a faster solver (e.g., OSQP with modest accuracy) and warm-start.
- Consider move blocking (tie inputs in groups) to reduce decision variables for long horizons.

### 11) Lab: Simple MPC for linear systems
You will implement and test a constrained MPC on a discrete-time LTI system (choose the double integrator or a 2D temperature model). Use Python with `cvxpy` (which can call OSQP) or `qpsolvers`. MATLAB users can follow analogous steps.

- **System**: double integrator with \(T_s = 0.1\) s.
- **Constraints**: \(|u| \le 1.0\), \(|v| \le 2.0\).
- **Horizon**: \(N = 15\).
- **Weights**: \(q_p = 10, q_v = 1, r = 0.1, s = 0.01\).
- **Task**: bring position from \(p_0 = 0\) to \(p_{ref}=1\) with minimal overshoot and smooth control.

Suggested steps:
1. Discretize the model (or use the given \(A, B\)).
2. Write a function to construct \(\mathcal{A}, \mathcal{B}\) for a given \(N\).
3. Build block-diagonal weight matrices and the lifted output mapping for tracking \(p\) only (optionally include velocity tracking with a small weight).
4. Assemble \(H, f\) and linear constraints in the standard QP form.
5. Implement the receding horizon loop with warm starts and state updates.
6. Plot \(p(t), v(t), u(t)\). Examine constraint activity and sensitivity to \(N\), \(q_p\), \(r\), and \(s\).

Reference CVXPY skeleton (nu = 1 for simplicity):
```python
import numpy as np
import cvxpy as cp

Ts = 0.1
A = np.array([[1.0, Ts], [0.0, 1.0]])
B = np.array([[0.5*Ts**2], [Ts]])

# horizon and sizes
N = 15
nx, nu = A.shape[0], B.shape[1]

# weights
q_p, q_v, r, s = 10.0, 1.0, 0.1, 0.01
Qy = np.diag([q_p, q_v])
R = r * np.eye(nu)
S = s * np.eye(nu)

# constraints
u_max = 1.0
v_max = 2.0

# variables for one solve (stack u_k..u_{k+N-1})
U = cp.Variable((nu*N, 1))

# helper to build prediction matrices
def build_lifted(A, B, N):
    nx, nu = A.shape[0], B.shape[1]
    A_bar = np.zeros((nx*N, nx))
    B_bar = np.zeros((nx*N, nu*N))
    A_pow = np.eye(nx)
    for i in range(1, N+1):
        A_pow = A @ A_pow
        A_bar[(i-1)*nx:i*nx, :] = A_pow
        for j in range(i):
            A_j = np.linalg.matrix_power(A, i-1-j)
            B_bar[(i-1)*nx:i*nx, j*nu:(j+1)*nu] = A_j @ B
    return A_bar, B_bar

A_bar, B_bar = build_lifted(A, B, N)

# prebuild block-diagonal weight matrices for outputs (track p,v)
Q_bar = np.kron(np.eye(N), Qy)
R_bar = np.kron(np.eye(N), R)

# reference over horizon (track position=1, velocity=0)
r = np.zeros((2*N, 1))
r[0::2, 0] = 1.0

# parameters that change each step
xk = cp.Parameter((nx, 1))

# predicted stacked states and outputs: x = A_bar xk + B_bar U
y = A_bar @ xk + B_bar @ U

# build input-difference operator for nu=1 to penalize Δu
if N > 1:
    D = np.eye(N) - np.roll(np.eye(N), 1, axis=0)
    D[0, :] = 0.0
    D_big = np.kron(D, np.eye(nu))  # (N*nu) x (N*nu)
    dU = D_big @ U
else:
    dU = 0.0

obj = cp.quad_form(y - r, Q_bar) + cp.quad_form(U, R_bar)
if N > 1:
    obj += s * cp.sum_squares(dU)

# inequalities: |u| <= nu_max, |v| <= v_max
constraints = []
for i in range(N):
    ui = U[i*nu:(i+1)*nu]
    constraints += [cp.abs(ui) <= nu_max]
for i in range(N):
    xi = y[i*nx:(i+1)*nx]
    constraints += [cp.abs(xi[1]) <= v_max]

prob = cp.Problem(cp.Minimize(obj), constraints)

# simulate
T = 60
x = np.zeros((nx, T+1))
U_hist = np.zeros((nu, T))

for k in range(T):
    xk.value = x[:, [k]]
    prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
    u0 = U.value[0:nu, 0]
    U_hist[:, k] = u0
    x[:, k+1] = (A @ x[:, k:k+1] + B @ u0.reshape(-1,1)).ravel()
```
Notes:
- The snippet is intentionally minimal; in practice, vectorize constraints for speed and exploit sparsity.
- The difference operator `D_big` penalizes input movement; replace with linear inequalities to enforce hard rate limits.
- Replace absolute-value constraints with pairwise linear inequalities if using a solver that requires pure QP form (no convex cones).

Deliverables:
- Plots of \(p, v, u\) versus time for several \(N\) and \(r, s\) values.
- A short note on solver times and the effect of warm starting.

### 12) Tuning guidelines
- **Horizon length \(N\)**: longer horizons improve foresight but cost more computation. Start with \(10\text{–}20\) for slow systems; reduce if solver is too slow. For fast systems, a shorter horizon may suffice when augmented with a terminal cost.
- **Weights**: increase \(Q\) to reduce tracking error; increase \(R\) to reduce actuation; use a small \(S\) (or movement penalty) to smooth inputs. Ensure units and magnitudes are scaled to similar orders of magnitude.
- **Constraint softness**: introduce slack if constraints are occasionally infeasible; penalize with large but finite weights. Prioritize safety-critical constraints as hard when possible.
- **Sampling time \(T_s\)**: faster sampling improves disturbance rejection but tightens timing constraints on the solver; if you reduce \(T_s\), consider reducing \(N\) to keep compute constant.
- **Warm starting and precomputation**: cache factorization or KKT structure where supported; reuse across steps.

### 13) From MPC to MPPI (preview)
Next week we study Model Predictive Path Integral control (MPPI), a sampling-based, derivative-free method.
- **MPC vs MPPI**:
  - MPC: solves a deterministic QP/NLP using a model and gradients; best for well-modeled, low-dimensional, convex or mildly nonconvex problems with hard constraints.
  - MPPI: generates many control perturbation rollouts through the dynamics and performs importance-weighted averaging; robust to model nonlinearity and discontinuities; naturally parallel on GPUs; handles nonconvexities.
- **Bridging concepts**:
  - Both optimize over a finite horizon and apply only the first action (receding horizon).
  - Both use cost shaping and can incorporate constraints (MPPI often via costs or sampling rejection rather than hard polytopic constraints).
  - Warm starts and trajectory shifting apply to both (seed rollouts with the shifted prior mean sequence in MPPI).
- **Preparation**: understand horizon shifting, cost design, and how rollout predictions inform control decisions. Practice visualizing predicted trajectories; this intuition transfers directly to MPPI’s sampled rollouts.

### 14) Common pitfalls and troubleshooting
- **State estimation**: MPC needs accurate \(x_k\). Use observers/filters; laggy estimates cause constraint mispredictions.
- **Model mismatch**: if the true system differs from \(A, B\), consider robust constraints, softening, disturbance modeling, or online system identification.
- **Poor scaling**: wildly different magnitudes in states/inputs lead to ill-conditioned QPs. Normalize variables and tune weights accordingly.
- **Solver timeouts**: reduce \(N\), simplify constraints, pre-factorize matrices, or choose a faster solver; check warm-start logic.
- **Constraint chattering**: add \(\Delta u\) penalties and/or reduce \(Q\) to avoid bang-bang behavior at limits; consider larger \(R\).
- **Reference steps**: for large setpoint changes, ramp references or use preview to avoid saturation bursts.

### 15) Reading guide for Camacho & Bordons, Ch. 3–4
- **Chapter 3**: Core predictive control concepts, dynamic matrix control (DMC), receding horizon implementation details, tuning philosophies (move suppression, horizon selection).
- **Chapter 4**: Constraint handling, set-point tracking structures (e.g., GPC formulation), terminal ingredients, and practical considerations for industrial MPC.
Focus on: how predictions are formed (dynamic matrices), the role of move blocking, the structure of the optimization, and treatment of constraints (hard vs soft).

### 16) Glossary
- **Receding horizon**: apply the first action of an optimal sequence, then re-solve at the next step.
- **Prediction horizon (N)**: number of future steps over which the cost and constraints are enforced.
- **Quadratic program (QP)**: convex optimization with quadratic objective and linear constraints.
- **Terminal cost/constraint**: ingredients ensuring stability and recursive feasibility.
- **Warm start**: initialize solver with a shifted prior solution.
- **Move blocking**: tie groups of future inputs to be equal to reduce decision variables.

### 17) Checklist
- Can you write the lifted prediction model \(\mathbf{x} = \mathcal{A} x_k + \mathcal{B} \mathbf{u}\)?
- Can you map weights and references to \(H\) and \(f\)?
- Can you encode input/state/rate bounds as \(G \mathbf{u} \le h + E x_k\)?
- Did you implement receding horizon with warm starts and produce time-series plots?
- Did you evaluate solver performance vs. \(N\) and weighting choices?

### 18) Further exploration
- Add a terminal LQR cost and verify improved stability/settling.
- Try move blocking (tie together groups of inputs to reduce decision variables).
- Test robustness: inject disturbances/model mismatch and observe behavior.
- Compare OSQP vs an interior-point solver in speed/accuracy and solution quality.
- Extend to output tracking with constraints on both states and outputs.

---

This concludes the basics of MPC: formulation, receding horizon control, and QP implementation. The lab will ground these ideas before we transition to sampling-based MPPI techniques next week.