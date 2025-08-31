### 第8週 — 模型預測控制（MPC）基礎

- **課程**: 機器人學與控制系統
- **週次**: 8
- **主題**: 模型預測控制（MPC）基礎
- **指定閱讀**: Camacho 與 Bordons，《Model Predictive Control》 第3–4章
- **實驗**: 線性系統的簡單 MPC（離散時間 LTI）

### 學習目標
本週結束時，你應能夠：
- **定義** 有限視窗 MPC 的標準形式與決策變數。
- **解釋** 滾動時域（Receding Horizon）原理，以及為何 MPC 以閉迴路方式反覆解開迴路最佳化問題。
- **建立** 線性 MPC 的二次規劃（QP）問題，含狀態與輸入約束。
- **組裝** 預測矩陣與約束矩陣，將 MPC 規格映射到數值求解器。
- **調參** 視窗長度與權重，平衡性能與計算負擔。
- **實作** 基本線性 MPC 控制器並在簡單 LTI 系統上測試。
- **對照** 決定論 MPC 與採樣式 MPPI（下週預告）。

### 1) 為什麼是 MPC？
經典線性控制器（如 PID、LQR）在許多任務上表現良好，但缺乏顯式處理硬性約束（致動器限制、安全邊界）與前瞻性規劃的能力。MPC 同時解決這兩點：
- **多步預測**：基於模型，對未來若干步狀態進行預測。
- **含約束最佳化**：在滿足狀態與輸入限制下，選擇使成本最小的輸入序列。
- **滾動時域回授**：僅施加第一個最佳化得到的控制量，下一個取樣時刻再以最新狀態重解。此作法讓系統對擾動與建模誤差更具魯棒性。

MPC 已廣泛應用於程序控制、車載系統（自適應定速、車道維持）、機器人（軌跡追蹤）、能源與航太等領域。

### 2) 離散時間 LTI 模型與記號
我們考慮取樣時間為 \(T_s\) 的離散時間線性時不變（LTI）系統：
\[ x_{k+1} = A x_k + B u_k, \qquad y_k = C x_k + D u_k. \]
- \(x_k \in \mathbb{R}^{n_x}\)：步驟 \(k\) 的狀態
- \(u_k \in \mathbb{R}^{n_u}\)：輸入
- \(y_k \in \mathbb{R}^{n_y}\)：輸出

常見約束：
\[ x_k \in \mathcal{X} = \{ x : x_{\min} \le x \le x_{\max} \}, \quad u_k \in \mathcal{U} = \{ u : u_{\min} \le u \le u_{\max} \}. \]
亦可加入輸入變化率約束 \(\Delta u_k = u_k - u_{k-1}\) 以保護致動器並降低抖動。

設預測視窗長度為 \(N\)。決策變數通常取為堆疊的未來輸入：
\[ \mathbf{u} = \begin{bmatrix} u_k^\top & u_{k+1}^\top & \cdots & u_{k+N-1}^\top \end{bmatrix}^\top. \]
在輸入僅決策的形式下，狀態以預測方程消去；也可採狀態-輸入聯合決策的等價形式。

### 3) MPC 目標與二次成本
標準追蹤型 MPC 以參考 \(r\) 為目標，懲罰輸出誤差與輸入（或輸入變化量）。設 \(Q \succeq 0\)、\(R \succ 0\)、末端權重 \(P \succeq 0\)：
\[ J(\mathbf{u}; x_k) = \sum_{i=1}^{N} \| y_{k+i} - r_{k+i} \|_{Q}^2 + \sum_{i=0}^{N-1} \| u_{k+i} - u_\text{ref} \|_{R}^2 + \| x_{k+N} - x_\text{ref} \|_{P}^2. \]
常見假設有 \(u_\text{ref}=0\) 與常值參考 \(r\)。亦常懲罰輸入變化量：
\[ J_\Delta = \sum_{i=0}^{N-1} \| \Delta u_{k+i} \|_{S}^2, \quad S \succeq 0. \]
當預測模型為線性時，上述成本為二次函數，配合線性約束即形成凸的二次規劃（QP），可由高效率求解器即時求解。

實務上應對狀態與輸入進行尺度化，使成本各項量級相當；否則將導致數值病態，拖慢收斂。

### 4) 視窗內的預測模型（提升式表達）
以堆疊形式表達未來狀態與輸出，令 \(D=0\) 以簡化：
\[
\mathbf{x} = \begin{bmatrix} x_{k+1} \\ x_{k+2} \\ \vdots \\ x_{k+N} \end{bmatrix}
= \underbrace{\begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}}_{\mathcal{A}} x_k
+ \underbrace{\begin{bmatrix}
B & 0 & \cdots & 0 \\
A B & B & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
A^{N-1} B & A^{N-2} B & \cdots & B
\end{bmatrix}}_{\mathcal{B}} \mathbf{u}.
\]
輸出類似地有 \(\mathbf{y} = \mathcal{Y} x_k + \mathcal{U} \mathbf{u}\)。

據此，成本可寫為標準 QP 形式：
\[ \min_{\mathbf{u}} \; \tfrac{1}{2} \mathbf{u}^\top H \, \mathbf{u} + f(x_k)^\top \, \mathbf{u} + \text{const}, \]
其中
\[ H = 2 (\mathcal{U}^\top \bar{Q} \, \mathcal{U} + \bar{R} + \bar{S}), \qquad f(x_k) = 2 \, \mathcal{U}^\top \bar{Q} (\mathcal{Y} x_k - \mathbf{r}) + \text{（含變化率項）}, \]
\(\bar{Q}, \bar{R}, \bar{S}\) 為沿視窗堆疊的區塊對角權重矩陣，\(\mathbf{r}\) 為堆疊之參考。末端成本可由最後預測狀態轉換至 \(H, f\)。

### 5) 視窗內約束
狀態與輸入約束皆可轉化為對 \(\mathbf{u}\) 的線性不等式：
\[ x_{\min} \le \mathbf{x} \le x_{\max} \;\; \Rightarrow \;\; G_x \, \mathbf{u} \le h_x + E_x x_k, \]
\[ u_{\min} \le \mathbf{u} \le u_{\max} \;\; \Rightarrow \;\; G_u \, \mathbf{u} \le h_u. \]
輸入變化率可用差分算子矩陣 \(D_\Delta\) 表示，使 \(\Delta \mathbf{u} = D_\Delta \mathbf{u} + d_\Delta u_{k-1}\) 而得線性約束。若設末端集合 \(x_{k+N} \in \mathcal{X}_f\) 為多面體，亦可成線性不等式；若 \(\mathcal{X}_f\) 為控制不變集合，可助於穩定性與遞迴可行性。

總結之，QP 可寫為：
\[
\begin{aligned}
\min_{\mathbf{u}} \;& \tfrac{1}{2} \mathbf{u}^\top H \mathbf{u} + f(x_k)^\top \mathbf{u} \\
\text{s.t. }\;& G \, \mathbf{u} \le h + E \, x_k, \\
& A_{eq} \, \mathbf{u} = b_{eq} + E_{eq} x_k \quad (\text{選配等式約束}).
\end{aligned}
\]
在權重半正定且約束線性時，此問題為凸 QP。

### 6) 滾動時域（Receding Horizon）原理
MPC 不是一次性開迴路規劃，而是每個取樣時刻皆執行：
1. **量測/估測** 目前狀態 \(x_k\)。
2. **求解** 有限視窗的含約束最佳化，得到 \(\mathbf{u}^* = [u_k^{*\top} \cdots u_{k+N-1}^{*\top}]^\top\)。
3. **僅施加第一個控制量**：\(u_k = u_k^*\)。
4. **系統前進** 至 \(k+1\)，更新狀態，視窗右移，重複以上步驟。

此法封閉回授迴路，對擾動與模型誤差具備魯棒性。實務中常以前一步解的平移解作為熱啟動（warm start），可大幅減少迭代次數。

### 7) MPC 與二次規劃（QP）
當成本二次且約束線性時，MPC 轉化為 QP：
- **目標**：對 \(\mathbf{u}\) 的凸二次函數。
- **約束**：線性不等式/等式。
- **求解器**：主動集法、內點法、運算子分裂（如 OSQP/ADMM）常用於即時 MPC。

求解關鍵考量：
- **熱啟動**：以移位後的上一步解初始化，能顯著減少迭代。
- **即時可行**：選擇 \(N\) 與模型維度，使求解時間小於 \(T_s\)。
- **尺度化**：狀態/輸入的尺度需相近，避免病態化。
- **軟性約束**：為避免不可行，可加入鬆弛變數 \(\epsilon \ge 0\) 並施以大權重懲罰。
- **疏矩陣**：利用帶狀/區塊結構，降低計算與記憶體負擔。

內點法收斂性佳，但每次迭代需分解矩陣；運算子分裂方法（如 OSQP）可預先分解結構固定的 KKT 系統，對固定結構的 MPC 問題相當高效。

### 8) 穩定性與可行性（概覽）
MPC 的閉迴路穩定性常藉由：
- **末端成本** \(\|x_{k+N} - x_\text{ref}\|_P^2\)，其中 \(P\) 可取無約束無限視窗 LQR 的黎卡提解，以近似視窗之外的尾端成本。
- **末端約束** \(x_{k+N} \in \mathcal{X}_f\)，確保視窗之外存在局部穩定器維持在可行集合內。
- **遞迴可行性**：透過不變末端集合與適當的約束收縮（魯棒 MPC）確保每一步皆有解。

本週以直觀與實作為主，形式性證明與魯棒 MPC 將於進階主題討論。

### 9) 範例：雙積分器（Double Integrator）含約束
考慮一維質點，狀態為位置與速度 \(x = [p, v]^\top\)，輸入為加速度 \(u\)，取樣時間 \(T_s\)：
\[ A = \begin{bmatrix} 1 & T_s \\ 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} \tfrac{1}{2} T_s^2 \\ T_s \end{bmatrix}. \]
目標：在約束 \(|u| \le u_{max}\)、\(|v| \le v_{max}\)（以及選擇性 \(|\Delta u| \le d_{max}|\)）下，將 \(p\) 推向設定值 \(p_{ref}\)。

典型成本（視窗 \(N\)）：
\[ J = \sum_{i=1}^{N} (p_{k+i}-p_{ref})^2 q_p + v_{k+i}^2 q_v + \sum_{i=0}^{N-1} u_{k+i}^2 r + \sum_{i=0}^{N-1} (u_{k+i}-u_{k+i-1})^2 s. \]
建立 \(\mathcal{A}, \mathcal{B}\) 與約束後解 QP。MPC 可前瞻性地平衡追蹤速度與限制遵循，通常能避免過衝並產生平滑控制。

觀察：
- 增大 \(q_p\) 可強化位置追蹤但可能使輸入更激進。
- 增大 \(r\) 可降低輸入幅值；小 \(s\) 有助於平滑，減少抖動。
- 當 \(v_{max}\) 較嚴格時，最佳化會以安全為優先，延長收斂時間。

### 10) 實作模式：如何組裝 QP
每一步 \(k\)：
- **輸入**：當前狀態估測 \(\hat{x}_k\)、參考序列 \(\mathbf{r}\)、前一時刻輸入 \(u_{k-1}\)（若懲罰/限制 \(\Delta u\)）。
- **建構預測矩陣**：\(\mathcal{A}, \mathcal{B}, \mathcal{Y}, \mathcal{U}\)（視窗 \(N\)）。
- **建立成本**：由 \(Q, R, S, P\) 算得 \(H, f\)，含末端項。
- **建立約束**：狀態/輸入界、變化率限制、末端集合，整合成 \(G, h, E\)。
- **解 QP**：使用上一步移位解熱啟動。
- **施加控制**：只取第一個控制量，並保存解以供下次熱啟動。

數值訣竅：
- 對固定 \(A, B, C\) 與 \(N\) 預先計算結構，僅更新依賴 \(x_k\) 與 \(\mathbf{r}\) 的部分。
- 採用稀疏矩陣，充分運用帶狀/區塊結構。
- 算時吃緊時，可縮短 \(N\)、簡化約束、換更快的求解器（如 OSQP）並使用熱啟動。
- 可採「動作區塊化」（move blocking）以減少決策維度，適用於長視窗。

### 11) 實驗（Lab）：線性系統的簡單 MPC
請在離散時間 LTI 系統上實作含約束的 MPC（建議使用雙積分器或簡化熱傳模型）。語言建議 Python，並使用 `cvxpy`（可調用 OSQP）或 `qpsolvers`；MATLAB 亦可。

- **系統**：雙積分器，\(T_s = 0.1\) s。
- **約束**：\(|u| \le 1.0\)、\(|v| \le 2.0\)。
- **視窗**：\(N = 15\)。
- **權重**：\(q_p = 10, q_v = 1, r = 0.1, s = 0.01\)。
- **任務**：將位置由 \(p_0 = 0\) 以平滑控制追蹤至 \(p_{ref} = 1\)，儘量降低過衝。

建議步驟：
1. 離散化模型（或直接使用給定 \(A, B\)）。
2. 撰寫函式建立 \(\mathcal{A}, \mathcal{B}\)（視窗 \(N\)）。
3. 建立區塊對角權重矩陣與輸出提升映射（可只追蹤位置）。
4. 組裝 \(H, f\) 與線性約束成標準 QP。
5. 以熱啟動實作滾動時域閉迴路模擬。
6. 繪製 \(p(t), v(t), u(t)\)，觀察約束活化、對 \(N, q_p, r, s\) 的敏感度。

CVXPY 參考骨架（為簡潔，採 \(n_u=1\)）：
```python
import numpy as np
import cvxpy as cp

Ts = 0.1
A = np.array([[1.0, Ts], [0.0, 1.0]])
B = np.array([[0.5*Ts**2], [Ts]])

# 視窗與維度
N = 15
nx, nu = A.shape[0], B.shape[1]

# 權重
q_p, q_v, r, s = 10.0, 1.0, 0.1, 0.01
Qy = np.diag([q_p, q_v])
R = r * np.eye(nu)

# 約束界
nu_max = 1.0
v_max = 2.0

# 決策變數：堆疊的未來輸入 U = [u_k ... u_{k+N-1}]
U = cp.Variable((nu*N, 1))

# 建立提升式預測矩陣
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

# 區塊對角權重
Q_bar = np.kron(np.eye(N), Qy)
R_bar = np.kron(np.eye(N), R)

# 視窗內參考（追蹤 p=1, v=0）
r = np.zeros((2*N, 1))
r[0::2, 0] = 1.0

# 參數：每步更新的當前狀態
xk = cp.Parameter((nx, 1))

# 預測狀態堆疊 y = A_bar xk + B_bar U（此處 y 含 p,v）
y = A_bar @ xk + B_bar @ U

# 懲罰 Δu（可選）：建立差分算子
if N > 1:
    D = np.eye(N) - np.roll(np.eye(N), 1, axis=0)
    D[0, :] = 0.0
    D_big = np.kron(D, np.eye(nu))
    dU = D_big @ U
else:
    dU = 0.0

obj = cp.quad_form(y - r, Q_bar) + cp.quad_form(U, R_bar)
if N > 1:
    obj += s * cp.sum_squares(dU)

# 不等式約束：|u| <= nu_max, |v| <= v_max
constraints = []
for i in range(N):
    ui = U[i*nu:(i+1)*nu]
    constraints += [cp.abs(ui) <= nu_max]
for i in range(N):
    xi = y[i*nx:(i+1)*nx]
    constraints += [cp.abs(xi[1]) <= v_max]

prob = cp.Problem(cp.Minimize(obj), constraints)

# 模擬
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
注意：
- 範例著重清晰性；實務中應向量化約束並運用稀疏結構。
- 差分算子 `D_big` 懲罰輸入變化；若要硬性限制變化率，請改以成對線性不等式表示。
- 若求解器僅接受純 QP 形式（無絕對值/錐約束），請以成對線性不等式取代 `abs`。

繳交：
- 不同 \(N\)、\(r, s\) 下的 \(p, v, u\) 時間歷程圖。
- 求解時間與熱啟動效果之簡短報告。

### 12) 調參指南
- **視窗長度 \(N\)**：越長前瞻性越好，但計算越重。慢系統可由 \(10\text{–}20\) 起步；若時限緊，可縮短 \(N\)。
- **權重**：增大 \(Q\) 降低追蹤誤差；增大 \(R\) 降低致動；小 \(S\) 平滑輸入。確保單位與量級合理。
- **軟性約束**：偶發不可行時加入鬆弛變數並重罰；安全關鍵約束仍建議維持為硬性。
- **取樣時間 \(T_s\)**：越快的取樣可提升擾動抑制，但留給求解的時間越短；減小 \(T_s\) 時可相應縮短 \(N\)。
- **熱啟動與預計算**：快取分解或 KKT 結構，於連續步驟重用。

### 13) 從 MPC 到 MPPI（預告）
下週介紹 Model Predictive Path Integral（MPPI），一種採樣式、無需導數的方法。
- **MPC vs MPPI**：
  - MPC：以模型與梯度解 QP/NLP；適合模型良好、維度較低、凸或輕度非凸且需硬性約束者。
  - MPPI：產生大量擾動控制軌跡模擬並做重要性加權平均；對非線性與不連續較魯棒；易於 GPU 並行；處理非凸性。
- **橋接概念**：
  - 皆於有限視窗最佳化，僅施加第一步控制（滾動時域）。
  - 皆重視成本設計與視窗平移；皆可納入（硬/軟）約束概念。
  - 熱啟動/軌跡平移在 MPPI 中可作為擾動序列的均值初始化。
- **準備**：熟悉視窗平移、成本 shaping、以及如何解讀與運用預測軌跡；這些直覺將直接移植到 MPPI 的採樣軌跡上。

### 14) 常見陷阱與除錯
- **狀態估測**：需準確 \(x_k\)。使用觀測器/濾波器；延遲或偏差會造成約束預測錯誤。
- **模型失配**：若真實系統與 \(A,B\) 有差距，考慮魯棒約束、軟化、擾動建模或線上識別。
- **尺度不佳**：狀態/輸入量級差太大導致病態化；請尺度化並重設權重。
- **求解逾時**：縮短 \(N\)、簡化約束、預分解矩陣、選更快求解器並檢查熱啟動。
- **約束振盪**：增加 \(\Delta u\) 懲罰及/或降低 \(Q\)，或提高 \(R\) 抑制 bang-bang 行為。
- **大階躍參考**：以斜坡參考或預覽減輕飽和衝擊。

### 15) 閱讀指南：Camacho 第3–4章
- **第3章**：預測控制核心概念、動態矩陣控制（DMC）、滾動時域實作細節、調參哲學（如抑制控制動作）。
- **第4章**：約束處理、設定值追蹤結構（如 GPC）、末端項與工業 MPC 實務。
建議聚焦：預測的動態矩陣如何形成、動作區塊化的角色、最佳化結構與約束處理（硬 vs 軟）。

### 16) 術語表
- **滾動時域（Receding Horizon）**：僅施加最優序列第一步，下一步重解。
- **預測視窗（N）**：未來強制成本與約束的步數。
- **二次規劃（QP）**：二次目標、線性約束之凸最佳化。
- **末端成本/約束**：助於穩定性與遞迴可行性。
- **熱啟動（Warm Start）**：以前一步移位解初始化求解器。
- **動作區塊化（Move Blocking）**：將多步輸入綁定相等以降維。

### 17) 檢核清單
- 能否寫出提升式預測 \(\mathbf{x} = \mathcal{A} x_k + \mathcal{B} \mathbf{u}\)？
- 能否將權重與參考映射成 \(H\) 與 \(f\)？
- 能否將輸入/狀態/變化率界轉成 \(G \mathbf{u} \le h + E x_k\)？
- 是否實作滾動時域與熱啟動並產生時間歷程圖？
- 是否評估 \(N\) 與權重設定對求解時間與性能之影響？

### 18) 延伸探索
- 加入末端 LQR 成本，觀察穩定性與收斂改善。
- 嘗試動作區塊化，於長視窗時降低決策維度。
- 做魯棒性測試：加入擾動與模型失配。
- 比較 OSQP 與內點法於速度/精度/解品質之差異。
- 擴展至輸出受限、狀態受限的更完整追蹤問題。

---

本週我們奠定 MPC 基礎：問題表述、滾動時域原理與 QP 實作。透過實驗，你將把理論落地；下週我們將過渡到採樣為本的 MPPI，探討其在非線性、非凸情境下的優勢。