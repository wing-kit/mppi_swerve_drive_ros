### 第1週 — 機器人學與路徑規劃導論
課程：Introduction to Path Planning and Control in Robotics  
受眾：資訊工程大學部學生

### 學習目標
- 了解何謂機器人（Robot），並聚焦於行動式機器人（Mobile robots）。
- 解釋 swerve drive 平台並推導基本運動學（kinematics）關係。
- 能以正式方式表述路徑規劃（Path planning）問題（機器人、環境、目標、限制）。
- 定義組態空間（Configuration space, C-space），理解障礙如何映射至機器人的可運動空間。
- 建置 Python 與 ROS 的開發環境以進行簡單模擬。
- 將上述觀念連結至本 repo 的 swerve drive 脈絡，將理論落實到實作。

---

## 1. 什麼是機器人？為什麼需要路徑規劃？
- **Robot（機器人）**：能對環境進行感知、運算並執行動作的實體代理。典型流程：perception → planning → control → actuation。
- **Mobile robots（行動式機器人）**：可在空間中移動（2D/3D）；相對於固定基座的 manipulator（機械手臂）。本週聚焦於地面行動式機器人。
- **Path planning（路徑規劃）**：在滿足限制並避免碰撞的前提下，從起點到目標找出可行路徑的計算問題。
- **Control（控制）**：將路徑或軌跡即時轉換為馬達指令，同時處理擾動與不確定性。

文字示意圖：
```
[Sensors] -> [State Estimation] -> [Planning] -> [Control] -> [Actuators]
                             ^                                 |
                             |                                 v
                         [World / Environment / Robot Dynamics]
```

為何重要：
- 機器人系統必須安全且有效率地移動。
- 規劃將「做什麼」（高層）與「如何驅動」（低層）分離，提升模組化。

推薦閱讀：
- LaValle, Planning Algorithms，第1–2章（總覽與基礎概念）。可於作者網站找到免費版（搜尋「LaValle Planning Algorithms PDF」）。

---

## 2. 行動式機器人概觀
- **Differential drive（差速驅動）**：左右兩輪，控制左右輪速；簡單、具非完整性（nonholonomic）。
- **Ackermann（汽車式）**：以轉向角與車速控制；非完整性，存在最小轉彎半徑。
- **Omnidirectional（全向，如 mecanum/omni、swerve）**：可在平面上獨立平移與旋轉；在運動學層次常視為近似 holonomic。
- **Tracked robots（履帶式）**：以履帶移動；常以類似差速模型近似，但接觸力學不同。

常用假設：
- 平面運動（SE(2): x, y, θ）。
- 低速無打滑（pure rolling）。
- 以運動學近似（忽略複雜動力學）。

---

## 3. Swerve drive：直觀與運動學（kinematics）
Swerve 模組允許每個輪子同時具備轉向（steer）與驅動（drive）能力。此機構能在平面上朝任意方向平移，並同時任意旋轉，近似 holonomic 行為。

俯視文字圖：
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

記號：
- 機器人座標系中的期望底盤 twist：vx（m/s）、vy（m/s）、ω（rad/s）。
- 第 i 個輪子相對機器人中心的位置：(rxi, ryi)。
- 第 i 輪的平面速度向量：
  - vix = vx − ω·ryi
  - viy = vy + ω·rxi
- 第 i 輪速：si = sqrt(vix^2 + viy^2)
- 第 i 輪轉向角：αi = atan2(viy, vix)

實務重點：
- 若任一 si 超過最大輪速，需對所有 si 等比例縮放，以保持方向同時滿足限制。
- 為減少轉向行程，可在必要時對輪子方向加 π 並反向轉速，以最小化轉向角變化（angle optimization）。
- Field-centric control：使用全球座標（如 IMU/odometry）將操控向量旋回到機器人座標後再套用運動學。

產生 swerve 命令的偽程式：
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

與本 repo 的連結：
- 我們會以本 repo 的 swerve drive 脈絡實作上述映射，並在模擬中視覺化規劃動作由 swerve 底盤執行。後續實驗將把 planner 與 swerve 運動學、控制器串接。

---

## 4. Path planning 基礎
一般而言，運動規劃問題包含：
- **Input**：
  - Robot model（幾何與運動/動力學）。
  - Environment（障礙、地圖）。
  - 起始組態 qstart 與目標組態 qgoal。
  - 限制條件（運動學、非完整性、動力學、安全）。
  - 目標（最短路徑、最少時間、能源、風險等）。
- **Output**：在滿足條件下，由 qstart 到 qgoal 的可行路徑（或軌跡）。

關鍵術語（LaValle Ch. 1–2）：
- **State vs configuration**：
  - Configuration q：描述機器人幾何自由度（例：平面底盤 q = (x, y, θ)）。
  - State x 可含速度、感測器狀態等。幾何規劃常在 Configuration space（C-space）操作。
- **C-space（Configuration space）**：
  - 每一點對應機器人一個組態。
  - 工作空間（workspace）的障礙對應到 C-space 的禁區。
  - 可行集合為 C_free；禁區為 C_obs；全集為 C。
- **Holonomic vs nonholonomic**：
  - Holonomic：可即時在各 DOF 獨立控制（受界限約束）。
  - Nonholonomic：存在微分限制，限制當下可達速度（如汽車式）。

---

## 5. 組態空間（C-space）與障礙
想法：把「有尺寸的機器人 + 障礙」轉換為「C-space 的點 + 膨脹後的障礙」。

- 若機器人是半徑 r 的圓，在 2D 多邊形障礙中：
  - 將障礙以 r 膨脹（與半徑 r 圓的 Minkowski sum）。
  - 將機器人視為點。
  - 計劃此點在膨脹障礙外的路徑。
- 剛體多邊形機器人可類推（若含旋轉，C-space 變成 3D：x, y, θ）。

ASCII 圖：
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

形式化要素：
- C：所有組態的集合。
- C_obs：造成碰撞的組態集合。
- C_free = C \ C_obs。
- 規劃問題化簡為尋找連續映射 σ: [0,1] → C_free，且 σ(0)=qstart、σ(1)=qgoal。

本週聚焦 2D 幾何規劃以建立直覺；後續週次加入動力/運動學限制。

---

## 6. 離散 vs 連續規劃
- **離散規劃（Discrete planning）**：在 C_free 上建立圖/路網並搜尋（BFS、Dijkstra、A*）。常見於格點地圖或採樣路標。
  - 優點：簡單、在離散圖上具保證、應用廣。
  - 缺點：格點解析度影響品質；高維度計算昂貴。
- **連續規劃（Continuous planning）**：直接在連續 C 中規劃；方法含 potential fields、取樣式（PRM、RRT）、變分/最佳化法。
  - 優點：自然處理連續空間；取樣式具可擴充性。
  - 缺點：需要仔細的碰撞檢查；常為機率性保證。

我們先從格點與 BFS/A* 開始，再連結到 swerve 控制。

---

## 7. 可立即動手的簡單演算法

### 7.1 佔用格（Occupancy grid）與 BFS 尋路（2D）
- 以 2D 格點表示環境；0 = free，1 = obstacle。
- 在無權重格點上使用 BFS 可求最短步數路徑（4/8 邻接）。
- BFS 在等權重圖上保證最短路徑。

偽程式：
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

小例子（文字格）：
```
S . . # .
. # . # .
. # . . G
. . . # .
```
- S = start，G = goal，. = free，# = obstacle。
- BFS 會回傳一條最短格點路徑。

討論：
- 在較大地圖可用 A*（Euclidean 或 Manhattan heuristic）加速。
- 離散化會造成路徑鋸齒；後續可做平滑化。

### 7.2 由格點路徑到 swerve 底盤速度（high-level）
- 將格點中心轉為公尺座標的 waypoints。
- 每個時間步，計算朝向下一個 waypoint 的期望速度向量；可選擇 heading 策略（朝運動方向或朝目標）。
- 將期望 vx、vy、ω 餵給 swerve 運動學，取得各模組速度與角度。

高層偽程式：
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

這將離散規劃（waypoints）連接到 swerve 底盤的連續控制輸入。

---

## 8. 形式化問題定義（LaValle 視角）
基本幾何規劃問題：
- **C**：組態空間（如平面機器人的 SE(2)）。
- **qstart, qgoal ∈ C_free**：起點與終點組態。
- **C_obs ⊂ C**：障礙區域。
- **C_free = C \ C_obs**。
- **Objective**：找連續路徑 σ: [0,1] → C_free，σ(0)=qstart，σ(1)=qgoal。
- **Cost functional**（本週可選）：路徑長度 L(σ)、曲率懲罰、與障礙距離等。

若加入運動/微分限制（nonholonomic），每個組態的可行速度受限；此時常於 state space 以系統模型 ẋ = f(x, u) 規劃。

對 swerve 而言：
- 在運動學層常近似 holonomic：可在界限內任意指令平面 twist（vx, vy, ω）。
- 這使規劃相較汽車式（car-like）更簡化。

---

## 9. 實驗環境建置（Lab setup）
以 Python 進行快速原型；以 ROS 作為中介軟體（middleware）、訊息傳遞與模擬/整合。

### 9.1 作業系統假設
- 推薦 Linux（Ubuntu 22.04 LTS 或 24.04 LTS）。目前環境為 Linux，非常合適。
- Windows/macOS 可考慮 WSL2 或容器化。

### 9.2 Python 環境
- 安裝 Python 3.10+ 與 pip。
- 以虛擬環境隔離相依套件。
- 安裝常見科學運算套件。

指令：
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

可選但實用：
```bash
pip install shapely pillow tqdm
```

測試：
```bash
python -c "import numpy, matplotlib; print('Python OK')"
```

### 9.3 Jupyter 進行快速實驗
```bash
pip install jupyterlab
jupyter lab  # or jupyter notebook
```
- 建立本週 Notebook「W1_Planning_Basics.ipynb」。
- 用來原型 BFS 與 swerve 運動學工具函式。

### 9.4 ROS 2 安裝（Ubuntu 22.04 用 Humble；24.04 用 Jazzy）
請依官方 ROS 2 文件安裝（參見「Installation」）。
- Ubuntu 22.04（Humble）最小步驟如下：

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

驗證：
```bash
ros2 run demo_nodes_cpp talker | cat
# 於另一個終端：
ros2 run demo_nodes_cpp listener | cat
```

### 9.5 建立 ROS 2 工作空間
```bash
mkdir -p ~/ppc_ws/src
cd ~/ppc_ws
colcon build | cat
echo "source ~/ppc_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 9.6 IDE 與 Linter
- VS Code，安裝 Python 與 ROS 擴充。
- Linter/Formatter：`ruff`、`black`、`clang-format`（若用 C++），視需要安裝。

### 9.7 與本 repo 串接
- 將本 repo clone 至 `~/ppc_ws/src` 或你偏好的工作目錄。
- 若含 `CMakeLists.txt`/`package.xml`（ROS 2 套件），`colcon build` 會一併建置；否則以 Python path 或 venv 使用。
- 本週與下週會以本 repo 的 swerve 模組/範例作為規劃器的測試場域。

---

## 10. 範例演練（Worked examples）

### 10.1 膨脹障礙以建構圓形機器人的 C-space
已知：
- 機器人半徑 r。
- 二值地圖 M（1 = obstacle，0 = free）。
- 以半徑 r 的圓形結構元素進行二值膨脹（binary dilation）。

偽程式：
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
- 這在格點空間近似 Minkowski sum。

### 10.2 在佔用格上以 BFS 規劃並視覺化
- 使用前述 BFS 取得路徑。
- 以 `matplotlib` 繪製地圖與路徑。

最小繪圖例：
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

### 10.3 由期望 twist 計算 swerve 模組命令
- 對 L × W 底盤（原點在中心），輪位：
  - FL: (+L/2, +W/2)
  - FR: (+L/2, −W/2)
  - RL: (−L/2, +W/2)
  - RR: (−L/2, −W/2)

偽程式：
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

實驗：
- vx=0.5 m/s, vy=0.0, ω=0.0 → 所有輪角~0、等速。
- vx=0.0, vy=0.5 → 輪角~π/2、等速。
- vx=0.0, vy=0.0, ω=1.0 → 輪角呈切向；速度與至中心距離成正比。
- 混合 vx、vy、ω 以同時平移與旋轉。

---

## 11. 常見地雷與提示
- **忽略機器人尺寸**：若不膨脹障礙或未考慮幾何，易發生碰撞。
- **格點解析度不當**：太粗會切角，太細計算重。
- **座標系不清**：務必區分世界座標與機器人座標；swerve 常用機器人座標輸入；field-centric 需先旋回。
- **輪角換算**：避免大角度跳轉；以角度優化減少致動負擔。
- **速度飽和**：模組超限時等比例縮放；在縮放下維持一致的 heading 策略。
- **非完整性效應**：實機 swerve 可能非完美 holonomic（摩擦、延遲、限制），需靠控制器處理。

---

## 12. 與 LaValle（Ch. 1–2）的對應
- Ch. 1：動機、機器人型態、規劃流程——對應本章的概觀與規劃/控制分工。
- Ch. 2：基礎數學模型——C-space、障礙、state vs configuration、holonomic 限制。本章的圓形機器人膨脹與詞彙即源於此。

建議閱讀任務：
- 用你自己的話定義 C、C_free、C_obs。
- 手繪一個簡單環境，並畫出圓形機器人的膨脹障礙。
- 列出將 swerve 視為近似運動學 holonomic 所需的假設。

---

## 13. 實驗：繳交內容
本週結束前，請提交：
- 一個 notebook/script，示範：
  - 建立小型佔用格地圖與障礙。
  - 對半徑 r（格點數）的圓形機器人進行障礙膨脹。
  - 執行 BFS（或 A*）取得路徑。
  - 以 matplotlib 繪製地圖與路徑。
- 一個函式：對給定底盤 twist（vx, vy, ω）與底盤尺寸（L, W）計算 swerve 模組角度與速度，包含速度縮放。
- 1–2 段反思：C-space 的價值為何？car-like 與 swerve 在規劃上的差異？

評分重點：
- BFS/A* 的正確性與路徑重建。
- C-space 膨脹之合理性。
- 自底盤 twist 到模組命令之正確映射。
- 程式清晰度、必要註解、圖形可讀性。

---

## 14. 延伸挑戰（選做）
- 以 A*（Euclidean heuristic）取代 BFS，並比較展開節點數與時間。
- 路徑平滑化：以捷徑（shortcutting）或立方樣條（cubic spline）。
- 模擬簡單 pure pursuit 追蹤器，輸出（vx, vy, ω）。
- 加入 swerve 轉向角優化（必要時加 π）。
- Field-centric 指令：已知全域朝向後，先將速度向量旋回到機器人座標，再做 swerve 映射。

---

## 15. 速查表（Quick reference）

- **Configuration space（C-space）**：
  - 組態 q：決定工作空間幾何位置的參數。
  - C_obs：造成碰撞的組態集合。
  - 圓形機器人膨脹障礙 ≈ Minkowski sum。

- **Grid BFS**：
  - 4 vs 8 鄰接。
  - BFS 回傳邊數最短路；大圖上以 A* 提升效率。

- **Swerve kinematics**：
  - v_i = [vx − ω·ryi, vy + ω·rxi]。
  - speed = sqrt(vix^2 + viy^2)，angle = atan2(viy, vix)。
  - 超限時等比縮放；可考慮角度翻轉優化。

- **Path → Control 映射**：
  - 路徑格點 → 公尺座標的 waypoints。
  - 以 lookahead 取目標，計算機器人座標（vx, vy, ω）。
  - 輸入 swerve 模組映射。

- **Development environment**：
  - Python venv + numpy/matplotlib/scipy。
  - Jupyter 快速實驗。
  - ROS 2（Humble/Jazzy）中介、模擬、整合。
  - colcon 建置；source 安裝腳本。

---

## 16. 精煉思考題（Minimal study questions）
- Configuration 與 State 的差異？以平面底盤舉例。
- 為何對圓形機器人進行障礙膨脹？計算上有何好處？
- 在 swerve 中，ω 與輪位如何影響模組速度與角度？
- BFS 中 4 與 8 鄰接的權衡？
- A* 如何優於 BFS？什麼是 admissible heuristic？

---

## 17. 展望
- 第2週：基於搜尋的規劃（A*、D*、啟發式設計）、cost maps 與路徑平滑；將簡易追隨器整合到本 repo 的 swerve 底盤。
- 第3–4週：取樣式規劃（PRM、RRT/RRT*）、碰撞檢查與軌跡生成。
- 第5週起：回授控制、軌跡追蹤、處理不確定性之魯棒性。

---

## 18. 簡短詞彙表（Short glossary）
- **C-space（Configuration space）**：所有機器人組態的空間。
- **C_free**：C 中不碰撞的子集。
- **BFS**：無權重圖上的最短路寬度優先搜尋。
- **A***：具啟發式的最佳優先搜尋；啟發式可接受時具最優性。
- **Minkowski sum**：用於以機器人形狀膨脹障礙的幾何運算。
- **Holonomic**：理想下可瞬時控制所有自由度。
- **Swerve drive**：各輪可獨立轉向與驅動的全向底盤。

---

## 19. 快速檢核清單（Setup checklist）
- 已建立並啟用 Python venv。
- 已安裝並測試 numpy/matplotlib。
- Jupyter 可用；本週 Notebook 已建立。
- ROS 2 已安裝；demo talker/listener 驗證通過。
- Repo 已 clone；若為 ROS 套件可 colcon build；或可被 Python 匯入。
- BFS 範例可執行並繪圖。
- swerve 模組映射對測試 twist 產生合理角度/速度。

---

- 在 Notebook 實作 BFS 與障礙膨脹。
- 實作 swerve 運動學函式並以樣例 twist 測試。
- 確保 Python 與 ROS 環境可支援後續實驗。
