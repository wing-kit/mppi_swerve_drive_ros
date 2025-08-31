## 課程總覽：機器人路徑規劃與控制導論

本節將帶你了解本課程會學什麼、如何學，以及你將使用的工具。也會說明為何路徑規劃與控制是機器人的核心支柱，並展示課程的知識與實作路徑——我們將以本儲存庫中的 Model Predictive Path Integral（MPPI）controller 作為動手實作的基礎。

### 機器人簡介（Introduction to Robotics）

- **什麼是機器人**
  - **Robot**：可程式化的實體系統，能感知世界、做出決策，並執行動作。
  - 典型組成：
    - **Sensing**：相機、LIDAR、IMU、編碼器。
    - **Planning**：路徑／軌跡規劃、任務排序。
    - **Control**：精準且安全地追蹤期望的運動。
    - **Actuation**：馬達、伺服、輪子、螺旋槳。
- **Sense–Plan–Act 迴圈（文字圖示）**
  - 「圖示」：想像三個方塊組成的迴圈。
    - 方塊 1：「Sense（Perception）：估測機器人與環境的狀態。」
    - 方塊 2：「Plan（Decision）：計算安全且可行的軌跡。」
    - 方塊 3：「Act（Control）：以回授追蹤軌跡。」
  - 箭頭由 Sense → Plan → Act → 回到 Sense，隨著新資料持續更新。
- **課程定位**
  - 本課聚焦於「Plan」與「Act」的交界，重點在：
    - 在複雜環境中產生運動（planning）。
    - 在不確定與動態條件下穩健追蹤運動（control）。

### 為什麼路徑規劃與控制重要（Why Path Planning and Control Matter）

- **安全與可行性**
  - 規劃確保路徑能避開碰撞，並滿足運動學／動力學限制（例如 nonholonomic constraints）。
  - 控制確保機器人在擾動、打滑或建模誤差下仍能追蹤路徑。
- **效能與穩健性**
  - 優秀的規劃器能在大型地圖或動態障礙下迅速找到路徑。
  - 優秀的控制器能在精度、能耗、平順度與回應速度間取得平衡。
- **真實世界範例**
  - **倉儲機器人**：在貨架與工人之間規劃無碰路徑；控制在載重下仍能平順追蹤。
  - **自駕車**：規劃換道與匯入；控制在風力、坡度、車流影響下穩定速度與轉向。
  - **四旋翼**：規劃穿越窗戶的 3D 軌跡；控制推力與姿態應對陣風。
- **Planning vs. Control vs. Trajectory Optimization**
  - Planning 回答「要去哪裡？」針對地圖與限制找可行路徑。
  - Control 回答「如何跟上規劃好的軌跡？」在動力學與擾動下維持追蹤。
  - Trajectory optimization（如 MPPI）直接在動力學與成本下優化控制序列，融合規劃與控制。

### 課程形式與時程（Course Format and Duration）

- **時程**：14 週。
- **課程會議**：
  - 每週 2 堂講課（各 75 分鐘）。
  - 每週 1 堂實驗課（120 分鐘），使用 ROS、Gazebo 與本儲存庫。
- **評量**：
  - 5–6 次實驗（個人，具體評分規準）。
  - 2 次短測（針對核心演算法的概念測驗）。
  - 1 次期中考（規劃＋控制基礎）。
  - 期末專題（團隊式，整合 MPPI、感知與規劃）。
- **教學風格**：
  - 先以概念為主，搭配具體範例。
  - 以 MPPI controller 為基線的動手實作實驗。
  - 難度逐步提升，從 2D 差速驅動到更高維系統。

### 學習目標（Learning Objectives）

修課完成後，你將能：

- **概念**
  - 說明 Sense–Plan–Act 架構與規劃、控制的角色。
  - 比較圖搜尋、抽樣式與最佳化式的規劃方法。
  - 描述回授控制基礎（PID、LQR）與機器人系統的限制。
  - 說明隨機控制的直覺，以及為何 MPPI 適合非線性系統。
- **演算法與實作**
  - 實作 grid/graph 規劃器（Dijkstra/A*）與抽樣式規劃器（RRT/RRT*）。
  - 建立軌跡最佳化的成本函數與限制條件。
  - 使用、設定、擴充 MPPI controller 以支援不同機器人模型。
- **工具與系統**
  - 使用 ROS 進行訊息傳遞與節點協作；使用 Gazebo 進行模擬。
  - 透過 topics/services 與標準訊息型態銜接規劃與控制。
  - 在模擬中除錯、調參與評測規劃–控制管線。
- **評估與溝通**
  - 設計實驗、蒐集結果、分析效能／安全取捨。
  - 撰寫清楚的技術報告並進行精簡的示範。

### 先修需求（Prerequisites）

- **數學**
  - 線性代數（向量、矩陣、特徵值）。
  - 微積分（導數、梯度、泰勒展開）。
  - 機率基礎（分佈、期望、變異數）。
- **資工背景**
  - 資料結構與演算法（圖、搜尋、複雜度）。
  - 熟悉 Python，並了解 C++ 為佳。
  - Linux 指令列、Git 與 VS Code 或等效工具。
- **建議背景**
  - 基礎控制（transfer functions、stability）有幫助但非必備；課程會複習要點。
  - 熟悉常微分方程與數值積分。

### 必要資源（Required Resources）

- **教科書（建議，非全數必備）**
  - Steven M. LaValle, “Planning Algorithms” （線上免費）。
  - Sebastian Thrun, Wolfram Burgard, Dieter Fox, “Probabilistic Robotics.”
  - B. Siciliano 等, “Robotics: Modelling, Planning and Control”（動力學／控制背景）。
- **軟體**
  - Linux 環境（建議 Ubuntu；Windows 可用 WSL2）。
  - ROS 2（建議 Humble 或 Iron）。
  - Gazebo（Classic 或新一代 Gazebo；每次實驗會指定）。
  - Python 3.10+；視專題需求可用 C++17+。
  - 視覺化工具：RViz2、Matplotlib/PlotJuggler。
- **本儲存庫（This repository）**
  - 內容包含：
    - 可用的 MPPI controller（本課實驗的主要基線）。
    - 範例機器人模型與設定檔。
    - 實驗骨架、測試地圖與工具腳本。
    - 規劃器起始程式（或 stub）供你擴充。
  - 你會：
    - 在 Gazebo 直接執行 MPPI controller 作為基線展示。
    - 修改成本項、動力學模型與抽樣策略。
    - 將 MPPI 與你的規劃器整合，形成端到端自動化。
- **線上資源**
  - ROS 2 Tutorials 與概念（nodes、topics、services、parameters）。
  - Gazebo 教學（world 建立、sensor、robot models）。
  - 開源 MPPI 參考與文獻供深入閱讀。
  - 授課教師提供的講義、投影片與註解程式碼。

### 我們如何在課程中使用 MPPI（How We Will Use MPPI）

- **MPPI 概覽**
  - Model Predictive Path Integral（MPPI）為抽樣式 model predictive control 方法。
  - 展開多組含雜訊的控制序列，沿著系統動力學評估成本，並以成本加權平均產生控制更新。
  - 優點：可處理非線性動力學與非凸成本；容易平行化；彈性高且模組化。
- **本儲存庫包含**
  - 一個 MPPI controller，具備：
    - 可插拔的成本函數（抵達目標、碰撞懲罰、平順性）。
    - 常見平台的動力學模型（例如 differential drive；可擴充至 unicycle、car-like、quadrotor）。
    - 與 ROS topics 介面，用於狀態估測與控制指令。
    - 可設定的 horizon、抽樣變異度、成本權重。
- **你的 MPPI 學習路徑**
  - 實驗 1–2：在模擬中執行 MPPI 以抵達單一目標；在 RViz2 視覺化軌跡；解讀成本項。
  - 實驗 3–4：將簡單規劃器（A*）與 MPPI 整合；MPPI 進一步修飾／追蹤軌跡並避障。
  - 實驗 5–6：延伸 MPPI：
    - 加入動態障礙成本（time-indexed collision checks）。
    - 調整抽樣與 horizon，平衡反應性與平順性。
    - 替換動力學（例如差速驅動改車輛模型）。
  - 期末專題：以 MPPI 作為控制底層，整合感知與規劃，完成端到端示範。

「文字圖示」：一個星形「Planner」節點產生路徑；一個齒輪形「MPPI」節點在下游，沿動力學展開候選控制並以加權策略更新選擇動作；一個推進器形「Robot」執行並發布新狀態；箭頭迴圈回來，形成 receding horizon control。

### 每週主題概覽（可能微調）

- **第 1–2 週**：機器人基礎；運動學、動力學；Sense–Plan–Act；ROS/Gazebo 入門；執行 MPPI 基線。
- **第 3–4 週**：configuration space、occupancy grids；圖搜尋（Dijkstra、A*）、啟發式、motion primitives。
- **第 5–6 週**：抽樣式規劃（PRM、RRT、RRT*）；kinodynamic 變體；可行性 vs. 最適性。
- **第 7 週**：trajectory optimization 概論；MPC vs. MPPI；成本設計；限制；warm starts。
- **第 8 週**：回授控制；PID、LQR；線性化；追蹤控制器；controller–planner 介面。
- **第 9 週**：不確定性；stochastic optimal control；belief-space planning；不確定性下的 MPPI。
- **第 10 週**：動態障礙；時間參數化成本；預測與反應式規劃。
- **第 11 週**：多機協作；共享空間；大規模避撞。
- **第 12 週**：整合週；規劃–控制堆疊；benchmark 與 ablation studies。
- **第 13–14 週**：期末專題、展示與報告。

### 範例情境走讀（Example Scenario Walkthroughs）

- **迷宮中的差速驅動（differential-drive）機器人**
  - 規劃器：A* 於 2D occupancy grid 產生無碰路徑。
  - 控制器：MPPI 修飾並追蹤曲率受限的軌跡，懲罰急轉與貼牆行為。
  - 結果：平順通過並維持穩定安全距離。
- **停車場中的車輛型（car-like）機器人（nonholonomic）**
  - 規劃器：Hybrid A* 搭配可行 motion primitives。
  - 控制器：MPPI 採車輛動力學；成本包含車道中心、朝向對齊、jerk。
  - 結果：逼真的轉向與穩定倒車操作。
- **四旋翼（quadcopter）航點任務**
  - 規劃器：PRM 或 kinodynamic RRT*，滿足高度限制的 3D 路徑。
  - 控制器：MPPI 採簡化四旋翼動力學；高度與姿態正則化。
  - 結果：穩健追蹤航點並能抗擾動。

「文字圖示」：想像一個 2D 方格地圖，起點在左下，終點在右上。A* 找到一條貼著自由空間邊界的折線路徑。MPPI 疊加多條透明候選軌跡，沿 A* 路徑「微擺」，選出平順且安全的一條來追蹤。

### 課程政策與合作（Course Policies and Collaboration）

- **合作**
  - 歡迎討論概念與除錯策略。
  - 上交程式碼必須是你（或你的團隊）原創。
  - 若改編他人想法或片段，請註明來源。
- **可重現性**
  - 實驗與專題需附設定檔與腳本以重現結果。
  - 維護清楚的實驗記錄（隨機種子、horizon、noise scales、成本權重）。
- **專業實務**
  - 撰寫可讀性高的模組化程式，並以有意義的訊息常態提交。
  - 團隊合作時以 issues/PRs 維持清晰可審查的流程。

### 工具起步指南（Getting Started with Tools）

- **環境安裝選項**
  - 建議：Ubuntu（原生或 WSL2）。
  - 替代：課程提供的 Docker 映像，確保 ROS/Gazebo/colcon 一致性。
- **ROS 2 基礎**
  - 概念：nodes、topics、services、parameters、launch files。
  - 常用訊息：odometry、transforms（TF）、laser scans、`cmd_vel` 或控制輸入。
  - 除錯工具：`rqt_graph`、`rviz2`、`ros2 topic echo`、`ros2 param list`。
- **Gazebo 基礎**
  - Worlds：預建室內／室外測試地圖。
  - Models：差速驅動底盤、車輛型底盤、四旋翼（本庫提供）。
  - Sensors：LIDAR、深度相機、IMU。
- **儲存庫典型結構**
  - `mppi_controller/`：MPPI 核心程式。
  - `robots/`：URDF/SDF 模型與動力學設定。
  - `configs/`：成本權重、horizons、noise scales、規劃–控制配線。
  - `worlds/`：Gazebo worlds 與地圖。
  - `labs/`：實驗起始程式與說明。
  - `scripts/`：繪圖、記錄、評估的工具。

- **首次執行檢查表**
  - 下載本儲存庫。
  - 按指引安裝 ROS 2（Humble 或 Iron）與 Gazebo。
  - 建置並 `source` 工作空間。
  - 啟動最小模擬並確認 MPPI 有發布控制訊息。

- **常見指令（示例）**
```bash
# 建置工作空間
colcon build --symlink-install

# 於新終端載入 overlay
source install/setup.bash

# 檢查 ROS topics
ros2 topic list

# 啟動 MPPI controller 與示例世界（範例）
ros2 launch mppi_controller demo_world.launch.py
```

- **除錯思維**
  - 若機器人產生振盪：降低控制噪聲尺度、提高平順性成本、縮短 horizon。
  - 若發生碰撞：提高障礙／淨空成本、調整地圖解析度、改善規劃種子。
  - 若反應遲鈍：縮短 horizon 或提高控制頻率；調整權重以偏向進度。

「文字圖示」：想像三個滑桿，標示「Goal Progress」、「Smoothness」、「Clearance」。移動任一滑桿都會影響軌跡形狀；藝術在於三者的平衡。

### 實驗如何把概念落地（How Labs Connect Concepts to Practice）

- **實驗 1：MPPI 暖身**
  - 讓 point-mass 或 differential-drive 機器人於空場到達目標。
  - 成果：軌跡圖、討論 noise scale 與 horizon 對行為的影響。
- **實驗 2：障礙與成本**
  - 加入基於地圖的碰撞成本；在淨空與路徑長度間調參；視覺化成本熱圖。
  - 成果：並排比較、成本項刪減（ablation）研究。
- **實驗 3：規劃種子**
  - 實作 A*；將輸出與 MPPI 介接；衡量有／無規劃種子的追蹤誤差差異。
- **實驗 4：動力學切換**
  - 自差速驅動改為車輛型動力學；更新限制；重新調參。
- **實驗 5：動態障礙**
  - 導入移動障礙（時間索引成本）；評估 time-to-collision 與安全邊際。
- **實驗 6：穩健性研究**
  - 注入擾動（如模擬風）；比較不同參數集的穩定性；撰寫報告。

### 成功長什麼樣（What Success Looks Like）

- **技術面**
  - 能針對地圖與平台選擇並論證合適的規劃器。
  - 能設定與擴充 MPPI 以符合任務限制。
  - 能產出可重現且具體的評估指標（成功率、時間、淨空、控制能量）。
- **專業面**
  - 程式碼模組化、測試完善且有文件。
  - 報告精煉，含圖表與適切指標。
  - 能清楚溝通設計決策與取捨。

### 一個帶著走的心智模型（A Mental Model to Carry Forward）

- **層次**
  - Planning 提供在環境限制下的可行、導向目標的路徑。
  - MPPI 位於控制（或軌跡最佳化）層，將高階意圖轉為即時、符合動力學的控制。
- **你將內化的迴圈**
  - Sense → Plan/Update → Optimize（MPPI） → Act → Repeat。
- **工藝**
  - 成本塑形是一種語言：進度、安全、平順、能量之間的平衡。
  - 動力學寫實度重要：模型越貼近現實，預測越好——直到計算預算的界線。

### 常見問答（FAQ）

- **需要先會 ROS 嗎？**
  - 不需要。前兩週會教授基礎並在實驗中反覆練習。
- **MPPI controller 可以直接套在任何機器人上嗎？**
  - 需要調整。你將學會如何針對不同平台修改動力學與成本。
- **數學會很重嗎？**
  - 會，但偏應用。我們著重直覺與實作，數學用來支撐設計與調參。

### 最後的話（Final Note）

本課把理論與實務緊密結合。本儲存庫中的 MPPI controller 是你的沙盒：具體且可修改的地基，讓抽象概念立刻落地。課程結束時，你不僅理解規劃與控制，還能打造並調參一個能即時規劃與控制的系統。

- 「文字圖示」：一個分層蛋糕。底層：「Dynamics & Actuation」。中層：「Control（MPPI）」。頂層：「Planning & Decision」。蛋糕外層糖霜：「Perception & State Estimation」。頂端蠟燭：「Task Goal」。慶祝的是把蠟燭送到正確位置——安全、高效、可靠。

- 「文字圖示」：錐桶圍成的賽道。Planner 用粉筆畫出粗糙路線；MPPI 畫出平順賽車線，持續適應風與抓地力變化；控制器穩健轉向，一圈又一圈。

---

- **重點整理（Key takeaways）**
  - 你將學到核心的規劃與控制方法，並在 ROS/Gazebo 中實作。
  - 來自本儲存庫的 MPPI controller 是所有實驗與專題的主幹。
  - 預期融合理論、程式撰寫、調參與嚴謹評估，最後以整合式專題作結。