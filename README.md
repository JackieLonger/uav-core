# UAV Multi-Drone Signal Optimization System# 🎯 UAV 訊號優化系統 - 快速開始



> 多無人機協同 LoRa 訊號優化系統 - 基於 ROS2 + PX4 的自主訊號搜尋與定位## 📚 核心文檔 (按優先級)



[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)| 文檔 | 用途 | 讀者 | 時間 |

[![PX4](https://img.shields.io/badge/PX4-v1.14.3-green)](https://docs.px4.io/)|------|------|------|------|

[![Python](https://img.shields.io/badge/Python-3.10-yellow)](https://www.python.org/)| **TESTING_GUIDE.md** ⭐⭐⭐ | 完整測試流程和邏輯 | **所有人 必讀** | 20 分鐘 |

[![License](https://img.shields.io/badge/License-MIT-orange)](LICENSE)| **QUICK_REFERENCE.md** ⭐⭐ | 快速命令參考 | 開發者 | 5 分鐘 |

| **JETSON_DEPLOYMENT_GUIDE_FINAL.md** ⭐⭐ | Jetson 部署詳細指南 | Jetson 用戶 | 15 分鐘 |

---| **FINAL_BUILD_REPORT.md** | 編譯驗證報告 | 構建工程師 | 10 分鐘 |



## 📖 項目簡介---



本系統實現多架無人機協同優化 Meshtastic LoRa 訊號質量，每架無人機自主搜索其綁定的地面 Tracker 組合，找到最佳訊號接收位置。## ⚡ 快速啟動命令



### ✨ 核心特性### 版本 A: 持續搜索 (永遠移動)



- 🚁 **多機協同**: 支持最多 3 架無人機同時作業（可擴展）```bash

- 🎯 **自主優化**: 基於 RSSI/SNR 的梯度上升算法# 終端 1

- 📡 **獨立綁定**: 每架無人機綁定專屬的兩個 Meshtastic TrackerMicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

- 🎨 **即時可視化**: RViz2 顯示所有無人機狀態、軌跡和訊號質量

- 🔧 **靈活部署**: 支援 Jetson Orin Nano 機載計算 + 筆電地面站架構# 終端 2

cd ~/uav-core && source install/setup.bash

### 🏗️ 系統架構ros2 launch px4_offboard single_drone_fast.launch.py \

  search_mode:=fast convergence_iterations:=999

``````

┌─────────────────────────────────────────────────────────────────┐

│                    筆記本（地面站）                              │**詳見:** `MODE_SEARCH.md`

│  ┌───────────────────────────────────────────────────────────┐  │

│  │  multi_drone_signal_optimizer.py (決策中心)              │  │---

│  │  - 多線程異步處理                                         │  │

│  │  - RViz2 可視化                                           │  │### 版本 B: 自動停留 (推薦) ⭐

│  └───────────────────────────────────────────────────────────┘  │

└─────────────────────────────────────────────────────────────────┘```bash

         ↕ (WiFi/DDS)# 終端 1

┌─────────────────────┐              ┌─────────────────────────┐MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

│  Jetson N (機載)    │              │  Pixhawk 6C (飛控)      │

│  - fast_scan_node   │◄────────────►│  - PX4 v1.14.3          │# 終端 2

│  - velocity_control │  (UART/MAVLink)│  - Offboard Mode       │cd ~/uav-core && source install/setup.bash

│  綁定: TrackerNA/NB │              └─────────────────────────┘ros2 launch px4_offboard single_drone_fast.launch.py \

└─────────────────────┘  search_mode:=fast convergence_iterations:=5

``````



**重要**: 6 個 Tracker 總數，每機綁定 2 個（Drone 1 → 1A/1B, Drone 2 → 2A/2B, Drone 3 → 3A/3B）**詳見:** `MODE_HOLD.md`



------



## 📋 目錄導航## 🧪 單機測試 5 步



### 🎯 快速開始1. **準備:** Jetson/PX4/Meshtastic 都已開機

- **[QUICK_START.md](QUICK_START.md)** - 5 分鐘快速啟動指南 ⭐ **推薦首先閱讀**2. **MicroXRCE:** 終端 1 執行上面的命令

- **[SINGLE_MACHINE_TESTING.md](SINGLE_MACHINE_TESTING.md)** - 單機測試步驟3. **ROS2 節點:** 終端 2 執行上面的命令

4. **監控:** 看終端 2 輸出 3-5 分鐘

### 📚 詳細文檔5. **完成:** 看到 `🎯 收斂完成!` 或永遠移動

- **[MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md)** - 完整系統使用手冊 ⭐ **主要文檔**

- **[HOLD_MECHANISM_REFACTORING_CHINESE.md](HOLD_MECHANISM_REFACTORING_CHINESE.md)** - 悬停優化機制說明**詳見:** `SINGLE_MACHINE_TEST.md`



### 🔧 開發文檔---

- **[thirdparty.repos](thirdparty.repos)** - 依賴包配置

- **[setup.sh](setup.sh)** - 環境設置腳本## 🔧 如何修改



---### 改變搜索算法



## 🛠️ 系統要求方案:

- 隨機遊走 (Random Walk) - 當前

### 硬體需求- 網格搜索 (Grid Search) - 完整覆蓋

- 梯度上升 (Gradient Ascent) - 聰明移動

| 組件 | 規格 | 數量 |- 螺旋搜索 (Spiral Search) - 系統搜索

|------|------|------|

| **無人機平台** | 支援 PX4 的四旋翼 | 1-3 台 |**詳見:** `HOW_TO_MODIFY.md` → 修改 1

| **飛控** | ARK Electronics Pixhawk 6C | 1-3 台 |

| **機載電腦** | NVIDIA Jetson Orin Nano (8GB) | 1-3 台 |---

| **地面站** | 筆記本電腦（Ubuntu 22.04） | 1 台 |

| **LoRa 模組** | Heltec Tracker V3 (Meshtastic) | 6 台 |### 改變信號平滑度

| **通訊** | WiFi 路由器（ROS2 DDS） | 1 台 |

改變訊號窗口大小:

### 軟體版本```python

if len(self.signal_history) > 3:  # ← 改 3 為其他值

#### 地面站 (筆記本)    self.signal_history.pop(0)

```yaml```

作業系統: Ubuntu 22.04 LTS

ROS2: Humble Hawksbill- 更小 (如 2): 反應快, 雜訊多

Python: 3.10+- 更大 (如 5): 反應慢, 更穩定

可視化: RViz2

依賴: numpy, scipy**詳見:** `HOW_TO_MODIFY.md` → 修改 2

```

---

#### 機載電腦 (Jetson)

```yaml### 改變改進判定

作業系統: Ubuntu 22.04 LTS (ARM64)

ROS2: Humble Hawksbill```python

Python: 3.10+if improvement > self.convergence_threshold:  # ← 改這裡

Meshtastic: meshtastic-python >= 2.2.0```

```

- 降低 (如 0.1): 更容易認為改進

#### 飛控 (Pixhawk)- 提高 (如 1.0): 只有顯著改進

```yaml

韌體: PX4 v1.14.3**詳見:** `HOW_TO_MODIFY.md` → 修改 3

協議: MAVLink 2.0

通訊: MicroXRCE-DDS Agent---

```

## � 快速啟動 (30 秒)

---

### Jetson 端

## 🚀 安裝步驟

```bash

### 1️⃣ 克隆倉庫# 1. 進入工作目錄並編譯 (首次)

cd ~/uav-core && colcon build --merge-install

```bash

cd ~# 2. 載入環境

git clone https://github.com/JackieLonger/uav-core.gitsource install/setup.bash

cd uav-core

```# 3. 啟動優化器

ros2 run ros2_px4_offboard_example signal_optimizer_node_v3.py

### 2️⃣ 安裝 ROS2 依賴```



```bash### 筆電端 (監控)

# 安裝 ROS2 Humble (如果未安裝)

# 參考: https://docs.ros.org/en/humble/Installation.html```bash

# 終端 1: 訊號掃描

# 安裝 vcs 工具source ~/uav-core/install/setup.bash

sudo apt install python3-vcstoolros2 run ros2_px4_offboard_example fast_scan_node.py



# 拉取第三方依賴包# 終端 2: 監看無人機狀態

vcs import src < thirdparty.reposros2 topic echo /drone_1/optimizer_status



# 安裝 ROS2 依賴# 終端 3: 監看訊號歷史

cd ~/uav-coreros2 topic echo /drone_1/signal_history

rosdep install --from-paths src --ignore-src -r -y```

```

---

### 3️⃣ 安裝 Python 依賴

## 📋 測試流程

```bash

# 地面站和 Jetson 都需要### 🎯 單機測試流程

pip3 install meshtastic numpy scipy

``````

[Jetson 連接 Pixhawk]

### 4️⃣ 編譯工作空間         ↓

[啟動優化器節點]

```bash         ↓

cd ~/uav-core[檢查訊號接收]

colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo         ↓

```[自動 ARM 和起飛]

         ↓

### 5️⃣ 配置環境變量[在 3×3×3m 搜索最佳訊號點]

         ↓

```bash[30-90 秒後收斂]

# 添加到 ~/.bashrc         ↓

echo "source ~/uav-core/install/setup.bash" >> ~/.bashrc[進入 HOLD 模式停留]

source ~/.bashrc```

```

**詳細步驟 → 參考 TESTING_GUIDE.md**

### 6️⃣ Jetson 額外配置

### 🚁 多機獨立測試流程 (推薦)

```bash

# 在每台 Jetson 上安裝 MicroXRCE-DDS Agent```

# 參考: https://docs.px4.io/main/en/middleware/uxrce_dds.html[Jetson-1]  [Jetson-2]  [Jetson-3]

     ↓           ↓            ↓

# 或使用快速安裝腳本  [啟動]      [啟動]       [啟動]

cd ~/uav-core     ↓           ↓            ↓

./setup.sh  [搜索]      [搜索]       [搜索]

```  空間1      空間2       空間3

  

---完全獨立，無干擾 ✅

```

## ⚡ 快速啟動

**詳細步驟 → 參考 TESTING_GUIDE.md**

### Jetson 端（每台獨立啟動）

---

```bash

# Drone 1## ✅ 成功標誌

cd ~/uav-core

./launch_jetson.sh 1 !tracker1A !tracker1B### 單機測試

```bash

# Drone 2# 預期輸出

./launch_jetson.sh 2 !tracker2A !tracker2B[INFO] 🤖 Signal optimizer v3 ready | Drone: drone_1 | Mode: search

[INFO] 🔋 Vehicle status: ARMING_STATE_STANDBY

# Drone 3[INFO] 🚀 Armed successfully

./launch_jetson.sh 3 !tracker3A !tracker3B[INFO] ✈️ Takeoff complete, starting search

```[INFO] 📊 Evaluating signal... Score: 42.5

# ... 繼續搜索 ...

**注意**: 替換 `!tracker1A` 等為實際的 Meshtastic 節點 ID[INFO] 🎯 Convergence detected! Switching to HOLD mode

[INFO] ⏸️ Holding position at (x, y, z)

### 筆記本端（地面站）```



```bash### 多機測試

# 終端 1: 啟動優化器```bash

cd ~/uav-core# 終端輸出 (多個無人機)

./launch_multi_drone_optimizer.sh/drone_1/optimizer_status

/drone_2/optimizer_status

# 終端 2: 啟動 RViz2 可視化/drone_3/optimizer_status

./launch_rviz.sh

```# 各無人機位置不重疊，獨立收斂

```

### 完整啟動流程

---

詳見 **[QUICK_START.md](QUICK_START.md)** 中的 5 步驟說明

## 🆘 常見問題

---

| 問題 | 解決方案 |

## 📊 主要功能|------|--------|

| **無法接收訊號** | 檢查 Meshtastic 是否配置；查看 `/link_quality` 話題 |

### 1. 訊號掃描 (`fast_scan_node.py`)| **無法 ARM** | 檢查 PX4 狀態；查看電池電量 |

- 每 2 秒掃描綁定的 Meshtastic Tracker| **無人機不動** | 檢查 OffboardControlMode 和速度命令是否發送 |

- 發布 `/drone_N/link_quality` 話題（RSSI + SNR）| **多機位置重疊** | 調整 `search_bounds_x/y/z` 參數；或分別在不同位置起飛 |

- 支援參數化配置 Tracker ID

**詳細故障排查 → 參考 TESTING_GUIDE.md**

### 2. 訊號優化 (`multi_drone_signal_optimizer.py`)

- 多線程異步決策（每機獨立線程）---

- 梯度上升算法（優先 Z 軸 → XY 平面）

- RSSI/SNR 綜合評分（各佔 50%）## 📚 文檔導航

- 自適應步長（0.1m ~ 0.5m）

- 3m × 3m × 3m 搜索空間- **TESTING_GUIDE.md** ⭐ - 完整測試邏輯和流程 (必讀!)

- **QUICK_REFERENCE.md** - ROS2 快速命令參考

### 3. 速度控制 (`velocity_control.py`)- **JETSON_DEPLOYMENT_GUIDE_FINAL.md** - Jetson 部署詳細步驟

- 轉換優化器速度指令為 PX4 格式- **FINAL_BUILD_REPORT.md** - 編譯驗證報告

- ARK Electronics 速度控制接口

- 100Hz 高頻發布---



### 4. 可視化 (`multi_drone_visualizer.py`)## 📝 每次修改後

- RViz2 MarkerArray 顯示

- 即時訊號質量（顏色編碼）```bash

- 飛行軌跡歷史cd ~/uav-core

- 每機獨立搜索邊界

- Tracker 綁定資訊顯示# 1. 重新編譯

colcon build --merge-install

---

# 2. 重新載入

## 🎨 RViz2 可視化說明source install/setup.bash



啟動 `./launch_rviz.sh` 後可看到：# 3. 重新啟動

ros2 run ros2_px4_offboard_example signal_optimizer_node_v3.py

| 元素 | 說明 |```

|------|------|

| 🟢 綠色球體 | 訊號質量良好（Q > 0.5） |---

| 🟡 黃色球體 | 訊號質量中等（Q ≈ 0.5） |

| 🔴 紅色球體 | 訊號質量差（Q < 0.5） |## 🎓 核心概念

| 🟦 青色框 | Drone 1 搜索邊界 |

| 🟪 洋紅框 | Drone 2 搜索邊界 || 概念 | 說明 |

| 🟨 黃色框 | Drone 3 搜索邊界 ||------|------|

| 📝 文本標籤 | 顯示 Drone ID、質量分數、Tracker 綁定 || **訊號評分** | `Score = SNR × 0.7 + (RSSI / -50) × 100 × 0.3` |

| ➡️ 黃色箭頭 | 當前速度向量 || **3×3×3m 搜索** | 每個無人機在 (±1.5m X/Y, 0~3m Z) 範圍內搜索 |

| 🔵 藍色軌跡 | 歷史飛行路徑（最近 100 點） || **雙 Tracker** | 優化器等待兩個信號源都提供數據後再決策 |

| **收斂判定** | 連續 5 次 (可配置) 無改進就進入 HOLD 模式 |

---| **HOLD 模式** | 無人機停留在最佳點，零速度命令 |



## 🧪 測試流程---



### 單機測試（5 分鐘）
參考 **[SINGLE_MACHINE_TESTING.md](SINGLE_MACHINE_TESTING.md)**

### 多機測試（15 分鐘）
參考 **[MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md)** 第 3 節

---

## 🔧 參數配置

### Jetson Launch 參數

```python
# jetson_onboard.launch.py
drone_id: int          # 無人機編號 (1, 2, 3)
tracker_a_id: str      # Tracker A 的 Meshtastic ID
tracker_b_id: str      # Tracker B 的 Meshtastic ID
```

### 優化器參數

```python
# multi_drone_optimizer.launch.py
num_drones: int        # 無人機數量 (預設: 3)
drone_ids: list        # 無人機 ID 列表 (預設: [1, 2, 3])
search_bounds_x: float # X 軸搜索範圍 (預設: 1.5m)
search_bounds_y: float # Y 軸搜索範圍 (預設: 1.5m)
search_bounds_z: float # Z 軸搜索範圍 (預設: 3.0m)
step_size: float       # 移動步長 (預設: 0.2m)
```

詳細參數說明見 **[MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md)** 第 5 節

---

## 🆘 常見問題

### Q1: 編譯時找不到 px4_msgs
```bash
# 解決方法：確保已拉取依賴
vcs import src < thirdparty.repos
colcon build --merge-install
```

### Q2: Jetson 連接不上 Pixhawk
```bash
# 檢查串口權限
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0

# 檢查 MicroXRCE Agent
ps aux | grep MicroXRCEAgent
```

### Q3: RViz2 看不到無人機
```bash
# 檢查話題
ros2 topic list | grep drone

# 檢查網路連接（地面站和 Jetson 需在同一網段）
ping [Jetson IP]
```

### Q4: 無人機不移動
```bash
# 檢查 Offboard 模式
ros2 topic echo /fmu/out/vehicle_status

# 檢查速度命令
ros2 topic echo /drone_1/offboard_velocity_cmd
```

更多問題見 **[MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md)** 第 6 節

---

## 📁 項目結構

```
uav-core/
├── src/
│   ├── ROS2_PX4_Offboard_Example/      # 主要 ROS2 套件
│   │   ├── px4_offboard/               # Python 節點
│   │   │   ├── fast_scan_node.py       # Meshtastic 掃描
│   │   │   ├── multi_drone_signal_optimizer.py  # 優化器
│   │   │   ├── multi_drone_visualizer.py       # 可視化
│   │   │   └── velocity_control.py     # 速度控制
│   │   ├── launch/                     # Launch 文件
│   │   │   ├── jetson_onboard.launch.py
│   │   │   └── multi_drone_optimizer.launch.py
│   │   └── resource/                   # RViz 配置
│   │       └── multi_drone.rviz
│   ├── px4_msgs/                       # PX4 消息定義（子模組）
│   └── px4_ros_com/                    # PX4-ROS2 橋接（子模組）
├── launch_jetson.sh                    # Jetson 啟動腳本
├── launch_multi_drone_optimizer.sh     # 地面站優化器啟動
├── launch_rviz.sh                      # RViz2 啟動
├── setup.sh                            # 環境設置腳本
├── thirdparty.repos                    # 依賴倉庫列表
├── QUICK_START.md                      # 快速開始指南
├── MULTI_DRONE_OPTIMIZER_GUIDE.md      # 完整使用手冊
├── SINGLE_MACHINE_TESTING.md           # 單機測試文檔
└── README.md                           # 本文件
```

---

## 🤝 貢獻指南

歡迎提交 Issue 和 Pull Request！

1. Fork 本倉庫
2. 創建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 開啟 Pull Request

---

## 📄 授權協議

本項目採用 MIT 授權 - 詳見 [LICENSE](LICENSE) 文件

---

## 👥 作者

**Jackie Longer** - [GitHub](https://github.com/JackieLonger)

---

## 🙏 致謝

- [PX4 Autopilot](https://px4.io/) - 開源飛控系統
- [ROS2](https://www.ros.org/) - 機器人操作系統
- [Meshtastic](https://meshtastic.org/) - LoRa 網狀網路通訊
- [ARK Electronics](https://arkelectron.com/) - Pixhawk 6C 飛控硬體

---

## 📞 聯繫方式

- 🐛 Issues: [GitHub Issues](https://github.com/JackieLonger/uav-core/issues)
- 💬 Discussions: [GitHub Discussions](https://github.com/JackieLonger/uav-core/discussions)

---

**⚠️ 安全提示**: 
- 請在安全的測試環境中操作無人機
- 遵守當地無人機飛行法規
- 確保有經驗的飛手在場
- 保持安全距離
- 隨時準備手動接管控制
