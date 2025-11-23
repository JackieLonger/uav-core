# 🎯 UAV 多無人機訊號優化系統

> 多無人機協同 LoRa 訊號優化系統 - 基於 ROS2 + PX4 的自主訊號搜尋與定位

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.14.3-green)](https://docs.px4.io/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-orange)](LICENSE)

---

## 📚 核心文檔

| 文檔 | 用途 | 時間 |
|------|------|------|
| **README.md** | 項目總覽與快速入門 | 10 分鐘 |
| **QUICK_START.md** | 快速命令參考 | 5 分鐘 |
| **DYNAMIC_TRACKER_TEST_GUIDE.md** | 動態 Tracker ID 測試 | 15 分鐘 |
| **DUAL_TERMINAL_TEST_GUIDE.md** | 雙終端測試流程 | 20 分鐘 |
| **MULTI_DRONE_OPTIMIZER_GUIDE.md** | 多機優化器詳細手冊 | 30 分鐘 |
| **FINAL_EXPERIMENT_CONFIRMATION.md** | 實驗前檢查清單 | 10 分鐘 |
| **ROS2_TOPIC_FLOW.md** | Topic 訂閱關係圖 | 10 分鐘 |

---

## 📖 項目簡介

本系統實現多架無人機協同優化 Meshtastic LoRa 訊號質量，每架無人機自主搜索其綁定的地面 Tracker 組合，找到最佳訊號接收位置。

### ✨ 核心特性

- 🚁 **多機協同**: 支持最多 3 架無人機同時作業（可擴展）
- 🎯 **自主優化**: 基於 RSSI/SNR 的梯度上升算法
- 📡 **動態綁定**: 每架無人機自動識別其綁定的兩個 Meshtastic Tracker
- 🎨 **即時可視化**: RViz2 顯示所有無人機狀態、軌跡和訊號質量
- 🔧 **靈活部署**: 支援 Jetson Orin Nano 機載計算 + 筆電地面站架構
- 🛡️ **安全保障**: RC Override 隨時可介入，邊界保護，速度限制

### 🏗️ 系統架構

```
┌─────────────────────────────────────────────────────────────────┐
│                    筆記本（地面站）                              │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  multi_drone_signal_optimizer.py (決策中心)              │  │
│  │  - 多線程異步處理                                         │  │
│  │  - 動態 Tracker ID 識別                                  │  │
│  │  - RViz2 可視化                                           │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
         ↕ (WiFi/ROS2 DDS)
┌─────────────────────┐              ┌─────────────────────────┐
│  Jetson N (機載)    │              │  Pixhawk 6C (飛控)      │
│  - fast_scan_node   │◄────────────►│  - PX4 v1.14.3          │
│  - velocity_control │  (UART/MAVLink)│  - Offboard Mode       │
│  綁定: TrackerNA/NB │              └─────────────────────────┘
└─────────────────────┘
         ↕ (USB)
┌─────────────────────┐
│  Meshtastic LoRa    │
│  Tracker A & B      │
└─────────────────────┘
```

**重要**: 
- 6 個 Tracker 總數，每機綁定 2 個（Drone 1 → 1A/1B, Drone 2 → 2A/2B, Drone 3 → 3A/3B）
- Tracker ID 在 Jetson 的 `fast_scan_node.py` 中手動配置
- 筆電端的優化器會自動識別每台無人機回傳的 Tracker ID（動態綁定）

---

## ⚡ 快速啟動命令

### Jetson 端（無人機機載）

```bash
# 1. 修改 fast_scan_node.py 中的 Tracker ID
# 檔案路徑：src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py
# 找到第 34-41 行配置區域，填入實際的 Tracker ID

# 2. 編譯並啟動
cd ~/uav-core
colcon build --merge-install
source install/setup.bash
./launch_jetson.sh 1  # Drone 1（只需指定 drone_id）
```

### 筆電端（地面站）

```bash
# 終端 1: 啟動優化器
cd ~/uav-core
source install/setup.bash
./launch_multi_drone_optimizer.sh

# 終端 2: 啟動 RViz2 可視化
cd ~/uav-core
source install/setup.bash
./launch_rviz.sh
```

---

## 🧪 測試步驟

### 情況 A：有真實硬體（推薦）

1. **Jetson 端**：連接 Meshtastic 硬體，啟動 `./launch_jetson.sh 1`
2. **筆電端**：啟動優化器和 RViz
3. **觀察日誌**：筆電端應顯示 "收到 Tracker 信號" 和 "移動 x/5"

### 情況 B：無硬體測試（代碼驗證）

1. **Jetson 端**：手動發送模擬數據
   ```bash
   # 循環發送模擬訊號
   while true; do
     ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
       "data: '{\"target_id\": \"!TEST_A\", \"status\": \"Success\", \"forward_rssi\": -80, \"forward_snr\": 10, \"return_rssi\": -85, \"return_snr\": 8, \"timestamp\": \"123456\"}'"
     sleep 1
     ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
       "data: '{\"target_id\": \"!TEST_B\", \"status\": \"Success\", \"forward_rssi\": -75, \"forward_snr\": 12, \"return_rssi\": -78, \"return_snr\": 11, \"timestamp\": \"123456\"}'"
     sleep 2
   done
   ```

2. **筆電端**：啟動優化器，觀察是否收到並處理數據

**詳細測試指南**：請參考 `DYNAMIC_TRACKER_TEST_GUIDE.md` 和 `DUAL_TERMINAL_TEST_GUIDE.md`

---

## 📋 系統需求

### 硬體需求

**無人機端（每台）**：
- Jetson Orin Nano (8GB 推薦)
- Pixhawk 6C 飛控
- Meshtastic Heltec Tracker V3 × 2（綁定專屬）
- GPS 模組（3D Fix）
- 電源系統

**地面站**：
- 筆記本電腦（Ubuntu 22.04）
- WiFi 連接（與 Jetson 同網段）

**地面 Tracker**：
- Meshtastic Heltec Tracker V3 × 6（固定位置）

### 軟體需求

**Jetson & 筆電（兩端都需安裝）**：
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- PX4 v1.14.3
- Meshtastic Python CLI

---

## 🛠️ 安裝步驟

### 1. Clone 專案

```bash
cd ~
git clone --recursive https://github.com/JackieLonger/uav-core.git
cd uav-core
```

### 2. 安裝 ROS2 依賴

```bash
# 安裝 PX4 消息定義
vcs import src < thirdparty.repos

# 安裝 Python 依賴
pip3 install meshtastic pyserial

# 編譯
colcon build --merge-install
```

### 3. 配置 Tracker ID（僅 Jetson 端）

編輯 `src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`：

```python
# 第 34-41 行
TRACKER_A_ID = "!實際的TrackerA_ID"  # ← 修改這裡
TRACKER_B_ID = "!實際的TrackerB_ID"  # ← 修改這裡
```

查詢 Tracker ID：
```bash
meshtastic --info
# 查看 "User ID" 欄位（格式：!xxxxxxxx）
```

### 4. 重新編譯

```bash
colcon build --merge-install
source install/setup.bash
```

---

## 📊 核心參數

| 參數 | 值 | 說明 |
|------|-----|------|
| **MAX_MOVEMENTS** | 5 | 最大移動次數 |
| **MAX_VELOCITY** | 0.3 m/s | 最大速度限制 |
| **ALTITUDE_THRESHOLD** | 1.5 m | 優先上升高度 |
| **BOUNDS_X** | ±1.5 m | X 軸邊界（相對起飛點）|
| **BOUNDS_Y** | ±1.5 m | Y 軸邊界（相對起飛點）|
| **BOUNDS_Z** | +3.0 m | Z 軸上限（只能上升）|
| **PUBLISH_RATE** | 100 Hz | 速度指令發布頻率 |

---

## 🚨 安全機制

1. **RC Override**: 遙控器隨時可切換回手動模式（PX4 內建）
2. **Failsafe 監控**: 失聯/電量低自動觸發 RTL
3. **邊界保護**: 超出 3m × 3m × 3m 空間自動停止
4. **速度限制**: 最大速度 0.3 m/s
5. **懸停機制**: 移動 3 秒後自動歸零速度

---

## 📝 核心檔案說明

### Python 節點

| 檔案 | 功能 | 運行端 |
|------|------|--------|
| `fast_scan_node.py` | 掃描 Meshtastic Tracker | Jetson |
| `velocity_control.py` | 速度控制（Twist → PX4）| Jetson |
| `multi_drone_signal_optimizer.py` | 多機優化決策中心 | 筆電 |
| `multi_drone_visualizer.py` | RViz2 可視化 | 筆電 |
| `control.py` | 鍵盤手動控制（選用）| Jetson/筆電 |
| `processes.py` | 輔助啟動腳本（選用）| 筆電 |

### Launch 腳本

| 腳本 | 功能 |
|------|------|
| `launch_jetson.sh` | Jetson 端啟動腳本 |
| `launch_multi_drone_optimizer.sh` | 筆電端優化器啟動 |
| `launch_rviz.sh` | RViz2 可視化啟動 |

---

## 🔧 常見問題排查

### 問題 1：筆電端收不到 link_quality

**檢查**：
```bash
# 在筆電上
ros2 topic list | grep link_quality
# 應顯示：/drone_1/link_quality, /drone_2/link_quality, ...

ros2 topic echo /drone_1/link_quality
# 應有 JSON 格式數據
```

**原因**：
- Jetson 未啟動或網路未連接
- Tracker 未綁定或 ID 錯誤
- ROS2 DDS 配置問題

### 問題 2：無人機不懸停，持續移動

**立即處理**：用遙控器切回手動模式

**檢查**：
```bash
# 在 Jetson 上查看日誌
journalctl -f | grep "移动完成"
# 應看到 "移动完成，悬停等待下一轮扫描"
```

**解決**：
- 確認 Git commit `85d5aff` 是否正確拉取
- 重新編譯

### 問題 3：RViz2 看不到無人機

**檢查**：
- 確認 Jetson 端發布了 `/drone_N/fmu/out/vehicle_local_position`
- 檢查 RViz 的 Fixed Frame 設置（應為 `map`）
- 確認 `multi_drone_visualizer.py` 正常運行

---

## 📖 延伸閱讀

- [ROS2 官方文檔](https://docs.ros.org/en/humble/)
- [PX4 開發指南](https://docs.px4.io/main/en/)
- [Meshtastic Python API](https://meshtastic.org/docs/software/python/cli/)

---

## 👥 貢獻者

- **開發者**: JackieLonger
- **測試團隊**: [待補充]

---

## 📄 License

MIT License - 詳見 [LICENSE](LICENSE) 文件

---

**最後更新**: 2025-11-23  
**版本**: v1.0 (Dynamic Tracker ID)
