# 🎯 UAV 多無人機訊號優化系統

> 多無人機協同 LoRa 訊號優化系統 - 基於 ROS2 + PX4 的自主訊號搜尋與定位

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.14.3-green)](https://docs.px4.io/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-orange)](LICENSE)

---

## 📚 核心文檔

| 文檔 | 用途 | 閱讀時間 |
|------|------|---------|
| **README.md** | 📖 項目總覽與快速入門 | 10 分鐘 |
| **LAUNCH_SCRIPTS_GUIDE.md** | � 啟動腳本完整指南（實際部署必讀） | 15 分鐘 |
| **BOUNDARY_LIMIT_EXPLANATION.md** | 🛡️ 邊界限制與安全機制說明 | 10 分鐘 |
| **ROS2_TOPIC_FLOW.md** | 📡 ROS2 Topic 訂閱關係與系統架構 | 10 分鐘 |

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
         ↕ (WiFi 網路 / ROS2 Topic 通訊)
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

## ⚡ 快速啟動

### 📱 Jetson 端（無人機機載）

**前置準備**：修改 `fast_scan_node.py` 中的 Tracker ID
```bash
# 編輯：src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py
# 第 34-41 行配置區域，填入實際的 Tracker ID

# 編譯並啟動
cd ~/uav-core
colcon build --merge-install
source install/setup.bash
./launch_jetson_simple.sh 1  # Drone 1
```

### 💻 筆電端（地面站）

```bash
# 啟動優化器 + 視覺化 + RViz2
cd ~/uav-core
source install/setup.bash
./launch_laptop.sh

# 選擇選項 3：同時啟動優化器和視覺化（推薦）
```

**純 RViz2 測試**：
```bash
# 測試軌跡顯示功能（不需要真實硬體）
./test_rviz.sh
```

---

## 🎨 RViz2 視覺化

啟動後可在 RViz2 中看到：
- 🔴🟢🔵 **無人機球體**：顏色根據信號質量變化（紅→黃→綠）
- 🟧🟩🟦 **飛行軌跡**：Path 顯示每架無人機的移動軌跡
- 📦 **邊界框**：青色框線顯示每架無人機的飛行範圍（±1.5m × ±1.5m × 0-3m）
- 🏷️ **信號數據**：每架無人機顯示實時 RSSI、SNR 和質量分數

**信號歷史記錄**：
- 關閉視覺化器時自動保存 CSV 檔案
- 檔案名：`drone_N_signal_history_YYYYMMDD_HHMMSS.csv`
- 包含：時間戳、訊號質量、RSSI、SNR、位置等數據

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
| `fast_scan_node.py` | 掃描 Meshtastic Tracker 信號 | Jetson |
| `velocity_control.py` | 速度控制（Twist → PX4 軌跡）| Jetson |
| `multi_drone_signal_optimizer.py` | 多機優化決策中心 | 筆電 |
| `multi_drone_visualizer.py` | RViz2 可視化 + 信號記錄 | 筆電 |

### 啟動腳本

| 腳本 | 功能 | 使用端 |
|------|------|--------|
| `launch_laptop.sh` | 筆電端統一啟動器（4 種模式） | 筆電 |
| `launch_jetson_simple.sh` | Jetson 端統一啟動器 | Jetson |
| `test_rviz.sh` | RViz2 測試腳本（無需硬體） | 筆電 |

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

**原因與解決**：
- Jetson 未啟動或網路未連接 → 檢查 WiFi 同網段
- Tracker 未綁定或 ID 錯誤 → 修改 `fast_scan_node.py` 的 ID
- ROS2 網路問題 → 確認 `ROS_DOMAIN_ID` 一致（預設為 0）

### 問題 2：RViz2 看不到無人機或軌跡

**檢查**：
```bash
# 確認 topic 是否發布
ros2 topic list | grep -E "vehicle_local_position|path"
# 應看到：/drone_N/fmu/out/vehicle_local_position 和 /drone_N/path

ros2 topic hz /drone_1/path
# 應顯示更新頻率
```

**解決**：
- 確保 `multi_drone_visualizer.py` 正在運行
- 檢查 RViz2 配置檔案：`multi_drone.rviz`
- 確認座標系 Fixed Frame 設為 `map`

### 問題 3：無人機不懸停，持續移動

**立即處理**：用遙控器切回手動模式！

**原因**：
- `velocity_control.py` 沒有收到速度命令時會保持懸停
- 如果持續移動，可能是優化器異常

**檢查**：
```bash
# 在筆電上查看優化器狀態
ros2 topic echo /drone_1/offboard_velocity_cmd
# 懸停時應全為 0：linear: {x: 0.0, y: 0.0, z: 0.0}
```

---

## 🛡️ 安全注意事項

### 飛行前檢查
- ✅ 確認所有 Jetson 端 Tracker ID 已正確配置
- ✅ 檢查 ROS2 網路連接（筆電能 ping 通所有 Jetson）
- ✅ 確認遙控器電量充足，可隨時接管
- ✅ 測試 RC Override 功能正常
- ✅ 檢查邊界設定符合飛行場地

### 飛行中監控
- 📊 實時觀察 RViz2 中的無人機位置和信號
- 🎮 遙控器隨時準備切換到手動模式
- 📡 監控 `/drone_N/offboard_velocity_cmd` topic
- 🔋 注意電池電量和飛行時間

### 緊急處理
1. **立即切回手動模式**（最高優先）
2. 手動降落到安全位置
3. 檢查日誌找出問題原因
4. 修正後重新測試

---

## 📖 延伸閱讀

- [LAUNCH_SCRIPTS_GUIDE.md](LAUNCH_SCRIPTS_GUIDE.md) - 啟動腳本詳細說明
- [BOUNDARY_LIMIT_EXPLANATION.md](BOUNDARY_LIMIT_EXPLANATION.md) - 邊界限制機制
- [ROS2_TOPIC_FLOW.md](ROS2_TOPIC_FLOW.md) - ROS2 Topic 架構
- [ROS2 官方文檔](https://docs.ros.org/en/humble/)
- [PX4 開發指南](https://docs.px4.io/main/en/)
- [Meshtastic Python API](https://meshtastic.org/docs/software/python/cli/)

---

## � 授權

本項目採用 MIT 授權條款。

---

## 📄 License

MIT License - 詳見 [LICENSE](LICENSE) 文件

---

**最後更新**: 2025-11-23  
**版本**: v1.0 (Dynamic Tracker ID)
