# 🚁 多無人機信號優化系統 - 啟動指南

## 📁 腳本文件總覽

### 🖥️ 筆電端腳本

| 腳本文件 | 功能 | 使用場景 |
|---------|------|---------|
| `launch_laptop.sh` | 筆電端統一啟動腳本（4 種模式） | **實際飛行時使用** |
| `test_rviz.sh` | RViz2 測試腳本 | **測試視覺化功能，不需要硬體** |

### 🤖 Jetson 端腳本

| 腳本文件 | 功能 | 使用場景 |
|---------|------|---------|
| `launch_jetson_simple.sh` | Jetson 端統一啟動腳本 | **實際飛行時使用** |

---

## 🚀 快速開始

### 場景 1：測試 RViz2 視覺化（不需要硬體）

**目的**：測試軌跡顯示、信號記錄等視覺化功能

```bash
# 在筆電上執行
cd ~/uav-core
./test_rviz.sh
```

**期望結果**：
- ✅ 啟動 Visualizer 和 RViz2
- ✅ 模擬 Drone 1 飛行 20 個點
- ✅ 顯示橘紅色軌跡線
- ✅ 無人機球體顏色隨信號變化
- ✅ 青色邊界框顯示正常

---

### 場景 2：實際飛行測試

#### 步驟 1：在每台 Jetson 上設定 Tracker ID

編輯 `src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`：

```python
# 第 34-41 行
TRACKER_A_ID = "!e2e5b7c4"  # 👈 修改為實際的 Tracker ID
TRACKER_B_ID = "!e2e5b8f8"  # 👈 修改為實際的 Tracker ID
```

#### 步驟 2：在每台 Jetson 上啟動

```bash
# Jetson 1 (Drone 1)
cd ~/uav-core
./launch_jetson_simple.sh 1

# Jetson 2 (Drone 2)
cd ~/uav-core
./launch_jetson_simple.sh 2

# Jetson 3 (Drone 3)
cd ~/uav-core
./launch_jetson_simple.sh 3
```

#### 步驟 3：在筆電上啟動優化器

```bash
# 筆電端
cd ~/uav-core
./launch_laptop.sh

# 選擇選項 3：同時啟動優化器和視覺化
```

**期望結果**：
- ✅ Jetson 掃描到 Meshtastic Tracker 信號
- ✅ 筆電優化器接收信號並計算速度
- ✅ 無人機根據信號梯度移動
- ✅ RViz2 顯示實時位置和軌跡
- ✅ 關閉後自動保存信號歷史 CSV 檔案

---

## 📋 詳細使用說明

### `launch_laptop.sh` - 筆電端啟動器

**功能選項**：

1. **選項 1**：僅啟動優化器
   - 訂閱 3 個 `/drone_X/link_quality` 話題
   - 發布 3 個 `/drone_X/offboard_velocity_cmd` 話題
   - 多線程處理（2 線程/無人機）

2. **選項 2**：僅啟動視覺化 + RViz2
   - RViz2 顯示無人機軌跡（Path 顯示）
   - MarkerArray 顯示無人機狀態（球體、標籤、邊界框）
   - 實時更新位置和速度
   - 記錄信號歷史數據（CSV 格式）

3. **選項 3**：同時啟動優化器和視覺化（推薦）
   - 完整功能
   - 關閉 RViz2 時自動清理所有進程

4. **選項 4**：僅啟動 RViz2
   - 用於查看已運行的系統狀態
   - 不啟動 visualizer，僅顯示界面

**使用範例**：

```bash
./launch_laptop.sh
# 選擇選項 3
# 等待系統啟動（約 5-10 秒）
# 查看 RViz2 視覺化和終端輸出
# 關閉 RViz2 即可結束所有進程
```

---

### `launch_jetson_simple.sh` - Jetson 端啟動器

**功能**：
- 啟動 `fast_scan_node`（掃描 Meshtastic Tracker）
- 啟動 `velocity_control`（接收速度命令，控制 PX4）

**使用範例**：

```bash
# 在 Drone 1 的 Jetson 上
./launch_jetson_simple.sh 1

# 在 Drone 2 的 Jetson 上
./launch_jetson_simple.sh 2

# 在 Drone 3 的 Jetson 上
./launch_jetson_simple.sh 3
```

**注意事項**：
- ⚠️ 執行前必須修改 `fast_scan_node.py` 中的 Tracker ID
- ⚠️ 確保 PX4 已連接（UART/Serial）
- ⚠️ 確保與筆電在同一網路（ROS_DOMAIN_ID 相同）
- ⚠️ 確保 Meshtastic Tracker 已通過 USB 連接

---

### `test_rviz.sh` - RViz2 視覺化測試

**功能**：
- 測試軌跡顯示功能（Path）
- 測試信號記錄功能（CSV）
- 測試坐標轉換（NED → ENU）
- 不需要真實硬體

**測試流程**：

1. 啟動 Visualizer（背景）
2. 啟動 RViz2（前景）
3. 模擬 Drone 1 飛行 20 個點
4. 每 0.5 秒發布一次位置和信號
5. 按 Enter 關閉

**觀察重點**：

```
RViz2 應該顯示：
- 橘紅色軌跡線（Path）
- 無人機球體（顏色隨信號變化）
- 青色邊界框（3m × 3m × 3m）
- 信號質量標籤
```

**信號歷史記錄**：
- 關閉時自動保存 CSV
- 檔名：`drone_1_signal_history_YYYYMMDD_HHMMSS.csv`
- 包含 1000 點歷史數據

---

## 🔧 故障排除

### 問題 1：筆電收不到 `/drone_X/link_quality`
./test_dynamic_signals.sh <drone_id>

# 範例
./test_dynamic_signals.sh 1  # 模擬 Drone 1
./test_dynamic_signals.sh 2  # 模擬 Drone 2
```

**參數說明**：
- `drone_id`：無人機 ID（1、2 或 3）
- 發布話題：`/drone_<id>/link_quality`
- 信號格式：JSON 字串（target_id、RSSI、SNR、status）

**信號變化規律**：
```python
Tracker A (循環 i = 1..20):
  RSSI = -100 + i * 2        # -98, -96, ..., -60
  SNR  = 5 + i / 2           # 5, 6, ..., 15

Tracker B:
  RSSI = -75 + (i % 3) - 1   # -76, -75, -74 (波動)
  SNR  = 10 + (i % 3) - 1    # 9, 10, 11 (波動)
```

---

## 🔧 故障排除

### 問題 1：RViz2 沒有顯示無人機

**解決方法**：

1. 檢查 RViz 配置文件是否存在：
   ```bash
   ls src/ROS2_PX4_Offboard_Example/config/multi_drone.rviz
   ```

2. 手動添加顯示項目：
   - 點擊 "Add" → "By topic"
   - 添加 `/drone_1/path`、`/drone_2/path`、`/drone_3/path`
   - 添加 `/multi_drone_markers`

### 問題 2：優化器收不到信號

**檢查清單**：

```bash
# 1. 確認話題是否發布
ros2 topic list | grep link_quality

# 2. 查看信號內容
ros2 topic echo /drone_1/link_quality

# 3. 檢查 ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # 應該相同

# 4. 檢查網路連接
ping <jetson_ip>
```

### 問題 3：Jetson 掃描不到 Tracker

**檢查清單**：

1. Tracker ID 是否正確設定在 `fast_scan_node.py`
2. Meshtastic 設備是否已連接（USB）
3. 執行測試指令：
   ```bash
   python3 -c "import meshtastic; print(meshtastic.__version__)"
   ```

### 問題 4：無人機不移動

**可能原因**：

1. **信號品質相同**：無梯度 → 速度為 0
   - 解決：移動無人機使信號產生變化

2. **RC Override 未關閉**：遙控器優先權較高
   - 解決：關閉遙控器或切換到 Offboard 模式

3. **PX4 未 arm**：無人機未解鎖
   - 解決：使用 QGroundControl 解鎖

---

## 📊 系統架構圖

```
┌─────────────────────────────────────────────────────────────┐
│                        筆電端 (Laptop)                       │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────┐   │
│  │  multi_drone_signal_optimizer.py                     │   │
│  │  - 訂閱: /drone_1/link_quality                       │   │
│  │  - 訂閱: /drone_2/link_quality                       │   │
│  │  - 訂閱: /drone_3/link_quality                       │   │
│  │  - 發布: /drone_1/offboard_velocity_cmd             │   │
│  │  - 發布: /drone_2/offboard_velocity_cmd             │   │
│  │  - 發布: /drone_3/offboard_velocity_cmd             │   │
│  │  - 多線程: 2 線程/無人機 (決策 + 發布@100Hz)         │   │
│  └──────────────────────────────────────────────────────┘   │
│                              │                               │
│  ┌──────────────────────────▼───────────────────────────┐   │
│  │  multi_drone_visualizer.py                           │   │
│  │  - RViz2 Marker 發布                                 │   │
│  │  - Path 軌跡顯示                                     │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                      │ WiFi 網路 / ROS2 Topic
         ┌────────────────────┼────────────────────┐
         │                    │                    │
┌────────▼────────┐  ┌────────▼────────┐  ┌────────▼────────┐
│   Jetson 1      │  │   Jetson 2      │  │   Jetson 3      │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│ fast_scan_node  │  │ fast_scan_node  │  │ fast_scan_node  │
│ - 掃描 Tracker  │  │ - 掃描 Tracker  │  │ - 掃描 Tracker  │
│ - 發布信號品質  │  │ - 發布信號品質  │  │ - 發布信號品質  │
├─────────────────┤  ├─────────────────┤  ├─────────────────┤
│velocity_control │  │velocity_control │  │velocity_control │
│ - 接收速度命令  │  │ - 接收速度命令  │  │ - 接收速度命令  │
│ - 控制 PX4      │  │ - 控制 PX4      │  │ - 控制 PX4      │
└────────┬────────┘  └────────┬────────┘  └────────┬────────┘
         │                    │                    │
    ┌────▼────┐          ┌────▼────┐          ┌────▼────┐
    │  PX4 1  │          │  PX4 2  │          │  PX4 3  │
    └─────────┘          └─────────┘          └─────────┘
```

---

## 📝 配置清單

### Jetson 端配置

**必須修改的文件**：
- `src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`
  - 第 34 行：`TRACKER_A_ID`
  - 第 35 行：`TRACKER_B_ID`

**網路配置**：
```bash
# 在 ~/.bashrc 添加
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

### 筆電端配置

**網路配置**：
```bash
# 在 ~/.bashrc 添加
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

**防火牆設定**（Ubuntu）：
```bash
sudo ufw allow from 192.168.0.0/24
```

---

## 🎯 測試檢查清單

### RViz2 視覺化測試（不需硬體）

- [ ] 執行 `./test_rviz.sh`
- [ ] RViz2 成功啟動
- [ ] 看到橘紅色軌跡線
- [ ] 無人機球體顏色變化
- [ ] 青色邊界框顯示正常
- [ ] 關閉後產生 CSV 檔案

### 實際飛行測試

- [ ] 所有 Jetson 已設定 Tracker ID
- [ ] 所有設備在同一網路
- [ ] ROS_DOMAIN_ID 相同
- [ ] PX4 已連接並解鎖
- [ ] 遙控器隨時可接管
- [ ] 筆電優化器正常運行
- [ ] Jetson 掃描到 Tracker 信號
- [ ] 無人機響應速度命令
- [ ] RViz2 實時顯示軌跡

---

## 📞 技術支援

如有問題，請檢查：

1. **日誌文件**：`log/latest/`
2. **ROS2 話題**：`ros2 topic list`
3. **節點狀態**：`ros2 node list`
4. **參考文檔**：
   - [BOUNDARY_LIMIT_EXPLANATION.md](BOUNDARY_LIMIT_EXPLANATION.md) - 邊界限制說明
   - [ROS2_TOPIC_FLOW.md](ROS2_TOPIC_FLOW.md) - Topic 架構圖
   - [README.md](README.md) - 項目總覽
4. **網路連接**：`ros2 doctor`

---

**最後更新**：2025-11-23
**版本**：v1.0 - 動態 Tracker ID 版本
