# 單機測試指南

**目的**：驗證單架無人機的訊號優化功能

---

## 🎯 測試目標

1. ✅ 驗證 Jetson 上的 `fast_scan_node` 正常掃描綁定的 Tracker
2. ✅ 驗證地面站 `multi_drone_signal_optimizer` 接收訊號並計算
3. ✅ 驗證速度指令生成並發布到正確的 topic
4. ✅ 驗證 RViz2 可視化顯示

---

## 🛠️ 測試環境

### 硬體需求
- 1 台無人機（含 Pixhawk 6C + Jetson Orin Nano）
- 2 個 Heltec Tracker V3（Meshtastic）
- 1 台筆記本（地面站）
- WiFi 路由器（確保 Jetson 和筆記本在同一網段）

### 軟體準備
```bash
# 1. 確認已編譯
cd ~/uav-core
colcon build --merge-install

# 2. 確認環境變量
source install/setup.bash
```

---

## 📋 測試步驟

### 步驟 1: Jetson 端啟動

在 Jetson 上執行：

```bash
cd ~/uav-core
source install/setup.bash

# 啟動 Jetson 節點（替換為實際的 Tracker ID）
./launch_jetson.sh 1 !tracker1A !tracker1B
```

**預期輸出**：
```
========================================
啟動 Jetson 機載節點 - Drone 1
Tracker A: !tracker1A
Tracker B: !tracker1B
========================================
[INFO] [fast_scan_node]: 🚀 Fast scan node started for drone_1
[INFO] [fast_scan_node]: 📡 Bound trackers: !tracker1A, !tracker1B
[INFO] [velocity_control]: Velocity control node started for drone_1
```

**驗證**：
```bash
# 檢查 topic 是否發布
ros2 topic list | grep drone_1

# 應該看到：
# /drone_1/link_quality
# /drone_1/offboard_velocity_cmd
```

---

### 步驟 2: 地面站啟動優化器

在筆記本上執行：

```bash
cd ~/uav-core
source install/setup.bash

# 啟動優化器（單機模式）
ros2 run px4_offboard multi_drone_signal_optimizer \
    --ros-args \
    -p num_drones:=1 \
    -p drone_ids:="[1]"
```

**預期輸出**：
```
[INFO] [multi_drone_signal_optimizer]: 🚀 Multi-drone optimizer started
[INFO] [multi_drone_signal_optimizer]: 📊 Managing 1 drone(s): [1]
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 1: 決策線程啟動
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 1: 速度發布線程啟動 (100Hz)
```

**驗證**：
```bash
# 檢查訂閱關係
ros2 topic info /drone_1/link_quality

# 應該看到：
# Publisher count: 1 (fast_scan_node)
# Subscription count: 1 (multi_drone_signal_optimizer)
```

---

### 步驟 3: 啟動 RViz2 可視化

在筆記本上開新終端：

```bash
cd ~/uav-core
source install/setup.bash

# 啟動可視化
./launch_rviz.sh
```

**預期看到**：
- 🟢/🟡/🔴 無人機球體（根據訊號質量變色）
- 📝 文本標籤顯示：
  - `Drone 1`
  - `Q: 0.XX`
  - `TrackerA: !tracker1A`
  - `TrackerB: !tracker1B`
- 🟦 青色邊界框（3m × 3m × 3m）
- 🔵 藍色飛行軌跡（移動後出現）

---

### 步驟 4: 監控訊號接收

在筆記本上開新終端：

```bash
source ~/uav-core/install/setup.bash

# 監控訊號數據
ros2 topic echo /drone_1/link_quality
```

**預期輸出**（每 2 秒更新）：
```json
{
  "target_id": "!tracker1A",
  "forward_rssi": -85,
  "forward_snr": 9.5,
  "return_rssi": -88,
  "return_snr": 8.2
}
```

---

### 步驟 5: 手動起飛並切換 Offboard

**使用遙控器**：
1. 解鎖（ARM）無人機
2. 手動起飛到安全高度（建議 3-5m）
3. 切換到 Offboard 模式

**優化器日誌應顯示**：
```
[INFO] [multi_drone_signal_optimizer]: 📍 Drone 1 進入 Offboard 模式
[INFO] [multi_drone_signal_optimizer]: 📍 記錄起飛位置: (0.0, 0.0, 3.2)
[INFO] [multi_drone_signal_optimizer]: 🎯 Drone 1 搜索邊界設定完成
```

---

### 步驟 6: 觀察自動優化

系統會自動執行以下流程：

```
1. 等待兩個 Tracker 訊號都到達
   ├─ TrackerA 訊號 ✓
   ├─ TrackerB 訊號 ✓
   └─ 計算平均質量分數

2. 決策移動方向
   ├─ 優先上升到 1.5m
   ├─ 訊號改善 → 繼續當前方向
   └─ 訊號變差 → XY 隨機搜索

3. 發布速度指令（100Hz）
   └─ /drone_1/offboard_velocity_cmd

4. 移動並記錄
   ├─ 移動計數 +1
   └─ 更新最佳位置

5. 重複 3-5 次後停止
   └─ 懸停在最佳位置
```

**監控速度指令**：
```bash
ros2 topic echo /drone_1/offboard_velocity_cmd
```

**預期輸出**：
```
linear:
  x: 0.2    # vx (NED)
  y: -0.15  # vy (NED)
  z: -0.3   # vz (NED, 負值=上升)
angular:
  z: 0.0    # yaw_rate
```

---

## ✅ 成功標誌

### Jetson 端
```
[INFO] [fast_scan_node]: 📡 掃描 !tracker1A... RSSI=-85 SNR=9.5
[INFO] [fast_scan_node]: 📡 掃描 !tracker1B... RSSI=-88 SNR=8.2
[INFO] [fast_scan_node]: ✅ 發布訊號質量到 /drone_1/link_quality
```

### 地面站端
```
[INFO] [multi_drone_signal_optimizer]: 📊 Drone 1 質量分數: 0.75
[INFO] [multi_drone_signal_optimizer]: ⬆️ Drone 1 優先上升 (當前高度: 1.2m)
[INFO] [multi_drone_signal_optimizer]: ✅ Drone 1 移動 #1 完成
[INFO] [multi_drone_signal_optimizer]: 🎯 Drone 1 已收斂，懸停在最佳位置
```

### RViz2
- 無人機球體從紅色 → 黃色 → 綠色（訊號改善）
- 藍色軌跡顯示移動路徑
- 速度箭頭指向移動方向

---

## 🧪 驗證檢查清單

- [ ] Jetson 上 `fast_scan_node` 成功啟動
- [ ] 地面站 `multi_drone_signal_optimizer` 成功啟動
- [ ] `/drone_1/link_quality` topic 有數據
- [ ] `/drone_1/offboard_velocity_cmd` topic 有速度指令
- [ ] RViz2 顯示無人機位置和訊號質量
- [ ] 切換 Offboard 後記錄起飛位置
- [ ] 無人機開始自動移動搜索
- [ ] 訊號質量分數逐漸提高
- [ ] 達到移動次數限制後懸停
- [ ] 無 ERROR 日誌

---

## 🐛 故障排查

### 問題 1: `/drone_1/link_quality` 沒有數據

**檢查**：
```bash
# 在 Jetson 上檢查 Meshtastic 連接
meshtastic --info

# 檢查 fast_scan_node 是否運行
ps aux | grep fast_scan

# 檢查 topic 是否存在
ros2 topic list | grep link_quality
```

**解決**：
- 確認 Meshtastic USB 連接正常
- 確認 Tracker ID 正確（使用 `meshtastic --nodes` 查看）
- 重啟 `fast_scan_node`

---

### 問題 2: 優化器沒有發布速度指令

**檢查**：
```bash
# 檢查是否進入 Offboard 模式
ros2 topic echo /fmu/out/vehicle_status --once | grep nav_state
# nav_state: 14 = Offboard

# 檢查位置數據是否接收
ros2 topic echo /drone_1/fmu/out/vehicle_local_position --once
```

**解決**：
- 確認無人機已進入 Offboard 模式
- 檢查 PX4-ROS2 橋接是否正常（MicroXRCE Agent）
- 查看優化器日誌確認決策線程啟動

---

### 問題 3: RViz2 看不到無人機

**檢查**：
```bash
# 檢查 visualizer 是否運行
ps aux | grep multi_drone_visualizer

# 檢查 MarkerArray topic
ros2 topic echo /visualization_marker_array --once
```

**解決**：
- 確認 `./launch_rviz.sh` 成功執行
- 檢查 RViz2 左側 Displays 面板是否勾選 `MarkerArray`
- 確認 Fixed Frame 設為 `map`

---

### 問題 4: 無人機超出邊界

**檢查日誌**：
```
[WARN] [multi_drone_signal_optimizer]: ⚠️ Drone 1 超出 X 邊界
```

**原因**：
- 起飛位置記錄錯誤
- 風力干擾導致漂移
- 速度指令過大

**解決**：
- 降低 `MAX_VELOCITY`（修改代碼）
- 在無風環境測試
- 檢查 PX4 EKF 質量

---

## 📊 測試數據記錄

建議記錄以下數據：

| 時間 | 質量分數 | 位置 (X, Y, Z) | 速度 (vx, vy, vz) | 備註 |
|------|---------|---------------|------------------|------|
| t=0s | 0.45 | (0.0, 0.0, 3.2) | (0, 0, 0) | 起飛位置 |
| t=10s | 0.52 | (0.2, -0.1, 3.5) | (0.2, -0.1, -0.3) | 移動 #1 |
| t=20s | 0.68 | (0.5, -0.3, 4.0) | (0.15, -0.1, -0.2) | 移動 #2 |
| ... | ... | ... | ... | ... |
| t=60s | 0.82 | (1.2, -0.8, 4.5) | (0, 0, 0) | 收斂懸停 |

---

## 📝 完成報告

測試完成後填寫：

```
測試日期: ___________
測試地點: ___________
無人機 ID: 1
綁定 Tracker: !tracker1A, !tracker1B

起飛位置: (__, __, __)
最終位置: (__, __, __)
最佳質量分數: ____

移動次數: ____ 次
測試時長: ____ 分鐘

結果: ☐ 通過 / ☐ 失敗

備註:
_________________________
_________________________
```

---

## 🔄 多機測試

單機測試通過後，可進行多機測試：

參考 **[MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md)** 中的完整多機啟動流程。

主要差異：
- Jetson：每台獨立啟動（drone_id = 1, 2, 3）
- 地面站：`num_drones:=3 drone_ids:="[1, 2, 3]"`
- RViz2：同時顯示 3 架無人機

---

**版本**: v2.0  
**更新**: 2025-11-12  
**適用**: Multi-Drone Signal Optimizer (final_test_1 分支)
  target_id: 'tracker2'
```

---

## 時間線 (約 120 秒等待)

```
t=45s:   Tracker 1 訊號到達
         Signal Optimizer 收到 → 等待 Tracker 2

t=120s:  Tracker 2 訊號到達
         Signal Optimizer 計算平均評分 → 建立基準 (無移動)

         📊 信號評分:
         Tracker 1: SNR=8.5, RSSI=-95
         Tracker 2: SNR=9.2, RSSI=-92
         
         計算:
         score1 = 8.5 + (-95/-50) × 100 × 0.5 = 8.5 + 95 = 103.5
         score2 = 9.2 + (-92/-50) × 100 × 0.5 = 9.2 + 92 = 101.2
         avg = (103.5 + 101.2) / 2 = 102.35 ✓ (基準)

t=165s:  Tracker 1 訊號到達 (第2週期)

t=240s:  Tracker 2 訊號到達 (第2週期)
         檢查改進:
         
         場景 A: 兩個都改進 ✅
         score1_new = 110.0 (改進)
         score2_new = 108.0 (改進)
         avg_new = 109.0 > 102.35 → ✅ 觸發移動 #1
         
         場景 B: 只有一個改進 ❌
         score1_new = 110.0 (改進)
         score2_new = 95.0 (惡化)
         avg_new = 102.5 ≈ 102.35 → ❌ 不移動

t=285s:  Tracker 1 訊號 (第3週期)
t=360s:  Tracker 2 訊號 (第3週期)
         → 檢查改進 → 可能移動 #2

... 重複直到:
- 10 次移動 → HOLD
- 5 次無改進 → HOLD
```

---

## 檢查日誌

### 終端 2 看到的內容

**第1個訊號** (t≈120s):
```
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker1 | SNR=8.5 RSSI=-95 Score=103.5
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker2 | SNR=9.2 RSSI=-92 Score=101.2
[DEBUG] [signal_optimizer_node]: 📊 Tracker Scores: avg=102.35
[INFO] [signal_optimizer_node]: 📍 基準評分已建立 (baseline=102.35)
```

**第2個訊號 - 兩個都改進** (t≈240s):
```
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker1 | SNR=9.5 RSSI=-88 Score=110.0
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker2 | SNR=9.8 RSSI=-86 Score=108.0
[DEBUG] [signal_optimizer_node]: 📊 Tracker Scores: avg=109.0
[INFO] [signal_optimizer_node]: ✅ 移動有效 #1 (平均分=109.0, 改進=6.65)
```

**第3個訊號 - 只有一個改進** (t≈360s):
```
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker1 | SNR=10.0 RSSI=-85 Score=115.0
[DEBUG] [signal_optimizer_node]: 📡 Tracker tracker2 | SNR=8.0 RSSI=-100 Score=92.0
[DEBUG] [signal_optimizer_node]: 📊 Tracker Scores: avg=103.5
[DEBUG] [signal_optimizer_node]: ⚠️ 無改進 #1: 當前平均=103.5, 最佳平均=109.0, 改進=-5.5
```

---

## 進入 Offboard 模式

執行此命令模擬進入 Offboard:
```bash
ros2 topic pub /fmu/out/vehicle_status px4_msgs/msg/VehicleStatus \
  '{arming_state: 2, nav_state: 14, pre_flight_checks_pass: true}' --once
```

**終端 2 看到**:
```
[INFO] [signal_optimizer_node]: ✅ Offboard mode activated by operator
  Takeoff position: {'x': 0.0, 'y': 0.0, 'z': 3.0}
```

---

## 監控位置

### 終端 4: 監控生成位置
```bash
source /home/landis/uav-core/install/setup.bash
ros2 topic echo /drone1/target_position
```

**看到的內容**:
```
pose:
  position:
    x: 0.82
    y: -0.45
    z: 3.0
```

**驗證在範圍內**:
```
起飛點: (0.0, 0.0)
位置: (0.82, -0.45)
距離: dx=0.82 < 1.5 ✓, dy=0.45 < 1.5 ✓ 
在 ±1.5m 範圍內！
```

---

## 完整場景流程

```
流程 1: 基準建立 (NO MOVE)
├─ 訊號1到達: tracker1=8.5, tracker2=9.2
├─ 平均評分: 102.35
├─ 日誌: "📍 基準評分已建立"
└─ 位置: 無生成

流程 2: 改進 - 移動 #1 (MOVE)
├─ 訊號2到達: tracker1=9.5, tracker2=9.8
├─ 新平均評分: 109.0
├─ 改進: 109.0 - 102.35 = 6.65 > 0.5 ✓
├─ 日誌: "✅ 移動有效 #1"
└─ 位置: 生成 (0.82, -0.45, 3.0)

流程 3: 只有一個改進 - 不移動
├─ 訊號3到達: tracker1=10.0, tracker2=8.0
├─ 新平均評分: 103.5
├─ 改進: 103.5 - 109.0 = -5.5 < 0.5 ✗
├─ 日誌: "⚠️ 無改進 #1"
└─ 位置: 無生成

流程 4-N: 重複直到停止
├─ 重複流程 2-3
├─ movement_count: 1 → 10
└─ 10 次後進入 HOLD

流程 HOLD: 停止移動
├─ 日誌: "⏸️ HOLD 模式啟動"
├─ 位置: 停止生成
└─ 速度: (0, 0, 0) 每 100ms
```

---

## ✅ 測試通過條件

1. ✓ 看到兩個 Tracker 訊號 (tracker1, tracker2)
2. ✓ 評分計算正確 (103.5, 101.2, avg=102.35)
3. ✓ 基準建立，無移動
4. ✓ 兩個都改進 → "✅ 移動有效"
5. ✓ 只有一個改進 → "⚠️ 無改進"
6. ✓ 位置在 ±1.5m 範圍內
7. ✓ 10 次移動後 HOLD
8. ✓ HOLD 時發佈 (0, 0, 0)

---

## 🐛 故障排除

| 問題 | 解決 |
|-----|------|
| 沒收到訊號 | `ros2 topic echo /meshtastic/signal_report` 檢查 |
| 收不到 tracker2 | 檢查 enable_dual_tracker_wait 參數 |
| 位置超出範圍 | `ros2 param set /signal_optimizer_node search_bounds_x_min -1.5` |
| Offboard 進不去 | 發送 VehicleStatus (arming=2, nav=14) |
| 沒看到移動 | 等待足夠的訊號週期，查看改進值 |

---

## 📋 檢查清單

- [ ] 終端 1: Fast Scan 運行中
- [ ] 終端 2: Signal Optimizer 運行中
- [ ] 終端 3: 看到 tracker1 和 tracker2 訊號
- [ ] 看到 "基準評分已建立" 日誌
- [ ] 看到 "✅ 移動有效 #1" 日誌
- [ ] 位置在 ±1.5m 範圍
- [ ] 10 次移動後進入 HOLD
- [ ] HOLD 時發佈零速度
- [ ] 無 ERROR 日誌

**結論**: ☐ 通過 / ☐ 失敗

