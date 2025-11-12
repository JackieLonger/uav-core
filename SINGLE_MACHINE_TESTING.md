# 單機測試指南

驗證信號優化邏輯：**兩個 Tracker 訊號都改進 → 移動**

---

## 啟動 (3 個終端)

### 終端 1: Fast Scan
```bash
cd /home/landis/uav-core && source install/setup.bash
ros2 run ros2_px4_offboard_example fast_scan_node.py
```

**看到這個**:
```
[INFO] [fast_scan_node]: Tracker 1: Scanning...
[INFO] [fast_scan_node]: Tracker 2: Scanning...
```

### 終端 2: Signal Optimizer
```bash
cd /home/landis/uav-core && source install/setup.bash
ros2 run ros2_px4_offboard_example signal_optimizer_node_v3.py
```

**看到這個**:
```
[INFO] [signal_optimizer_node]: Node started
[INFO] [signal_optimizer_node]: ⏳ Waiting for RC manual ARM
```

### 終端 3: 監控訊號
```bash
source /home/landis/uav-core/install/setup.bash
ros2 topic echo /meshtastic/signal_report
```

**看到這個** (每 45 秒重複):
```
data:
  return_snr: 8.5
  return_rssi: -95
  status: 'Success'
  target_id: 'tracker1'
---
data:
  return_snr: 9.2
  return_rssi: -92
  status: 'Success'
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

