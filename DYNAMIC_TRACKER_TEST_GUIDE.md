# 動態 Tracker ID 測試指南

## 📋 修改說明
`multi_drone_signal_optimizer.py` 已修改為**動態綁定模式**：
- 不再需要在筆電端硬編碼 Tracker ID
- 優化器會自動學習每架無人機回傳的前兩個有效 Tracker ID
- 只要 Jetson 端的 `fast_scan_node.py` 設定正確，筆電端就能自動適應

---

## 🧪 測試步驟

### 1. 準備工作
確保所有終端都已 source 環境：
```bash
cd /home/landis/uav-core
source install/setup.bash
```

### 2. 終端 A：模擬 Jetson 端 (無人機)
此終端模擬無人機上的節點運行。
**注意**：實際飛行時，這些指令是在 Jetson 上執行。

```bash
# 啟動模擬的 Jetson 節點 (Drone 1)
# 這裡我們手動發布 link_quality 來模擬 fast_scan_node 的行為
# 因為沒有真實的 LoRa 硬體

# 1. 啟動 velocity_control (接收速度指令)
ros2 run px4_offboard velocity_control --ros-args -r offboard_velocity_cmd:=/drone_1/offboard_velocity_cmd &

# 2. 模擬發布 Tracker A 訊號 (每 5 秒一次)
while true; do
  ros2 topic pub /drone_1/link_quality std_msgs/msg/String "data: '{\"target_id\": \"!TEST_A\", \"status\": \"Success\", \"forward_rssi\": -80, \"forward_snr\": 10, \"return_rssi\": -85, \"return_snr\": 8, \"timestamp\": \"123456\"}'" -1
  sleep 1
  ros2 topic pub /drone_1/link_quality std_msgs/msg/String "data: '{\"target_id\": \"!TEST_B\", \"status\": \"Success\", \"forward_rssi\": -75, \"forward_snr\": 12, \"return_rssi\": -78, \"return_snr\": 11, \"timestamp\": \"123456\"}'" -1
  sleep 4
done
```

### 3. 終端 B：筆電端 (地面站)
此終端運行優化器和視覺化工具。

```bash
# 1. 啟動優化器 (單機模式測試)
ros2 run px4_offboard multi_drone_signal_optimizer --ros-args -p num_drones:=1 -p drone_ids:="[1]"

# (另開分頁) 2. 啟動 RViz2
./launch_rviz.sh
```

### 4. 驗證重點
觀察終端 B 的輸出，應該看到：
1. `[INFO] ...: 收到 !TEST_A 信号 ...`
2. `[INFO] ...: 收到 !TEST_B 信号 ...`
3. `[INFO] ...: Drone 1: 移动 1/5, 速度 ...` (表示成功識別兩個 ID 並開始決策)
4. `[INFO] ...: Drone 1: 移动完成，悬停等待下一轮扫描`

---

## 🚁 實機飛行設置

### Jetson 端
只需確保 `fast_scan_node.py` 內的 ID 設定正確：
```python
# fast_scan_node.py
TRACKER_A_ID = "!實際ID_1"
TRACKER_B_ID = "!實際ID_2"
```

### 筆電端
無需任何修改！直接啟動：
```bash
./launch_multi_drone_optimizer.sh
```
它會自動偵測 Drone 1 回傳的是哪兩個 ID，Drone 2 回傳的是哪兩個 ID，以此類推。
