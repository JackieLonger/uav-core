# 🧪 雙分頁實際測試指南

## 📊 系統架構確認

### ✅ 完全支援異步多線程

**每台無人機有 2 個獨立線程**：
1. **Decision Thread**（決策線程）- 等待 Tracker 數據 → 計算速度 → 更新狀態
2. **Publish Thread**（發布線程）- 100Hz 持續發布當前速度指令

**筆電端總共運行**（以 3 台無人機為例）：
- 3 個 Decision Threads（獨立決策，互不阻塞）
- 3 個 Publish Threads（100Hz 發布）
- 1 個 MultiThreadedExecutor（處理 ROS2 回調）

**關鍵優勢**：
- ✅ Drone 1 和 Drone 2 可以**同時**移動，互不等待
- ✅ 每台無人機的 Tracker 數據**完全隔離**（通過 topic 命名空間）
- ✅ 每台無人機有**獨立的狀態鎖**（`DroneState.lock`）

---

## 🔍 如何確保不會混亂？

### Topic 隔離機制

```
Jetson 1 (192.168.0.150) 發布：
  /drone_1/link_quality → {"target_id": "!11111111", ...}
  /drone_1/link_quality → {"target_id": "!11111112", ...}

Jetson 2 (192.168.0.151) 發布：
  /drone_2/link_quality → {"target_id": "!22222221", ...}
  /drone_2/link_quality → {"target_id": "!22222222", ...}

Jetson 3 (192.168.0.152) 發布：
  /drone_3/link_quality → {"target_id": "!33333331", ...}
  /drone_3/link_quality → {"target_id": "!33333332", ...}
```

### 筆電端狀態隔離

```python
# 筆電端維護 3 個獨立的 DroneState
drone_states[1].dynamic_tracker_ids = ['!11111111', '!11111112']
drone_states[2].dynamic_tracker_ids = ['!22222221', '!22222222']
drone_states[3].dynamic_tracker_ids = ['!33333331', '!33333332']

# 每個 DroneState 有獨立的鎖，互不干擾
drone_states[1].lock  # 只保護 Drone 1 的數據
drone_states[2].lock  # 只保護 Drone 2 的數據
drone_states[3].lock  # 只保護 Drone 3 的數據
```

**結論：完全不會混亂！** ✅

---

## 🧪 測試步驟（雙分頁）

### 準備工作

**網路確認**：
- Jetson IP: 192.168.0.150
- 筆電 IP: 192.168.0.161
- 確保在同一網段（192.168.0.x）

**環境檢查**：
```bash
# 兩邊都執行
ping 192.168.0.150  # 在筆電測試
ping 192.168.0.161  # 在 Jetson 測試
```

---

## 測試 1: 單機模擬測試（無硬體）

### VS Code 分頁 1 (Jetson)

```bash
cd ~/uav-core
source install/setup.bash

# 模擬 Drone 1 發送訊號（循環發送）
while true; do
  echo "=== Drone 1 發送 Tracker A ==="
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!11111111\", \"status\": \"Success\", \"forward_rssi\": -80, \"forward_snr\": 10, \"return_rssi\": -85, \"return_snr\": 8, \"timestamp\": \"123456\"}'"
  
  sleep 1
  
  echo "=== Drone 1 發送 Tracker B ==="
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!11111112\", \"status\": \"Success\", \"forward_rssi\": -75, \"forward_snr\": 12, \"return_rssi\": -78, \"return_snr\": 11, \"timestamp\": \"123456\"}'"
  
  echo "=== 等待 3 秒 ==="
  sleep 3
done
```

### VS Code 分頁 2 (筆電)

**終端 1**：
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
[INFO] ...: 订阅 /drone_1/link_quality
[INFO] ...: 发布到 /drone_1/offboard_velocity_cmd
[INFO] ...: 启动 Drone 1 决策线程
[INFO] ...: Drone 1: 收到 !11111111 信号 (RSSI: -80.0, SNR: 10.0)
[INFO] ...: Drone 1: 收到 !11111112 信号 (RSSI: -75.0, SNR: 12.0)
[INFO] ...: Drone 1: 移动 1/5, 速度 (0.XX, 0.XX, -0.XX), 质量 0.XXX
[INFO] ...: Drone 1: 移动完成，悬停等待下一轮扫描  ← 關鍵！
```

**成功標準**：
- [x] 看到 "收到 !11111111 信号"
- [x] 看到 "收到 !11111112 信号"
- [x] 看到 "移动 1/5"
- [x] 看到 "移动完成，悬停等待下一轮扫描"

---

## 測試 2: 檢查動態 Tracker ID 識別

### 目標
驗證筆電端能自動學習 Jetson 發送的任意 Tracker ID（不需事先知道）

### Jetson 端（發送隨機 ID）

```bash
cd ~/uav-core
source install/setup.bash

# 發送完全不同的 Tracker ID（測試動態識別）
while true; do
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!RANDOM_X\", \"status\": \"Success\", \"forward_rssi\": -82, \"forward_snr\": 9, \"return_rssi\": -87, \"return_snr\": 7, \"timestamp\": \"123456\"}'"
  sleep 1
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!RANDOM_Y\", \"status\": \"Success\", \"forward_rssi\": -73, \"forward_snr\": 13, \"return_rssi\": -76, \"return_snr\": 12, \"timestamp\": \"123456\"}'"
  sleep 3
done
```

### 筆電端

```bash
# 同上啟動 optimizer
ros2 run px4_offboard multi_drone_signal_optimizer \
  --ros-args -p num_drones:=1 -p drone_ids:="[1]"
```

**預期輸出**：
```
[INFO] ...: Drone 1: 收到 !RANDOM_X 信号 ...  ← 自動識別！
[INFO] ...: Drone 1: 收到 !RANDOM_Y 信号 ...  ← 自動識別！
[INFO] ...: Drone 1: 移动 1/5 ...
```

**成功標準**：
- [x] 筆電端成功識別 `!RANDOM_X` 和 `!RANDOM_Y`
- [x] 沒有報錯 "unknown tracker"
- [x] 正常進入決策和移動

---

## 測試 3: 多機異步測試（模擬 2 台）

### Jetson 端（模擬 2 台無人機）

**終端 1**（模擬 Drone 1）：
```bash
while true; do
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!1A\", \"status\": \"Success\", \"forward_rssi\": -80, \"forward_snr\": 10, \"return_rssi\": -85, \"return_snr\": 8, \"timestamp\": \"123\"}'"
  sleep 1
  ros2 topic pub --once /drone_1/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!1B\", \"status\": \"Success\", \"forward_rssi\": -75, \"forward_snr\": 12, \"return_rssi\": -78, \"return_snr\": 11, \"timestamp\": \"123\"}'"
  sleep 3
done
```

**終端 2**（模擬 Drone 2）：
```bash
while true; do
  ros2 topic pub --once /drone_2/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!2A\", \"status\": \"Success\", \"forward_rssi\": -85, \"forward_snr\": 8, \"return_rssi\": -90, \"return_snr\": 6, \"timestamp\": \"456\"}'"
  sleep 1
  ros2 topic pub --once /drone_2/link_quality std_msgs/msg/String \
    "data: '{\"target_id\": \"!2B\", \"status\": \"Success\", \"forward_rssi\": -70, \"forward_snr\": 14, \"return_rssi\": -73, \"return_snr\": 13, \"timestamp\": \"456\"}'"
  sleep 3
done
```

### 筆電端（多機模式）

```bash
cd ~/uav-core
source install/setup.bash

# 啟動雙機優化器
ros2 run px4_offboard multi_drone_signal_optimizer \
  --ros-args \
  -p num_drones:=2 \
  -p drone_ids:="[1, 2]"
```

**預期輸出（交錯顯示）**：
```
[INFO] ...: Drone 1: 收到 !1A 信号 ...
[INFO] ...: Drone 2: 收到 !2A 信号 ...
[INFO] ...: Drone 1: 收到 !1B 信号 ...
[INFO] ...: Drone 2: 收到 !2B 信号 ...
[INFO] ...: Drone 1: 移动 1/5, 速度 ...  ← 獨立決策
[INFO] ...: Drone 2: 移动 1/5, 速度 ...  ← 同時進行！
[INFO] ...: Drone 1: 移动完成，悬停等待下一轮扫描
[INFO] ...: Drone 2: 移动完成，悬停等待下一轮扫描
```

**成功標準**：
- [x] 兩台無人機的日誌交錯顯示（證明異步）
- [x] `!1A/!1B` 歸給 Drone 1，`!2A/!2B` 歸給 Drone 2
- [x] 沒有混亂或錯誤

---

## 測試 4: RViz2 可視化

### 筆電端

**終端 1**（優化器）：
```bash
cd ~/uav-core
source install/setup.bash
./launch_multi_drone_optimizer.sh
```

**終端 2**（RViz）：
```bash
cd ~/uav-core
source install/setup.bash
./launch_rviz.sh
```

**預期結果**：
- [x] RViz 視窗開啟
- [x] 看到 3 個邊界框（紅/綠/藍）
- [x] 看到球體標記（如果有位置數據）
- [x] 無 TF 錯誤

---

## 測試 5: Topic 檢查

### 驗證 Topic 隔離

**Jetson 端**：
```bash
# 檢查正在發布的 topics
ros2 topic list | grep link_quality
```

**預期輸出**：
```
/drone_1/link_quality  ← Jetson 1
```

**筆電端**：
```bash
# 檢查訂閱的 topics
ros2 topic list | grep link_quality
```

**預期輸出**：
```
/drone_1/link_quality
/drone_2/link_quality
/drone_3/link_quality
```

**驗證數據流**：
```bash
# 在筆電上監聽 Jetson 發送的數據
ros2 topic echo /drone_1/link_quality
```

**預期輸出**：
```json
data: '{"target_id": "!11111111", "status": "Success", ...}'
---
data: '{"target_id": "!11111112", "status": "Success", ...}'
---
```

---

## 測試 6: 速度指令發布檢查

### 筆電端發布速度指令

**監聽速度指令**（在 Jetson 端）：
```bash
ros2 topic echo /drone_1/offboard_velocity_cmd
```

**預期輸出**（優化器決策後）：
```
linear:
  x: 0.15
  y: -0.10
  z: -0.30
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 0.0   ← 3 秒後歸零！
  y: 0.0
  z: 0.0
...
```

**成功標準**：
- [x] 移動時速度非零
- [x] 3 秒後速度變為 (0, 0, 0)
- [x] 發布頻率約 100Hz

---

## 測試 7: 真實硬體測試（有 LoRa）

### Jetson 端（真實 Meshtastic）

1. **修改 Tracker ID**：
   ```bash
   nano src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py
   # 第 34-41 行，填入實際的 Tracker ID
   ```

2. **查詢 Tracker ID**：
   ```bash
   meshtastic --info
   # 記下 "User ID": !xxxxxxxx
   ```

3. **啟動**：
   ```bash
   cd ~/uav-core
   source install/setup.bash
   ./launch_jetson.sh 1
   ```

**預期輸出**：
```
[INFO] ...: FastScanNode 啟動，綁定 Tracker: ['!實際ID_A', '!實際ID_B']
[INFO] ...: ✓ 掃描 !實際ID_A 成功，RSSI: -82.0 dBm, SNR: 9.5 dB
[INFO] ...: ✓ 掃描 !實際ID_B 成功，RSSI: -78.0 dBm, SNR: 11.0 dB
```

### 筆電端

```bash
./launch_multi_drone_optimizer.sh
```

**預期輸出**：
```
[INFO] ...: Drone 1: 收到 !實際ID_A 信号 (RSSI: -82.0, SNR: 9.5)
[INFO] ...: Drone 1: 收到 !實際ID_B 信号 (RSSI: -78.0, SNR: 11.0)
[INFO] ...: Drone 1: 移动 1/5 ...
```

---

## 🚨 常見問題排查

### Q1: 筆電收不到 Jetson 的訊號

**檢查**：
```bash
# 在筆電上
ros2 topic list | grep link_quality
# 應該看到 /drone_1/link_quality

ros2 topic hz /drone_1/link_quality
# 應該顯示發布頻率
```

**可能原因**：
- 網路未連接（ping 測試）
- Jetson 端未啟動 fast_scan_node
- ROS_DOMAIN_ID 不同

### Q2: 多台無人機的數據混在一起

**這不可能發生！** 因為：
- 每台使用不同 topic (`/drone_1/...`, `/drone_2/...`)
- 每台有獨立的 `DroneState`
- 每台有獨立的線程

**但如果真的看到混亂**，檢查：
```bash
# 確認 Jetson 發布到正確的 topic
ros2 topic echo /drone_1/link_quality  # 應只看到 Drone 1 的數據
ros2 topic echo /drone_2/link_quality  # 應只看到 Drone 2 的數據
```

### Q3: 優化器沒有反應

**檢查**：
```bash
# 查看日誌
ros2 run px4_offboard multi_drone_signal_optimizer \
  --ros-args -p num_drones:=1 -p drone_ids:="[1]" \
  --log-level debug
```

**可能原因**：
- 沒收到兩個 Tracker 的數據（只收到一個）
- JSON 格式錯誤
- Tracker status 不是 "Success"

---

## ✅ 測試成功標準總結

| 測試項目 | 成功標準 |
|---------|---------|
| **Topic 隔離** | 每個 drone_id 有獨立的 topic |
| **動態 ID** | 筆電自動識別任意 Tracker ID |
| **異步處理** | 多台無人機日誌交錯顯示 |
| **速度歸零** | 移動 3 秒後速度變為 (0,0,0) |
| **RViz** | 顯示邊界框和球體，無錯誤 |
| **真實硬體** | 成功掃描並發布實際 LoRa 數據 |

---

**現在你可以按照這些步驟逐一測試了！** 🚀

有任何問題隨時告訴我！
