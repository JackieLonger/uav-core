# 🎯 UAV 訊號優化系統 - 快速開始

## 📚 核心文檔 (按優先級)

| 文檔 | 用途 | 讀者 | 時間 |
|------|------|------|------|
| **TESTING_GUIDE.md** ⭐⭐⭐ | 完整測試流程和邏輯 | **所有人 必讀** | 20 分鐘 |
| **QUICK_REFERENCE.md** ⭐⭐ | 快速命令參考 | 開發者 | 5 分鐘 |
| **JETSON_DEPLOYMENT_GUIDE_FINAL.md** ⭐⭐ | Jetson 部署詳細指南 | Jetson 用戶 | 15 分鐘 |
| **FINAL_BUILD_REPORT.md** | 編譯驗證報告 | 構建工程師 | 10 分鐘 |

---

## ⚡ 快速啟動命令

### 版本 A: 持續搜索 (永遠移動)

```bash
# 終端 1
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# 終端 2
cd ~/uav-core && source install/setup.bash
ros2 launch px4_offboard single_drone_fast.launch.py \
  search_mode:=fast convergence_iterations:=999
```

**詳見:** `MODE_SEARCH.md`

---

### 版本 B: 自動停留 (推薦) ⭐

```bash
# 終端 1
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# 終端 2
cd ~/uav-core && source install/setup.bash
ros2 launch px4_offboard single_drone_fast.launch.py \
  search_mode:=fast convergence_iterations:=5
```

**詳見:** `MODE_HOLD.md`

---

## 🧪 單機測試 5 步

1. **準備:** Jetson/PX4/Meshtastic 都已開機
2. **MicroXRCE:** 終端 1 執行上面的命令
3. **ROS2 節點:** 終端 2 執行上面的命令
4. **監控:** 看終端 2 輸出 3-5 分鐘
5. **完成:** 看到 `🎯 收斂完成!` 或永遠移動

**詳見:** `SINGLE_MACHINE_TEST.md`

---

## 🔧 如何修改

### 改變搜索算法

方案:
- 隨機遊走 (Random Walk) - 當前
- 網格搜索 (Grid Search) - 完整覆蓋
- 梯度上升 (Gradient Ascent) - 聰明移動
- 螺旋搜索 (Spiral Search) - 系統搜索

**詳見:** `HOW_TO_MODIFY.md` → 修改 1

---

### 改變信號平滑度

改變訊號窗口大小:
```python
if len(self.signal_history) > 3:  # ← 改 3 為其他值
    self.signal_history.pop(0)
```

- 更小 (如 2): 反應快, 雜訊多
- 更大 (如 5): 反應慢, 更穩定

**詳見:** `HOW_TO_MODIFY.md` → 修改 2

---

### 改變改進判定

```python
if improvement > self.convergence_threshold:  # ← 改這裡
```

- 降低 (如 0.1): 更容易認為改進
- 提高 (如 1.0): 只有顯著改進

**詳見:** `HOW_TO_MODIFY.md` → 修改 3

---

## � 快速啟動 (30 秒)

### Jetson 端

```bash
# 1. 進入工作目錄並編譯 (首次)
cd ~/uav-core && colcon build --merge-install

# 2. 載入環境
source install/setup.bash

# 3. 啟動優化器
ros2 run ros2_px4_offboard_example signal_optimizer_node_v3.py
```

### 筆電端 (監控)

```bash
# 終端 1: 訊號掃描
source ~/uav-core/install/setup.bash
ros2 run ros2_px4_offboard_example fast_scan_node.py

# 終端 2: 監看無人機狀態
ros2 topic echo /drone_1/optimizer_status

# 終端 3: 監看訊號歷史
ros2 topic echo /drone_1/signal_history
```

---

## 📋 測試流程

### 🎯 單機測試流程

```
[Jetson 連接 Pixhawk]
         ↓
[啟動優化器節點]
         ↓
[檢查訊號接收]
         ↓
[自動 ARM 和起飛]
         ↓
[在 3×3×3m 搜索最佳訊號點]
         ↓
[30-90 秒後收斂]
         ↓
[進入 HOLD 模式停留]
```

**詳細步驟 → 參考 TESTING_GUIDE.md**

### 🚁 多機獨立測試流程 (推薦)

```
[Jetson-1]  [Jetson-2]  [Jetson-3]
     ↓           ↓            ↓
  [啟動]      [啟動]       [啟動]
     ↓           ↓            ↓
  [搜索]      [搜索]       [搜索]
  空間1      空間2       空間3
  
完全獨立，無干擾 ✅
```

**詳細步驟 → 參考 TESTING_GUIDE.md**

---

## ✅ 成功標誌

### 單機測試
```bash
# 預期輸出
[INFO] 🤖 Signal optimizer v3 ready | Drone: drone_1 | Mode: search
[INFO] 🔋 Vehicle status: ARMING_STATE_STANDBY
[INFO] 🚀 Armed successfully
[INFO] ✈️ Takeoff complete, starting search
[INFO] 📊 Evaluating signal... Score: 42.5
# ... 繼續搜索 ...
[INFO] 🎯 Convergence detected! Switching to HOLD mode
[INFO] ⏸️ Holding position at (x, y, z)
```

### 多機測試
```bash
# 終端輸出 (多個無人機)
/drone_1/optimizer_status
/drone_2/optimizer_status
/drone_3/optimizer_status

# 各無人機位置不重疊，獨立收斂
```

---

## 🆘 常見問題

| 問題 | 解決方案 |
|------|--------|
| **無法接收訊號** | 檢查 Meshtastic 是否配置；查看 `/link_quality` 話題 |
| **無法 ARM** | 檢查 PX4 狀態；查看電池電量 |
| **無人機不動** | 檢查 OffboardControlMode 和速度命令是否發送 |
| **多機位置重疊** | 調整 `search_bounds_x/y/z` 參數；或分別在不同位置起飛 |

**詳細故障排查 → 參考 TESTING_GUIDE.md**

---

## 📚 文檔導航

- **TESTING_GUIDE.md** ⭐ - 完整測試邏輯和流程 (必讀!)
- **QUICK_REFERENCE.md** - ROS2 快速命令參考
- **JETSON_DEPLOYMENT_GUIDE_FINAL.md** - Jetson 部署詳細步驟
- **FINAL_BUILD_REPORT.md** - 編譯驗證報告

---

## 📝 每次修改後

```bash
cd ~/uav-core

# 1. 重新編譯
colcon build --merge-install

# 2. 重新載入
source install/setup.bash

# 3. 重新啟動
ros2 run ros2_px4_offboard_example signal_optimizer_node_v3.py
```

---

## 🎓 核心概念

| 概念 | 說明 |
|------|------|
| **訊號評分** | `Score = SNR × 0.7 + (RSSI / -50) × 100 × 0.3` |
| **3×3×3m 搜索** | 每個無人機在 (±1.5m X/Y, 0~3m Z) 範圍內搜索 |
| **雙 Tracker** | 優化器等待兩個信號源都提供數據後再決策 |
| **收斂判定** | 連續 5 次 (可配置) 無改進就進入 HOLD 模式 |
| **HOLD 模式** | 無人機停留在最佳點，零速度命令 |

---

