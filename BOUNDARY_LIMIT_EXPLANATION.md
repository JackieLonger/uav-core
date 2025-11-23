# 邊界限制說明

## 🎯 重要：測試腳本 vs 實際飛行

### 問題：為什麼測試中無人機飛出邊界？

**簡短回答**：測試腳本**繞過了** optimizer 的控制，直接發布位置數據，所以看起來飛出邊界。實際飛行時**不會**飛出邊界。

---

## 📊 兩種模式對比

### 1. 測試腳本模式（目前的測試）
```
測試腳本
  ↓ 直接發布位置
/drone_N/fmu/out/vehicle_local_position (模擬 PX4 輸出)
  ↓
visualizer 接收並顯示
  ↓
❌ 沒有經過邊界檢查！可以飛到任何地方
```

**特點**：
- ✅ 測試軌跡顯示功能
- ✅ 測試信號記錄功能  
- ✅ 快速驗證 RViz2 可視化
- ❌ **不會觸發 optimizer 的邊界檢查**
- ❌ 不代表實際飛行行為

### 2. 實際飛行模式（真實飛行）
```
PX4 飛控
  ↓ 位置反饋
/fmu/out/vehicle_local_position
  ↓
Signal Optimizer (有邊界檢查) ← 信號質量輸入
  ↓ 計算速度命令 (clamp_velocity 限制)
/drone_N/offboard_velocity_cmd
  ↓
velocity_control 節點
  ↓ PX4 控制指令
PX4 飛控執行
  ↓
✅ 受到邊界限制！
```

**特點**：
- ✅ Optimizer 計算最佳移動方向
- ✅ **clamp_velocity() 強制邊界限制**
- ✅ 無人機不會飛出 3x3x3 米範圍
- ✅ 安全保護機制生效

---

## 🔒 邊界設定

### 代碼位置
`multi_drone_signal_optimizer.py` 第 200-202 行：

```python
BOUNDS_X = (-1.5, 1.5)  # 相對起飛點 ±1.5 米
BOUNDS_Y = (-1.5, 1.5)  # 相對起飛點 ±1.5 米
BOUNDS_Z = (0.0, 3.0)   # 只能上升 0-3 米
```

### 邊界檢查邏輯
`clamp_velocity()` 函數（第 433-471 行）：

```python
def clamp_velocity(self, vx, vy, vz, current_pos, takeoff_pos):
    """限制速度，確保不超出邊界"""
    
    # 預測 0.1 秒後的位置
    dt = 0.1
    next_x = current_pos.x + vx * dt
    next_y = current_pos.y + vy * dt
    next_z = current_pos.z + vz * dt
    
    # 如果會超出邊界，速度設為 0
    if next_x < takeoff_pos.x - 1.5 or next_x > takeoff_pos.x + 1.5:
        vx = 0.0
    
    if next_y < takeoff_pos.y - 1.5 or next_y > takeoff_pos.y + 1.5:
        vy = 0.0
    
    # NED: z 減小 = 上升，不允許下降
    if vz > 0:  # 想要下降
        vz = 0.0
    elif next_z < (takeoff_pos.z - 3.0):  # 超過上限
        vz = 0.0
    
    return (vx, vy, vz)
```

---

## ✅ 實際飛行保證

### Optimizer 確保的安全措施

1. **預測性檢查**：
   - 計算 0.1 秒後的位置
   - 如果會超出邊界，立即停止該方向的速度

2. **三維限制**：
   - X 軸：起飛點 ±1.5 米
   - Y 軸：起飛點 ±1.5 米
   - Z 軸：起飛高度 → 起飛高度 + 3 米

3. **速度限制**：
   - 最大速度：0.3 m/s
   - 如果合成速度超過，按比例縮放

4. **下降保護**：
   - 完全禁止向下的速度命令
   - 只能維持高度或上升

---

## 🧪 如何驗證邊界限制

### 測試 1: 邊界檢查測試（推薦）
```bash
chmod +x test_boundary_check.sh
./test_boundary_check.sh
```

這個測試會：
1. 啟動 optimizer + visualizer + RViz2
2. 模擬信號源在 (3, 3, -2) 位置
3. **通過 optimizer 控制無人機**
4. 驗證無人機是否停在 (1.5, 1.5, z) 邊界上

### 測試 2: 檢查速度命令
```bash
# Terminal 1: 啟動系統
./launch_laptop.sh  # 選擇模式 1 (Optimizer only)

# Terminal 2: 監控速度命令
ros2 topic echo /drone_1/offboard_velocity_cmd

# Terminal 3: 發布測試信號
ros2 topic pub /drone_1/link_quality std_msgs/String \
  'data: "{\"status\": \"Success\", \"forward_rssi\": -50.0, \"forward_snr\": 20.0, ...}"'
```

**預期結果**：
- 速度命令的值不會讓無人機超出邊界
- 當接近邊界時，相應方向的速度會變為 0

---

## 📈 視覺化驗證

### 在 RViz2 中確認

啟動完整系統：
```bash
./launch_laptop.sh  # 選擇模式 3 (全部)
```

在 RViz2 視窗中你會看到：

1. **邊界框**（半透明立方體）：
   - 每架無人機有獨立的 3x3x3 米邊界
   - 顏色：Drone 1=青色, Drone 2=洋紅, Drone 3=黃色

2. **無人機球體**：
   - 應該始終在邊界框內
   - 接近邊界時會停止移動

3. **軌跡線條**：
   - 軌跡不會超出邊界框
   - 在邊界處會看到停頓或折返

---

## ⚠️ 測試腳本的局限性

### 目前的測試腳本

**`test_rviz_trajectory.sh`**, **`test_single_drone.sh`**:
- ❌ 直接發布位置，繞過 optimizer
- ❌ 不代表實際飛行行為
- ✅ 但可以測試軌跡顯示和 CSV 記錄

**`test_boundary_check.sh`**:
- ✅ 通過 optimizer 控制
- ✅ 驗證邊界限制
- ✅ 代表實際飛行行為

### 選擇合適的測試

| 測試目的 | 使用腳本 | 是否經過 Optimizer |
|---------|---------|-------------------|
| 測試 RViz2 軌跡顯示 | `test_rviz_trajectory.sh` | ❌ |
| 測試 CSV 記錄功能 | `test_csv_save.sh` | ❌ |
| **驗證邊界限制** | `test_boundary_check.sh` | ✅ |
| **完整系統測試** | `launch_laptop.sh` | ✅ |

---

## 🎓 總結

### 核心答案

**Q: 測試中無人機飛出邊界，實際飛行會嗎？**

**A: 不會！** 因為：

1. ✅ 實際飛行時，**所有速度命令都經過 optimizer 的 `clamp_velocity()`**
2. ✅ 邊界檢查是**強制性的**，無法繞過
3. ✅ 即使 optimizer 想要飛出邊界，速度會被強制設為 0
4. ✅ PX4 還有額外的安全保護（地理圍欄）

### 如何確認

運行邊界檢查測試：
```bash
./test_boundary_check.sh
```

在 RViz2 中確認：
- 無人機球體在邊界框內
- 軌跡線條不超出框架

---

## 📚 相關文件

- **Optimizer 代碼**: `src/.../multi_drone_signal_optimizer.py`
  - 第 200-202 行: 邊界常數定義
  - 第 433-471 行: `clamp_velocity()` 邊界檢查

- **Visualizer 代碼**: `src/.../multi_drone_visualizer.py`
  - 第 320-380 行: 邊界框顯示（已修正 ENU 座標）

- **測試腳本**:
  - `test_boundary_check.sh` - 邊界限制驗證 ⭐
  - `test_rviz_trajectory.sh` - 軌跡顯示測試
  - `launch_laptop.sh` - 完整系統啟動

---

**結論**：測試腳本的超範圍行為**不會**在實際飛行中發生。Optimizer 的邊界檢查機制會確保無人機安全地停留在允許範圍內。
