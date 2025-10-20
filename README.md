# 📖 快速參考指南

## 📚 文檔列表 (只需要這些!)

| 文檔 | 用途 | 閱讀時間 |
|------|------|---------|
| **MODE_SEARCH.md** | 永遠移動找訊號 | 3 分鐘 |
| **MODE_HOLD.md** | 找到最佳點後停留 | 3 分鐘 |
| **SINGLE_MACHINE_TEST.md** | 單機測試步驟 | 5 分鐘 |
| **HOW_TO_MODIFY.md** | 修改算法/平滑度 | 10 分鐘 |
| **SYSTEM_ARCHITECTURE.md** | 系統設計文檔 | 10 分鐘 |

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

## 📊 參數解釋

| 參數 | 值 | 含義 |
|------|-----|------|
| `search_mode:=fast` | 2×2m | 快速小範圍 |
| `search_mode:=thorough` | 3×3m | 完整大範圍 |
| `convergence_iterations:=3` | 3 次 | 3 次無改進停留 (2-3m) |
| `convergence_iterations:=5` | 5 次 | 5 次無改進停留 (3-5m) ⭐ |
| `convergence_iterations:=7` | 7 次 | 7 次無改進停留 (5-7m) |
| `convergence_iterations:=999` | ∞ | 永遠搜索 |

---

## 🎯 選擇指南

### 我想要什麼? 用哪個?

```
✓ 永遠尋找最強訊號
  → MODE_SEARCH.md (convergence_iterations:=999)

✓ 找到最佳點後停留
  → MODE_HOLD.md (convergence_iterations:=3/5/7)

✓ 快速完成 (2-3 分鐘)
  → MODE_HOLD.md + convergence_iterations:=3

✓ 精確搜索 (5-7 分鐘)
  → MODE_HOLD.md + search_mode:=thorough + convergence_iterations:=7

✓ 改變算法
  → HOW_TO_MODIFY.md (修改 1)

✓ 改變平滑度
  → HOW_TO_MODIFY.md (修改 2)

✓ 進行單機測試
  → SINGLE_MACHINE_TEST.md
```

---

## ✅ 成功標誌

### 版本 A (持續搜索)
```
不停止 (直到 Ctrl+C)
```

### 版本 B (自動停留)
```
[INFO] 🎯 收斂完成！切換到 HOLD 模式
[INFO] ⏸️ 停留在最佳點 (X, Y) | Best Signal: XXX
```

---

## 🆘 快速幫助

| 問題 | 檢查 |
|------|------|
| MicroXRCE 無反應 | 終端 1 是否有 "Connected!" |
| 無節點輸出 | 終端 2 是否 source setup.bash |
| 無訊號 | Meshtastic 是否連接 |
| 無人機不動 | PX4 是否進入 Offboard 模式 |

**詳細:** `SINGLE_MACHINE_TEST.md` → 故障排除

---

## 📝 修改完成後

每次修改代碼後:

```bash
# 1. 重新編譯
cd ~/uav-core
colcon build --merge-install

# 2. 重新載入環境
source install/setup.bash

# 3. 重新啟動節點
ros2 launch px4_offboard single_drone_fast.launch.py ...
```

---

## 🚀 一鍵啟動

**版本 A (搜索):**
```bash
cd ~/uav-core && source install/setup.bash && \
ros2 launch px4_offboard single_drone_fast.launch.py search_mode:=fast convergence_iterations:=999
```

**版本 B (停留) [推薦]:**
```bash
cd ~/uav-core && source install/setup.bash && \
ros2 launch px4_offboard single_drone_fast.launch.py search_mode:=fast convergence_iterations:=5
```

---

## 📊 版本對比

| 項目 | 版本 A (搜索) | 版本 B (停留) |
|------|--------------|--------------|
| 行為 | 永遠移動 | 搜索→停留 |
| 終止 | 手動停止 | 自動停留 |
| 時間 | 無限 | 3-5 分鐘 |
| 用途 | 環境監測 | 信號優化 |
| 能耗 | ❌ 高 | ✅ 低 |
| 適用 | 動態環境 | 靜態部署 |

---

