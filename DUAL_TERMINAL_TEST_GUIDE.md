# 雙終端測試指南

## 📋 測試前準備

### 配置 Tracker ID（每台 Jetson 飛行前必做）

編輯 `/home/landis/uav-core/src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`：

```python
# ===== 配置区域：第 34-41 行 =====
TRACKER_A_ID = "!e2e5b7c4"  # ← 修改為實際的 Tracker A ID
TRACKER_B_ID = "!e2e5b8f8"  # ← 修改為實際的 Tracker B ID
# ===================================
```

**查詢 Tracker ID**：
```bash
meshtastic --info
# 找到 "User ID" 欄位（格式：!xxxxxxxx）
```

**重新編譯**：
```bash
cd /home/landis/uav-core
colcon build --merge-install
source install/setup.bash
```

---

## 🖥️ 終端 1：地面站（筆電）

### 啟動優化器 + RViz2
```bash
cd /home/landis/uav-core
source install/setup.bash
./launch_multi_drone_optimizer.sh
```

**預期輸出**：
```
==========================================
启动多无人机信号优化系统
==========================================
[INFO] [multi_drone_signal_optimizer]: 等待 Drone 1 的信号数据...
[INFO] [multi_drone_signal_optimizer]: 等待 Drone 2 的信号数据...
[INFO] [multi_drone_signal_optimizer]: 等待 Drone 3 的信号数据...
```

### RViz2 可視化
另開終端：
```bash
cd /home/landis/uav-core
source install/setup.bash
./launch_rviz.sh
```

**檢查項目**：
- [ ] 看到三個 3m × 3m × 3m 的邊界框（紅/綠/藍）
- [ ] 看到三個球體標記（代表無人機當前位置）
- [ ] 看到 `/drone_N/link_quality` topic 正常接收

---

## 🚁 終端 2：Jetson 機載（每台無人機）

### Drone 1 Jetson
```bash
cd /home/landis/uav-core
source install/setup.bash
./launch_jetson.sh 1
```

### Drone 2 Jetson
```bash
cd /home/landis/uav-core
source install/setup.bash
./launch_jetson.sh 2
```

### Drone 3 Jetson
```bash
cd /home/landis/uav-core
source install/setup.bash
./launch_jetson.sh 3
```

**預期輸出**：
```
==========================================
启动 Jetson 机载节点
无人机编号: 1
==========================================
启动节点...
  - fast_scan_node → /drone_1/link_quality
  - velocity_control → /drone_1/offboard_velocity_cmd

[INFO] [fast_scan_node]: FastScanNode 啟動，綁定 Tracker: ['!e2e5b7c4', '!e2e5b8f8']
[INFO] [velocity_control]: OffboardControl 節點啟動
```

---

## ✅ 運行檢查清單

### 啟動階段
- [ ] 地面站：優化器成功啟動，等待 3 架無人機數據
- [ ] RViz2：顯示三個邊界框和無人機標記
- [ ] Jetson 1/2/3：成功連接 Meshtastic，開始掃描

### 飛行前檢查
- [ ] 遙控器：確認所有無人機響應手動控制
- [ ] PX4：設置 Offboard 模式為可用飛行模式之一
- [ ] 電池：所有設備電量充足
- [ ] GPS：3D Fix，HDOP < 1.5

### 起飛與切換 Offboard
1. **手動起飛**（遙控器操作）：
   - [ ] 解鎖（Arm）
   - [ ] 起飛至約 5m 高度
   - [ ] 確認穩定懸停

2. **切換至 Offboard**（遙控器飛行模式開關）：
   - [ ] 地面站日誌顯示：`[INFO] Drone 1: 進入 OFFBOARD 模式`
   - [ ] RViz2 中球體開始移動

### 運行階段監控

#### 地面站日誌應顯示：
```
[INFO] Drone 1: 收到 Tracker A 数据，等待 Tracker B...
[INFO] Drone 1: 收到 Tracker B 数据，质量 0.750
[INFO] Drone 1: 移动 1/5, 速度 (0.15, -0.10, 0.00), 质量 0.750
[INFO] Drone 1: 移动完成，悬停等待下一轮扫描  ← 重要！確認此訊息
```

#### Jetson 日誌應顯示：
```
[INFO] [fast_scan_node]: ✓ 掃描 !e2e5b7c4 成功，RSSI: -85.0 dBm, SNR: 8.5 dB
[INFO] [fast_scan_node]: ✓ 掃描 !e2e5b8f8 成功，RSSI: -82.0 dBm, SNR: 9.0 dB
```

#### 檢查項目：
- [ ] 無人機移動約 3 秒後**自動懸停**（速度歸零）
- [ ] 懸停期間 Tracker 進行新一輪掃描（~120 秒）
- [ ] 收到新數據後，再次計算移動方向
- [ ] 重複上述循環，最多 5 次移動

### 安全測試
- [ ] **RC Override 測試**：移動遙控器搖桿 → 無人機立即切回手動模式
- [ ] **模式切換測試**：遙控器切換飛行模式 → 離開 Offboard
- [ ] **Failsafe 測試**：關閉遙控器 → 觸發 RTL（返航）

---

## 🚨 異常處理

### 問題 1：地面站未收到 link_quality
**症狀**：地面站持續顯示 "等待信号数据"

**檢查**：
```bash
# 在筆電上
ros2 topic list | grep link_quality
# 應顯示：/drone_1/link_quality, /drone_2/link_quality, /drone_3/link_quality

ros2 topic echo /drone_1/link_quality
# 應有 JSON 格式數據
```

**原因**：
- Jetson 未啟動或網路未連接
- Tracker 未綁定或 ID 錯誤

### 問題 2：無人機不懸停，持續移動
**症狀**：移動後不停止，一直飛

**檢查**：
```bash
# 在 Jetson 上查看日誌
journalctl -f | grep "移动完成"
# 應看到 "移动完成，悬停等待下一轮扫描"
```

**解決**：
- 立即用遙控器切回手動模式
- 檢查 Git commit `85d5aff` 是否正確拉取
- 重新編譯

### 問題 3：遙控器無法介入
**症狀**：搖動搖桿無反應（**極不可能發生**）

**立即處理**：
1. 切換飛行模式開關（強制離開 Offboard）
2. 關閉遙控器（觸發 Failsafe RTL）
3. 按下緊急停止按鈕（如有）

**事後檢查**：
- 檢查 PX4 參數：`COM_RC_OVERRIDE` 是否啟用
- 檢查遙控器校準和連接狀態

---

## 📊 成功標準

### 單次測試成功標準
- [ ] 無人機完成 1-5 次移動
- [ ] 每次移動後正確懸停
- [ ] Tracker 掃描數據持續更新
- [ ] 遙控器隨時可以介入
- [ ] 無碰撞、無失控

### 最終收斂標準（優化器）
滿足以下**任一條件**自動停止：
1. **移動次數達上限**：`movement_count >= 5`
2. **連續無改善**：連續 3 次移動後信號品質未提升

**停止後行為**：
- 無人機懸停在最佳位置
- 繼續監控信號品質（不再移動）
- 等待手動切回手動模式降落

---

## 🎯 測試目標

### 階段 1：單機功能測試
- [ ] 單架無人機完整循環（移動 → 懸停 → 掃描）
- [ ] 遙控器介入測試
- [ ] 邊界保護測試（接近 ±1.5m 邊界）

### 階段 2：多機協同測試
- [ ] 三架同時起飛並切換 Offboard
- [ ] 三架同步執行優化循環
- [ ] 各自收斂至最佳位置

### 階段 3：信號品質驗證
- [ ] 記錄初始信號品質（起飛位置）
- [ ] 記錄每次移動後的信號品質
- [ ] 驗證最終位置信號品質提升

---

## 📝 測試日誌範本

### 測試資訊
- **日期**：____________________
- **地點**：____________________
- **天氣**：____________________
- **測試人員**：____________________

### 設備狀態
- **Drone 1**：Tracker ID: __________ / __________
- **Drone 2**：Tracker ID: __________ / __________
- **Drone 3**：Tracker ID: __________ / __________

### 測試結果
| 項目 | Drone 1 | Drone 2 | Drone 3 |
|------|---------|---------|---------|
| 啟動成功 | ☐ | ☐ | ☐ |
| 切換 Offboard | ☐ | ☐ | ☐ |
| 懸停正常 | ☐ | ☐ | ☐ |
| RC Override | ☐ | ☐ | ☐ |
| 優化完成 | ☐ | ☐ | ☐ |

### 異常記錄
_______________________________________________
_______________________________________________
_______________________________________________

---

**準備好了嗎？開始測試！🚀**
