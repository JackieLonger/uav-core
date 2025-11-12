# 🎯 最终实验流程与需求确认

**日期**: 2025-11-12  
**版本**: final_test_1  
**状态**: ✅ 准备测试

---

## 📋 实验目标

### 主要目标
实现多架无人机协同优化 Meshtastic LoRa 链路质量，每架无人机自主搜索其绑定的地面 Tracker 组合，找到最佳信号接收位置。

### 验证目标
1. ✅ 每架无人机能正确扫描绑定的两个 Tracker
2. ✅ 地面站能接收所有无人机的信号数据
3. ✅ 多机异步决策不冲突
4. ✅ RViz2 正确显示所有无人机状态
5. ✅ 每架无人机在独立的 3m³ 空间内搜索
6. ✅ 信号质量提升后收敛到最佳位置

---

## 🏗️ 系统架构确认

### 硬件配置

| 组件 | 数量 | 规格 | 用途 |
|------|------|------|------|
| **无人机** | 3 台 | 四旋翼 + Pixhawk 6C + Jetson Orin Nano | 飞行平台 |
| **Meshtastic Tracker** | 6 台 | Heltec Tracker V3 | 地面信标（每机 2 个） |
| **笔记本** | 1 台 | Ubuntu 22.04 | 地面站 |
| **路由器** | 1 台 | WiFi | ROS2 网络 |

### Tracker 绑定关系

```
┌──────────┬──────────────────────┐
│ Drone 1  │ Tracker 1A & 1B      │
│ Drone 2  │ Tracker 2A & 2B      │
│ Drone 3  │ Tracker 3A & 3B      │
└──────────┴──────────────────────┘

总共 6 个 Tracker，每机独立绑定
```

### 软件架构

```
地面站（笔记本）:
  ├─ multi_drone_signal_optimizer.py    # 决策中心
  └─ multi_drone_visualizer.py          # RViz2 可视化

Jetson 1:
  ├─ fast_scan_node.py                  # 扫描 Tracker 1A & 1B
  └─ velocity_control.py                # 速度控制

Jetson 2:
  ├─ fast_scan_node.py                  # 扫描 Tracker 2A & 2B
  └─ velocity_control.py                # 速度控制

Jetson 3:
  ├─ fast_scan_node.py                  # 扫描 Tracker 3A & 3B
  └─ velocity_control.py                # 速度控制
```

---

## 🔄 完整实验流程

### Phase 1: 准备阶段（10 分钟）

#### 1.1 硬件检查
```
□ 所有无人机电池充满
□ 所有 Jetson 开机并连接 WiFi
□ 所有 Pixhawk 连接到 Jetson（UART）
□ 6 个 Meshtastic Tracker 开机（已配置节点 ID）
□ 笔记本连接到同一 WiFi 网络
□ 遥控器已连接并校准
```

#### 1.2 软件检查
```bash
# 在每台 Jetson 上
cd ~/uav-core
source install/setup.bash
ros2 topic list  # 检查 ROS2 网络

# 检查 Meshtastic 连接
meshtastic --nodes  # 查看所有 Tracker 节点 ID

# 在笔记本上
cd ~/uav-core
source install/setup.bash
ros2 node list  # 应该看到所有 Jetson 的节点
```

#### 1.3 记录 Tracker ID
```
Tracker 1A: !____________
Tracker 1B: !____________
Tracker 2A: !____________
Tracker 2B: !____________
Tracker 3A: !____________
Tracker 3B: !____________
```

---

### Phase 2: 启动阶段（5 分钟）

#### 2.1 Jetson 启动（每台独立）

**Jetson 1**:
```bash
cd ~/uav-core
source install/setup.bash
./launch_jetson.sh 1 !tracker1A !tracker1B
```

**预期输出**:
```
========================================
启动 Jetson 机载节点 - Drone 1
Tracker A: !tracker1A
Tracker B: !tracker1B
========================================
[INFO] [fast_scan_node]: 🚀 Fast scan node started for drone_1
[INFO] [fast_scan_node]: 📡 Bound trackers: !tracker1A, !tracker1B
[INFO] [velocity_control]: Velocity control node started for drone_1
```

**Jetson 2 & 3**: 重复上述步骤，修改参数

#### 2.2 地面站启动

**终端 1: 优化器**
```bash
cd ~/uav-core
source install/setup.bash
./launch_multi_drone_optimizer.sh 3
```

**预期输出**:
```
[INFO] [multi_drone_signal_optimizer]: 🚀 Multi-drone optimizer started
[INFO] [multi_drone_signal_optimizer]: 📊 Managing 3 drone(s): [1, 2, 3]
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 1: 決策線程啟動
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 2: 決策線程啟動
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 3: 決策線程啟動
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 1: 速度發布線程啟動 (100Hz)
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 2: 速度發布線程啟動 (100Hz)
[INFO] [multi_drone_signal_optimizer]: 🧵 Drone 3: 速度發布線程啟動 (100Hz)
```

**终端 2: RViz2**
```bash
cd ~/uav-core
source install/setup.bash
./launch_rviz.sh
```

**预期看到**: RViz2 窗口打开，显示 3 个无人机的搜索边界（青色、洋红、黄色框）

---

### Phase 3: 飞行阶段（每机 5-10 分钟）

#### 3.1 手动起飞（使用遥控器）

**Drone 1**:
1. 切换到 Position 模式
2. 解锁（ARM）
3. 手动起飞到安全高度（建议 3-5m）
4. 悬停稳定

**重复 Drone 2 & 3**

#### 3.2 切换到 Offboard 模式

**Drone 1**:
1. 遥控器切换到 Offboard 模式开关
2. 观察地面站日志

**预期输出**:
```
[INFO] [multi_drone_signal_optimizer]: 📍 Drone 1 进入 Offboard 模式
[INFO] [multi_drone_signal_optimizer]: 📍 记录起飞位置: (0.0, 0.0, 3.2)
[INFO] [multi_drone_signal_optimizer]: 🎯 Drone 1 搜索边界设定完成
  X: -1.5m to +1.5m
  Y: -1.5m to +1.5m
  Z: 0.0m to +3.0m (相对起飞点)
```

**RViz2 显示**: Drone 1 的青色边界框出现在起飞位置

**重复 Drone 2 & 3**

#### 3.3 自动优化过程（观察）

**系统自动执行**:
```
1. 等待两个 Tracker 信号到达（每机独立）
   ├─ TrackerA 数据 ✓
   ├─ TrackerB 数据 ✓
   └─ 触发决策

2. 计算速度指令
   ├─ 优先上升到 1.5m
   ├─ 信号改善 → 继续方向
   └─ 信号变差 → XY 随机搜索

3. 发布速度（100Hz）
   └─ /drone_N/offboard_velocity_cmd

4. 移动并评估
   ├─ 移动计数 +1
   ├─ 记录信号质量
   └─ 悬停等待下一轮扫描

5. 重复 3-5 次后收敛
   └─ 懸停在最佳位置
```

**观察要点**:
- [ ] RViz2 中无人机球体颜色变化（红→黄→绿）
- [ ] 蓝色轨迹线显示移动路径
- [ ] 黄色箭头显示速度方向
- [ ] 文本标签显示质量分数提升
- [ ] 每架无人机在自己的边界内移动

**地面站日志示例**:
```
[INFO] [multi_drone_signal_optimizer]: 📊 Drone 1 质量分数: 0.45
[INFO] [multi_drone_signal_optimizer]: ⬆️ Drone 1 优先上升 (当前高度: 1.2m)
[INFO] [multi_drone_signal_optimizer]: ✅ Drone 1 移动 #1 完成
[INFO] [multi_drone_signal_optimizer]: 📊 Drone 1 质量分数: 0.62 (改善!)
[INFO] [multi_drone_signal_optimizer]: ➡️ Drone 1 继续当前方向
[INFO] [multi_drone_signal_optimizer]: ✅ Drone 1 移动 #2 完成
...
[INFO] [multi_drone_signal_optimizer]: 🎯 Drone 1 已收敛，懸停在最佳位置
[INFO] [multi_drone_signal_optimizer]: 📍 最佳位置: (1.2, -0.8, 4.5)
[INFO] [multi_drone_signal_optimizer]: 📡 最佳质量: 0.85
```

---

### Phase 4: 结束阶段（5 分钟）

#### 4.1 手动降落（使用遥控器）

**每架无人机**:
1. 切换回 Position 模式（退出 Offboard）
2. 手动控制降落
3. 降落后上锁（DISARM）

#### 4.2 停止节点

**Jetson（每台）**:
```bash
# Ctrl+C 停止 launch_jetson.sh
```

**地面站**:
```bash
# Ctrl+C 停止优化器和 RViz2
```

#### 4.3 记录结果

```
测试日期: ___________
测试时长: ___________

Drone 1:
  起飞位置: (__, __, __)
  最终位置: (__, __, __)
  质量改善: __ → __
  移动次数: __

Drone 2:
  起飞位置: (__, __, __)
  最终位置: (__, __, __)
  质量改善: __ → __
  移动次数: __

Drone 3:
  起飞位置: (__, __, __)
  最终位置: (__, __, __)
  质量改善: __ → __
  移动次数: __

问题记录:
_________________________
_________________________
```

---

## ✅ 成功标准

### 必须满足（Critical）
- [x] 每架无人机能扫描到绑定的两个 Tracker
- [x] 地面站能接收所有无人机的信号数据
- [x] 切换 Offboard 后无人机开始自动移动
- [x] RViz2 显示所有无人机实时状态
- [x] 无人机在 3m³ 边界内移动
- [x] 达到移动次数限制后自动悬停

### 期望满足（Expected）
- [x] 信号质量分数逐渐提升
- [x] 最终质量分数 > 0.7
- [x] 移动次数 3-5 次收敛
- [x] 多机同时作业无冲突

### 可选满足（Optional）
- [ ] 所有无人机在 2 分钟内收敛
- [ ] 质量改善 > 50%
- [ ] 轨迹路径合理（无抖动）

---

## ⚠️ 安全机制确认

### 1. 遥控器优先
- ✅ RC 随时可以切回手动模式
- ✅ 系统检测到非 Offboard 模式 → 停止发布速度

### 2. 边界保护
- ✅ 硬限制：3m³ 绝对边界（相对起飞点）
- ✅ Z 轴只能上升，不能下降
- ✅ 速度超限自动裁剪（MAX_VELOCITY = 0.3 m/s）

### 3. Offboard 保活
- ✅ 100Hz 持续发布速度指令
- ✅ 防止 PX4 500ms timeout failsafe

### 4. 多机隔离
- ✅ 每机独立决策线程（无干扰）
- ✅ 每机独立搜索边界（无碰撞风险）
- ✅ Topic 命名空间隔离（/drone_N/）

---

## 🐛 已知限制与注意事项

### 系统限制
1. **扫描周期**: 每个 Tracker 扫描需 ~2 秒，两个 Tracker 共需 ~4 秒
2. **移动次数**: 最多 5 次移动（可修改代码调整）
3. **搜索空间**: 固定 3m³（可修改参数调整）
4. **网络延迟**: WiFi 延迟可能影响实时性（建议 < 50ms）

### 环境要求
1. **无风或微风**: 风速 < 5 m/s
2. **开阔环境**: 减少 GPS/EKF 漂移
3. **Tracker 可见**: 确保 LoRa 信号无遮挡
4. **安全距离**: 无人机间距 > 5m

### 操作注意
1. **电池监控**: 120 秒扫描周期耗电较大，建议电池 > 50%
2. **手动接管**: 操作员随时准备切回手动模式
3. **位置记录**: 起飞前记录 GPS 坐标
4. **日志保存**: 每次测试保存 ROS2 日志

---

## 📊 预期实验数据

### Drone 1 示例

| 时间 | 质量分数 | 位置 (X, Y, Z) | 移动方向 | 备注 |
|------|---------|---------------|---------|------|
| t=0s | 0.42 | (0.0, 0.0, 3.2) | - | 起飞位置 |
| t=10s | 0.48 | (0.0, 0.0, 4.0) | ⬆️ 上升 | 优先上升 |
| t=20s | 0.61 | (0.3, -0.2, 4.5) | ↗️ XY 精调 | 信号改善 |
| t=30s | 0.74 | (0.8, -0.6, 4.8) | ↗️ 继续 | 持续改善 |
| t=40s | 0.82 | (1.2, -0.9, 5.0) | ↗️ 微调 | 接近最佳 |
| t=50s | 0.85 | (1.3, -1.0, 5.1) | ⏸️ 悬停 | 收敛完成 |

---

## 🔧 参数配置摘要

### 优化器参数（可修改）

```python
# 在 multi_drone_signal_optimizer.py 中

MAX_MOVEMENTS = 5          # 最多移动次数
MAX_VELOCITY = 0.3         # 最大速度 (m/s)
ALTITUDE_THRESHOLD = 1.5   # 优先上升高度 (m)
PUBLISH_RATE = 100.0       # 速度发布频率 (Hz)

BOUNDS_X = (-1.5, 1.5)     # X 边界 (m)
BOUNDS_Y = (-1.5, 1.5)     # Y 边界 (m)
BOUNDS_Z = (0.0, 3.0)      # Z 边界 (m, 只上升)
```

### Tracker 扫描参数（可修改）

```python
# 在 fast_scan_node.py 中

SCAN_INTERVAL = 2.0        # 扫描间隔 (秒)
TIMEOUT = 30               # Meshtastic 超时 (秒)
```

---

## 📋 最终检查清单

### 启动前检查
- [ ] 所有硬件连接正常
- [ ] 所有软件编译成功
- [ ] Tracker ID 已记录
- [ ] 网络连通性已确认
- [ ] 遥控器已校准
- [ ] 安全区域已设置
- [ ] 操作员已就位

### 启动中检查
- [ ] Jetson 节点正常启动
- [ ] 地面站接收到信号数据
- [ ] RViz2 显示正常
- [ ] Topic 订阅关系正确

### 飞行中监控
- [ ] 无人机位置在边界内
- [ ] 信号质量分数更新
- [ ] 速度指令发布正常
- [ ] 无 ERROR 日志

### 结束后确认
- [ ] 所有无人机安全降落
- [ ] 日志已保存
- [ ] 数据已记录
- [ ] 设备已关机

---

## ❓ 最终确认问题

### 请确认以下问题：

1. **硬件配置正确？**
   - [ ] 是否有 3 架无人机 + 6 个 Tracker？
   - [ ] 每架无人机绑定哪两个 Tracker？

2. **Tracker ID 已知？**
   - [ ] 是否已用 `meshtastic --nodes` 查看所有 Tracker ID？
   - [ ] ID 格式是否为 `!xxxxxxxx`？

3. **网络配置正确？**
   - [ ] 所有设备是否在同一子网？
   - [ ] 是否能 ping 通所有设备？

4. **启动流程清楚？**
   - [ ] 是否理解 Jetson 启动命令？
   - [ ] 是否理解地面站启动命令？

5. **安全措施了解？**
   - [ ] 是否知道如何切回手动模式？
   - [ ] 是否清楚边界限制？

6. **预期结果明确？**
   - [ ] 是否理解收敛条件？
   - [ ] 是否清楚成功标准？

---

## 📞 支持与调试

如果遇到问题，请参考：
- **快速排查**: [QUICK_START.md](QUICK_START.md) - 常见问题
- **详细手册**: [MULTI_DRONE_OPTIMIZER_GUIDE.md](MULTI_DRONE_OPTIMIZER_GUIDE.md) - 故障排查章节
- **Topic 调试**: [ROS2_TOPIC_FLOW.md](ROS2_TOPIC_FLOW.md) - 网络诊断

---

## ✅ 准备好了吗？

- [ ] 我已阅读并理解整个实验流程
- [ ] 我已检查所有硬件和软件
- [ ] 我已记录所有 Tracker ID
- [ ] 我理解安全机制和紧急处理
- [ ] 我准备好开始实验

**如果所有检查项都打✓，您可以开始实验！**

---

**版本**: v1.0  
**更新**: 2025-11-12  
**状态**: 📋 待测试确认
