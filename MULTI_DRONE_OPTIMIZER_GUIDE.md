# 多无人机信号优化器使用指南

## 📋 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         笔记本（地面站）                          │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │    multi_drone_signal_optimizer.py (决策中心)             │  │
│  │    - 多线程异步处理                                        │  │
│  │    - 每个无人机独立决策线程                                 │  │
│  │    - 100Hz 速度指令发布                                    │  │
│  │    - RViz2 可视化显示                                      │  │
│  └───────────────────────────────────────────────────────────┘  │
│         ↓ 订阅                        ↑ 发布                     │
│  /drone_N/link_quality          /drone_N/offboard_velocity_cmd  │
└─────────────────────────────────────────────────────────────────┘
         ↑                                      ↓
         │                                      │
┌────────┴──────────────────────────────────────┴─────────────────┐
│                    ROS2 网络（WiFi/DDS）                         │
└──────────────────────────────────────────────────────────────────┘
         ↑                                      ↓
         │                                      │
┌────────┴──────────┐              ┌────────────┴──────────────────┐
│  Jetson N (机载)  │              │    Pixhawk 6C (飞控)          │
│ ┌───────────────┐ │              │  ┌─────────────────────────┐  │
│ │fast_scan_node │ │              │  │ PX4 v1.14.3             │  │
│ │  绑定:        │ │              │  │ - Offboard Mode         │  │
│ │  TrackerNA    │ │              │  │ - 速度控制              │  │
│ │  TrackerNB    │ │              │  └─────────────────────────┘  │
│ └───────────────┘ │              │                                │
│ ┌───────────────┐ │              │                                │
│ │velocity_control│◄─────────────►│                                │
│ │  (速度转换)    │ │  /fmu/...    │                                │
│ └───────────────┘ │              │                                │
└───────────────────┘              └────────────────────────────────┘

重要说明：
✅ 每架无人机绑定不同的两个 Tracker（总共需要 6 个 Tracker）
✅ Drone 1 → Tracker 1A & 1B
✅ Drone 2 → Tracker 2A & 2B  
✅ Drone 3 → Tracker 3A & 3B
✅ RViz2 在笔记本上显示所有无人机的实时状态
```

## 🚀 快速启动

### 1. 笔记本（地面站）

#### 启动优化器
```bash
# 进入工作目录
cd ~/uav-core

# Source 环境
source install/setup.bash

# 启动优化器（3架无人机）
./launch_multi_drone_optimizer.sh 3

# 或手动启动
ros2 run px4_offboard multi_drone_signal_optimizer \
    --ros-args \
    -p num_drones:=3 \
    -p drone_ids:="[1, 2, 3]"
```

#### 启动 RViz2 可视化（推荐！）
```bash
# 新开一个终端
cd ~/uav-core
source install/setup.bash

# 启动 RViz2
./launch_rviz.sh

# 或手动启动
ros2 run px4_offboard multi_drone_visualizer --ros-args -p num_drones:=3 -p drone_ids:="[1, 2, 3]" &
rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz
```

### 2. Jetson N（每架无人机上）

#### 方法1：使用一键启动脚本（推荐）
```bash
# 在 Jetson 上进入工作目录
cd ~/uav-core
source install/setup.bash

# 启动（自动配置 topic + Tracker 绑定）
./launch_jetson.sh [无人机编号] [TrackerA ID] [TrackerB ID]

# 例如：
./launch_jetson.sh 1 !tracker1A !tracker1B   # Drone 1
./launch_jetson.sh 2 !tracker2A !tracker2B   # Drone 2
./launch_jetson.sh 3 !tracker3A !tracker3B   # Drone 3

# 重要：每架无人机绑定不同的两个 Tracker
# 请替换为实际的 Meshtastic Tracker 节点 ID
```

#### 方法2：使用 ROS2 Launch 文件
```bash
source install/setup.bash

ros2 launch px4_offboard jetson_onboard.launch.py \
    drone_id:=1 \
    tracker_a_id:=!tracker1A \
    tracker_b_id:=!tracker1B
```

#### 方法3：手动启动（不推荐）
```bash
# Terminal 1: 启动扫描节点
source install/setup.bash
ros2 run px4_offboard fast_scan --ros-args -r link_quality:=/drone_N/link_quality

# Terminal 2: 启动速度控制
ros2 run px4_offboard velocity_control --ros-args -r offboard_velocity_cmd:=/drone_N/offboard_velocity_cmd
```

注意：将 `N` 替换为实际的无人机编号（1, 2, 3）

## 📊 工作流程

### Phase 1: 启动准备
1. **地面站**：启动 `multi_drone_signal_optimizer`
2. **Jetson**：每架无人机启动 `fast_scan_node` + `velocity_control`
3. **系统**：检查 ROS2 topics 连接

### Phase 2: 手动起飞
```
操作员用遥控器：
1. 解锁（ARM）无人机
2. 手动起飞到安全高度（建议 5m 以上）
3. 手动切换到 Offboard 模式（模式开关）
```

### Phase 3: 自动优化（每架无人机独立）
```
系统自动流程（每架无人机）：

1. 记录起飞位置（切入 Offboard 时的位置）
   ├─ takeoff_position = current_position
   └─ 设定 3m³ 搜索边界

2. 悬停等待信号（初始状态）
   ├─ 速度 = (0, 0, 0)
   └─ 100Hz 持续发布

3. 扫描循环（120秒/轮）
   ├─ fast_scan_node 扫描 Tracker A (45s)
   ├─ 延迟 (30s)
   ├─ fast_scan_node 扫描 Tracker B (45s)
   └─ 发布 /drone_N/link_quality

4. 等待两个信号都到达
   ├─ tracker_a_ready == True
   ├─ tracker_b_ready == True
   └─ 触发决策

5. 计算速度指令（独立线程）
   ├─ 优先上升到 1.5m
   ├─ 信号改善 → 继续方向
   ├─ 信号变差 → XY 随机搜索
   └─ 边界限制（3m³）

6. 发布速度（100Hz 线程）
   ├─ /drone_N/offboard_velocity_cmd (Twist)
   └─ velocity_control 转换为 TrajectorySetpoint

7. 移动并悬停
   ├─ 执行移动
   ├─ 移动计数 +1
   ├─ 重置 tracker_ready flags
   └─ 悬停等待下一轮扫描

8. 循环 5 次后停止
   └─ movement_count >= 5 → 悬停在最佳位置
```

### Phase 4: 手动降落
```
操作员用遥控器：
1. 切回手动模式（Position/Manual）
2. 手动降落
3. 上锁（DISARM）
```

## 🎯 算法详解

### 信号质量评分
```python
# RSSI 归一化（-100 to -20 dBm）
rssi_avg = (forward_rssi + return_rssi) / 2.0
rssi_score = (rssi_avg + 100) / 80.0  # 0-1

# SNR 归一化（-10 to 20 dB）
snr_avg = (forward_snr + return_snr) / 2.0
snr_score = (snr_avg + 10) / 30.0  # 0-1

# 综合评分（50% RSSI + 50% SNR）
quality = 0.5 * rssi_score + 0.5 * snr_score
```

### 移动策略（优先上升）
```python
# 阶段 1: 优先上升到 1.5m
if current_altitude < 1.5m:
    if 信号改善:
        vz = -0.3  # 正常上升
    else:
        vz = -0.2  # 慢速上升

# 阶段 2: 到达高度后精调 XY
else:
    if 信号改善:
        继续当前方向（vx, vy, 小幅 vz）
    else:
        XY 随机搜索（-0.2 to 0.2 m/s）
```

### 边界限制（绝对坐标）
```python
# 相对于 takeoff_position
X: ±1.5m
Y: ±1.5m
Z: 0 to +3m（只能上升，不能下降）

# 速度限制
MAX_VELOCITY = 0.3 m/s
```

## 🔧 参数配置

### ROS2 参数
```yaml
num_drones: 3              # 无人机数量
drone_ids: [1, 2, 3]       # 无人机ID列表
```

### 代码常量（可在代码中修改）
```python
MAX_MOVEMENTS = 5          # 最多移动 5 次
MAX_VELOCITY = 0.3         # 最大速度 0.3 m/s
ALTITUDE_THRESHOLD = 1.5   # 优先上升到 1.5m
PUBLISH_RATE = 100.0       # 100Hz 发布
BOUNDS_X = (-1.5, 1.5)     # X 边界
BOUNDS_Y = (-1.5, 1.5)     # Y 边界
BOUNDS_Z = (0.0, 3.0)      # Z 边界（只上升）
```

## 🎨 RViz2 可视化

**重要**：RViz2 在笔记本（地面站）上显示，不在 Jetson 上！

### 你会看到什么？

#### 1. **无人机**（彩色球体）
- **颜色**：
  - 🔴 红色 = 信号质量差（quality < 0.5）
  - 🟡 黄色 = 信号质量中等（quality ≈ 0.5）
  - 🟢 绿色 = 信号质量好（quality > 0.5）
- **文本标签**（4行显示）：
  - `Drone N`
  - `Q: 0.XX`（当前质量分数）
  - `TrackerA: !trackerNA`
  - `TrackerB: !trackerNB`

#### 2. **运动轨迹**（蓝色线条）
- 每架无人机的历史飞行路径
- 最多显示最近 100 个点
- 半透明效果

#### 3. **搜索边界**（彩色线框，每架无人机独立）
- **每架无人机都有自己的 3m × 3m × 3m 搜索空间**
- 中心：该无人机的起飞位置（takeoff_position）
- 颜色区分：
  - 🟦 Drone 1: 青色边框
  - 🟪 Drone 2: 洋红边框
  - 🟨 Drone 3: 黄色边框
- 边界中心有文本标签：`Drone N 搜索范围`
- **重要**：每架无人机只在自己的边界内移动！

#### 4. **速度向量**（黄色箭头）
- 当前速度指令方向
- 箭头长度 = 速度大小（放大 2 倍显示）
- 只在无人机移动时显示

### Tracker 绑定架构
```
总共 6 个 Meshtastic Tracker（地面信标）：
┌──────────┬─────────────────┐
│ Drone 1  │ Tracker 1A, 1B  │
│ Drone 2  │ Tracker 2A, 2B  │
│ Drone 3  │ Tracker 3A, 3B  │
└──────────┴─────────────────┘
```
每架无人机的 Jetson 只扫描绑定的两个 Tracker，信号质量基于这两个 Tracker 的平均值。

### 操作技巧

#### 视角控制
- **左键拖拽**：旋转视角
- **中键拖拽**：平移视角
- **滚轮**：缩放

#### 调整显示
1. 左侧 `Displays` 面板
2. 展开 `MarkerArray`
3. 勾选/取消勾选命名空间：
   - `drones`：无人机球体
   - `drone_ids`：文本标签
   - `trajectories`：轨迹线
   - `search_bounds_drone_N`：每架无人机的边界框（彩色）
   - `bounds_label_drone_N`：边界标签
   - `trackers`：地面信标
   - `tracker_labels`：信标标签
   - `velocities`：速度箭头

### RViz2 界面示意图

```
┌─────────────────────────────────────────────────────┐
│          RViz2 - 多无人机信号优化系统                │
├─────────────────────────────────────────────────────┤
│                                                     │
│    🟠 Tracker A (!e2e5b7c4)                         │
│                                                     │
│         ┌─────────┐                                │
│         │ 🟦青色  │  ← Drone 1 的搜索边界           │
│    🟢   │ 框线    │                                │
│  Drone 1│         │                                │
│  Q: 0.82└─────────┘                                │
│    ━━━━━━ (蓝色轨迹)                               │
│                                                     │
│                    ┌─────────┐                     │
│                    │ 🟪洋红  │  ← Drone 2 的边界    │
│              🟡    │ 框线    │                     │
│            Drone 2 │         │                     │
│            Q: 0.65 └─────────┘                     │
│               ↗ (黄色速度箭头)                      │
│                                                     │
│                           ┌─────────┐              │
│                           │ 🟨黄色  │  ← Drone 3   │
│                      🔴   │ 框线    │   的边界      │
│                   Drone 3 │         │              │
│                   Q: 0.32 └─────────┘              │
│                                                     │
│    🟠 Tracker B (!e2e5b8f8)                         │
│                                                     │
└─────────────────────────────────────────────────────┘

重点说明：
✅ 每架无人机有独立的彩色边界框（3m³）
✅ 边界框中心 = 该无人机的起飞位置
✅ 无人机只在自己的边界内搜索
✅ 所有无人机都扫描相同的两个 Tracker
```

## 📡 Topic 接口

### 订阅（每架无人机）
```
/drone_N/link_quality                      (std_msgs/String)
  - JSON 格式: {target_id, forward_rssi, forward_snr, ...}

/drone_N/fmu/out/vehicle_local_position    (px4_msgs/VehicleLocalPosition)
  - NED 坐标系位置
```

### 发布（每架无人机）
```
/drone_N/offboard_velocity_cmd             (geometry_msgs/Twist)
  - linear.x: vx (NED)
  - linear.y: vy (NED)
  - linear.z: vz (NED, 负值=上升)
  - angular.z: yaw_rate
```

## 🧵 多线程架构

### 每架无人机的线程
```
Drone N:
  ├─ 订阅回调（ReentrantCallbackGroup）
  │   ├─ link_quality_callback()      # 接收信号数据
  │   └─ position_callback()          # 接收位置数据
  │
  ├─ 决策线程（独立 Python 线程）
  │   ├─ 等待两个信号都到达
  │   ├─ 计算速度指令
  │   ├─ 更新状态
  │   └─ 重置 tracker flags
  │
  └─ 速度发布线程（独立 Python 线程）
      ├─ 100Hz 持续发布
      ├─ 读取 drone_state.current_velocity
      └─ 发布到 /drone_N/offboard_velocity_cmd
```

### 线程安全
```python
# 每个 DroneState 都有独立的锁
with state.lock:
    # 修改状态
    state.current_velocity = (vx, vy, vz)
    state.movement_count += 1
```

## 🛡️ 安全机制

### 1. 遥控器优先
- RC 随时可以切回手动模式
- 系统检测到非 Offboard 模式 → 停止发布速度

### 2. 边界保护
- 硬限制：3m³ 绝对边界
- Z 轴只能上升，不能下降
- 速度超限自动裁剪

### 3. Offboard 保活
- 100Hz 持续发布速度指令
- 防止 PX4 500ms timeout failsafe

### 4. 移动次数限制
- 最多 5 次移动
- 达到限制后自动悬停

## 📈 监控与调试

### 查看日志
```bash
# 查看优化器日志
ros2 topic echo /rosout | grep multi_drone

# 查看单个无人机状态
ros2 topic echo /drone_1/offboard_velocity_cmd

# 查看信号数据
ros2 topic echo /drone_1/link_quality
```

### 实时监控
```bash
# 监控所有 topics
ros2 topic list

# 监控特定无人机
watch -n 0.5 'ros2 topic echo /drone_1/offboard_velocity_cmd --once'
```

## 🐛 故障排查

### 问题 1: 无人机不移动
**检查**:
- [ ] 是否切换到 Offboard 模式？
- [ ] `link_quality` topic 是否有数据？
- [ ] 两个 tracker 是否都扫描成功？
- [ ] 是否达到 5 次移动限制？

**解决**:
```bash
# 检查 topic 连接
ros2 topic list | grep drone_1

# 检查信号数据
ros2 topic echo /drone_1/link_quality
```

### 问题 2: 多架无人机不协同
**检查**:
- [ ] 每架 Jetson 的 topic 名称是否正确（/drone_N/...）？
- [ ] ROS2 网络是否连通？
- [ ] 决策线程是否都启动？

**解决**:
```bash
# 检查 ROS2 网络
ros2 node list

# 检查线程状态（查看日志）
# 应该看到 "Drone N: 决策线程启动"
```

### 问题 3: 漂移超出边界
**检查**:
- [ ] `takeoff_position` 是否正确记录？
- [ ] 边界检查逻辑是否生效？
- [ ] PX4 EKF 是否稳定？

**解决**:
- 降低 `MAX_VELOCITY`（改为 0.2 m/s）
- 检查 PX4 位置估计质量

## 📝 实验记录建议

### 每次飞行记录
```
日期: ___________
无人机: Drone N
起飞位置: (x, y, z)
天气: ___________

移动记录:
移动1: 质量 ___ → ___, 速度 (vx, vy, vz)
移动2: 质量 ___ → ___, 速度 (vx, vy, vz)
...

最佳位置: (x, y, z)
最佳质量: ___
备注: ___________
```

## 📚 相关文件

- `multi_drone_signal_optimizer.py`: 主程序
- `fast_scan_node.py`: Meshtastic 扫描器
- `velocity_control.py`: 速度控制器（ARK版本）
- `launch_multi_drone_optimizer.sh`: 启动脚本

## ⚠️ 注意事项

1. **安全第一**：遥控器随时准备接管
2. **网络延迟**：WiFi 不稳定会影响协同
3. **电池监控**：120秒扫描周期耗电较大
4. **信号范围**：确保 Meshtastic 在有效范围内
5. **多机间隔**：建议无人机间距 > 3m 避免冲突

---

**版本**: v1.0  
**更新**: 2025-11-12  
**作者**: Multi-Drone Signal Optimizer Team
