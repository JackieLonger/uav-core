# ROS2 Topic 订阅关系与信息流

## 🌐 系统拓扑图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          笔记本（地面站）                                      │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              multi_drone_signal_optimizer.py                          │  │
│  │              (决策中心 - 多线程异步处理)                                │  │
│  │                                                                         │  │
│  │  订阅 (Subscribers):                                                   │  │
│  │    • /drone_1/link_quality                                            │  │
│  │    • /drone_2/link_quality                                            │  │
│  │    • /drone_3/link_quality                                            │  │
│  │    • /drone_1/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_2/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_3/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_1/fmu/out/vehicle_status                                 │  │
│  │    • /drone_2/fmu/out/vehicle_status                                 │  │
│  │    • /drone_3/fmu/out/vehicle_status                                 │  │
│  │                                                                         │  │
│  │  发布 (Publishers):                                                    │  │
│  │    • /drone_1/offboard_velocity_cmd   (100Hz)                        │  │
│  │    • /drone_2/offboard_velocity_cmd   (100Hz)                        │  │
│  │    • /drone_3/offboard_velocity_cmd   (100Hz)                        │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              multi_drone_visualizer.py                                │  │
│  │              (RViz2 可视化 - 实时显示)                                 │  │
│  │                                                                         │  │
│  │  订阅 (Subscribers):                                                   │  │
│  │    • /drone_1/link_quality                                            │  │
│  │    • /drone_2/link_quality                                            │  │
│  │    • /drone_3/link_quality                                            │  │
│  │    • /drone_1/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_2/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_3/fmu/out/vehicle_local_position                         │  │
│  │    • /drone_1/offboard_velocity_cmd                                  │  │
│  │    • /drone_2/offboard_velocity_cmd                                  │  │
│  │    • /drone_3/offboard_velocity_cmd                                  │  │
│  │                                                                         │  │
│  │  发布 (Publishers):                                                    │  │
│  │    • /visualization_marker_array      (50Hz)                         │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              RViz2                                                     │  │
│  │              (3D 可视化界面)                                            │  │
│  │                                                                         │  │
│  │  订阅 (Subscribers):                                                   │  │
│  │    • /visualization_marker_array                                      │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                ↕ (WiFi/DDS - ROS2 网络)
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Jetson N (机载 - 每架无人机)                         │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              fast_scan_node.py                                        │  │
│  │              (Meshtastic 扫描器)                                       │  │
│  │                                                                         │  │
│  │  硬件接口:                                                              │  │
│  │    • USB → Meshtastic Heltec Tracker V3                              │  │
│  │    • 扫描绑定的 TrackerNA & TrackerNB                                  │  │
│  │                                                                         │  │
│  │  发布 (Publishers):                                                    │  │
│  │    • /drone_N/link_quality            (每 2 秒)                       │  │
│  │       格式: JSON String                                                │  │
│  │       内容: {target_id, forward_rssi, forward_snr,                    │  │
│  │              return_rssi, return_snr}                                 │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              velocity_control.py                                      │  │
│  │              (速度控制转换器)                                           │  │
│  │                                                                         │  │
│  │  订阅 (Subscribers):                                                   │  │
│  │    • /drone_N/offboard_velocity_cmd                                   │  │
│  │       格式: geometry_msgs/Twist                                        │  │
│  │       内容: linear(vx,vy,vz), angular(yaw_rate)                       │  │
│  │                                                                         │  │
│  │  发布 (Publishers):                                                    │  │
│  │    • /fmu/in/trajectory_setpoint      (100Hz)                        │  │
│  │    • /fmu/in/offboard_control_mode    (100Hz)                        │  │
│  │    • /fmu/in/vehicle_command          (根据需要)                       │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                ↕ (UART - MicroXRCE-DDS Agent)
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Pixhawk 6C (飞控)                                   │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │              PX4 v1.14.3 Autopilot                                    │  │
│  │              (MAVLink 2.0 + MicroXRCE-DDS)                            │  │
│  │                                                                         │  │
│  │  订阅 (Subscribers - 来自 Jetson):                                     │  │
│  │    • /fmu/in/trajectory_setpoint                                      │  │
│  │    • /fmu/in/offboard_control_mode                                    │  │
│  │    • /fmu/in/vehicle_command                                          │  │
│  │                                                                         │  │
│  │  发布 (Publishers - 发送到 Jetson):                                    │  │
│  │    • /fmu/out/vehicle_local_position  (50Hz)                         │  │
│  │    • /fmu/out/vehicle_status          (1Hz)                          │  │
│  │    • /fmu/out/vehicle_attitude        (50Hz)                         │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📊 详细 Topic 信息表

### 1️⃣ 信号质量数据流

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/drone_N/link_quality` | `std_msgs/String` | `fast_scan_node` (Jetson) | `multi_drone_signal_optimizer` (地面站)<br>`multi_drone_visualizer` (地面站) | 0.5 Hz<br>(每 2 秒) | JSON: `{target_id, forward_rssi, forward_snr, return_rssi, return_snr}` |

**数据示例**:
```json
{
  "target_id": "!tracker1A",
  "forward_rssi": -85,
  "forward_snr": 9.5,
  "return_rssi": -88,
  "return_snr": 8.2
}
```

**信息流向**:
```
Meshtastic Tracker → USB → Jetson (fast_scan_node)
    ↓ 发布 /drone_N/link_quality
笔记本 (multi_drone_signal_optimizer) ← WiFi/DDS
```

---

### 2️⃣ 速度指令数据流

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/drone_N/offboard_velocity_cmd` | `geometry_msgs/Twist` | `multi_drone_signal_optimizer` (地面站) | `velocity_control` (Jetson)<br>`multi_drone_visualizer` (地面站) | 100 Hz | `linear: (vx, vy, vz)`<br>`angular: (yaw_rate)` |

**数据示例**:
```yaml
linear:
  x: 0.2    # vx (m/s, NED)
  y: -0.15  # vy (m/s, NED)
  z: -0.3   # vz (m/s, NED, 负=上升)
angular:
  x: 0.0
  y: 0.0
  z: 0.0    # yaw_rate (rad/s)
```

**信息流向**:
```
笔记本 (multi_drone_signal_optimizer)
    ↓ 发布 /drone_N/offboard_velocity_cmd (100Hz)
Jetson (velocity_control) ← WiFi/DDS
    ↓ 转换为 TrajectorySetpoint
Pixhawk (PX4) ← UART/MicroXRCE
```

---

### 3️⃣ 位置数据流

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/drone_N/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 (Pixhawk) | `multi_drone_signal_optimizer` (地面站)<br>`multi_drone_visualizer` (地面站) | 50 Hz | NED 坐标系位置<br>`x, y, z` (m)<br>`vx, vy, vz` (m/s) |

**数据示例**:
```yaml
x: 1.234        # NED X (m)
y: -0.567       # NED Y (m)
z: 3.890        # NED Z (m, 负=高度)
vx: 0.15        # X 速度
vy: -0.08       # Y 速度
vz: 0.02        # Z 速度
```

**信息流向**:
```
Pixhawk (PX4 EKF)
    ↓ 通过 MicroXRCE-DDS Agent
Jetson (ROS2 网络)
    ↓ 通过 WiFi/DDS
笔记本 (multi_drone_signal_optimizer, visualizer)
```

---

### 4️⃣ 状态数据流

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/drone_N/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | PX4 (Pixhawk) | `multi_drone_signal_optimizer` (地面站) | 1 Hz | `arming_state`<br>`nav_state`<br>`failsafe` |

**重要字段**:
```yaml
arming_state: 2    # 2 = ARMED
nav_state: 14      # 14 = Offboard Mode
failsafe: false
```

**用途**: 
- 优化器检测 Offboard 模式激活
- 记录起飞位置（切入 Offboard 时）
- 监控无人机状态

---

### 5️⃣ PX4 控制指令数据流（Jetson → Pixhawk）

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | `velocity_control` (Jetson) | PX4 (Pixhawk) | 100 Hz | 速度设定点<br>`velocity[3]`<br>`yaw, yawspeed` |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | `velocity_control` (Jetson) | PX4 (Pixhawk) | 100 Hz | 控制模式标志<br>`velocity: true` |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | `velocity_control` (Jetson) | PX4 (Pixhawk) | 根据需要 | ARM/DISARM 命令 |

---

### 6️⃣ RViz2 可视化数据流

| Topic | 类型 | 发布者 | 订阅者 | 频率 | 内容 |
|-------|------|--------|--------|------|------|
| `/visualization_marker_array` | `visualization_msgs/MarkerArray` | `multi_drone_visualizer` (地面站) | RViz2 | 50 Hz | Markers:<br>- 无人机球体<br>- 文本标签<br>- 轨迹线<br>- 边界框<br>- 速度箭头 |

**Marker 类型**:
- `SPHERE`: 无人机位置（颜色=信号质量）
- `TEXT_VIEW_FACING`: 文本标签
- `LINE_STRIP`: 飞行轨迹
- `LINE_LIST`: 边界框
- `ARROW`: 速度向量

---

## 🔄 完整信息流时序图

```
时间轴:

t=0s
┌─ Jetson (fast_scan_node) 启动
│  └─ 开始扫描 Meshtastic Tracker
│
├─ Pixhawk (PX4) 启动
│  └─ 发布 /fmu/out/vehicle_local_position (50Hz)
│  └─ 发布 /fmu/out/vehicle_status (1Hz)
│
└─ 地面站 (multi_drone_signal_optimizer) 启动
   └─ 订阅 /drone_N/link_quality
   └─ 订阅 /drone_N/fmu/out/vehicle_local_position
   └─ 订阅 /drone_N/fmu/out/vehicle_status
   └─ 启动决策线程 (每机一个)
   └─ 启动速度发布线程 (100Hz)

---

t=2s (首次信号扫描)
Jetson (fast_scan_node):
  ├─ 扫描 TrackerA → 获取 RSSI/SNR
  └─ 发布 /drone_N/link_quality (JSON)
      ↓
地面站 (multi_drone_signal_optimizer):
  ├─ 收到 link_quality 数据
  ├─ 解析 JSON
  ├─ 计算信号质量分数
  └─ 等待两个 Tracker 都到达

---

t=4s (第二个 Tracker 扫描)
Jetson (fast_scan_node):
  ├─ 扫描 TrackerB → 获取 RSSI/SNR
  └─ 发布 /drone_N/link_quality (JSON)
      ↓
地面站 (multi_drone_signal_optimizer):
  ├─ 收到第二个 Tracker 数据
  ├─ 计算平均质量分数
  ├─ 决策线程触发
  ├─ 计算速度指令 (vx, vy, vz)
  └─ 速度发布线程发布 /drone_N/offboard_velocity_cmd (100Hz)
      ↓
Jetson (velocity_control):
  ├─ 收到 Twist 速度指令
  ├─ 转换为 TrajectorySetpoint (NED)
  └─ 发布 /fmu/in/trajectory_setpoint (100Hz)
      ↓
Pixhawk (PX4):
  ├─ 收到 TrajectorySetpoint
  ├─ 执行速度控制
  └─ 无人机开始移动

---

t=每 0.01s (100Hz 循环)
地面站 (multi_drone_signal_optimizer):
  └─ 发布 /drone_N/offboard_velocity_cmd
      ↓
Jetson (velocity_control):
  ├─ 转换 Twist → TrajectorySetpoint
  └─ 发布 /fmu/in/trajectory_setpoint
  └─ 发布 /fmu/in/offboard_control_mode
      ↓
Pixhawk (PX4):
  └─ 保持 Offboard 连接 (防止 500ms 超时)

---

t=每 0.02s (50Hz 循环)
Pixhawk (PX4):
  └─ 发布 /fmu/out/vehicle_local_position
      ↓
地面站 (visualizer):
  ├─ 收到位置更新
  ├─ 更新 Marker 位置
  ├─ 更新轨迹线
  └─ 发布 /visualization_marker_array
      ↓
RViz2:
  └─ 显示更新的可视化

---

t=收敛后
地面站 (multi_drone_signal_optimizer):
  ├─ 检测到收敛条件
  ├─ 设置 hold_mode = True
  └─ 速度发布线程发布 (0, 0, 0) (100Hz)
      ↓
Jetson (velocity_control):
  └─ 发布零速度到 Pixhawk
      ↓
Pixhawk (PX4):
  └─ 无人机悬停
```

---

## 🌐 网络拓扑

```
┌───────────────────────────────────────────────────────────────┐
│                    WiFi 路由器                                 │
│                 (ROS2 DDS 通信)                                │
└───────────────────────────────────────────────────────────────┘
         ↕                 ↕                 ↕                 ↕
┌────────────────┐ ┌────────────────┐ ┌────────────────┐ ┌────────────────┐
│  笔记本        │ │  Jetson 1      │ │  Jetson 2      │ │  Jetson 3      │
│  (地面站)      │ │  (Drone 1)     │ │  (Drone 2)     │ │  (Drone 3)     │
│                │ │                │ │                │ │                │
│  IP: 192.168.1.10 │  IP: 192.168.1.101 │  IP: 192.168.1.102 │  IP: 192.168.1.103
│                │ │      ↕         │ │      ↕         │ │      ↕         │
│                │ │  Pixhawk 1     │ │  Pixhawk 2     │ │  Pixhawk 3     │
│                │ │  (UART)        │ │  (UART)        │ │  (UART)        │
└────────────────┘ └────────────────┘ └────────────────┘ └────────────────┘

网络要求:
- 所有设备在同一子网 (192.168.1.0/24)
- ROS2 DDS 自动发现（Multicast）
- UDP 端口 7400-7500 开放
```

---

## 📋 Topic 检查命令

### 查看所有 Topic
```bash
ros2 topic list
```

**预期输出**（3 架无人机）:
```
/drone_1/link_quality
/drone_1/offboard_velocity_cmd
/drone_1/fmu/out/vehicle_local_position
/drone_1/fmu/out/vehicle_status
/drone_2/link_quality
/drone_2/offboard_velocity_cmd
/drone_2/fmu/out/vehicle_local_position
/drone_2/fmu/out/vehicle_status
/drone_3/link_quality
/drone_3/offboard_velocity_cmd
/drone_3/fmu/out/vehicle_local_position
/drone_3/fmu/out/vehicle_status
/visualization_marker_array
/fmu/in/trajectory_setpoint
/fmu/in/offboard_control_mode
/fmu/in/vehicle_command
```

### 查看 Topic 详情
```bash
ros2 topic info /drone_1/link_quality
```

**预期输出**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 2
```

### 查看发布频率
```bash
ros2 topic hz /drone_1/offboard_velocity_cmd
```

**预期输出**:
```
average rate: 100.000
  min: 0.010s max: 0.010s std dev: 0.00012s window: 100
```

### 实时监控数据
```bash
ros2 topic echo /drone_1/link_quality
```

---

## 🔍 故障排查

### 问题 1: `/drone_N/link_quality` 没有数据

**检查步骤**:
```bash
# 1. Jetson 上检查节点是否运行
ros2 node list | grep fast_scan

# 2. 检查 Topic 是否存在
ros2 topic list | grep link_quality

# 3. 检查发布者数量
ros2 topic info /drone_1/link_quality
# 应该显示: Publisher count: 1

# 4. 在 Jetson 上检查 Meshtastic 连接
meshtastic --info
```

---

### 问题 2: 地面站收不到信号数据

**检查步骤**:
```bash
# 1. 检查网络连通性
ping 192.168.1.101  # Jetson 1 IP

# 2. 检查 ROS2 节点发现
ros2 node list
# 应该看到来自 Jetson 和地面站的节点

# 3. 检查 Topic 订阅关系
ros2 topic info /drone_1/link_quality
# 应该显示: Subscription count: 2 (optimizer + visualizer)

# 4. 检查防火墙
sudo ufw status
# 确保 UDP 7400-7500 端口开放
```

---

### 问题 3: Pixhawk 不响应速度指令

**检查步骤**:
```bash
# 1. 检查 velocity_control 是否运行
ros2 node list | grep velocity_control

# 2. 检查是否收到速度指令
ros2 topic echo /drone_1/offboard_velocity_cmd --once

# 3. 检查 PX4 Topic 是否发布
ros2 topic list | grep "/fmu/in/"

# 4. 在 Jetson 上检查 MicroXRCE Agent
ps aux | grep MicroXRCEAgent
```

---

## 📊 数据流量统计

| 连接 | 方向 | 频率 | 带宽估算 |
|------|------|------|---------|
| Jetson → 地面站 | link_quality | 0.5 Hz | ~0.5 KB/s |
| 地面站 → Jetson | offboard_velocity_cmd | 100 Hz | ~10 KB/s |
| Pixhawk → Jetson | vehicle_local_position | 50 Hz | ~5 KB/s |
| Pixhawk → Jetson | vehicle_status | 1 Hz | ~0.1 KB/s |
| Jetson → Pixhawk | trajectory_setpoint | 100 Hz | ~10 KB/s |
| 地面站 → RViz2 | visualization_marker_array | 50 Hz | ~50 KB/s |

**总带宽（单机）**: ~75 KB/s  
**总带宽（3 机）**: ~225 KB/s

WiFi 完全足够，延迟 < 10ms。

---

**版本**: v1.0  
**更新**: 2025-11-12  
**适用**: uav-core (final_test_1 分支)
