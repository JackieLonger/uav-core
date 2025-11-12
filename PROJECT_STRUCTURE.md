# 项目结构与使用说明

## 📁 完整目录结构

```
uav-core/
├── 📚 文档 (Documentation)
│   ├── README.md                                    # 项目主文档（入口）
│   ├── QUICK_START.md                               # 快速开始（5分钟）
│   ├── MULTI_DRONE_OPTIMIZER_GUIDE.md               # 完整使用手册
│   ├── SINGLE_MACHINE_TESTING.md                    # 单机测试指南
│   ├── HOLD_MECHANISM_REFACTORING_CHINESE.md        # 技术文档（悬停机制）
│   ├── PROJECT_STRUCTURE.md                         # 本文件
│   └── ROS2_TOPIC_FLOW.md                          # Topic 订阅关系图
│
├── 🚀 启动脚本 (Launch Scripts)
│   ├── launch_jetson.sh                            # Jetson 机载启动（每台无人机）
│   ├── launch_multi_drone_optimizer.sh             # 地面站优化器启动
│   ├── launch_rviz.sh                              # RViz2 可视化启动
│   └── setup.sh                                    # 环境设置脚本
│
├── ⚙️ 配置文件 (Configuration)
│   └── thirdparty.repos                            # ROS2 依赖包列表
│
├── 📦 源代码 (Source Code)
│   └── src/
│       ├── ROS2_PX4_Offboard_Example/              # 主要 ROS2 包
│       │   ├── px4_offboard/                       # Python 节点目录
│       │   │   ├── __init__.py
│       │   │   ├── fast_scan_node.py               # ⭐ Meshtastic 扫描（Jetson）
│       │   │   ├── velocity_control.py             # ⭐ 速度控制（Jetson）
│       │   │   ├── multi_drone_signal_optimizer.py # ⭐ 优化器（地面站）
│       │   │   ├── multi_drone_visualizer.py       # ⭐ 可视化（地面站）
│       │   │   ├── diagnose_node.py                # 诊断工具
│       │   │   ├── fast_scan_node_advanced.py      # 高级扫描（备用）
│       │   │   └── fast_scan_node_textmsg.py       # 文本消息版（备用）
│       │   │
│       │   ├── launch/                             # Launch 文件
│       │   │   ├── jetson_onboard.launch.py        # ⭐ Jetson 启动配置
│       │   │   ├── multi_drone_optimizer.launch.py # ⭐ 地面站启动配置
│       │   │   └── offboard_velocity_control.launch.py
│       │   │
│       │   ├── resource/                           # 资源文件
│       │   │   └── multi_drone.rviz                # ⭐ RViz2 配置文件
│       │   │
│       │   ├── setup.py                            # Python 包配置
│       │   ├── package.xml                         # ROS2 包清单
│       │   └── CMakeLists.txt                      # 编译配置
│       │
│       ├── px4_msgs/                               # PX4 消息定义（子模块）
│       └── px4_ros_com/                            # PX4-ROS2 桥接（子模块）
│
├── 🔨 编译输出 (Build Output)
│   ├── build/                                      # 编译中间文件（忽略）
│   ├── install/                                    # 安装文件（忽略）
│   └── log/                                        # 编译日志（忽略）
│
└── 🖼️ 其他
    └── jpg/                                        # 实验概念图片
```

---

## 🎯 核心文件功能说明

### 📄 文档文件（按阅读顺序）

| 文件 | 用途 | 目标读者 | 阅读时间 |
|------|------|---------|---------|
| **README.md** | 项目概述、安装步骤、快速导航 | 所有人（首次阅读） | 10 分钟 |
| **QUICK_START.md** | 快速命令参考 | 熟悉项目的用户 | 2 分钟 |
| **SINGLE_MACHINE_TESTING.md** | 单机测试流程 | 测试人员 | 15 分钟 |
| **MULTI_DRONE_OPTIMIZER_GUIDE.md** | 完整系统手册 | 所有用户（详细了解） | 30 分钟 |
| **HOLD_MECHANISM_REFACTORING_CHINESE.md** | 悬停机制技术细节 | 开发者 | 20 分钟 |
| **PROJECT_STRUCTURE.md** | 项目结构说明（本文件） | 开发者/维护者 | 10 分钟 |
| **ROS2_TOPIC_FLOW.md** | Topic 订阅关系图 | 开发者/调试人员 | 10 分钟 |

---

### 🚀 启动脚本

#### `launch_jetson.sh` ⭐ 核心脚本
**位置**: 项目根目录  
**运行环境**: Jetson Orin Nano（机载电脑）  
**功能**: 启动单架无人机的所有机载节点

**使用方法**:
```bash
cd ~/uav-core
./launch_jetson.sh [drone_id] [tracker_a_id] [tracker_b_id]

# 示例：
./launch_jetson.sh 1 !tracker1A !tracker1B
./launch_jetson.sh 2 !tracker2A !tracker2B
./launch_jetson.sh 3 !tracker3A !tracker3B
```

**启动内容**:
- `fast_scan_node.py` - Meshtastic 扫描节点
- `velocity_control.py` - 速度控制节点
- 自动配置 topic 命名空间 (`/drone_N/`)

---

#### `launch_multi_drone_optimizer.sh`
**位置**: 项目根目录  
**运行环境**: 笔记本（地面站）  
**功能**: 启动多机优化器

**使用方法**:
```bash
cd ~/uav-core
./launch_multi_drone_optimizer.sh [num_drones]

# 示例：
./launch_multi_drone_optimizer.sh 3  # 3 架无人机
./launch_multi_drone_optimizer.sh 1  # 单机测试
```

**启动内容**:
- `multi_drone_signal_optimizer.py` - 决策中心
- 自动订阅所有无人机的 `/drone_N/link_quality`
- 发布速度指令到 `/drone_N/offboard_velocity_cmd`

---

#### `launch_rviz.sh`
**位置**: 项目根目录  
**运行环境**: 笔记本（地面站）  
**功能**: 启动 RViz2 可视化

**使用方法**:
```bash
cd ~/uav-core
./launch_rviz.sh
```

**启动内容**:
- `multi_drone_visualizer.py` - 可视化节点
- RViz2 - 3D 可视化界面
- 自动加载配置文件 `multi_drone.rviz`

---

### 🐍 Python 节点

#### `fast_scan_node.py` ⭐ 机载节点
**位置**: `src/ROS2_PX4_Offboard_Example/px4_offboard/`  
**运行环境**: Jetson（每架无人机）  
**功能**: 扫描绑定的 Meshtastic Tracker 并发布链路质量

**ROS2 参数**:
```python
tracker_a_id: str   # Tracker A 的 Meshtastic ID
tracker_b_id: str   # Tracker B 的 Meshtastic ID
```

**发布 Topic**:
- `/drone_N/link_quality` (std_msgs/String)
  - JSON 格式: `{target_id, forward_rssi, forward_snr, return_rssi, return_snr}`

**扫描周期**: 每 2 秒扫描一次（每个 Tracker）

---

#### `velocity_control.py` - 机载节点
**位置**: `src/ROS2_PX4_Offboard_Example/px4_offboard/`  
**运行环境**: Jetson（每架无人机）  
**功能**: 转换 Twist 速度指令为 PX4 TrajectorySetpoint

**订阅 Topic**:
- `/drone_N/offboard_velocity_cmd` (geometry_msgs/Twist)

**发布 Topic**:
- `/fmu/in/trajectory_setpoint` (px4_msgs/TrajectorySetpoint)
- `/fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode)
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand)

**转换逻辑**:
```
Twist (vx, vy, vz) → TrajectorySetpoint (NED 坐标系)
```

---

#### `multi_drone_signal_optimizer.py` ⭐ 地面站节点
**位置**: `src/ROS2_PX4_Offboard_Example/px4_offboard/`  
**运行环境**: 笔记本（地面站）  
**功能**: 多机协同优化决策中心

**ROS2 参数**:
```python
num_drones: int         # 无人机数量（默认: 3）
drone_ids: list[int]    # 无人机 ID 列表（默认: [1, 2, 3]）
```

**每架无人机的线程**:
- 决策线程: 等待信号 → 计算速度 → 更新状态
- 速度发布线程: 100Hz 持续发布速度指令

**订阅 Topic**（每架无人机）:
- `/drone_N/link_quality` (std_msgs/String)
- `/drone_N/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition)
- `/drone_N/fmu/out/vehicle_status` (px4_msgs/VehicleStatus)

**发布 Topic**（每架无人机）:
- `/drone_N/offboard_velocity_cmd` (geometry_msgs/Twist)

---

#### `multi_drone_visualizer.py` - 地面站节点
**位置**: `src/ROS2_PX4_Offboard_Example/px4_offboard/`  
**运行环境**: 笔记本（地面站）  
**功能**: RViz2 实时可视化

**ROS2 参数**:
```python
num_drones: int                     # 无人机数量
drone_ids: list[int]                # 无人机 ID 列表
drone_tracker_bindings: dict        # Tracker 绑定关系
```

**订阅 Topic**（每架无人机）:
- `/drone_N/link_quality` (std_msgs/String)
- `/drone_N/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition)
- `/drone_N/offboard_velocity_cmd` (geometry_msgs/Twist)

**发布 Topic**:
- `/visualization_marker_array` (visualization_msgs/MarkerArray)

**显示内容**:
- 无人机球体（颜色表示信号质量）
- 文本标签（Drone ID、质量分数、Tracker 绑定）
- 飞行轨迹（蓝色线条）
- 搜索边界（彩色框，每机独立）
- 速度向量（黄色箭头）

---

## 🔄 使用流程

### 步骤 1: 编译项目（首次/修改后）

```bash
cd ~/uav-core
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

### 步骤 2: Jetson 启动（每台独立）

```bash
# Jetson 1
cd ~/uav-core
source install/setup.bash
./launch_jetson.sh 1 !tracker1A !tracker1B

# Jetson 2
./launch_jetson.sh 2 !tracker2A !tracker2B

# Jetson 3
./launch_jetson.sh 3 !tracker3A !tracker3B
```

### 步骤 3: 地面站启动

```bash
# 终端 1: 优化器
cd ~/uav-core
source install/setup.bash
./launch_multi_drone_optimizer.sh 3

# 终端 2: 可视化
./launch_rviz.sh
```

### 步骤 4: 飞行操作

1. 使用遥控器手动起飞（每架无人机）
2. 切换到 Offboard 模式
3. 系统自动开始优化
4. 完成后手动降落

---

## 🛠️ 开发/调试工具

### 查看 Topic 列表
```bash
ros2 topic list
```

### 监控特定 Topic
```bash
# 监控信号质量
ros2 topic echo /drone_1/link_quality

# 监控速度指令
ros2 topic echo /drone_1/offboard_velocity_cmd

# 监控位置
ros2 topic echo /drone_1/fmu/out/vehicle_local_position
```

### 检查节点状态
```bash
ros2 node list
ros2 node info /multi_drone_signal_optimizer
```

### 查看参数
```bash
ros2 param list /multi_drone_signal_optimizer
ros2 param get /multi_drone_signal_optimizer num_drones
```

### 实时监控（多个窗口）
```bash
# 窗口 1: 优化器日志
ros2 run px4_offboard multi_drone_signal_optimizer

# 窗口 2: 信号数据
watch -n 0.5 'ros2 topic echo /drone_1/link_quality --once'

# 窗口 3: 速度指令
watch -n 0.5 'ros2 topic echo /drone_1/offboard_velocity_cmd --once'

# 窗口 4: 系统日志
ros2 topic echo /rosout | grep -i "drone_1"
```

---

## 📝 修改代码后的流程

```bash
# 1. 修改代码（例如修改 fast_scan_node.py）
vim src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py

# 2. 重新编译
cd ~/uav-core
colcon build --merge-install

# 3. 重新加载环境
source install/setup.bash

# 4. 重启节点
# Jetson: Ctrl+C 停止，然后重新运行 ./launch_jetson.sh
# 地面站: Ctrl+C 停止，然后重新运行 ./launch_multi_drone_optimizer.sh
```

---

## 🔍 常用命令速查

```bash
# === 编译相关 ===
colcon build --merge-install                    # 编译所有包
colcon build --packages-select px4_offboard     # 只编译特定包
source install/setup.bash                       # 加载环境

# === Topic 相关 ===
ros2 topic list                                 # 列出所有 topic
ros2 topic echo /topic_name                     # 查看 topic 内容
ros2 topic hz /topic_name                       # 查看发布频率
ros2 topic info /topic_name                     # 查看 topic 详情

# === 节点相关 ===
ros2 node list                                  # 列出所有节点
ros2 node info /node_name                       # 查看节点详情
ros2 run pkg_name node_name                     # 手动运行节点

# === 参数相关 ===
ros2 param list /node_name                      # 列出节点参数
ros2 param get /node_name param_name            # 获取参数值
ros2 param set /node_name param_name value      # 设置参数值

# === 调试相关 ===
ros2 wtf                                        # 系统诊断
ros2 doctor                                     # 健康检查
```

---

## 📂 忽略的文件/目录（不要修改）

```
build/          # 编译中间文件
install/        # 安装文件
log/            # 日志文件
__pycache__/    # Python 缓存
*.pyc           # Python 编译文件
.vscode/        # VS Code 配置
```

这些目录由 `.gitignore` 管理，不会提交到 Git。

---

## 🎓 学习路径建议

1. **新手**: 
   - 阅读 README.md（了解项目）
   - 阅读 QUICK_START.md（快速上手）
   - 运行单机测试（SINGLE_MACHINE_TESTING.md）

2. **使用者**:
   - 阅读 MULTI_DRONE_OPTIMIZER_GUIDE.md（完整手册）
   - 运行多机实验
   - 查看 ROS2_TOPIC_FLOW.md（理解数据流）

3. **开发者**:
   - 阅读本文件（PROJECT_STRUCTURE.md）
   - 阅读 HOLD_MECHANISM（技术细节）
   - 修改代码并测试

---

**版本**: v1.0  
**更新**: 2025-11-12  
**适用**: uav-core (final_test_1 分支)
