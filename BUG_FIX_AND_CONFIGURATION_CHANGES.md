# Bug修复与配置简化说明

## 修改日期
2025年1月（最终测试前）

## 问题描述

### 1. 速度重置Bug（Critical）
**问题**：无人机移动后没有重置速度为零，导致持续移动而不是悬停等待下一轮扫描。

**现象**：
- 计算出速度后，无人机开始移动
- 移动应该持续约3秒后停止
- 实际情况：速度未重置为(0,0,0)，导致无人机持续以最后的速度移动
- 无法实现"移动→悬停→等待扫描→再移动"的循环

**根本原因**：
- `velocity_publish_thread` 持续以100Hz频率发布 `drone_state.current_velocity`
- `decision_thread` 计算新速度后更新了 `current_velocity`
- 但移动完成后没有重置为 `(0.0, 0.0, 0.0)`

### 2. Tracker ID配置问题
**问题**：Jetson终端无法正确处理"!"符号。

**现象**：
- 通过命令行传递 Tracker ID（如 `!e2e5b7c4`）时
- Jetson的bash shell无法正确处理"!"字符
- 导致启动失败或参数传递错误

**解决方案**：
- 将Tracker ID从启动参数改为在代码中硬编码
- 每架无人机的Jetson需手动修改 `fast_scan_node.py`

---

## 修改内容

### 文件1：`multi_drone_signal_optimizer.py`

**位置**：`src/ROS2_PX4_Offboard_Example/px4_offboard/multi_drone_signal_optimizer.py`

**修改位置**：`decision_thread` 函数（约510行）

#### 原代码（有Bug）
```python
# 更新速度
with state.lock:
    state.current_velocity = (vx, vy, vz)
    state.movement_count += 1

# 重置 tracker flags
state.reset_tracker_flags()

self.get_logger().info(
    f"Drone {drone_id}: 移动 {state.movement_count}/{self.MAX_MOVEMENTS}, "
    f"速度 ({vx:.2f}, {vy:.2f}, {vz:.2f}), "
    f"质量 {state.current_quality:.3f}"
)

# 等待下一轮扫描（约 120 秒）
time.sleep(2.0)
```

#### 修正后代码（已修复）
```python
# 更新速度
with state.lock:
    state.current_velocity = (vx, vy, vz)
    state.movement_count += 1

# 重置 tracker flags
state.reset_tracker_flags()

self.get_logger().info(
    f"Drone {drone_id}: 移动 {state.movement_count}/{self.MAX_MOVEMENTS}, "
    f"速度 ({vx:.2f}, {vy:.2f}, {vz:.2f}), "
    f"质量 {state.current_quality:.3f}"
)

# 移动约 3 秒后，重置速度为零（悬停等待下一轮扫描）
time.sleep(3.0)

# 重置速度为零，无人机悬停等待下一轮 Tracker 扫描
with state.lock:
    state.current_velocity = (0.0, 0.0, 0.0)

self.get_logger().info(
    f"Drone {drone_id}: 移动完成，悬停等待下一轮扫描"
)

# 等待下一轮扫描（约 120 秒）- 已在悬停状态
time.sleep(1.0)
```

**关键变化**：
1. 添加 `time.sleep(3.0)` - 允许无人机移动约3秒
2. 添加速度重置：`state.current_velocity = (0.0, 0.0, 0.0)`
3. 添加日志确认悬停状态

---

### 文件2：`fast_scan_node.py`

**位置**：`src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`

**修改位置**：`FastScanNode.__init__` 函数（约24-39行）

#### 原代码（使用ROS2参数）
```python
class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node')
        
        # 聲明參數：每架無人機綁定自己的兩個 Tracker
        self.declare_parameter('tracker_a_id', '!e2e5b7c4')  # 默认值（可覆盖）
        self.declare_parameter('tracker_b_id', '!e2e5b8f8')  # 默认值（可覆盖）
        
        # 獲取參數
        tracker_a = self.get_parameter('tracker_a_id').value
        tracker_b = self.get_parameter('tracker_b_id').value
        self.target_node_ids = [tracker_a, tracker_b]
        
        self.callback_group = ReentrantCallbackGroup()
        self.link_pub = self.create_publisher(String, 'link_quality', 10)
```

#### 修正后代码（硬编码配置）
```python
class FastScanNode(Node):
    def __init__(self):
        super().__init__('fast_scan_node')
        
        # ===== 配置区域：每架无人机的 Jetson 需手动修改以下 Tracker ID =====
        # Tracker ID 格式：以 "!" 开头的 Meshtastic 节点 ID
        # 例如：TRACKER_A_ID = "!e2e5b7c4"
        #       TRACKER_B_ID = "!e2e5b8f8"
        TRACKER_A_ID = "!e2e5b7c4"  # ← 修改为实际的 Tracker A 节点 ID
        TRACKER_B_ID = "!e2e5b8f8"  # ← 修改为实际的 Tracker B 节点 ID
        # =====================================================================
        
        self.target_node_ids = [TRACKER_A_ID, TRACKER_B_ID]
        
        self.callback_group = ReentrantCallbackGroup()
        self.link_pub = self.create_publisher(String, 'link_quality', 10)
```

**关键变化**：
1. 移除ROS2参数声明（`declare_parameter`）
2. 移除参数获取（`get_parameter`）
3. 添加明确的配置区域标注
4. 直接硬编码 Tracker ID（每架Jetson手动修改）

---

### 文件3：`launch_jetson.sh`

**位置**：`launch_jetson.sh`（项目根目录）

#### 原代码（接受Tracker ID参数）
```bash
#!/bin/bash
# Jetson 机载节点启动脚本
# 
# 用法：
#   ./launch_jetson.sh [无人机编号] [Tracker_A_ID] [Tracker_B_ID]
#   例如：./launch_jetson.sh 1 !aabbcc11 !aabbcc22

set -e

# 检查参数
if [ -z "$1" ]; then
    echo "错误: 请指定无人机编号"
    echo "用法: $0 [drone_id] [tracker_a_id] [tracker_b_id]"
    echo "例如: $0 1 !aabbcc11 !aabbcc22"
    echo ""
    echo "如果不指定 Tracker ID，将使用默认值（测试用）："
    echo "  Tracker A: !e2e5b7c4"
    echo "  Tracker B: !e2e5b8f8"
    exit 1
fi

DRONE_ID=$1
TRACKER_A_ID=${2:-"!e2e5b7c4"}  # 默认值
TRACKER_B_ID=${3:-"!e2e5b8f8"}  # 默认值

echo "=========================================="
echo "启动 Jetson 机载节点"
echo "无人机编号: $DRONE_ID"
echo "绑定 Tracker A: $TRACKER_A_ID"
echo "绑定 Tracker B: $TRACKER_B_ID"
echo "=========================================="

# ... 省略中间部分 ...

echo ""
echo "启动节点..."
echo "  - fast_scan_node → /drone_${DRONE_ID}/link_quality"
echo "  - 扫描 Tracker: ${TRACKER_A_ID}, ${TRACKER_B_ID}"
echo "  - velocity_control → /drone_${DRONE_ID}/offboard_velocity_cmd"
echo ""

# 使用 launch 文件启动
ros2 launch px4_offboard jetson_onboard.launch.py \
    drone_id:=${DRONE_ID} \
    tracker_a_id:=${TRACKER_A_ID} \
    tracker_b_id:=${TRACKER_B_ID}
```

#### 修正后代码（简化参数）
```bash
#!/bin/bash
# Jetson 机载节点启动脚本
# 
# 用法：
#   ./launch_jetson.sh [无人机编号]
#   例如：./launch_jetson.sh 1
#
# 注意：Tracker ID 已在 fast_scan_node.py 中硬编码，
#       每架无人机的 Jetson 需手动修改该文件中的配置

set -e

# 检查参数
if [ -z "$1" ]; then
    echo "错误: 请指定无人机编号"
    echo "用法: $0 [drone_id]"
    echo "例如: $0 1"
    echo ""
    echo "注意: Tracker ID 已在 fast_scan_node.py 中硬编码"
    echo "      每架 Jetson 需手动修改该文件的配置区域"
    exit 1
fi

DRONE_ID=$1

echo "=========================================="
echo "启动 Jetson 机载节点"
echo "无人机编号: $DRONE_ID"
echo "=========================================="

# ... 省略中间部分 ...

echo ""
echo "启动节点..."
echo "  - fast_scan_node → /drone_${DRONE_ID}/link_quality"
echo "  - velocity_control → /drone_${DRONE_ID}/offboard_velocity_cmd"
echo ""
echo "注意: Tracker ID 已在 fast_scan_node.py 中硬编码"
echo ""

# 使用 launch 文件启动
ros2 launch px4_offboard jetson_onboard.launch.py \
    drone_id:=${DRONE_ID}
```

**关键变化**：
1. 移除 `tracker_a_id` 和 `tracker_b_id` 参数
2. 简化用法说明
3. 添加硬编码配置的提示信息
4. 简化launch命令

---

### 文件4：`jetson_onboard.launch.py`

**位置**：`src/ROS2_PX4_Offboard_Example/launch/jetson_onboard.launch.py`

#### 原代码（声明Tracker ID参数）
```python
def generate_launch_description():
    # 声明参数
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='无人机编号（1, 2, 3, ...）'
    )
    
    tracker_a_arg = DeclareLaunchArgument(
        'tracker_a_id',
        default_value='!e2e5b7c4',
        description='绑定的第一个 Tracker ID'
    )
    
    tracker_b_arg = DeclareLaunchArgument(
        'tracker_b_id',
        default_value='!e2e5b8f8',
        description='绑定的第二个 Tracker ID'
    )
    
    drone_id = LaunchConfiguration('drone_id')
    tracker_a_id = LaunchConfiguration('tracker_a_id')
    tracker_b_id = LaunchConfiguration('tracker_b_id')
    
    # 1. fast_scan_node（扫描 Meshtastic）
    fast_scan_node = Node(
        package='px4_offboard',
        executable='fast_scan',
        name=['fast_scan_drone_', drone_id],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('link_quality', ['/drone_', drone_id, '/link_quality'])
        ],
        parameters=[{
            'use_sim_time': False,
            'tracker_a_id': tracker_a_id,
            'tracker_b_id': tracker_b_id,
        }]
    )
    
    # ... 省略 velocity_control_node ...
    
    return LaunchDescription([
        drone_id_arg,
        tracker_a_arg,
        tracker_b_arg,
        fast_scan_node,
        velocity_control_node,
    ])
```

#### 修正后代码（移除Tracker ID参数）
```python
def generate_launch_description():
    # 声明参数
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='1',
        description='无人机编号（1, 2, 3, ...）'
    )
    
    drone_id = LaunchConfiguration('drone_id')
    
    # 1. fast_scan_node（扫描 Meshtastic）
    # 注意：Tracker ID 已在节点代码中硬编码
    fast_scan_node = Node(
        package='px4_offboard',
        executable='fast_scan',
        name=['fast_scan_drone_', drone_id],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('link_quality', ['/drone_', drone_id, '/link_quality'])
        ],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # ... 省略 velocity_control_node ...
    
    return LaunchDescription([
        drone_id_arg,
        fast_scan_node,
        velocity_control_node,
    ])
```

**关键变化**：
1. 移除 `tracker_a_arg` 和 `tracker_b_arg` 声明
2. 移除 `tracker_a_id` 和 `tracker_b_id` 配置获取
3. 从 `fast_scan_node` 的parameters中移除tracker ID
4. 从 `LaunchDescription` 中移除tracker参数

---

## 实施步骤

### 步骤1：应用代码修改
所有修改已完成，文件包括：
1. ✅ `multi_drone_signal_optimizer.py` - 修复速度重置Bug
2. ✅ `fast_scan_node.py` - 改为硬编码Tracker ID
3. ✅ `launch_jetson.sh` - 简化启动参数
4. ✅ `jetson_onboard.launch.py` - 移除Tracker ID参数

### 步骤2：配置每架Jetson
**在每架无人机的Jetson上**：

1. 找到文件：
   ```bash
   cd ~/uav-core/src/ROS2_PX4_Offboard_Example/px4_offboard
   nano fast_scan_node.py
   ```

2. 找到配置区域（约第26-35行）：
   ```python
   # ===== 配置区域：每架无人机的 Jetson 需手动修改以下 Tracker ID =====
   TRACKER_A_ID = "!e2e5b7c4"  # ← 修改为实际的 Tracker A 节点 ID
   TRACKER_B_ID = "!e2e5b8f8"  # ← 修改为实际的 Tracker B 节点 ID
   # =====================================================================
   ```

3. 修改为实际的Tracker ID：
   ```python
   # 例如，无人机1绑定 Tracker 1 和 2：
   TRACKER_A_ID = "!aabbcc11"  # Tracker 1 的实际 ID
   TRACKER_B_ID = "!aabbcc22"  # Tracker 2 的实际 ID
   ```

4. 保存文件（Ctrl+O, Enter, Ctrl+X）

### 步骤3：重新编译
```bash
cd ~/uav-core
colcon build --merge-install
source install/setup.bash
```

### 步骤4：测试验证

#### 测试1：速度重置验证
观察日志输出，应该看到：
```
Drone 1: 移动 1/5, 速度 (0.20, 0.10, 0.00), 质量 0.850
Drone 1: 移动完成，悬停等待下一轮扫描  ← 新增日志
```

无人机行为：
- 计算速度后开始移动
- 移动约3秒
- **停止并悬停**（重要！）
- 等待约120秒（下一轮Tracker扫描）
- 重复循环

#### 测试2：硬编码Tracker ID验证
启动Jetson节点：
```bash
./launch_jetson.sh 1
```

应该看到：
```
启动 Jetson 机载节点
无人机编号: 1
启动节点...
  - fast_scan_node → /drone_1/link_quality
  - velocity_control → /drone_1/offboard_velocity_cmd
注意: Tracker ID 已在 fast_scan_node.py 中硬编码
```

检查fast_scan_node日志，应该正确扫描配置的两个Tracker。

---

## 期望行为

### 单个移动周期
1. **等待阶段（悬停）**
   - 速度：(0, 0, 0)
   - 状态：悬停在当前位置
   - 等待：两个Tracker扫描完成（~120秒）

2. **计算阶段**
   - 收到两个Tracker的link_quality
   - 计算信号质量和梯度
   - 计算下一步速度向量

3. **移动阶段**
   - 速度：(vx, vy, vz)，持续约3秒
   - 日志：显示移动次数、速度、质量

4. **停止阶段（新增！）**
   - 速度重置为 (0, 0, 0)
   - 日志："移动完成，悬停等待下一轮扫描"
   - 无人机悬停

5. **循环**
   - 回到步骤1，等待下一轮扫描

### 整体实验流程
1. 3架无人机分别在自己区域内搜索各自的2个Tracker
2. 每架无人机独立执行"等待→计算→移动→悬停"循环
3. 每架无人机执行5次移动或连续3次无改善后停止
4. 最终停在历史最佳位置（信号质量最高点）

---

## 注意事项

### 1. Tracker ID配置
- **每架Jetson只需配置一次**（部署时）
- 修改后需要重新编译：`colcon build`
- 不同无人机绑定不同的Tracker组合
- ID格式必须以"!"开头（Meshtastic格式）

### 2. 速度重置时机
- 移动3秒后自动重置
- 收到offboard disable命令也会重置
- 达到移动限制时重置
- 两个Tracker未就绪时保持零速度

### 3. 边界保护
仍然有效：
- 位置预测超出范围 → 速度裁剪
- 实际位置超出范围 → 位置钳制
- Z轴保持在 2.0m（不改变高度）

---

## 验证清单

- [ ] 速度重置功能正常（日志确认）
- [ ] 无人机移动后能悬停
- [ ] Jetson能正常启动（不需要传递Tracker ID参数）
- [ ] fast_scan_node能扫描到配置的两个Tracker
- [ ] 三架无人机分别搜索各自的2个Tracker（共6个）
- [ ] 单机测试：一架无人机能完成完整的搜索循环

---

## 文件清单

修改的文件：
1. `src/ROS2_PX4_Offboard_Example/px4_offboard/multi_drone_signal_optimizer.py`
2. `src/ROS2_PX4_Offboard_Example/px4_offboard/fast_scan_node.py`
3. `launch_jetson.sh`
4. `src/ROS2_PX4_Offboard_Example/launch/jetson_onboard.launch.py`

新增文档：
5. `BUG_FIX_AND_CONFIGURATION_CHANGES.md` (本文件)

---

## 后续步骤

1. **重新编译**
   ```bash
   cd ~/uav-core
   colcon build --merge-install
   ```

2. **配置每架Jetson**（修改fast_scan_node.py中的Tracker ID）

3. **单机测试**
   - 启动一架无人机的Jetson节点
   - 启动地面站optimizer节点
   - 观察移动周期：移动→停止→悬停→等待→再移动

4. **三机联合测试**
   - 同时启动三架无人机
   - 验证各自独立搜索各自的两个Tracker
   - 确认没有互相干扰

---

**修改完成时间**：2025年1月
**测试状态**：待验证
**文档版本**：v1.0
