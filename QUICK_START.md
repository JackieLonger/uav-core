# 多无人机信号优化器 - 快速启动卡片

## 🚀 启动顺序

### 1️⃣ 笔记本（地面站）

```bash
cd ~/uav-core
source install/setup.bash

# Terminal 1: 启动优化器
./launch_multi_drone_optimizer.sh 3

# Terminal 2: 启动 RViz2（可选但推荐）
./launch_rviz.sh
```

---

### 2️⃣ Jetson 1（无人机1）

```bash
cd ~/uav-core
source install/setup.bash

# 指定绑定的两个 Tracker ID
./launch_jetson.sh 1 !tracker1A !tracker1B
```

---

### 3️⃣ Jetson 2（无人机2）

```bash
cd ~/uav-core
source install/setup.bash

# 指定绑定的两个 Tracker ID
./launch_jetson.sh 2 !tracker2A !tracker2B
```

---

### 4️⃣ Jetson 3（无人机3）

```bash
cd ~/uav-core
source install/setup.bash

# 指定绑定的两个 Tracker ID
./launch_jetson.sh 3 !tracker3A !tracker3B
```

**重要**: 将 `!trackerXX` 替换为实际的 Meshtastic Tracker ID！

---

## ✈️ 飞行操作

### 起飞
1. ✅ 检查 ROS2 网络连接
2. ✅ 遥控器 ARM
3. ✅ 手动起飞到 5m 高度
4. ✅ 切换到 **Offboard 模式**

### 自动优化
- 系统自动开始搜索
- RViz2 实时显示位置和信号
- 最多 5 次移动
- 每次移动后悬停等待扫描

### 降落
1. ✅ 遥控器切回 Position/Manual
2. ✅ 手动降落
3. ✅ DISARM

---

## 📊 RViz2 颜色编码（笔记本上显示）

### 信号质量（无人机球体颜色）
- 🔴 **红色**：质量差 (Q < 0.5)
- 🟡 **黄色**：质量中 (Q ≈ 0.5)
- 🟢 **绿色**：质量好 (Q > 0.5)

### 搜索边界（立方体框线颜色）
- 🟦 **青色**：Drone 1 的边界
- 🟪 **洋红**：Drone 2 的边界
- 🟨 **黄色**：Drone 3 的边界

### 无人机标签显示
```
Drone N
Q: 0.XX
绑定: !trackerNA
      !trackerNB
```
每架无人机显示其绑定的两个 Tracker ID

### 其他元素
- 🔵 **蓝色线条**：飞行轨迹
- 🟡 **黄色箭头**：速度向量

---

## 🔧 常用检查命令

```bash
# 检查所有 topics
ros2 topic list | grep drone

# 查看无人机1的信号
ros2 topic echo /drone_1/link_quality

# 查看无人机1的速度指令
ros2 topic echo /drone_1/offboard_velocity_cmd

# 检查节点运行状态
ros2 node list
```

---

## ⚠️ 故障排查

### 无人机不移动？
- [ ] 是否切换到 Offboard？
- [ ] `/drone_N/link_quality` 有数据吗？
- [ ] 两个 Tracker 都扫描成功了吗？

### RViz2 看不到东西？
- [ ] 检查 Fixed Frame 是否为 `map`
- [ ] 展开 `MarkerArray` 勾选所有命名空间
- [ ] 调整视角（滚轮缩放）

### 网络连接问题？
```bash
# 检查 ROS_DOMAIN_ID（所有设备必须相同）
echo $ROS_DOMAIN_ID

# 如果未设置，添加到 ~/.bashrc
export ROS_DOMAIN_ID=0
```

---

## 📝 重要参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 最大移动次数 | 5 | 达到后悬停 |
| 最大速度 | 0.3 m/s | 安全速度 |
| 优先上升高度 | 1.5 m | 先上升再XY搜索 |
| 搜索边界 | 3m³ | ±1.5m XY, 0~+3m Z |
| 发布频率 | 100 Hz | 防止 PX4 failsafe |
| 扫描周期 | 120 秒 | 两个 Tracker 完整扫描 |

---

**版本**: v1.0  
**日期**: 2025-11-12  
