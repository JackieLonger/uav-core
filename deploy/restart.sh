#!/usr/bin/env bash
set -euo pipefail
WS=${WS:-$HOME/uav_core_ws}
cd "$WS"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# 例：重建後啟動（取消註解並改成你的 launch）
# ros2 launch ROS2_PX4_Offboard_Example offboard.launch.py
echo "[OK] rebuilt. (如需可在此加入 ros2 launch/systemd 啟動指令)"
