#!/bin/bash

echo "======================================"
echo "RViz2 可視化測試"
echo "======================================"
echo ""
echo "這個腳本會："
echo "  1. 啟動 Visualizer"
echo "  2. 啟動 RViz2"
echo "  3. 模擬 Drone 1 飛行"
echo "  4. 顯示軌跡和信號質量"
echo ""

cd /home/landis/uav-core
source install/setup.bash

# 清理
pkill -f multi_drone_visualizer 2>/dev/null
pkill -f rviz2 2>/dev/null
sleep 1

echo "啟動 Visualizer..."
ros2 run ros2_px4_offboard_example multi_drone_visualizer.py &
VIS_PID=$!
sleep 2

echo "啟動 RViz2..."
(
    unset GTK_PATH
    unset LD_LIBRARY_PATH
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    rviz2 -d src/ROS2_PX4_Offboard_Example/resource/multi_drone.rviz
) &
RVIZ_PID=$!
sleep 5

echo ""
echo "開始模擬 Drone 1 飛行 (20 個點)..."
for i in {0..19}; do
    x=$i
    y=$(echo "$i * 0.5" | bc)
    z=$(echo "-$i * 0.2" | bc)
    
    ros2 topic pub --once /drone_1/fmu/out/vehicle_local_position \
        px4_msgs/VehicleLocalPosition \
        "{timestamp: $i, x: $x.0, y: $y, z: $z}" > /dev/null 2>&1
    
    rssi=$(echo "-75 + $i * 1.5" | bc)
    snr=$(echo "8 + $i * 0.5" | bc)
    ros2 topic pub --once /drone_1/link_quality std_msgs/String \
        "data: '{\"status\": \"Success\", \"forward_rssi\": $rssi, \"forward_snr\": $snr, \"return_rssi\": $rssi, \"return_snr\": $snr}'" \
        > /dev/null 2>&1
    
    echo -ne "  進度: $((i+1))/20\r"
    sleep 0.5
done

echo ""
echo ""
echo "✅ 完成！請在 RViz2 中查看："
echo "  • 橘紅色軌跡線"
echo "  • 無人機球體（顏色隨信號變化）"
echo "  • 青色邊界框"
echo ""
echo "按 Enter 關閉..."
read

kill $RVIZ_PID $VIS_PID 2>/dev/null
sleep 2

echo "測試完成！"
