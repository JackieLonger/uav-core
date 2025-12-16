cmake_minimum_required(VERSION 3.5)
project(ros2_px4_offboard_example)

# 設定 C++ 標準
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 相依套件
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(meshtastic_ros_bridge_msgs REQUIRED)

# 安裝 Python 套件
ament_python_install_package(px4_offboard)

# 安裝 Python 可執行檔
install(PROGRAMS
  px4_offboard/velocity_control.py
  px4_offboard/control.py
  px4_offboard/fast_scan_node.py
  px4_offboard/multi_drone_signal_optimizer.py
  px4_offboard/multi_drone_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安裝啟動檔
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

