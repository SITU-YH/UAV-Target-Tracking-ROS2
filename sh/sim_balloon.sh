#!/bin/bash

# 1. 启动 Micro XRCE-DDS Agent（ROS2 与 PX4 仿真的通信桥梁，假设使用 UDP 默认端口）
gnome-terminal --window -e 'bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"' \
--tab -e 'bash -c "sleep 2; echo \"请在此窗口手动启动你的 Gazebo PX4 仿真环境\"; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/ros2_ws/install/setup.bash; ros2 launch detect_object sim_balloon.launch.py; exec bash"'