#!/bin/bash

# 1. 启动 Micro XRCE-DDS Agent
gnome-terminal --window -e 'bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"' \
--tab -e 'bash -c "sleep 2; echo \"请在此窗口手动启动你的 Gazebo PX4 仿真环境\"; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/ros2_ws/install/setup.bash; ros2 launch detect_object sim_doorframe.launch.py; exec bash"'