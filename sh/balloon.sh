#!/bin/bash

# 真机通过串口或者网口连接 XRCE-DDS Agent（这里以常见的串口 ttyUSB0 为例）
gnome-terminal --window -e 'bash -c "MicroXRCEAgent serial -d /dev/ttyUSB0 -b 921600; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/ros2_ws/install/setup.bash; ros2 run detect_object detect_balloon --ros-args -p simulation:=false -p save_video:=true; exec bash"'