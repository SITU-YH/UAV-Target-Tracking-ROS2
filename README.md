本项目基于 [[Detect_Object](https://github.com/Lovely-XPP/Detect_Object.git)] 进行重构，适配 ROS2 Humble 环境，并优化了 PX4 通信架构

---

# Detect Object 

基于 ROS2 和 PX4 的无人机视觉目标检测与控制程序（支持红球撞击、圆环穿越等）。

## 环境依赖

* **ROS2** (建议 Humble 或更高版本)
* **PX4 Autopilot** (通过 Micro XRCE-DDS 与飞控通信)
* **OpenCV**
* **cv_bridge**

## 编译方法

```bash
# 进入你的 ROS2 工作空间
cd ~/ros2_ws/src
git clone https://github.com/SITU-YH/uav-target-tracking-ros2.git
cd ..
colcon build --packages-select uav_target_tracking --symlink-install
source install/setup.bash

```

## 运行方法

在运行前，请确保已启动 PX4 仿真环境（如 Gazebo）以及对应的 DDS Agent。

### 1. 撞击红球

```bash
ros2 launch detect_object sim_balloon.launch.py

```

### 2. 穿越圆形门框

```bash
ros2 launch detect_object sim_doorframe.launch.py

```

### 3. 一键快速启动 (脚本)

如果需要配合终端脚本使用：

```bash
bash $(ros2 pkg prefix detect_object)/share/detect_object/sh/sim_balloon.sh

```

## 主要功能

* **图像处理**：基于 OpenCV 的 HSV 颜色分割与轮廓提取。
* **逻辑控制**：计算目标在图像中的偏差，并通过 ROS2 原生话题 `/fmu/in/trajectory_setpoint` 向飞控发送机体系（BODY）速度控制指令。
* **视频录制**：支持启动时通过参数 `save_video:=true` 保存处理后的检测过程。

## 注意事项

* **坐标系**：本项目生成的控制指令基于无人机机体系，请确保飞控处于 Offboard 模式。
* **参数配置**：可在 Launch 文件中通过参数动态修改相机话题名（`camera_topic`）。

---

