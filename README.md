本项目基于 [[Detect_Object](https://github.com/Lovely-XPP/Detect_Object.git)] 进行重构，适配 ROS2 Humble 环境，并优化了 PX4 通信架构

---

# Detect Object 

基于 ROS2 和 PX4 的无人机视觉目标检测与控制程序（支持红球撞击、圆环穿越等）。

## 环境依赖

* [**ROS2**](https://docs.ros.org/en/humble/Installation.html) (建议 Humble 或更高版本)

* [**gazebo-classic**](https://github.com/gazebosim/gazebo-classic.git) 可使用`aptitude`安装

* [**PX4 Autopilot**](https://github.com/PX4/PX4-Autopilot.git) (通过 Micro XRCE-DDS 与飞控通信)

* **ros-humble-gazebo-ros-pkgs**
```bash
  sudo apt install ros-humble-gazebo-ros
```

* [**px4通信包**](https://github.com/PX4/px4_msgs.git)

* [**启动 XRCE-DDS Agent**](https://github.com/eProsima/Micro-XRCE-DDS-Agent.git)

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


### 1. 启动 PX4 + Gazebo 仿真

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

### 2. 启动 XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888

```

### 3. 加载圆环和红球

```bash
bash $(ros2 pkg prefix uav_target_tracking)/share/uav_target_tracking/sh/sim_balloon.sh

```

## 主要功能

* **图像处理**：基于 OpenCV 的 HSV 颜色分割与轮廓提取。
* **逻辑控制**：计算目标在图像中的偏差，并通过 ROS2 原生话题 `/fmu/in/trajectory_setpoint` 向飞控发送机体系（BODY）速度控制指令。
* **视频录制**：支持启动时通过参数 `save_video:=true` 保存处理后的检测过程。

## 注意事项

* **坐标系**：本项目生成的控制指令基于无人机机体系，请确保飞控处于 Offboard 模式。
* **参数配置**：可在 Launch 文件中通过参数动态修改相机话题名（`camera_topic`）。

---

px4_msgs


sudo apt update
sudo apt install git cmake g++ build-essential libasio-dev libtinyxml2-dev -y
MicroXRCEAgent udp4 -p 8888




gazebo classic

```bash
# 1. 下载官方 Agent 源码
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent

# 2. 创建编译目录并编译
mkdir build && cd build
cmake ..
make
sudo make install

# 3. 更新系统动态链接库
sudo ldconfig
```