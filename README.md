本项目基于 [[Detect_Object](https://github.com/Lovely-XPP/Detect_Object.git)] 进行重构，适配 ROS2 Humble 环境，并优化了 PX4 通信架构

---

# UAV Target Tracking ROS2

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

为了跑通完整的联合仿真，请务必开 3个不同的终端窗口 按以下严格顺序启动。

### 1. 启动 XRCE-DDS Agent 通信代理 (终端 1)

```bash
MicroXRCEAgent udp4 -p 8888

```

### 2. 启动 PX4 + Gazebo 仿真 (终端 2) 

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

### 3. 注入模型并启动算法总控 (终端 3)

```bash
source ~/uav_ws/install/setup.bash
ros2 launch uav_target_tracking sim_mission.launch.py
```

### 4. 解锁起飞 (在终端 2 中输入)

```bash
# 1. 解锁无人机电机
commander arm

# 2. 将无人机控制权交给你的 C++ 节点 (切入外部 Offboard 模式)
commander mode offboard
```

## 主任务状态机控制逻辑

* **Stage 1 (蓝色圆环追踪)**：识别前方圆环，利用速度闭环控制无人机对准圆环核心并不断逼近。
* **Stage 2 (盲冲穿越)**：接近圆环至一定大小时，算法完全屏蔽画面并加大马力（0.6 m/s）笔直盲冲 2.5 秒以通过圆环。
* **Stage 3 (红色气球追击)**：盲冲结束后状态机自动切换，算法开始在视野中捕捉远处的红球，并控制无人机精确加速实施撞击。

## 注意事项

* **坐标系**：本项目生成的控制指令基于无人机机体系，请确保飞控处于 Offboard 模式。
* **参数配置**：可在 Launch 文件中通过参数动态修改相机话题名（`camera_topic`）。

---



