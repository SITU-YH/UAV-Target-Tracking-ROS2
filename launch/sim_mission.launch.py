import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_target_tracking = get_package_share_directory('uav_target_tracking')
    
    ball_model_path = os.path.join(pkg_uav_target_tracking, 'models', 'RedBall', 'model.sdf')
    door_model_path = os.path.join(pkg_uav_target_tracking, 'models', 'CircularFrame', 'model.sdf')

    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/iris/usb_cam/image_raw')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='true')
    save_video_arg = DeclareLaunchArgument('save_video', default_value='false')

    # 1. 生成圆形门框（放在无人机正前方：X=5.0, Y=0.0, Z=5.0）
    spawn_doorframe_node = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'doorframe', '-file', door_model_path, '-x', '5.0', '-y', '0.0', '-z', '5.0', '-Y', '1.57']
    )

    # 2. 生成红球（放在门框后面，错开距离：X=12.0, Y=0.0, Z=5.0）
    spawn_redball_node = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-entity', 'red_ball', '-file', ball_model_path, '-x', '12.0', '-y', '0.0', '-z', '5.0']
    )

    # 3. 启动统一的阶段总控节点
    mission_node = Node(
        package='uav_target_tracking',
        executable='detect_mission',  # 对应我们下一步写的新可执行文件
        name='detect_mission', output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'simulation': LaunchConfiguration('simulation'),
            'save_video': LaunchConfiguration('save_video'),
        }]
    )

    return LaunchDescription([
        camera_topic_arg, simulation_arg, save_video_arg,
        spawn_doorframe_node, spawn_redball_node, mission_node
    ])