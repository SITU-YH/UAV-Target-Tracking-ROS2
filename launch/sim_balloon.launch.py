import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_target_tracking = get_package_share_directory('uav_target_tracking')
    
    default_model_path = os.path.join(pkg_uav_target_tracking, 'models', 'RedBall', 'model.sdf')
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/iris/usb_cam/image_raw',
        description='Camera image topic name'
    )
    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='true',
        description='Whether running in simulation'
    )
    save_video_arg = DeclareLaunchArgument(
        'save_video', default_value='false',
        description='Save processed image into local video file'
    )
    red_ball_dir_arg = DeclareLaunchArgument(
        'red_ball_dir', default_value=default_model_path,
        description='Path to the RedBall SDF model file'
    )

    # 4. 在 Gazebo 环境中生成（Spawn）红球模型的节点
    # 注意：ROS2 对应的 gazebo 插件节点类型为 spawn_entity.py
    spawn_redball_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'red_ball',         # 模型实体名称
            '-file', LaunchConfiguration('red_ball_dir'),
            '-x', '5.0',
            '-y', '5.0',
            '-z', '5.0'
        ]
    )

    # 5. 启动我们刚才重构好的自定义 ROS2 气球检测节点
    detect_balloon_node = Node(
        package='duav_target_tracking',
        executable='detect_balloon',      # 必须与 CMakeLists.txt 中编译出来的可执行文件名一致
        name='detect_balloon',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'simulation': LaunchConfiguration('simulation'),
            'save_video': LaunchConfiguration('save_video'),
        }]
    )

    # 6. 构建并返回包含所有声明和节点的启动描述器
    return LaunchDescription([
        camera_topic_arg,
        simulation_arg,
        save_video_arg,
        red_ball_dir_arg,
        spawn_redball_node,
        detect_balloon_node
    ])