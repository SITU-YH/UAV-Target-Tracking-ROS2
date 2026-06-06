import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包所在共享路径
    pkg_detect_object = get_package_share_directory('detect_object')
    
    # 2. 默认的 CircularFrame 门框模型路径
    default_model_path = os.path.join(pkg_detect_object, 'models', 'CircularFrame', 'model.sdf')

    # 3. 声明 Launch 参数
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/iris/usb_cam/image_raw'
    )
    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='true'
    )
    save_video_arg = DeclareLaunchArgument(
        'save_video', default_value='false'
    )
    door_frame_dir_arg = DeclareLaunchArgument(
        'door_frame_dir', default_value=default_model_path,
        description='Path to the CircularFrame SDF model file'
    )

    # 4. 在 Gazebo 环境中生成门框的节点
    # 提示：ROS2 中参数 -R,-P,-Y 通常会自动转换为四元数传给 Gazebo
    spawn_doorframe_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'doorframe',
            '-file', LaunchConfiguration('door_frame_dir'),
            '-x', '0.0',
            '-y', '5.0',
            '-z', '5.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '1.57'
        ]
    )

    # 5. 启动自定义的 ROS2 门框检测节点
    detect_doorframe_node = Node(
        package='detect_object',
        executable='detect_doorframe',
        name='detect_doorframe',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'simulation': LaunchConfiguration('simulation'),
            'save_video': LaunchConfiguration('save_video'),
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        simulation_arg,
        save_video_arg,
        door_frame_dir_arg,
        spawn_doorframe_node,
        detect_doorframe_node
    ])