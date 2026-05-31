import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取功能包路径
    fastlio_imu_pkg = get_package_share_directory('fastlio_imu')
    # livox_driver_pkg = get_package_share_directory('livox_ros_driver2')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    fastlio_pgo_pkg = get_package_share_directory('fastlio_pgo')

    # Optional pre-built map for ICP localization
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value='',
        description='Path to pre-built PCD map for ICP localization (empty = skip localizer)'
    )

    # 1. 启动 Livox 驱动 (msg_MID360_launch.py)
    # livox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(livox_driver_pkg, 'launch_ROS2', 'msg_MID360_launch.py')
    #     )
    # )

    # 2. 启动 FAST-LIO (mapping.launch.py)
    # 注意：确保 fast_lio 包里的 launch 文件名确实是 mapping.launch.py
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        )
    )

    sc_pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_pgo_pkg, 'launch', 'sc_pgo.launch.py')
        )
    )

    # 3. 启动转发节点 (本包内的 C++ 节点)
    forward_node = Node(
        package='fastlio_imu',
        executable='fastlio_to_fcu',
        name='fastlio_to_fcu',
        output='screen'
    )


    map_localizer_node = Node(
        package='fastlio_imu',
        executable='map_localizer',
        name='map_localizer',
        output='screen',
        parameters=[{
            'map_file': LaunchConfiguration('map_file'),
        }]
    )

    return LaunchDescription([
        map_file_arg,
        # livox_launch,
        fastlio_launch,
        forward_node,
        sc_pgo_launch,
        map_localizer_node,
    ])
