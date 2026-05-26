import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取功能包路径
    fastlio_imu_pkg = get_package_share_directory('fastlio_imu')
    # livox_driver_pkg = get_package_share_directory('livox_ros_driver2')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    fastlio_pgo_pkg = get_package_share_directory('fastlio_pgo')

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


    ros_to_serial_node = Node(
        package='fastlio_imu',
        executable='ros_to_serial.py',
        name='ros_to_serial_node',
        output='screen',
    )


    cloud_filter_node = Node(
        package='lidox_detect',
        executable='cloud_filter_node',
        name='cloud_filter_node',
        output='screen'
    )

    shape_detect_node = Node(
        package='lidox_detect',
        executable='shape_detect_node',
        name='shape_detect_node',
        output='screen',
        parameters=[{
            'cluster_tolerance': 0.12,
            'min_cluster_size_ring': 40,
            'min_cluster_size_pillar': 80,
            'max_cluster_size': 8000,
            'ring_fit_tolerance': 0.08,
            'ring_inner_radius': 0.40,
            'ring_outer_radius': 0.70,
            'ring_inlier_ratio_min': 0.45,
            'ring_max_points': 800,
            'pillar_l2_l1_max': 0.35,
            'pillar_l1_l3_min': 8.0,
        }]
    )

    return LaunchDescription([
        # livox_launch,
        fastlio_launch,
        forward_node,
        ros_to_serial_node,
        sc_pgo_launch,
        cloud_filter_node,
        shape_detect_node
    ])
