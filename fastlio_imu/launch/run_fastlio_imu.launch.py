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
        output='screen',
        parameters=[{
            # XY: focus on ring ~1m ahead, narrow lateral window
            'x_min': 0.0,
            'x_max': 1.8,
            'y_min': -0.7,
            'y_max': 0.7,
            # Z: full ring vertical span (drone at any reasonable height)
            'z_min': -0.6,
            'z_max': 1.5,
            # voxel
            'voxel_leaf': 0.02,
            # SOR: remove sparse mesh points, keep dense ring surface
            'sor_enabled': True,
            'sor_mean_k': 20,
            'sor_stddev': 1.0,
        }]
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
            # multi-frame accumulation — short window avoids ghosting during rotation
            'accumulate_window': 0.3,
            'accumulate_voxel': 0.03,
            # RANSAC (after fusion — ring is dense enough to survive)
            'ground_removal_enabled': True,
            'wall_removal_enabled': True,
            'ransac_dist_thresh': 0.03,
            'ransac_ground_nz_min': 0.7,
            'ransac_wall_min_ratio': 0.30,
        }]
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
        ros_to_serial_node,
        sc_pgo_launch,
        cloud_filter_node,
        shape_detect_node,
        map_localizer_node,
    ])
