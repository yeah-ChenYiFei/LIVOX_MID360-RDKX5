from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cloud_filter_node = Node(
        package='lidox_detect',
        executable='cloud_filter_node',
        name='cloud_filter_node',
        output='screen',
        parameters=[{
            'x_min': -1.3, 'x_max': 1.6,
            'y_min': -2.2, 'y_max': 0.0,
            'z_min': -0.5, 'z_max': 1.4,
            'voxel_leaf': 0.03,
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
            'min_cluster_size_ring': 60,
            'min_cluster_size_pillar': 80,
            'max_cluster_size': 20000,
            'ring_fit_tolerance': 0.05,
            'ring_inner_radius': 0.42,
            'ring_outer_radius': 0.63,
            'ring_inlier_ratio_min': 0.55,
            'ring_hollow_ratio_max': 0.15,
            'ring_max_points': 2000,
            'pillar_l2_l1_max': 0.35,
            'pillar_l1_l3_min': 8.0,
            'accumulate_window': 0.8,
            'accumulate_voxel': 0.02,
            'ground_removal_enabled': True,
            'wall_removal_enabled': True,
            'ransac_dist_thresh': 0.03,
            'ransac_ground_nz_min': 0.7,
            'ransac_wall_min_ratio': 0.30,
            # ── World-frame passthrough filter ──
            'world_filter_enabled': True,
            'world_x_min': 5.3, 'world_x_max': 6.9,
            'world_y_min': -2.7, 'world_y_max': 0.0,
            'world_z_min': 1.0, 'world_z_max': 2.4,
        }]
    )

    return LaunchDescription([
        cloud_filter_node,
        shape_detect_node,
    ])
