from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cloud_filter_node = Node(
        package='lidox_detect',
        executable='cloud_filter_node',
        name='cloud_filter_node',
        output='screen',
        parameters=[{
            # 以自身为中心 3×6×3 (X前深 Y宽 Z高)
            'x_min': -1.5,
            'x_max': 1.5,
            'y_min': -3.0,
            'y_max': 3.0,
            'z_min': -1.5,
            'z_max': 1.5,
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
            'min_cluster_size_ring': 40,
            'min_cluster_size_pillar': 80,
            'max_cluster_size': 20000,
            'ring_fit_tolerance': 0.08,
            'ring_inner_radius': 0.42,
            'ring_outer_radius': 0.63,
            'ring_inlier_ratio_min': 0.45,
            'ring_max_points': 2000,
            'pillar_l2_l1_max': 0.35,
            'pillar_l1_l3_min': 8.0,
            'accumulate_window': 1.5,
            'accumulate_voxel': 0.02,
            'ground_removal_enabled': True,
            'wall_removal_enabled': True,
            'ransac_dist_thresh': 0.03,
            'ransac_ground_nz_min': 0.7,
            'ransac_wall_min_ratio': 0.30,
        }]
    )

    return LaunchDescription([
        cloud_filter_node,
        shape_detect_node,
    ])
