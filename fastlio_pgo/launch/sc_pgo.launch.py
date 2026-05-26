from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastlio_pgo',
            executable='sc_pgo_node',
            name='sc_pgo_node',
            output='screen',
            parameters=[{
                'keyframe_distance': 0.5,
                'keyframe_time': 1.0,
                'keyframe_angle': 0.35,
                'loop_closure_interval': 1.0,
                'sc_distance_thresh': 0.4,
                'voxel_size': 0.5,
                'loop_min_index_diff': 30,
                'icp_max_translation': 0.1,
                'icp_max_rotation': 0.1,
            }]
        )
    ])