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
                'keyframe_distance': 0.08,
                'keyframe_time': 0.1,
                'keyframe_angle': 0.2,
                'loop_closure_interval': 1.0,
                'sc_distance_thresh': 0.4,
                'sc_num_candidates': 5,
                'voxel_size': 0.5,
            }]
        )
    ])