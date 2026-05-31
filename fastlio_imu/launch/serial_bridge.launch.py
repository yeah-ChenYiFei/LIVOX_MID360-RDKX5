from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ros_to_serial_node = Node(
        package='fastlio_imu',
        executable='ros_to_serial.py',
        name='ros_to_serial_node',
        output='screen',
    )

    return LaunchDescription([ros_to_serial_node])
