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


        # ============ 最终修复：调用系统全局python3 ============
    # 获取Python脚本的绝对路径（确保路径正确）
    ros_to_serial_script = "/home/sunrise/livox_ws/src/fastlio_imu/src/ros_to_serial.py"
    # 验证脚本路径是否存在（可选，调试用）
    if not os.path.exists(ros_to_serial_script):
        raise FileNotFoundError(f"Python脚本不存在：{ros_to_serial_script}")

    ros_to_serial_node = Node(
        # 核心1：删除package参数，不限制查找范围
        # package='fastlio_imu',  
        # 核心2：用系统全局的python3（可先用which python3确认路径）
        executable='/usr/bin/python3',
        # 核心3：传入脚本绝对路径作为参数
        arguments=[ros_to_serial_script],
        name='ros_to_serial_node',
        output='screen',
    )
    # ======================================================


    return LaunchDescription([
        # livox_launch,
        fastlio_launch,
        forward_node,
        ros_to_serial_node,
        sc_pgo_launch
    ])
