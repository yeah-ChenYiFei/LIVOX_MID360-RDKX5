#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from fastlio_imu.msg import Newmsg
import serial
import struct
import math

# ================== 配置区 ==================
SERIAL_PORT = '/dev/ttyS1'
BAUD_RATE = 500000
TOPIC_NAME = '/test/newmsg'

# 帧格式定义（与接收端一致）
FRAME_HEAD = b'\xAA'
POS_FLAG   = b'\x50'   # 位置数据标志，对应字符 'P'
VEL_FLAG   = b'\x54'   # 速度数据标志，对应字符 'T'
# =============================================

def quaternion_to_yaw_deg(qx, qy, qz, qw):
    """
    将四元数转换为偏航角（单位：度），左转为负，右转为正。
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = yaw_rad #* 180.0 / math.pi
    return -yaw_deg

class RosToSerialNode(Node):
    def __init__(self):
        super().__init__('ros_to_serial_node')
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
            self.get_logger().info(f"✅ 串口打开成功：{SERIAL_PORT} 波特率{BAUD_RATE}bps")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败：{e}")
            raise SystemExit(1)
        
        self.subscription = self.create_subscription(
            Newmsg,
            TOPIC_NAME,
            self.callback,
            10
        )
        self.get_logger().info(f"📡 正在订阅话题：{TOPIC_NAME}")

    def callback(self, msg):
        """回调函数：将ROS消息打包为二进制帧并发送"""
        try:
            # 1. 提取数据
            x  = msg.pose.position.x
            y  = msg.pose.position.y
            z  = msg.pose.position.z
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            wx = msg.twist.angular.x
            wy = msg.twist.angular.y
            wz = msg.twist.angular.z

            yaw_deg = quaternion_to_yaw_deg(qx, qy, qz, qw)
            self.get_logger().info(f"当前偏航角: {yaw_deg:.2f}rad")

            # 2. 打包位置数据（7个 double，小端序）—— 注意：这里改为4个double？需与接收端协议一致
            # 原协议：7个double (x,y,z,qx,qy,qz,qw)
            # 你现在改为4个double (x,y,z,yaw_deg)，必须确保接收端也相应修改。
            # 如果接收端仍期待7个double，这里应恢复为：
            # pos_data = struct.pack('<7d', x, y, z, qx, qy, qz, qw)
            # 但根据你之前意图，我们按你当前代码（4个double）执行。
            pos_data = struct.pack('<4d', x, y, z, yaw_deg)
            # 3. 打包速度数据（6个 double，小端序）
            vel_data = struct.pack('<6d', vx, vy, vz, wx, wy, wz)

            # 4. 组装完整帧：帧头 + 位置标志 + 位置数据 + 速度标志 + 速度数据
            frame = FRAME_HEAD + POS_FLAG + pos_data + VEL_FLAG + vel_data

            # 5. 发送
            self.ser.write(frame)

            # 可选：打印调试信息
            self.get_logger().info(
                f"📤 发送二进制帧，长度{len(frame)}字节，"
                f"前8字节: {frame[:8].hex()}"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ 发送失败：{e}")
            import traceback
            traceback.print_exc()

    def close_serial(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("🔌 串口已关闭")

def main(args=None):
    rclpy.init(args=args)
    node = RosToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()