#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from fastlio_imu.msg import Newmsg
from geometry_msgs.msg import PoseStamped
import serial
import struct
import math
import time
import threading
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import control_t as servo

# ================== 配置区 ==================
SERIAL_PORT = '/dev/ttyS1'
BAUD_RATE = 500000
TOPIC_NAME = '/test/newmsg'

# 帧格式定义（与接收端一致）
FRAME_HEAD = b'\xAA'
POS_FLAG   = b'\x50'   # 位置数据标志，对应字符 'P'
VEL_FLAG   = b'\x54'   # 速度数据标志，对应字符 'T'
SPOT_FLAG  = b'\x53'   # 圆环检测标志，对应字符 'S'

# 飞控命令
CMD_HEAD  = 0xAA
CMD_OPEN  = 0x01   # 投放指令
CMD_TAIL  = 0xFF
# =============================================

# ================== 四元数转偏航 ==================
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
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"串口打开成功：{SERIAL_PORT} 波特率{BAUD_RATE}bps")
        except Exception as e:
            self.get_logger().error(f"串口打开失败：{e}")
            raise SystemExit(1)

        self.subscription = self.create_subscription(
            Newmsg,
            TOPIC_NAME,
            self.callback,
            10
        )

        # 订阅圆环检测结果
        self.ring_sub = self.create_subscription(
            PoseStamped,
            '/lidox/ring_center',
            self.ring_callback,
            10
        )

        self.latest_ring = None   # 缓存最新圆环位置
        self._recv_running = True

        # 初始化舵机
        servo.init()
        servo.close()
        time.sleep(0.3)

        # 启动串口接收线程
        self._recv_thread = threading.Thread(target=self._serial_recv_loop, daemon=True)
        self._recv_thread.start()

        self.get_logger().info(f"正在订阅话题：{TOPIC_NAME}，串口接收已启动")

    def ring_callback(self, msg):
        """缓存最新圆环检测结果"""
        self.latest_ring = msg

    def _serial_recv_loop(self):
        """后台线程：监听飞控下发的指令"""
        buf = bytearray()
        while self._recv_running:
            try:
                data = self.ser.read(1)
                if not data:
                    continue
                buf.append(data[0])

                # 搜索帧头 0xAA
                while len(buf) > 0 and buf[0] != CMD_HEAD:
                    buf.pop(0)

                # 等待完整帧: AA cmd FF (3字节)
                if len(buf) >= 3:
                    if buf[0] == CMD_HEAD and buf[2] == CMD_TAIL:
                        cmd = buf[1]
                        if cmd == CMD_OPEN:
                            self.get_logger().info("收到投放指令 AA 01 FF")
                            servo.open()
                            time.sleep(1.0)
                            servo.close()
                        else:
                            self.get_logger().warn(f"未知指令: AA {cmd:02X} FF")
                        buf = bytearray()  # 清空缓冲区
                    else:
                        buf.pop(0)  # 不匹配，丢弃首字节继续搜索
            except Exception as e:
                self.get_logger().warn(f"串口接收异常: {e}")
                time.sleep(0.01)

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

            # 4. 打包圆环检测数据（实际由 ring_detect_node 提供）
            if self.latest_ring is not None:
                rx = self.latest_ring.pose.position.x
                ry = self.latest_ring.pose.position.y
                rz = self.latest_ring.pose.position.z
            else:
                rx = ry = rz = 0.0   # 未检测到圆环时填 0
            spot_data = struct.pack('<3f', rx, ry, rz)

            # 5. 组装完整帧：帧头 + 位置 + 速度 + 圆环
            frame = FRAME_HEAD + POS_FLAG + pos_data + VEL_FLAG + vel_data + SPOT_FLAG + spot_data

            # 6. 发送
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
        self._recv_running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        servo.off()

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