#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 -> Serial bridge for FCU communication.

Tx (PC -> FCU):
    Frame:  HEAD(AA) + P-flag(50) + pos(4d) + T-flag(54) + vel(6d) + S-flag(53) + ring(3f)
    Total:  1 + 1 + 32 + 1 + 48 + 1 + 12 = 96 bytes

Rx (FCU -> PC):
    AA 01 FF  — advance servo to next stage (0->40->80->115, 3 advances total)
"""

import rclpy
from rclpy.node import Node
from fastlio_imu.msg import FcuState
from geometry_msgs.msg import PoseStamped
import serial
import struct
import math
import time
import random
import threading
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import servo_control as servo

# ========================= 配置 =========================
SERIAL_PORT = "/dev/ttyS1"
BAUD_RATE   = 500000
TOPIC_NAME  = "/fcu/state"

FRAME_HEAD = b"\xAA"
POS_FLAG   = b"\x50"    # 'P'
VEL_FLAG   = b"\x54"    # 'T'
SPOT_FLAG  = b"\x53"    # 'S'

CMD_HEAD = 0xAA
CMD_OPEN = 0x01          # FCU -> servo advance
CMD_TAIL = 0xFF

SEND_RING  = True        # True = include ring data in serial frame
TEST_RING  = False         # True = random ring coords (only when SEND_RING=True)
# =========================================================


def quaternion_to_yaw_rad(qx, qy, qz, qw):
    """Yaw from quaternion (radians), CCW positive."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return -math.atan2(siny, cosy)


class RosToSerialNode(Node):
    def __init__(self):
        super().__init__("ros_to_serial_node")

        # --- serial port ---
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"串口打开: {SERIAL_PORT} @ {BAUD_RATE} bps")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            raise SystemExit(1)

        # --- subscriptions ---
        self.subscription = self.create_subscription(
            FcuState, TOPIC_NAME, self._on_fcu_state, 10)

        self.ring_sub = self.create_subscription(
            PoseStamped, "/lidox/ring_center", self._on_ring, 10)

        self.latest_ring = None
        self._recv_running = True

        # --- servo ---
        servo.init()
        time.sleep(0.3)

        # --- FCU receive thread ---
        self._recv_thread = threading.Thread(target=self._serial_recv_loop, daemon=True)
        self._recv_thread.start()

        self.get_logger().info(
            f"订阅: {TOPIC_NAME} + /lidox/ring_center  |  "
            f"环数据: {'随机测试' if TEST_RING else '真实检测'}  |  "
            f"串口接收已启动"
        )

    # ---- ring callback ----
    def _on_ring(self, msg: PoseStamped):
        self.latest_ring = msg

    # ---- FCU -> PC receive thread ----
    def _serial_recv_loop(self):
        buf = bytearray()
        while self._recv_running:
            try:
                data = self.ser.read(1)
                if not data:
                    continue
                buf.append(data[0])

                while buf and buf[0] != CMD_HEAD:
                    buf.pop(0)

                if len(buf) >= 3:
                    if buf[0] == CMD_HEAD and buf[2] == CMD_TAIL:
                        cmd = buf[1]
                        if cmd == CMD_OPEN:
                            self.get_logger().info("[FCU] 收到投放指令 AA 01 FF")
                            has_more = servo.next_stage()
                            if has_more:
                                self.get_logger().info(
                                    f"[servo] 等待下次指令 (当前 {servo.current_angle()}deg)")
                            else:
                                self.get_logger().info(
                                    f"[servo] 已到最终阶段 {servo.current_angle()}deg")
                        else:
                            self.get_logger().warn(f"[FCU] 未知指令: AA {cmd:02X} FF")
                        buf.clear()
                    else:
                        buf.pop(0)
            except Exception as e:
                self.get_logger().warn(f"串口接收异常: {e}")
                time.sleep(0.01)

    # ---- PC -> FCU: FcuState callback ----
    def _on_fcu_state(self, msg: FcuState):
        try:
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

            yaw = quaternion_to_yaw_rad(qx, qy, qz, qw)

            # pack position (4 doubles, little-endian)
            pos_data = struct.pack("<4d", x, y, z, yaw)

            # pack velocity (6 doubles, little-endian)
            vel_data = struct.pack("<6d", vx, vy, vz, wx, wy, wz)

            # ring / spot data
            if SEND_RING:
                if not TEST_RING and self.latest_ring is not None:
                    rx = self.latest_ring.pose.position.x
                    ry = self.latest_ring.pose.position.y
                    rz = self.latest_ring.pose.position.z
                elif TEST_RING:
                    rx = random.uniform(-15.0, 15.0)
                    ry = random.uniform(-15.0, 15.0)
                    rz = random.uniform(0.0, 10.0)
                else:
                    rx = ry = rz = 0.0
                spot_data = struct.pack("<3d", rx, ry, rz)

            # assemble & send
            frame = FRAME_HEAD + POS_FLAG + pos_data + VEL_FLAG + vel_data
            if SEND_RING:
                frame += SPOT_FLAG + spot_data
            self.ser.write(frame)

            ring_str = f"ring=({rx:.1f},{ry:.1f},{rz:.1f})" if SEND_RING else "ring=OFF"
            self.get_logger().info(
                f"TX {len(frame)}B  "
                f"pos=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f}rad  "
                f"{ring_str}"
            )

        except Exception as e:
            self.get_logger().error(f"发送失败: {e}")
            import traceback
            traceback.print_exc()

    def close_serial(self):
        self._recv_running = False
        if hasattr(self, "ser") and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已关闭")
        servo.shutdown()


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


if __name__ == "__main__":
    main()
