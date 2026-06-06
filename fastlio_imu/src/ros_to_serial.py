#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 -> Serial bridge for FCU communication.

Tx (PC -> FCU):
    Frame:  HEAD(AA) + P-flag(50) + pos(4d) + T-flag(54) + vel(6d) + S-flag(53) + ring(3f)
    Total:  1 + 1 + 32 + 1 + 48 + 1 + 12 = 96 bytes

Rx (FCU -> PC):
    AA 01 FF  — advance servo to next stage (0->40->80->120, 3 advances total)
"""

import rclpy
from rclpy.node import Node
from fastlio_imu.msg import FcuState
import serial
import struct
import math
import time
import random
import threading
import sys
import os
import statistics

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

CMD_HEAD  = 0xAA
CMD_OPEN   = 0x01          # FCU -> servo advance
CMD_START  = 0x02          # FCU -> start ring collection
CMD_STOP   = 0x03          # FCU -> stop ring collection, trimmed mean, lock
CMD_TAIL   = 0xFF

SEND_RING  = True        # True = include ring data in serial frame
TEST_RING  = False       # True = random ring coords (only when SEND_RING=True)

# ── ring collection parameters ──
COLLECT_MIN_SAMPLES  = 10      # minimum samples before stop (~5s at 2-3Hz)
COLLECT_TRIM_RATIO   = 0.20    # ±20% around median
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

        # --- sender queue (capacity 1, drop old) ---
        self._latest_frame = None
        self._frame_cond = threading.Condition()
        self._send_running = True

        self._recv_running = True

        # --- ring collection state machine ---
        self._collecting = False          # AA 02 start, AA 03 stop
        self._locked_ring = None          # (x, y, z) locked world position
        self._ring_buf = []               # list of (x, y, z)

        # --- servo ---
        servo.init()
        time.sleep(0.3)

        # --- FCU send thread (non-blocking for callback) ---
        self._send_thread = threading.Thread(target=self._serial_send_loop, daemon=True)
        self._send_thread.start()

        # --- FCU receive thread ---
        self._recv_thread = threading.Thread(target=self._serial_recv_loop, daemon=True)
        self._recv_thread.start()

        self.get_logger().info(
            f"订阅: {TOPIC_NAME}  |  "
            f"环数据: {'随机测试' if TEST_RING else '世界系(world frame)'}  |  "
            f"串口收发已启动"
        )

    # ---- FCU -> PC receive thread ----
    def _serial_recv_loop(self):
        buf = bytearray()
        while self._recv_running:
            try:
                if not self.ser.is_open:
                    break
                # read without lock — pySerial is thread-safe on Linux
                data = self.ser.read(1)
                if not data:
                    time.sleep(0.002)
                    continue
                buf.append(data[0])

                while buf and buf[0] != CMD_HEAD:
                    buf.pop(0)

                if len(buf) >= 3:
                    if buf[0] == CMD_HEAD and buf[2] == CMD_TAIL:
                        cmd = buf[1]
                        if cmd == CMD_OPEN:
                            self.get_logger().info("[FCU] 投放指令 AA 01 FF")
                            has_more = servo.next_stage()
                            if has_more:
                                self.get_logger().info(
                                    f"[servo] 等待下次指令 (当前 {servo.current_angle()}deg)")
                            else:
                                self.get_logger().info(
                                    f"[servo] 已到最终阶段 {servo.current_angle()}deg")
                        elif cmd == CMD_START:
                            self._collecting = True
                            self._ring_buf.clear()
                            self.get_logger().info("[ring] AA 02 FF → 开始采集")
                        elif cmd == CMD_STOP:
                            self._collecting = False
                            self._handle_stop_collect()
                        else:
                            self.get_logger().warn(f"[FCU] 未知指令: AA {cmd:02X} FF")
                        buf.clear()
                    else:
                        buf.pop(0)
            except Exception as e:
                self.get_logger().warn(f"串口接收异常: {e}")
                time.sleep(0.01)

    # ---- PC -> FCU: FcuState callback (fast, no serial I/O) ----
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

            # ring / spot data (world frame, origin = takeoff point)
            if SEND_RING:
                if not TEST_RING:
                    # collect during AA 02→AA 03 window
                    if self._collecting:
                        self._ring_buf.append((
                            msg.ring_world.position.x,
                            msg.ring_world.position.y,
                            msg.ring_world.position.z))
                    # use locked position if validated
                    if self._locked_ring is not None:
                        rx, ry, rz = self._locked_ring
                    else:
                        rx = msg.ring_world.position.x
                        ry = msg.ring_world.position.y
                        rz = msg.ring_world.position.z
                elif TEST_RING:
                    rx = random.uniform(-15.0, 15.0)
                    ry = random.uniform(-15.0, 15.0)
                    rz = random.uniform(0.0, 10.0)
                else:
                    rx = ry = rz = 0.0
                spot_data = struct.pack("<3d", rx, ry, rz)

            # assemble
            frame = FRAME_HEAD + POS_FLAG + pos_data + VEL_FLAG + vel_data
            if SEND_RING:
                frame += SPOT_FLAG + spot_data

            # non-blocking: push to sender thread
            with self._frame_cond:
                self._latest_frame = frame
                self._frame_cond.notify()

            self.get_logger().debug(
                f"TX {len(frame)}B  "
                f"pos=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f}rad"
            )

        except Exception as e:
            self.get_logger().error(f"打包失败: {e}")

    # ---- dedicated serial send thread ----
    def _serial_send_loop(self):
        while self._send_running:
            with self._frame_cond:
                self._frame_cond.wait(timeout=0.1)
                frame = self._latest_frame
                self._latest_frame = None
            if frame is not None:
                try:
                    self.ser.write(frame)
                except Exception as e:
                    self.get_logger().error(f"串口发送失败: {e}")

    # ── ring collection: median ±20% trimmed mean ──
    def _handle_stop_collect(self):
        n = len(self._ring_buf)
        self.get_logger().info(f"[ring] AA 03 FF → 停止采集，n={n}")
        if n < COLLECT_MIN_SAMPLES:
            self.get_logger().warn(f"[ring] 样本不足: {n} < {COLLECT_MIN_SAMPLES}")
            return

        def trimmed_mean_axis(vals):
            vals.sort()
            med = vals[n // 2]
            delta = abs(med) * COLLECT_TRIM_RATIO + 0.01
            keep = [v for v in vals if abs(v - med) <= delta]
            return sum(keep) / len(keep) if keep else med

        xs = [p[0] for p in self._ring_buf]
        ys = [p[1] for p in self._ring_buf]
        zs = [p[2] for p in self._ring_buf]

        avg = (trimmed_mean_axis(xs), trimmed_mean_axis(ys), trimmed_mean_axis(zs))
        std = (statistics.pstdev(xs), statistics.pstdev(ys), statistics.pstdev(zs))

        self.get_logger().info(
            f"[ring] 结果: avg=({avg[0]:.3f},{avg[1]:.3f},{avg[2]:.3f}) "
            f"std=({std[0]:.3f},{std[1]:.3f},{std[2]:.3f})")

        self._locked_ring = avg
        self.get_logger().info(f"[ring] 锁定坐标: ({avg[0]:.3f},{avg[1]:.3f},{avg[2]:.3f})，持续输出")

    def close_serial(self):
        self._send_running = False
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
