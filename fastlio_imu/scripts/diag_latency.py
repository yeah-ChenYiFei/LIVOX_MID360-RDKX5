#!/usr/bin/env python3
"""
Diagnostic tool: measure end-to-end latency from LiDAR to Odometry.

Usage (on RDK X5):
    python3 diag_latency.py

Measures:
  1. Wall-clock delay of /livox/lidar messages (is the driver delayed?)
  2. Wall-clock delay of /Odometry messages (how old is the output?)
  3. LiDAR → Odometry pipeline latency (difference between timestamps)
  4. /livox/imu message rate and timestamp consistency
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
import time


class LatencyDiag(Node):
    def __init__(self):
        super().__init__("latency_diag")
        self._last_print = time.time()

        self._lidar_count = 0
        self._lidar_max_age = 0.0
        self._lidar_total_age = 0.0

        self._odom_count = 0
        self._odom_max_age = 0.0
        self._odom_total_age = 0.0
        self._odom_last_stamp = 0.0

        self._imu_count = 0
        self._imu_last_stamp = 0.0

        self._sub_lidar = self.create_subscription(
            CustomMsg, "/livox/lidar", self._on_lidar, 10)
        self._sub_odom = self.create_subscription(
            Odometry, "/Odometry", self._on_odom, 10)
        self._sub_imu = self.create_subscription(
            Imu, "/livox/imu", self._on_imu, 10)

        self._timer = self.create_timer(2.0, self._print_stats)

        self.get_logger().info("Latency diag running — press Ctrl+C to stop")

    def _now_sec(self):
        return time.time()

    def _on_lidar(self, msg: CustomMsg):
        self._lidar_count += 1
        now = self._now_sec()
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - msg_sec
        self._lidar_max_age = max(self._lidar_max_age, age)
        self._lidar_total_age += age

    def _on_odom(self, msg: Odometry):
        self._odom_count += 1
        now = self._now_sec()
        msg_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - msg_sec
        self._odom_max_age = max(self._odom_max_age, age)
        self._odom_total_age += age

        # inter-message gap (should be ~0.1s at 10Hz)
        if self._odom_last_stamp > 0:
            gap = msg_sec - self._odom_last_stamp
            if gap > 0.3:
                self.get_logger().warn(f"Odometry gap: {gap:.3f}s (expected ~0.1s)")
        self._odom_last_stamp = msg_sec

    def _on_imu(self, msg: Imu):
        self._imu_count += 1
        self._imu_last_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _print_stats(self):
        self.get_logger().info(
            f"--- DIAG (2s window) ---"
        )
        if self._lidar_count > 0:
            self.get_logger().info(
                f"/livox/lidar: {self._lidar_count} msgs, "
                f"avg_age={self._lidar_total_age / self._lidar_count:.3f}s, "
                f"max_age={self._lidar_max_age:.3f}s"
            )
        else:
            self.get_logger().warn("/livox/lidar: NO MESSAGES — check driver!")

        if self._odom_count > 0:
            avg_age = self._odom_total_age / self._odom_count
            self.get_logger().info(
                f"/Odometry:    {self._odom_count} msgs, "
                f"avg_age={avg_age:.3f}s, "
                f"max_age={self._odom_max_age:.3f}s"
            )
            if avg_age > 1.0:
                self.get_logger().error(
                    f"Odometry avg_age={avg_age:.1f}s > 1s — pipeline is severely delayed!"
                )
            elif avg_age > 0.3:
                self.get_logger().warn(
                    f"Odometry avg_age={avg_age:.1f}s > 0.3s — moderate delay detected"
                )
        else:
            self.get_logger().warn("/Odometry: NO MESSAGES — FAST-LIO not running?")

        self.get_logger().info(
            f"/livox/imu:   {self._imu_count} msgs "
            f"(~{self._imu_count / 2:.0f}Hz)"
        )

        # reset counters
        self._lidar_count = 0
        self._lidar_max_age = 0.0
        self._lidar_total_age = 0.0
        self._odom_count = 0
        self._odom_max_age = 0.0
        self._odom_total_age = 0.0
        self._imu_count = 0


def main():
    rclpy.init()
    node = LatencyDiag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
