#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from fastlio_imu.msg import Newmsg
import serial

# ================== 配置区 ==================
SERIAL_PORT = '/dev/ttyS1'
BAUD_RATE = 115200
TOPIC_NAME = '/test/newmsg'

INT_TOTAL_LEN = 3    # 整数部分（含符号）固定3位
DECIMAL_DIGITS = 4   # 小数部分固定4位
# =============================================

class RosToSerialNode(Node):
    def __init__(self):
        super().__init__('ros_to_serial_node')
        try:
            self.ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
            self.get_logger().info(f"✅ 串口打开成功：{SERIAL_PORT}")
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

    def format_num(self, num):
        # 1. 四舍五入保留小数精度
        num_rounded = round(num, DECIMAL_DIGITS)
        
        # 2. 转字符串并拆分
        num_str = f"{num_rounded:.{DECIMAL_DIGITS}f}"
        if '.' in num_str:
            int_part_str, decimal_part_str = num_str.split('.', 1)
        else:
            int_part_str = num_str
            decimal_part_str = ''
        
        # 3. 处理整数部分（含符号）
        sign = '-' if '-' in int_part_str else ''
        int_digit_only = int_part_str.replace('-', '')  # 纯数字部分
        
        # 计算补零数量：总长度3 - 符号长度 - 现有数字长度
        zero_pad = INT_TOTAL_LEN - len(sign) - len(int_digit_only)
        
        # 如果计算出的补零数为负数，说明整数位超出限制，这里不做截断处理(视数据范围正常)
        if zero_pad < 0:
            zero_pad = 0
            
        int_digit_padded = '0' * zero_pad + int_digit_only
        int_total_str = sign + int_digit_padded
        
        # 4. 处理小数部分：确保4位
        decimal_str = decimal_part_str.ljust(DECIMAL_DIGITS, '0')[:DECIMAL_DIGITS]
        
        # 5. 最终拼接
        # 【关键修改】：去掉了之前 if int_total_str == '000': int_total_str = '00' 的逻辑
        formatted = f"{int_total_str}.{decimal_str}"
        
        return formatted

    def callback(self, msg):
        try:
            # 格式化所有需要发送的数值
            x = self.format_num(msg.pose.position.x)
            y = self.format_num(msg.pose.position.y)
            z = self.format_num(msg.pose.position.z)
            qx = self.format_num(msg.pose.orientation.x)
            qy = self.format_num(msg.pose.orientation.y)
            qz = self.format_num(msg.pose.orientation.z)
            qw = self.format_num(msg.pose.orientation.w)
            vx = self.format_num(msg.twist.linear.x)
            vy = self.format_num(msg.twist.linear.y)
            vz = self.format_num(msg.twist.linear.z)
            wx = self.format_num(msg.twist.angular.x)
            wy = self.format_num(msg.twist.angular.y)
            wz = self.format_num(msg.twist.angular.z)

            # 拼接完整字符串
            data_str = f"P{x}{y}{z}{qx}{qy}{qz}{qw}T{vx}{vy}{vz}{wx}{wy}{wz}\n"

            # 发送串口（前缀0xAA + 字符串编码）
            hex_header = b'\xAA'
            num_bytes = data_str.encode('utf-8')
            data_to_send = hex_header + num_bytes
            self.ser.write(data_to_send)

            # 日志输出（验证格式）
            self.get_logger().info(f"📤 发送成功 → {data_str.strip()}")
            
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
