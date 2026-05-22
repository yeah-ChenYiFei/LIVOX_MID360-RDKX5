"""
Interactive servo angle test utility.
Uses servo_control module for PWM hardware access.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import servo_control


def angle_test():
    print("\n" + "=" * 50)
    print("舵机角度控制测试")
    print("=" * 50)
    print("输入 0~180 的角度值，舵机转到对应位置")
    print("输入 q 退出\n")

    while True:
        ans = input("输入角度 (0~180) 或 q: ").strip()
        if ans.lower() == 'q':
            break
        try:
            angle = float(ans)
            servo_control.set_angle(angle)
        except ValueError:
            print("请输入有效数字！")


if __name__ == "__main__":
    servo_control.init()
    try:
        angle_test()
    finally:
        servo_control.pwm_off()
