import time
import os

# ================== PWM 配置 ==================
PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000
MIN_PW_US = 500     # 0度
MAX_PW_US = 2500    # 180度

# ================== 角度配置 ==================
ANGLE_CLOSE = 0     # 锁止/关闭
ANGLE_OPEN  = 90    # 投放/打开

_initialized = False


def init():
    global _initialized
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    if not os.path.exists(base):
        with open(f"/sys/class/pwm/pwmchip{PWM_CHIP}/export", "w") as f:
            f.write(str(PWM_CHANNEL))
        time.sleep(0.5)
    with open(base + "/period", "w") as f:
        f.write(str(PERIOD_NS))
    with open(base + "/duty_cycle", "w") as f:
        f.write("0")
    with open(base + "/enable", "w") as f:
        f.write("1")
    _initialized = True
    print("[servo] PWM 初始化完成")


def set_angle(degrees):
    degrees = max(0, min(180, degrees))
    pw_us = MIN_PW_US + (MAX_PW_US - MIN_PW_US) * degrees / 180.0
    pw_ns = int(pw_us * 1000)
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(pw_ns))
    print(f"[servo] 角度: {degrees:.0f}°  (脉宽: {pw_us:.0f}us)")


def open():
    print("[servo] >>> 投放！")
    set_angle(ANGLE_OPEN)


def close():
    set_angle(ANGLE_CLOSE)
    print("[servo] 已锁止")


def off():
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/enable", "w") as f:
        f.write("0")
    print("[servo] PWM 已关闭")


# ================== 独立测试 ==================
if __name__ == "__main__":
    init()
    close()
    print("\n指令: 1=打开 2=关闭 q=退出")
    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == '1':
                open()
            elif cmd == '2':
                close()
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        off()
