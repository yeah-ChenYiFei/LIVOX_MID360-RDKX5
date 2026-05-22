import time
import os

PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000

def pwm_init(chip, channel):
    base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    export_path = f"/sys/class/pwm/pwmchip{chip}/export"
    unexport_path = f"/sys/class/pwm/pwmchip{chip}/unexport"

    if os.path.exists(base):
        try:
            with open(unexport_path, "w") as f:
                f.write(str(channel))
        except:
            pass
        time.sleep(0.3)

    try:
        with open(export_path, "w") as f:
            f.write(str(channel))
    except:
        pass
    time.sleep(0.5)

    try:
        with open(base + "/enable", "w") as f:
            f.write("0")
    except:
        pass
    time.sleep(0.1)

    with open(base + "/period", "w") as f:
        f.write(str(PERIOD_NS))
    time.sleep(0.1)

    with open(base + "/duty_cycle", "w") as f:
        f.write("0")
    time.sleep(0.1)

    with open(base + "/enable", "w") as f:
        f.write("1")
    print("PWM init OK")

def set_pulse(us):
    ns = us * 1000
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(ns))

def pwm_off():
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/enable", "w") as f:
        f.write("0")

def find_deadzone():
    print("\n" + "=" * 55)
    print("  360度舵机 死区精密扫描 (1us 步进)")
    print("=" * 55)
    print("目标: 找到让舵机【静止不动】的脉宽中心值")
    print("步骤: 先找上边界(开始正转) → 再找下边界(开始反转)")
    print("      死区中心 = (上边界 + 下边界) / 2\n")

    # ---- 第一步: 找到近似停止点 ----
    print("【第0步】先找到近似停止范围 (10us粗扫)")
    current = 1500
    print(f"从 {current}us 开始, 观察舵机正转还是反转...")
    while True:
        set_pulse(current)
        time.sleep(0.4)
        resp = input(f"{current}us → 正转(+)/反转(-)/不动(0)? ").strip()
        if resp == '+':
            current -= 20
        elif resp == '-':
            current += 20
        elif resp == '0':
            break
        elif resp == 'q':
            pwm_off()
            return

    approx_stop = current
    print(f"近似停止点: {approx_stop}us\n")

    # ---- 第二步: 向上扫描找上边界 ----
    print("【第1步】向上扫描找死区上边界 (开始正转的临界点)")
    current = approx_stop
    while True:
        set_pulse(current)
        time.sleep(0.5)
        resp = input(f"{current}us → 舵机在转吗? (y=在转/n=不动/q=跳过)? ").strip().lower()
        if resp == 'y':
            dead_upper = current  # 记录开始转的脉宽
            print(f"上边界: {dead_upper}us (以此为界，再大1us就会正转)\n")
            break
        elif resp == 'n':
            current += 1
        elif resp == 'q':
            dead_upper = current
            print(f"跳过，记录当前为上边界: {dead_upper}us\n")
            break

    # ---- 第三步: 向下扫描找下边界 ----
    print("【第2步】向下扫描找死区下边界 (开始反转的临界点)")
    current = approx_stop
    while True:
        set_pulse(current)
        time.sleep(0.5)
        resp = input(f"{current}us → 舵机在转吗? (y=在转/n=不动/q=跳过)? ").strip().lower()
        if resp == 'y':
            dead_lower = current
            print(f"下边界: {dead_lower}us (以此为界，再小1us就会反转)\n")
            break
        elif resp == 'n':
            current -= 1
        elif resp == 'q':
            dead_lower = current
            print(f"跳过，记录当前为下边界: {dead_lower}us\n")
            break

    # ---- 第四步: 计算死区中心 ----
    dead_center = (dead_upper + dead_lower) // 2
    dead_width = dead_upper - dead_lower

    print("=" * 55)
    print(f"  死区上边界: {dead_upper} us")
    print(f"  死区下边界: {dead_lower} us")
    print(f"  死区宽度:   {dead_width} us")
    print(f"  死区中心:   {dead_center} us  ← 填入 servo_control.py 的 STOP_PW_US")
    print("=" * 55)

    # ---- 第五步: 验证 ----
    print(f"\n【验证】写入死区中心 {dead_center}us，观察舵机是否完全锁死...")
    set_pulse(dead_center)
    print("请用手轻拨摆杆测试。如果拨动后它缓慢自转，")
    print(f"说明死区中心需要微调。")
    print(f"建议: 用 1us 为步进微调，直到拨动后不转为止。")

    pwm_off()
    return dead_center

if __name__ == "__main__":
    pwm_init(PWM_CHIP, PWM_CHANNEL)
    time.sleep(0.5)
    find_deadzone()
