import time
import os

# ---------- 配置 ----------
PWM_CHIP = 0       # PWM0（需确认29引脚对应PWM0）
PWM_CHANNEL = 0    # 通道0
PERIOD = 20000     # 周期 20ms = 50Hz（单位：微秒）
MIN_PW = 750      # 1.0ms → -60°（左极限）
MAX_PW = 2250      # 2.0ms → +60°（右极限）

# ---------- 初始化 PWM ----------
def pwm_init(chip, channel):
    base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    # 导出通道
    if not os.path.exists(base):
        with open(f"/sys/class/pwm/pwmchip{chip}/export", "w") as f:
            f.write(str(channel))
    # 设置周期
    with open(base + "/period", "w") as f:
        f.write(str(PERIOD))
    # 先关输出
    with open(base + "/enable", "w") as f:
        f.write("0")

# ---------- 设置角度（适配MG996R） ----------
def servo_set_angle(chip, channel, angle):
    # 将代码中的angle（0~180）映射到舵机有效角度（-60~+60）
    servo_angle = (angle / 180) * 120 - 60  # 0→-60°, 90→0°, 180→+60°
    # 限幅到舵机有效范围
    servo_angle = max(-60, min(60, servo_angle))
    # 映射到脉宽（1.0ms~2.0ms）
    pw = MIN_PW + (MAX_PW - MIN_PW) * (servo_angle + 60) / 120
    base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    # 设置脉宽
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(int(pw)))
    # 开启输出
    with open(base + "/enable", "w") as f:
        f.write("1")

# ---------- 主循环 ----------
if __name__ == "__main__":
    pwm_init(PWM_CHIP, PWM_CHANNEL)
    try:
        while True:
            print("-60°（左极限）")
            servo_set_angle(PWM_CHIP, PWM_CHANNEL, 0)   # 代码angle=0 → 舵机-60°
            time.sleep(2)

            print("0°（中心）")
            servo_set_angle(PWM_CHIP, PWM_CHANNEL, 90)  # 代码angle=90 → 舵机0°
            time.sleep(2)

            print("+60°（右极限）")
            servo_set_angle(PWM_CHIP, PWM_CHANNEL, 180) # 代码angle=180 → 舵机+60°
            time.sleep(2)
    except KeyboardInterrupt:
        # 退出时关闭 PWM
        base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
        with open(base + "/enable", "w") as f:
            f.write("0")
        print("\n退出，PWM 已关闭")
