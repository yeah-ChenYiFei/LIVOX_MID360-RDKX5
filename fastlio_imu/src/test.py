import time
import os

PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000

def pwm_init(chip, channel):
    base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    if not os.path.exists(base):
        with open(f"/sys/class/pwm/pwmchip{chip}/export", "w") as f:
            f.write(str(channel))
        time.sleep(0.5)
    with open(base + "/enable", "w") as f:
        f.write("0")
    with open(base + "/period", "w") as f:
        f.write(str(PERIOD_NS))
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(PERIOD_NS))

def set_pulse_width(us):
    ns = us * 1000
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(ns))
    with open(base + "/enable", "w") as f:
        f.write("1")

def pwm_off():
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/enable", "w") as f:
        f.write("0")

def calibrate_360():
    print("\n" + "="*50)
    print("🛠️ 360度舵机 停止点校准 (粗调+细调)")
    print("="*50)
    print("目标：找到让舵机【完全静止不动】的脉宽值")
    print("如果它在慢转/摆动，说明还没找到停止点！\n")
    
    current_pw = 1500  # 从1500开始
    
    # ---------- 第一阶段：粗调 (步进20us) ----------
    print("【第一阶段：粗调】(每次变化20us)")
    print("观察舵机：如果在转，就输入相反方向让它慢下来！\n")
    
    while True:
        set_pulse_width(current_pw)
        time.sleep(0.3)
        print(f"当前脉宽: {current_pw}us - 舵机状态？")
        print("  W/↑: 还在正转(变大) | S/↓: 还在反转(变小) | D: 停下不动了！ | Q: 退出")
        ans = input("请选择: ").strip().lower()
        
        if ans in ['w', 'up']:
            current_pw += 20  # 粗调加20
        elif ans in ['s', 'down']:
            current_pw -= 20  # 粗调减20
        elif ans == 'd':
            print(f"\n✅ 粗调完成！大致停止点在 {current_pw}us 附近")
            break
        elif ans == 'q':
            pwm_off()
            exit()

    # ---------- 第二阶段：细调 (步进2us) ----------
    print("\n【第二阶段：细调】(每次变化2us)")
    print("舵机现在可能还有轻微抖动或极慢爬行，微调到彻底锁死！\n")
    
    while True:
        set_pulse_width(current_pw)
        time.sleep(0.3)
        print(f"当前脉宽: {current_pw}us - 舵机状态？")
        print("  W/↑: 微微正转(加2) | S/↓: 微微反转(减2) | D: 完美锁死不动！ | Q: 退出")
        ans = input("请选择: ").strip().lower()
        
        if ans in ['w', 'up']:
            current_pw += 2  # 细调加2
        elif ans in ['s', 'down']:
            current_pw -= 2  # 细调减2
        elif ans == 'd':
            print("\n" + "="*50)
            print(f"🎉🎉🎉 校准成功！精准停止点为：{current_pw}us 🎉🎉🎉")
            print("="*50)
            print(f"👉 请把代码开头的 STOP_PW_US 修改为 {current_pw}\n")
            break
        elif ans == 'q':
            break
            
    pwm_off()

if __name__ == "__main__":
    pwm_init(PWM_CHIP, PWM_CHANNEL)
    calibrate_360()
