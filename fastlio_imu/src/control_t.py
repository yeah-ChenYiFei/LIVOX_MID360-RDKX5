import time
import os

PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000

# ==========================================
# ⚠️ 核心配置区（必须根据你的实际情况修改！）
# ==========================================
STOP_PW_US = 1900      # 之前校准出来的停止点

# --- 速度配置 ---
OPEN_PW_US = 2000      # 打开方向（正转），数值越大越快
CLOSE_PW_US = 1820     # 关闭方向（反转），数值越小越快
HOME_PW_US  = 1850     # 归位速度（慢速反转），介于STOP和CLOSE之间，慢一点撞击更安全

# --- 时间配置（核心！如果行程不一致，就改这两个时间！）---
OPEN_TIME  = 1.0       # 打开需要转多久（秒），自己实测调整
CLOSE_TIME = 1.0       # 关闭需要转多久（秒），通常关闭因摩擦力需要稍微长一点
HOME_TIME  = 2.0       # 归位最长等待时间（秒），保证能撞到限位即可
# ==========================================

def pwm_init(chip, channel):
    base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    export_path = f"/sys/class/pwm/pwmchip{chip}/export"
    unexport_path = f"/sys/class/pwm/pwmchip{chip}/unexport"
    
    # ✅ 第1步：先清理残留（无视报错），确保干净状态
    if os.path.exists(base):
        try:
            with open(unexport_path, "w") as f:
                f.write(str(channel))
        except:
            pass
        time.sleep(0.3)

    # ✅ 第2步：重新导出 PWM 通道
    try:
        with open(export_path, "w") as f:
            f.write(str(channel))
    except:
        pass  # 可能已经导出了，无视报错
    time.sleep(0.5)

    # ✅ 第3步：必须先禁用！！！（period=0时不能操作其他参数）
    try:
        with open(base + "/enable", "w") as f:
            f.write("0")
    except:
        pass
    time.sleep(0.1)

    # ✅ 第4步：设置周期（必须在 duty_cycle 之前！）
    with open(base + "/period", "w") as f:
        f.write(str(PERIOD_NS))
    time.sleep(0.1)

    # ✅ 第5步：设置占空比为0（不能等于period！否则某些驱动报错）
    with open(base + "/duty_cycle", "w") as f:
        f.write("0")
    time.sleep(0.1)

    # ✅ 第6步：现在才真正启用
    with open(base + "/enable", "w") as f:
        f.write("1")
    
    print("✅ PWM 初始化成功")

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

def airdrop_homing():
    """上电归位：寻找机械零点（关闭位置）"""
    print("\n🚀 正在归位（寻找关闭限位）...")
    print("舵机将慢速转向关闭方向，直到撞到挡块。")
    set_pulse_width(CLOSE_PW_US)  # 慢速反转
    time.sleep(HOME_TIME)        # 让它转足够久，确保撞到限位并死磕一会
    set_pulse_width(STOP_PW_US)  # 立刻停止，消除嗡嗡声
    print("✅ 归位完成！当前位置：关闭（初始位）")

def airdrop_open():
    print(f"→ 正在打开（转{OPEN_TIME}秒）...")
    set_pulse_width(OPEN_PW_US)
    time.sleep(OPEN_TIME)
    set_pulse_width(STOP_PW_US)
    print("✅ 打开完毕")

def airdrop_close():
    print(f"→ 正在关闭（转{CLOSE_TIME}秒）...")
    set_pulse_width(CLOSE_PW_US)
    time.sleep(CLOSE_TIME)
    set_pulse_width(STOP_PW_US)
    print("✅ 关闭完毕")

if __name__ == "__main__":
    pwm_init(PWM_CHIP, PWM_CHANNEL)
    time.sleep(0.5)  
    
    set_pulse_width(STOP_PW_US)
    time.sleep(0.5)  # 再等一下，让硬件稳定 
    # ⚠️ 每次上电，必须先归位！
    airdrop_homing()
    print("\n空投器准备就绪！")
    try:
        while True:
            cmd = input("输入指令：1=打开 | 2=关闭 | q=退出: ").strip().lower()
            if cmd == '1': 
                airdrop_open()
            elif cmd == '2': 
                airdrop_close()
            elif cmd == 'q': 
                break
    except KeyboardInterrupt:
        pass
    finally:
        pwm_off()
        print("\n退出，PWM已关闭")
