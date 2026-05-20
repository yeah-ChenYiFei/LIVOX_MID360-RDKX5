import time
import os
import threading

PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000

# ==========================================
# 核心配置区（校准后更新 STOP_PW_US）
# ==========================================
STOP_PW_US = 1900      # 死区中心脉宽，用 calibrate_deadzone.py 校准后更新

# --- 速度配置 ---
OPEN_PW_US = 2000      # 打开方向（正转），离 STOP 越远越快
CLOSE_PW_US = 1820     # 关闭方向（反转）
HOME_PW_US  = 1850     # 归位速度（慢速反转）

# --- 时间配置 ---
OPEN_TIME  = 1.0
CLOSE_TIME = 1.0
HOME_TIME  = 2.0

# --- 驻留保持配置 ---
HOLD_REFRESH_INTERVAL = 0.1   # 空闲时每隔 0.1s 重写停止脉冲，防止漂移
# ==========================================

# 空闲时是否正在保持停止
_holding = False
_hold_lock = threading.Lock()


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

    print("PWM 初始化成功")


def _write_duty(ns):
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/duty_cycle", "w") as f:
        f.write(str(ns))


def set_pulse_width(us):
    """设置脉宽，不重复写 enable"""
    _write_duty(us * 1000)


def pwm_off():
    base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
    with open(base + "/enable", "w") as f:
        f.write("0")


def _refresh_stop():
    """连续两次写入停止脉宽，确保信号稳定"""
    set_pulse_width(STOP_PW_US)
    time.sleep(0.03)
    set_pulse_width(STOP_PW_US)


def airdrop_homing():
    print("\n  正在归位（寻找关闭限位）...")
    set_pulse_width(CLOSE_PW_US)
    time.sleep(HOME_TIME)
    _refresh_stop()
    print("  归位完成！当前位置：关闭（初始位）")


def airdrop_open():
    global _holding
    with _hold_lock:
        _holding = False
    print(f"  正在打开（转{OPEN_TIME}秒）...")
    set_pulse_width(OPEN_PW_US)
    time.sleep(OPEN_TIME)
    _refresh_stop()
    with _hold_lock:
        _holding = True
    print("  打开完毕，已驻留停止")


def airdrop_close():
    global _holding
    with _hold_lock:
        _holding = False
    print(f"  正在关闭（转{CLOSE_TIME}秒）...")
    set_pulse_width(CLOSE_PW_US)
    time.sleep(CLOSE_TIME)
    _refresh_stop()
    with _hold_lock:
        _holding = True
    print("  关闭完毕，已驻留停止")


def _hold_loop():
    """后台线程：空闲时持续重写停止脉冲，防止漂移"""
    global _holding
    while True:
        with _hold_lock:
            if _holding:
                set_pulse_width(STOP_PW_US)
        time.sleep(HOLD_REFRESH_INTERVAL)


if __name__ == "__main__":
    pwm_init(PWM_CHIP, PWM_CHANNEL)
    time.sleep(0.3)

    _refresh_stop()
    with _hold_lock:
        _holding = True

    # 启动驻留保持线程
    hold_thread = threading.Thread(target=_hold_loop, daemon=True)
    hold_thread.start()

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
        with _hold_lock:
            _holding = False
        pwm_off()
        print("\n退出，PWM已关闭")
