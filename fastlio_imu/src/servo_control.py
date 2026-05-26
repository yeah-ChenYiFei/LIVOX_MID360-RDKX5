"""
Servo staged control via sysfs PWM.
Protocol: advance one stage per FCU command, only forward.
Stages: 0deg -> 40deg -> 80deg -> 120deg (3 advances total).
"""

import time
import os

PWM_CHIP = 0
PWM_CHANNEL = 0
PERIOD_NS = 20000000       # 50 Hz
MIN_PW_US = 500              # 0 deg
MAX_PW_US = 2500             # 180 deg

STAGES = [0, 40, 80, 120]    # advance-only stages

_current_stage = 0
_initialized = False


def _base():
    return f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"


def init():
    """Export PWM channel, initialise to stage 0."""
    global _initialized, _current_stage
    base = _base()
    try:
        if not os.path.exists(base):
            with open(f"/sys/class/pwm/pwmchip{PWM_CHIP}/export", "w") as f:
                f.write(str(PWM_CHANNEL))
            time.sleep(0.5)
        with open(base + "/period", "w") as f:
            f.write(str(PERIOD_NS))
        with open(base + "/enable", "w") as f:
            f.write("1")
        _initialized = True
        _current_stage = 0
        _write_angle(STAGES[0])
        print(f"[servo] 初始化完成  chip{PWM_CHIP}:ch{PWM_CHANNEL}  {STAGES[0]}deg")
    except (PermissionError, FileNotFoundError, OSError) as e:
        print(f"[servo] 初始化失败: {e}")
        _initialized = False


def _write_angle(degrees: float):
    """Set servo angle via duty_cycle (no bounds warning)."""
    clamped = max(0.0, min(180.0, degrees))
    pw_us = MIN_PW_US + (MAX_PW_US - MIN_PW_US) * clamped / 180.0
    pw_ns = int(pw_us * 1000)
    try:
        with open(_base() + "/duty_cycle", "w") as f:
            f.write(str(pw_ns))
    except (PermissionError, FileNotFoundError, OSError) as e:
        print(f"[servo] 写入duty_cycle失败: {e}")


def set_angle(degrees: float):
    """Public API: move servo to an arbitrary angle (0-180deg, clamped)."""
    clamped = max(0.0, min(180.0, degrees))
    _write_angle(clamped)
    print(f"[servo] 设置角度: {clamped:.1f}deg")


def next_stage():
    """Advance servo to the next stage.

    Returns:
        True   more stages remain after this advance
        False  already at the final stage (no-op)
    """
    global _current_stage
    if _current_stage >= len(STAGES) - 1:
        print(f"[servo] 已在最后阶段 ({STAGES[-1]}deg)，无法继续推进")
        return False
    _current_stage += 1
    _write_angle(STAGES[_current_stage])
    print(f"[servo] >>> 阶段 {_current_stage}/{len(STAGES)-1}: {STAGES[_current_stage]}deg")
    return _current_stage < len(STAGES) - 1


def current_angle() -> int:
    """Return current stage angle in degrees."""
    return STAGES[_current_stage]


def shutdown():
    """Disable PWM output."""
    try:
        with open(_base() + "/enable", "w") as f:
            f.write("0")
        print("[servo] PWM 已关闭")
    except (PermissionError, FileNotFoundError, OSError) as e:
        print(f"[servo] PWM关闭失败: {e}")


def pwm_off():
    """Disable PWM output (alias for shutdown)."""
    shutdown()
