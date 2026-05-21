import time
  import os

  PWM_CHIP = 0
  PWM_CHANNEL = 0
  PERIOD_NS = 20000000

  # 角度范围对应的脉宽（常见值，根据实际舵机微调）
  MIN_PW_US = 500    # 0度脉宽
  MAX_PW_US = 2500   # 180度脉宽


  def pwm_init(chip, channel):
      base = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
      if not os.path.exists(base):
          with open(f"/sys/class/pwm/pwmchip{chip}/export", "w") as f:
              f.write(str(channel))
          time.sleep(0.5)
      with open(base + "/period", "w") as f:
          f.write(str(PERIOD_NS))
      with open(base + "/duty_cycle", "w") as f:
          f.write("0")
      with open(base + "/enable", "w") as f:S
          f.write("1")


  def set_angle(degrees):           # degrees: 0~180
      """把角度映射为脉宽并输出"""
      degrees = max(0, min(180, degrees))   # 钳位到 [0, 180]
      # 线性映射：0°→MIN_PW_US, 180°→MAX_PW_US
      pw_us = MIN_PW_US + (MAX_PW_US - MIN_PW_US) * degrees / 180.0
      pw_ns = int(pw_us * 1000)

      base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
      with open(base + "/duty_cycle", "w") as f:
          f.write(str(pw_ns))
      print(f"舵机角度: {degrees}°  (脉宽: {pw_ns}ns / {pw_us:.0f}us)")


  def pwm_off():
      base = f"/sys/class/pwm/pwmchip{PWM_CHIP}/pwm{PWM_CHANNEL}"
      with open(base + "/enable", "w") as f:
          f.write("0")


  def angle_test():
      print("\n" + "=" * 50)
      print("角度舵机控制测试")
      print("=" * 50)
      print("输入 0~180 的角度值，舵机转到对应位置")
      print("输入 q 退出\n")

      while True:
          ans = input("输入角度 (0~180) 或 q: ").strip()
          if ans.lower() == 'q':
              break
          try:
              angle = float(ans)
              set_angle(angle)
          except ValueError:
              print("请输入有效数字！")


  if __name__ == "__main__":
      pwm_init(PWM_CHIP, PWM_CHANNEL)
      try:
          angle_test()
      finally:
          pwm_off()