#!/bin/bash
LOG_FILE="/home/sunrise/livox_ws/src/fastlio_imu/log/lidar_auto.log"
mkdir -p "$(dirname "$LOG_FILE")"
exec &> >(tee -a "$LOG_FILE")

echo "========================================="
echo "启动时间: $(date)"
echo "========================================="

INTERFACE="eth0"
LIDAR_IP="192.168.1.186"
MAX_RETRIES=5
RETRY_DELAY=5

# 0. 封印系统的自动NTP，防止运行中连WiFi导致时间突变崩溃
sudo systemctl stop ntp.service 2>/dev/null

# 等待网络和雷达就绪
while ! ip link show "$INTERFACE" | grep -q "state UP"; do
    echo "等待网卡 $INTERFACE 启动..."
    sleep 2
done
while ! ip addr show "$INTERFACE" | grep -q "inet "; do
    echo "等待 IP 地址分配..."
    sleep 2
done
echo "等待雷达 IP $LIDAR_IP 可 ping 通..."
while ! ping -c 1 -W 2 "$LIDAR_IP" >/dev/null 2>&1; do
    echo "未 ping 通，等待 2 秒..."
    sleep 2
done
echo "雷达网络已就绪"

# === 核心改造：安全的时间初始化策略 ===
echo "检查系统时间有效性..."
CURRENT_YEAR=$(date +%Y)

if [ "$CURRENT_YEAR" -lt 2023 ]; then
    echo "警告：系统时间错误 ($CURRENT_YEAR 年)，尝试单次快速 NTP 同步（最多等待 5 秒）..."
    
    # 尝试单次同步，设置严格超时(-t 5)，不阻塞
    if sudo ntpdate -u -t 5 ntp.ubuntu.com >/dev/null 2>&1; then
        echo "✅ NTP 快速同步成功: $(date)"
    else
        echo "❌ 无网络或同步超时。为防止死循环，强制设定静态安全时间..."
        # 强制设定一个 2026 年的静态时间。只要不连网，时间就静止在这里。
        # 主板和雷达都会使用这个相同的静态时间，频率绝对稳定在 10Hz。
        sudo date -s "2026-01-01 00:00:00" >/dev/null 2>&1
        echo "✅ 已使用静态时间: $(date)"
    fi
else
    echo "✅ 系统时间已就绪: $(date)"
fi

# 等待雷达内部初始化
echo "等待雷达内部初始化 (15秒)..."
sleep 15

source /opt/ros/humble/setup.bash
source /home/sunrise/livox_ws/install/setup.bash

# ================= 以下逻辑保持不变 =================
while true; do
    for ((i=1; i<=MAX_RETRIES; i++)); do
        echo "===== 第 $i 次尝试启动驱动 ====="
        pkill -f "livox_ros_driver2_node" 2>/dev/null
        sleep 2

        ros2 launch livox_ros_driver2 msg_MID360_launch.py &
        LAUNCH_PID=$!

        echo "等待驱动节点启动并注册到 ROS DDS..."
        NODE_FOUND=false
        for ((j=1; j<=3; j++)); do
            sleep 2
            if ros2 node list 2>/dev/null | grep -q "livox_lidar_publisher"; then
                NODE_FOUND=true
                echo "✅ 节点已发现 (尝试 $j/3)"
                break
            fi
            echo "正在寻找节点... ($j/3)"
        done

        if ! $NODE_FOUND; then
            echo "❌ 错误：多次尝试后驱动节点仍未启动"
            kill $LAUNCH_PID 2>/dev/null
            sleep $RETRY_DELAY
            continue
        fi

        # 连续2次10Hz即放行
        CONFIRM_COUNT=0
        START_TIME=$(date +%s)
        TIMEOUT=60
        SUCCESS=false
        while [ $(($(date +%s) - START_TIME)) -lt $TIMEOUT ]; do
            FREQ_OUTPUT=$(ros2 topic hz /livox/lidar --window 5 2>/dev/null | grep "average rate" | tail -1)
            if [[ -n "$FREQ_OUTPUT" ]]; then
                FREQ_INT=$(echo "$FREQ_OUTPUT" | awk '{print $3}' | cut -d'.' -f1)
                echo "当前频率: $FREQ_INT Hz"
                if [[ "$FREQ_INT" -ge 9 && "$FREQ_INT" -le 11 ]]; then
                    ((CONFIRM_COUNT++))
                    if [[ $CONFIRM_COUNT -ge 2 ]]; then
                        SUCCESS=true
                        break
                    fi
                else
                    CONFIRM_COUNT=0
                fi
            else
                CONFIRM_COUNT=0
            fi
            sleep 2
        done

        if $SUCCESS; then
            echo "✅ 驱动启动成功，10Hz稳定。进入持续监控..."
            break 2
        else
            echo "❌ 未能稳定输出10Hz，杀死驱动并重试..."
            kill $LAUNCH_PID 2>/dev/null
            sleep $RETRY_DELAY
        fi
    done
    echo "❌ 重试次数用完，退出（systemd将重启服务）"
    exit 1
done

# 持续监控阶段
while true; do
    FAIL_COUNT=0
    while [ $FAIL_COUNT -lt 2 ]; do
        sleep 5
        FREQ_OUTPUT=$(ros2 topic hz /livox/lidar --window 5 2>/dev/null | grep "average rate" | tail -1)
        if [[ -n "$FREQ_OUTPUT" ]]; then
            FREQ_INT=$(echo "$FREQ_OUTPUT" | awk '{print $3}' | cut -d'.' -f1)
            echo "监控: 当前频率 $FREQ_INT Hz"
            if [[ "$FREQ_INT" -ge 9 && "$FREQ_INT" -le 11 ]]; then
                FAIL_COUNT=0
            else
                ((FAIL_COUNT++))
                echo "频率异常 ($FREQ_INT Hz)，失败计数 $FAIL_COUNT"
            fi
        else
            ((FAIL_COUNT++))
            echo "无法获取频率，失败计数 $FAIL_COUNT"
        fi
    done
    echo "⚠️ 雷达频率连续异常，准备重启驱动..."
    pkill -f "livox_ros_driver2_node" 2>/dev/null
    kill $LAUNCH_PID 2>/dev/null
    exit 1
done
