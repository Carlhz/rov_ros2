#!/bin/bash
# ═══════════════════════════════════════════════════════
#  ROV 手柄控制器一键启动 (VM 端)
#  纯 Python 直读 joystick，无需 ros-foxy-joy
#  运行: bash ~/rov_ros2_ws/start_joy.sh
# ═══════════════════════════════════════════════════════

source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║     ROV 手柄控制器                          ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# 检查手柄
if ls /dev/input/js0 &>/dev/null; then
    echo "[OK] 手柄: /dev/input/js0"
    cat /proc/bus/input/devices 2>/dev/null | grep -A1 "js0" | grep "Name=" | head -1 | sed 's/N: /  设备: /'
else
    echo "[ERROR] 未找到手柄 /dev/input/js0"
    echo "  请连接手柄后重试"
    exit 1
fi

# 检查权限
if [ -r /dev/input/js0 ]; then
    echo "[OK] 手柄可读"
else
    echo "[WARN] 手柄无读权限，尝试修复..."
    echo "  请执行: echo 159357 | sudo -S usermod -a -G input carl"
    echo "  然后注销重新登录"
    exit 1
fi

echo ""

# 检查 RK3588 电机控制器
echo "── 检查 RK3588 连接 ──"
if timeout 3 ros2 topic list 2>/dev/null | grep -q "/rov/cmd_vel"; then
    echo "[OK] /rov/cmd_vel 已连接（RK3588 在线）"
else
    echo "[INFO] /rov/cmd_vel 不在线（如未启动电机控制器可忽略）"
fi
echo ""

# 启动手柄控制器（内含 joystick 直读）
echo "[>>] 启动 ROV joy_controller（Ctrl+C 退出）"
echo ""
echo "操作说明:"
echo "  左摇杆上: 下潜        左摇杆下: 上浮"
echo "  右摇杆上: 前进        右摇杆下: 后退"
echo "  右摇杆左: 左转        右摇杆右: 右转"
echo "  A:     急停          B:   恢复控制"
echo "  X:     开/关定航向 (自动回正到指定yaw)"
echo "  Y:     开/关深度悬停"
echo "  LB/RB: 降/升档 / 调目标深度(悬停时) / 调航向±5度(定航时)"
echo "  --scan: 扫描所有按键和轴映射"
echo "  档位: 1档=1100~1200  2档=1100~1400  3档=1100~1600 (尾部基准)"
echo ""

python3 "${SCRIPT_DIR}/monitor/joy_controller.py" "$@"

echo ""
echo "手柄控制器已退出"
