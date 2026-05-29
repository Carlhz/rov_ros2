#!/bin/bash
# ============================================================
# ROV INS 控制台 一键启动脚本
# 使用方法：双击运行，或 bash launch_ins_gui.sh
# ============================================================

ROS_DISTRO="foxy"
WS_DIR="$HOME/rov_ros2_ws"
ROS_DOMAIN_ID=42

echo "=============================="
echo "  ROV INS 控制台 启动中..."
echo "=============================="

# 1. Source ROS2
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    echo "[OK] ROS2 ${ROS_DISTRO} 已加载"
else
    echo "[ERROR] 找不到 ROS2 ${ROS_DISTRO}，请检查安装"
    read -p "按 Enter 退出..."
    exit 1
fi

# 2. Source 工作空间
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source ${WS_DIR}/install/setup.bash
    echo "[OK] 工作空间已加载: ${WS_DIR}"
else
    echo "[WARN] 工作空间未编译，正在自动编译..."
    cd ${WS_DIR}
    colcon build --packages-select rov_ins_interface rov_ins_control rov_ins_monitor
    source install/setup.bash
    echo "[OK] 编译完成"
fi

# 3. 设置环境变量
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0
echo "[OK] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 4. VMware 兼容性：禁用 GPU 渲染，避免 RenderAddGlyphs 崩溃
export LIBGL_ALWAYS_SOFTWARE=1
export GDK_BACKEND=x11
export DISPLAY=${DISPLAY:-:0}
echo "[OK] VMware 兼容模式已启用"

# 4. 启动 GUI
echo "[OK] 启动 GUI..."
cd ${WS_DIR}
python3 src/rov_ins_control/rov_ins_control/ins_gui.py
