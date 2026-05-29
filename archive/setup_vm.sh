#!/bin/bash
# Ubuntu VM 一键部署脚本
# 在 ~/rov_ros2 目录下运行

set -e

echo "=========================================="
echo "ROV ROS2 Foxy - Ubuntu VM 部署脚本"
echo "=========================================="

# 检查 ROS2 Foxy
if [ ! -f /opt/ros/foxy/setup.bash ]; then
    echo "错误: 未找到 ROS2 Foxy"
    echo "请先安装 ROS2 Foxy"
    exit 1
fi

# 设置 DDS 环境
echo ""
echo "[1/5] 配置 DDS 多机通信..."
if ! grep -q "ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
    echo "✓ DDS 配置已添加到 ~/.bashrc"
else
    echo "✓ DDS 配置已存在"
fi

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 创建工作空间
echo ""
echo "[2/5] 创建工作空间..."
WS_DIR="$HOME/rov_ros2_ws"
mkdir -p "$WS_DIR/src"
cd "$WS_DIR"

# 复制源代码
echo ""
echo "[3/5] 复制源代码..."
if [ -d "$HOME/rov_ros2/src" ]; then
    cp -r "$HOME/rov_ros2/src/"* "$WS_DIR/src/"
    echo "✓ 源代码已复制"
else
    echo "⚠ 未找到 $HOME/rov_ros2/src，请确认已拷贝 rov_ros2 文件夹"
    exit 1
fi

# 构建
echo ""
echo "[4/5] 构建 ROS2 包..."
source /opt/ros/foxy/setup.bash

# 安装依赖
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y || true

# 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✓ 构建完成"

# 创建启动脚本
echo ""
echo "[5/5] 创建启动脚本..."
cat > "$WS_DIR/start_monitor.sh" << 'EOF'
#!/bin/bash
# 启动上位机监控

WS_DIR="$HOME/rov_ros2_ws"
cd "$WS_DIR"

source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "=========================================="
echo "启动 INS 监控节点"
echo "=========================================="
echo ""
echo "可用命令:"
echo "  start  - 启动 INS 数据输出"
echo "  stop   - 停止 INS 数据输出"
echo "  lat X  - 设置纬度"
echo "  lon X  - 设置经度"
echo "  status - 显示统计"
echo "  quit   - 退出"
echo ""

ros2 launch rov_topside topside.launch.py
EOF

chmod +x "$WS_DIR/start_monitor.sh"

cat > "$WS_DIR/quick_test.sh" << 'EOF'
#!/bin/bash
# 快速测试 - 只查看话题

WS_DIR="$HOME/rov_ros2_ws"
cd "$WS_DIR"

source /opt/ros/foxy/setup.bash
source install/setup.bash

echo "=========================================="
echo "ROS2 话题测试"
echo "=========================================="
echo ""
echo "当前话题列表:"
ros2 topic list
echo ""
echo "查看 INS 数据 (按 Ctrl+C 退出):"
ros2 topic echo /ins/data
EOF

chmod +x "$WS_DIR/quick_test.sh"

echo "✓ 启动脚本已创建"

echo ""
echo "=========================================="
echo "部署完成！"
echo "=========================================="
echo ""
echo "使用方法:"
echo ""
echo "1. 重新加载环境:"
echo "   source ~/.bashrc"
echo ""
echo "2. 启动监控 (交互式):"
echo "   cd ~/rov_ros2_ws"
echo "   ./start_monitor.sh"
echo ""
echo "3. 快速测试 (仅查看话题):"
echo "   cd ~/rov_ros2_ws"
echo "   ./quick_test.sh"
echo ""
echo "4. 手动发送命令:"
echo "   ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: \"start\"}' --once"
echo ""
echo "确保 RK3588 上的 ins_driver_node 已在运行！"
echo ""
