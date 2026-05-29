#!/bin/bash
# ROS2 Foxy 部署脚本 - 部署到 RK3588
# 用法: ./deploy_to_rk3588.sh [RK3588_IP]

set -e

# 配置
RK3588_IP=${1:-172.16.28.82}
RK3588_USER=root
WORKSPACE_DIR="/opt/ros/rov_ros2_ws"
LOCAL_WS="$(cd "$(dirname "$0")/.." && pwd)"

echo "=========================================="
echo "ROS2 Foxy 部署到 RK3588"
echo "=========================================="
echo "目标: ${RK3588_USER}@${RK3588_IP}"
echo "本地工作空间: ${LOCAL_WS}"
echo "远程工作空间: ${WORKSPACE_DIR}"
echo ""

# 检查 SSH 连接
echo "[1/6] 检查 SSH 连接..."
if ! ssh -o ConnectTimeout=5 "${RK3588_USER}@${RK3588_IP}" "echo 'SSH OK'" > /dev/null 2>&1; then
    echo "错误: 无法连接到 RK3588 (${RK3588_IP})"
    echo "请检查:"
    echo "  1. RK3588 是否开机"
    echo "  2. 网络是否连通 (ping ${RK3588_IP})"
    echo "  3. SSH 服务是否运行"
    exit 1
fi
echo "✓ SSH 连接正常"

# 创建远程工作空间
echo ""
echo "[2/6] 创建远程工作空间..."
ssh "${RK3588_USER}@${RK3588_IP}" "mkdir -p ${WORKSPACE_DIR}/src"
echo "✓ 工作空间已创建"

# 同步源代码
echo ""
echo "[3/6] 同步源代码到 RK3588..."
rsync -avz --delete \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='.git/' \
    "${LOCAL_WS}/src/" \
    "${RK3588_USER}@${RK3588_IP}:${WORKSPACE_DIR}/src/"
echo "✓ 源代码同步完成"

# 在 RK3588 上构建
echo ""
echo "[4/6] 在 RK3588 上构建 ROS2 包..."
ssh "${RK3588_USER}@${RK3588_IP}" << EOF
    cd ${WORKSPACE_DIR}
    
    # Source ROS2 Foxy
    source /opt/ros/setup.bash
    
    # 安装依赖
    echo "安装依赖..."
    rosdep install --from-paths src --ignore-src -y || true
    
    # 构建
    echo "开始构建..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    echo "✓ 构建完成"
EOF

# 创建启动脚本
echo ""
echo "[5/6] 创建启动脚本..."
ssh "${RK3588_USER}@${RK3588_IP}" << EOF
    cat > ${WORKSPACE_DIR}/start_ins_driver.sh << 'SCRIPT'
#!/bin/bash
# INS Driver 启动脚本

WS_DIR="/opt/ros/rov_ros2_ws"

# Source ROS2
source /opt/ros/setup.bash
source \${WS_DIR}/install/setup.bash

# 设置 DDS 发现（多机通信）
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

echo "=========================================="
echo "启动 INS Driver Node"
echo "=========================================="

# 启动节点
ros2 launch rov_ins_driver ins_driver.launch.py
SCRIPT
    chmod +x ${WORKSPACE_DIR}/start_ins_driver.sh
    
    cat > ${WORKSPACE_DIR}/start_ins_driver_bg.sh << 'SCRIPT'
#!/bin/bash
# INS Driver 后台启动脚本

WS_DIR="/opt/ros/rov_ros2_ws"

source /opt/ros/setup.bash
source \${WS_DIR}/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 后台启动
nohup ros2 run rov_ins_driver ins_driver_node > /tmp/ins_driver.log 2>&1 &
echo "INS Driver 已在后台启动"
echo "查看日志: tail -f /tmp/ins_driver.log"
SCRIPT
    chmod +x ${WORKSPACE_DIR}/start_ins_driver_bg.sh
    
    echo "✓ 启动脚本已创建"
EOF

# 创建 systemd 服务（可选）
echo ""
echo "[6/6] 创建 systemd 服务..."
ssh "${RK3588_USER}@${RK3588_IP}" << EOF
    cat > /etc/systemd/system/rov-ins-driver.service << 'SERVICE'
[Unit]
Description=ROV INS Driver Node
After=network.target

[Service]
Type=simple
User=root
Environment="ROS_DOMAIN_ID=42"
Environment="ROS_LOCALHOST_ONLY=0"
WorkingDirectory=/opt/ros/rov_ros2_ws
ExecStart=/bin/bash -c 'source /opt/ros/setup.bash && source install/setup.bash && ros2 run rov_ins_driver ins_driver_node'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
SERVICE
    
    systemctl daemon-reload
    echo "✓ systemd 服务已创建"
    echo ""
    echo "服务管理命令:"
    echo "  启动: systemctl start rov-ins-driver"
    echo "  停止: systemctl stop rov-ins-driver"
    echo "  开机自启: systemctl enable rov-ins-driver"
EOF

echo ""
echo "=========================================="
echo "部署完成！"
echo "=========================================="
echo ""
echo "在 RK3588 上启动 INS Driver:"
echo "  ssh ${RK3588_USER}@${RK3588_IP}"
echo "  cd ${WORKSPACE_DIR}"
echo "  ./start_ins_driver.sh"
echo ""
echo "或后台启动:"
echo "  ./start_ins_driver_bg.sh"
echo ""
echo "使用 systemd 服务:"
echo "  systemctl start rov-ins-driver"
echo ""
