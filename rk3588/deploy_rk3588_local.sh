#!/bin/bash
# RK3588 本地编译部署脚本
# 在 RK3588 上直接执行

set -e

echo "========================================"
echo "RK3588 INS Driver 本地编译部署"
echo "========================================"

# 检查是否在 RK3588 上
if [[ $(uname -m) != "aarch64" ]]; then
    echo "警告：当前不是 aarch64 架构，可能不是在 RK3588 上运行"
    read -p "是否继续? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 1. 检查系统空间
echo ""
echo "[1/6] 检查系统空间..."
avail_space=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
if [ "$avail_space" -lt 2 ]; then
    echo "错误：根分区空间不足 ${avail_space}GB，需要至少 2GB"
    exit 1
fi
echo "可用空间: ${avail_space}GB ✓"

# 2. 检查并安装基础依赖
echo ""
echo "[2/6] 安装 ROS2 基础依赖..."
apt-get update
apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    cmake \
    git \
    wget \
    curl \
    || echo "部分包安装失败，尝试继续..."

# 3. 安装 Python 依赖
echo ""
echo "[3/6] 安装 Python 依赖..."
pip3 install --user empy==3.3.4

# 4. 检查 ROS2 是否已安装
echo ""
echo "[4/6] 检查 ROS2 安装..."
if [ -d "/opt/ros/humble" ]; then
    ROS_DISTRO="humble"
elif [ -d "/opt/ros/foxy" ]; then
    ROS_DISTRO="foxy"
elif [ -d "/opt/ros/galactic" ]; then
    ROS_DISTRO="galactic"
else
    echo "ROS2 未安装，尝试安装..."
    
    # 添加 ROS2 源
    apt-get install -y curl gnupg lsb-release
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    apt-get update
    
    # 尝试安装 ROS2 Humble（ARM64）
    apt-get install -y ros-humble-ros-base || {
        echo "ROS2 Humble 安装失败，尝试 Foxy..."
        apt-get install -y ros-foxy-ros-base || {
            echo "错误：无法安装 ROS2"
            exit 1
        }
        ROS_DISTRO="foxy"
    }
    ROS_DISTRO="humble"
fi

echo "ROS2 版本: $ROS_DISTRO ✓"

# 5. 创建工作空间并复制源码
echo ""
echo "[5/6] 创建工作空间..."
WORKSPACE="$HOME/rov_ros2_ws"
mkdir -p "$WORKSPACE/src"

# 从共享目录复制（如果有挂载）
if [ -d "/mnt/hgfs/CarlWS/rov_ros2/src" ]; then
    echo "从共享目录复制源码..."
    cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_interface "$WORKSPACE/src/"
    cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_driver "$WORKSPACE/src/"
else
    echo "警告：共享目录未挂载，需要手动复制源码到 $WORKSPACE/src/"
    echo "请确保以下目录存在："
    echo "  - $WORKSPACE/src/rov_ins_interface"
    echo "  - $WORKSPACE/src/rov_ins_driver"
    read -p "按回车继续..."
fi

# 6. 编译
echo ""
echo "[6/6] 编译 ROS2 包..."
cd "$WORKSPACE"

# Source ROS2 环境
source "/opt/ros/$ROS_DISTRO/setup.bash"

# 编译
colcon build --packages-select rov_ins_interface rov_ins_driver

echo ""
echo "========================================"
echo "编译完成！"
echo "========================================"
echo ""
echo "启动命令："
echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
echo "  source $WORKSPACE/install/setup.bash"
echo "  export ROS_DOMAIN_ID=42"
echo "  ros2 run rov_ins_driver ins_driver"
echo ""
