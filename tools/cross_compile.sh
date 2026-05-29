#!/bin/bash
# VM 交叉编译脚本
# 在 VM 上执行，编译 ARM64 版本的 ROS2 包

set -e

echo "========================================"
echo "交叉编译 ROS2 包 (ARM64)"
echo "========================================"

# 1. 配置交叉编译工具链
echo "[1/5] 配置交叉编译工具链..."
if [ -f ~/RK3588/rk3588_linux_release/ubuntu/environment ]; then
    source ~/RK3588/rk3588_linux_release/ubuntu/environment
    echo "工具链已加载"
else
    echo "错误：找不到工具链环境脚本"
    echo "请确认 ~/RK3588/rk3588_linux_release/ubuntu/environment 存在"
    exit 1
fi

# 2. 创建工作空间
echo "[2/5] 创建工作空间..."
WORKSPACE="$HOME/rov_ros2_cross_ws"
mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE"

# 3. 复制源码
echo "[3/5] 复制源码..."
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_interface src/
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_driver src/

# 4. 配置交叉编译参数
echo "[4/5] 配置交叉编译参数..."
# 创建 colcon 配置文件
mkdir -p .colcon

# 设置交叉编译环境变量
export CMAKE_SYSTEM_NAME=Linux
export CMAKE_SYSTEM_PROCESSOR=aarch64
export CMAKE_C_COMPILER=aarch64-linux-gnu-gcc
export CMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
export PYTHON_EXECUTABLE=/usr/bin/python3

# 5. 配置交叉编译环境
echo "[5/5] 配置交叉编译环境并编译..."

# 设置正确的交叉编译器路径（x86_64 宿主机版本），保留原有 PATH
export PATH="$PATH:/home/carl/RK3588/rk3588_linux_release/prebuilts/gcc/linux-x86/aarch64/gcc-arm-10.3-2021.07-x86_64-aarch64-none-linux-gnu/bin"

# 验证编译器
if ! command -v aarch64-none-linux-gnu-gcc &> /dev/null; then
    echo "错误：找不到交叉编译器"
    exit 1
fi

echo "交叉编译器: $(which aarch64-none-linux-gnu-gcc)"

# 设置交叉编译环境变量
SYSROOT="/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux"
export CMAKE_SYSROOT="$SYSROOT"
export CMAKE_FIND_ROOT_PATH="$SYSROOT"

# 设置交叉编译 Python 路径
PYTHON_INCLUDE="$SYSROOT/usr/include/python3.8"
PYTHON_LIB="$SYSROOT/usr/lib"

echo "Python include: $PYTHON_INCLUDE"
echo "Python lib: $PYTHON_LIB"
echo "Sysroot: $SYSROOT"

# 编译
colcon build --packages-select rov_ins_interface rov_ins_driver \
    --cmake-args \
    -DCMAKE_SYSTEM_NAME=Linux \
    -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER=aarch64-none-linux-gnu-gcc \
    -DCMAKE_CXX_COMPILER=aarch64-none-linux-gnu-g++ \
    -DCMAKE_SYSROOT="$SYSROOT" \
    -DCMAKE_FIND_ROOT_PATH="$SYSROOT" \
    -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER \
    -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY \
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR="$PYTHON_INCLUDE" \
    -DPYTHON_LIBRARY="$PYTHON_LIB/libpython3.8.so"

echo ""
echo "========================================"
echo "编译完成！"
echo "========================================"
echo ""
echo "编译输出目录: $WORKSPACE/install"
echo ""
echo "下一步: 将 install 目录传到 RK3588"
echo "  scp -r $WORKSPACE/install root@172.16.28.82:/opt/ros/rov_ros2_ws/"
