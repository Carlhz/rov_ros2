# CMake 交叉编译工具链文件
# 目标: ARM64 (RK3588 aarch64)
# 主机: x86_64 (VM Ubuntu 20.04)
# 基于 TRONLONG TL3588 SDK

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING TRUE)

# SDK sysroot 路径
set(SDK_SYSROOT /home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux)
set(CMAKE_SYSROOT ${SDK_SYSROOT})
set(CMAKE_STAGING_PREFIX /home/carl/rov_ros2_ws/install)

# 交叉编译器
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 程序查找：NEVER（使用 HOST 程序）
# 库查找：ONLY（仅使用 sysroot）
# 头文件查找：ONLY（仅使用 sysroot）
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 防止 cmake 在 HOST 系统路径搜索库
set(CMAKE_FIND_ROOT_PATH ${SDK_SYSROOT})
set(ENV{PKG_CONFIG_SYSROOT_DIR} ${SDK_SYSROOT})
set(ENV{PKG_CONFIG_PATH} "")

# 链接器标志：不在 HOST 系统搜索
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${SDK_SYSROOT}/usr/lib/aarch64-linux-gnu:${SDK_SYSROOT}/opt/ros/lib:${SDK_SYSROOT}/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link,${SDK_SYSROOT}/usr/lib/aarch64-linux-gnu:${SDK_SYSROOT}/opt/ros/lib:${SDK_SYSROOT}/lib/aarch64-linux-gnu")
