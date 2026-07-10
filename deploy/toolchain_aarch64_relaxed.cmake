# CMake 交叉编译工具链文件 (宽松版 - 用于驱动包)
# 与 toolchain_aarch64.cmake 相同，但库和头文件搜索不限于 sysroot

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING TRUE)

set(SDK_SYSROOT /home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux)
set(CMAKE_SYSROOT ${SDK_SYSROOT})

# 交叉编译器
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 程序查找：NEVER（使用 HOST 程序，由 PATH 提供）
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# 库查找：BOTH（先在 sysroot 找，再在 host 找）
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
# 头文件查找：BOTH
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# 添加 install 目录到搜索路径
set(CMAKE_FIND_ROOT_PATH /home/carl/rov_ros2_ws/install ${SDK_SYSROOT})

set(ENV{PKG_CONFIG_SYSROOT_DIR} ${SDK_SYSROOT})
set(ENV{PKG_CONFIG_PATH} "")

# 链接器：优先查找 ARM64 sysroot 库而非 host x86_64 库
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${SDK_SYSROOT}/usr/lib/aarch64-linux-gnu:${SDK_SYSROOT}/opt/ros/lib:${SDK_SYSROOT}/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link,${SDK_SYSROOT}/usr/lib/aarch64-linux-gnu:${SDK_SYSROOT}/opt/ros/lib:${SDK_SYSROOT}/lib/aarch64-linux-gnu")

# 交叉编译：预置 try_run / try_compile 结果（避免在 host 上运行 ARM64 二进制）
set(THREADS_PTHREAD_ARG "0" CACHE STRING "Result from TRY_RUN" FORCE)
set(THREADS_HAVE_PTHREAD_ARG 1 CACHE BOOL "Result from TRY_RUN" FORCE)
set(HAVE_PTHREAD_H 1 CACHE BOOL "Have pthread.h" FORCE)
set(CMAKE_HAVE_LIBC_PTHREAD 1 CACHE BOOL "Have pthread" FORCE)
set(CMAKE_HAVE_THREADS_LIBRARY 1 CACHE BOOL "Have threads" FORCE)
set(CMAKE_USE_PTHREADS_INIT 1 CACHE BOOL "Use pthreads" FORCE)

# rcutils 相关
set(HAVE_LIBDL 1 CACHE BOOL "Have libdl" FORCE)
set(HAVE_DLFCN_H 1 CACHE BOOL "Have dlfcn.h" FORCE)
