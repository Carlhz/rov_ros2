# ROS2 C++ 节点交叉编译部署指南

> 基于 ROV 项目现有交叉编译基础设施整理
> 适用: VM Ubuntu 20.04 (x86_64) → RK3588 (ARM64 aarch64)
> ROS2 Foxy + TRONLONG TL3588 SDK

---

## 1. 环境概览

### 1.1 三台机器的角色

| 机器 | 架构 | 角色 | 关键路径 |
|------|------|------|----------|
| Windows 工作站 | x86_64 | 写代码 | `D:\Carl_WorkStation\rov_ros2\src\` |
| VM Ubuntu | x86_64 | 交叉编译 | `/home/carl/rov_ros2_ws/` + SDK |
| RK3588 | aarch64 | 运行节点 | `/opt/ros/rov_ros2_ws/install/` |

### 1.2 VM 上的关键路径

```
/home/carl/RK3588/rk3588_linux_release/     # TRONLONG SDK 根目录
  ubuntu/
    environment                              # 交叉编译环境脚本 (source 它)
    sysroots/armv8a-linux/                   # ARM64 sysroot (含 ROS2 Foxy + 系统库)
      opt/ros/                               # ARM64 ROS2 Foxy
      usr/lib/aarch64-linux-gnu/             # ARM64 系统库 (libpython3.8, libssl 等)
      usr/include/python3.8/                 # ARM64 Python 头文件

/home/carl/rov_ros2_ws/                      # VM 上的 ROS2 工作空间
  src/                                       # 源码 (从共享文件夹复制)
  build/                                     # 编译中间产物
  install/                                   # 编译输出 (部署到 RK3588)

/mnt/hgfs/CarlWS/rov_ros2/                   # 共享文件夹 → D:\Carl_WorkStation\rov_ros2\
  deploy/toolchain_aarch64.cmake             # 交叉编译工具链 (严格模式)
  deploy/toolchain_aarch64_relaxed.cmake     # 交叉编译工具链 (宽松模式)
  deploy/build_and_deploy_vm.sh              # 一键编译+部署脚本
  src/                                       # ROS2 包源码
```

### 1.3 交叉编译工具链

你已有两个 CMake 工具链文件:

| 文件 | 模式 | 用途 |
|------|------|------|
| `toolchain_aarch64.cmake` | ONLY (严格) | 库/头文件只在 sysroot 找, 隔离 host |
| `toolchain_aarch64_relaxed.cmake` | BOTH (宽松) | 先 sysroot 再 host, 适合有复杂依赖的包 |

两个文件的核心配置:
```cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING TRUE)
set(SDK_SYSROOT /home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux)
set(CMAKE_SYSROOT ${SDK_SYSROOT})
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
```

---

## 2. 第1步: 创建 C++ ROS2 包

### 2.1 包结构

在 Windows 上 `D:\Carl_WorkStation\rov_ros2\src\` 下创建:

```
src/my_cpp_node/
  CMakeLists.txt
  package.xml
  src/
    my_node.cpp
  launch/                    # 可选
    my_node.launch.py
  config/                    # 可选
    my_node.yaml
```

### 2.2 package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_cpp_node</name>
  <version>1.0.0</version>
  <description>My ROS2 C++ node for RK3588</description>
  <maintainer email="rov@example.com">ROV Team</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <!-- 按需添加更多依赖 -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2.3 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_node)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 可执行文件
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
  std_msgs
)

# 安装目标
install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch / config (可选)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 2.4 C++ 节点模板 (src/my_node.cpp)

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // 声明参数 (可在 launch 中覆盖)
        this->declare_parameter("rate_hz", 10);
        double rate = this->get_parameter("rate_hz").as_double();

        // 发布者
        pub_ = this->create_publisher<std_msgs::msg::String>("/my_topic", 10);

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
            std::bind(&MyNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "MyNode started, rate=%.1f Hz", rate);
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "hello from RK3588";
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
```

### 2.5 参考实例: 声纳驱动

你现有的 `src/rov_sonar_driver/` 就是一个完整的 C++ 节点包, 可直接参考:
- `CMakeLists.txt` — 标准的 ament_cmake 结构
- `package.xml` — 依赖 rclcpp + sensor_msgs + 自定义接口
- `src/sonar_omni_driver.cpp` — 完整的 ROS2 节点 (参数声明、UDP 通信、PointCloud2 发布)

---

## 3. 第2步: 在 VM 上交叉编译

### 3.1 手动编译 (推荐理解流程)

SSH 或控制台进入 VM:

```bash
# 1. source 交叉编译 SDK 环境
source /home/carl/RK3588/rk3588_linux_release/ubuntu/environment

# 2. source VM 本地的 ROS2 Foxy (提供 colcon 和构建工具)
source /opt/ros/foxy/setup.bash

# 3. 准备工作空间
cd /home/carl/rov_ros2_ws
mkdir -p src
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/my_cpp_node src/

# 4. 交叉编译
colcon build \
  --packages-select my_cpp_node \
  --symlink-install \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=/mnt/hgfs/CarlWS/rov_ros2/deploy/toolchain_aarch64_relaxed.cmake \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_LIBRARY=/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux/usr/lib/aarch64-linux-gnu/libpython3.8.so \
    -DPYTHON_INCLUDE_DIR=/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux/usr/include/python3.8 \
    -DPYTHON_SOABI=cpython-38-aarch64-linux-gnu \
    -DOPENSSL_ROOT_DIR=/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux/usr \
    -DOPENSSL_CRYPTO_LIBRARY=/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux/usr/lib/aarch64-linux-gnu/libcrypto.so \
    -DOPENSSL_SSL_LIBRARY=/home/carl/RK3588/rk3588_linux_release/ubuntu/sysroots/armv8a-linux/usr/lib/aarch64-linux-gnu/libssl.so \
    -DTHREADS_PTHREAD_ARG=0 \
    -DCMAKE_HAVE_LIBC_PTHREAD=1 \
    -DCMAKE_HAVE_THREADS_LIBRARY=1

# 5. 验证产物架构
file install/my_cpp_node/lib/my_cpp_node/my_node
# 应输出: ELF 64-bit LSB executable, ARM aarch64
```

### 3.2 使用现有一键脚本

你已有 `deploy/build_and_deploy_vm.sh`, 修改其中的包名即可:

```bash
# 在 VM 上
cd /home/carl/rov_ros2_ws
bash /mnt/hgfs/CarlWS/rov_ros2/deploy/build_and_deploy_vm.sh
```

脚本自动完成: 环境检查 → source SDK → 复制源码 → colcon build → 验证 → scp 部署

### 3.3 关键 CMake 参数说明

| 参数 | 说明 |
|------|------|
| `CMAKE_TOOLCHAIN_FILE` | 指定交叉编译工具链 (aarch64-gnu-gcc/g++) |
| `PYTHON_LIBRARY` | ARM64 版 libpython3.8.so (rosidl 代码生成需要) |
| `PYTHON_INCLUDE_DIR` | ARM64 Python 头文件路径 |
| `PYTHON_SOABI` | `cpython-38-aarch64-linux-gnu` (Python 扩展模块后缀) |
| `OPENSSL_*` | ARM64 OpenSSL 库 (FastDDS 依赖) |
| `THREADS_PTHREAD_ARG=0` | 预置 try_run 结果 (交叉编译不能在 host 运行 ARM64 二进制) |

### 3.4 如果包有自定义 msg/srv 接口

需要先编译接口包, 再编译驱动包 (colcon 自动处理依赖顺序):

```bash
colcon build \
  --packages-select my_interface my_cpp_node \
  --symlink-install \
  --cmake-args \
    -DCMAKE_TOOLCHAIN_FILE=.../toolchain_aarch64_relaxed.cmake \
    ... (其余参数同上)
```

接口包的 CMakeLists.txt 参考 `src/rov_sonar_interface/CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SonarConfig.srv"
)
ament_export_dependencies(rosidl_default_runtime)
```

---

## 4. 第3步: 部署到 RK3588

### 4.1 从 VM 直接 scp

```bash
# 在 VM 上
scp -r /home/carl/rov_ros2_ws/install/* root@172.16.28.82:/opt/ros/rov_ros2_ws/install/
```

### 4.2 从 Windows scp (推荐, 不依赖 VM)

```bash
# 在 Windows Git Bash 中
# 先从 VM 下载 install 包
scp -r carl@<VM_IP>:/home/carl/rov_ros2_ws/install/* /tmp/rk3588_install/

# 再上传到 RK3588
scp -r /tmp/rk3588_install/* root@172.16.28.82:/opt/ros/rov_ros2_ws/install/
```

### 4.3 install 目录结构

编译后 `install/` 的结构:
```
install/
  my_cpp_node/
    lib/my_cpp_node/my_node          # ARM64 ELF 可执行文件
    share/my_cpp_node/
      launch/my_node.launch.py       # launch 文件
      config/my_node.yaml            # 配置文件
      package.xml                    # 包描述
      cmake/                         # CMake 配置 (供其他包依赖)
  my_interface/                      # 如果有接口包
    lib/libmy_interface__*.so        # ARM64 共享库
    lib/python3.8/site-packages/     # Python 绑定
```

---

## 5. 第4步: 在 RK3588 上运行

### 5.1 手动运行

```bash
# SSH 到 RK3588
ssh root@172.16.28.82

# source ROS2 环境
source /opt/ros/setup.bash

# source 你的工作空间
source /opt/ros/rov_ros2_ws/install/local_setup.sh

# 设置 DDS 域
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 运行节点
ros2 run my_cpp_node my_node

# 或用 launch (如果安装了 launch 文件)
ros2 launch my_cpp_node my_node.launch.py rate_hz:=20
```

### 5.2 集成到 start_all.sh

在 RK3588 的 `/opt/ros/rov_ros2_ws/start_all.sh` 中添加启动逻辑:

```bash
# 启动 C++ 节点
echo "[>>] 启动 my_cpp_node ..."
source /opt/ros/rov_ros2_ws/install/local_setup.sh
ros2 run my_cpp_node my_node > /tmp/my_cpp_node.log 2>&1 &
echo "     PID=$!"
echo $! > /tmp/my_cpp_node.pid
```

### 5.3 验证节点运行

```bash
# 检查节点
ros2 node list

# 检查话题
ros2 topic list
ros2 topic echo /my_topic

# 检查可执行文件架构
file /opt/ros/rov_ros2_ws/install/my_cpp_node/lib/my_cpp_node/my_node
# 应输出: ELF 64-bit LSB executable, ARM aarch64, version 1 (GNU/Linux)
```

---

## 6. 完整工作流速查

```
Windows (写代码)                    VM (交叉编译)                      RK3588 (运行)
─────────────────                ──────────────────                ──────────────────
src/my_cpp_node/                 source SDK environment            source /opt/ros/setup.bash
  src/my_node.cpp                source /opt/ros/foxy/setup.bash   source /opt/ros/rov_ros2_ws/
  CMakeLists.txt                 cd ~/rov_ros2_ws                        install/local_setup.sh
  package.xml                    cp -r /mnt/hgfs/.../src/my_pkg   export ROS_DOMAIN_ID=42
        │                        colcon build --packages-select          ros2 run my_cpp_node my_node
        │                          my_cpp_node --cmake-args
        │                          -DCMAKE_TOOLCHAIN_FILE=...
        ▼                                    │
  /mnt/hgfs/CarlWS/rov_ros2/                 ▼
  (共享文件夹自动同步)               install/my_cpp_node/lib/
                                              my_cpp_node/my_node
                                              (ARM64 ELF)
                                                       │
                                                       ▼
                                            scp -r install/* root@172.16.28.82:
                                              /opt/ros/rov_ros2_ws/install/
```

---

## 7. 常见问题

### Q: 编译报 "cannot find -lpython3.8"

A: 没有指定 ARM64 Python 库路径。确保 CMake 参数包含:
```
-DPYTHON_LIBRARY=<SDK_SYSROOT>/usr/lib/aarch64-linux-gnu/libpython3.8.so
```

### Q: 编译报 "try_run() failed" 或卡在 threads 检测

A: 交叉编译不能在 host 上运行 ARM64 二进制。使用宽松工具链 (`toolchain_aarch64_relaxed.cmake`) 并添加:
```
-DTHREADS_PTHREAD_ARG=0 -DCMAKE_HAVE_LIBC_PTHREAD=1 -DCMAKE_HAVE_THREADS_LIBRARY=1
```

### Q: RK3588 上运行报 "error while loading shared libraries"

A: 缺少依赖库。检查:
```bash
# 在 RK3588 上
ldd /opt/ros/rov_ros2_ws/install/my_cpp_node/lib/my_cpp_node/my_node
```
确保所有 .so 都能找到。RK3588 上的 `/opt/ros/` 应已安装完整 ROS2 Foxy。

### Q: RK3588 上 source local_setup.sh 报错

A: 先 source 基础 ROS2 环境:
```bash
source /opt/ros/setup.bash        # 先 source 系统 ROS2
source /opt/ros/rov_ros2_ws/install/local_setup.sh  # 再 source 你的包
```

### Q: 自定义 msg/srv 在 RK3588 上找不到

A: 接口包和驱动包必须一起部署。确保 `install/` 包含两个包的完整产物。

### Q: 编译产物是 x86_64 而不是 aarch64

A: 工具链文件没有生效。检查:
```bash
# 在 VM 上编译后验证
file ~/rov_ros2_ws/install/my_cpp_node/lib/my_cpp_node/my_node
# 必须是 "ARM aarch64", 不能是 "x86-64"
```
如果还是 x86_64, 说明 colcon 没有使用工具链文件, 检查 `--cmake-args` 参数是否正确传递。
