# ROV ROS2 Foxy 集成系统

全套 ROS2 Foxy 集成的 ROV INS 驱动系统，支持 RK3588 和上位机（Ubuntu VM）多机通信。

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      Ubuntu VM (上位机)                          │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  rov_topside 包                                          │   │
│  │  ├── ins_monitor_node  (订阅/ins/data, 显示数据)          │   │
│  │  └── ins_logger_node   (记录数据到CSV)                    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                          ↑ DDS (ROS_DOMAIN_ID=42)              │
│                          ↓                                     │
└──────────────────────────┬──────────────────────────────────────┘
                           │ 172.16.28.x 网络
┌──────────────────────────┼──────────────────────────────────────┐
│                     RK3588 (ROV 主控)                           │
│  ┌───────────────────────┴─────────────────────────────────┐   │
│  │  rov_ins_driver 包                                       │   │
│  │  └── ins_driver_node  (接收INS UDP, 发布/ins/data)        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                          ↑ UDP 8008                            │
│                          ↓                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  INS 设备 (192.168.0.7)                                  │   │
│  │  └── 输出 0x50 数据帧 (78字节)                            │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
rov_ros2/
├── src/
│   ├── rov_ins_interfaces/     # 自定义消息定义
│   │   ├── msg/
│   │   │   ├── InsData.msg     # INS 数据消息
│   │   │   └── InsCommand.msg  # INS 控制命令
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── rov_ins_driver/         # RK3588 端驱动包
│   │   ├── rov_ins_driver/
│   │   │   ├── __init__.py
│   │   │   └── ins_driver_node.py
│   │   ├── launch/
│   │   │   └── ins_driver.launch.py
│   │   ├── config/
│   │   │   └── ins_driver.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── rov_topside/            # 上位机监控包
│       ├── rov_topside/
│       │   ├── __init__.py
│       │   ├── ins_monitor_node.py
│       │   └── ins_logger_node.py
│       ├── launch/
│       │   ├── topside.launch.py
│       │   └── monitor_only.launch.py
│       ├── package.xml
│       └── setup.py
│
├── deploy/                     # 部署脚本
│   ├── deploy_to_rk3588.sh     # 部署到 RK3588
│   ├── setup_dds.sh            # DDS 多机通信配置
│   └── quick_start.sh          # 上位机快速启动
│
└── README.md
```

## 快速开始

### 1. 上位机（Ubuntu VM）准备

```bash
# 进入工作空间
cd ~/rov_ros2

# 构建
colcon build --symlink-install

# 配置 DDS（只需运行一次）
./deploy/setup_dds.sh 172.16.28.82 42

# 重新加载环境
source ~/.bashrc
```

### 2. 部署到 RK3588

```bash
# 从 Windows 复制到 Ubuntu VM 后，运行部署脚本
./deploy/deploy_to_rk3588.sh 172.16.28.82
```

### 3. 启动系统

**RK3588 端：**
```bash
ssh root@172.16.28.82
cd /opt/ros/rov_ros2_ws
./start_ins_driver.sh
```

**上位机端：**
```bash
# 方式1: 快速启动
./deploy/quick_start.sh

# 方式2: 手动启动
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch rov_topside topside.launch.py
```

## 使用方法

### 查看数据

```bash
# 查看话题列表
ros2 topic list

# 查看 INS 数据
ros2 topic echo /ins/data

# 查看数据频率
ros2 topic hz /ins/data
```

### 发送控制命令

```bash
# 启动 INS 数据输出
ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: "start"}' --once

# 停止 INS 数据输出
ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: "stop"}' --once

# 设置初始纬度
ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: "set_lat", latitude: 31.23}' --once

# 设置初始经度
ros2 topic pub /ins/command rov_ins_interfaces/msg/InsCommand '{command: "set_lon", longitude: 121.47}' --once
```

### 监控节点交互命令

在 `ins_monitor_node` 运行后，可以在控制台输入：

- `start` - 启动 INS 数据输出
- `stop` - 停止 INS 数据输出
- `lat XX.XX` - 设置纬度（例如: `lat 31.23`）
- `lon XXX.XXX` - 设置经度（例如: `lon 121.47`）
- `status` - 显示接收统计
- `help` - 显示帮助
- `quit` - 退出程序

## 消息定义

### InsData.msg

```yaml
std_msgs/Header header

# 位置
float64 latitude      # 纬度（度）
float64 longitude     # 经度（度）
float32 altitude      # 高度（米）

# 姿态
float32 roll          # 横滚角（度）
float32 pitch         # 俯仰角（度）
float32 yaw           # 航向角（度）

# 速度
float32 north_vel     # 北向速度（m/s）
float32 east_vel      # 东向速度（m/s）
float32 down_vel      # 向下速度（m/s）

# 角速度
float32 gyro_x        # X轴角速度（deg/s）
float32 gyro_y        # Y轴角速度（deg/s）
float32 gyro_z        # Z轴角速度（deg/s）

# 加速度
float32 acc_x         # X轴加速度（m/s²）
float32 acc_y         # Y轴加速度（m/s²）
float32 acc_z         # Z轴加速度（m/s²）

# 状态
uint8   work_status
uint8   dvl_calib_status
uint8   gnss_pos_status
uint8   combination_status

# 状态描述（人类可读）
string  work_status_desc
string  dvl_calib_desc
string  gnss_pos_desc
string  combination_desc

# 原始数据
uint8[] raw_frame
bool    valid
```

## 网络配置

### RK3588 网络

```
eth0:
  - 172.16.28.82/24    (主 IP，与上位机通信)
  - 192.168.0.99/24    (副 IP，与 INS 通信)
```

### INS 设备

```
IP: 192.168.0.7
数据端口: 8008 (UDP)
命令端口: 8007 (UDP)
```

## 故障排除

### 1. 无法发现话题

```bash
# 检查 ROS_DOMAIN_ID 是否一致
echo $ROS_DOMAIN_ID

# 检查网络连通
ping 172.16.28.82

# 检查防火墙
sudo ufw status
```

### 2. 没有 INS 数据

```bash
# 在 RK3588 上检查 UDP 接收
nc -u -l 8008

# 检查 INS 是否发送数据
# 确保已发送 start 命令
```

### 3. 构建失败

```bash
# 清理并重新构建
rm -rf build/ install/ log/
colcon build --symlink-install
```

## 依赖

- ROS2 Foxy Fitzroy
- Python 3.8+
- colcon
- rosdep

## 许可证

MIT
