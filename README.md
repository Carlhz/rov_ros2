# ROV ROS2 集成系统

ROV 水下机器人 ROS2 集成控制系统，覆盖 INS 导航、D30 深温计（深度计）、SF 超声波测深仪（高度计），支持 RK3588（aarch64）和上位机（Ubuntu VM x86_64）多机通信。

## 系统架构

```
┌────────────────────────────────────────────────────────────┐
│                    Ubuntu VM (上位机)                       │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  sensor_monitor.py            (聚合显示所有传感器)     │  │
│  │  ins_monitor_full.py          (INS 姿态/位置监控)      │  │
│  │  ros2 topic echo /rov/xxx     (临时查看单个话题)       │  │
│  └──────────────────────────────────────────────────────┘  │
└──────────────────────┬─────────────────────────────────────┘
                       │ DDS UDP 多播 (ROS_DOMAIN_ID=42)
                       │ 172.16.28.x 网络
┌──────────────────────┼─────────────────────────────────────┐
│                 RK3588 (ROV 主控, 172.16.28.82)            │
│  ┌──────────────────┴───────────────────────────────────┐  │
│  │  ins_driver_full.py     ← UDP 8008 → INS 导航仪      │  │
│  │  depth_sensor_driver.py ← ttyS3   → D30 深温计       │  │
│  │  altimeter_driver.py    ← ttyS5   → SF 高度计        │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### 传感器连接

| 传感器 | 型号 | 接口 | RK3588 串口 | 协议 |
|--------|------|------|-------------|------|
| INS 导航仪 | — | 以太网 (UDP 8008) | eth0:192.168.0.99 | 自定义 202字节帧 |
| D30 深温计 | CERPTS05D30-485 | RS-485 | ttyS3, 19200bps | MODBUS-RTU |
| SF 高度计 | 超声波测深仪 | RS-485 | ttyS5, 9600bps | 自定义 AA/A0 帧 |

### 发布话题一览

**INS (ins_driver_full.py):**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ins/data` | InsData | INS 完整数据（姿态/位置/速度/GNSS） |
| `/ins/imu` | — | IMU 九轴数据 |
| `/ins/gps` | — | GNSS 定位数据 |
| ... | | 共 20+ 话题，100Hz |

**深度计 (depth_sensor_driver.py):**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rov/depth` | Float32 | 水深（米） |
| `/rov/depth_temp` | Float32 | 水温（°C） |
| `/rov/depth_pressure` | Float32 | 压力（MPa） |

**高度计 (altimeter_driver.py):**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rov/altitude` | Float32 | 距底高度-最强目标（米） |
| `/rov/altitude_nearest` | Float32 | 距底高度-最近目标（米） |
| `/rov/altitude_raw` | Float32 | 原始最强距离（米） |

## 目录结构

```
rov_ros2/
├── sensors/                       # 传感器驱动（纯 Python，零编译）
│   ├── depth_sensor_driver.py     # D30 深温计驱动 → /rov/depth*
│   └── altimeter_driver.py        # SF 高度计驱动   → /rov/altitude*
│
├── vm/                            # 上位机监控（Ubuntu VM 端运行）
│   ├── sensor_monitor.py          # 聚合显示深度/高度/水温（彩色终端）
│   └── ins_monitor_full.py        # INS 姿态/位置监控
│
├── rk3588/                        # RK3588 启动脚本
│   ├── start_sensors.sh           # 传感器一键启停管理
│   └── start_ins_driver.sh        # INS 驱动启动
│
├── docs/                          # 协议文档
│   ├── D30_depth_sensor_protocol.md
│   └── SF_altimeter_protocol.md
│
├── deploy/                        # 部署工具
│   └── deploy_sensors.sh          # 从 Windows 部署到 RK3588
│
├── src/                           # INS 相关 ROS2 包（需 colcon build）
│   ├── rov_ins_interfaces/
│   ├── rov_ins_driver/
│   └── rov_topside/
│
└── README.md
```

## 快速开始

### 环境要求

- **RK3588**：预装 ROS2（含 rclpy），Python 3，零额外依赖
- **上位机 VM**：ROS2 Humble/Foxy，Python 3

### 必需环境变量（两端都要设）

```bash
source /opt/ros/humble/setup.bash          # 按实际版本调整
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

建议写入 `~/.bashrc` 永久生效：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
source ~/.bashrc
```

### 1. 部署到 RK3588

从 Windows 主机（需安装 PuTTY，使用 pscp/plink）：

```bash
# 上传传感器驱动
pscp -pw <密码> sensors/depth_sensor_driver.py root@172.16.28.82:/opt/ros/rov_ros2_ws/
pscp -pw <密码> sensors/altimeter_driver.py  root@172.16.28.82:/opt/ros/rov_ros2_ws/
pscp -pw <密码> rk3588/start_sensors.sh      root@172.16.28.82:/opt/ros/rov_ros2_ws/

# 设置可执行权限
plink -pw <密码> root@172.16.28.82 "chmod +x /opt/ros/rov_ros2_ws/start_sensors.sh"
```

### 2. 启动传感器驱动（RK3588 端）

```bash
ssh root@172.16.28.82
cd /opt/ros/rov_ros2_ws/

# 后台启动全部传感器
./start_sensors.sh bg

# 查看运行状态
./start_sensors.sh status

# 停止全部传感器
./start_sensors.sh stop
```

**单独运行调试：**

```bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
python3 depth_sensor_driver.py    # 仅深度计（前台，可看日志）
python3 altimeter_driver.py       # 仅高度计
```

### 3. 上位机查看数据（VM 端）

```bash
# 确认能收到话题
ros2 topic list
# 应看到：/rov/depth  /rov/depth_temp  /rov/depth_pressure
#          /rov/altitude  /rov/altitude_nearest  /rov/altitude_raw

# 方式 A：聚合监控（推荐，一个窗口看全部）
python3 vm/sensor_monitor.py

# 方式 B：逐个查看
ros2 topic echo /rov/altitude
ros2 topic echo /rov/depth

# 方式 C：查看话题频率
ros2 topic hz /rov/altitude
```

## 日常操作速查

### RK3588 端

```bash
ssh root@172.16.28.82
cd /opt/ros/rov_ros2_ws/

./start_sensors.sh bg       # 启动
./start_sensors.sh stop     # 停止
./start_sensors.sh status   # 看谁在跑
```

### VM 端

```bash
# 彩色监控面板
python3 ~/rov_ros2_ws/vm/sensor_monitor.py

# 单话题调试
ros2 topic echo /rov/altitude
ros2 topic hz /rov/altitude

# 节点拓扑
ros2 node list
ros2 node info /altimeter_driver
```

## 故障排除

### VM 上看不到话题

```bash
# 1. 网络连通？
ping 172.16.28.82

# 2. ROS_DOMAIN_ID 一致？
echo $ROS_DOMAIN_ID        # 两端都应该是 42

# 3. VM 网卡模式？
#    NAT 模式 → UDP 多播过不去，必须改为桥接模式

# 4. 防火墙？
sudo ufw disable            # 临时测试
```

### 传感器无数据

```bash
# RK3588 上测试硬件
python3 /opt/ros/rov_ros2_ws/test_altimeter_raw.py   # 高度计诊断
python3 /opt/ros/rov_ros2_ws/test_depth_raw.py       # 深度计诊断

# 查看驱动日志
cat /tmp/alti.log
cat /tmp/depth.log
```

### 重启流程

```bash
# RK3588:
./start_sensors.sh stop && ./start_sensors.sh bg

# VM:
Ctrl+C 退出 sensor_monitor.py，再重新 python3 sensor_monitor.py
```

## 协议文档

- [D30 深温计 MODBUS-RTU 协议](docs/D30_depth_sensor_protocol.md)
- [SF 超声波测深仪协议](docs/SF_altimeter_protocol.md)

## 设计说明

### 为什么不用 colcon build？

传感器驱动使用**纯 Python + 标准消息类型**（`std_msgs/Float32`），直接 `python3 xxx.py` 运行，跳过 colcon 编译：

- 不需要 `package.xml` / `setup.py` / `CMakeLists.txt`
- 不需要自定义 `.msg` 文件
- RK3588 上零额外 Python 依赖（只用 `termios`、`os`、`struct` 等内置模块）
- 修改代码后直接 scp 上传即生效，无需重新 build

当需要自定义消息类型或 `ros2 launch` 管理多节点时，再切 colcon 包即可。

## 许可证

MIT
