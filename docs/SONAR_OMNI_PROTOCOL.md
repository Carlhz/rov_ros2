# Scanfish-II 全向扫描声纳协议文档

## 概述

- **设备型号**: Scanfish-II 全向扫描声纳 (Omnidirectional Scanning Sonar)
- **通信方式**: UDP
- **默认 IP**: 192.168.0.7
- **默认端口**: 23
- **帧格式**: FE/FD 定界帧
- **ROS2 驱动**: `rov_sonar_driver` (C++), 运行于 RK3588
- **ROS2 监控**: `rov_sonar_monitor` (Python), 运行于 VM 上位机

## 网络拓扑

```
[全向声纳 192.168.0.7] ─── 交换机 ─── [RK3588 192.168.0.99/172.16.28.82]
                                            │
                                            │ ROS2 (DOMAIN_ID=42)
                                            │
                                     [VM上位机 172.16.28.x]
```

## 命令帧格式 (上位机 → 声纳)

长度固定 28 字节，以 `0xFE` 开头、`0xFD` 结尾。

| 字节 | 字段 | 说明 |
|------|------|------|
| 0 | 0xFE | 帧头 |
| 1 | 0x00 | Broadcast |
| 2 | WorkStatus | 工作状态 (0x23=运行, 0x2B=停止) |
| 3 | Range | 量程 1-20 米 |
| 4 | StartGain | 增益 0-40 dB |
| 5 | LOGF | 声扩散系数: 15/20/30/40 |
| 6 | Absorption | 声吸收系数 dB/100m |
| 7 | StepSize | 步距 (仅支持 1) |
| 8-9 | SoundSpeed | 声速 1300-1700 m/s (大端) |
| 10-11 | TrainAngle | 中心方位 0-3599, 0.1°单位 (大端) |
| 12-13 | SectorWidth | 扇扫角度 0-3600, 0.1°单位 (大端), 3600=全向PPI |
| 14-15 | DataLen | 数据长度, 最大1000 (大端) |
| 16 | PulseType | 0=自动, 1=CW短, 2=CW长 |
| 17 | Res1 | 保留 (填0) |
| 18-19 | Gate | 剖面门限 100-16000 (大端) |
| 20-21 | MinRange | 最小距离样点数 (大端) |
| 22-23 | Delay | 发射延时 us (大端) |
| 24 | Res2 | 保留 (填0) |
| 25 | Res3 | 保留 (填0) |
| 26 | Freq | 0=自动, 68=低频, 221=高频 |
| 27 | 0xFD | 帧尾 |

**示例命令 (4m量程, PPI全向扫描)**:
```
FE 00 23 04 14 28 0A 01  05 CD 00 00 0E 10 03 E8
00 00 00 C8 00 96 00 00  00 00 00 FD
```

### WorkStatus (字节2) 位定义

| Bit | 含义 |
|-----|------|
| 7 | 0=工作命令, 1=状态查询 |
| 6 | 0=Master, 1=Slave |
| 5 | 0=正常扇扫, 1=单向扇扫 |
| 4 | 0=正常, 1=转至0方位 (Slave) |
| 3 | 0=正常, 1=暂停(Hold) |
| 2 | 0=正常, 1=逆扫 |
| 1 | Slave模式下: 1=发送数据 |
| 0 | Slave模式下: 1=发射脉冲 |

常用值:
- `0x23` (0010 0011): Master, 正常扇扫, 发送数据, 发射脉冲 — **正常运行**
- `0x2B` (0010 1011): Master, 正常扇扫, 暂停, ... — **停止扫描**

## 应答帧格式 (声纳 → 上位机)

### 纯应答帧 (8字节, 无数据)

| 字节 | 内容 |
|------|------|
| 0 | 0xFE |
| 1 | ID |
| 2 | 状态 |
| 3-4 | 数据长度=0 (7-bit编码) |
| 5-6 | 角度 (7-bit编码) |
| 7 | 0xFD |

### 数据应答帧 (8+N+2 字节)

| 字节 | 内容 |
|------|------|
| 0 | 0xFE |
| 1 | ID |
| 2 | 状态 |
| 3-4 | 数据长度 N (7-bit编码) |
| 5-6 | 角度 (7-bit编码) |
| 7 | 0xFD |
| 8 ~ 8+N-1 | 声学回波数据 (N字节, 每字节=强度值0-127) |
| 8+N | 剖面距离低字节 (7-bit) |
| 8+N+1 | 剖面距离高字节 (7-bit) |

### 7-bit 编码

```
实际值 = ((高字节 & 0x7F) << 7) | (低字节 & 0x7F)
```

### 角度计算

```
角度(°) = 7bit编码值 / 800.0 × 360.0
弧度 = 角度 × π / 180.0
```

### 距离计算

```
采样间隔 = 2.5 μs
每个采样点距离 = 采样间隔 × 声速 / 2
剖面边界距离 = 剖面7bit值 × 2.5e-6 × 声速 / 2
```

## ROS2 话题

### 发布话题 (RK3588 → VM上位机)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/sonar/omni/original` | `sensor_msgs/PointCloud2` | 原始回波点云 (x,y,z,intensity) |
| `/sonar/omni/rigidity` | `sensor_msgs/PointCloud2` | 差分刚性检测点云 |
| `/sonar/omni/boundary` | `sensor_msgs/PointCloud2` | 剖面边界点 (单点) |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/sonar/omni/config` | `rov_sonar_interface/SonarConfig` | 声纳参数配置 |

### 坐标系

- 采用东北天(XYZ)坐标系
- X轴正方向 = 声纳0°方位 (正前方)
- Y轴正方向 = 右侧
- 点云坐标: `x = range × cos(θ)`, `y = -range × sin(θ)`, `z = 0`

## 运行方式

### RK3588 (驱动节点)

```bash
# 方式1: 直接运行
source /opt/ros/humble/setup.bash
source /opt/ros/rov_ros2_ws/install/setup.bash
ros2 run rov_sonar_driver sonar_omni_driver

# 方式2: Launch 文件
ros2 launch rov_sonar_driver sonar_omni.launch.py

# 自定义参数
ros2 run rov_sonar_driver sonar_omni_driver \
    --ros-args -p server_ip:=192.168.0.7 -p range:=10 -p sector_width:=3600
```

### VM上位机 (监控节点)

```bash
source ~/rov_ros2_ws/install/setup.bash
ros2 run rov_sonar_monitor sonar_monitor_node

# 键盘控制:
#   s = 开始扫描    t = 停止
#   1/2/3/4 = 量程 4/10/20/60m
#   +/- = 增益调节
#   [/] = 扇扫角度调节
#   o = 全向360°    d = 定向180°
#   q = 退出
```

### 构建

```bash
cd ~/rov_ros2_ws
colcon build --packages-select rov_sonar_interface rov_sonar_driver rov_sonar_monitor
source install/setup.bash
```

## 部署

从 Windows 工作站部署到 RK3588:

```bash
cd /d/Carl_WorkStation/rov_ros2/deploy
bash deploy_sonar_omni.sh
```
