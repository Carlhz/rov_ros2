# ROV 水下机器人操作手册

> 版本 1.0 | 2026-06-29 | 基于 ROS2 Foxy

---

## 目录

1. [系统架构](#1-系统架构)
2. [硬件清单](#2-硬件清单)
3. [网络拓扑](#3-网络拓扑)
4. [快速启动](#4-快速启动)
5. [手柄操控](#5-手柄操控)
6. [传感器监控](#6-传感器监控)
7. [INS 惯导对准](#7-ins-惯导对准)
8. [声纳系统](#8-声纳系统)
9. [部署与升级](#9-部署与升级)
10. [故障排查](#10-故障排查)
11. [参考附录](#11-参考附录)

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROV 控制系统                              │
│                                                                   │
│  ┌──────────┐     ROS2 DDS (Domain 42)     ┌──────────────────┐  │
│  │ Ubuntu VM │◄──────────────────────────► │   RK3588 (工控机)  │  │
│  │ (上位机)   │    172.16.28.x / 30.x       │   172.16.28.82    │  │
│  │           │                              │   192.168.0.99    │  │
│  └─────┬─────┘                              └────────┬─────────┘  │
│        │                                             │            │
│  Logitech                                     ┌──────┼──────┐    │
│   F710                                        │  交换机      │    │
│   手柄                                        └──────┬──────┘    │
│                                              ╱      │      ╲     │
│                                      ┌──────┐ ┌──────┐ ┌──────┐  │
│                                      │ INS  │ │声纳  │ │PC    │  │
│                                      │0.0.7 │ │0.0.5 │ │0.0.100│ │
│                                      └──────┘ └──────┘ └──────┘  │
│                                                                   │
│  RK3588 串口:                                                     │
│    ttyS3 → SF 高度计 (RS485, 9600/8N1)                            │
│    ttyS5 → D30 深温计 (RS485, 19200/8N1)                          │
│    can0  → 螺旋桨电机 (MCP2515 SPI-CAN, 500kbps)                   │
└─────────────────────────────────────────────────────────────────┘
```

### 数据流

```
[手柄] → joy_node → /joy → joy_controller.py (VM)
                                ↓
                          /rov/cmd_vel (Twist)
                                ↓  ROS2 DDS
                          motor_controller.py (RK3588)
                                ↓
                          /usr/can_motor_v1.0
                                ↓ CAN bus
                           螺旋桨电机

[INS] → UDP:8008 → ins_driver_auto.py → /ins/* 话题
[D30] → ttyS5 → depth_sensor_driver.py → /rov/depth*
[SF]  → ttyS3 → altimeter_driver.py → /rov/altitude*
```

---

## 2. 硬件清单

| 设备 | 型号/说明 | 接口 | 地址 | 关键参数 |
|------|----------|------|------|---------|
| 工控机 | RK3588 (TRONLONG TL3588) | 以太网 | 172.16.28.82 / 192.168.0.99 | ROS2 Foxy, Ubuntu |
| 上位机 | VMware Ubuntu VM | 桥接网络 | DHCP (172.16.30.x) | ROS2 Foxy |
| 惯导 | INS400 | UDP | 192.168.0.7:8008 | 202字节帧, 100Hz |
| 全向声纳 | Scanfish-II | UDP | 192.168.0.5:23 | FE/FD协议, 360° PPI |
| 深度计 | CERPTS05D30-485 | RS485(ttyS5) | 设备地址1 | MODBUS-RTU, 19200/8N1, 5MPa量程 |
| 高度计 | SF超声波测深仪 | RS485(ttyS3) | 机号1 | AA/A0协议, 9600/8N1, 盲区<20cm |
| 手柄 | Logitech F710 | USB | /dev/input/js0 | XInput模式 |
| 螺旋桨 | 8通道直流无刷 | CAN | can0 @ 500kbps | 1200~1800 rpm |
| 交换机 | - | - | 192.168.0.x网段 | 连接RK3588/INS/声纳 |

---

## 3. 网络拓扑

```
[VM上位机] ───── 172.16.28.x / 30.x ───── [RK3588 eth0]
 桥接模式                                172.16.28.82/22
                                         192.168.0.99/24
                                              │
                                          交换机
                                         ╱  │  ╲
                          [全向声纳]  [INS惯导]  [HP电脑]
                         192.168.0.5  192.168.0.7  192.168.0.100
```

| 设备 | IP 地址 | 说明 |
|------|---------|------|
| RK3588 eth0（主） | 172.16.28.82/22 | 与管理网络通信 |
| RK3588 eth0（副） | 192.168.0.99/24 | 与传感器通信（netplan持久化） |
| INS | 192.168.0.7 | UDP 8008接收数据, 8007发送指令 |
| 全向声纳 | 192.168.0.5 | UDP 23, 内置有人物联网StP62-K7 |
| Ubuntu VM | DHCP (172.16.30.x) | 桥接, 与RK3588同子网 |

> **ROS2 通信**：统一使用 `ROS_DOMAIN_ID=42`，DDS组播发现

---

## 4. 快速启动

### 4.1 上电顺序

1. **交换机** → 通电，等待就绪
2. **传感器**（INS / 声纳 / 深度计 / 高度计）→ 通电
3. **RK3588** → 开机（CAN 和 IP 自动配置）
4. **VM Ubuntu** → 开机（或从已开虚拟机的Windows端操作）
5. **手柄** → 插入USB（可在VM开机后随时连接）

### 4.2 RK3588 端启动

SSH 登录 RK3588：

```bash
ssh root@172.16.28.82
# 密码: 159357
```

一键启动所有驱动（CAN + INS + 深度计 + 高度计 + 电机控制器）：

```bash
cd /opt/ros/rov_ros2_ws
./start_all.sh bg
```

**子命令：**

| 命令 | 功能 |
|------|------|
| `./start_all.sh bg` | 后台启动全部驱动 |
| `./start_all.sh status` | 查看所有进程状态 |
| `./start_all.sh stop` | 停止全部驱动 |
| `./start_all.sh logs` | 实时查看所有日志 |

> ⚠️ **注意**：首次启动后 INS 需要 3-10 分钟对准（见[第7节](#7-ins-惯导对准)），请保持机器人静止！

### 4.3 VM 端启动

#### 手柄控制

```bash
ssh carl@172.16.30.0
# 密码: 159357

bash ~/rov_ros2_ws/start_joy.sh
```

启动后终端会显示实时操控面板。

#### 传感器综合监控

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
python3 ~/rov_ros2_ws/monitor/integrated_monitor.py
```

---

## 5. 手柄操控

### 5.1 按键映射 (Logitech F710)

```
    ┌─────────────────────────────────┐
    │           Logitech F710          │
    │                                  │
    │   ┌──────────┐    ┌──────────┐   │
    │   │  左摇杆   │    │  右摇杆   │   │
    │   │          │    │          │   │
    │   │  ▲  前进  │    │  ▲  上浮  │   │
    │   │◄├─► 转向  │    │  │       │   │
    │   │  ▼  后退  │    │  ▼  下潜  │   │
    │   └──────────┘    └──────────┘   │
    │                                  │
    │   LB(降档)         RB(升档)       │
    │                                  │
    │   X(急停)   Y       B(恢复)      │
    │   A(急停)                       │
    │                                  │
    │   BACK           START(初始化)    │
    └─────────────────────────────────┘
```

| 操作 | 按键 | 说明 |
|------|------|------|
| **前进/后退** | 左摇杆 ↑/↓ | 控制水平面进退 |
| **左转/右转** | 左摇杆 ←/→ | 原地转向 |
| **上浮/下潜** | 右摇杆 ↑/↓ | 垂直方向升降 |
| **急停** | A 或 X | 立即停止所有电机 |
| **恢复** | B | 解除急停，恢复控制 |
| **升档** | RB | 提高最大转速 |
| **降档** | LB | 降低最大转速 |
| **初始化** | START | 发送ROV初始化指令 |

### 5.2 速度档位

| 档位 | 最大转速 | 适用场景 |
|------|---------|---------|
| 1档 | ±1200 rpm | 微调/精确定位 |
| 2档 | ±1400 rpm | **默认档位**，巡航 |
| 3档 | ±1600 rpm | 快速移动 |
| 4档 | ±1800 rpm | 最大推力/抗流 |

> **摇杆死区**：8%以内视为零输入，避免漂移
> **转速限制**：螺旋桨最低启动转速 1200 rpm，低于此值自动提升

### 5.3 安全机制

| 保护措施 | 触发条件 | 动作 |
|---------|---------|------|
| 手柄心跳 | joy_controller 10Hz 持续发布 | — |
| 命令超时 | RK3588 超过 0.5s 未收到命令 | 自动停止所有电机 |
| can_motor 心跳 | 进程自身 3s 超时 | 自动停止 |
| 急停按钮 | 按 A 或 X | 立即停机，禁止摇杆 |
| 退出保护 | Ctrl+C 或进程退出 | 发送 can_motor stop |
| CAN 总线 | 开机自启 | systemd 自动配置 can0 @ 500kbps |

---

## 6. 传感器监控

### 6.1 话题一览

#### INS 惯导话题 (RK3588 发布)

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `/ins/attitude` | Vector3 | 100Hz | yaw, pitch, roll (角度) |
| `/ins/velocity` | Vector3 | 100Hz | ve, vn, vd (m/s) |
| `/ins/position` | Vector3 | 100Hz | lat, lon, alt |
| `/ins/alignment` | Int8 | 100Hz | 对准状态: 0=监控 1=粗对准 2=精对准 3=导航 |
| `/ins/status` | String(JSON) | 1Hz | 完整状态: 温度/卫星/HDOP/工作模式等 |

#### 传感器话题 (RK3588 发布)

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `/rov/depth` | Float32 | 1Hz | 深度 (m) |
| `/rov/depth_temp` | Float32 | 1Hz | 水温 (°C) |
| `/rov/depth_pressure` | Float32 | 1Hz | 压力 (MPa) |
| `/rov/altitude` | Float32 | 1Hz | 最强目标高度 (m) |
| `/rov/altitude_nearest` | Float32 | 1Hz | 最近目标高度 (m) |

#### 控制话题

| 话题 | 类型 | 方向 | 频率 | 内容 |
|------|------|------|------|------|
| `/rov/cmd_vel` | Twist | VM→RK3588 | 10Hz | 归一化控制值 (±1) |
| `/rov/motor_state` | String(JSON) | RK3588→VM | 1Hz | 当前执行转速 |
| `/rov/joy_state` | String(JSON) | VM→RK3588 | 事件 | 手柄事件/按键 |
| `/rov/cmd_vel_echo` | Twist | RK3588→VM | 10Hz | 实际转速回显 |

#### 声纳话题 (声纳驱动发布, Domain=0)

| 话题 | 类型 | 内容 |
|------|------|------|
| `/sonar/omni/original` | PointCloud2 | 原始点云 |
| `/sonar/omni/rigidity` | PointCloud2 | 刚性目标点云 |
| `/sonar/omni/boundary` | PointCloud2 | 边界点云 |

### 6.2 综合监控界面 (integrated_monitor.py)

```
╔══════════════════════════════════════════════════════════╗
║              ROV 综合监控仪表板              FPS: 9.8    ║
╠══════════════════════════════════════════════════════════╣
║  INS 姿态     航向:  45.2°  俯仰:  -1.0°  横滚:  -1.4°║
║  INS 速度     Ve: 0.01  Vn: 0.01  Vd:-0.01  (m/s)     ║
║  INS 位置     Lat:22.7300  Lon:113.5400  Alt:51.2m     ║
║  GNSS         卫星:12  HDOP:0.8  Fix:3D  温度:41°C    ║
║  INS 状态     ⚠ 精对准中... 数据仅供参考               ║
╠══════════════════════════════════════════════════════════╣
║  D30 深温计   深度:0.12m  水温:27.6°C  压力:0.001MPa  ║
║  SF 高度计    最强:1.36m  最近:1.36m                   ║
╚══════════════════════════════════════════════════════════╝
```

**颜色编码**：
- 🟢 绿色 — 正常/导航模式，数据可信
- 🟡 黄色 — 粗对准中，数据仅供参考
- 🔴 红色 — 数据超时 >5秒，设备离线

---

## 7. INS 惯导对准

### 7.1 对准流程

```
上电 ─→ 监控状态 ─→ 输入位置命令 ─→ 粗对准 ─→ 精对准 ─→ 导航
 (0)               (0x4C/54/45)    (1)       (2)       (3)
                                      2~5分钟    1~3分钟
```

1. **上电**：INS 进入"监控状态"（alignment=0），不输出有效姿态
2. **自动初始化**（ins_driver_auto.py 自动执行）：
   - 输入纬度 → 经度 → 海拔（各发送一次命令）
   - 发送启动命令
3. **对准阶段**：
   - **粗对准 (1)**：约 2-5 分钟，INS 初步确定姿态
   - **精对准 (2)**：约 1-3 分钟，姿态精度提高
   - **导航 (3)**：对准完成，航向/位置数据可信
4. **默认参考位置**：22.73°N, 113.54°E, 50m（深圳）

### 7.2 对准注意事项

- ⚠️ **保持机器人完全静止！** 对准期间任何移动都会延长对准时间
- 对准完成后自动进入"导航模式"，此前位置/航向数据不可用
- 可在综合监控界面观察对准状态变化
- **不要重复发送启动命令！** 不会加速对准，可能干扰

### 7.3 手动修改参考位置

如需更改参考位置（非深圳地区），在 RK3588 上：

```bash
cd /opt/ros/rov_ros2_ws
python3 ins_driver_auto.py --lat 39.9 --lon 116.4 --alt 50
```

---

## 8. 声纳系统

### 8.1 启动声纳

声纳使用独立的 DDS Domain (0)，与 INS/控制分开：

```bash
# RK3588 上
source /opt/ros/setup.bash
source /opt/ros/rov_ros2_ws/install/local_setup.bash
ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5
```

### 8.2 声纳监控

```bash
# 仪表板模式
export ROS_DOMAIN_ID=0
python3 ~/rov_ros2_ws/monitor/sonar_monitor.py

# 3D可视化
python3 ~/rov_ros2_ws/monitor/sonar_3d_view.py --range 5
```

---

## 9. 部署与升级

### 9.1 从 Windows 一键部署

```powershell
cd D:\Carl_WorkStation\rov_ros2\deploy
python deploy_now.py --all
```

| 参数 | 说明 |
|------|------|
| `--all` | 同时部署到 RK3588 + VM |
| `--rk3588` | 仅部署到 RK3588 |
| `--vm` | 仅部署到 VM |

### 9.2 已部署文件清单

**RK3588** (`/opt/ros/rov_ros2_ws/`)：

| 文件 | 说明 |
|------|------|
| `start_all.sh` | 一键启动主脚本 |
| `setup_ip.sh` | IP永久配置 |
| `setup_can.sh` | CAN总线配置 |
| `ins_driver_auto.py` | INS自动驱动 |
| `motor_controller.py` | 电机控制器 |
| `sensors/depth_sensor_driver.py` | D30深度计驱动 |
| `sensors/altimeter_driver.py` | SF高度计驱动 |

**VM** (`~/rov_ros2_ws/`)：

| 文件 | 说明 |
|------|------|
| `start_joy.sh` | 手柄一键启动 |
| `monitor/joy_controller.py` | 手柄控制器 |
| `monitor/integrated_monitor.py` | 综合监控界面 |
| `monitor/sonar_monitor.py` | 声纳仪表板 |
| `monitor/sonar_3d_view.py` | 声纳3D可视化 |

### 9.3 日志文件

| 日志 | 路径 | 内容 |
|------|------|------|
| INS驱动 | `/tmp/ins_driver.log` | INS帧解析、对准状态 |
| 深度计 | `/tmp/depth_sensor.log` | D30 MODBUS通信 |
| 高度计 | `/tmp/altimeter.log` | SF串口通信 |
| 电机控制器 | ROS2日志 | 转速命令执行 |
| joy_node | `/tmp/joy_node.log` | 手柄驱动 |

---

## 10. 故障排查

### 10.1 启动问题

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| `start_joy.sh` 报 "No such file" | 路径错误 | 确认 `joy_controller.py` 在 `monitor/` 子目录 |
| 手柄无响应 | 手柄未连接 | `ls /dev/input/js0` 检查设备 |
| 电机不转 | CAN 未配置 | `./setup_can.sh force` 强制配置 |
| 电机转速 <1200 不转 | 启动阈值 | 提升档位（LB/RB），最小启动转速 1200 rpm |

### 10.2 通信问题

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| VM 看不到话题 | ROS_DOMAIN_ID 不一致 | `export ROS_DOMAIN_ID=42` + `ros2 topic list` |
| INS 无数据 | INS 未上电/IP不通 | `ping 192.168.0.7` |
| INS 卡在监控状态 | 未输入参考位置 | 检查日志 `tail -f /tmp/ins_driver.log` |
| 深度计无数据 | 串口路径错误 | 确认 ttyS5 存在: `ls /dev/ttyS5` |
| 高度计无数据 | 串口干扰/连接 | 检查日志 `cat /tmp/altimeter.log` |

### 10.3 常规检查命令

```bash
# SSH 到 RK3588
ssh root@172.16.28.82

# 查看所有驱动进程
./start_all.sh status

# 查看网络
ip addr show eth0 | grep inet
ping 192.168.0.7

# 查看 CAN 状态
./setup_can.sh status

# 查看 ROS2 话题
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list

# 查看实时日志
tail -f /tmp/ins_driver.log /tmp/depth_sensor.log /tmp/altimeter.log
```

---

## 11. 参考附录

### 11.1 常用命令速查

#### 电机手动控制（仅调试用）

```bash
/usr/can_motor_v1.0 init                          # 上电初始化（必须先执行）
/usr/can_motor_v1.0 move 1500                     # 前进 1500rpm
/usr/can_motor_v1.0 up 1500                       # 上浮 1500rpm
/usr/can_motor_v1.0 yaw 1200                      # 右转 1200rpm
/usr/can_motor_v1.0 run move=1500 up=1000 yaw=0   # 前进+上浮（持续心跳, Ctrl+C停止）
/usr/can_motor_v1.0 stop                          # 全部停止
/usr/can_motor_v1.0 status                        # 读取电机反馈
```

#### CAN 总线管理

```bash
./setup_can.sh          # 配置 can0 up @ 500kbps
./setup_can.sh status   # 查看详细状态
./setup_can.sh force    # 强制重新配置（先 down 再 up）
```

#### ROS2 调试

```bash
# 查看 /rov/cmd_vel 实时数据
ros2 topic echo /rov/cmd_vel

# 查看 INS 姿态
ros2 topic echo /ins/attitude

# 查看电机状态
ros2 topic echo /rov/motor_state

# 查看节点图
rqt_graph
```

### 11.2 密码表

| 设备 | 用户名 | 密码 |
|------|--------|------|
| RK3588 SSH | root | 159357 |
| VM SSH | carl | 159357 |
| 声纳 WEB管理 | admin | admin |

### 11.3 ROS2 配置

| 参数 | 值 | 说明 |
|------|-----|------|
| ROS_DOMAIN_ID | 42 | INS/控制域 |
| ROS_DOMAIN_ID (声纳) | 0 | 声纳独立域 |
| ROS_LOCALHOST_ONLY | 0 | 允许跨主机通信 |
| ROS版本 | Foxy | RK3588 & VM 一致 |

### 11.4 INS 协议摘要

- **帧格式**：202字节，帧头 `0x5A 0xA5`，帧尾 `0x55`
- **波特率**：UDP 8008（接收）/ 8007（发送）
- **频率**：100Hz
- **启动流程**：输入纬度(0x4C) → 经度(0x54) → 海拔(0x45) → 启动(0x47)
- **默认位置**：22.73°N, 113.54°E, 50m

### 11.5 传感器协议摘要

| 传感器 | 协议 | 参数 | 设备地址 |
|--------|------|------|---------|
| D30 深温计 | MODBUS-RTU | 19200/8N1 | 1 |
| SF 高度计 | AA/A0 自定义 | 9600/8N1 | 1 |
| 声纳 | FE/FD 自定义 | UDP 23 | — |

### 11.6 文件路径速查

| 位置 | 路径 |
|------|------|
| 项目源码 (Windows) | `D:\Carl_WorkStation\rov_ros2\` |
| RK3588 工作空间 | `/opt/ros/rov_ros2_ws/` |
| VM 工作空间 | `~/rov_ros2_ws/` |
| CAN 电机程序 | `/usr/can_motor_v1.0` |
| 部署脚本 | `D:\Carl_WorkStation\rov_ros2\deploy\deploy_now.py` |
| 协议文档 | `D:\Carl_WorkStation\rov_ros2\docs\` |
| INS400文档 | `D:\Carl_WorkStation\FileData\` |

---

> **快速启动卡片**（打印备用）：
>
> ```
> 1. RK3588:  ssh root@172.16.28.82
>             cd /opt/ros/rov_ros2_ws && ./start_all.sh bg
>
> 2. VM手柄:   ssh carl@172.16.30.0
>             bash ~/rov_ros2_ws/start_joy.sh
>
> 3. VM监控:   source /opt/ros/foxy/setup.bash
>             export ROS_DOMAIN_ID=42
>             python3 ~/rov_ros2_ws/monitor/integrated_monitor.py
>
> 急停: 按 A 或 X 键 | 恢复: 按 B 键
> ```
