# ROV ROS2 集成控制系统

七推进器小型 ROV 的 ROS2 Foxy 全栈控制软件，运行在 RK3588 (aarch64) + Ubuntu VM (x86_64) 双机架构上。

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                      Ubuntu VM (上位机 x86_64)                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  joy_controller.py      ← 罗技 F710 手柄，10Hz cmd_vel      │  │
│  │  integrated_monitor.py  ← 彩色终端：深度/姿态/电机/航向     │  │
│  └────────────────────────────────────────────────────────────┘  │
└─────────────────────────┬────────────────────────────────────────┘
                          │ DDS UDP (ROS_DOMAIN_ID=42)
                          │ 172.16.30.0/22 ←→ 172.16.28.82
┌─────────────────────────┼────────────────────────────────────────┐
│                 RK3588 (ROV 主控, aarch64, 172.16.28.82)          │
│  ┌──────────────────────┴──────────────────────────────────────┐ │
│  │  motor_controller.py  ← 双阶段PID定深/定航向 + 前馈补偿      │ │
│  │  ins_driver_auto.py   ← INS 导航仪 (UDP:192.168.0.7:8008)  │ │
│  │  dvl_driver.py        ← PathFinder DVL (TCP:192.168.0.6)   │ │
│  │  depth_sensor_driver  ← D30 深温计 (Modbus-RTU, ttyS5)     │ │
│  │  altimeter_driver     ← SF 超声波高度计 (ttyS3)              │ │
│  │  sonar_omni_driver*   ← 全向声纳 (UDP:192.168.0.5:23, C++) │ │
│  └─────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

## 硬件配置

### 推进器布局（7 电机）

| ID | 位置（船身坐标系）| 方向 | RPM 范围 | 备注 |
|----|-------------------|------|---------|------|
| 0 | 尾部左舷上方 *(面对尾部时在右上)* | 上倾 +22.5° | 1100–1550 | 前进产生下压 |
| 1 | 尾部左舷下方 *(面对尾部时在右下)* | 下倾 -22.5° | 1100–1550 | 反装，后退产生下压 |
| 2 | 尾部右舷下方 *(面对尾部时在左下)* | 下倾 -22.5° | 1100–1550 | 反装，后退产生下压 |
| 3 | 尾部右舷上方 *(面对尾部时在左上)* | 上倾 +22.5° | 1100–1550 | 前进产生下压 |
| 5 | 垂直推进 | — | 1100–1550 | ID5 CW→下潜 |
| 6 | 垂直推进 | — | 1100–1550 | ID6 CW→下潜（CAN 反相）|
| 7 | 横向/转向 | — | 1100–1400 | YAW_DIRECTION=-1 修正 |

### 传感器一览

| 传感器 | 型号 | 接口 | 话题 |
|--------|------|------|------|
| INS 导航仪 | — | UDP 8008 (192.168.0.7) | `/ins/attitude`, `/ins/velocity`, `/ins/position` |
| D30 深温计 | CERPTS05D30-485 | RS-485, ttyS5, 19200bps | `/rov/depth`, `/rov/depth_temp`, `/rov/depth_pressure` |
| SF 高度计 | 超声波 | RS-485, ttyS3, 9600bps | `/rov/altitude`, `/rov/altitude_nearest` |
| DVL | PathFinder 600kHz | TCP 192.168.0.6 (PD0) | `/rov/dvl/bottom_vel`, `/rov/dvl/altitude`, `/rov/dvl/status` |
| 全向声纳 | — | UDP 192.168.0.5:23 | `/rov/sonar_omni/*` |

### 网络拓扑

```
VM (172.16.30.0/22)
  ↕
RK3588 (172.16.28.82 / 192.168.0.99)
  ↕ 交换机
  ├── 声纳 (192.168.0.5)
  ├── INS  (192.168.0.7)
  └── DVL  (192.168.0.6)
```

## 目录结构

```
rov_ros2/
├── rk3588/                  # RK3588 端主程序
│   ├── motor_controller.py  # 电机控制 v8.0（PID + 推力分配 + CAN）
│   ├── thrust_allocator.py  # B+ 伪逆推力分配矩阵
│   ├── start_all.sh         # 统一启停（传感器 + 电机 + DVL）
│   ├── dvl_driver.py        # PathFinder DVL PD0 协议驱动
│   ├── ins_driver_auto.py   # INS 导航仪 202 字节协议驱动
│   ├── auto_depth_test.py   # 自动化 5 轮定深测试
│   ├── setup_can.sh         # CAN0 初始化（500kbps, systemd 自启）
│   └── setup_ip.sh          # 辅助 IP 192.168.0.99
│
├── vm/                      # VM 上位机端
│   ├── joy_controller.py    # 手柄控制 v5.0（4 档位，定深/定航向）
│   ├── integrated_monitor.py# 集成监控（深度/姿态/PID/电机/航向）
│   ├── start_joy.sh         # 手柄启动脚本
│   └── start_monitor.sh     # 监控启动脚本
│
├── sensors/                 # 传感器驱动（纯 Python）
│   ├── depth_sensor_driver.py   # D30 深温计 Modbus-RTU
│   └── altimeter_driver.py      # SF 高度计 AA/A0 协议
│
├── src/                     # ROS2 colcon 包（需交叉编译）
│   ├── rov_sonar_driver/    # 全向声纳 C++ 驱动
│   ├── rov_sonar_interface/ # 声纳配置服务（.srv）
│   └── rov_sonar_monitor/   # 声纳可视化节点
│
├── deploy/                  # 部署工具
│   ├── deploy_now.py        # 一键部署到 RK3588 + VM
│   ├── toolchain_aarch64.cmake      # 交叉编译工具链
│   └── toolchain_aarch64_relaxed.cmake
│
├── tools/                   # 辅助工具
│   ├── train_depth_ff.py    # 前馈补偿模型训练（最小二乘）
│   ├── yaw_balance_calc.py  # Yaw 推力平衡分析
│   └── pull_csv.py          # 从 RK3588 取回 CSV 数据
│
├── docs/                    # 文档
│   ├── ROV_OPERATION_MANUAL.md
│   ├── SONAR_OMNI_PROTOCOL.md
│   └── ROS2_CPP_CROSS_COMPILE_GUIDE.md
│
└── test/                    # 测试脚本
```

## 快速开始

### 环境变量（两端都必须设置）

```bash
source /opt/ros/foxy/setup.bash    # RK3588 用 Foxy
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

### RK3588 端：一键启动全部

```bash
ssh root@172.16.28.82
cd /opt/ros/rov_ros2_ws

./start_all.sh bg       # 后台启动（传感器 + DVL + 电机控制）
./start_all.sh stop     # 停止全部
./start_all.sh status   # 查看运行状态
./start_all.sh logs     # 查看所有日志
```

### VM 端：手柄 + 监控

```bash
# 终端 1：手柄控制
source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=42
bash ~/rov_ros2_ws/start_joy.sh

# 终端 2：彩色监控面板
source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=42
python3 ~/rov_ros2_ws/vm/integrated_monitor.py
```

### 从 Windows 一键部署

```bash
cd D:\Carl_WorkStation\rov_ros2\deploy
python deploy_now.py --all     # 部署到 RK3588 + VM
python deploy_now.py --vm      # 仅部署 VM 脚本
```

## 控制功能

### 手柄操作（罗技 F710 D 模式）

| 操作 | 功能 |
|------|------|
| 左摇杆 Y（上下）| 前进/后退 |
| 右摇杆 Y（上下）| 下潜/上浮 |
| 右摇杆 X（左右）| 左转/右转 |
| A 键 | 急停（所有电机停转）|
| B 键 | 恢复 |
| LB / RB | 降档/升档（1~4 档，4 档=定深专用）|
| X 键 | 定航向开关（任意档位）|
| Y 键 | 定深悬停开关（仅 4 档）|

定航向开启后，LB/RB 调整目标航向 ±5°；定深悬停后，LB/RB 调整目标深度 ±0.1m。

### 控制模式

- **手动模式**：手柄直接映射到 7 路电机 RPM，PID 不介入
- **定深模式**（4 档 + Y 键）：两阶段深度控制 + Roll/Pitch 姿态 PID
  - 阶段 1（误差 >0.10m）：固定推力快速趋近
  - 阶段 2（误差 ≤0.10m）：PID 精细调节 + 前馈补偿
- **定航向模式**（X 键）：两阶段 Yaw 控制，ID7 主导 + 尾推辅助
  - 阶段 1（误差 >10°）：固定 1400 RPM 大转速回正
  - 阶段 2（误差 ≤10°）：高增益 PID 微调（KP=0.15, KI=0.06）

### 推力分配

基于 B+ 伪逆矩阵的 7×6 推力分配（`thrust_allocator.py`），将 6-DOF 归一化力/力矩映射到 7 路电机 RPM。支持：
- 尾部电机差速产生 Roll/Pitch/Yaw 力矩
- 垂直推力的尾推辅助混合（防俯仰摆动）
- Fz 深度控制绕开 B+ 分配器，手工分配到推力矩阵中

### 安全保护

- 5 秒无 cmd_vel → 自动全停
- Pitch >30° 线性降推，>55° 全部归零
- A 键急停（发布零速 cmd_vel）

## 话题速查

### 传感器话题

| 话题 | 发布频率 | 说明 |
|------|---------|------|
| `/rov/depth` | 10Hz | 水深 (m) |
| `/rov/depth_temp` | 1Hz | 水温 (°C) |
| `/rov/altitude` | 5Hz | 距底高度 (m) |
| `/ins/attitude` | 100Hz | roll/pitch/yaw (°) |
| `/ins/velocity` | 100Hz | ve/vn (m/s) |
| `/ins/acceleration` | 100Hz | ax/ay/az (m/s²) |
| `/ins/angular_rate` | 100Hz | wx/wy/wz (°/s) |
| `/ins/alignment` | 1Hz | INS 对准状态 |

### 控制话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `/cmd_vel` | VM → RK3588 | 手柄控制指令 |
| `/rov/motor_state` | RK3588 → VM | 电机状态（JSON），含深度/PID/RPM/姿态/定航向 |

## 交叉编译

全向声纳驱动使用 C++ 编写，在 VM 上交叉编译后部署到 RK3588 (aarch64)。

```bash
# VM 上编译
source /home/carl/RK3588/rk3588_linux_release/ubuntu/environment
colcon build --packages-select rov_sonar_driver --cmake-args \
  -DCMAKE_TOOLCHAIN_FILE=/mnt/hgfs/CarlWS/rov_ros2/deploy/toolchain_aarch64_relaxed.cmake

# 部署到 RK3588
scp -r install/* root@172.16.28.82:/opt/ros/rov_ros2_ws/install/
```

详见 [docs/ROS2_CPP_CROSS_COMPILE_GUIDE.md](docs/ROS2_CPP_CROSS_COMPILE_GUIDE.md)。

## 故障排查

| 现象 | 检查项 |
|------|--------|
| VM 看不到话题 | `echo $ROS_DOMAIN_ID` 是否=42？`ping 172.16.28.82`？|
| 传感器无数据 | RK3588 上 `cat /tmp/{depth_sensor,altimeter,dvl_driver,ins_driver}.log` |
| 电机不转 | `cat /tmp/motor_controller.log`，检查 CAN 状态 |
| INS 无数据 | `ping 192.168.0.7`，冷启动需 2-5 分钟对准 |
| DVL 无数据 | `ping 192.168.0.6`，DVL 核心可能 hung 需物理断电重启 |
| 部署失败 | 需要 paramiko 包，用项目配套 Python 路径执行 |

## 协议文档

- [D30 深温计 MODBUS-RTU 协议](docs/D30_depth_sensor_protocol.md)
- [SF 超声波测深仪协议](docs/SF_altimeter_protocol.md)
- [SONAR_OMNI 全向声纳协议](docs/SONAR_OMNI_PROTOCOL.md)
- [ROV 操作手册](docs/ROV_OPERATION_MANUAL.md)
- [C++ 交叉编译指南](docs/ROS2_CPP_CROSS_COMPILE_GUIDE.md)

## 许可证

MIT
