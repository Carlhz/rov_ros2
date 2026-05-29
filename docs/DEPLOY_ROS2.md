# ROS2 纯架构部署指南

## 架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                         VM (Ubuntu)                              │
│  ┌──────────────────┐    ┌───────────────────────────────────┐  │
│  │ ins_control_client│───→│ 调用 /ins/control 服务             │  │
│  │ (命令行工具)      │    │                                   │  │
│  └──────────────────┘    │  CMD_STOP    → 停止INS             │  │
│  ┌──────────────────┐    │  CMD_START   → 启动INS             │  │
│  │ ins_monitor       │←───│  CMD_SET_POS → 设置初始位置        │  │
│  │ (监控节点)        │    │  CMD_GET_STATUS → 获取状态         │  │
│  └──────────────────┘    └───────────────────────────────────┘  │
│                              │                                    │
│                              ↓ ROS2 DDS (Domain 42)               │
└─────────────────────────────────────────────────────────────────┘
                               │
                               ↓ 网络 (172.16.28.x)
┌─────────────────────────────────────────────────────────────────┐
│                      RK3588 (192.168.0.99)                       │
│  ┌───────────────────────────────────────────────────────────┐   │
│  │              ins_driver_controlled.py                      │   │
│  │                                                            │   │
│  │  ┌─────────────┐    ┌────────────────────────────────┐   │   │
│  │  │ /ins/control│←───│ 接收 ROS2 服务请求              │   │   │
│  │  │   服务      │    │                                │   │   │
│  │  └─────────────┘    │  CMD_STOP  → 发送 STOP_CMD     │   │   │
│  │                     │  CMD_SET_POS→发送位置命令      │   │   │
│  │  ┌─────────────┐    │  CMD_START → 发送 START_CMD    │   │   │
│  │  │ 数据发布    │    └────────────────────────────────┘   │   │
│  │  │ /ins/*      │                                         │   │
│  │  └─────────────┘    ┌────────────────────────────────┐   │   │
│  │                     │ UDP Socket                     │   │   │
│  └─────────────────────┤  8008: 接收INS数据              ├───┘   │
└────────────────────────│  8007: 发送控制命令             │───────┘
                         └────────────────────────────────┘
                                    │
                                    ↓ UDP
                         ┌─────────────────────┐
                         │   INS (192.168.0.7) │
                         │  8007: 控制端口      │
                         │  8008: 数据端口      │
                         └─────────────────────┘
```

## 文件结构

```
rov_ros2/src/
├── rov_ins_interface/          # 服务接口定义
│   ├── srv/INSCommand.srv      # 控制服务定义
│   ├── CMakeLists.txt
│   └── package.xml
├── rov_ins_driver/             # RK3588 驱动
│   ├── rov_ins_driver/
│   │   ├── ins_driver_controlled.py   # 带控制服务
│   │   └── ins_driver_full.py         # 旧版（备用）
│   ├── package.xml
│   └── setup.py
├── rov_ins_control/            # VM 控制工具
│   ├── rov_ins_control/
│   │   └── ins_control_client.py      # 命令行客户端
│   ├── package.xml
│   └── setup.py
└── rov_ins_monitor/            # VM 监控节点
    ├── rov_ins_monitor/
    │   └── ins_monitor.py
    ├── package.xml
    └── setup.py
```

---

## 部署步骤

### 第一步：编译接口包（VM 和 RK3588 都需要）

**在 VM 上：**
```bash
cd ~/rov_ros2_ws
colcon build --packages-select rov_ins_interface
source install/setup.bash
```

**在 RK3588 上：**
```bash
# 创建工作空间
mkdir -p ~/rov_ros2_ws/src
cd ~/rov_ros2_ws/src

# 复制接口包（从共享目录或 scp）
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_interface .

# 编译
cd ~/rov_ros2_ws
colcon build --packages-select rov_ins_interface
source install/setup.bash
```

---

### 第二步：部署 RK3588 驱动

**在 RK3588 上：**
```bash
# 复制驱动包
cd ~/rov_ros2_ws/src
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_driver .

# 编译
cd ~/rov_ros2_ws
colcon build --packages-select rov_ins_driver
source install/setup.bash

# 运行
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 run rov_ins_driver ins_driver
```

验证服务是否可用：
```bash
# 在 RK3588 上
ros2 service list | grep control
# 应该看到: /ins/control
```

---

### 第三步：部署 VM 控制工具

**在 VM 上：**
```bash
cd ~/rov_ros2_ws/src

# 复制控制包和监控包
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_control .
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_monitor .

# 编译
cd ~/rov_ros2_ws
colcon build --packages-select rov_ins_control rov_ins_monitor
source install/setup.bash
```

---

## 使用方式

### 1. 启动监控节点（VM）

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 run rov_ins_monitor ins_monitor
```

### 2. 发送控制命令（VM）

**停止 INS（进入监控状态）：**
```bash
ros2 run rov_ins_control ins_control_client stop
```

**设置初始位置：**
```bash
ros2 run rov_ins_control ins_control_client setpos 31.234567 121.456789 0.0
```

**启动 INS（开始对准）：**
```bash
ros2 run rov_ins_control ins_control_client start
```

**获取当前状态：**
```bash
ros2 run rov_ins_control ins_control_client status
```

---

## 完整工作流程

```bash
# 1. 确保 RK3588 驱动已运行
# (在 RK3588 上)
ros2 run rov_ins_driver ins_driver

# 2. 在 VM 上打开两个终端

# 终端 1: 监控
ros2 run rov_ins_monitor ins_monitor

# 终端 2: 控制
# 停止 INS
ros2 run rov_ins_control ins_control_client stop

# 等待状态变为"监控"

# 设置初始位置
ros2 run rov_ins_control ins_control_client setpos 31.234567 121.456789

# 启动 INS 对准
ros2 run rov_ins_control ins_control_client start

# 监控状态变化: 监控 → 粗对准 → 精对准 → INS导航
```

---

## 故障排查

### 问题：服务不可用

```bash
# 检查 RK3588 服务是否注册
ros2 service list | grep ins

# 检查网络发现
ros2 node list
```

### 问题：命令发送失败

```bash
# 检查 ROS_DOMAIN_ID 是否一致
echo $ROS_DOMAIN_ID

# 检查网络连通
ping 192.168.0.99  # 从 VM ping RK3588
```

### 问题：编译失败

```bash
# 清理重新编译
cd ~/rov_ros2_ws
rm -rf build/ install/ log/
colcon build
```

---

## 注意事项

1. **ROS_DOMAIN_ID**: 两边必须一致（建议 42）
2. **网络**: VM 和 RK3588 必须在同一网段（172.16.28.x）
3. **防火墙**: 确保没有阻断 ROS2 DDS 端口
4. **设置位置命令**: 当前是占位实现，需要根据实际 INS 协议调整
