# RK3588 部署指南

## 硬件信息
- **设备**: RK3588
- **串口**: COM5 (115200 baud) - 仅用于调试
- **主 IP**: 172.16.28.82 (连接 VM)
- **副 IP**: 192.168.0.99/24 (连接 INS)
- **网络拓扑**: INS → 交换机 → RK3588 eth0

## 网络配置确认

在 RK3588 上执行：

```bash
# 检查 IP 配置
ip addr show eth0

# 应该看到：
# - 172.16.28.82 (连接 VM)
# - 192.168.0.99 (连接 INS)

# 测试网络连通性
ping 192.168.0.7     # INS 设备
ping 172.16.28.x     # VM 的 IP
```

## 第一步：确认 ROS2 已安装

```bash
# 检查 ROS2 版本
ls /opt/ros/

# 应该看到 foxy 或 humble

# 测试 ROS2 命令
source /opt/ros/foxy/setup.bash
ros2 --help
```

如果 ROS2 未安装，参考官方文档安装：
- Foxy: https://docs.ros.org/en/foxy/Installation.html
- Humble: https://docs.ros.org/en/humble/Installation.html

## 第二步：创建工作空间

```bash
# 创建工作空间
mkdir -p ~/rov_ros2_ws/src
cd ~/rov_ros2_ws

# 确认当前路径
pwd
# 输出: /home/你的用户名/rov_ros2_ws
```

## 第三步：复制源码

### 方式1：从 VMware 共享目录复制（推荐）

```bash
cd ~/rov_ros2_ws/src

# 复制三个包
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_interface .
cp -r /mnt/hgfs/CarlWS/rov_ros2/src/rov_ins_driver .

# 检查是否复制成功
ls -la
# 应该看到 rov_ins_interface 和 rov_ins_driver
```

### 方式2：使用 scp 从 VM 复制

在 VM 上执行：
```bash
cd ~/rov_ros2_ws/src
scp -r rov_ins_interface rov_ins_driver root@192.168.0.99:~/rov_ros2_ws/src/
```

### 方式3：使用 U 盘或 git

将 D:\Carl_WorkStation\rov_ros2\src 下的两个包复制到 RK3588 的 ~/rov_ros2_ws/src/

## 第四步：编译

```bash
cd ~/rov_ros2_ws

# 清理旧编译（如果有）
rm -rf build/ install/ log/

# Source ROS2
source /opt/ros/foxy/setup.bash

# 编译接口包（先编译，因为驱动依赖它）
colcon build --packages-select rov_ins_interface

# 检查是否成功
ls install/

# 编译驱动包
colcon build --packages-select rov_ins_driver

# 完整编译（如果上面都成功，可以一起编译）
# colcon build --packages-select rov_ins_interface rov_ins_driver
```

## 第五步：验证编译结果

```bash
cd ~/rov_ros2_ws

# Source 编译结果
source install/setup.bash

# 检查包是否识别
ros2 pkg list | grep rov_ins

# 应该看到：
# rov_ins_driver
# rov_ins_interface

# 检查可执行文件
ls install/rov_ins_driver/lib/rov_ins_driver/

# 应该看到 ins_driver 和 ins_driver_old
```

## 第六步：运行驱动

### 终端 1：启动驱动

```bash
cd ~/rov_ros2_ws

# 设置环境
source /opt/ros/foxy/setup.bash
source install/setup.bash

# 设置 ROS2 域 ID（必须与 VM 相同）
export ROS_DOMAIN_ID=42

# 允许跨网络发现
export ROS_LOCALHOST_ONLY=0

# 运行驱动
ros2 run rov_ins_driver ins_driver
```

正常输出：
```
[INFO] [ins_driver_controlled]: INS Driver with Control Service starting...
[INFO] [ins_driver_controlled]: Control service /ins/control ready
[INFO] [ins_driver_controlled]: Driver ready, listening on 192.168.0.99:8008
[INFO] [ins_driver_controlled]: NOTE: INS control commands must be sent manually via /ins/control service
```

### 终端 2：检查话题（可选）

```bash
# 另开一个终端
cd ~/rov_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# 列出所有话题
ros2 topic list

# 应该看到 /ins/latitude, /ins/pose, /ins/imu 等

# 检查话题是否有数据
ros2 topic echo /ins/latitude
```

## 第七步：与 VM 联调

### 在 RK3588 上确认服务可用

```bash
cd ~/rov_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# 列出服务
ros2 service list | grep ins

# 应该看到 /ins/control
```

### 在 VM 上测试控制命令

```bash
# 在 VM 上执行
cd ~/rov_ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# 停止 INS
ros2 run rov_ins_control ins_control_client stop

# 设置位置
ros2 run rov_ins_control ins_control_client setpos 31.234567 121.456789

# 启动 INS
ros2 run rov_ins_control ins_control_client start

# 获取状态
ros2 run rov_ins_control ins_control_client status
```

## 常见问题

### 问题1：找不到 ROS2 命令

```bash
# 检查 ROS2 安装
ls /opt/ros/

# 手动 source
source /opt/ros/foxy/setup.bash
```

### 问题2：编译失败，找不到接口定义

```bash
# 确保先编译接口包
colcon build --packages-select rov_ins_interface
source install/setup.bash
colcon build --packages-select rov_ins_driver
```

### 问题3：VM 和 RK3588 无法互通

```bash
# 在 RK3588 上 ping VM
ping 172.16.28.x  # VM 的 IP

# 在 VM 上 ping RK3588
ping 172.16.28.82

# 检查防火墙
sudo iptables -L
```

### 问题4：ROS2 话题看不到

确保两边环境变量一致：
```bash
# 两边都执行
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 检查
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
```

### 问题5：没有 INS 数据

```bash
# 检查网络
ping 192.168.0.7

# 抓包测试
sudo tcpdump -i eth0 udp port 8008

# 检查驱动日志
ros2 run rov_ins_driver ins_driver 2>&1 | tee driver.log
```

## 开机自启动（可选）

创建 systemd 服务：

```bash
# 创建服务文件
sudo tee /etc/systemd/system/rov_ins_driver.service << 'EOF'
[Unit]
Description=ROV INS Driver
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/root/rov_ros2_ws
Environment="ROS_DOMAIN_ID=42"
Environment="ROS_LOCALHOST_ONLY=0"
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run rov_ins_driver ins_driver'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 启用服务
sudo systemctl daemon-reload
sudo systemctl enable rov_ins_driver.service
sudo systemctl start rov_ins_driver.service

# 查看状态
sudo systemctl status rov_ins_driver.service
```

## 总结

| 步骤 | 命令 | 说明 |
|------|------|------|
| 1 | `source /opt/ros/foxy/setup.bash` | 加载 ROS2 |
| 2 | `colcon build` | 编译 |
| 3 | `source install/setup.bash` | 加载工作空间 |
| 4 | `export ROS_DOMAIN_ID=42` | 设置域 ID |
| 5 | `ros2 run rov_ins_driver ins_driver` | 运行驱动 |

RK3588 只需要运行驱动节点，控制命令从 VM 通过 ROS2 服务发送过来。
