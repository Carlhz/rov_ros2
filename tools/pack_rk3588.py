#!/usr/bin/env python3
"""
RK3588 部署包打包脚本
将所有需要部署到 RK3588 的文件整合成一个 tar.gz 包
"""
import os
import shutil
import tarfile
import pathlib
from datetime import datetime

# ─────────────────────────────────────────────
ROOT = pathlib.Path("D:/Carl_WorkStation/rov_ros2")
OUT_DIR = pathlib.Path("D:/Carl_WorkStation")
STAGE = pathlib.Path("D:/Carl_WorkStation/rov_ros2_rk3588_deploy")
# ─────────────────────────────────────────────


def copy(src: pathlib.Path, dst: pathlib.Path):
    """复制文件或目录（目录用 copytree）；shell 脚本强制转换为 LF 行尾"""
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.is_dir():
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        # 递归 dos2unix 目录内所有 .sh
        for f in dst.rglob("*.sh"):
            _dos2unix(f)
    else:
        shutil.copy2(src, dst)
        if src.suffix == ".sh":
            _dos2unix(dst)
    print(f"  [+] {dst.relative_to(STAGE)}")


def _dos2unix(path: pathlib.Path):
    """将文件行尾从 CRLF 转换为 LF"""
    data = path.read_bytes()
    if b"\r\n" in data:
        path.write_bytes(data.replace(b"\r\n", b"\n"))



def main():
    # 清理旧暂存目录
    if STAGE.exists():
        shutil.rmtree(STAGE)
    STAGE.mkdir(parents=True)

    print("=" * 60)
    print("  ROV RK3588 部署包打包")
    print("=" * 60)

    # ─────────────────────────────────────────────
    # 1. INS 驱动（纯 Python，直接运行）
    # ─────────────────────────────────────────────
    print("\n[1] INS 驱动")
    ins_dst = STAGE / "ins_driver"
    copy(ROOT / "src/rov_ins_driver/ins_driver_full.py",      ins_dst / "ins_driver_full.py")
    copy(ROOT / "rk3588/rk3588_ins_controller.py",            ins_dst / "rk3588_ins_controller.py")
    copy(ROOT / "rk3588/start_rk3588_controller.sh",          ins_dst / "start_ins_controller.sh")

    # ─────────────────────────────────────────────
    # 2. 深度计 & 高度计 驱动（纯 Python，直接运行）
    # ─────────────────────────────────────────────
    print("\n[2] 深度计 / 高度计驱动")
    sensor_dst = STAGE / "sensors"
    copy(ROOT / "sensors/depth_sensor_driver.py",   sensor_dst / "depth_sensor_driver.py")
    copy(ROOT / "sensors/altimeter_driver.py",      sensor_dst / "altimeter_driver.py")
    copy(ROOT / "rk3588/start_sensors.sh",          sensor_dst / "start_sensors.sh")
    copy(ROOT / "rk3588/test_depth_raw.py",         sensor_dst / "test_depth_raw.py")
    copy(ROOT / "rk3588/test_altimeter_raw.py",     sensor_dst / "test_altimeter_raw.py")

    # ─────────────────────────────────────────────
    # 3. 全向声纳驱动（已交叉编译的 ARM64 包）
    # ─────────────────────────────────────────────
    print("\n[3] 全向声纳驱动（已编译 ARM64）")
    sonar_dst = STAGE / "sonar"
    copy(ROOT / "deploy/sonar_install.tar.gz",      sonar_dst / "sonar_install.tar.gz")
    # 声纳 launch/config 源文件（方便查看/修改参数）
    copy(ROOT / "src/rov_sonar_driver/launch/sonar_omni.launch.py",
         sonar_dst / "launch/sonar_omni.launch.py")
    copy(ROOT / "src/rov_sonar_driver/config/sonar_omni.yaml",
         sonar_dst / "config/sonar_omni.yaml")

    # ─────────────────────────────────────────────
    # 4. CAN 电机控制（预编译二进制 + 源码）
    # ─────────────────────────────────────────────
    print("\n[4] CAN 电机控制")
    motor_dst = STAGE / "motors"
    copy(ROOT / "motors/bin/can_motor_v1.0",        motor_dst / "bin/can_motor_v1.0")
    copy(ROOT / "motors/bin/can_demo_v1.7.1",       motor_dst / "bin/can_demo_v1.7.1")
    copy(ROOT / "motors/bin/h1000test",             motor_dst / "bin/h1000test")
    copy(ROOT / "motors/src/can_motor_v1.0.c",      motor_dst / "src/can_motor_v1.0.c")
    copy(ROOT / "motors/src/can_demo_v1.7.c",       motor_dst / "src/can_demo_v1.7.c")

    # ─────────────────────────────────────────────
    # 5. 协议文档
    # ─────────────────────────────────────────────
    print("\n[5] 协议文档")
    doc_dst = STAGE / "docs"
    for doc in ["D30_depth_sensor_protocol.md", "SF_altimeter_protocol.md",
                "SONAR_OMNI_PROTOCOL.md", "DEPLOY_RK3588.md"]:
        src = ROOT / "docs" / doc
        if src.exists():
            copy(src, doc_dst / doc)

    # ─────────────────────────────────────────────
    # 6. 写 README + 部署说明
    # ─────────────────────────────────────────────
    print("\n[6] 生成 README.md")
    readme = STAGE / "README.md"
    readme.write_text(generate_readme(), encoding="utf-8", newline="\n")
    print(f"  [+] README.md")

    # ─────────────────────────────────────────────
    # 7. 生成一键部署脚本 install.sh
    # ─────────────────────────────────────────────
    print("\n[7] 生成 install.sh")
    install_sh = STAGE / "install.sh"
    install_sh.write_text(generate_install_sh(), encoding="utf-8", newline="\n")
    install_sh.chmod(0o755)
    print(f"  [+] install.sh")

    # ─────────────────────────────────────────────
    # 8. 打包
    # ─────────────────────────────────────────────
    timestamp = datetime.now().strftime("%Y%m%d_%H%M")
    pkg_name = f"rov_rk3588_deploy_{timestamp}.tar.gz"
    pkg_path = OUT_DIR / pkg_name

    print(f"\n[8] 打包 → {pkg_path}")
    with tarfile.open(pkg_path, "w:gz") as tar:
        tar.add(STAGE, arcname="rov_rk3588")

    size_mb = pkg_path.stat().st_size / 1024 / 1024
    print(f"  包大小: {size_mb:.1f} MB")

    # 清理暂存目录
    shutil.rmtree(STAGE)

    print("\n" + "=" * 60)
    print(f"  打包完成：{pkg_path}")
    print("=" * 60)
    print()
    print("传输到 RK3588：")
    print(f"  scp {pkg_path.name} root@172.16.28.82:/opt/")
    print()
    print("在 RK3588 上部署：")
    print("  cd /opt && tar xzf rov_rk3588_deploy_*.tar.gz")
    print("  cd /opt/rov_rk3588 && bash install.sh")

    return pkg_path


def generate_readme():
    return """\
# ROV RK3588 部署包

## 目录结构

```
rov_rk3588/
├── install.sh              ← 一键部署脚本（在 RK3588 上运行）
├── README.md               ← 本文档
│
├── ins_driver/             ← INS 惯性导航驱动（纯 Python）
│   ├── ins_driver_full.py        ROS2 驱动节点（发布 20+ 话题，100Hz）
│   ├── rk3588_ins_controller.py  INS 控制器节点
│   └── start_ins_controller.sh   启动脚本
│
├── sensors/                ← 深度计 & 高度计驱动（纯 Python）
│   ├── depth_sensor_driver.py    D30 深温计驱动（ttyS3, MODBUS-RTU）
│   ├── altimeter_driver.py       SF 超声波高度计驱动（ttyS5）
│   ├── start_sensors.sh          一键启动两路传感器
│   ├── test_depth_raw.py         深度计原始数据测试
│   └── test_altimeter_raw.py     高度计原始数据测试
│
├── sonar/                  ← 全向声纳驱动（已交叉编译 ARM64）
│   ├── sonar_install.tar.gz      ARM64 编译好的 ROS2 包
│   ├── launch/sonar_omni.launch.py
│   └── config/sonar_omni.yaml
│
├── motors/                 ← CAN 总线电机控制
│   ├── bin/can_motor_v1.0        预编译二进制（ARMv8）
│   ├── bin/can_demo_v1.7.1
│   ├── bin/h1000test
│   └── src/                      C 源码（如需重新编译）
│
└── docs/                   ← 协议文档
    ├── D30_depth_sensor_protocol.md
    ├── SF_altimeter_protocol.md
    ├── SONAR_OMNI_PROTOCOL.md
    └── DEPLOY_RK3588.md
```

## 硬件配置

| 设备 | 接口 | 地址/端口 |
|------|------|-----------|
| INS | UDP | 192.168.0.7:8008 |
| 全向声纳 | UDP | 192.168.0.5:23 |
| D30 深温计 | RS485 ttyS3 | 19200 baud, addr=1 |
| SF 高度计 | RS485 ttyS5 | 9600 baud, addr=1 |
| CAN 电机 | SPI-CAN | /dev/spidev0.0 (或 canbus) |

## 网络配置

- RK3588 eth0：172.16.28.82（上位机通信）
- RK3588 eth0 别名：192.168.0.99/24（传感器子网）
- `ROS_DOMAIN_ID=42`（INS/传感器驱动）
- `ROS_DOMAIN_ID=0`（声纳驱动默认）

## 快速开始

```bash
# 1. 解压
cd /opt && tar xzf rov_rk3588_deploy_*.tar.gz

# 2. 一键安装（自动复制文件到正确位置）
cd /opt/rov_rk3588 && bash install.sh

# 3. 启动各模块（分别在不同终端）
# INS 驱动
source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42
python3 /opt/ros/rov_ros2_ws/ins_driver_full.py

# 深度计 + 高度计
/opt/ros/rov_ros2_ws/sensors/start_sensors.sh bg

# 声纳驱动（ROS_DOMAIN_ID=0）
source /opt/ros/setup.bash
source /opt/ros/rov_ros2_ws/install/local_setup.bash
ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5

# CAN 电机
/opt/ros/rov_ros2_ws/motors/bin/can_motor_v1.0
```

## 移植注意事项

### 串口映射
如果新板子串口号不同，用环境变量覆盖默认值：
```bash
DEPTH_PORT=/dev/ttyUSB0 ALTI_PORT=/dev/ttyUSB1 ./start_sensors.sh bg
```

### 声纳驱动安装
声纳驱动需要先解压安装（已编译为 ARM64，Foxy 版本）：
```bash
cd /opt/ros/rov_ros2_ws
tar xzf /opt/rov_rk3588/sonar/sonar_install.tar.gz
# 验证：
source /opt/ros/setup.bash && source install/local_setup.bash
ros2 launch rov_sonar_driver sonar_omni.launch.py --show-args
```

### 网络子网配置
如果新板子没有 192.168.0.99 地址，需要添加：
```bash
# 临时
ip addr add 192.168.0.99/24 dev eth0
# 永久（写入 /etc/network/interfaces 或 netplan）
```

### CAN 总线
电机二进制文件针对 ARMv8 编译，如不能运行则需重新编译：
```bash
gcc -o can_motor_v1.0 motors/src/can_motor_v1.0.c -lpthread -lm
```
"""


def generate_install_sh():
    return """\
#!/bin/bash
# RK3588 一键部署脚本
# 用法: bash install.sh [--workspace /opt/ros/rov_ros2_ws]

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS="${1:-/opt/ros/rov_ros2_ws}"

echo "========================================"
echo "  ROV RK3588 一键部署"
echo "  工作空间: $WS"
echo "========================================"

# 检测 ROS2 路径
if [ -f "/opt/ros/setup.bash" ]; then
    ROS_SETUP="/opt/ros/setup.bash"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/foxy/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
else
    echo "[WARN] 未找到 ROS2 setup.bash，跳过声纳驱动安装"
    ROS_SETUP=""
fi

# 1. 创建工作空间目录
echo ""
echo "[1/5] 创建工作空间..."
mkdir -p "$WS"
mkdir -p "$WS/sensors"
mkdir -p "$WS/motors/bin"
mkdir -p "$WS/motors/src"

# 2. 部署 INS 驱动
echo ""
echo "[2/5] 部署 INS 驱动..."
cp "$SCRIPT_DIR/ins_driver/ins_driver_full.py"       "$WS/ins_driver_full.py"
cp "$SCRIPT_DIR/ins_driver/rk3588_ins_controller.py" "$WS/rk3588_ins_controller.py"
cp "$SCRIPT_DIR/ins_driver/start_ins_controller.sh"  "$WS/start_ins_controller.sh"
chmod +x "$WS/start_ins_controller.sh"
echo "  [OK] INS 驱动 → $WS/"

# 3. 部署传感器驱动
echo ""
echo "[3/5] 部署深度计 / 高度计驱动..."
cp "$SCRIPT_DIR/sensors/"*.py   "$WS/sensors/"
cp "$SCRIPT_DIR/sensors/start_sensors.sh" "$WS/sensors/start_sensors.sh"
chmod +x "$WS/sensors/start_sensors.sh"
echo "  [OK] 传感器驱动 → $WS/sensors/"

# 4. 部署声纳驱动（解压编译好的 ARM64 包）
echo ""
echo "[4/5] 部署全向声纳驱动..."
if [ -f "$SCRIPT_DIR/sonar/sonar_install.tar.gz" ] && [ -n "$ROS_SETUP" ]; then
    tar xzf "$SCRIPT_DIR/sonar/sonar_install.tar.gz" -C "$WS/"
    echo "  [OK] 声纳驱动已解压到 $WS/install/"
    echo "  启动命令:"
    echo "    source $ROS_SETUP"
    echo "    source $WS/install/local_setup.bash"
    echo "    ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5"
else
    echo "  [SKIP] 声纳驱动包未找到或 ROS2 未安装"
fi

# 5. 部署 CAN 电机
echo ""
echo "[5/5] 部署 CAN 电机控制..."
cp "$SCRIPT_DIR/motors/bin/"*  "$WS/motors/bin/"
cp "$SCRIPT_DIR/motors/src/"*  "$WS/motors/src/"
chmod +x "$WS/motors/bin/"*
echo "  [OK] CAN 电机 → $WS/motors/"

echo ""
echo "========================================"
echo "  部署完成！"
echo ""
echo "  快速启动："
echo "  - INS:     python3 $WS/ins_driver_full.py"
echo "  - 传感器:  $WS/sensors/start_sensors.sh bg"
echo "  - 声纳:    ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5"
echo "  - 电机:    $WS/motors/bin/can_motor_v1.0"
echo "========================================"
"""


if __name__ == "__main__":
    main()
