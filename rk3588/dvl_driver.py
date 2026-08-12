#!/usr/bin/env python3
"""
Hover H1000 DVL (Doppler Velocity Log) Driver — 运行于 RK3588

通过 TCP 网口连接 H1000 DVL，解析 PD6 ASCII 协议，发布 ROS2 话题。

连接方式:
  - 命令端口 10000: 发送下行命令 (CS/CZ/DF 等)
  - 数据端口 10001: 接收上行 PD6 ASCII 数据

PD6 数据格式 (6 条语句):
  :SA  姿态 (pitch, roll, heading)
  :TS  时间与环境 (timestamp, salinity, temp, depth, sound_speed, status)
  :BI  设备坐标系速度 (X, Y, Z, error) mm/s
  :BS  船体坐标系速度 (X, Y, Z) mm/s
  :BE  大地坐标系速度 (E, N, U) mm/s  ← 直接用于 bottom_vel
  :BD  大地坐标系距离 (E, N, U, altitude, valid_time)  ← altitude 用于 /rov/dvl/altitude

发布话题:
  /rov/dvl/bottom_vel  (Vector3) 底跟踪速度: E/N/U (m/s)
  /rov/dvl/altitude    (Float32) 距底高度 (m)
  /rov/dvl/status      (String)  完整状态 JSON

用法:
  python3 dvl_driver.py                          # 默认 192.168.0.11
  python3 dvl_driver.py --ip 192.168.0.11
  python3 dvl_driver.py --ip 192.168.0.11 --data-port 10001 --cmd-port 10000
"""

import os
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

import socket
import json
import time
import threading
import signal
import sys
import argparse
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3


# ═══════════════════════════════════════════
#  H1000 命令格式化 (16 字节固定长度)
# ═══════════════════════════════════════════

def format_command(cmd: str) -> bytes:
    """将命令格式化为 H1000 要求的 16 字节格式。

    格式: [CMD][空格][参数]\r[0填充至16字节]
    无参数时: [CMD]\r[0填充至16字节]

    示例:
      "CS"       → b"CS\\r000000000000"    (16 bytes)
      "CZ"       → b"CZ\\r000000000000"    (16 bytes)
      "DF 0"     → b"DF 0\\r00000000000"   (16 bytes)
      "PC 1500.00" → b"PC 1500.00\\r0000"  (16 bytes)
    """
    raw = cmd + '\r'
    if len(raw) > 16:
        raw = raw[:16]
    else:
        raw = raw + '0' * (16 - len(raw))
    return raw.encode('ascii')


# ═══════════════════════════════════════════
#  DVL 驱动节点
# ═══════════════════════════════════════════

class H1000DvlDriver(Node):
    """H1000 DVL ROS2 驱动节点"""

    def __init__(self, ip='192.168.0.11', data_port=10001, cmd_port=10000):
        super().__init__('h1000_dvl_driver')

        self.ip = ip
        self.data_port = data_port
        self.cmd_port = cmd_port

        # ── ROS2 话题 ──
        self.pub_bottom_vel = self.create_publisher(Vector3, '/rov/dvl/bottom_vel', 10)
        self.pub_altitude = self.create_publisher(Float32, '/rov/dvl/altitude', 10)
        self.pub_status = self.create_publisher(String, '/rov/dvl/status', 10)

        # ── 数据状态 ──
        self.latest = {
            'ensemble': 0,
            'altitude': 0.0,
            'depth': 0.0,
            'bottom_vel_east': 0.0,
            'bottom_vel_north': 0.0,
            'bottom_vel_up': 0.0,
            'heading': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'temperature': 0.0,
            'salinity': 0.0,
            'speed_of_sound': 1500,
            'bt_status': 'V',
            'coord_system': 'EARTH',
            'timestamp': '',
        }
        self._ensemble_count = 0
        self._parse_errors = 0
        self._last_data_time = 0.0

        # ── 网络 ──
        self._cmd_sock = None
        self._data_sock = None
        self._running = True
        self._connected = False

        # ── 启动 ──
        self.get_logger().info(f'H1000 DVL Driver 启动 — IP: {ip}')
        self.get_logger().info(f'  数据端口: {data_port}, 命令端口: {cmd_port}')

        # 启动数据接收线程
        self._thread = threading.Thread(target=self._main_loop, daemon=True)
        self._thread.start()

        # 定时发布
        self.create_timer(0.2, self._publish_data)  # 5Hz 发布

        # 定时检查连接
        self.create_timer(5.0, self._check_connection)

    # ═══════════════════════════════════════════
    #  网络连接
    # ═══════════════════════════════════════════

    def _connect_cmd(self):
        """连接命令端口 10000，发送初始化命令"""
        try:
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._cmd_sock.settimeout(3.0)
            self._cmd_sock.connect((self.ip, self.cmd_port))
            self.get_logger().info(f'命令端口 {self.ip}:{self.cmd_port} 已连接')

            # 发送初始化命令序列: CZ → DF 0 → CS
            time.sleep(0.3)
            self._send_cmd('CZ')          # 停止测量
            time.sleep(0.2)
            self._send_cmd('DF 0')        # 设置 PD6 格式
            time.sleep(0.2)
            self._send_cmd('CS')          # 开始测量
            time.sleep(0.3)
            self._send_cmd('SA')          # 启动姿态 :SA 输出 (pitch/roll/heading)
            time.sleep(0.3)
            self.get_logger().info('初始化命令已发送 (CZ → DF 0 → CS → SA)')

            return True
        except Exception as e:
            self.get_logger().warn(
                f'命令端口连接失败: {e}（设备可能已在测量中，尝试仅连数据端口）')
            if self._cmd_sock:
                try:
                    self._cmd_sock.close()
                except Exception:
                    pass
                self._cmd_sock = None
            return False

    def _connect_data(self):
        """连接数据端口 10001"""
        try:
            self._data_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._data_sock.settimeout(3.0)
            self._data_sock.connect((self.ip, self.data_port))
            self._data_sock.settimeout(1.0)  # 连接后设置读取超时
            self.get_logger().info(f'数据端口 {self.ip}:{self.data_port} 已连接')
            return True
        except Exception as e:
            self.get_logger().error(f'数据端口连接失败: {e}')
            if self._data_sock:
                try:
                    self._data_sock.close()
                except Exception:
                    pass
                self._data_sock = None
            return False

    def _send_cmd(self, cmd: str):
        """通过命令端口发送 16 字节命令"""
        if not self._cmd_sock:
            return
        try:
            data = format_command(cmd)
            self._cmd_sock.sendall(data)
            self.get_logger().info(f'发送命令: {cmd} → {data}')
        except Exception as e:
            self.get_logger().warn(f'发送命令失败 ({cmd}): {e}')

    # ═══════════════════════════════════════════
    #  主循环
    # ═══════════════════════════════════════════

    def _main_loop(self):
        """主数据接收循环"""
        while self._running:
            # 连接命令端口 (可选)
            if not self._cmd_sock:
                self._connect_cmd()

            # 连接数据端口 (必须)
            if not self._data_sock:
                if not self._connect_data():
                    self.get_logger().warn(f'数据端口连接失败，{5}秒后重试...')
                    time.sleep(5.0)
                    continue

            self._connected = True
            self.get_logger().info('开始接收 PD6 数据...')

            # 数据接收循环
            buf = b''
            while self._running and self._data_sock:
                try:
                    ready, _, _ = select.select([self._data_sock], [], [], 2.0)
                    if not ready:
                        # 检查是否超时无数据
                        if self._last_data_time > 0 and \
                           time.time() - self._last_data_time > 10.0:
                            self.get_logger().warn('10秒无数据，可能连接断开')
                            break
                        continue

                    chunk = self._data_sock.recv(4096)
                    if not chunk:
                        self.get_logger().warn('数据端口连接已关闭 (recv=0)')
                        break

                    buf += chunk
                    self._last_data_time = time.time()

                    # 按行解析 PD6 数据
                    while b'\n' in buf:
                        line_bytes, buf = buf.split(b'\n', 1)
                        line = line_bytes.decode('ascii', errors='replace').strip()
                        if line:
                            self._parse_pd6_line(line)

                except socket.timeout:
                    continue
                except ConnectionResetError:
                    self.get_logger().warn('连接被重置')
                    break
                except Exception as e:
                    self.get_logger().warn(f'数据接收错误: {e}')
                    break

            # 清理断开连接
            self._connected = False
            if self._data_sock:
                try:
                    self._data_sock.close()
                except Exception:
                    pass
                self._data_sock = None
            if self._cmd_sock:
                try:
                    self._cmd_sock.close()
                except Exception:
                    pass
                self._cmd_sock = None

            if self._running:
                self.get_logger().info('等待 3 秒后重连...')
                time.sleep(3.0)

    # ═══════════════════════════════════════════
    #  PD6 协议解析
    # ═══════════════════════════════════════════

    def _parse_pd6_line(self, line: str):
        """解析一行 PD6 数据

        PD6 语句格式: :TAG,field1,field2,...
        以 : 开头，逗号分隔字段
        """
        if not line or line[0] != ':':
            return

        # 跳过配置确认包 (如 :PC 1500.00:)
        if line.endswith(':') and ',' not in line:
            return

        parts = line.split(',')
        tag = parts[0]

        try:
            if tag == ':SA':
                self._parse_sa(parts)
            elif tag == ':TS':
                self._parse_ts(parts)
            elif tag == ':BE':
                self._parse_be(parts)
            elif tag == ':BD':
                self._parse_bd(parts)
            elif tag == ':BI':
                self._parse_bi(parts)
            elif tag == ':BS':
                self._parse_bs(parts)
            # 其他标签忽略
        except Exception as e:
            self._parse_errors += 1
            if self._parse_errors <= 10:
                self.get_logger().warn(f'PD6 解析错误: {e} | line: {line[:100]}')

    def _safe_float(self, s: str, default: float = 0.0) -> float:
        """安全转换为浮点数"""
        s = s.strip()
        if not s or s == 'A' or s == 'V':
            return default
        try:
            return float(s)
        except ValueError:
            return default

    def _parse_sa(self, parts):
        """:SA,pitch,roll,heading,"""
        if len(parts) < 4:
            return
        self.latest['pitch'] = self._safe_float(parts[1])
        self.latest['roll'] = self._safe_float(parts[2])
        self.latest['heading'] = self._safe_float(parts[3])

    def _parse_ts(self, parts):
        """:TS,timestamp,salinity,temp,depth,sound_speed,status"""
        if len(parts) < 6:
            return
        self.latest['timestamp'] = parts[1].strip()
        self.latest['salinity'] = self._safe_float(parts[2])
        self.latest['temperature'] = self._safe_float(parts[3])
        self.latest['depth'] = self._safe_float(parts[4])
        self.latest['speed_of_sound'] = int(self._safe_float(parts[5], 1500))

    def _parse_be(self, parts):
        """:BE,E,N,U,status — 大地坐标系速度 (mm/s)"""
        if len(parts) < 5:
            return
        e_mm = self._safe_float(parts[1])
        n_mm = self._safe_float(parts[2])
        u_mm = self._safe_float(parts[3])
        status = parts[4].strip()

        # 只有 A (有效) 时才更新速度
        if status == 'A':
            self.latest['bottom_vel_east'] = e_mm * 0.001    # mm/s → m/s
            self.latest['bottom_vel_north'] = n_mm * 0.001
            self.latest['bottom_vel_up'] = u_mm * 0.001
            self.latest['bt_status'] = 'A'
        else:
            self.latest['bottom_vel_east'] = 0.0
            self.latest['bottom_vel_north'] = 0.0
            self.latest['bottom_vel_up'] = 0.0
            self.latest['bt_status'] = 'V'

        self._ensemble_count += 1
        self.latest['ensemble'] = self._ensemble_count

        if self._ensemble_count <= 5:
            self.get_logger().info(
                f':BE #{self._ensemble_count} E={e_mm}mm/s N={n_mm}mm/s '
                f'U={u_mm}mm/s status={status} → '
                f'({self.latest["bottom_vel_east"]:.3f},'
                f'{self.latest["bottom_vel_north"]:.3f},'
                f'{self.latest["bottom_vel_up"]:.3f})m/s')

    def _parse_bd(self, parts):
        """:BD,E_dist,N_dist,U_dist,altitude,valid_time — 大地坐标系距离"""
        if len(parts) < 5:
            return
        # parts[4] = 设备离底距离 (altitude)
        altitude = self._safe_float(parts[4])
        if altitude > 0:
            self.latest['altitude'] = altitude

    def _parse_bi(self, parts):
        """:BI,X,Y,Z,error,status — 设备坐标系速度 (mm/s)"""
        pass  # 不直接使用，:BE 提供大地坐标系速度

    def _parse_bs(self, parts):
        """:BS,X,Y,Z,status — 船体坐标系速度 (mm/s)"""
        pass  # 不直接使用，:BE 提供大地坐标系速度

    # ═══════════════════════════════════════════
    #  数据发布
    # ═══════════════════════════════════════════

    def _publish_data(self):
        """定时发布 ROS2 话题 (5Hz)"""
        if self._ensemble_count == 0:
            return

        # 底跟踪速度 E/N/U (m/s)
        vel_msg = Vector3()
        vel_msg.x = self.latest['bottom_vel_east']
        vel_msg.y = self.latest['bottom_vel_north']
        vel_msg.z = self.latest['bottom_vel_up']
        self.pub_bottom_vel.publish(vel_msg)

        # 距底高度
        alt_msg = Float32()
        alt_msg.data = self.latest['altitude']
        self.pub_altitude.publish(alt_msg)

        # 完整状态 JSON
        status = {
            'ensemble': self.latest['ensemble'],
            'altitude': round(self.latest['altitude'], 3),
            'depth': round(self.latest['depth'], 3),
            'bottom_vel': {
                'east':  round(self.latest['bottom_vel_east'], 4),
                'north': round(self.latest['bottom_vel_north'], 4),
                'up':    round(self.latest['bottom_vel_up'], 4),
            },
            'attitude': {
                'heading': round(self.latest['heading'], 2),
                'pitch':   round(self.latest['pitch'], 2),
                'roll':    round(self.latest['roll'], 2),
            },
            'temperature': round(self.latest['temperature'], 2),
            'salinity': round(self.latest['salinity'], 1),
            'speed_of_sound': self.latest['speed_of_sound'],
            'bt_status': self.latest['bt_status'],
            'bt_amplitude': [0, 0, 0, 0],
            'bt_percent_good': [0, 0, 0, 0],
            'bt_corr': [0, 0, 0, 0],
            'coord_system': self.latest['coord_system'],
            'num_beams': 4,
            'ensembles': self._ensemble_count,
            'parse_errors': self._parse_errors,
            'connection_mode': 'tcp',
            'connected': self._connected,
            'device': 'H1000',
            'timestamp': self.latest['timestamp'],
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    # ═══════════════════════════════════════════
    #  连接监控
    # ═══════════════════════════════════════════

    def _check_connection(self):
        """定期检查连接状态"""
        if not self._connected:
            self.get_logger().warn('DVL 未连接，等待重连...')
            return

        # 检查数据超时
        if self._last_data_time > 0:
            elapsed = time.time() - self._last_data_time
            if elapsed > 15.0:
                self.get_logger().warn(f'已 {elapsed:.0f}s 未收到 DVL 数据')
        elif self._ensemble_count == 0 and self._connected:
            self.get_logger().info('已连接但尚未收到数据，等待 PD6 输出...')

    # ═══════════════════════════════════════════
    #  生命周期
    # ═══════════════════════════════════════════

    def shutdown(self):
        self._running = False
        # 尝试发送停止命令
        if self._cmd_sock:
            try:
                self._send_cmd('CZ')
            except Exception:
                pass
        # 关闭 socket
        for s in [self._cmd_sock, self._data_sock]:
            if s:
                try:
                    s.close()
                except Exception:
                    pass
        self._cmd_sock = None
        self._data_sock = None
        self.get_logger().info('H1000 DVL Driver 已关闭')

    def destroy_node(self):
        self.shutdown()
        super().destroy_node()


# ═══════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════

def main(args=None):
    parser = argparse.ArgumentParser(description='H1000 DVL Driver')
    parser.add_argument('--ip', type=str, default='192.168.0.11',
                        help='DVL IP 地址 (默认 192.168.0.11)')
    parser.add_argument('--data-port', type=int, default=10001,
                        help='数据端口 (默认 10001)')
    parser.add_argument('--cmd-port', type=int, default=10000,
                        help='命令端口 (默认 10000)')
    parsed, _ = parser.parse_known_args()

    print(f'H1000 DVL Driver — IP: {parsed.ip}')
    print(f'  数据端口: {parsed.data_port}, 命令端口: {parsed.cmd_port}')
    sys.stdout.flush()

    rclpy.init(args=args)
    node = H1000DvlDriver(
        ip=parsed.ip,
        data_port=parsed.data_port,
        cmd_port=parsed.cmd_port,
    )

    def sig_handler(sig, frame):
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
