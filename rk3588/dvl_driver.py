#!/usr/bin/env python3
"""
PathFinder DVL (Doppler Velocity Log) Driver — 运行于 RK3588
支持两种连接方式:
  1. TCP Server 模式: 监听 0.0.0.0:1034，DVL 主动连接 (DVL Web 配置 PD0 -> TCP -> 192.168.0.99:1034)
  2. 串口模式: --serial /dev/ttyS0 (RS232，波特率可配)

PD0 二进制协议解析 + ROS2 话题发布

发布话题:
  /rov/dvl/bottom_vel  (Vector3) 底跟踪速度: E/N/U (m/s)
  /rov/dvl/altitude    (Float32) 距底高度 (m)
  /rov/dvl/status      (String)  完整状态 JSON

用法:
  python3 dvl_driver.py                          # 默认 TCP Server 模式
  python3 dvl_driver.py --serial /dev/ttyS0      # 串口模式
  python3 dvl_driver.py --serial /dev/ttyS9 --baud 115200
  python3 dvl_driver.py --tcp 192.168.0.6        # 使用旧版 TCP 连接模式
"""

import os
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

import socket
import struct
import json
import time
import threading
import signal
import sys
import argparse
import termios
import fcntl
import array

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
import math

# ── PD0 数据段 ID ────────────────────────────
PID_FIXED_LEADER    = 0x0000
PID_VARIABLE_LEADER = 0x0080
PID_VELOCITY        = 0x0100
PID_CORRELATION     = 0x0200
PID_ECHO_INTENSITY  = 0x0300
PID_PERCENT_GOOD    = 0x0400
PID_STATUS          = 0x0500
PID_BOTTOM_TRACK    = 0x0600


# ═══════════════════════════════════════════
#  内建串口类 (无需 pyserial，纯 Python termios 实现)
# ═══════════════════════════════════════════

class SerialPort:
    """纯 Python 串口通信 (不依赖 pyserial)

    使用 termios + fcntl + select 实现。
    参考 PySerial 源码，但裁剪到只包含所需功能。
    """

    def __init__(self, port, baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._fd = None
        self._is_open = False

    def open(self):
        if self._is_open:
            return
        self._fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self._is_open = True
        self._reconfigure_port()
        # 恢复阻塞模式
        flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    def _reconfigure_port(self):
        """设置串口属性: 8N1, raw mode"""
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self._fd)

        # Raw mode
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                   | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB)
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL

        # 硬件流控关闭
        cflag &= ~termios.CRTSCTS

        # 波特率
        baud_const = getattr(termios, f'B{self.baudrate}', termios.B115200)
        ispeed = baud_const
        ospeed = baud_const

        # 超时 (VMIN=0, VTIME=timeout*10 决定读取阻塞时间)
        cc[termios.VMIN] = 1
        cc[termios.VTIME] = max(0, min(255, int(self.timeout * 10)))

        termios.tcsetattr(self._fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

    def send_break(self, duration=0.3):
        """发送 break 信号"""
        termios.tcsendbreak(self._fd, int(duration * 4))
        time.sleep(duration)

    def write(self, data):
        if isinstance(data, str):
            data = data.encode('ascii', errors='replace')
        return os.write(self._fd, data)

    def read(self, size=1):
        """读取最多 size 字节 (带超时)"""
        try:
            return os.read(self._fd, size)
        except BlockingIOError:
            return b''

    def read_all(self):
        """读取所有可用数据"""
        buf = b''
        # 设置非阻塞模式
        flags = fcntl.fcntl(self._fd, fcntl.F_GETFL)
        fcntl.fcntl(self._fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        try:
            while True:
                chunk = os.read(self._fd, 4096)
                if not chunk:
                    break
                buf += chunk
        except BlockingIOError:
            pass
        finally:
            fcntl.fcntl(self._fd, fcntl.F_SETFL, flags)
        return buf

    def close(self):
        if self._fd is not None:
            try:
                os.close(self._fd)
            except Exception:
                pass
            self._fd = None
            self._is_open = False

    def flush_input(self):
        """丢弃输入缓冲区"""
        try:
            termios.tcflush(self._fd, termios.TCIFLUSH)
        except Exception:
            pass

    @property
    def is_open(self):
        return self._is_open


# ═══════════════════════════════════════════
#  DVL Driver Node
# ═══════════════════════════════════════════

class DvlDriver(Node):
    """DVL PD0 协议解析与 ROS2 发布

    支持两种连接方式:
      - TCP:   _cmd_sock (1033) + _data_sock (1034)
      - Serial: _serial 双向通信 (命令 + 数据同端口)
    """

    def __init__(self, mode='tcp', tcp_ip='192.168.0.6',
                 serial_port='/dev/ttyS0', serial_baud=115200):
        super().__init__('dvl_driver')

        self.mode = mode
        self.tcp_ip = tcp_ip
        self.serial_port = serial_port
        self.serial_baud = serial_baud

        # ── 发布者 ──
        self.pub_bottom_vel = self.create_publisher(Vector3, '/rov/dvl/bottom_vel', 10)
        self.pub_altitude   = self.create_publisher(Float32, '/rov/dvl/altitude', 10)
        self.pub_status     = self.create_publisher(String,  '/rov/dvl/status', 10)

        # ── 状态变量 ──
        self._running = True
        self._cmd_sock = None     # TCP 命令 socket
        self._data_sock = None    # TCP 数据 socket
        self._serial = None       # Serial 连接对象
        self._ensemble_count = 0
        self._last_cmd_keepalive = 0.0
        self._parse_errors = 0
        self._peek_data = b''  # 初始化时 peek 到的数据

        # 最新解析数据
        self.latest = {
            'ensemble': 0,
            'bottom_vel_east': 0.0,
            'bottom_vel_north': 0.0,
            'bottom_vel_up': 0.0,
            'altitude': 0.0,
            'depth': 0.0,
            'heading': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'temperature': 0.0,
            'salinity': 0.0,
            'speed_of_sound': 0.0,
            'bt_range': [0, 0, 0, 0],
            'bt_vel': [0, 0, 0, 0],
            'bt_amplitude': [0, 0, 0, 0],
            'bt_corr': [0, 0, 0, 0],
            'bt_percent_good': [0, 0, 0, 0],
            'bt_status': 'N',
            'coord_system': 'UNKNOWN',
            'num_beams': 0,
        }

        # ── 启动连接线程 ──
        self._conn_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._conn_thread.start()

        # ── 发布定时器 (5Hz) ──
        self._pub_timer = self.create_timer(0.2, self._publish_data)

        if self.mode == 'tcp':
            self.get_logger().info(f'DVL Driver TCP模式 -> {self.tcp_ip}:1033/1034')
        else:
            self.get_logger().info(f'DVL Driver 串口模式 -> {self.serial_port} @ {self.serial_baud}')

    # ═══════════════════════════════════════════
    #  连接管理
    # ═══════════════════════════════════════════

    def _configure_dvl_tcp(self):
        """TCP 模式：不通过 TCP 发送配置命令，避免污染 PD0 数据流。

        坐标系配置通过 DVL Web 界面或现场串口终端完成。
        驱动使用 BEAM->ENU 转换确保 /rov/dvl/bottom_vel 始终输出 E/N/U。
        """
        self.get_logger().info(
            'DVL 坐标系配置：请通过 Web 界面或串口终端设置。'
            '驱动将使用 BEAM->ENU 转换作为后备。')
        # 注意：不要通过数据端口 1034 发送 ASCII 命令，会污染 PD0 数据流。
        # 命令端口 1033 在 USR-TCP232 单端口转发模式下通常不可用。

    def _configure_dvl_serial(self):
        """串口模式：不通过串口发送配置命令，避免进入命令模式。

        坐标系配置通过 DVL Web 界面或现场串口终端完成。
        """
        self.get_logger().info(
            'DVL 坐标系配置：请通过 Web 界面或串口终端设置。'
            '驱动将使用 BEAM->ENU 转换作为后备。')
    def _connection_loop(self):
        """主连接循环"""
        RECONNECT_DELAY = 3.0
        while self._running:
            try:
                if self.mode == 'tcp':
                    self._connect_tcp_client()
                else:
                    self._connect_serial()
            except Exception as e:
                self.get_logger().warn(f'连接异常: {e}，{RECONNECT_DELAY}s 后重连...')
            if self._running:
                time.sleep(RECONNECT_DELAY)

    # ── TCP 模式 ──

    def _connect_tcp_client(self):
        """TCP Client 模式: 连接 DVL 192.168.0.6 端口 1034 接收 PD0 二进制数据。

        DVL Web 配置: PD0 -> TCP -> 192.168.0.6:1034 (DVL 自身 TCP Server)
        代码作为 TCP 客户端连接读取，不发任何命令。
        """
        host = self.tcp_ip  # 192.168.0.6
        self.get_logger().info(f'TCP Client: 连接 DVL {host}:1034 读取 PD0 数据...')

        self._data_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self._data_sock.settimeout(1.0)

        try:
            self._data_sock.connect((host, 1034))
            self.get_logger().info(f'已连接 DVL {host}:1034，开始读取 PD0 数据...')
            self._data_loop_tcp_client()
        except ConnectionRefusedError:
            self.get_logger().warn(f'DVL {host}:1034 拒绝连接（可能未启动采集或端口未就绪）')
        except socket.timeout:
            self.get_logger().warn(f'DVL {host}:1034 连接超时')
        except Exception as e:
            self.get_logger().warn(f'DVL {host}:1034 连接失败: {e}')
        finally:
            if self._data_sock:
                try:
                    self._data_sock.close()
                except:
                    pass
                self._data_sock = None

    def _data_loop_tcp_client(self):
        """TCP Client 数据读取循环"""
        buf = b''
        self._parse_errors = 0
        msg_count = 0

        while self._running:
            try:
                chunk = self._data_sock.recv(65536)
                if not chunk:
                    self.get_logger().info('DVL 关闭了连接')
                    break
                buf += chunk
                msg_count += 1
                if msg_count == 1:
                    self.get_logger().info(
                        f'收到首个 TCP 数据 ({len(chunk)}B)')
            except socket.timeout:
                if buf:
                    buf = self._parse_buffer(buf)
                continue
            except ConnectionResetError:
                self.get_logger().warn('DVL 重置了连接')
                break
            except Exception as e:
                self.get_logger().warn(f'TCP 读取错误: {e}')
                break

            buf = self._parse_buffer(buf)

        if self._data_sock:
            try:
                self._data_sock.close()
            except:
                pass
        self._data_sock = None

    # ── 串口模式 ──

    def _connect_serial(self):
        """串口连接: 使用 break 唤醒 DVL，通过同一串口进行命令+数据通信"""
        self.get_logger().info(f'正在打开串口 {self.serial_port} @ {self.serial_baud}...')
        self._serial = SerialPort(self.serial_port, self.serial_baud, timeout=1.0)
        self._serial.open()
        self._serial.flush_input()
        self.get_logger().info('串口已打开')

        # 1. 发送 break 唤醒
        self.get_logger().info('发送 break 信号...')
        self._serial.send_break(0.3)
        time.sleep(0.5)

        # 读取唤醒响应
        resp = self._serial.read_all()
        if resp:
            self.get_logger().info(f'Break 响应 ({len(resp)}B): {resp.decode(errors="replace")[:200]}')
        else:
            self.get_logger().warn('Break 后无响应，继续尝试...')

        # 2. 发送 CR1 → BT3/WV3 → CS 命令序列
        time.sleep(0.3)
        self._send_cmd_serial('CR1\r\n')
        time.sleep(1.0)

        # 配置坐标系为 EARTH（在 CS 之前发送）
        self._configure_dvl_serial()

        time.sleep(0.5)
        self._send_cmd_serial('CS\r\n')
        time.sleep(0.3)

        # 3. 读取 PD0 数据
        self.get_logger().info('开始读取 PD0 数据...')
        self._data_loop_serial()

    def _send_cmd_serial(self, cmd):
        """通过串口发送命令并读取回复"""
        try:
            self._serial.flush_input()
            self._serial.write(cmd)
            time.sleep(0.3)
            resp = self._serial.read_all()
            if resp:
                self.get_logger().info(f'CMD {cmd.strip()}: {resp.decode(errors="replace").strip()[:200]}')
            else:
                self.get_logger().debug(f'CMD {cmd.strip()}: (无应答)')
        except Exception as e:
            self.get_logger().warn(f'CMD {cmd.strip()} 失败: {e}')

    def _data_loop_serial(self):
        """串口 PD0 数据读取循环"""
        CMD_KEEPALIVE = 60.0  # 串口不需要频繁心跳
        buf = b''
        self._last_cmd_keepalive = time.time()
        self._parse_errors = 0

        while self._running:
            try:
                chunk = self._serial.read(4096)
                if chunk:
                    buf += chunk
            except Exception as e:
                self.get_logger().warn(f'串口读取错误: {e}')
                break

            # 超时后尝试解析
            buf = self._parse_buffer(buf)

            # 偶尔检查心跳
            now = time.time()
            if now - self._last_cmd_keepalive > CMD_KEEPALIVE:
                try:
                    self._serial.write(b'\r\n')
                except Exception:
                    self.get_logger().warn('串口写入失败，将重连')
                    break
                self._last_cmd_keepalive = now

        # 清理
        if self._serial:
            self._serial.close()
            self._serial = None

    # ═══════════════════════════════════════════
    #  PD0 协议解析 (TCP 和 Serial 共用)
    # ═══════════════════════════════════════════

    def _parse_buffer(self, buf):
        """在缓冲区中查找并解析 PD0 帧"""
        while len(buf) >= 6:
            idx = buf.find(b'\x7f\x7f')
            if idx < 0:
                keep = min(len(buf), 5)
                return buf[-keep:] if keep > 0 else b''

            if idx > 0:
                buf = buf[idx:]

            if len(buf) < 6:
                return buf

            # Header: 7F 7F [bytes_LSB bytes_MSB] [spare] [num_types]
            _, _, n_bytes = struct.unpack_from('<BBH', buf, 0)
            n_types = buf[5]

            header_size = 6 + n_types * 2
            total_size = n_bytes + 2  # +2 for checksum

            if total_size < header_size + 4:
                self._parse_errors += 1
                buf = buf[2:]  # 跳过第一个 0x7F
                continue

            if len(buf) < total_size:
                return buf

            ensemble_raw = buf[:total_size]

            try:
                self._parse_ensemble(ensemble_raw, header_size, n_types, n_bytes)
                self._ensemble_count += 1
            except Exception as e:
                self._parse_errors += 1
                if self._parse_errors <= 5:
                    self.get_logger().warn(f'PD0 解析错误 (共{self._parse_errors}): {e}')

            buf = buf[total_size:]

        return buf

    def _parse_ensemble(self, data, header_size, n_types, n_bytes):
        """解析一个完整的 PD0 ensemble"""
        offsets = []
        for i in range(n_types):
            if 6 + i * 2 + 2 <= len(data):
                off = struct.unpack_from('<H', data, 6 + i * 2)[0]
                offsets.append(off)

        bt_data = fl_data = vl_data = None

        for off in offsets:
            if off + 2 > len(data):
                continue
            data_id = struct.unpack_from('<H', data, off)[0]

            if data_id == PID_BOTTOM_TRACK:
                bt_data = data[off:]
            elif data_id == PID_VARIABLE_LEADER:
                vl_data = data[off:]
            elif data_id == PID_FIXED_LEADER:
                fl_data = data[off:]

        # 调试日志（前3个ensemble）
        if self._ensemble_count < 3:
            ids = []
            for off in offsets:
                if off + 2 <= len(data):
                    ids.append(f'0x{struct.unpack_from("<H", data, off)[0]:04X}@{off}')
            self.get_logger().info(
                f'PD0 ensemble #{self._ensemble_count}: offsets={offsets} ids={ids} '
                f'FL={"Y" if fl_data else "N"} VL={"Y" if vl_data else "N"} BT={"Y" if bt_data else "N"}')

        if fl_data and len(fl_data) >= 60:
            self._parse_fixed_leader(fl_data)
        if vl_data and len(vl_data) >= 40:
            self._parse_variable_leader(vl_data)
        if bt_data and len(bt_data) >= 80:
            self._parse_bottom_track(bt_data)

    def _parse_fixed_leader(self, data):
        if len(data) >= 30:
            ex_map = {0x00: 'BEAM', 0x01: 'INST', 0x02: 'SHIP', 0x03: 'EARTH'}
            coord_byte = data[26] & 0x03
            self.latest['coord_system'] = ex_map.get(coord_byte, f'UNKNOWN(0x{coord_byte:02X})')
            # 警告：非 EARTH 坐标系时速度不是 E/N/U
            if self.latest['coord_system'] != 'EARTH' and self._ensemble_count < 20:
                self.get_logger().warn(
                    f'DVL 坐标系 = {self.latest["coord_system"]} (非 EARTH!) '
                    f'速度值为波束速度，非 E/N/U。请通过 Web 界面或 BT3/WV3 命令配置。')
        if len(data) >= 11:
            self.latest['num_beams'] = data[10]
        # 读取波束角度 (Fixed Leader bytes 28-29, 1/100 deg)
        if len(data) >= 30:
            raw_angle = struct.unpack_from('<h', data, 28)[0]
            # 0 or -32768 means angle not available, default to 30 deg
            if raw_angle not in (-32768, 0):
                self.latest['beam_angle_deg'] = raw_angle * 0.01
            else:
                self.latest['beam_angle_deg'] = 30.0
        else:
            self.latest['beam_angle_deg'] = 30.0  # 默认 30°

    def _parse_variable_leader(self, data):
        if len(data) < 50:
            return
        # PathFinder VL: 2-byte fields, little-endian
        self.latest['ensemble'] = struct.unpack_from('<H', data, 2)[0]
        if len(data) >= 16:
            self.latest['speed_of_sound'] = struct.unpack_from('<H', data, 14)[0]
        if len(data) >= 18:
            depth_dm = struct.unpack_from('<H', data, 16)[0]
            self.latest['depth'] = depth_dm * 0.1  # dm -> m
        if len(data) >= 20:
            self.latest['heading'] = struct.unpack_from('<h', data, 18)[0] * 0.01
        if len(data) >= 22:
            self.latest['pitch'] = struct.unpack_from('<h', data, 20)[0] * 0.01
        if len(data) >= 24:
            self.latest['roll'] = struct.unpack_from('<h', data, 22)[0] * 0.01
        if len(data) >= 26:
            self.latest['salinity'] = struct.unpack_from('<H', data, 24)[0]
        if len(data) >= 28:
            self.latest['temperature'] = struct.unpack_from('<h', data, 26)[0] * 0.01

    def _parse_bottom_track(self, data):
        if len(data) < 50:
            return
        # PathFinder BT: 2-byte velocities at offset 10, 2-byte ranges at offset 18
        for i in range(4):
            self.latest['bt_vel'][i] = struct.unpack_from('<h', data, 10 + i * 2)[0]
            self.latest['bt_range'][i] = struct.unpack_from('<H', data, 18 + i * 2)[0]
        # 65535 = invalid range
        valid = [r for r in self.latest['bt_range'] if 0 < r < 65535]
        self.latest['altitude'] = (sum(valid) / len(valid) * 0.01) if valid else 0.0
        # ── 速度解析：根据坐标系决定是否转换 ───────────
        # PD0 底跟踪速度单位：mm/s (有符号 16 位，0.001 m/s)
        # coord_system 来自 Fixed Leader byte 26
        coord = self.latest.get('coord_system', 'BEAM')
        bt_vel_mm = self.latest['bt_vel']  # 4 个波束速度 (mm/s)

        if coord == 'BEAM':
            # ═══════════════════════════════════════════
            # BEAM → ENU 转换 (Janus 4波束标准公式)
            # ═══════════════════════════════════════════
            # 波束角 (来自 Fixed Leader，默认 30°)
            beam_deg = self.latest.get('beam_angle_deg', 30.0)
            beam_rad = math.radians(beam_deg)
            sin_th = math.sin(beam_rad)
            cos_th = math.cos(beam_rad)

            v = bt_vel_mm  # [v1, v2, v3, v4] mm/s

            # BEAM → 仪器坐标系 (机体坐标系)
            # 标准 Janus 配置: 波束1(+X) 波束2(-X) 波束3(+Y) 波束4(-Y)
            Vx_inst = (v[0] - v[1]) / (2.0 * sin_th)   # mm/s
            Vy_inst = (v[2] - v[3]) / (2.0 * sin_th)   # mm/s
            Vz_inst = (v[0] + v[1] + v[2] + v[3]) / (4.0 * cos_th)  # mm/s

            # 仪器 → 大地坐标系 (需要航向角)
            hdg = math.radians(self.latest.get('heading', 0.0))
            cos_h = math.cos(hdg)
            sin_h = math.sin(hdg)

            Ve_mm = Vx_inst * cos_h - Vy_inst * sin_h   # mm/s 东向
            Vn_mm = Vx_inst * sin_h + Vy_inst * cos_h   # mm/s 北向
            Vu_mm = -Vz_inst                                 # mm/s 天向 (仪器z向下)

            self.latest['bottom_vel_east']  = Ve_mm * 0.001
            self.latest['bottom_vel_north'] = Vn_mm * 0.001
            self.latest['bottom_vel_up']    = Vu_mm * 0.001

            if self._ensemble_count < 10:
                self.get_logger().info(
                    f'BEAM→ENU 转换: beam=({v[0]},{v[1]},{v[2]},{v[3]})mm/s '
                    f'→ ENU=({Ve_mm*0.001:.3f},{Vn_mm*0.001:.3f},{Vu_mm*0.001:.3f})m/s '
                    f'(hdg={self.latest.get("heading",0):.1f}° beam_angle={beam_deg:.1f}°)')

        elif coord == 'EARTH':
            # 已是大地坐标系，直接转换单位
            self.latest['bottom_vel_east']  = bt_vel_mm[0] * 0.001
            self.latest['bottom_vel_north'] = bt_vel_mm[1] * 0.001
            self.latest['bottom_vel_up']    = bt_vel_mm[2] * 0.001

        else:
            # INST/SHIP 坐标系：无法直接解释为 E/N/U，做 BEAM→ENU 近似
            self.get_logger().warn(
                f'坐标系 {coord} 无法直接输出 E/N/U，做 BEAM→ENU 近似')
            # 复用 BEAM 转换逻辑
            beam_deg = self.latest.get('beam_angle_deg', 30.0)
            beam_rad = math.radians(beam_deg)
            sin_th = math.sin(beam_rad)
            v = bt_vel_mm
            Vx_inst = (v[0] - v[1]) / (2.0 * sin_th)
            Vy_inst = (v[2] - v[3]) / (2.0 * sin_th)
            Vz_inst = (v[0] + v[1] + v[2] + v[3]) / (4.0 * cos_th)
            hdg = math.radians(self.latest.get('heading', 0.0))
            Ve_mm = (Vx_inst * math.cos(hdg) - Vy_inst * math.sin(hdg))
            Vn_mm = (Vx_inst * math.sin(hdg) + Vy_inst * math.cos(hdg))
            Vu_mm = -Vz_inst
            self.latest['bottom_vel_east']  = Ve_mm * 0.001
            self.latest['bottom_vel_north'] = Vn_mm * 0.001
            self.latest['bottom_vel_up']    = Vu_mm * 0.001
        # PathFinder PD0 Bottom Track 字段偏移 (标准 PD0 兼容)
        # offset 26-29: Amplitude (回波幅度, dB)
        # offset 30-33: Correlation (相关性, 0-255)
        # offset 38-41: Percent Good (有效百分比, 0-100)
        for i in range(4):
            self.latest['bt_amplitude'][i] = data[26 + i] if len(data) > 26 + i else 0
            self.latest['bt_corr'][i] = data[30 + i] if len(data) > 30 + i else 0
            self.latest['bt_percent_good'][i] = data[38 + i] if len(data) > 38 + i else 0
        # 用 %Good 判断有效波束 (>%20 视为有效)
        good_beams = sum(1 for g in self.latest['bt_percent_good'] if g > 20)
        self.latest['bt_status'] = 'A' if good_beams >= 3 else 'V'
        # 浅水/无回波时速度置零，避免噪声
        if self.latest['bt_status'] == 'V':
            self.latest['bottom_vel_east'] = 0.0
            self.latest['bottom_vel_north'] = 0.0
            self.latest['bottom_vel_up'] = 0.0

    # ═══════════════════════════════════════════
    #  数据发布
    # ═══════════════════════════════════════════

    def _publish_data(self):
        vel_msg = Vector3()
        vel_msg.x = self.latest['bottom_vel_east']
        vel_msg.y = self.latest['bottom_vel_north']
        vel_msg.z = self.latest['bottom_vel_up']
        self.pub_bottom_vel.publish(vel_msg)

        alt_msg = Float32()
        alt_msg.data = self.latest['altitude']
        self.pub_altitude.publish(alt_msg)

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
            'bt_amplitude': self.latest['bt_amplitude'],
            'bt_percent_good': self.latest['bt_percent_good'],
            'bt_corr': self.latest['bt_corr'],
            'coord_system': self.latest['coord_system'],
            'num_beams': self.latest['num_beams'],
            'ensembles': self._ensemble_count,
            'parse_errors': self._parse_errors,
            'connection_mode': self.mode,
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    # ═══════════════════════════════════════════
    #  生命周期
    # ═══════════════════════════════════════════

    def shutdown(self):
        self._running = False
        if self.mode == 'tcp':
            for s in [self._cmd_sock, self._data_sock]:
                if s:
                    try: s.close()
                    except Exception: pass
        else:
            if self._serial:
                self._serial.close()
                self._serial = None
        self.get_logger().info('DVL Driver 已关闭')

    def destroy_node(self):
        self.shutdown()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='PathFinder DVL Driver')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--serial', type=str, metavar='PORT',
                       help='串口模式，指定端口 (如 /dev/ttyS0)')
    group.add_argument('--tcp', type=str, metavar='IP', default=None,
                       help='TCP 模式，指定 IP (默认 192.168.0.6)')
    parser.add_argument('--baud', type=int, default=115200,
                        help='串口波特率 (默认 115200)')
    parsed, _ = parser.parse_known_args()

    if parsed.serial:
        mode = 'serial'
        serial_port = parsed.serial
        tcp_ip = None
        serial_baud = parsed.baud
    else:
        mode = 'tcp'
        tcp_ip = parsed.tcp or '192.168.0.6'
        serial_port = None
        serial_baud = parsed.baud

    print(f'PathFinder DVL Driver — 模式: {mode}')
    if mode == 'tcp':
        print(f'  TCP: {tcp_ip}:1033/1034')
    else:
        print(f'  Serial: {serial_port} @ {serial_baud}')
    sys.stdout.flush()

    rclpy.init(args=args)
    node = DvlDriver(
        mode=mode,
        tcp_ip=tcp_ip,
        serial_port=serial_port,
        serial_baud=serial_baud,
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
