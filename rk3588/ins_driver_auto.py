#!/usr/bin/env python3
"""
RK3588 INS 自动驱动 — 纯 Python 实现
根据 INS400 使用说明 V6 和 202帧定义文档重写。

操作流程（源自 INS400 使用说明 第6节）：
  1. INS 上电后自动进入"监控状态"（alignment=0）
  2. 输入地理纬度（0x4C 命令，无GPS时必须）
  3. 发送启动命令（0x47，仅一次）
  4. 保持静止 → 静态粗对准(1) → 精对准(2) → INS导航(3)
  5. 导航模式后可发送停止命令回到监控状态

话题发布（ROS_DOMAIN_ID=42）：
  /ins/attitude     geometry_msgs/Vector3  (x=pitch, y=roll, z=yaw)  deg
  /ins/velocity     geometry_msgs/Vector3  (x=ve, y=vn, z=vd)   m/s
  /ins/position     geometry_msgs/Vector3  (x=lat, y=lon, z=alt)
  /ins/acceleration geometry_msgs/Vector3  (x=ax, y=ay, z=az)  m/s²
  /ins/angular_rate geometry_msgs/Vector3  (x=wx, y=wy, z=wz)  deg/s
  /ins/status       std_msgs/String        (JSON 完整状态)
  /ins/alignment    std_msgs/Int8          (0=监控 1=粗对准 2=精对准 3=导航)

用法:
  python3 ins_driver_auto.py [--lat 39.9 --lon 116.4 --alt 50.0]
"""

import os, sys, time, socket, struct, json, threading, argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Vector3

# ══════════════════  INS 网络配置 ══════════════════
INS_IP = "192.168.0.7"
INS_CMD_PORT = 8007      # INS 命令端口（发往 INS）
INS_DATA_PORT = 8008     # INS 数据端口（从 INS 接收）

# ══════════════════  默认位置（深圳/广东，无GPS时需手动输入）══════════════
DEFAULT_LAT = 22.73      # 纬度 °N
DEFAULT_LON = 113.54     # 经度 °E
DEFAULT_ALT = 50.0       # 海拔 m

# ══════════════════  对准状态 ══════════════════
ALIGNMENT_TEXT = {
    0: "监控状态",
    1: "粗对准",
    2: "精对准",
    3: "INS导航模式"
}


def xor_checksum(data: bytes) -> int:
    """计算字节数组的异或校验"""
    result = 0
    for b in data:
        result ^= b
    return result


def build_cmd_9byte(cmd_type: int, payload: bytes) -> bytes:
    """
    构建 9 字节控制命令帧
    ┌──────┬──────┬──────┬────────────┬──────┬──────┐
    │ 0x5A │ 0xA5 │ type │ payload(4) │ XOR  │ 0x55 │
    └──────┴──────┴──────┴────────────┴──────┴──────┘
    """
    assert len(payload) == 4, f"payload must be 4 bytes, got {len(payload)}"
    body = bytes([cmd_type]) + payload
    checksum = xor_checksum(body)
    return b'\x5A\xA5' + body + bytes([checksum, 0x55])


def build_lat_cmd(lat_deg: float) -> bytes:
    """构建纬度输入命令 (0x4C) — 表3.1.1"""
    return build_cmd_9byte(0x4C, struct.pack('<f', lat_deg))


def build_lon_cmd(lon_deg: float) -> bytes:
    """构建经度输入命令 (0x54) — 表3.3.1"""
    return build_cmd_9byte(0x54, struct.pack('<f', lon_deg))


def build_alt_cmd(alt_m: float) -> bytes:
    """构建海拔输入命令 (0x45) — 表3.5.1"""
    return build_cmd_9byte(0x45, struct.pack('<f', alt_m))


def build_start_cmd(position_valid=True, attitude_valid=True):
    """
    构建启动命令 (0x47) — 表3.10.1
    byte[3]: 0=停止, 1=启动
    byte[4]: 0=位置无效, 1=位置有效（需先发送纬度/经度/海拔）
    byte[5]: 0=姿态无效, 1=方位有效, 2=方位+姿态有效
    byte[6]: 保留 0x00
    """
    start_stop = 0x01  # 启动
    pos_valid  = 0x01 if position_valid else 0x00
    att_valid  = 0x02 if attitude_valid else 0x00
    reserved   = 0x00
    body = bytes([0x47, start_stop, pos_valid, att_valid, reserved])
    checksum = xor_checksum(body)
    return b'\x5A\xA5' + body + bytes([checksum, 0x55])


def build_stop_cmd():
    """构建停止命令"""
    body = bytes([0x47, 0x00, 0x01, 0x00, 0x00])
    checksum = xor_checksum(body)
    return b'\x5A\xA5' + body + bytes([checksum, 0x55])


class INSAutoDriver(Node):
    """INS 自动驱动节点 — 遵循 INS400 协议：输入位置→启动→等对准→导航"""

    def __init__(self, lat: float, lon: float, alt: float):
        super().__init__('ins_auto_driver')

        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt

        # ── 发布者 ──
        self.pub_attitude  = self.create_publisher(Vector3, '/ins/attitude', 10)
        self.pub_velocity  = self.create_publisher(Vector3, '/ins/velocity', 10)
        self.pub_position  = self.create_publisher(Vector3, '/ins/position', 10)
        self.pub_accel     = self.create_publisher(Vector3, '/ins/acceleration', 10)
        self.pub_gyro      = self.create_publisher(Vector3, '/ins/angular_rate', 10)
        self.pub_status    = self.create_publisher(String,   '/ins/status',   10)
        self.pub_alignment = self.create_publisher(Int8,     '/ins/alignment', 10)

        # ── 内部状态 ──
        self.data = {
            'alignment': 0, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
            've': 0.0, 'vn': 0.0, 'vd': 0.0,
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
            'acc_h': 0.0, 'acc_v': 0.0,
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'sats': 0, 'temp': 0, 'hdop': 0.0, 'fix_type': 0,
            'comb_status': 0, 'frame_count': 0
        }
        self._last_alignment = -1    # 追踪对准状态变化
        self._startup_done = False   # 启动流程是否完成
        self._step = 0               # 启动步骤
        self._step_start_time = 0.0

        self.ins_socket = None
        self.running = False
        self.ins_thread = None
        self._last_recv_time = 0.0
        self._first_frame_time = 0.0

        self.get_logger().info('═' * 55)
        self.get_logger().info('  INS400 自动驱动 v2 (基于 INS400 使用说明 V6)')
        self.get_logger().info(f'  INS IP: {INS_IP}:{INS_DATA_PORT}')
        self.get_logger().info(f'  参考位置: lat={lat:.4f} lon={lon:.4f} alt={alt:.1f}m')
        self.get_logger().info(f'  话题前缀: /ins/attitude velocity position alignment status')
        self.get_logger().info('═' * 55)

        # ── 延迟 2s 后启动连接流程 ──
        self._startup_timer = self.create_timer(2.0, self._init_sequence)

    # ── 初始化序列 ────────────────────────────
    def _init_sequence(self):
        """初始化序列（仅执行一次）"""
        self._startup_timer.cancel()
        self._connect_ins()
        if not self.running:
            self.get_logger().error('UDP 绑定失败，初始化中止')
            return
        # 步骤 1: 输入参考位置（0.5s 间隔，避免 INS 指令缓冲区溢出）
        self._step = 1
        self._step_start_time = time.time()
        self._step_timer = self.create_timer(0.6, self._run_steps)

    def _run_steps(self):
        """分步执行初始化序列"""
        elapsed = time.time() - self._step_start_time

        if self._step == 1 and elapsed >= 1.0:
            self.get_logger().info(
                f'[步骤 1/4] 输入参考纬度 {self.ref_lat}° (0x4C) ...')
            cmd = build_lat_cmd(self.ref_lat)
            self._send_cmd(cmd)
            self._step = 2

        elif self._step == 2 and elapsed >= 1.8:
            self.get_logger().info(
                f'[步骤 2/4] 输入参考经度 {self.ref_lon}° (0x54) ...')
            cmd = build_lon_cmd(self.ref_lon)
            self._send_cmd(cmd)
            self._step = 3

        elif self._step == 3 and elapsed >= 2.6:
            self.get_logger().info(
                f'[步骤 3/4] 输入参考海拔 {self.ref_alt}m (0x45) ...')
            cmd = build_alt_cmd(self.ref_alt)
            self._send_cmd(cmd)
            self._step = 4

        elif self._step == 4 and elapsed >= 3.5:
            self.get_logger().info(
                '[步骤 4/4] 发送 INS 启动命令 (0x47) — 仅一次！')
            cmd = build_start_cmd(position_valid=True, attitude_valid=True)
            self._send_cmd(cmd)
            self.get_logger().info('')
            self.get_logger().info('  ╔════════════════════════════════════════╗')
            self.get_logger().info('  ║  INS 已启动，等待对准流程...         ║')
            self.get_logger().info('  ║  请保持设备静止不动！                ║')
            self.get_logger().info('  ║  流程: 粗对准 → 精对准 → INS导航     ║')
            self.get_logger().info('  ║  预计 3-10 分钟（取决于环境和设备）   ║')
            self.get_logger().info('  ╚════════════════════════════════════════╝')
            self.get_logger().info('')
            self._startup_done = True
            self._step_timer.cancel()

    def _send_cmd(self, cmd: bytes):
        """发送命令到 INS CMD 端口"""
        try:
            sock = self.ins_socket if self.ins_socket else socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(cmd, (INS_IP, INS_CMD_PORT))
            return True
        except Exception as e:
            self.get_logger().warn(f'发送命令失败: {e}')
            return False

    # ── INS 连接 ────────────────────────────────
    def _connect_ins(self):
        """绑定 UDP 端口接收 INS 数据"""
        if self.running:
            return
        try:
            self.ins_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ins_socket.setblocking(True)
            self.ins_socket.settimeout(1.0)
            self.ins_socket.bind(('0.0.0.0', INS_DATA_PORT))
            self.running = True
            self._last_recv_time = time.time()

            self.ins_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.ins_thread.start()
            self.get_logger().info(
                f'UDP 已绑定 0.0.0.0:{INS_DATA_PORT}，等待 INS 数据...')
        except Exception as e:
            self.get_logger().error(f'绑定 UDP 端口失败: {e}')

    # ── UDP 接收循环 ─────────────────────────────
    def _recv_loop(self):
        """接收 INS UDP 数据的主循环"""
        warned_no_data = False
        warned_align_time = False

        while self.running and rclpy.ok():
            try:
                data, addr = self.ins_socket.recvfrom(512)
                if len(data) >= 202:
                    was_first = (self.data['frame_count'] == 0)
                    self._parse_frame(data)
                    if was_first and self.data['frame_count'] > 0:
                        self._first_frame_time = time.time()
                        self.get_logger().info(
                            f'>>> INS 首帧已接收 ({addr[0]}:{addr[1]})'
                            f' — 对准状态={ALIGNMENT_TEXT.get(self.data["alignment"], "?")}')
                    self._last_recv_time = time.time()
            except socket.timeout:
                if not self._startup_done and not warned_no_data:
                    waited = time.time() - self._last_recv_time
                    if waited > 30:
                        self.get_logger().warn(
                            f'已等待 {waited:.0f}s 未收到 INS 数据帧。'
                            f'请检查: ① INS 是否上电 ② 网线是否连接')
                        warned_no_data = True
                # 对准阶段提醒
                if (self._startup_done and self.data['frame_count'] > 0
                        and self.data['alignment'] < 3
                        and not warned_align_time):
                    if self._first_frame_time > 0:
                        align_elapsed = time.time() - self._first_frame_time
                        if align_elapsed > 120 and self.data['alignment'] <= 1:
                            self.get_logger().warn(
                                f'对准已进行 {align_elapsed:.0f}s，'
                                f'当前状态={ALIGNMENT_TEXT.get(self.data["alignment"], "?")}。'
                                f'请确认设备是否静止、参考位置是否准确')
                            warned_align_time = True
                continue
            except OSError:
                break
            except Exception as e:
                if self.running:
                    self.get_logger().warn(f'UDP 接收异常: {e}')

    # ── 帧解析（202字节，详见惯导202帧定义.docx）────────
    def _parse_frame(self, data):
        if data[0] != 0x5A or data[1] != 0xA5 or data[201] != 0x55:
            return
        try:
            # 字节 2 bit1..0: 0=监控 1=粗对准 2=精对准 3=INS导航
            alignment = data[2] & 0x03
            # INS 安装方向修正: roll↔pitch 交换, roll 正负取反
            # 原始 data[33:37] 实际是物理 roll(正负对), data[37:41] 实际是物理 pitch(正负反)
            raw_pitch = struct.unpack('<f', data[33:37])[0]
            raw_roll  = struct.unpack('<f', data[37:41])[0]
            roll  = raw_pitch           # 物理 roll = 原始 pitch (正负对)
            pitch = -raw_roll           # 物理 pitch = -原始 roll (原始存的是物理 pitch 且正负反了)
            yaw   = struct.unpack('<f', data[41:45])[0]
            ve = struct.unpack('<f', data[45:49])[0]
            vn = struct.unpack('<f', data[49:53])[0]
            vd = struct.unpack('<f', data[53:57])[0]
            # 加速度 (字节 21-32, m/s²)
            ax = struct.unpack('<f', data[21:25])[0]
            ay = struct.unpack('<f', data[25:29])[0]
            az = struct.unpack('<f', data[29:33])[0]
            # 角速率 (字节 9-20, deg/s)
            wx = struct.unpack('<f', data[9:13])[0]
            wy = struct.unpack('<f', data[13:17])[0]
            wz = struct.unpack('<f', data[17:21])[0]
            # 水平面加速度 + 天向加速度 (字节 129-136, m/s²)
            acc_h = struct.unpack('<f', data[129:133])[0]
            acc_v = struct.unpack('<f', data[133:137])[0]
            lat = struct.unpack('<i', data[177:181])[0] * 1e-7
            lon = struct.unpack('<i', data[181:185])[0] * 1e-7
            alt = struct.unpack('<f', data[77:81])[0]
            sats = data[5]
            fix_type = data[4]
            hdop = struct.unpack('<f', data[89:93])[0]
            comb_status = data[197]
            temp = data[198]

            self.data.update({
                'alignment': alignment, 'pitch': pitch, 'roll': roll, 'yaw': yaw,
                've': ve, 'vn': vn, 'vd': vd,
                'ax': ax, 'ay': ay, 'az': az,
                'wx': wx, 'wy': wy, 'wz': wz,
                'acc_h': acc_h, 'acc_v': acc_v,
                'lat': lat, 'lon': lon, 'alt': alt,
                'sats': sats, 'fix_type': fix_type, 'hdop': hdop,
                'comb_status': comb_status, 'temp': temp,
            })
            self.data['frame_count'] += 1

            self._publish_all()

            # 对准状态变化时立即打印
            if alignment != self._last_alignment:
                self._last_alignment = alignment
                elapsed = (time.time() - self._first_frame_time
                           if self._first_frame_time > 0 else 0)
                marker = ''
                if alignment == 0:
                    marker = '  ⚠ 监控状态 — 无有效姿态'
                elif alignment == 1:
                    marker = f'  ⏳ 粗对准开始 (已过 {elapsed:.0f}s) — 请保持静止'
                elif alignment == 2:
                    marker = f'  🔄 精对准 (已过 {elapsed:.0f}s) — 继续静止'
                elif alignment == 3:
                    marker = f'  ✅ INS 导航模式！姿态/位置数据可信 (对准耗时 {elapsed:.0f}s)'
                self.get_logger().info(
                    f'>>> 对准状态变化: {ALIGNMENT_TEXT.get(alignment, "?")}{marker}')

            # 每 200 帧打印状态（~2秒一次 @100Hz）
            if self.data['frame_count'] % 200 == 0:
                self._log_status()
        except Exception as e:
            self.get_logger().error(f'帧解析错误: {e}')

    # ── 发布消息 ────────────────────────────────
    def _publish_all(self):
        d = self.data
        att = Vector3(); att.x = d['pitch']; att.y = d['roll']; att.z = d['yaw']
        self.pub_attitude.publish(att)
        vel = Vector3(); vel.x = d['ve']; vel.y = d['vn']; vel.z = d['vd']
        self.pub_velocity.publish(vel)
        pos = Vector3(); pos.x = d['lat']; pos.y = d['lon']; pos.z = d['alt']
        self.pub_position.publish(pos)
        acc = Vector3(); acc.x = d['ax']; acc.y = d['ay']; acc.z = d['az']
        self.pub_accel.publish(acc)
        gyro = Vector3(); gyro.x = d['wx']; gyro.y = d['wy']; gyro.z = d['wz']
        self.pub_gyro.publish(gyro)
        ali = Int8(); ali.data = d['alignment']
        self.pub_alignment.publish(ali)
        st = String()
        st.data = json.dumps({
            'alignment': d['alignment'],
            'alignment_text': ALIGNMENT_TEXT.get(d['alignment'], '未知'),
            'pitch': round(d['pitch'], 2), 'roll': round(d['roll'], 2),
            'yaw': round(d['yaw'], 2),
            've': round(d['ve'], 3), 'vn': round(d['vn'], 3), 'vd': round(d['vd'], 3),
            'ax': round(d['ax'], 3), 'ay': round(d['ay'], 3), 'az': round(d['az'], 3),
            'wx': round(d['wx'], 2), 'wy': round(d['wy'], 2), 'wz': round(d['wz'], 2),
            'acc_h': round(d['acc_h'], 3), 'acc_v': round(d['acc_v'], 3),
            'lat': round(d['lat'], 7), 'lon': round(d['lon'], 7), 'alt': round(d['alt'], 2),
            'sats': d['sats'], 'fix_type': d['fix_type'], 'hdop': round(d['hdop'], 1),
            'temp': d['temp'], 'comb_status': d['comb_status'],
            'frames': d['frame_count']
        })
        self.pub_status.publish(st)

    # ── 日志 ────────────────────────────────
    def _log_status(self):
        d = self.data
        ali_text = ALIGNMENT_TEXT.get(d['alignment'], '???')
        hint = ''
        if d['alignment'] < 3:
            hint = ' [等待对准 | 保持静止]'
        self.get_logger().info(
            f'[{d["frame_count"]:>6d}f] {ali_text} | '
            f'Yaw={d["yaw"]:+6.1f}deg | '
            f'Pitch={d["pitch"]:+5.1f}deg Roll={d["roll"]:+5.1f}deg | '
            f'Acc={d["ax"]:+6.2f},{d["ay"]:+6.2f},{d["az"]:+6.2f}m/s² | '
            f'Alt={d["alt"]:.1f}m | '
            f'Sats={d["sats"]:2d} HDOP={d["hdop"]:.1f} | '
            f'Temp={d["temp"]}C{hint}')

    # ── 清理 ────────────────────────────────
    def destroy_node(self):
        self.running = False
        if self.ins_thread:
            self.ins_thread.join(timeout=2)
        if self.ins_socket:
            self.ins_socket.close()
        self.get_logger().info('INS 驱动已停止')
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='INS400 自动驱动 v2')
    parser.add_argument('--lat', type=float, default=DEFAULT_LAT,
                        help=f'参考纬度 °N (默认 {DEFAULT_LAT})')
    parser.add_argument('--lon', type=float, default=DEFAULT_LON,
                        help=f'参考经度 °E (默认 {DEFAULT_LON})')
    parser.add_argument('--alt', type=float, default=DEFAULT_ALT,
                        help=f'参考海拔 m (默认 {DEFAULT_ALT})')
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = INSAutoDriver(lat=parsed.lat, lon=parsed.lon, alt=parsed.alt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
