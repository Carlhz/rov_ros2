#!/usr/bin/env python3
"""
ROV 综合监控 — 运行于 VM Ubuntu
集成显示: INS惯导 + D30深温计 + SF高度计

订阅话题:
  /ins/attitude      (Vector3)  姿态 Pitch/Roll/Yaw
  /ins/velocity      (Vector3)  速度 Ve/Vn/Vd
  /ins/position      (Vector3)  位置 Lat/Lon/Alt
  /ins/alignment     (Int8)     对准状态 0-3
  /rov/depth          (Float32)  水深
  /rov/depth_temp     (Float32)  水温
  /rov/depth_pressure (Float32)  压力
  /rov/altitude       (Float32)  高度(最强目标)
  /rov/altitude_nearest(Float32) 高度(最近目标)
  /rov/dvl/bottom_vel (Vector3)  DVL底跟踪速度 E/N/U (m/s)
  /rov/dvl/altitude   (Float32)  DVL距底高度 (m)
  /rov/dvl/status     (String)   DVL 完整状态 JSON

终端彩色仪表板，清晰标注 INS 对准状态
粗对准时醒目警告，导航模式后数据标记可信

运行:
  source /opt/ros/foxy/setup.bash
  export ROS_DOMAIN_ID=42
  python3 integrated_monitor.py
"""

import os, sys, time, json, signal

# 强制设置 ROS_DOMAIN_ID（与 RK3588 保持一致）
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['ROS_LOCALHOST_ONLY'] = '0'

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Float32
from geometry_msgs.msg import Vector3

# ── 电机转速条形图配置 ──
MOTOR_IDS     = [0, 1, 2, 3, 5, 6, 7]
MOTOR_MAX_RPM = 1800

# ── 终端控制 ────────────────────────────────
CLEAR = '\033[2J\033[H'         # 清屏 + 光标归位
HIDE_CURSOR = '\033[?25l'
SHOW_CURSOR = '\033[?25h'

# ── 颜色 ────────────────────────────────────
C_HEADER  = '\033[1;36m'   # 青色粗体
C_OK      = '\033[92m'     # 绿色
C_WARN    = '\033[93m'     # 黄色
C_ERROR   = '\033[91m'     # 红色
C_INFO    = '\033[96m'     # 青色
C_VALUE   = '\033[1;97m'   # 白色粗体
C_DIM     = '\033[2;37m'   # 灰色暗色
C_RESET   = '\033[0m'
C_BOLD    = '\033[1m'
C_BLINK   = '\033[5m'
C_BG_RED  = '\033[41;1;37m'
C_BG_YELLOW = '\033[43;1;30m'
C_BG_GREEN = '\033[42;1;37m'

# 对准状态
ALI_TEXT = {0: '未对准', 1: '粗对准', 2: '精对准', 3: 'INS导航模式'}
ALI_COLOR = {0: C_ERROR, 1: C_WARN, 2: C_WARN, 3: C_OK}

MAX_AGE = 5.0  # 数据超时阈值（秒）
YAW_HOLD_THRESHOLD = 10.0  # 定航向误差阈值 (度), <10°显示绿色, >=10°黄色


class ROVMonitor(Node):
    """ROV 综合监控节点"""

    def __init__(self):
        super().__init__('rov_integrated_monitor')

        # ── INS 数据 ──
        self.ins = {
            'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
            've': 0.0, 'vn': 0.0, 'vd': 0.0,
            'ax': 0.0, 'ay': 0.0, 'az': 0.0,
            'wx': 0.0, 'wy': 0.0, 'wz': 0.0,
            'acc_h': 0.0, 'acc_v': 0.0,
            'lat': 0.0, 'lon': 0.0, 'alt': 0.0,
            'alignment': 0, 'sats': 0, 'fix_type': 0,
            'hdop': 0.0, 'temp': 0, 'comb_status': 0,
            'frame_count': 0, 'age': 999.0
        }

        # ── 传感器数据 ──
        self.depth = {'value': None, 'age': 999.0}
        self.temp  = {'value': None, 'age': 999.0}
        self.pressure = {'value': None, 'age': 999.0}
        self.altitude = {'value': None, 'age': 999.0}
        self.alti_near = {'value': None, 'age': 999.0}

        # ── 电机转速数据 ──
        self.motors = {i: 0 for i in MOTOR_IDS}   # 当前各电机 RPM
        self.motor_move = 0.0                      # 归一化 move
        self.motor_up   = 0.0                      # 归一化 up
        self.motor_yaw  = 0.0                      # 归一化 yaw
        self.motor_dive = False                    # 定深档标志
        self.motor_age  = 999.0                    # motor_state 数据年龄
        self.motor_yaw_target = 0.0               # 目标航向 (度)
        self.motor_yaw_hold_active = False        # 定航向是否激活
        self.motor_yaw_hold_target = 0.0          # 定航向目标 (度)
        self.motor_yaw_err = 0.0                  # 航向误差 (度)

        # ── DVL 数据 ──
        self.dvl = {
            'bottom_vel_east': 0.0, 'bottom_vel_north': 0.0, 'bottom_vel_up': 0.0,
            'altitude': None, 'depth': None, 'heading': 0.0, 'pitch': 0.0, 'roll': 0.0,
            'temperature': None, 'bt_status': 'N', 'bt_percent_good': [0,0,0,0],
            'bt_corr': [0,0,0,0], 'ensembles': 0, 'parse_errors': 0,
            'coord_system': 'UNKNOWN', 'age': 999.0, 'salinity': 0.0,
        }

        # ── 计数 ──
        self.fps_count = {'ins': 0, 'depth': 0, 'alti': 0}
        self.fps = {'ins': 0.0, 'depth': 0.0, 'alti': 0.0}
        self._last_fps = time.time()

        # ── 订阅 INS ──
        self.sub_attitude  = self.create_subscription(Vector3, '/ins/attitude',  self._cb_attitude,  10)
        self.sub_velocity  = self.create_subscription(Vector3, '/ins/velocity',  self._cb_velocity,  10)
        self.sub_position  = self.create_subscription(Vector3, '/ins/position',  self._cb_position,  10)
        self.sub_accel     = self.create_subscription(Vector3, '/ins/acceleration', self._cb_accel,   10)
        self.sub_gyro      = self.create_subscription(Vector3, '/ins/angular_rate', self._cb_gyro,    10)
        self.sub_alignment = self.create_subscription(Int8,    '/ins/alignment', self._cb_alignment, 10)
        self.sub_status    = self.create_subscription(String,  '/ins/status',    self._cb_status,    10)

        # ── 订阅传感器 ──
        self.sub_depth     = self.create_subscription(Float32, '/rov/depth',           self._cb_depth,    10)
        self.sub_temp      = self.create_subscription(Float32, '/rov/depth_temp',      self._cb_temp,     10)
        self.sub_pressure  = self.create_subscription(Float32, '/rov/depth_pressure',  self._cb_pressure, 10)
        self.sub_altitude  = self.create_subscription(Float32, '/rov/altitude',        self._cb_altitude, 10)
        self.sub_alti_near = self.create_subscription(Float32, '/rov/altitude_nearest',self._cb_alti_near,10)

        # ── 订阅电机状态 ──
        self.sub_motor = self.create_subscription(String, '/rov/motor_state', self._cb_motor, 10)

        # ── 订阅 DVL ──
        self.sub_dvl_vel = self.create_subscription(Vector3, '/rov/dvl/bottom_vel', self._cb_dvl_vel, 10)
        self.sub_dvl_alt = self.create_subscription(Float32, '/rov/dvl/altitude', self._cb_dvl_alt, 10)
        self.sub_dvl_sts = self.create_subscription(String, '/rov/dvl/status', self._cb_dvl_status, 10)

        # ── 刷新定时器 (4Hz) ──
        self._start_time = time.time()
        self.timer = self.create_timer(0.25, self._render)

        self.get_logger().info('ROV 综合监控已启动')

        self._destroyed = False  # 防止双重销毁

        # 首帧将在 0.25s 后由定时器触发（此时 main() 的启动信息已显示完）

    # ── INS 回调 ─────────────────────────
    def _cb_attitude(self, msg):
        self.ins['pitch'] = msg.x; self.ins['roll'] = msg.y; self.ins['yaw'] = msg.z
        self.ins['age'] = 0; self.fps_count['ins'] += 1

    def _cb_velocity(self, msg):
        self.ins['ve'] = msg.x; self.ins['vn'] = msg.y; self.ins['vd'] = msg.z

    def _cb_accel(self, msg):
        """IMU 加速度 Ax/Ay/Az (m/s²)"""
        self.ins['ax'] = msg.x; self.ins['ay'] = msg.y; self.ins['az'] = msg.z

    def _cb_gyro(self, msg):
        """IMU 角速率 Wx/Wy/Wz (deg/s)"""
        self.ins['wx'] = msg.x; self.ins['wy'] = msg.y; self.ins['wz'] = msg.z

    def _cb_position(self, msg):
        self.ins['lat'] = msg.x; self.ins['lon'] = msg.y; self.ins['alt'] = msg.z

    def _cb_alignment(self, msg):
        self.ins['alignment'] = msg.data

    def _cb_status(self, msg):
        try:
            s = json.loads(msg.data)
            for k in ['sats', 'fix_type', 'hdop', 'temp', 'comb_status', 'frame_count',
                       'ax', 'ay', 'az', 'wx', 'wy', 'wz', 'acc_h', 'acc_v']:
                if k in s:
                    self.ins[k] = s[k]
        except:
            pass

    # ── 传感器回调 ───────────────────────
    def _cb_depth(self, msg):
        self.depth['value'] = msg.data; self.depth['age'] = 0; self.fps_count['depth'] += 1

    def _cb_temp(self, msg):
        self.temp['value'] = msg.data; self.temp['age'] = 0

    def _cb_pressure(self, msg):
        self.pressure['value'] = msg.data; self.pressure['age'] = 0

    def _cb_altitude(self, msg):
        self.altitude['value'] = msg.data; self.altitude['age'] = 0; self.fps_count['alti'] += 1

    def _cb_alti_near(self, msg):
        self.alti_near['value'] = msg.data; self.alti_near['age'] = 0

    def _cb_motor(self, msg):
        """解析 /rov/motor_state JSON，更新各电机转速"""
        try:
            s = json.loads(msg.data)
            if 'motors' in s:
                m = s['motors']
                for k in MOTOR_IDS:
                    self.motors[k] = m.get(str(k), m.get(k, 0))
            self.motor_move = s.get('move_norm', self.motor_move)
            self.motor_up   = s.get('up_norm',   self.motor_up)
            self.motor_yaw  = s.get('yaw_norm',  self.motor_yaw)
            self.motor_dive = s.get('dive_flag', 0.0) > 0.1  # 定深档标志
            self.motor_yaw_target = s.get('yaw_target', 0.0)
            self.motor_yaw_hold_active = s.get('yaw_hold_active', False)
            self.motor_yaw_hold_target = s.get('yaw_hold_target', 0.0)
            self.motor_age  = 0.0
        except Exception:
            pass

    # ── DVL 回调 ─────────────────────────
    def _cb_dvl_vel(self, msg):
        self.dvl['bottom_vel_east']  = msg.x
        self.dvl['bottom_vel_north'] = msg.y
        self.dvl['bottom_vel_up']    = msg.z
        self.dvl['age'] = 0.0

    def _cb_dvl_alt(self, msg):
        self.dvl['altitude'] = msg.data
        self.dvl['age'] = 0.0

    def _cb_dvl_status(self, msg):
        try:
            s = json.loads(msg.data)
            for k in ['depth', 'heading', 'pitch', 'roll', 'temperature',
                       'bt_status', 'bt_percent_good', 'bt_corr', 'ensembles',
                       'parse_errors', 'coord_system', 'salinity']:
                if k in s:
                    self.dvl[k] = s[k]
            if 'attitude' in s:
                for ak in ['heading', 'pitch', 'roll']:
                    self.dvl[ak] = s['attitude'].get(ak, self.dvl[ak])
            self.dvl['age'] = 0.0
        except Exception:
            pass

    # ── 渲染 ─────────────────────────────
    def _fmt_val(self, value, fmt='.2f', stale_age=None):
        """格式化值，超时显示 ---"""
        if value is None:
            return f'{C_DIM}   ---   {C_RESET}'
        if stale_age is not None and stale_age > MAX_AGE:
            return f'{C_DIM}{value:{fmt}}{C_RESET}'
        return f'{C_VALUE}{value:{fmt}}{C_RESET}'

    def _age_flag(self, age):
        """超时标记"""
        if age > MAX_AGE:
            return f'{C_BG_RED} ! {C_RESET}'
        return f'{C_OK} ✓ {C_RESET}'

    # ── 方框自动对齐辅助函数 ──
    import re
    _ANSI_RE = re.compile(r'\033\[[0-9;]*m')

    def _vlen(self, s):
        """计算去掉 ANSI 转义码后的可见字符长度"""
        return len(self._ANSI_RE.sub('', s))

    def _box_top(self, title, w=72):
        """生成方框顶部: ┌── title ──────┐  (总可见宽度 = w)"""
        # 可见: ┌(1) ──(2) 空格(1) title 空格(1) ─*pad ┐(1) = 6 + vlen(title) + pad = w
        pad = w - 6 - self._vlen(title)
        if pad < 1:
            pad = 1
        return f'{C_HEADER}┌── {title} {"─" * pad}┐{C_RESET}'

    def _box_bot(self, w=72):
        """生成方框底部: └──────────────┘  (总可见宽度 = w)"""
        # 可见: └(1) ─*(w-2) ┘(1) = w
        return f'{C_HEADER}└{"─" * (w - 2)}┘{C_RESET}'

    def _box_line(self, content, w=72):
        """生成方框内容行: │  content     │"""
        inner = w - 2
        vlen = self._vlen(content)
        pad = inner - vlen
        if pad < 0:
            # 内容太长，截断
            # 从右边逐步去掉内容直到 fit
            # 简单做法：去掉尾部
            while self._vlen(content) > inner and len(content) > 10:
                content = content[:-1]
            pad = inner - self._vlen(content)
        return f'{C_HEADER}│{C_RESET}{content}{" " * pad}{C_HEADER}│{C_RESET}'

    def _render(self):
        """绘制仪表板 — 自动对齐版"""
        now = time.time()
        elapsed = now - self._start_time
        BW = 72  # 统一方框宽度

        # 更新 age
        dt = 0.25
        self.ins['age'] += dt
        self.depth['age'] += dt
        self.temp['age'] += dt
        self.pressure['age'] += dt
        self.altitude['age'] += dt
        self.alti_near['age'] += dt
        self.motor_age += dt
        self.dvl['age'] += dt

        # 更新 FPS
        if now - self._last_fps >= 1.0:
            fps_dt = now - self._last_fps
            self.fps['ins']   = self.fps_count['ins']   / fps_dt
            self.fps['depth'] = self.fps_count['depth'] / fps_dt
            self.fps['alti']  = self.fps_count['alti']  / fps_dt
            self.fps_count = {'ins': 0, 'depth': 0, 'alti': 0}
            self._last_fps = now

        # ── INS 数据状态判定 ──
        ali = self.ins['alignment']
        ali_text = ALI_TEXT.get(ali, '未知')
        ali_color = ALI_COLOR.get(ali, C_ERROR)
        ins_ok = ali >= 3
        ins_stale = self.ins['age'] > MAX_AGE
        ali_warning = ali < 3 and not ins_stale
        ali_age_text = f'({self.ins["age"]:.0f}s未更新)' if ins_stale else ''

        lines = []
        m, s = divmod(int(elapsed), 60)
        h, m = divmod(m, 60)

        # 标题栏
        lines.append(
            f'{C_BG_GREEN}  ROV 综合监控  {C_RESET}  '
            f'{C_INFO}FPS: INS={self.fps["ins"]:5.0f}  '
            f'Depth={self.fps["depth"]:4.0f}  '
            f'Alti={self.fps["alti"]:4.0f}  {C_RESET}  '
            f'{C_DIM}ROS_DOMAIN=42{C_RESET}  '
            f'{C_DIM}运行 {h:02d}:{m:02d}:{s:02d}{C_RESET}'
        )

        # ═══ INS 区块 ═══
        lines.append(self._box_top('INS 惯导系统', BW))

        if ins_stale:
            status_line = f'{C_BG_RED} ▲ 无 INS 数据！请检查 RK3588 驱动是否启动 {ali_age_text} {C_RESET}'
        elif ins_ok:
            status_line = f'{C_BG_GREEN} ● INS导航模式 — 姿态/位置数据可信 {C_RESET}'
        elif ali == 0:
            status_line = f'{C_BG_YELLOW} ◐ 未对准 — INS 尚未开始对准流程，等待中... {C_RESET}'
        elif ali == 1:
            status_line = f'{C_BG_YELLOW}{C_BLINK} ▲ 粗对准中 — 数据仅供参考，请等待对准完成！▲ {C_RESET}'
        elif ali == 2:
            status_line = f'{C_BG_YELLOW} ◑ 精对准中 — 数据正在收敛，请稍候... {C_RESET}'
        else:
            status_line = f'{ali_color} 对准状态: {ali_text} {C_RESET}'
        lines.append(self._box_line(f'  {status_line}', BW))

        ins_age = self.ins['age']
        yaw_f   = self._fmt_val(self.ins['yaw'],   '6.1f', ins_age)
        pitch_f = self._fmt_val(self.ins['pitch'], '6.1f', ins_age)
        roll_f  = self._fmt_val(self.ins['roll'],  '6.1f', ins_age)
        lines.append(self._box_line(
            f'  姿态:   {C_INFO}Yaw{C_RESET}={yaw_f}°  '
            f'{C_INFO}Pitch{C_RESET}={pitch_f}°  '
            f'{C_INFO}Roll{C_RESET}={roll_f}°', BW))

        ve_f = self._fmt_val(self.ins['ve'], '7.3f', ins_age)
        vn_f = self._fmt_val(self.ins['vn'], '7.3f', ins_age)
        vd_f = self._fmt_val(self.ins['vd'], '7.3f', ins_age)
        lines.append(self._box_line(
            f'  速度:   {C_INFO}Ve{C_RESET}={ve_f}m/s  '
            f'{C_INFO}Vn{C_RESET}={vn_f}m/s  '
            f'{C_INFO}Vd{C_RESET}={vd_f}m/s', BW))

        ax_f = self._fmt_val(self.ins['ax'], '6.2f', ins_age)
        ay_f = self._fmt_val(self.ins['ay'], '6.2f', ins_age)
        az_f = self._fmt_val(self.ins['az'], '6.2f', ins_age)
        lines.append(self._box_line(
            f'  加速度: {C_INFO}Ax{C_RESET}={ax_f}m/s²  '
            f'{C_INFO}Ay{C_RESET}={ay_f}m/s²  '
            f'{C_INFO}Az{C_RESET}={az_f}m/s²', BW))

        wx_f = self._fmt_val(self.ins['wx'], '6.2f', ins_age)
        wy_f = self._fmt_val(self.ins['wy'], '6.2f', ins_age)
        wz_f = self._fmt_val(self.ins['wz'], '6.2f', ins_age)
        lines.append(self._box_line(
            f'  角速率: {C_INFO}Wx{C_RESET}={wx_f}°/s  '
            f'{C_INFO}Wy{C_RESET}={wy_f}°/s  '
            f'{C_INFO}Wz{C_RESET}={wz_f}°/s', BW))

        lat_f = self._fmt_val(self.ins['lat'], '11.7f', ins_age)
        lon_f = self._fmt_val(self.ins['lon'], '11.7f', ins_age)
        alt_f = self._fmt_val(self.ins['alt'], '7.2f', ins_age)
        lines.append(self._box_line(
            f'  位置:   {C_INFO}Lat{C_RESET}={lat_f}  '
            f'{C_INFO}Lon{C_RESET}={lon_f}  '
            f'{C_INFO}Alt{C_RESET}={alt_f}m', BW))

        sats_f = self._fmt_val(self.ins['sats'], '2d', ins_age)
        hdop_f = self._fmt_val(self.ins['hdop'], '.1f', ins_age)
        temp_f = self._fmt_val(self.ins['temp'], 'd', ins_age)
        lines.append(self._box_line(
            f'  GNSS:   卫星={sats_f}颗  HDOP={hdop_f}  '
            f'FixType={self.ins["fix_type"]}  '
            f'温度={temp_f}°C', BW))

        if ali_warning:
            lines.append(self._box_line(
                f'  {C_WARN}⚠ 提示: INS 粗对准通常需要 2-5 分钟，进入「INS导航模式」后位置/航向才准确{C_RESET}', BW))

        lines.append(self._box_bot(BW))

        # ═══ 传感器区块 ═══
        lines.append(self._box_top('水深/高度传感器', BW))

        d_flag = self._age_flag(self.depth['age'])
        t_flag = self._age_flag(self.temp['age'])
        d_f = self._fmt_val(self.depth['value'], '7.2f', self.depth['age'])
        t_f = self._fmt_val(self.temp['value'], '6.2f', self.temp['age'])
        p_f = self._fmt_val(self.pressure['value'], '8.4f', self.pressure['age'])
        lines.append(self._box_line(
            f'  {d_flag} {C_INFO}深度{C_RESET}={d_f} m  |  '
            f'{t_flag} {C_INFO}水温{C_RESET}={t_f} °C  |  '
            f'{C_INFO}压力{C_RESET}={p_f} MPa', BW))

        a_flag = self._age_flag(self.altitude['age'])
        a_f = self._fmt_val(self.altitude['value'], '7.2f', self.altitude['age'])
        n_f = self._fmt_val(self.alti_near['value'], '7.2f', self.alti_near['age'])
        lines.append(self._box_line(
            f'  {a_flag} {C_INFO}高度(最强){C_RESET}={a_f} m  |  '
            f'{C_INFO}最近目标{C_RESET}={n_f} m', BW))

        lines.append(self._box_bot(BW))

        # ═══ 电机转速区块 ═══
        lines.append(self._box_top('电机转速实时', BW))

        if self.motor_age > MAX_AGE:
            lines.append(self._box_line(
                f'  {C_BG_RED} ▲ 无电机数据！请检查 RK3588 motor_controller 是否运行 {C_RESET}', BW))
        else:
            up_n = self.motor_up
            if up_n > 0.01:
                up_status = f'{C_BG_GREEN} ↓下潜 {C_RESET}'
            elif up_n < -0.01:
                up_status = f'{C_BG_YELLOW} ↑上浮 {C_RESET}'
            else:
                up_status = f'{C_DIM} 悬停 {C_RESET}'

            dive_status = f'{C_BG_GREEN} ★定深档 {C_RESET}' if self.motor_dive else f'{C_DIM}普通档{C_RESET}'

            move_n = -self.motor_move  # 显示方向修正: joy_controller用-RY发送,正=前进
            if move_n > 0.01:
                mv_status = f'{C_OK}→前进{C_RESET}'
            elif move_n < -0.01:
                mv_status = f'{C_WARN}←后退{C_RESET}'
            else:
                mv_status = f'{C_DIM}  停  {C_RESET}'

            lines.append(self._box_line(
                f'  状态: {up_status}  {mv_status}  {dive_status}  '
                f'move={C_VALUE}{self.motor_move:+.3f}{C_RESET}  '
                f'up={C_VALUE}{self.motor_up:+.3f}{C_RESET}  '
                f'yaw={C_VALUE}{self.motor_yaw:+.3f}{C_RESET}', BW))

            # 定航向状态行
            if self.motor_yaw_hold_active:
                yaw_target = self.motor_yaw_hold_target
                # 从 INS 获取当前实际航向
                ins_age = self.ins['age']
                ins_yaw = self.ins['yaw'] if ins_age < MAX_AGE and self.ins['alignment'] >= 1 else None
                if ins_yaw is not None:
                    # 计算误差 (统一用最小角度差)
                    yaw_err = yaw_target - ins_yaw
                    while yaw_err > 180: yaw_err -= 360
                    while yaw_err < -180: yaw_err += 360
                    err_color = C_OK if abs(yaw_err) < YAW_HOLD_THRESHOLD else C_WARN
                    yaw_line = (f'  定航向: 目标={C_VALUE}{yaw_target:.0f}{C_RESET}°  '
                                f'实际={C_VALUE}{ins_yaw:.1f}{C_RESET}°  '
                                f'误差={err_color}{yaw_err:+.1f}{C_RESET}°')
                else:
                    yaw_line = (f'  定航向: 目标={C_VALUE}{yaw_target:.0f}{C_RESET}°  '
                                f'{C_WARN}(无INS数据){C_RESET}')
                lines.append(self._box_line(yaw_line, BW))

            def _rpm_str(v):
                if v == 0:
                    return f'{C_DIM}    0{C_RESET}'
                color = C_OK if v > 0 else C_WARN
                return f'{color}{v:+5d}{C_RESET}'

            def _rpm_bar(v, width=8):
                half = width // 2
                ratio = abs(v) / MOTOR_MAX_RPM
                blocks = int(ratio * half + 0.5)
                bar_c = C_OK if v >= 0 else C_WARN
                if v > 0:
                    bar = ' ' * half + '|' + f'{bar_c}' + '█' * blocks + C_RESET + '░' * (half - blocks)
                elif v < 0:
                    bar = '░' * (half - blocks) + f'{bar_c}' + '█' * blocks + C_RESET + '|' + ' ' * half
                else:
                    bar = '░' * half + '|' + '░' * half
                return bar

            row1 = '  '.join(
                f'ID{i}:{_rpm_str(self.motors[i])}rpm {_rpm_bar(self.motors[i])}'
                for i in [0, 1, 2, 3]
            )
            lines.append(self._box_line(f'  尾部: {row1}', BW))

            row2 = '  '.join(
                f'ID{i}:{_rpm_str(self.motors[i])}rpm {_rpm_bar(self.motors[i])}'
                for i in [5, 6, 7]
            )
            lines.append(self._box_line(f'  垂直: {row2}', BW))

        lines.append(self._box_bot(BW))

        # ═══ DVL 区块 ═══
        lines.append(self._box_top('DVL 多普勒计程仪 (PathFinder)', BW))

        dvl_age = self.dvl['age']
        dvl_stale = dvl_age > MAX_AGE

        if dvl_stale:
            lines.append(self._box_line(
                f'  {C_BG_RED} ▲ 无 DVL 数据！请检查 RK3588 DVL 驱动是否启动 {C_RESET}', BW))
        else:
            ve_f = self._fmt_val(self.dvl['bottom_vel_east'], '7.3f', dvl_age)
            vn_f = self._fmt_val(self.dvl['bottom_vel_north'], '7.3f', dvl_age)
            vu_f = self._fmt_val(self.dvl['bottom_vel_up'], '7.3f', dvl_age)
            lines.append(self._box_line(
                f'  底速:   {C_INFO}E{C_RESET}={ve_f}  '
                f'{C_INFO}N{C_RESET}={vn_f}  '
                f'{C_INFO}U{C_RESET}={vu_f} m/s', BW))

            alt_f = self._fmt_val(self.dvl['altitude'], '6.2f', dvl_age)
            bt_status = self.dvl['bt_status']
            if bt_status == 'A':
                bt_flag = f'{C_BG_GREEN} 有效 {C_RESET}'
            elif bt_status == 'V':
                bt_flag = f'{C_BG_YELLOW} 无效 {C_RESET}'
            else:
                bt_flag = f'{C_DIM} 等待 {C_RESET}'

            depth_dvl = self._fmt_val(self.dvl['depth'], '6.2f', dvl_age)
            temp_f = self._fmt_val(self.dvl['temperature'], '5.1f', dvl_age)
            lines.append(self._box_line(
                f'  高度:   {C_INFO}底高{C_RESET}={alt_f} m  {bt_flag}  |  '
                f'{C_INFO}水深{C_RESET}={depth_dvl} m  |  '
                f'{C_INFO}水温{C_RESET}={temp_f} °C', BW))

            dvl_yaw   = self._fmt_val(self.dvl['heading'], '6.1f', dvl_age)
            dvl_pitch = self._fmt_val(self.dvl['pitch'], '5.1f', dvl_age)
            dvl_roll  = self._fmt_val(self.dvl['roll'], '5.1f', dvl_age)
            lines.append(self._box_line(
                f'  姿态:   {C_INFO}Yaw{C_RESET}={dvl_yaw}°  '
                f'{C_INFO}Pitch{C_RESET}={dvl_pitch}°  '
                f'{C_INFO}Roll{C_RESET}={dvl_roll}°', BW))

            pg = self.dvl['bt_percent_good']
            pg_str = '  '.join(f'B{i}:{pg[i]:3d}%' for i in range(4))
            lines.append(self._box_line(f'  %Good:  {pg_str}', BW))

            coord = self.dvl.get('coord_system', '?')
            ens   = self.dvl.get('ensembles', 0)
            errs  = self.dvl.get('parse_errors', 0)
            salinity_f = self._fmt_val(self.dvl.get('salinity', 0), '4.1f', dvl_age)
            lines.append(self._box_line(
                f'  坐标={coord}  |  '
                f'Ensembles={ens}  |  '
                f'盐度={salinity_f} ppt  |  '
                f'解析错误={errs}', BW))

        lines.append(self._box_bot(BW))

        # ═══ 底部状态栏 ═══
        ins_status = ins_stale and f'{C_ERROR}OFFLINE{C_RESET}' or \
                     ins_ok and f'{C_OK}NAV-OK{C_RESET}' or \
                     f'{C_WARN}{ali_text}{C_RESET}'
        depth_status = self.depth['age'] < MAX_AGE and self.depth['value'] is not None and f'{C_OK}OK{C_RESET}' or f'{C_ERROR}---{C_RESET}'
        alti_status  = self.altitude['age'] < MAX_AGE and self.altitude['value'] is not None and f'{C_OK}OK{C_RESET}' or f'{C_ERROR}---{C_RESET}'
        dvl_status   = self.dvl['age'] < MAX_AGE and self.dvl['altitude'] is not None and f'{C_OK}OK{C_RESET}' or f'{C_ERROR}---{C_RESET}'
        lines.append(
            f'\n{C_DIM}[STATUS]{C_RESET}  '
            f'INS={ins_status}  |  '
            f'Depth={depth_status}  |  '
            f'Alti={alti_status}  |  '
            f'DVL={dvl_status}  |  '
            f'Motor={f"{C_OK}OK{C_RESET}" if self.motor_age < MAX_AGE else f"{C_ERROR}---{C_RESET}"}  |  '
            f'{C_DIM}数据刷新: INS={self.ins["age"]:.0f}s  '
            f'Depth={self.depth["age"]:.0f}s  '
            f'Alti={self.altitude["age"]:.0f}s  '
            f'DVL={self.dvl["age"]:.0f}s{C_RESET}')

        # 输出
        sys.stdout.write(CLEAR + '\n'.join(lines) + '\n')
        sys.stdout.flush()

    def destroy_node(self):
        if self._destroyed:
            return
        self._destroyed = True
        sys.stdout.write(SHOW_CURSOR + CLEAR)
        sys.stdout.flush()
        super().destroy_node()


def main(args=None):
    print(f'{C_BOLD}ROV 综合监控 — 启动中...{C_RESET}')
    print(f'  步骤 1/4: 初始化 ROS2 DDS 发现 (ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "42")})...')
    sys.stdout.flush()

    rclpy.init(args=args)

    print(f'  {C_OK}✓{C_RESET} 步骤 2/4: 创建监控节点...')
    sys.stdout.flush()

    node = ROVMonitor()
    node.get_logger().info(f'创建完成，等待传感器数据...')

    # 隐藏光标
    sys.stdout.write(HIDE_CURSOR)
    sys.stdout.flush()

    def sig_handler(sig, frame):
        # 不需要手动清理，finally 块会处理
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    print(f'  {C_OK}✓{C_RESET} 步骤 3/4: 订阅话题完成 ({15} 个话题)')
    print(f'  {C_OK}✓{C_RESET} 步骤 4/4: 进入监听循环 (刷新率 4Hz)')
    print(f'\n{C_DIM}  提示: 按 Ctrl+C 退出{C_RESET}')
    sys.stdout.flush()

    # 短暂延迟让用户看到启动信息，然后进入循环
    time.sleep(0.5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
