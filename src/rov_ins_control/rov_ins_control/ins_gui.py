#!/usr/bin/env python3
"""
INS Control GUI Panel
Runs on VM - graphical interface for INS control and monitoring
"""

import threading
import subprocess
import sys
import os
import time

# ── VMware 兼容性修复：禁用硬件字体渲染 ────────────────────────
os.environ.setdefault('GDK_BACKEND', 'x11')
os.environ.setdefault('LIBGL_ALWAYS_SOFTWARE', '1')
# 禁用 Xft 字体抗锯齿（避免 RenderAddGlyphs 崩溃）
os.environ['DISPLAY'] = os.environ.get('DISPLAY', ':0')

import tkinter as tk
from tkinter import ttk, messagebox, font

# ── ROS2 环境初始化 ────────────────────────────────────────────
ROS_SETUP = '/opt/ros/foxy/setup.bash'
WS_SETUP = os.path.expanduser('~/rov_ros2_ws/install/setup.bash')
ROS_DOMAIN_ID = '42'

def _source_ros_env():
    """获取 source ROS2 后的环境变量"""
    cmd = f'source {ROS_SETUP} && source {WS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && export ROS_LOCALHOST_ONLY=0 && env'
    result = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True)
    env = {}
    for line in result.stdout.splitlines():
        if '=' in line:
            k, _, v = line.partition('=')
            env[k] = v
    return env

try:
    ROS_ENV = _source_ros_env()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64, Int32, String
    from geometry_msgs.msg import Vector3
    from sensor_msgs.msg import Imu
    ROS_AVAILABLE = True
except Exception:
    ROS_ENV = os.environ.copy()
    ROS_AVAILABLE = False

# ── 颜色主题 ─────────────────────────────────────────────────
COLORS = {
    'bg':        '#1e1e2e',
    'panel':     '#2a2a3e',
    'border':    '#44475a',
    'text':      '#cdd6f4',
    'subtext':   '#a6adc8',
    'green':     '#a6e3a1',
    'yellow':    '#f9e2af',
    'red':       '#f38ba8',
    'blue':      '#89b4fa',
    'teal':      '#94e2d5',
    'btn_stop':  '#f38ba8',
    'btn_start': '#a6e3a1',
    'btn_pos':   '#89b4fa',
    'btn_query': '#cba6f7',
}

ALIGN_STATUS = {
    0: ('监控状态',  '#f9e2af'),
    1: ('粗对准中',  '#89b4fa'),
    2: ('精对准中',  '#94e2d5'),
    3: ('INS 导航', '#a6e3a1'),
}

GNSS_FIX = {
    0: ('无定位',   '#f38ba8'),
    1: ('单点定位', '#f9e2af'),
    2: ('差分定位', '#89b4fa'),
    4: ('RTK 固定', '#a6e3a1'),
    5: ('RTK 浮动', '#94e2d5'),
}

# ── 字体配置（VMware 兼容）────────────────────────────────────
# 优先使用 Liberation/DejaVu，这两个在 Ubuntu 上几乎必有
# 如果都没有，回退到 sans/monospace（Pango 通用名称）
_SANS_CANDIDATES   = ['Liberation Sans', 'DejaVu Sans', 'Ubuntu', 'Noto Sans CJK SC', 'sans']
_MONO_CANDIDATES   = ['Liberation Mono', 'DejaVu Sans Mono', 'Ubuntu Mono', 'Courier New', 'monospace']

def _pick_font(candidates):
    """从候选列表中选第一个系统已安装的字体，否则返回 TkDefaultFont 家族"""
    try:
        available = set(tk.font.families())
        for name in candidates:
            if name in available:
                return name
    except Exception:
        pass
    return 'TkDefaultFont'

# 字体将在 Tk 窗口创建后初始化
FONT_UI   = None   # 普通 UI 文字
FONT_MONO = None   # 等宽数值

def _init_fonts():
    global FONT_UI, FONT_MONO
    sans = _pick_font(_SANS_CANDIDATES)
    mono = _pick_font(_MONO_CANDIDATES)
    FONT_UI   = sans
    FONT_MONO = mono


# ── ROS2 通信线程 ─────────────────────────────────────────────
class ROSBridge(threading.Thread):
    """后台 ROS2 订阅线程，把数据写入 data_store"""

    def __init__(self, data_store):
        super().__init__(daemon=True)
        self.data = data_store
        self._node = None
        self.running = True

    def run(self):
        if not ROS_AVAILABLE:
            return
        try:
            rclpy.init()
            self._node = rclpy.create_node('ins_gui_monitor')
            n = self._node
            d = self.data

            def cb_f64(key):
                def _cb(msg): d[key] = msg.data
                return _cb

            def cb_i32(key):
                def _cb(msg): d[key] = msg.data
                return _cb

            def cb_v3(key):
                def _cb(msg): d[key] = (msg.x, msg.y, msg.z)
                return _cb

            def cb_str(key):
                def _cb(msg): d[key] = msg.data
                return _cb

            subs = [
                (Float64, '/ins/latitude',        cb_f64('lat')),
                (Float64, '/ins/longitude',       cb_f64('lon')),
                (Float64, '/ins/altitude',        cb_f64('alt')),
                (Vector3, '/ins/pose',            cb_v3('pose')),
                (Vector3, '/ins/twist',           cb_v3('twist')),
                (Int32,   '/ins/align_status',    cb_i32('align')),
                (Int32,   '/ins/work_status',     cb_i32('work')),
                (Int32,   '/ins/gnss_fix_type',   cb_i32('fix')),
                (Int32,   '/ins/gnss_satellites', cb_i32('sats')),
                (Float64, '/ins/gnss_hdop',       cb_f64('hdop')),
                (Float64, '/ins/gnss_heading',    cb_f64('gnss_hdg')),
                (Float64, '/ins/gnss_latitude',   cb_f64('gnss_lat')),
                (Float64, '/ins/gnss_longitude',  cb_f64('gnss_lon')),
                (Float64, '/ins/gnss_altitude',   cb_f64('gnss_alt')),
                (Vector3, '/ins/dvl_velocity',    cb_v3('dvl_vel')),
                (Float64, '/ins/dvl_depth',       cb_f64('dvl_depth')),
                (Int32,   '/ins/temperature',     cb_i32('temp')),
                (Int32,   '/ins/combined_status', cb_i32('comb')),
                (String,  '/ins/raw',             cb_str('raw')),
            ]
            for msg_type, topic, cb in subs:
                n.create_subscription(msg_type, topic, cb, 10)

            while self.running and rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.05)

        except Exception as e:
            self.data['ros_error'] = str(e)
        finally:
            if self._node:
                self._node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def stop(self):
        self.running = False


def send_ros_command(cmd, lat=None, lon=None, alt=0.0):
    """通过 subprocess 调用 ins_control_client"""
    if cmd == 'stop':
        args = ['ros2', 'run', 'rov_ins_control', 'ins_control_client', 'stop']
    elif cmd == 'start':
        args = ['ros2', 'run', 'rov_ins_control', 'ins_control_client', 'start']
    elif cmd == 'setpos':
        args = ['ros2', 'run', 'rov_ins_control', 'ins_control_client',
                'setpos', str(lat), str(lon), str(alt)]
    elif cmd == 'status':
        args = ['ros2', 'run', 'rov_ins_control', 'ins_control_client', 'status']
    else:
        return False, 'Unknown command'

    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=10, env=ROS_ENV)
        output = result.stdout + result.stderr
        success = result.returncode == 0
        return success, output
    except subprocess.TimeoutExpired:
        return False, 'Timeout - RK3588 未响应，请检查连接'
    except Exception as e:
        return False, str(e)


# ── GUI 主界面 ────────────────────────────────────────────────
class INSGui:
    def __init__(self):
        self.data = {}
        self.ros_bridge = ROSBridge(self.data)

        self.root = tk.Tk()
        self.root.title('ROV INS 控制台')
        self.root.configure(bg=COLORS['bg'])
        self.root.geometry('900x680')
        self.root.resizable(True, True)

        # 在 Tk 初始化后检测可用字体（必须在此之后才能查询 font.families）
        _init_fonts()

        self._build_ui()

        self.ros_bridge.start()
        self._refresh()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.mainloop()

    # ── UI 构建 ────────────────────────────────────────────────
    def _build_ui(self):
        # 标题栏
        title_frame = tk.Frame(self.root, bg=COLORS['panel'], height=50)
        title_frame.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(title_frame, text='ROV INS 控制台',
                 font=(FONT_UI, 16, 'bold'),
                 fg=COLORS['blue'], bg=COLORS['panel']).pack(side=tk.LEFT, padx=16, pady=8)

        # ROS 状态指示
        self.lbl_ros = tk.Label(title_frame,
                                text='● ROS2 连接中...',
                                font=(FONT_UI, 10),
                                fg=COLORS['yellow'], bg=COLORS['panel'])
        self.lbl_ros.pack(side=tk.RIGHT, padx=16)

        # 主内容区（左右两栏）
        content = tk.Frame(self.root, bg=COLORS['bg'])
        content.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        content.rowconfigure(0, weight=1)

        left = tk.Frame(content, bg=COLORS['bg'])
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 4))

        right = tk.Frame(content, bg=COLORS['bg'])
        right.grid(row=0, column=1, sticky='nsew', padx=(4, 0))

        # ── 左栏：状态监控 ──
        self._build_status_panel(left)
        self._build_position_panel(left)
        self._build_attitude_panel(left)

        # ── 右栏：控制 + 日志 ──
        self._build_control_panel(right)
        self._build_log_panel(right)

        # 底部状态栏
        self.status_bar = tk.Label(
            self.root, text='就绪',
            font=(FONT_UI, 9), fg=COLORS['subtext'],
            bg=COLORS['panel'], anchor='w')
        self.status_bar.pack(fill=tk.X, padx=8, pady=(4, 8))

    def _panel(self, parent, title):
        """创建一个带标题的面板"""
        frame = tk.LabelFrame(parent,
                              text=f'  {title}  ',
                              font=(FONT_UI, 10, 'bold'),
                              fg=COLORS['blue'], bg=COLORS['panel'],
                              bd=1, relief='solid',
                              labelanchor='nw')
        frame.pack(fill=tk.X, pady=3)
        return frame

    def _row(self, parent, label, default='--'):
        """创建一行 label: value"""
        row = tk.Frame(parent, bg=COLORS['panel'])
        row.pack(fill=tk.X, padx=10, pady=2)
        tk.Label(row, text=label, width=14, anchor='w',
                 font=(FONT_UI, 9), fg=COLORS['subtext'],
                 bg=COLORS['panel']).pack(side=tk.LEFT)
        val = tk.Label(row, text=default, anchor='w',
                       font=(FONT_MONO, 10, 'bold'),
                       fg=COLORS['text'], bg=COLORS['panel'])
        val.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return val

    def _build_status_panel(self, parent):
        p = self._panel(parent, '系统状态')
        # 对准状态大字显示
        self.lbl_align = tk.Label(p, text='-- 监控状态 --',
                                  font=(FONT_UI, 13, 'bold'),
                                  fg=COLORS['yellow'], bg=COLORS['panel'])
        self.lbl_align.pack(pady=(8, 4))

        self.val_work   = self._row(p, '工作字节')
        self.val_comb   = self._row(p, '组合状态')
        self.val_temp   = self._row(p, '内部温度')
        tk.Frame(p, bg=COLORS['panel'], height=6).pack()

    def _build_position_panel(self, parent):
        p = self._panel(parent, '位置 / GNSS')
        self.val_lat      = self._row(p, 'INS 纬度')
        self.val_lon      = self._row(p, 'INS 经度')
        self.val_gnss_lat = self._row(p, 'GNSS 纬度')
        self.val_gnss_lon = self._row(p, 'GNSS 经度')
        self.val_alt      = self._row(p, '海拔高度')
        self.val_gnss_fix = self._row(p, 'GNSS 状态')
        self.val_sats     = self._row(p, '卫星数量')
        self.val_hdop     = self._row(p, 'HDOP')
        tk.Frame(p, bg=COLORS['panel'], height=6).pack()

    def _build_attitude_panel(self, parent):
        p = self._panel(parent, '姿态 / 速度')
        self.val_roll  = self._row(p, 'Roll')
        self.val_pitch = self._row(p, 'Pitch')
        self.val_yaw   = self._row(p, 'Yaw')
        self.val_vn    = self._row(p, '北向速度')
        self.val_ve    = self._row(p, '东向速度')
        self.val_vd    = self._row(p, '天向速度')
        tk.Frame(p, bg=COLORS['panel'], height=6).pack()

    def _build_control_panel(self, parent):
        p = self._panel(parent, 'INS 控制')

        # 大按钮：停止 / 启动
        btn_row = tk.Frame(p, bg=COLORS['panel'])
        btn_row.pack(fill=tk.X, padx=10, pady=10)

        self.btn_stop = tk.Button(
            btn_row, text='[停止 INS]',
            font=(FONT_UI, 12, 'bold'),
            fg='#1e1e2e', bg=COLORS['btn_stop'],
            activebackground='#eb6f82',
            relief='flat', cursor='hand2', bd=0,
            command=self._cmd_stop)
        self.btn_stop.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=10, padx=(0, 4))

        self.btn_start = tk.Button(
            btn_row, text='[启动 INS]',
            font=(FONT_UI, 12, 'bold'),
            fg='#1e1e2e', bg=COLORS['btn_start'],
            activebackground='#7acc7a',
            relief='flat', cursor='hand2', bd=0,
            command=self._cmd_start)
        self.btn_start.pack(side=tk.LEFT, fill=tk.X, expand=True, ipady=10, padx=(4, 0))

        # 位置设置
        sep = tk.Frame(p, bg=COLORS['border'], height=1)
        sep.pack(fill=tk.X, padx=10, pady=4)

        tk.Label(p, text='设置初始位置',
                 font=(FONT_UI, 10, 'bold'),
                 fg=COLORS['text'], bg=COLORS['panel']).pack(anchor='w', padx=10, pady=(4, 2))

        pos_grid = tk.Frame(p, bg=COLORS['panel'])
        pos_grid.pack(fill=tk.X, padx=10, pady=4)

        # 纬度
        tk.Label(pos_grid, text='纬度:', font=(FONT_UI, 9),
                 fg=COLORS['subtext'], bg=COLORS['panel']).grid(row=0, column=0, sticky='w', pady=3)
        self.entry_lat = tk.Entry(pos_grid,
                                  font=(FONT_MONO, 10),
                                  bg=COLORS['bg'], fg=COLORS['text'],
                                  insertbackground=COLORS['text'],
                                  relief='flat', bd=4, width=18)
        self.entry_lat.grid(row=0, column=1, padx=(6, 0), pady=3, sticky='ew')
        self.entry_lat.insert(0, '31.2345670')

        # 经度
        tk.Label(pos_grid, text='经度:', font=(FONT_UI, 9),
                 fg=COLORS['subtext'], bg=COLORS['panel']).grid(row=1, column=0, sticky='w', pady=3)
        self.entry_lon = tk.Entry(pos_grid,
                                  font=(FONT_MONO, 10),
                                  bg=COLORS['bg'], fg=COLORS['text'],
                                  insertbackground=COLORS['text'],
                                  relief='flat', bd=4, width=18)
        self.entry_lon.grid(row=1, column=1, padx=(6, 0), pady=3, sticky='ew')
        self.entry_lon.insert(0, '121.4567890')

        # 高度
        tk.Label(pos_grid, text='高度(m):', font=(FONT_UI, 9),
                 fg=COLORS['subtext'], bg=COLORS['panel']).grid(row=2, column=0, sticky='w', pady=3)
        self.entry_alt = tk.Entry(pos_grid,
                                  font=(FONT_MONO, 10),
                                  bg=COLORS['bg'], fg=COLORS['text'],
                                  insertbackground=COLORS['text'],
                                  relief='flat', bd=4, width=18)
        self.entry_alt.grid(row=2, column=1, padx=(6, 0), pady=3, sticky='ew')
        self.entry_alt.insert(0, '0.0')

        pos_grid.columnconfigure(1, weight=1)

        self.btn_setpos = tk.Button(
            p, text='[设置位置]',
            font=(FONT_UI, 11, 'bold'),
            fg='#1e1e2e', bg=COLORS['btn_pos'],
            activebackground='#6aa3e8',
            relief='flat', cursor='hand2', bd=0,
            command=self._cmd_setpos)
        self.btn_setpos.pack(fill=tk.X, padx=10, ipady=8, pady=6)

        self.btn_status = tk.Button(
            p, text='[查询状态]',
            font=(FONT_UI, 10),
            fg=COLORS['text'], bg=COLORS['panel'],
            activebackground=COLORS['border'],
            relief='solid', cursor='hand2', bd=1,
            command=self._cmd_status)
        self.btn_status.pack(fill=tk.X, padx=10, ipady=5, pady=(0, 10))

    def _build_log_panel(self, parent):
        p = self._panel(parent, '操作日志')
        p.pack(fill=tk.BOTH, expand=True, pady=3)

        self.log_text = tk.Text(
            p, height=10,
            font=(FONT_MONO, 9),
            bg=COLORS['bg'], fg=COLORS['text'],
            insertbackground=COLORS['text'],
            relief='flat', bd=4,
            state='disabled', wrap='word')
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # 颜色标签
        self.log_text.tag_config('ok',   foreground=COLORS['green'])
        self.log_text.tag_config('err',  foreground=COLORS['red'])
        self.log_text.tag_config('info', foreground=COLORS['yellow'])
        self.log_text.tag_config('time', foreground=COLORS['subtext'])

    # ── 命令处理 ──────────────────────────────────────────────
    def _set_buttons_state(self, state):
        for b in [self.btn_stop, self.btn_start, self.btn_setpos, self.btn_status]:
            b.config(state=state)

    def _cmd_stop(self):
        self._log('发送停止命令...', 'info')
        self._set_buttons_state('disabled')
        threading.Thread(target=self._run_cmd, args=('stop',), daemon=True).start()

    def _cmd_start(self):
        self._log('发送启动命令...', 'info')
        self._set_buttons_state('disabled')
        threading.Thread(target=self._run_cmd, args=('start',), daemon=True).start()

    def _cmd_setpos(self):
        try:
            lat = float(self.entry_lat.get().strip())
            lon = float(self.entry_lon.get().strip())
            alt = float(self.entry_alt.get().strip())
        except ValueError:
            messagebox.showerror('输入错误', '请输入有效的纬度/经度/高度数值')
            return
        self._log(f'发送位置设置: lat={lat:.7f}  lon={lon:.7f}  alt={alt:.1f}', 'info')
        self._set_buttons_state('disabled')
        threading.Thread(
            target=self._run_cmd, args=('setpos', lat, lon, alt), daemon=True).start()

    def _cmd_status(self):
        self._log('查询当前状态...', 'info')
        self._set_buttons_state('disabled')
        threading.Thread(target=self._run_cmd, args=('status',), daemon=True).start()

    def _run_cmd(self, cmd, lat=None, lon=None, alt=0.0):
        ok, msg = send_ros_command(cmd, lat, lon, alt)
        tag = 'ok' if ok else 'err'
        self.root.after(0, self._log, msg.strip(), tag)
        self.root.after(0, self._set_buttons_state, 'normal')
        self.root.after(0, self._update_statusbar, '就绪' if ok else '⚠ 命令失败')

    # ── 数据刷新 ──────────────────────────────────────────────
    def _refresh(self):
        d = self.data
        try:
            # ROS 连接状态
            if not ROS_AVAILABLE:
                self.lbl_ros.config(text='● ROS2 不可用', fg=COLORS['red'])
            elif d:
                self.lbl_ros.config(text='● ROS2 已连接', fg=COLORS['green'])
            else:
                self.lbl_ros.config(text='● 等待数据...', fg=COLORS['yellow'])

            # 对准状态
            align = d.get('align', -1)
            name, color = ALIGN_STATUS.get(align, ('--', COLORS['subtext']))
            self.lbl_align.config(text=f'◉  {name}', fg=color)

            # 系统状态
            self.val_work.config(text=f"0x{d.get('work', 0):02X}")
            self.val_comb.config(text=f"0x{d.get('comb', 0):02X}")
            temp = d.get('temp', None)
            self.val_temp.config(
                text=f"{temp} °C" if temp is not None else '--',
                fg=COLORS['red'] if temp and temp > 70 else COLORS['text'])

            # 位置
            lat = d.get('lat');  lon = d.get('lon')
            self.val_lat.config(text=f'{lat:.7f}°' if lat else '--')
            self.val_lon.config(text=f'{lon:.7f}°' if lon else '--')
            glat = d.get('gnss_lat'); glon = d.get('gnss_lon')
            self.val_gnss_lat.config(text=f'{glat:.7f}°' if glat else '--')
            self.val_gnss_lon.config(text=f'{glon:.7f}°' if glon else '--')
            alt = d.get('alt')
            self.val_alt.config(text=f'{alt:.2f} m' if alt else '--')

            # GNSS
            fix = d.get('fix', -1)
            fix_name, fix_color = GNSS_FIX.get(fix, ('--', COLORS['subtext']))
            self.val_gnss_fix.config(text=fix_name, fg=fix_color)
            sats = d.get('sats')
            self.val_sats.config(text=str(sats) if sats is not None else '--',
                                 fg=COLORS['red'] if sats is not None and sats < 4 else COLORS['text'])
            hdop = d.get('hdop')
            self.val_hdop.config(text=f'{hdop:.2f}' if hdop else '--',
                                 fg=COLORS['red'] if hdop and hdop > 3.0 else COLORS['text'])

            # 姿态
            pose = d.get('pose')
            if pose:
                roll, pitch, yaw = pose
                self.val_roll.config(text=f'{roll:.3f}°')
                self.val_pitch.config(text=f'{pitch:.3f}°')
                self.val_yaw.config(text=f'{yaw:.3f}°')

            # 速度
            twist = d.get('twist')
            if twist:
                vn, ve, vd = twist
                self.val_vn.config(text=f'{vn:.3f} m/s')
                self.val_ve.config(text=f'{ve:.3f} m/s')
                self.val_vd.config(text=f'{vd:.3f} m/s')

        except Exception as e:
            pass

        self.root.after(500, self._refresh)

    # ── 日志 ─────────────────────────────────────────────────
    def _log(self, msg, tag='info'):
        if not msg.strip():
            return
        ts = time.strftime('%H:%M:%S')
        self.log_text.config(state='normal')
        self.log_text.insert('end', f'[{ts}] ', 'time')
        self.log_text.insert('end', msg + '\n', tag)
        self.log_text.see('end')
        self.log_text.config(state='disabled')

    def _update_statusbar(self, msg):
        self.status_bar.config(text=f'  {msg}')

    # ── 退出 ─────────────────────────────────────────────────
    def _on_close(self):
        self.ros_bridge.stop()
        self.root.destroy()


def main():
    INSGui()


if __name__ == '__main__':
    main()
