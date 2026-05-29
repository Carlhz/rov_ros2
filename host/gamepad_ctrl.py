#!/usr/bin/env python3
"""
gamepad_ctrl.py  —  主机手柄 → SSH → 远端电机守护进程

依赖：
    pip install pygame paramiko

手柄映射（Xbox/通用布局，可在 MAPPING 部分修改）：
    左摇杆 Y 轴 (Axis 1)  → 前进/后退速度
    右摇杆 X 轴 (Axis 2)  → 左转/右转（差速）
    LT/RT (Axis 2/5)      → 油门（可选模式）
    B 键 / Circle         → 紧急停止
    Start                 → 退出程序

用法：
    python gamepad_ctrl.py --host 192.168.1.100 --user pi --password yourpass
    python gamepad_ctrl.py --host 192.168.1.100 --user pi --key ~/.ssh/id_rsa
"""

import argparse
import sys
import time
import threading
import subprocess

try:
    import pygame
except ImportError:
    print("[ERROR] pygame not found. Install: pip install pygame")
    sys.exit(1)

try:
    import paramiko
except ImportError:
    print("[ERROR] paramiko not found. Install: pip install paramiko")
    sys.exit(1)

# ─────────────────────────────────────────────
# 配置区（根据实际情况修改）
# ─────────────────────────────────────────────

# 远端守护进程路径
REMOTE_CMD = "sudo /home/pi/can_demo_daemon"   # 改成你的实际路径

# 手柄轴编号（pygame axis index）
# 运行 python gamepad_ctrl.py --list-axes 可查看你的手柄布局
AXIS_FORWARD  = 1    # 左摇杆 Y 轴（上推=负，下拉=正，需要取反）
AXIS_TURN     = 3    # 右摇杆 X 轴（左=负，右=正）

# 手柄按钮编号
BTN_ESTOP     = 1    # B键/Circle — 紧急停止
BTN_QUIT      = 7    # Start 键 — 退出

# 速度参数
MAX_RPM       = 2000  # 最大转速（rpm）
DEADZONE      = 0.08  # 摇杆死区（0~1）

# 发送频率（Hz）
SEND_HZ       = 20    # 每秒发 20 条指令（50ms 间隔，远端心跳 200ms）

# ─────────────────────────────────────────────
# 工具函数
# ─────────────────────────────────────────────

def apply_deadzone(v, dz):
    """摇杆死区处理"""
    if abs(v) < dz:
        return 0.0
    sign = 1 if v > 0 else -1
    return sign * (abs(v) - dz) / (1.0 - dz)

def joystick_to_rpm(forward_axis, turn_axis):
    """
    差速转换：
        forward_axis: -1 (满前进) ~ +1 (满后退)
        turn_axis:    -1 (左)     ~ +1 (右)
    返回 (left_rpm, right_rpm)
    """
    fwd   = -apply_deadzone(forward_axis, DEADZONE)  # 取反：向前推=正
    turn  =  apply_deadzone(turn_axis,    DEADZONE)

    # 混合：left = fwd - turn,  right = fwd + turn
    # 然后归一化到 [-1, 1]
    left  = fwd - turn
    right = fwd + turn

    # 如果超过1，等比例缩放
    max_val = max(abs(left), abs(right), 1.0)
    left  /= max_val
    right /= max_val

    left_rpm  = int(left  * MAX_RPM)
    right_rpm = int(right * MAX_RPM)

    # 履带：前进时左右方向相反（根据实际安装调整）
    return left_rpm, -right_rpm

def list_axes():
    """列出所有手柄轴和按钮，用于调试映射"""
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick found!")
        return
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Joystick: {js.get_name()}")
    print(f"  Axes: {js.get_numaxes()}")
    print(f"  Buttons: {js.get_numbuttons()}")
    print("\nMove axes and press buttons to see values. Ctrl+C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            axes = [js.get_axis(i) for i in range(js.get_numaxes())]
            btns = [js.get_button(i) for i in range(js.get_numbuttons())]
            axis_str = "  ".join(f"A{i}={v:+.2f}" for i, v in enumerate(axes))
            btn_str  = "  ".join(f"B{i}={v}" for i, v in enumerate(btns) if v)
            print(f"\r{axis_str}  [{btn_str}]          ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()

# ─────────────────────────────────────────────
# SSH 连接管理
# ─────────────────────────────────────────────

class SSHMotorBridge:
    """通过 SSH 连接到远端，向 stdin 发送指令"""

    def __init__(self, host, port, user, password=None, key_file=None):
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.key_file = key_file

        self.client = None
        self.channel = None
        self._lock = threading.Lock()
        self._connected = False

    def connect(self):
        print(f"[SSH] Connecting to {self.user}@{self.host}:{self.port} ...")
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        connect_kwargs = dict(
            hostname=self.host,
            port=self.port,
            username=self.user,
            timeout=10,
        )
        if self.key_file:
            connect_kwargs["key_filename"] = self.key_file
        elif self.password:
            connect_kwargs["password"] = self.password

        self.client.connect(**connect_kwargs)

        # 打开一个 interactive shell channel
        transport = self.client.get_transport()
        self.channel = transport.open_session()
        self.channel.get_pty()          # 需要 pty 才能 sudo
        self.channel.invoke_shell()

        # 等待 shell 就绪
        time.sleep(0.5)
        # 如果 sudo 需要密码，自动输入
        if self.password:
            out = self._drain(timeout=1.0)
            if "password" in out.lower():
                self.channel.send(self.password + "\n")
                time.sleep(0.5)

        # 启动守护进程
        self.channel.send(REMOTE_CMD + "\n")
        time.sleep(1.0)
        startup = self._drain(timeout=2.0)
        print(f"[SSH] Remote output:\n{startup}")

        if "Ready" not in startup and "ready" not in startup:
            print("[SSH] Warning: daemon may not have started correctly.")
        else:
            print("[SSH] Daemon started successfully.")

        self._connected = True
        print("[SSH] Connected and ready.")

    def _drain(self, timeout=0.5):
        """读取 channel 中现有的输出"""
        buf = ""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.channel.recv_ready():
                chunk = self.channel.recv(4096).decode("utf-8", errors="replace")
                buf += chunk
            else:
                time.sleep(0.05)
        return buf

    def send_cmd(self, cmd: str):
        """发送一条指令（不含换行，内部自动加）"""
        if not self._connected:
            return
        with self._lock:
            try:
                self.channel.send(cmd + "\n")
            except Exception as e:
                print(f"[SSH] Send error: {e}")
                self._connected = False

    def disconnect(self):
        if self._connected:
            self.send_cmd("quit")
            time.sleep(0.2)
        self._connected = False
        if self.channel:
            self.channel.close()
        if self.client:
            self.client.close()
        print("[SSH] Disconnected.")

# ─────────────────────────────────────────────
# 本地模式（调试用，不连 SSH，只打印指令）
# ─────────────────────────────────────────────

class LocalPrintBridge:
    def connect(self): print("[LOCAL] Debug mode: commands will be printed only.")
    def send_cmd(self, cmd): print(f">> {cmd}")
    def disconnect(self): print("[LOCAL] Done.")

# ─────────────────────────────────────────────
# 主循环
# ─────────────────────────────────────────────

def run(bridge):
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick/gamepad found!")
        bridge.disconnect()
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[JOY] Using: {js.get_name()}  "
          f"({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")

    interval = 1.0 / SEND_HZ
    estop    = False
    prev_l   = None
    prev_r   = None

    print("[CTRL] Running. Press B/Circle=EStop, Start=Quit")

    try:
        while True:
            t0 = time.time()
            pygame.event.pump()

            # 检查退出
            if js.get_button(BTN_QUIT):
                print("[CTRL] Quit button pressed.")
                break

            # 紧急停止
            if js.get_button(BTN_ESTOP):
                if not estop:
                    bridge.send_cmd("stop")
                    print("[CTRL] !!! EMERGENCY STOP !!!")
                    estop = True
                time.sleep(interval)
                continue
            else:
                estop = False

            # 读摇杆
            fwd_axis  = js.get_axis(AXIS_FORWARD)
            turn_axis = js.get_axis(AXIS_TURN)

            l_rpm, r_rpm = joystick_to_rpm(fwd_axis, turn_axis)

            # 只在值变化时发送（避免无意义的重复发送）
            if l_rpm != prev_l or r_rpm != prev_r:
                cmd = f"move {l_rpm} {r_rpm}"
                bridge.send_cmd(cmd)
                prev_l = l_rpm
                prev_r = r_rpm

            # 精确计时
            elapsed = time.time() - t0
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[CTRL] Ctrl+C, stopping...")

    bridge.send_cmd("stop")
    time.sleep(0.2)
    bridge.disconnect()
    pygame.quit()
    print("[CTRL] Exited.")

# ─────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Gamepad → SSH → Motor Daemon")
    parser.add_argument("--host",     default="",          help="远端 IP 地址")
    parser.add_argument("--port",     type=int, default=22, help="SSH 端口（默认22）")
    parser.add_argument("--user",     default="pi",         help="SSH 用户名")
    parser.add_argument("--password", default="",           help="SSH 密码")
    parser.add_argument("--key",      default="",           help="SSH 私钥路径")
    parser.add_argument("--local",    action="store_true",  help="本地调试模式（不连 SSH，只打印指令）")
    parser.add_argument("--list-axes",action="store_true",  help="列出手柄轴和按钮编号（调试映射用）")
    parser.add_argument("--max-rpm",  type=int, default=MAX_RPM, help=f"最大转速 rpm（默认{MAX_RPM}）")
    args = parser.parse_args()

    if args.list_axes:
        list_axes()
        return

    global MAX_RPM
    MAX_RPM = args.max_rpm

    if args.local:
        bridge = LocalPrintBridge()
        bridge.connect()
        run(bridge)
        return

    if not args.host:
        parser.print_help()
        print("\n[ERROR] --host is required (or use --local for debug mode)")
        sys.exit(1)

    bridge = SSHMotorBridge(
        host=args.host,
        port=args.port,
        user=args.user,
        password=args.password or None,
        key_file=args.key or None,
    )
    bridge.connect()
    run(bridge)

if __name__ == "__main__":
    main()
