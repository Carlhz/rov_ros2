#!/usr/bin/env python3
"""
声纳终端监控节点 (运行于 VM Ubuntu 上位机)
============================================
订阅 RK3588 上 sonar_omni_driver 发布的 3 个 PointCloud2 话题，
彩色终端实时显示声纳数据。

话题:
  /sonar/omni/original  — 所有有效回波点云
  /sonar/omni/rigidity  — 差分刚性检测点云
  /sonar/omni/boundary  — 底部边界点

环境变量:
  ROS_DOMAIN_ID=42
  ROS_LOCALHOST_ONLY=0

用法 (VM 上):
  cd ~/rov_ros2_ws
  source /opt/ros/foxy/setup.bash
  source install/local_setup.bash   # 需要接口包编译过 (可选, PointCloud2 是标准类型)
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=0
  python3 sonar_monitor.py
"""

import sys
import os
import time
import signal
import threading
import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

# ========== ANSI 颜色 ==========
C_RESET  = "\033[0m"
C_BOLD   = "\033[1m"
C_DIM    = "\033[2m"
C_RED    = "\033[91m"
C_GREEN  = "\033[92m"
C_YELLOW = "\033[93m"
C_BLUE   = "\033[94m"
C_MAGENTA= "\033[95m"
C_CYAN   = "\033[96m"
C_WHITE  = "\033[97m"
C_BG_BLUE = "\033[44m"

# ========== 移动终端光标 ==========
def cursor_up(n=1):   return f"\033[{n}A"
def cursor_hide():    return "\033[?25l"
def cursor_show():    return "\033[?25h"
def clear_screen():   return "\033[2J\033[H"
def line_clear():     return "\033[2K"


class SonarMonitorNode(Node):
    """订阅声纳点云话题，统计并缓存最新数据"""

    def __init__(self):
        super().__init__("sonar_monitor")

        # ---- 数据缓存 ----
        self.lock = threading.Lock()

        # 当前帧数据
        self.orig_count   = 0       # original 点数
        self.orig_max_int = 0.0     # 最大强度
        self.orig_range   = 0.0     # 最远回波距离
        self.orig_angle   = 0.0     # 当前扫描角度

        self.rig_count    = 0
        self.rig_max_rig  = 0.0

        self.bnd_range    = 0.0     # 边界距离
        self.bnd_angle    = 0.0

        # 统计
        self.orig_total   = 0
        self.rig_total    = 0
        self.bnd_total    = 0
        self.frames       = 0
        self.start_time   = time.time()
        self.last_frame   = time.time()

        # 最近 50 帧的边界数据 (画剖面用)
        self.bnd_history  = deque(maxlen=50)

        # ---- 订阅者 ----
        self.create_subscription(PointCloud2, "sonar/omni/original",
                                  self._cb_original, 10)
        self.create_subscription(PointCloud2, "sonar/omni/rigidity",
                                  self._cb_rigidity, 10)
        self.create_subscription(PointCloud2, "sonar/omni/boundary",
                                  self._cb_boundary, 10)

    # ========== 回调 ==========

    def _cb_original(self, msg: PointCloud2):
        with self.lock:
            n = msg.width
            self.orig_count = n
            self.frames += 1
            self.last_frame = time.time()

            if n == 0:
                return

            # 解析点云: point_step=16 (x,y,z,intensity 各 float32)
            data = msg.data
            step = msg.point_step  # 16

            max_int = 0.0
            max_range = 0.0
            # 取第一个点的 x,y 算角度 (所有点同角度)
            if len(data) >= 8:
                x = _bytes_to_float(data, 0)
                y = _bytes_to_float(data, 4)
                self.orig_angle = math.atan2(-y, x)

            for i in range(n):
                off = i * step
                if off + 16 > len(data):
                    break
                x = _bytes_to_float(data, off + 0)
                y = _bytes_to_float(data, off + 4)
                intensity = _bytes_to_float(data, off + 12)
                r = math.sqrt(x*x + y*y)
                if r > max_range:
                    max_range = r
                if intensity > max_int:
                    max_int = intensity

            self.orig_max_int = max_int
            self.orig_range   = max_range

    def _cb_rigidity(self, msg: PointCloud2):
        with self.lock:
            n = msg.width
            self.rig_count = n
            if n == 0:
                return

            data = msg.data
            step = msg.point_step

            # 取第一个点角度
            if len(data) >= 8:
                x = _bytes_to_float(data, 0)
                y = _bytes_to_float(data, 4)
                self.orig_angle = math.atan2(-y, x)

            max_rig = 0.0
            for i in range(n):
                off = i * step
                if off + 16 > len(data):
                    break
                rig = _bytes_to_float(data, off + 12)  # intensity 字段存的是 rigidity
                if rig > max_rig:
                    max_rig = rig
            self.rig_max_rig = max_rig

    def _cb_boundary(self, msg: PointCloud2):
        with self.lock:
            n = msg.width
            self.bnd_total += n
            if n == 0:
                return

            data = msg.data
            if len(data) >= 8:
                x = _bytes_to_float(data, 0)
                y = _bytes_to_float(data, 4)
                r = math.sqrt(x*x + y*y)
                ang = math.atan2(-y, x)
                self.bnd_range = r
                self.bnd_angle = ang
                self.bnd_history.append((math.degrees(ang), r))


def _bytes_to_float(data: bytes, offset: int) -> float:
    """小端 float32 解码"""
    import struct
    return struct.unpack_from("<f", data, offset)[0]


# ========== 终端渲染 ==========

def render_dashboard(node: SonarMonitorNode):
    """单帧渲染终端仪表板"""

    with node.lock:
        orig_n   = node.orig_count
        orig_int = node.orig_max_int
        orig_r   = node.orig_range
        orig_ang = node.orig_angle
        rig_n    = node.rig_count
        rig_max  = node.rig_max_rig
        bnd_r    = node.bnd_range
        bnd_ang  = node.bnd_angle
        bnd_hist = list(node.bnd_history)
        frames   = node.frames
        elapsed  = time.time() - node.start_time
        since    = time.time() - node.last_frame

    fps = frames / elapsed if elapsed > 0 else 0.0

    # ---- 状态颜色 ----
    if since < 0.5:
        status_color = C_GREEN
        status_text  = "● 在线"
    elif since < 3.0:
        status_color = C_YELLOW
        status_text  = "◐ 延迟"
    else:
        status_color = C_RED
        status_text  = "○ 离线"

    # ---- 角度条 ----
    ang_deg = math.degrees(orig_ang) % 360
    ang_bar = _bar(ang_deg / 360.0, 30, C_CYAN)

    # ---- 渲染 ----
    lines = []
    lines.append("")
    lines.append(f"  {C_BOLD}{C_BG_BLUE}  Scanfish-II 全向声纳监控  {C_RESET}  "
                 f"{status_color}{status_text}{C_RESET}  "
                 f"{C_DIM}刷新: {fps:5.1f} Hz  运行: {int(elapsed)}s{C_RESET}")
    lines.append("")
    lines.append(f"  {C_BOLD}扫描角度{C_RESET}  {ang_bar}  {ang_deg:6.1f}°")
    lines.append("")

    # ---- Original 点云 ----
    lines.append(f"  {C_BOLD}{C_BLUE}═══ Original 原始点云 ═══{C_RESET}")
    lines.append(f"   点数:  {orig_n:>6d}     最大强度: {orig_int:>6.0f}     最远距离: {orig_r:>6.2f} m")
    lines.append(f"   强度柱状图: {_histogram_bars(node)}")
    lines.append("")

    # ---- Rigidity 点云 ----
    lines.append(f"  {C_BOLD}{C_MAGENTA}═══ Rigidity 刚性点云 ═══{C_RESET}")
    lines.append(f"   点数:  {rig_n:>6d}     最大差分: {rig_max:>6.0f}")
    lines.append("")

    # ---- Boundary 边界 ----
    lines.append(f"  {C_BOLD}{C_GREEN}═══ Boundary 底部边界 ═══{C_RESET}")
    lines.append(f"   距离:  {bnd_r:>6.2f} m  (角度 {math.degrees(bnd_ang)%360:6.1f}°)")
    lines.append("")

    # ---- 底部剖面图 ----
    if bnd_hist:
        lines.append(f"  {C_BOLD}{C_YELLOW}═══ 底部剖面 (最近 {len(bnd_hist)} 帧) ═══{C_RESET}")
        lines.append(_draw_profile(bnd_hist, width=60, height=12))
        lines.append("")

    lines.append(f"  {C_DIM}按 Ctrl+C 退出{C_RESET}")
    lines.append("")

    return "\n".join(lines)


def _bar(ratio: float, width: int, color: str) -> str:
    """生成彩色进度条"""
    filled = int(ratio * width)
    if filled < 0: filled = 0
    if filled > width: filled = width
    return f"{color}{'█' * filled}{C_DIM}{'░' * (width - filled)}{C_RESET}"


def _histogram_bars(node: SonarMonitorNode) -> str:
    """简单强度分布指示 (0-255 强度分段)"""
    # 这里简化，只显示最大强度比例
    ratio = node.orig_max_int / 255.0 if node.orig_max_int > 0 else 0
    return _bar(ratio, 20, C_YELLOW) + f"  {node.orig_max_int:.0f} / 255"


def _draw_profile(points: list, width: int = 60, height: int = 12) -> str:
    """
    绘制底部剖面图 (ASCII art)
    points: [(angle_deg, range_m), ...]
    """
    if not points:
        return "  无数据"

    ranges = [p[1] for p in points]
    angles = [p[0] for p in points]
    max_r = max(ranges) if ranges else 10
    min_r = min(ranges) if ranges else 0
    ang_min = min(angles)
    ang_max = max(angles)

    if max_r == min_r:
        max_r = min_r + 1

    # 创建画布
    canvas = [[" "] * width for _ in range(height)]

    # 格线
    for row in range(height):
        for col in [0, width//4, width//2, 3*width//4, width-1]:
            if canvas[row][col] == " ":
                canvas[row][col] = C_DIM + "·" + C_RESET

    for col in range(width):
        for row in [0, height//3, 2*height//3, height-1]:
            if canvas[row][col] in (" ", C_DIM + "·" + C_RESET):
                canvas[row][col] = C_DIM + "·" + C_RESET

    # 左轴标签
    for i in range(0, height, max(1, height//4)):
        r = max_r - (i / (height-1)) * (max_r - min_r)
        label = f"{r:.1f}"
        for j, ch in enumerate(label[:4]):
            if j < width:
                canvas[i][j] = C_DIM + ch + C_RESET

    # 绘制点
    if ang_max > ang_min:
        for ang, rng in points:
            col = int((ang - ang_min) / (ang_max - ang_min) * (width - 1))
            row = int((max_r - rng) / (max_r - min_r) * (height - 1))
            col = max(0, min(width-1, col))
            row = max(0, min(height-1, row))
            canvas[row][col] = C_GREEN + "█" + C_RESET
    else:
        # 单角度: 画中轴线
        mid = width // 2
        for ang, rng in points:
            row = int((max_r - rng) / (max_r - min_r) * (height - 1))
            row = max(0, min(height-1, row))
            canvas[row][mid] = C_GREEN + "█" + C_RESET

    lines = []
    for row in range(height-1, -1, -1):
        line = "".join(canvas[row])
        # ANSI 序列不计宽度，手动补空格对齐
        visible = ""
        skip = 0
        for i, ch in enumerate(line):
            if skip > 0:
                skip -= 1
                continue
            if ch == "\033":
                # 跳过整个 ANSI 序列
                end = line.index("m", i)
                skip = end - i
                continue
            visible += ch
        pad = width - len(visible)
        lines.append(f"    {line}{' ' * max(0, pad)}")

    # 底部角度标签
    bottom = "    " + C_DIM + f"{ang_min:.0f}°" + C_RESET
    bottom += " " * (width - 8)
    bottom += C_DIM + f"{ang_max:.0f}°" + C_RESET
    lines.append(bottom)

    return "\n".join(lines)


# ========== 入口 ==========

def main():
    rclpy.init(args=None)
    node = SonarMonitorNode()

    # 检查 DDS 配置
    domain = os.environ.get("ROS_DOMAIN_ID", "(未设置)")
    localonly = os.environ.get("ROS_LOCALHOST_ONLY", "(未设置)")
    node.get_logger().info(
        f"声纳监控启动 | DOMAIN_ID={domain} | LOCALHOST_ONLY={localonly}"
    )

    # 隐藏光标
    sys.stdout.write(cursor_hide() + clear_screen())
    sys.stdout.flush()

    # 显示线程
    spinner = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]
    spinner_idx = [0]

    def _spin_once():
        """非阻塞 spin + 终端刷新"""
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        while rclpy.ok():
            _spin_once()

            # 每 100ms 刷新一次显示
            if spinner_idx[0] % 2 == 0:
                output = render_dashboard(node)
                sys.stdout.write(clear_screen() + output)
                sys.stdout.flush()

            spinner_idx[0] += 1

    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(cursor_show() + clear_screen())
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()
        print("声纳监控已退出。")


if __name__ == "__main__":
    main()
