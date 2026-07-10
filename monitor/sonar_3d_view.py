#!/usr/bin/env python3
"""
Sonar Omni 3D Point Cloud - Real-time Visualization (V12)
=========================================================
Angular ring buffer: keeps exactly one 360 sweep on screen.
As the sonar rotates past an angle, old points are replaced
in-place — the sweep paints and erases like a radar PPI.

Usage (VM desktop terminal):
  cd ~/rov_ros2_ws
  source /opt/ros/foxy/setup.bash
  export ROS_DOMAIN_ID=0
  python3 monitor/sonar_3d_view.py
"""

import os
import time
import math
import struct
import threading
import argparse

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2


# ============================================================
#  ROS2 data cache
# ============================================================
class SonarCache(Node):
    def __init__(self, topic="sonar/omni/original"):
        super().__init__("sonar_3d_view")
        self._lock = threading.Lock()
        self._points = []
        self._angle = 0.0
        self._fps = 0.0
        self._frame_count = 0
        self._last_sec = time.time()
        self.sub = self.create_subscription(PointCloud2, topic, self._cb, 10)
        self.get_logger().info(f"Subscribed: {topic}")

    def _cb(self, msg):
        n = msg.width
        if n == 0:
            return
        data, step = msg.data, msg.point_step
        pts = []
        for i in range(n):
            off = i * step
            if off + 16 > len(data):
                break
            pts.append((
                struct.unpack_from("<f", data, off)[0],
                struct.unpack_from("<f", data, off + 4)[0],
                struct.unpack_from("<f", data, off + 8)[0],
                struct.unpack_from("<f", data, off + 12)[0],
            ))
        now = time.time()
        with self._lock:
            self._points = pts
            if pts:
                self._angle = math.degrees(
                    math.atan2(-pts[0][1], pts[0][0])) % 360
            self._frame_count += 1
            dt = now - self._last_sec
            if dt >= 1.0:
                self._fps = self._frame_count / dt
                self._frame_count = 0
                self._last_sec = now

    def snapshot(self):
        with self._lock:
            return list(self._points), self._angle, self._fps


# ============================================================
#  Visualization — angular ring buffer (1 sweep = 360 bins)
# ============================================================
def run_visualizer(node, max_range=5.0, bin_count=360):
    """Each angle bin holds the latest frame at that heading.
    When the sonar sweeps past an angle, old points are replaced
    — exactly one full sweep stays on screen at all times."""
    import matplotlib
    try:
        import tkinter
        matplotlib.use("TkAgg")
    except Exception:
        pass

    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation

    # ---------- figure + axes ----------
    fig = plt.figure("Sonar Omni 3D Point Cloud", figsize=(10, 8),
                     facecolor="#0a0a0f")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0a0a0f")
    ax.view_init(elev=35, azim=-65)

    half_z = max(0.5, max_range * 0.1)
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-half_z, half_z)
    ax.set_xlabel("X (m)", color="#aaaaaa", fontsize=9)
    ax.set_ylabel("Y (m)", color="#aaaaaa", fontsize=9)
    ax.set_zlabel("Z", color="#555555", fontsize=8)
    ax.tick_params(colors="#888888", labelsize=7)

    for a in (ax.xaxis, ax.yaxis, ax.zaxis):
        a.pane.fill = False
        a.pane.set_edgecolor("#1a1a2e")
        a._axinfo["grid"]["color"] = "#151530"
        a._axinfo["grid"]["linewidth"] = 0.5

    ax.set_title("Sonar Omni - 3D Sweep View  (1 revolution)",
                 color="white", fontsize=13, fontweight="bold", pad=18)

    # Reference rings
    for r in [rad for rad in [1, 3, 5] if rad <= max_range]:
        th = np.linspace(0, 2 * np.pi, 360)
        ax.plot(r * np.cos(th), r * np.sin(th), np.zeros(360),
                color="#1a1a33", linewidth=0.6, alpha=0.6)

    # Origin marker
    ax.plot([0], [0], [0], "s", color="#00ffff", markersize=8,
            markeredgecolor="white", markeredgewidth=0.8)

    # Point cloud scatter (created once, updated via _offsets3d)
    sc = ax.scatter([], [], [], c=[], cmap="inferno",
                    vmin=0, vmax=255, s=9, alpha=0.85, edgecolors="none")

    # Colorbar
    cbar = fig.colorbar(sc, ax=ax, pad=0.12, shrink=0.6)
    cbar.set_label("Intensity", color="#aaaaaa", fontsize=9)
    cbar.ax.tick_params(labelsize=7, colors="#aaaaaa")
    cbar.outline.set_edgecolor("#444466")

    # HUD text (top-left)
    txt = ax.text2D(0.02, 0.98, "WAITING for sonar data...",
                    transform=ax.transAxes, color="white", fontsize=9,
                    family="monospace", va="top",
                    bbox=dict(boxstyle="round,pad=0.3",
                              facecolor="#0a0a0f",
                              edgecolor="#444466", alpha=0.85))

    # Tip text
    ax.text2D(0.98, 0.02, "[C] clear  [R] reset view",
              transform=ax.transAxes, color="#666688", fontsize=8,
              family="monospace", ha="right",
              bbox=dict(boxstyle="round,pad=0.3",
                        facecolor="#0a0a0f",
                        edgecolor="none", alpha=0.7))

    # ======== Angular ring buffer ========
    # angle_bins[bin_idx] = list of (x,y,z,intensity) or None
    angle_bins = [None] * bin_count

    def update_scatter(xs, ys, zs, its):
        """Update existing scatter — _offsets3d for mpl 3.3+."""
        n = len(xs)
        if n == 0:
            sc._offsets3d = (np.empty(0), np.empty(0), np.empty(0))
            sc.set_array(np.empty(0))
            return
        sc._offsets3d = (xs, ys, zs)
        sc.set_array(its)
        sc.set_clim(0, 255)

    def animate(_frame):
        pts, angle, fps = node.snapshot()

        # ---- store current frame into its angular bin ----
        if pts:
            bin_idx = int(angle * bin_count / 360.0) % bin_count
            angle_bins[bin_idx] = pts  # replace old sweep at this angle

        # ---- flatten all non-empty bins for rendering ----
        all_pts = []
        filled = 0
        for b in angle_bins:
            if b:
                filled += 1
                all_pts.extend(b)

        n = len(all_pts)
        if n == 0:
            txt.set_text("WAITING for sonar data...")
            return []

        xs = np.array([p[0] for p in all_pts], dtype=np.float32)
        ys = np.array([p[1] for p in all_pts], dtype=np.float32)
        zs = np.array([p[2] for p in all_pts], dtype=np.float32)
        its = np.array([p[3] for p in all_pts], dtype=np.float32)

        update_scatter(xs, ys, zs, its)

        max_d = float(np.max(np.hypot(xs, ys)))
        sweep_pct = filled / bin_count * 100
        txt.set_text(
            f"Ang:{angle:.0f}\u00b0 | {n} pts | {filled}/{bin_count} bins "
            f"({sweep_pct:.0f}%) | Dst:{max_d:.2f}m | FPS:{fps:.0f}"
        )

        return []

    def on_key(event):
        if event.key == "c":
            for i in range(bin_count):
                angle_bins[i] = None
            print("[CLEAR] Sweep buffer cleared")
        elif event.key == "r":
            ax.view_init(elev=35, azim=-65)
            print("[RESET] View reset")

    fig.canvas.mpl_connect("key_press_event", on_key)

    ani = FuncAnimation(fig, animate, interval=50, blit=False)
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Sonar Omni 3D Sweep Viewer (V12) — 1 revolution on screen")
    parser.add_argument("--range", type=float, default=5.0,
                        help="Display range in meters (default: 5)")
    parser.add_argument("--bins", type=int, default=360,
                        help="Angular bins for 360 sweep (default: 360)")
    parser.add_argument("--topic", default="sonar/omni/original")
    args = parser.parse_args()

    rclpy.init()
    node = SonarCache(topic=args.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print(f"\n{'='*60}")
    print(f"  Sonar Omni 3D Sweep Viewer  [V12]")
    print(f"  Topic    : {args.topic}")
    print(f"  Range    : +/- {args.range} m")
    print(f"  Bins     : {args.bins}  (1 sweep = {args.bins} bins)")
    print(f"  DOMAIN   : {os.environ.get('ROS_DOMAIN_ID', '0')}")
    print(f"  Keys     : C = clear | R = reset view")
    print(f"  Mouse    : Left-drag=rotate | Scroll=zoom | Right=pan")
    print(f"{'='*60}\n")

    try:
        run_visualizer(node, args.range, args.bins)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
