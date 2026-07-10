"""Capture 4 seconds of sonar data and print statistics. For VM deployment."""
import rclpy, struct, time, threading
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2
import numpy as np


class Logger(Node):
    def __init__(self):
        super().__init__("data_logger")
        self.frames = []
        self.lock = threading.Lock()
        self.sub = self.create_subscription(
            PointCloud2, "sonar/omni/original", self.cb, 10)

    def cb(self, msg):
        n = msg.width
        if n == 0:
            return
        pts = []
        data, step = msg.data, msg.point_step
        for i in range(n):
            off = i * step
            if off + 16 > len(data):
                break
            x = struct.unpack_from("<f", data, off)[0]
            y = struct.unpack_from("<f", data, off + 4)[0]
            z = struct.unpack_from("<f", data, off + 8)[0]
            v = struct.unpack_from("<f", data, off + 12)[0]
            pts.append((x, y, z, v))
        with self.lock:
            self.frames.append(pts)


rclpy.init()
node = Logger()
ex = SingleThreadedExecutor()
ex.add_node(node)
thr = threading.Thread(target=ex.spin, daemon=True)
thr.start()
time.sleep(4)
with node.lock:
    frames = list(node.frames)
rclpy.shutdown()

if not frames:
    print("NO DATA received! Check: ROS_DOMAIN_ID=0, sonar driver running?")
else:
    ap = [p for f in frames for p in f]
    xs = [p[0] for p in ap]
    ys = [p[1] for p in ap]
    zs = [p[2] for p in ap]
    its = [p[3] for p in ap]

    print(f"\n{'='*55}")
    print(f"  SONAR DATA SNAPSHOT (4 seconds)")
    print(f"{'='*55}")
    print(f"  Frames captured : {len(frames)}")
    print(f"  Total points    : {len(ap)}")
    print(f"  Pts per frame   : min={min(len(f) for f in frames)} "
          f"max={max(len(f) for f in frames)} "
          f"avg={len(ap)/len(frames):.1f}")
    print(f"  Effective FPS   : {len(frames)/4:.0f}")

    print(f"\n  --- Coordinate Range ---")
    print(f"  X : [{min(xs):.6f}, {max(xs):.6f}]  mean={sum(xs)/len(xs):.6f}")
    print(f"  Y : [{min(ys):.6f}, {max(ys):.6f}]  mean={sum(ys)/len(ys):.6f}")
    print(f"  Z : [{min(zs):.6f}, {max(zs):.6f}]  mean={sum(zs)/len(zs):.6f}")

    dists = [max(abs(p[0]), abs(p[1])) for p in ap]
    print(f"  Dist : [{min(dists):.4f}, {max(dists):.4f}]m  "
          f"mean={sum(dists)/len(dists):.4f}m")

    print(f"\n  --- Intensity Distribution ---")
    print(f"  Range: [{min(its):.0f}, {max(its):.0f}]  mean={sum(its)/len(its):.1f}")
    print()
    bins = [(0, 30), (30, 60), (60, 100), (100, 150),
            (150, 200), (200, 230), (230, 255)]
    labels = ["very weak ", "weak      ", "low-med   ",
              "medium    ", "strong    ", "v.strong  ", "max       "]
    for (lo, hi), lab in zip(bins, labels):
        cnt = sum(1 for v in its if lo <= v < (hi if hi < 255 else 256))
        pct = cnt / max(1, len(ap)) * 100
        bar = "#" * int(pct * 2)
        print(f"  [{lo:3d}-{hi:3d}) {lab}: {cnt:4d} pts ({pct:5.1f}%) {bar}")

    print(f"\n  --- Sample Frames ---")
    for i, f in enumerate(frames[:3]):
        a = np.arctan2(-f[0][1], f[0][0]) * 180 / np.pi % 360
        print(f"  Frame {i+1}: {len(f)} pts, angle={a:.1f} deg")
        for j, p in enumerate(f[:3]):
            print(f"    pt{j}: ({p[0]:.5f}, {p[1]:.5f}, {p[2]:.5f}) "
                  f"intensity={p[3]:.0f}")
        if len(f) > 3:
            print(f"    ... (+{len(f)-3} more points)")

    print()
