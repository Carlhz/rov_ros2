#!/usr/bin/env python3
"""
声纳数据快速查看 (轻量版)
==========================
只订阅话题，打印原始统计，不依赖 curses。

用法:
  python3 sonar_quick_view.py [--topic original|rigidity|boundary]
"""
import sys
import time
import math
import struct
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def bytes_to_float(data, off):
    return struct.unpack_from("<f", data, off)[0]


class QuickView(Node):
    def __init__(self, topic="original"):
        super().__init__("sonar_quick_view")
        self.count = 0
        self.start = time.time()

        full_topic = f"sonar/omni/{topic}"
        self.create_subscription(PointCloud2, full_topic, self.cb, 10)
        self.get_logger().info(f"订阅: {full_topic}")

    def cb(self, msg):
        self.count += 1
        n = msg.width
        elapsed = time.time() - self.start
        fps = self.count / elapsed if elapsed > 0 else 0

        if n == 0:
            print(f"[{self.count:5d}] 角度=???, 点数=0, FPS={fps:.1f}")
            return

        data = msg.data
        step = msg.point_step

        # 取第一个点算角度
        x = bytes_to_float(data, 0)
        y = bytes_to_float(data, 4)
        ang = math.atan2(-y, x)

        # 统计
        max_int = 0.0
        max_r = 0.0
        for i in range(n):
            off = i * step
            if off + 16 > len(data):
                break
            px = bytes_to_float(data, off)
            py = bytes_to_float(data, off + 4)
            intensity = bytes_to_float(data, off + 12)
            r = math.sqrt(px*px + py*py)
            if r > max_r: max_r = r
            if intensity > max_int: max_int = intensity

        print(f"[{self.count:5d}] 角度={math.degrees(ang)%360:6.1f}°, "
              f"点数={n:4d}, 最远={max_r:5.2f}m, 最大强度={max_int:5.0f}, "
              f"FPS={fps:5.1f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="original",
                        choices=["original", "rigidity", "boundary"])
    args = parser.parse_args()

    rclpy.init()
    node = QuickView(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
