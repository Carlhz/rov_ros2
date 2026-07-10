"""Quick test: does /joy topic have data?"""
import rclpy
from sensor_msgs.msg import Joy
import time, os

rclpy.init()
n = rclpy.create_node('joy_test')
got = []

def cb(m):
    got.append(m)
    print('GOT axes=' + str(len(m.axes)) + ' btns=' + str(len(m.buttons)))
    print('AXES:', [round(a, 3) for a in m.axes[:8]])
    print('BTNS:', [int(b) for b in m.buttons[:12]])

n.create_subscription(Joy, '/joy', cb, 10)
t0 = time.time()
while time.time() - t0 < 5 and not got:
    rclpy.spin_once(n, timeout_sec=0.3)

if not got:
    try:
        fd = os.open('/dev/input/js0', os.O_RDONLY | os.O_NONBLOCK)
        print('JS0_OPEN_OK fd=' + str(fd))
        import struct
        data = os.read(fd, 8)
        print('JS0_READ:', data.hex() if data else 'NO_DATA')
        os.close(fd)
    except Exception as e:
        print('JS0_ERR:', repr(e))

n.destroy_node()
rclpy.shutdown()
