#!/usr/bin/env python3
"""
SF 超声波测深仪（高度计）ROS2 驱动
零外部依赖（仅需 ROS2 rclpy）。运行于 RK3588，RS485 ttyS3。

话题：
  /rov/altitude         std_msgs/Float32  最强目标距离（米）
  /rov/altitude_nearest  std_msgs/Float32  最近目标距离（米）
  /rov/altitude_raw      std_msgs/Float32  最强目标原始值（cm）
"""
import os, sys, time, struct, select, termios, fcntl, signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

SERIAL_PORT = os.environ.get('ALTI_PORT', '/dev/ttyS3')
BAUDRATE = 9600
DEVICE_ID = 1
FRAME_LEN = 17
READ_BUF = FRAME_LEN + 6   # 预留前导干扰字节空间
BLIND_ZONE_CM = 20


# ── 原生串口（零依赖） ──────────────────────────────────
class NativeSerial:
    BAUDS = {9600: termios.B9600, 19200: termios.B19200,
             38400: termios.B38400, 57600: termios.B57600,
             115200: termios.B115200}

    def __init__(self, port, baudrate=9600, timeout=0.5):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        self.timeout = timeout
        self._saved = termios.tcgetattr(self.fd)
        attr = termios.tcgetattr(self.fd)
        attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                     | termios.ISTRIP | termios.INLCR | termios.IGNCR
                     | termios.ICRNL | termios.IXON)
        attr[1] &= ~termios.OPOST
        attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE
                     | termios.ISIG)
        bconst = self.BAUDS.get(baudrate, termios.B9600)
        attr[4] = bconst  # c_ispeed
        attr[5] = bconst  # c_ospeed
        attr[6][termios.VMIN] = 1
        attr[6][termios.VTIME] = int(timeout * 10) if timeout else 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        os.write(self.fd, data)

    def flush(self):
        termios.tcdrain(self.fd)

    def read(self, length):
        """VMIN=1 阻塞读取 + signal.alarm 总超时保护。
        RK3588 ttyS3 (dw-apb-uart) 的 select() 和 VMIN=0 均不工作，
        只有 VMIN=1 (阻塞直到至少1字节) 有效。用 SIGALRM 防止永久阻塞。"""
        attr = termios.tcgetattr(self.fd)
        attr[6][termios.VMIN] = 1
        attr[6][termios.VTIME] = max(1, int(self.timeout * 10))
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)

        buf = b''
        def _alarm_handler(signum, frame):
            raise TimeoutError('serial read timeout')
        old_handler = signal.signal(signal.SIGALRM, _alarm_handler)
        signal.setitimer(signal.ITIMER_REAL, self.timeout)
        try:
            while len(buf) < length:
                chunk = os.read(self.fd, length - len(buf))
                if not chunk:
                    break
                buf += chunk
        except (TimeoutError, OSError):
            pass
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, old_handler)
        return buf

    def reset_input_buffer(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self._saved)
        os.close(self.fd)


# ── 协议 ──────────────────────────────────────────────
def build_command(dev_id):
    cmd = bytearray([0xAA, 0xA0, dev_id, 0x00, 0x00])
    checksum = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4]
    cmd.append(checksum)
    return bytes(cmd)


def parse_response(data):
    """带帧同步的解析：在响应中搜索 AB A0 帧头"""
    if len(data) < FRAME_LEN:
        return None
    # 搜索 AB A0 帧头（允许前导垃圾字节）
    for offset in range(len(data) - FRAME_LEN + 1):
        if data[offset] == 0xAB and data[offset + 1] == 0xA0:
            d = data[offset:offset + FRAME_LEN]
            nearest = (d[4] << 8) | d[5]
            strongest = (d[8] << 8) | d[9]
            return nearest, strongest
    return None


# ── ROS2 节点 ─────────────────────────────────────────
class AltimeterDriver(Node):
    def __init__(self):
        super().__init__('altimeter_driver')
        self.pub_altitude = self.create_publisher(Float32, '/rov/altitude', 10)
        self.pub_nearest = self.create_publisher(Float32, '/rov/altitude_nearest', 10)
        self.pub_raw = self.create_publisher(Float32, '/rov/altitude_raw', 10)
        self.get_logger().info(f'打开 {SERIAL_PORT} @ {BAUDRATE}')
        self.ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=1.5)
        # 清空上电残留
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        self.cmd = build_command(DEVICE_ID)
        self.timer = self.create_timer(1.0, self.poll)
        self.fail = 0
        self.get_logger().info('SF 高度计驱动已启动')

    def poll(self):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self.cmd)
            self.ser.flush()
            time.sleep(0.3)
            resp = self.ser.read(READ_BUF)
            if len(resp) < FRAME_LEN:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn(f'短响应 {len(resp)}/{FRAME_LEN}')
                return
            r = parse_response(resp)
            if r is None:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn('帧头错误')
                return
            self.fail = 0
            near_cm, strong_cm = r
            near_m = near_cm / 100.0 if near_cm >= BLIND_ZONE_CM else -1.0
            strong_m = strong_cm / 100.0 if strong_cm >= BLIND_ZONE_CM else -1.0
            for pub, val in [(self.pub_altitude, strong_m),
                             (self.pub_nearest, near_m),
                             (self.pub_raw, float(strong_cm))]:
                msg = Float32()
                msg.data = val
                pub.publish(msg)
            if int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'最强={strong_m:.2f}m 最近={near_m:.2f}m')
        except Exception as e:
            self.fail += 1
            if self.fail == 1:
                self.get_logger().error(f'轮询异常: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AltimeterDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
