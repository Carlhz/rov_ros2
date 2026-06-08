#!/usr/bin/env python3
"""
D30 深温计 ROS2 驱动 — MODBUS-RTU 协议
零外部依赖（仅需 ROS2 rclpy）。运行于 RK3588，RS485 ttyS3。

话题：
  /rov/depth          std_msgs/Float32  水深（米）
  /rov/depth_pressure  std_msgs/Float32  压力（MPa）
  /rov/depth_temp      std_msgs/Float32  水温（摄氏度）
"""
import os, sys, time, struct, select, termios, fcntl
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

SERIAL_PORT = '/dev/ttyS3'
BAUDRATE = 19200
DEVICE_ADDR = 0x01


# ── 原生串口（零依赖） ──────────────────────────────────
class NativeSerial:
    BAUDS = {9600: termios.B9600, 19200: termios.B19200,
             38400: termios.B38400, 57600: termios.B57600,
             115200: termios.B115200}

    def __init__(self, port, baudrate=19200, timeout=0.5):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        self.timeout = timeout
        # 保存原属性
        self._saved = termios.tcgetattr(self.fd)
        # 配置 tty
        attr = termios.tcgetattr(self.fd)
        # iflag: 关闭字符处理
        attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                     | termios.ISTRIP | termios.INLCR | termios.IGNCR
                     | termios.ICRNL | termios.IXON)
        # oflag: raw output
        attr[1] &= ~termios.OPOST
        # cflag: 8N1, enable receiver
        attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        # lflag: 关闭 canonical/echo
        attr[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE
                     | termios.ISIG)
        # 波特率
        bconst = self.BAUDS.get(baudrate, termios.B19200)
        attr[4] = bconst  # c_ispeed
        attr[5] = bconst  # c_ospeed
        # VMIN/VTIME: blocking read with timeout
        attr[6][termios.VMIN] = 1
        attr[6][termios.VTIME] = int(timeout * 10) if timeout else 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        os.write(self.fd, data)

    def flush(self):
        termios.tcdrain(self.fd)

    def read(self, length):
        """循环读取直到收满 length 字节或超时"""
        buf = b''
        deadline = time.time() + self.timeout
        while len(buf) < length and time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], max(0.02, self.timeout / 10))
            if r:
                chunk = os.read(self.fd, length - len(buf))
                if chunk:
                    buf += chunk
                else:
                    break  # EOF/error
            # else: timeout, continue loop
        return buf

    def reset_input_buffer(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self._saved)
        os.close(self.fd)


# ── MODBUS CRC16 ──────────────────────────────────────
def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_cmd(addr):
    cmd = struct.pack('>BBHH', addr, 0x03, 0x8939, 4)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def parse_response(data):
    if len(data) < 13:
        return None
    addr, func, bc = data[0], data[1], data[2]
    if func != 0x03 or bc != 8:
        return None
    pressure = struct.unpack('>i', data[3:7])[0]
    temp = struct.unpack('>i', data[7:11])[0]
    crc_exp = modbus_crc16(data[:11])
    crc_got = struct.unpack('<H', data[11:13])[0]
    if crc_exp != crc_got:
        return None
    return pressure, temp


# ── ROS2 节点 ─────────────────────────────────────────
class DepthSensorDriver(Node):
    def __init__(self):
        super().__init__('depth_sensor_driver')
        self.pub_depth = self.create_publisher(Float32, '/rov/depth', 10)
        self.pub_pressure = self.create_publisher(Float32, '/rov/depth_pressure', 10)
        self.pub_temp = self.create_publisher(Float32, '/rov/depth_temp', 10)
        self.get_logger().info(f'打开 {SERIAL_PORT} @ {BAUDRATE}')
        self.ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=0.5)
        self.timer = self.create_timer(0.5, self.poll)
        self.cmd = build_read_cmd(DEVICE_ADDR)
        self.fail = 0
        self.get_logger().info('D30 深温计驱动已启动')

    def poll(self):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self.cmd)
            self.ser.flush()
            time.sleep(0.05)
            resp = self.ser.read(13)
            if len(resp) < 13:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn(f'短响应 {len(resp)}/13')
                return
            r = parse_response(resp)
            if r is None:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn('CRC 失败')
                return
            self.fail = 0
            p_cm, t_001c = r
            depth_m = p_cm / 100.0
            pressure_mpa = p_cm / 10000.0
            temp_c = t_001c / 1000.0
            for pub, val in [(self.pub_depth, depth_m),
                             (self.pub_pressure, pressure_mpa),
                             (self.pub_temp, temp_c)]:
                msg = Float32()
                msg.data = float(val)
                pub.publish(msg)
            if self.fail == 0 and int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'深度={depth_m:.2f}m 压力={pressure_mpa:.4f}MPa 水温={temp_c:.2f}°C')
        except Exception as e:
            self.fail += 1
            if self.fail == 1:
                self.get_logger().error(f'轮询异常: {e}')

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
