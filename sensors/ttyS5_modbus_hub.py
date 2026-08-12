#!/usr/bin/env python3
"""
ttyS5 Modbus-RTU 总线中枢 (RK3588)
====================================
同时管理 D30 深温计 和 HCX-8406 PWM 水下灯板：

  D30 深温计：读取压力/温度，发布 /rov/depth 等
  HCX-8406 PWM：控制 PWM1/PWM2 两路水下灯

两者共享 /dev/ttyS5 (19200/8N1)。本节点独占串口，避免多进程竞争。

地址方案（默认，与当前硬件一致）：
  D30 深温计 = 地址 3（需先用 tools/configure_depth_sensor.py 修改）
  HCX-8406 PWM = 地址 1（硬件固定，无法软件修改）

若现场接线/硬件不同，可通过环境变量覆盖：
  export DEPTH_ADDR=3 PWM_ADDR=1

ROS2 话题：
  发布：
    /rov/depth            std_msgs/Float32  水深（米）
    /rov/depth_pressure   std_msgs/Float32  压力（MPa）
    /rov/depth_temp       std_msgs/Float32  水温（摄氏度）
    /rov/light_state      std_msgs/Int8     当前灯状态：0=关,1=半亮,2=全亮
  订阅：
    /rov/light_cmd        std_msgs/String   'off' | 'half' | 'full'

环境变量：
  TTY_S5_PORT    默认 /dev/ttyS5
  DEPTH_ADDR     默认 3（D30 深温计）
  PWM_ADDR       默认 1（HCX-8406 PWM，硬件固定）
  PWM_ACTIVE_LOW 默认 1（0%占空比=全亮，100%=灭）；设 0 则相反
"""
import os, sys, time, struct, select, termios, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, String

SERIAL_PORT = os.environ.get('TTY_S5_PORT', '/dev/ttyS5')
BAUDRATE = 19200

# 设备地址（可通过环境变量覆盖）
DEPTH_ADDR = int(os.environ.get('DEPTH_ADDR', '3'), 0)
PWM_ADDR = int(os.environ.get('PWM_ADDR', '1'), 0)

# PWM 寄存器
PWM_REG_FREQ_CH1 = 0x0001
PWM_REG_FREQ_CH2 = 0x0002
PWM_REG_DUTY_CH1 = 0x0005
PWM_REG_DUTY_CH2 = 0x0006
PWM_FREQ = 20000  # 20 kHz

# 灯光状态（active-low：占空比 0=全亮，100=灭）
PWM_ACTIVE_LOW = os.environ.get('PWM_ACTIVE_LOW', '1') != '0'
LIGHT_STATES = {
    'off':  (0, 100 if PWM_ACTIVE_LOW else 0),
    'half': (1, 50),
    'full': (2, 0 if PWM_ACTIVE_LOW else 100),
}


class NativeSerial:
    BAUDS = {9600: termios.B9600, 19200: termios.B19200,
             38400: termios.B38400, 57600: termios.B57600,
             115200: termios.B115200}

    def __init__(self, port, baudrate=19200, timeout=0.5):
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
        bconst = self.BAUDS.get(baudrate, termios.B19200)
        attr[4] = bconst
        attr[5] = bconst
        attr[6][termios.VMIN] = 1
        attr[6][termios.VTIME] = int(timeout * 10) if timeout else 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        os.write(self.fd, data)

    def flush(self):
        termios.tcdrain(self.fd)

    def read(self, length):
        buf = b''
        deadline = time.time() + self.timeout
        while len(buf) < length and time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], max(0.02, self.timeout / 10))
            if r:
                chunk = os.read(self.fd, length - len(buf))
                if chunk:
                    buf += chunk
                else:
                    break
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


def build_read_cmd(addr, reg, count):
    cmd = struct.pack('>BBHH', addr, 0x03, reg, count)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def build_write_cmd(addr, reg, value):
    cmd = struct.pack('>BBHH', addr, 0x06, reg, value)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def parse_depth_response(data):
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


def parse_write_response(data, expected_first_6):
    if len(data) < 8:
        return False, f'短响应 {len(data)}/8'
    crc_exp = modbus_crc16(data[:6])
    crc_got = struct.unpack('<H', data[6:8])[0]
    if crc_exp != crc_got:
        return True, '非标准 ACK（HCX-8406 写操作可能返回自定义响应）'
    if data[:6] != expected_first_6:
        return False, f'响应不匹配 {data.hex()}'
    return True, None


# ── ROS2 节点 ─────────────────────────────────────────
class TtyS5ModbusHub(Node):
    def __init__(self):
        super().__init__('ttyS5_modbus_hub')
        self.pub_depth = self.create_publisher(Float32, '/rov/depth', 10)
        self.pub_pressure = self.create_publisher(Float32, '/rov/depth_pressure', 10)
        self.pub_temp = self.create_publisher(Float32, '/rov/depth_temp', 10)
        self.pub_light = self.create_publisher(Int8, '/rov/light_state', 10)
        self.sub_light = self.create_subscription(
            String, '/rov/light_cmd', self.on_light_cmd, 10)

        self.get_logger().info(
            f'打开 {SERIAL_PORT} @ {BAUDRATE} '
            f'(D30=addr{DEPTH_ADDR}, PWM=addr{PWM_ADDR}, active_low={PWM_ACTIVE_LOW})')
        self.ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=0.5)
        self.lock = threading.Lock()
        self.fail = 0
        self.light_state = 0  # 0=off,1=half,2=full

        # PWM 板每次断电会恢复到出厂 115200，需自动切换到 19200
        self._ensure_pwm_baud()

        # 初始化 PWM：频率 20kHz，默认关灯
        self._init_pwm()

        self.timer = self.create_timer(0.5, self.poll_depth)
        self.get_logger().info('ttyS5 Modbus 中枢已启动')

    def _serial_exchange(self, cmd, resp_len, label, timeout=0.2):
        """线程安全的 Modbus 收发"""
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(cmd)
            self.ser.flush()
            time.sleep(0.05)
            old_timeout = self.ser.timeout
            self.ser.timeout = timeout
            try:
                resp = self.ser.read(resp_len)
            finally:
                self.ser.timeout = old_timeout
            return resp

    def _ensure_pwm_baud(self):
        """PWM 板断电后恢复出厂 115200，需自动切换到 19200。
        策略：先在 19200 尝试读心跳，无响应则切到 115200 改波特率再切回。
        """
        # 先在 19200 尝试读 PWM 心跳
        cmd = build_read_cmd(PWM_ADDR, 0x0000, 1)
        resp = self._serial_exchange(cmd, 7, 'PWM heartbeat check', timeout=0.3)
        if len(resp) >= 7:
            crc_exp = modbus_crc16(resp[:5])
            crc_got = struct.unpack('<H', resp[5:7])[0]
            if crc_exp == crc_got and resp[0] == PWM_ADDR:
                self.get_logger().info(f'PWM 板已在 {BAUDRATE}，无需切换')
                return

        # PWM 板可能在 115200，需要切换
        self.get_logger().info(f'PWM 板在 {BAUDRATE} 无响应，尝试在 115200 切换波特率...')
        self.ser.close()
        time.sleep(0.2)

        try:
            ser_hi = NativeSerial(SERIAL_PORT, 115200, timeout=0.5)
        except Exception as e:
            self.get_logger().error(f'以 115200 打开串口失败: {e}，将以 {BAUDRATE} 继续')
            self.ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=0.5)
            return

        # 写波特率寄存器 0x000B = 1 (19200)
        baud_cmd = build_write_cmd(PWM_ADDR, 0x000B, 0x0001)
        ser_hi.reset_input_buffer()
        ser_hi.write(baud_cmd)
        ser_hi.flush()
        time.sleep(0.1)
        baud_resp = ser_hi.read(8)
        self.get_logger().info(f'PWM 波特率写入响应: {baud_resp.hex()} (len={len(baud_resp)})')
        ser_hi.close()
        time.sleep(0.3)

        # 重新以 19200 打开
        self.ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=0.5)
        time.sleep(0.1)

        # 验证 PWM 是否在 19200 响应
        resp = self._serial_exchange(cmd, 7, 'PWM heartbeat verify', timeout=0.3)
        if len(resp) >= 7:
            crc_exp = modbus_crc16(resp[:5])
            crc_got = struct.unpack('<H', resp[5:7])[0]
            if crc_exp == crc_got and resp[0] == PWM_ADDR:
                self.get_logger().info(f'PWM 板已切换到 {BAUDRATE} [OK]')
                return

        self.get_logger().warn(f'PWM 板切换后仍无响应，灯控可能不可用')

    def _init_pwm(self):
        self.get_logger().info('初始化 PWM 板：频率 20kHz，默认关灯')
        for reg in (PWM_REG_FREQ_CH1, PWM_REG_FREQ_CH2):
            cmd = build_write_cmd(PWM_ADDR, reg, PWM_FREQ)
            resp = self._serial_exchange(cmd, 8, 'PWM freq init', timeout=0.3)
            ok, err = parse_write_response(resp, cmd[:6])
            if not ok:
                self.get_logger().warn(f'PWM 频率初始化失败 ({reg:04X}): {err}, resp={resp.hex()}')

        # 默认关灯
        self._set_light_raw('off')

    def _set_light_raw(self, state):
        code, duty = LIGHT_STATES.get(state, (0, 100 if PWM_ACTIVE_LOW else 0))
        for reg in (PWM_REG_DUTY_CH1, PWM_REG_DUTY_CH2):
            cmd = build_write_cmd(PWM_ADDR, reg, duty)
            resp = self._serial_exchange(cmd, 8, f'PWM duty {state}', timeout=0.3)
            ok, err = parse_write_response(resp, cmd[:6])
            if not ok:
                self.get_logger().warn(f'PWM 占空比写入失败 ({reg:04X}={duty}): {err}, resp={resp.hex()}')
        self.light_state = code
        msg = Int8()
        msg.data = code
        self.pub_light.publish(msg)
        return code

    def on_light_cmd(self, msg):
        state = msg.data.strip().lower()
        if state not in LIGHT_STATES:
            self.get_logger().warn(f'未知灯光命令: {msg.data}，支持 off/half/full')
            return
        code = self._set_light_raw(state)
        self.get_logger().info(f'灯光命令: {msg.data} -> 状态 {code} (duty={LIGHT_STATES[state][1]})')

    def poll_depth(self):
        try:
            cmd = build_read_cmd(DEPTH_ADDR, 0x8939, 4)
            resp = self._serial_exchange(cmd, 13, 'D30 depth poll', timeout=0.5)
            if len(resp) < 13:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn(f'D30 短响应 {len(resp)}/13')
                return
            r = parse_depth_response(resp)
            if r is None:
                self.fail += 1
                if self.fail == 1:
                    self.get_logger().warn('D30 CRC 失败')
                return
            self.fail = 0
            p_cm, t_001c = r
            depth_m = p_cm / 100.0
            pressure_mpa = p_cm / 10000.0
            temp_c = t_001c / 1000.0
            for pub, val in [(self.pub_depth, depth_m),
                             (self.pub_pressure, pressure_mpa),
                             (self.pub_temp, temp_c)]:
                m = Float32()
                m.data = float(val)
                pub.publish(m)
            # 每10秒重发一次灯状态，让新订阅者能获取当前状态
            if self.fail == 0 and int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'深度={depth_m:.2f}m 压力={pressure_mpa:.4f}MPa 水温={temp_c:.2f}°C 灯={self.light_state}')
                light_msg = Int8()
                light_msg.data = self.light_state
                self.pub_light.publish(light_msg)
        except Exception as e:
            self.fail += 1
            if self.fail == 1:
                self.get_logger().error(f'D30 轮询异常: {e}')

    def destroy_node(self):
        # 退出前关灯
        try:
            self._set_light_raw('off')
        except Exception:
            pass
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TtyS5ModbusHub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
