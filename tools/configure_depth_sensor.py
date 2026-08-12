#!/usr/bin/env python3
"""
D30 深温计地址修改工具 (ASCII 协议)
====================================
D30 使用自定义 ASCII 协议（非标准 Modbus）设置地址：
  查询地址: sA00\r\n  -> sA{addr:02d}\r\n
  设置地址: sB{addr:02d}\r\n -> sBok\r\n (成功) / sBf\r\n (失败)

注意：D30 上电后只能设置一次地址，需重新上电才能再次设置。
      设置时总线上只能有一个传感器（避免冲突）。

修改后：
  D30 深温计 = 地址 3，波特率 19200
  HCX-8406 PWM = 地址 1，波特率 19200（硬件固定）

用法：
  python3 /opt/ros/rov_ros2_ws/tools/configure_depth_sensor.py
  python3 /opt/ros/rov_ros2_ws/tools/configure_depth_sensor.py --addr 1  # 回退
"""
import os, sys, time, struct, select, termios, argparse

SERIAL_PORT = os.environ.get('DEPTH_PORT', '/dev/ttyS5')
BAUDRATE = 19200
NEW_ADDR = 3  # 避免与 PWM 板默认地址 1 冲突


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


def query_addr_ascii(ser):
    """用 ASCII 协议查询当前地址: sA00 -> sA{addr:02d}"""
    cmd = b'sA00\r\n'
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.2)
    resp = ser.read(32)
    if len(resp) >= 6 and resp[0:2] == b'sA':
        try:
            addr_str = resp[2:4].decode('ascii')
            return int(addr_str)
        except (ValueError, UnicodeDecodeError):
            pass
    return None


def set_addr_ascii(ser, new_addr):
    """用 ASCII 协议设置地址: sB{addr:02d} -> sBok (成功) / sBf (失败)"""
    cmd = 'sB{:02d}\r\n'.format(new_addr).encode('ascii')
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.3)
    resp = ser.read(32)
    if b'ok' in resp or b'o\r\n' in resp:
        return True, resp
    if b'f\r\n' in resp:
        return False, resp
    return False, resp


def verify_modbus(ser, addr):
    """用 Modbus RTU 验证地址是否生效"""
    cmd = struct.pack('>BBHH', addr, 0x03, 0x8939, 4)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(13)
    if len(resp) >= 13:
        crc_exp = modbus_crc16(resp[:11])
        crc_got = struct.unpack('<H', resp[11:13])[0]
        if crc_exp == crc_got and resp[0] == addr:
            p = struct.unpack('>i', resp[3:7])[0]
            t = struct.unpack('>i', resp[7:11])[0]
            return True, p, t
    return False, 0, 0


def main():
    ap = argparse.ArgumentParser(description='D30 深温计地址配置 (ASCII 协议)')
    ap.add_argument('--addr', type=int, default=NEW_ADDR,
                    help=f'目标地址（默认 {NEW_ADDR}，回退可设 1）')
    args = ap.parse_args()

    target = args.addr
    if not (1 <= target <= 99):
        print('[ERROR] 地址必须在 1~99 之间')
        sys.exit(1)

    print(f'=== D30 深温计地址配置 (ASCII 协议) ===')
    print(f'端口: {SERIAL_PORT}')
    print(f'目标: 地址 {target:02d}')
    print()

    if not os.path.exists(SERIAL_PORT):
        print(f'[ERROR] 串口 {SERIAL_PORT} 不存在')
        sys.exit(1)

    print(f'[1/3] 以 {BAUDRATE} 打开串口，查询当前地址 (sA00)...')
    try:
        ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=1.0)
    except Exception as e:
        print(f'[ERROR] 打开串口失败: {e}')
        sys.exit(1)

    current = query_addr_ascii(ser)
    if current is None:
        print('[WARN] ASCII 查询失败，尝试 Modbus 验证...')
        for test_addr in [1, 3]:
            ok, p, t = verify_modbus(ser, test_addr)
            if ok:
                current = test_addr
                print(f'      Modbus 确认 D30 在地址 {test_addr} (depth={p/100.0:.2f}m)')
                break
        if current is None:
            print('[ERROR] 无法与 D30 通信')
            ser.close()
            sys.exit(1)
    else:
        print(f'      当前地址 = {current:02d}')

    if current == target:
        print(f'[INFO] 地址已经是 {target:02d}，无需修改')
        ok, p, t = verify_modbus(ser, target)
        if ok:
            print(f'      Modbus 验证: depth={p/100.0:.2f}m temp={t/1000.0:.2f}C')
        ser.close()
        return

    print(f'[2/3] 设置地址 {current:02d} -> {target:02d} (sB{target:02d})...')
    print(f'      注意: D30 上电后只能设置一次，如失败需断电重新上电')
    ok, resp = set_addr_ascii(ser, target)
    print(f'      响应: {resp}')
    if ok:
        print(f'      [OK] 设置成功!')
    else:
        print(f'      [FAIL] 设置失败! 响应: {resp}')
        print(f'      可能原因: 上电后已设置过一次，需对 D30 断电重新上电')
        ser.close()
        sys.exit(1)

    ser.close()
    time.sleep(0.5)

    print(f'[3/3] 以新地址 {target:02d} 验证...')
    ser = NativeSerial(SERIAL_PORT, BAUDRATE, timeout=0.5)

    # 先用 ASCII 验证
    addr = query_addr_ascii(ser)
    if addr == target:
        print(f'      ASCII 验证: sA{addr:02d} [OK]')
    else:
        print(f'      ASCII 验证: sA{addr} (预期 {target:02d})')

    # 再用 Modbus 验证
    ok, p, t = verify_modbus(ser, target)
    if ok:
        print(f'      Modbus 验证: [OK] depth={p/100.0:.2f}m temp={t/1000.0:.2f}C')
    else:
        print(f'      Modbus 验证: [FAIL]')

    ser.close()
    print()
    print('配置完成。')
    print(f'  D30 深温计地址 = {target}')
    print('  下一步：./start_all.sh stop && ./start_all.sh bg')


if __name__ == '__main__':
    main()
