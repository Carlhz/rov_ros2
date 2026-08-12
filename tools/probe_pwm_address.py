#!/usr/bin/env python3
"""
HCX-8406 PWM 板地址/波特率调试验证脚本
=====================================
在 RK3588 上运行，独占 /dev/ttyS5，尝试把 PWM 板从默认地址 1 改为 2，
并把波特率从 115200 改为 19200。

设计要点：
  - 先用 115200 与 PWM 板通信，此时 D30 深度计（19200）不会解码总线数据，
    避免地址冲突导致的总线碰撞。
  - 改完地址后再改波特率，最后验证 19200/addr2 是否可用。

用法：
  python3 /opt/ros/rov_ros2_ws/tools/probe_pwm_address.py
"""
import os, sys, time, struct, select, termios

SERIAL_PORT = os.environ.get('PWM_PORT', '/dev/ttyS5')

BAUD_DEFAULT = 115200
BAUD_TARGET = 19200
ADDR_DEFAULT = 1
ADDR_TARGET = 2

REG_ADDR = 0x000A
REG_BAUD = 0x000B
REG_HEARTBEAT = 0x0000

BAUD_CODE_19200 = 0x01  # 0=9600, 1=19200, 2=115200


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

    def set_baudrate(self, baudrate):
        bconst = self.BAUDS.get(baudrate, termios.B19200)
        attr = termios.tcgetattr(self.fd)
        attr[4] = bconst
        attr[5] = bconst
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


def read_reg(ser, addr, reg, count=1, timeout=0.3):
    cmd = struct.pack('>BBHH', addr, 0x03, reg, count)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    expected = 5 + count * 2
    resp = ser.read(expected)
    ok = False
    val = None
    if len(resp) >= 5:
        bc = resp[2]
        frame_len = 5 + bc
        if len(resp) >= frame_len:
            crc_exp = modbus_crc16(resp[:frame_len - 2])
            crc_got = struct.unpack('<H', resp[frame_len - 2:frame_len])[0]
            ok = (crc_exp == crc_got and resp[0] == addr and resp[1] == 0x03)
            if ok and count == 1:
                val = struct.unpack('>H', resp[3:5])[0]
    return ok, val, cmd.hex(), resp.hex()


def write_single(ser, addr, reg, value, timeout=0.3):
    cmd = struct.pack('>BBHH', addr, 0x06, reg, value)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    ok = False
    if len(resp) >= 8:
        crc_exp = modbus_crc16(resp[:6])
        crc_got = struct.unpack('<H', resp[6:8])[0]
        if crc_exp == crc_got and resp[0] == addr and resp[1] == 0x06:
            ok = True
    return ok, cmd.hex(), resp.hex()


def write_multiple(ser, addr, reg, values, timeout=0.3):
    """正确的 0x10 写多个寄存器格式"""
    byte_count = len(values) * 2
    cmd = struct.pack('>BBHHH', addr, 0x10, reg, len(values), byte_count)
    for v in values:
        cmd += struct.pack('>H', v)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    ok = False
    if len(resp) >= 8:
        crc_exp = modbus_crc16(resp[:6])
        crc_got = struct.unpack('<H', resp[6:8])[0]
        if crc_exp == crc_got and resp[0] == addr and resp[1] == 0x10:
            ok = True
    return ok, cmd.hex(), resp.hex()


def probe_comm(ser, baud, addr, label):
    print(f'\n[{label}] 以 {baud} 探测地址 {addr}...')
    ser.set_baudrate(baud)
    time.sleep(0.1)
    ok, val, cmd, resp = read_reg(ser, addr, REG_HEARTBEAT)
    print(f'  -> 心跳读取: {cmd}')
    print(f'  <- 响应: {resp}')
    if ok:
        print(f'  [OK] 地址 {addr} 通信正常，心跳={val}')
    else:
        print(f'  [FAIL] 地址 {addr} 无响应或异常')
    return ok


def main():
    print('=== HCX-8406 PWM 板地址/波特率调试验证 ===')
    print(f'端口: {SERIAL_PORT}')
    print(f'目标: 地址 {ADDR_DEFAULT}->{ADDR_TARGET}，波特率 {BAUD_DEFAULT}->{BAUD_TARGET}')
    print()

    if not os.path.exists(SERIAL_PORT):
        print(f'[ERROR] 串口 {SERIAL_PORT} 不存在')
        sys.exit(1)

    # 步骤 1: 用 115200 与 PWM 板通信（D30 此时不解码）
    print('[1] 以 115200 打开串口（PWM 出厂默认，D30 不会响应 115200 数据）...')
    try:
        ser = NativeSerial(SERIAL_PORT, BAUD_DEFAULT, timeout=0.5)
    except Exception as e:
        print(f'[ERROR] 打开串口失败: {e}')
        sys.exit(1)

    # 先读地址寄存器和波特率寄存器
    print('\n[2] 读取当前地址寄存器 0x000A 和波特率寄存器 0x000B...')
    ok, val, cmd, resp = read_reg(ser, ADDR_DEFAULT, REG_ADDR)
    print(f'  -> 读地址: {cmd}')
    print(f'  <- 响应: {resp}')
    if ok:
        print(f'  [OK] 当前地址寄存器值 = {val} (0x{val:04X})')
    else:
        print(f'  [WARN] 读地址寄存器失败，可能该寄存器不存在或设备未响应')

    ok, val, cmd, resp = read_reg(ser, ADDR_DEFAULT, REG_BAUD)
    print(f'  -> 读波特率: {cmd}')
    print(f'  <- 响应: {resp}')
    if ok:
        print(f'  [OK] 当前波特率码 = {val} (0=9600,1=19200,2=115200)')
    else:
        print(f'  [WARN] 读波特率寄存器失败')

    # 步骤 3: 尝试用 0x06 写地址 2
    print(f'\n[3] 尝试功能码 0x06 写地址 {ADDR_TARGET} 到 0x000A...')
    ok, cmd, resp = write_single(ser, ADDR_DEFAULT, REG_ADDR, ADDR_TARGET)
    print(f'  -> 命令: {cmd}')
    print(f'  <- 响应: {resp}')
    if ok:
        print('  [OK] 0x06 写入得到标准响应')
    else:
        print('  [INFO] 0x06 写入未得到标准响应（可能是自定义 ACK 或失败）')

    # 验证地址是否真的改了
    addr_changed = False
    if probe_comm(ser, BAUD_DEFAULT, ADDR_TARGET, '验证 0x06 后地址 2'):
        addr_changed = True
    else:
        # 步骤 4: 尝试用正确的 0x10 写地址 2
        print(f'\n[4] 0x06 未生效，尝试功能码 0x10 写地址 {ADDR_TARGET} 到 0x000A...')
        ok, cmd, resp = write_multiple(ser, ADDR_DEFAULT, REG_ADDR, [ADDR_TARGET])
        print(f'  -> 命令: {cmd}')
        print(f'  <- 响应: {resp}')
        if ok:
            print('  [OK] 0x10 写入得到标准响应')
        else:
            print('  [INFO] 0x10 写入未得到标准响应')

        if probe_comm(ser, BAUD_DEFAULT, ADDR_TARGET, '验证 0x10 后地址 2'):
            addr_changed = True

    # 如果地址改成功了，后续用新地址；否则仍用默认地址
    active_addr = ADDR_TARGET if addr_changed else ADDR_DEFAULT
    if addr_changed:
        print(f'\n[5] 地址已成功改为 {ADDR_TARGET}，准备改波特率到 {BAUD_TARGET}...')
    else:
        print(f'\n[5] 地址未能改为 {ADDR_TARGET}（可能硬件固定），仍以地址 {ADDR_DEFAULT} 改波特率...')

    # 步骤 5: 改波特率
    ok, cmd, resp = write_single(ser, active_addr, REG_BAUD, BAUD_CODE_19200)
    print(f'  -> 写波特率命令: {cmd}')
    print(f'  <- 响应: {resp}')
    if ok:
        print('  [OK] 波特率写入得到标准响应')
    else:
        print('  [INFO] 波特率写入未得到标准响应（可能仍是自定义 ACK）')

    # 步骤 6: 验证 19200 通信
    time.sleep(0.3)
    if probe_comm(ser, BAUD_TARGET, active_addr, f'验证 {BAUD_TARGET}/addr{active_addr}'):
        print('\n' + '='*60)
        print('[SUCCESS] PWM 板配置验证通过！')
        print(f'  地址 = {active_addr}')
        print(f'  波特率 = {BAUD_TARGET}')
        if active_addr == ADDR_TARGET:
            print('  说明：地址已成功改为 2，可与 D30(addr=1) 共存。')
        else:
            print('  警告：地址未能改为 2，需要改用其他方案：')
            print('        1) 给 PWM 板断电后查看是否有硬件拨码可改地址；')
            print('        2) 或运行 configure_depth_sensor.py 将 D30 改为地址 3。')
        print('='*60)
    else:
        print('\n' + '='*60)
        print('[FAIL] 波特率修改后无法通信，可能需要给 PWM 板断电重启。')
        print('='*60)

    ser.close()


if __name__ == '__main__':
    main()
