#!/usr/bin/env python3
"""临时诊断：尝试用 function 0x10 写 PWM 地址"""
import os, time, struct, select, termios

class NativeSerial:
    BAUDS = {9600: termios.B9600, 19200: termios.B19200,
             38400: termios.B38400, 57600: termios.B57600,
             115200: termios.B115200}
    def __init__(self, port, baudrate=19200, timeout=0.3):
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
    def write(self, data): os.write(self.fd, data)
    def flush(self): termios.tcdrain(self.fd)
    def read(self, length):
        buf = b''
        deadline = time.time() + self.timeout
        while len(buf) < length and time.time() < deadline:
            r, _, _ = select.select([self.fd], [], [], max(0.02, self.timeout / 10))
            if r:
                chunk = os.read(self.fd, length - len(buf))
                if chunk: buf += chunk
                else: break
        return buf
    def reset_input_buffer(self): termios.tcflush(self.fd, termios.TCIFLUSH)
    def close(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self._saved)
        os.close(self.fd)

def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1: crc = (crc >> 1) ^ 0xA001
            else: crc >>= 1
    return crc

def read_reg(ser, addr, reg, count=1):
    cmd = struct.pack('>BBHH', addr, 0x03, reg, count)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    expected = 5 + count * 2
    resp = ser.read(expected)
    return cmd.hex(), resp.hex(), expected

def write_single(ser, addr, reg, value):
    cmd = struct.pack('>BBHH', addr, 0x06, reg, value)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    return cmd.hex(), resp.hex()

def write_multiple(ser, addr, reg, values):
    # values: list of 16-bit ints
    payload = struct.pack('>BHH', len(values), 0, len(values) * 2)
    for v in values:
        payload += struct.pack('>H', v)
    cmd = struct.pack('>BBH', addr, 0x10, reg) + payload
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    return cmd.hex(), resp.hex()

if __name__ == '__main__':
    port = os.environ.get('PWM_PORT', '/dev/ttyS5')
    ser = NativeSerial(port, 115200, timeout=0.3)

    print('Initial state:')
    print(f'  addr reg: {read_reg(ser, 1, 0x000A)}')

    print()
    print('Try function 0x10 write addr=2 to 0x000A:')
    cmd, resp = write_multiple(ser, 1, 0x000A, [2])
    print(f'  cmd={cmd} resp={resp}')
    print(f'  addr reg after: {read_reg(ser, 1, 0x000A)}')

    print()
    print('Try function 0x06 write addr=3 to 0x000A:')
    cmd, resp = write_single(ser, 1, 0x000A, 3)
    print(f'  cmd={cmd} resp={resp}')
    print(f'  addr reg after: {read_reg(ser, 1, 0x000A)}')

    print()
    print('Try function 0x06 write to 0x0000 (heartbeat) value=999:')
    cmd, resp = write_single(ser, 1, 0x0000, 999)
    print(f'  cmd={cmd} resp={resp}')
    print(f'  heartbeat reg after: {read_reg(ser, 1, 0x0000)}')

    ser.close()
