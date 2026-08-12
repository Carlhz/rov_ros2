#!/usr/bin/env python3
"""
HCX-8406 PWM 板一次性配置工具
================================
该脚本在 RK3588 上运行，将 PWM 板从出厂默认波特率 115200 改为 19200，
使其能与 D30 深温计共享 /dev/ttyS5 总线。

重要发现：
  HCX-8406 的"硬件地址"寄存器（0x000A）无法通过 Modbus 指令修改。
  0x06 返回非标准响应，0x10 返回无 CRC 的截断响应，且写完后设备仍只
  响应地址 1。因此出厂地址固定为 1，本脚本改完波特率后需配合
  configure_depth_sensor.py 把 D30 深温计地址改为 3。

地址方案（默认）：
  PWM 板  = 地址 1（硬件固定，无法软件修改）
  D30 深温计 = 地址 3（通过 tools/configure_depth_sensor.py 修改）

用法（RK3588 串口 /dev/ttyS5）：
  python3 /opt/ros/rov_ros2_ws/tools/configure_pwm_board.py

配置完成后：
  python3 /opt/ros/rov_ros2_ws/tools/configure_depth_sensor.py
  ./start_all.sh stop && ./start_all.sh bg
"""
import os, sys, time, struct, select, termios

SERIAL_PORT = os.environ.get('PWM_PORT', '/dev/ttyS5')
BAUD_DEFAULT = 115200   # PWM 板出厂默认波特率
BAUD_TARGET = 19200     # 与 D30 深温计一致
ADDR_DEFAULT = 0x01     # PWM 板出厂默认地址（硬件固定）
BAUD_CODE_19200 = 0x01  # 寄存器 0x000B: 0=9600, 1=19200, 2=115200


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


def build_write_cmd(addr, reg, value):
    cmd = struct.pack('>BBHH', addr, 0x06, reg, value)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def build_write_multiple_cmd(addr, reg, values):
    """构造正确的 0x10 写多个寄存器命令"""
    byte_count = len(values) * 2
    cmd = struct.pack('>BBHHH', addr, 0x10, reg, len(values), byte_count)
    for v in values:
        cmd += struct.pack('>H', v)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def write_register(ser, addr, reg, value, label):
    cmd = build_write_cmd(addr, reg, value)
    print(f'  -> {label}: {cmd.hex()}')
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    if len(resp) < 8:
        print(f'     [WARN] 短响应 {len(resp)}/8: {resp.hex()}')
        return False
    crc_exp = modbus_crc16(resp[:6])
    crc_got = struct.unpack('<H', resp[6:8])[0]
    if crc_exp != crc_got:
        print(f'     [INFO] 非标准响应: {resp.hex()} (此板写操作返回自定义 ACK，不表示失败)')
        return True  # HCX-8406 写寄存器常返回非标准 ACK，但配置仍生效
    print(f'     [OK] 响应: {resp.hex()}')
    return True


def read_register(ser, addr, reg, count=1):
    """读取一个或多个保持寄存器，返回 (ok, values_hex_list, response_hex)"""
    cmd = struct.pack('>BBHH', addr, 0x03, reg, count)
    crc = modbus_crc16(cmd)
    cmd += struct.pack('<H', crc)
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    expected = 5 + count * 2
    resp = ser.read(expected)
    if len(resp) < expected:
        return False, [], resp.hex()
    crc_exp = modbus_crc16(resp[:-2])
    crc_got = struct.unpack('<H', resp[-2:])[0]
    if crc_exp != crc_got or resp[0] != addr or resp[1] != 0x03:
        return False, [], resp.hex()
    vals = [struct.unpack('>H', resp[3 + i * 2:5 + i * 2])[0] for i in range(count)]
    return True, vals, resp.hex()


def try_change_address(ser, from_addr, to_addr):
    """尝试把地址从 from_addr 改为 to_addr，返回是否成功"""
    print(f'\n  [尝试] 把地址 {from_addr} 改为 {to_addr} ...')

    # 方法 1: 0x06 写单个寄存器
    cmd = build_write_cmd(from_addr, 0x000A, to_addr)
    print(f'    -> 0x06 写地址: {cmd.hex()}')
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    print(f'    <- 响应: {resp.hex()}')

    # 立即验证新地址
    ok, vals, _ = read_register(ser, to_addr, 0x0000)
    if ok:
        print(f'    [OK] 地址已成功改为 {to_addr}，心跳={vals[0]}')
        return True

    # 方法 2: 0x10 写多个寄存器（正确格式）
    print(f'    [INFO] 0x06 未生效，尝试 0x10 ...')
    cmd = build_write_multiple_cmd(from_addr, 0x000A, [to_addr])
    print(f'    -> 0x10 写地址: {cmd.hex()}')
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.05)
    resp = ser.read(8)
    print(f'    <- 响应: {resp.hex()}')

    ok, vals, _ = read_register(ser, to_addr, 0x0000)
    if ok:
        print(f'    [OK] 地址已成功改为 {to_addr}，心跳={vals[0]}')
        return True

    print(f'    [FAIL] 地址无法改为 {to_addr}，设备仍只响应地址 {from_addr}')
    return False


def main():
    print(f'=== HCX-8406 PWM 板配置 ===')
    print(f'端口: {SERIAL_PORT}')
    print(f'目标: 波特率 {BAUD_TARGET}')
    print(f'      先尝试把地址从 {ADDR_DEFAULT} 改为 2；若失败则保持地址 1，')
    print(f'      并配合 configure_depth_sensor.py 把 D30 改为地址 3。')
    print()

    if not os.path.exists(SERIAL_PORT):
        print(f'[ERROR] 串口 {SERIAL_PORT} 不存在')
        sys.exit(1)

    # 首先检查 PWM 板是否已经是目标配置（支持幂等执行）
    print(f'[0/3] 先以 {BAUD_TARGET} 探测，看 PWM 板是否已配置好...')
    try:
        ser = NativeSerial(SERIAL_PORT, BAUD_TARGET, timeout=0.5)
    except Exception as e:
        print(f'[ERROR] 打开串口失败: {e}')
        sys.exit(1)

    ok, vals, _ = read_register(ser, ADDR_DEFAULT, 0x000B)
    if ok and vals[0] == BAUD_CODE_19200:
        hb_ok, hb_vals, _ = read_register(ser, ADDR_DEFAULT, 0x0000)
        print(f'      [OK] PWM 板已在 {BAUD_TARGET}/addr{ADDR_DEFAULT}，无需重复配置')
        if hb_ok:
            print(f'      心跳寄存器 = {hb_vals[0]}')
        ser.close()
        print()
        print('注意：PWM 板地址无法通过软件修改。')
        print('      默认地址方案：PWM=1（硬件固定），D30 深温计=3')
        print('      下一步：python3 /opt/ros/rov_ros2_ws/tools/configure_depth_sensor.py')
        return
    ser.close()

    # 步骤 1: 用出厂 115200 连接（此时 D30 因波特率不同不会响应，可安全访问 PWM）
    print(f'\n[1/3] 以 {BAUD_DEFAULT} 连接 PWM 板（出厂默认）...')
    try:
        ser = NativeSerial(SERIAL_PORT, BAUD_DEFAULT, timeout=0.5)
    except Exception as e:
        print(f'[ERROR] 打开串口失败: {e}')
        sys.exit(1)

    # 读取当前地址和波特率
    ok, vals, _ = read_register(ser, ADDR_DEFAULT, 0x000A)
    if ok:
        print(f'      当前地址寄存器 0x000A = {vals[0]}')
    ok, vals, _ = read_register(ser, ADDR_DEFAULT, 0x000B)
    if ok:
        print(f'      当前波特率寄存器 0x000B = {vals[0]} (0=9600,1=19200,2=115200)')

    # 步骤 2: 尝试改地址（实际测试表明 HCX-8406 不支持，但保留尝试逻辑）
    print(f'[2/3] 尝试把 PWM 地址改为 2...')
    addr_changed = try_change_address(ser, ADDR_DEFAULT, 2)
    active_addr = 2 if addr_changed else ADDR_DEFAULT

    # 步骤 3: 改波特率
    print(f'\n[3/3] 写入新波特率 19200（目标地址 {active_addr}）...')
    if not write_register(ser, active_addr, 0x000B, BAUD_CODE_19200, '波特率寄存器 0x000B'):
        print('[ERROR] 波特率写入失败，中止')
        ser.close()
        sys.exit(1)
    ser.close()
    time.sleep(0.5)

    # 验证：以 19200 读取心跳寄存器
    print(f'\n[验证] 以 {BAUD_TARGET} 重新打开串口，读取心跳寄存器...')
    try:
        ser = NativeSerial(SERIAL_PORT, BAUD_TARGET, timeout=0.5)
    except Exception as e:
        print(f'[ERROR] 以 {BAUD_TARGET} 打开串口失败: {e}')
        sys.exit(1)

    ok, vals, resp = read_register(ser, active_addr, 0x0000)
    if ok:
        print(f'[OK] 验证成功！心跳寄存器 = {vals[0]}')
        print(f'[OK] PWM 板已配置为 波特率={BAUD_TARGET}，地址={active_addr}')
    else:
        print(f'[WARN] 验证响应异常: {resp}')

    ser.close()
    print()
    if addr_changed:
        print('地址已成功改为 2，可与 D30(addr=1) 共存。')
        print('下一步：')
        print('  ./start_all.sh stop && ./start_all.sh bg')
    else:
        print('注意：PWM 板地址无法通过软件修改（已尝试 0x06 和 0x10）。')
        print('      默认地址方案：PWM=1（硬件固定），D30 深温计=3')
        print('      若你已将 PWM 板硬件地址改为 2，请设置环境变量 PWM_ADDR=2 并改回 DEPTH_ADDR=1。')
        print()
        print('下一步：')
        print('  python3 /opt/ros/rov_ros2_ws/tools/configure_depth_sensor.py')
        print('  ./start_all.sh stop && ./start_all.sh bg')


if __name__ == '__main__':
    main()
