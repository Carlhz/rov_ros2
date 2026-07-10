"""Analyze IMU data captured from YLX camera (EP 0x82 interrupt)."""
import struct

PCAP = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_fixed.pcap'

with open(PCAP, 'rb') as f:
    data = f.read()

offset = 24
imu_packets = []

while offset + 16 <= len(data):
    incl_len = struct.unpack_from('<I', data, offset + 8)[0]
    offset += 16
    if offset + incl_len > len(data):
        break
    if incl_len < 27:
        offset += incl_len
        continue
    
    pkt = data[offset:offset + incl_len]
    offset += incl_len
    
    header_len = struct.unpack_from('<H', pkt, 0)[0]
    urb_func = struct.unpack_from('<H', pkt, 14)[0]
    dev = struct.unpack_from('<H', pkt, 19)[0]
    ep = pkt[21]
    transfer = pkt[22]
    data_len = struct.unpack_from('<I', pkt, 23)[0]
    
    # Interrupt on EP 0x82
    if transfer == 1 and ep == 0x82 and data_len >= 8:
        ts_sec = struct.unpack_from('<I', data, offset - incl_len - 16)[0]
        ts_usec = struct.unpack_from('<I', data, offset - incl_len - 12)[0]
        pld = pkt[header_len:header_len + min(data_len, incl_len - header_len)]
        imu_packets.append({
            'ts': ts_sec + ts_usec / 1_000_000,
            'raw': pld[:8]
        })

print(f'Total IMU packets: {len(imu_packets)}')

if not imu_packets:
    print('No data!')
    exit()

# Rate calculation
if len(imu_packets) > 1:
    duration = imu_packets[-1]['ts'] - imu_packets[0]['ts']
    rate = len(imu_packets) / duration if duration > 0 else 0
    print(f'Duration: {duration:.1f}s, Rate: {rate:.1f} Hz')

# Analyze first 50 packets
print('\n=== First 30 packets ===')
print(f'{"#":>4s} {"ts_diff":>8s} {"header":>6s} {"raw(2-7)":>30s} {"X(int16)":>8s} {"Y(int16)":>8s} {"Z(int16)":>8s}')
prev_ts = imu_packets[0]['ts']

for i, p in enumerate(imu_packets[:30]):
    dt = (p['ts'] - prev_ts) * 1000
    prev_ts = p['ts']
    raw = p['raw']
    hdr = struct.unpack('<H', raw[0:2])[0]
    x = struct.unpack('<h', raw[2:4])[0]
    y = struct.unpack('<h', raw[4:6])[0]
    z = struct.unpack('<h', raw[6:8])[0]
    raw_hex = ' '.join(f'{b:02X}' for b in raw[2:8])
    print(f'{i+1:4d} {dt:7.1f}ms 0x{hdr:04X}  {raw_hex}  {x:8d} {y:8d} {z:8d}')

# Deduplicate to find unique values
unique = {}
for p in imu_packets:
    key = bytes(p['raw'])
    if key not in unique:
        unique[key] = 0
    unique[key] += 1

print(f'\n=== Value Distribution ===')
print(f'Unique values: {len(unique)} / {len(imu_packets)} total')
if len(unique) < 30:
    for key, count in sorted(unique.items(), key=lambda x: -x[1]):
        hdr = struct.unpack('<H', key[0:2])[0]
        x = struct.unpack('<h', key[2:4])[0]
        y = struct.unpack('<h', key[4:6])[0]
        z = struct.unpack('<h', key[6:8])[0]
        print(f'  0x{hdr:04X} x={x:6d} y={y:6d} z={z:6d}  [{count}x]')

# Try different interpretations
print(f'\n=== Alternative Parsing of First 5 Packets ===')
for p in imu_packets[:5]:
    raw = p['raw']
    print(f'\n  Raw: {" ".join(f"{b:02X}" for b in raw[:8])}')
    
    # Interpretation 1: int16 x 3 (gyro only)
    x, y, z = struct.unpack('<hhh', raw[2:8])
    print(f'  int16 x3:  X={x:6d}  Y={y:6d}  Z={z:6d}')
    
    # Interpretation 2: 12-bit (high 12 bits of int16)
    x12, y12, z12 = x & 0x0FFF, y & 0x0FFF, z & 0x0FFF
    print(f'  12-bit:     X={x12:4d}  Y={y12:4d}  Z={z12:4d}')
    
    # Interpretation 3: int16 x 3 + 2 reserved
    v0, v1, v2, v3 = struct.unpack('<hhhh', raw)
    print(f'  int16 x4:   V0={v0} V1={v1} V2={v2} V3={v3}')
    
    # Interpretation 4: uint16 x 4
    u0, u1, u2, u3 = struct.unpack('<HHHH', raw)
    print(f'  uint16 x4:  U0={u0} U1={u1} U2={u2} U3={u3}')

# Look at header byte patterns
headers = {}
for p in imu_packets:
    hdr = struct.unpack('<H', p['raw'][0:2])[0]
    headers[hdr] = headers.get(hdr, 0) + 1

print(f'\n=== Header Byte Patterns ===')
print(f'Distinct headers: {len(headers)}')
for hdr, cnt in sorted(headers.items(), key=lambda x: -x[1])[:10]:
    print(f'  0x{hdr:04X}: {cnt} packets ({cnt*100/len(imu_packets):.1f}%)')
