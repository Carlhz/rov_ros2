"""Analyze PCAP and show device/endpoint distribution."""
import struct, sys, os

PCAP = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_imu_stream.pcap'

with open(PCAP, 'rb') as f:
    data = f.read()

print(f'File size: {len(data)} bytes')

offset = 24
pkt_num = 0
devices = {}  # dev_id -> {ep -> count}
ctrl_packets = []
intr_packets = []
iso_packets = []
bulk_packets = []

while offset + 16 <= len(data):
    incl_len = struct.unpack_from('<I', data, offset+8)[0]
    offset += 16
    if offset + incl_len > len(data):
        break
    pkt = data[offset:offset+incl_len]
    offset += incl_len
    pkt_num += 1
    
    if len(pkt) < 28:
        continue
    hdr_len = struct.unpack('<H', pkt[0:2])[0]
    if hdr_len < 28:
        continue
    
    dev = struct.unpack('<H', pkt[19:21])[0]
    ep = pkt[21]
    xfer = pkt[22] & 0x03
    xfer_flags = pkt[22]
    dlen = struct.unpack('<I', pkt[24:28])[0]
    ts = struct.unpack('<Q', pkt[8:16])[0]
    
    if dev not in devices:
        devices[dev] = {}
    key = f'EP{ep:02X}'
    if key not in devices[dev]:
        devices[dev][key] = {'count': 0, 'xfer_type': xfer, 'flags': xfer_flags, 'dlen': dlen}
    devices[dev][key]['count'] += 1
    
    payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
    
    # Track first few packets per type
    info = {'num': pkt_num, 'dev': dev, 'ep': ep, 'xfer': xfer, 'flags': xfer_flags, 
            'dlen': dlen, 'ts': ts, 'data': payload[:32]}

print(f'Total packets parsed: {pkt_num}')

# Device summary
print(f'\n=== DEVICE / ENDPOINT DISTRIBUTION ===')
for dev in sorted(devices.keys()):
    print(f'Device {dev}:')
    for ep_key in sorted(devices[dev].keys()):
        ep_info = devices[dev][ep_key]
        xfer_names = {0: 'ISO', 1: 'INTR', 2: 'CTRL', 3: 'BULK'}
        print(f'  {ep_key}: {ep_info["count"]} pkts, type={xfer_names.get(ep_info["xfer_type"], "?")}, '
              f'dlen={ep_info["dlen"]}')

# Find the YLX camera device - try to identify by control transfers
print(f'\n=== CONTROL TRANSFERS (first 30) ===')
ctrl_count = 0
offset = 24
while offset + 16 <= len(data):
    incl_len = struct.unpack_from('<I', data, offset+8)[0]
    offset += 16
    if offset + incl_len > len(data):
        break
    pkt = data[offset:offset+incl_len]
    offset += incl_len
    
    if len(pkt) < 28:
        continue
    hdr_len = struct.unpack('<H', pkt[0:2])[0]
    if hdr_len < 28:
        continue
    
    dev = struct.unpack('<H', pkt[19:21])[0]
    ep = pkt[21]
    xfer = pkt[22] & 0x03
    dlen = struct.unpack('<I', pkt[24:28])[0]
    
    if xfer != 2:
        continue
    
    payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
    bmReq = payload[0] if len(payload) > 0 else 0
    bReq = payload[1] if len(payload) > 1 else 0
    wValue = struct.unpack('<H', payload[2:4])[0] if len(payload) >= 4 else 0
    wIndex = struct.unpack('<H', payload[4:6])[0] if len(payload) >= 6 else 0
    wLength = struct.unpack('<H', payload[6:8])[0] if len(payload) >= 8 else 0
    req_type = (bmReq >> 5) & 0x03
    t_names = {0: 'STD', 1: 'CLS', 2: 'VEN'}
    data_hex = payload[8:8+min(wLength, 64)].hex(' ') if len(payload) > 8 and wLength > 0 else ''
    
    ctrl_count += 1
    if ctrl_count <= 30:
        d_flag = 'IN' if bmReq & 0x80 else 'OUT'
        print(f'#{ctrl_count:3d} Dev{dev} EP{ep:02X} {d_flag} [{t_names.get(req_type, "?")}] '
              f'bReq=0x{bReq:02X} wVal=0x{wValue:04X} wIdx=0x{wIndex:04X} wLen={wLength} '
              f'data={data_hex[:80]}')
