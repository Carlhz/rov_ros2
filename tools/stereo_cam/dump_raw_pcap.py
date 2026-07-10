"""Dump raw pcap bytes - find correct USB header offsets."""
import struct

PCAP = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_fixed.pcap'

with open(PCAP, 'rb') as f:
    data = f.read()

print(f'Total size: {len(data)} bytes')

offset = 24
pkt_idx = 0

while offset + 16 <= len(data) and pkt_idx < 10:
    ts_sec = struct.unpack_from('<I', data, offset)[0]
    ts_usec = struct.unpack_from('<I', data, offset + 4)[0]
    incl_len = struct.unpack_from('<I', data, offset + 8)[0]
    orig_len = struct.unpack_from('<I', data, offset + 12)[0]
    
    offset += 16
    pkt_end = offset + incl_len
    if pkt_end > len(data) or incl_len < 27:
        offset = pkt_end
        continue
    
    pkt = data[offset:pkt_end]
    
    # Only show packets with data (dlen > 0)
    hdr_len = struct.unpack_from('<H', pkt, 0)[0]
    
    # Try all possible layouts
    print(f'\n=== Pkt{pkt_idx+1}: incl={incl_len} hdrLen={hdr_len} ===')
    
    # Raw bytes
    print(f'  Raw: {" ".join(f"{pkt[i]:02X}" for i in range(min(hdr_len+4, len(pkt))))}')
    
    # The headerLen could be 27 (compact) or variable
    # Try: urb_func at offset (hdr_len - 9), transfer at (hdr_len - 5), dlen at (hdr_len - 4)
    # Common USBPcap layout (27-byte header):
    # 0-1: headerLen
    # 2-3: ?  (maybe something)
    # 4-11: irpId or timestamp low
    # 12-15: status
    # 16-17: urb_function
    # 18: info
    # 19-20: bus
    # 21-22: device  
    # 23: endpoint
    # 24: transfer_type
    # 25-26: padding or part of dataLength?
    
    # OR the 27-byte layout:
    # headerLen = 27 means:
    # 0-1: headerLen (2)
    # 2-3: ? (2)
    # 4-11: ... (8)
    # 12-15: ... (4)
    # 16-17: urb_function (2)
    # 18: info (1)
    # 19-20: bus (2)
    # 21-22: device (2)
    # 23: endpoint (1)
    # 24: transfer_type (1)
    # 25: padding (1)
    # 26: dataLength? Or 25-26 as USHORT?
    # Wait, 2+2+8+4+2+1+2+2+1+1 = 25. Need 2 more bytes for dataLength.
    # Maybe dataLength is at 25-26 as USHORT?
    
    # Let me try: at hdr_len - 4, read dlen as ULONG (won't work with 27-byte header)
    # Actually, maybe the 27-byte header doesn't include dlen, and dlen = incl_len - hdr_len
    payload_len = incl_len - hdr_len
    print(f'  payload_len = incl_len - hdr_len = {payload_len}')
    
    # Try various interpretations of the bytes
    # Look at offset pattern in the raw data
    urb = struct.unpack_from('<H', pkt, 16)[0]
    info = pkt[18]
    bus = struct.unpack_from('<H', pkt, 19)[0]
    dev = struct.unpack_from('<H', pkt, 21)[0]
    ep = pkt[23]
    xfer = pkt[24]
    
    xfer_names = {0: 'ISO', 1: 'INTR', 2: 'CTRL', 3: 'BULK'}
    print(f'  urb=0x{urb:04X} info=0x{info:02X} bus={bus} dev={dev} ep=0x{ep:02X} xfer={xfer}({xfer_names.get(xfer, "?")}) dlen_calc={payload_len}')
    
    # Check if this looks right by comparing with known data
    if payload_len > 0 and hdr_len < len(pkt):
        pld = pkt[hdr_len:hdr_len + min(payload_len, 16)]
        print(f'  payload: {" ".join(f"{b:02X}" for b in pld)}')
    
    pkt_idx += 1
    offset = pkt_end

# Now scan for interrupt packets (xfer=1)
print('\n\n=== Scanning all packets for INTR ===')
offset = 24
intr_count = 0
while offset + 16 <= len(data):
    ts_sec = struct.unpack_from('<I', data, offset)[0]
    ts_usec = struct.unpack_from('<I', data, offset + 4)[0]
    incl_len = struct.unpack_from('<I', data, offset + 8)[0]
    orig_len = struct.unpack_from('<I', data, offset + 12)[0]
    
    offset += 16
    pkt_end = offset + incl_len
    if pkt_end > len(data) or incl_len < 27:
        offset = pkt_end
        continue
    
    pkt = data[offset:pkt_end]
    hdr_len = struct.unpack_from('<H', pkt, 0)[0]
    
    if hdr_len >= 27 and incl_len >= hdr_len:
        xfer = pkt[24]
        if xfer == 1:
            intr_count += 1
            bus = struct.unpack_from('<H', pkt, 19)[0]
            dev = struct.unpack_from('<H', pkt, 21)[0]
            ep = pkt[23]
            payload_len = incl_len - hdr_len
            pld = pkt[hdr_len:hdr_len + min(payload_len, 16)] if payload_len > 0 else b''
            print(f'  INTR: bus={bus} dev={dev} ep=0x{ep:02X} dlen={payload_len} data={" ".join(f"{b:02X}" for b in pld[:16])}')
            if intr_count >= 20:
                print(f'  ... stopping at 20')
                break
    
    offset = pkt_end

if intr_count == 0:
    print('  NO interrupt packets found with xfer=1 at offset 24')
    
    # Try different offset for transfer type
    print('\n  Trying xfer at offset 22...')
    offset = 24
    count22 = 0
    while offset + 16 <= len(data) and count22 < 5:
        ts_sec = struct.unpack_from('<I', data, offset)[0]
        ts_usec = struct.unpack_from('<I', data, offset + 4)[0]
        incl_len = struct.unpack_from('<I', data, offset + 8)[0]
        offset += 16
        pkt_end = offset + incl_len
        if pkt_end > len(data) or incl_len < 27:
            offset = pkt_end
            continue
        pkt = data[offset:pkt_end]
        hdr_len = struct.unpack_from('<H', pkt, 0)[0]
        xfer22 = pkt[22] if len(pkt) > 22 else 0
        if xfer22 == 1:
            count22 += 1
            print(f'    Found at offset 22: incl={incl_len} xfer22=1')
        offset = pkt_end
    print(f'  Found {count22} with xfer=1 at offset 22')
