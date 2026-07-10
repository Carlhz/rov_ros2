"""Capture USB during camera replug to catch UVC XU#4 initialization.
User flow: ylx_xu_cap_replug.pcap will be created upon script exit.
"""
import subprocess, time, os, struct, sys

OUTPUT = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_xu_cap_replug.pcap'
USPCAP = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

if os.path.exists(OUTPUT):
    os.remove(OUTPUT)

print("="*60)
print("Starting USBPcap capture NOW.")
print("Please unplug YLX camera, wait 3 seconds, then replug it.")
print("The script will auto-detect camera and open it.")
print("="*60)

# Start capture
proc = subprocess.Popen(
    [USPCAP, '-d', r'\\.\USBPcap1', '-o', OUTPUT, '-s', '4096', '-A', '--inject-descriptors'],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print(f"\nUSBPcapCMD PID: {proc.pid} - CAPTURING...")

# Wait for user to replug + Windows to enumerate
print("Waiting 15 seconds for replug + enumeration...")
for i in range(15, 0, -1):
    print(f"  {i}s...", end='\r')
    time.sleep(1)
print("  Now opening camera to trigger streaming...")

# Open camera
try:
    import cv2
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if cap.isOpened():
        print("  Camera opened, streaming 5 seconds...")
        for i in range(150):  # ~5s at 30fps
            ret, _ = cap.read()
            if i % 30 == 0 and ret:
                print(f"    Frame {i}", end='\r')
        cap.release()
        print("  Camera released                    ")
    else:
        print("  WARN: Cannot open camera index 0")
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        if cap.isOpened():
            print("  Camera opened on index 1")
            for i in range(150):
                ret, _ = cap.read()
            cap.release()
except Exception as e:
    print(f"  OpenCV error: {e}")

time.sleep(2)

# Stop capture
print("\nStopping capture...")
try:
    proc.terminate()
    proc.wait(timeout=5)
except:
    proc.kill()
    proc.wait()
time.sleep(1)

# Check
if not os.path.exists(OUTPUT):
    print("[FAIL] No capture file!")
    sys.exit(1)

size = os.path.getsize(OUTPUT)
print(f"\nCapture file: {size} bytes ({size/1024:.1f} KB)")

# ============ ANALYSIS ============
with open(OUTPUT, 'rb') as f:
    data = f.read()

offset = 24
pkt_num = 0
stats = {'ctrl': 0, 'intr': 0, 'iso': 0, 'bulk': 0, 'other': 0}
devices = set()
uvc_ctrl = []     # UVC/Vendor control transfers with full data
all_ctrl = []     # All setup packets
intr_data = []    # Interrupt packets

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
    dlen = struct.unpack('<I', pkt[24:28])[0]
    
    devices.add(dev)
    if xfer == 2: stats['ctrl'] += 1
    elif xfer == 1: stats['intr'] += 1
    elif xfer == 0: stats['iso'] += 1
    elif xfer == 3: stats['bulk'] += 1
    else: stats['other'] += 1
    
    payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
    
    # Control transfers
    if xfer == 2 and len(payload) >= 8:
        bmReq = payload[0]; bReq = payload[1]
        wValue = struct.unpack('<H',payload[2:4])[0]
        wIndex = struct.unpack('<H',payload[4:6])[0]
        wLength = struct.unpack('<H',payload[6:8])[0]
        req_type = (bmReq >> 5) & 0x03
        recipient = bmReq & 0x1F
        data_bytes = payload[8:8+min(wLength, 128)] if wLength > 0 and len(payload) > 8 else b''
        
        entry = {
            'num': pkt_num, 'dev': dev, 'ep': ep,
            'bmReq': bmReq, 'bReq': bReq,
            'wValue': wValue, 'wIndex': wIndex, 'wLength': wLength,
            'type': req_type, 'recipient': recipient,
            'data': data_bytes
        }
        all_ctrl.append(entry)
        if req_type in (1, 2):  # Class or Vendor
            uvc_ctrl.append(entry)
    
    # Interrupt packets
    if xfer == 1 and len(payload) > 0:
        intr_data.append({
            'num': pkt_num, 'dev': dev, 'ep': ep,
            'dlen': dlen, 'data': payload[:32]
        })

print(f"\n{'='*60}")
print(f"CAPTURE SUMMARY")
print(f"{'='*60}")
print(f"Packets: {pkt_num} | Devices: {sorted(devices)}")
print(f"Control: {stats['ctrl']} | Interrupt: {stats['intr']} | Iso: {stats['iso']} | Bulk: {stats['bulk']} | Other: {stats['other']}")

# Show all UVC/Vendor control transfers
uvc_codes = {0x01:'SET_CUR', 0x81:'GET_CUR', 0x82:'GET_MIN',
             0x83:'GET_MAX', 0x84:'GET_RES', 0x85:'GET_LEN', 0x86:'GET_INFO'}
type_names = {0:'STD', 1:'CLS', 2:'VEN'}
recip_names = {0:'DEV', 1:'IF', 2:'EP'}

if uvc_ctrl:
    print(f"\n{'='*60}")
    print(f"UVC/VENDOR CONTROL TRANSFERS ({len(uvc_ctrl)})")
    print(f"{'='*60}")
    for c in uvc_ctrl:
        t = type_names.get(c['type'], '?')
        d = 'IN ' if (c['bmReq'] & 0x80) else 'OUT'
        r = recip_names.get(c['recipient'], f"X{c['recipient']}")
        name = uvc_codes.get(c['bReq'], '')
        data_hex = c['data'].hex(' ') if c['data'] else ''
        print(f"[{t}] #{c['num']:5d} Dev{c['dev']} {d} {r} bReq=0x{c['bReq']:02X}({name:8s}) "
              f"wVal=0x{c['wValue']:04X} wIdx=0x{c['wIndex']:04X} wLen={c['wLength']} data={data_hex}")
else:
    print("\n[NO UVC/VENDOR control transfers found!]")

# Show first interrupt packets  
if intr_data:
    print(f"\n{'='*60}")
    print(f"INTERRUPT PACKETS ({len(intr_data)})")
    print(f"{'='*60}")
    for p in intr_data[:20]:
        d_hex = p['data'].hex(' ') if len(p['data']) <= 32 else p['data'][:16].hex(' ')+'...'
        print(f"#{p['num']:5d} Dev{p['dev']} EP0x{p['ep']:02X} {p['dlen']}B: {d_hex}")
    if len(intr_data) > 20:
        print(f"... and {len(intr_data)-20} more")
    
    # Try to parse IMU data: header[2] + X[2] + Y[2] + Z[2] (high 12 bits valid)
    if intr_data:
        first = intr_data[0]
        raw = first['data']
        if first['dlen'] >= 8:
            hdr = struct.unpack('<H', raw[0:2])[0]
            x = struct.unpack('<h', raw[2:4])[0]
            y = struct.unpack('<h', raw[4:6])[0]
            z = struct.unpack('<h', raw[6:8])[0]
            print(f"\nFirst IMU parse: Header=0x{hdr:04X} X={x} Y={y} Z={z}")
            # Try high 12-bit interpretation
            x12 = x & 0x0FFF
            y12 = y & 0x0FFF
            z12 = z & 0x0FFF
            print(f"  (12-bit): X={x12} Y={y12} Z={z12}")
else:
    print(f"\n[NO interrupt packets!]")

# Show all control setups (class + vendor) with details - helps identify XU#4 commands
if all_ctrl:
    print(f"\n{'='*60}")
    print(f"ALL CONTROL SETUP PACKETS ({len(all_ctrl)})")
    print(f"{'='*60}")
    for c in all_ctrl[:100]:
        t = type_names.get(c['type'], '?')
        d = 'IN ' if (c['bmReq'] & 0x80) else 'OUT'
        r = recip_names.get(c['recipient'], f"X{c['recipient']}")
        name = uvc_codes.get(c['bReq'], '')
        data_hex = c['data'].hex(' ') if c['data'] and len(c['data']) <= 64 else \
                   (c['data'][:32].hex(' ')+'...' if c['data'] else '')
        print(f"[{t}] #{c['num']:5d} Dev{c['dev']} {d} {r} bReq=0x{c['bReq']:02X}({name:8s}) "
              f"wVal=0x{c['wValue']:04X} wIdx=0x{c['wIndex']:04X} wLen={c['wLength']} | {data_hex}")

print(f"\n{'='*60}")
print("DONE. File: " + OUTPUT)
print(f"{'='*60}")
