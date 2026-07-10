"""Capture USB traffic from USBPcap1, trigger camera, analyze for UVC XU#4 control transfers"""
import subprocess, time, os, struct, sys

OUTPUT = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_xu_cap.pcap'
USPCAP = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

# Remove old file
if os.path.exists(OUTPUT):
    os.remove(OUTPUT)

print("[1] Starting USBPcapCMD capture (USBPcap1)...")
# Use -A mode for all traffic, -s 1024 snap length, --inject-descriptors
cmd = [
    USPCAP,
    '-d', r'\\.\USBPcap1',
    '-o', OUTPUT,
    '-s', '1024',
    '-A',
    '--inject-descriptors'
]

proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print(f"   PID: {proc.pid}")

# Wait for capture to start
time.sleep(1)

# [2] Open camera to trigger UVC initialization
print("[2] Opening camera (OpenCV) to trigger init...")
try:
    import cv2
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if cap.isOpened():
        print("   Camera opened")
        for i in range(50):
            ret, frame = cap.read()
            if ret and i % 10 == 0:
                print(f"   Frame {i}: {frame.shape[1]}x{frame.shape[0]}")
        cap.release()
        print("   Camera released")
    else:
        print("   FAILED to open camera index 0")
        # Try index 1
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        if cap.isOpened():
            print("   Camera opened on index 1")
            for i in range(50):
                ret, _ = cap.read()
                if ret and i % 10 == 0:
                    print(f"   Frame {i}")
            cap.release()
except Exception as e:
    print(f"   OpenCV error: {e}")
    # Fallback: try Windows API
    import ctypes
    try:
        import win32com.client
        print("   Trying DirectShow...")
    except:
        pass

time.sleep(2)

# [3] Kill capture
print("[3] Stopping capture...")
try:
    proc.terminate()
    proc.wait(timeout=5)
except:
    proc.kill()
    proc.wait()

time.sleep(1)

# [4] Check file
if not os.path.exists(OUTPUT):
    print("[FAIL] No capture file created!")
    sys.exit(1)

size = os.path.getsize(OUTPUT)
print(f"[4] Capture file: {size} bytes")

if size < 24:
    print(f"[FAIL] File too small ({size} bytes)")
    sys.exit(1)

# [5] Analyze
with open(OUTPUT, 'rb') as f:
    data = f.read()

# Parse pcap header
magic = struct.unpack('<I', data[0:4])[0]
print(f"   Magic: 0x{magic:08X}")

offset = 24
pkt_num = 0
stats = {'control': 0, 'interrupt': 0, 'iso': 0, 'bulk': 0, 'other': 0}
devices = set()
uvc_ctrl_packets = []
interrupt_packets = []

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
    
    usb_fn = struct.unpack('<H', pkt[14:16])[0]
    dev = struct.unpack('<H', pkt[19:21])[0]
    ep = pkt[21]
    xfer = pkt[22] & 0x03
    dlen = struct.unpack('<I', pkt[24:28])[0]
    
    devices.add(dev)
    
    if xfer == 2: stats['control'] += 1
    elif xfer == 1: stats['interrupt'] += 1
    elif xfer == 0: stats['iso'] += 1
    elif xfer == 3: stats['bulk'] += 1
    else: stats['other'] += 1
    
    payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
    
    # Control transfers - analyze setup packets
    if xfer == 2 and len(payload) >= 8:
        bmReq = payload[0]
        bReq = payload[1]
        wValue = struct.unpack('<H', payload[2:4])[0]
        wIndex = struct.unpack('<H', payload[4:6])[0]
        wLength = struct.unpack('<H', payload[6:8])[0]
        req_type = (bmReq >> 5) & 0x03
        
        if req_type in (1, 2):  # Class or Vendor
            uvc_ctrl_packets.append({
                'num': pkt_num, 'dev': dev, 'ep': ep,
                'bmReq': bmReq, 'bReq': bReq,
                'wValue': wValue, 'wIndex': wIndex, 'wLength': wLength,
                'data': payload[8:8+min(wLength, 64)]
            })
    
    # Interrupt packets (potential IMU data)
    if xfer == 1 and len(payload) > 0:
        interrupt_packets.append({
            'num': pkt_num, 'dev': dev, 'ep': ep,
            'dlen': dlen,
            'data': payload
        })

print(f"\n=== Capture Summary ===")
print(f"Total packets: {pkt_num}")
print(f"Devices: {sorted(devices)}")
print(f"Control: {stats['control']} | Interrupt: {stats['interrupt']} | Iso: {stats['iso']} | Bulk: {stats['bulk']} | Other: {stats['other']}")

print(f"\n=== UVC/Vendor Control Transfers ({len(uvc_ctrl_packets)}) ===")
uvc_codes = {0x01:'SET_CUR', 0x81:'GET_CUR', 0x82:'GET_MIN',
            0x83:'GET_MAX', 0x84:'GET_RES', 0x85:'GET_LEN', 0x86:'GET_INFO'}
for p in uvc_ctrl_packets:
    kind = 'UVC' if ((p['bmReq']>>5)&3)==1 else 'VEN'
    d = 'IN' if (p['bmReq']&0x80) else 'OUT'
    ri = {0:'DEV', 1:'IF', 2:'EP'}.get(p['bmReq']&0x1F, f"X{p['bmReq']&0x1F}")
    name = uvc_codes.get(p['bReq'], '')
    data_str = p['data'].hex(' ') if p['data'] else ''
    print(f"[{kind}] #{p['num']:5d} Dev{p['dev']} {d} {ri} bReq=0x{p['bReq']:02X}({name:8s}) "
          f"wVal=0x{p['wValue']:04X} wIdx=0x{p['wIndex']:04X} wLen={p['wLength']} data={data_str}")

if uvc_ctrl_packets:
    # Group by wValue (control selector from high byte for UVC)
    # For UVC: wValue high byte = control selector, low byte = unit ID
    print(f"\n=== Control Selectors (wValue breakdown) ===")
    # Check for XU#4: unit=0x04, so wIndex would contain the XU GUID somehow
    # Actually for XU: wIndex = XU unit ID in low byte + interface in high byte
    for p in uvc_ctrl_packets:
        unit = (p['wIndex'] >> 8) & 0xFF
        cs = (p['wValue'] >> 8) & 0xFF
        print(f"  #{p['num']:5d} wValue=0x{p['wValue']:04X} wIndex=0x{p['wIndex']:04X} "
              f"-> cs=0x{cs:02X} unit={unit} intf=0x{p['wIndex']&0xFF:02X}")

print(f"\n=== Interrupt Packets ({len(interrupt_packets)}) ===")
for p in interrupt_packets[:10]:
    d_hex = p['data'].hex(' ') if len(p['data']) <= 32 else p['data'][:16].hex(' ')+'...'
    print(f"#{p['num']:5d} Dev{p['dev']} EP0x{p['ep']:02X} {p['dlen']}B: {d_hex}")

if len(interrupt_packets) > 10:
    print(f"... and {len(interrupt_packets)-10} more")

# Show all control transfers (standard + class + vendor)
print(f"\n=== ALL Setup Packets (first 80) ===")
all_setup = []
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
    xfer = pkt[22] & 0x03
    dlen = struct.unpack('<I', pkt[24:28])[0]
    payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
    if xfer == 2 and len(payload) >= 8:
        bmReq = payload[0]
        rt = {0:'STD', 1:'CLS', 2:'VEN'}.get((bmReq>>5)&3, '?')
        d = 'IN ' if (bmReq&0x80) else 'OUT'
        ri = {0:'DEV', 1:'IF', 2:'EP'}.get(bmReq&0x1F, f"X{bmReq&0x1F}")
        all_setup.append((bmReq, pkt_num, rt, d, ri, payload[1], 
                         struct.unpack('<H',payload[2:4])[0],
                         struct.unpack('<H',payload[4:6])[0],
                         struct.unpack('<H',payload[6:8])[0],
                         payload[1]))

cnt = 0    
for s in all_setup:
    if cnt >= 80: break
    rt = s[2]
    if rt in ('CLS', 'VEN'):
        print(f"[{rt}] #{s[1]:5d} {s[3]} {s[4]} bReq=0x{s[5]:02X} wVal=0x{s[6]:04X} wIdx=0x{s[7]:04X} wLen={s[8]}")
    cnt += 1
