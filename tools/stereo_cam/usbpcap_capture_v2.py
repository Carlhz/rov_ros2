#!/usr/bin/env python3
"""
Capture USB traffic from YLX camera, focusing on UVC control transfers.
Strategy: start capture, open camera (triggers UVC init), capture for 10 seconds.
"""
import subprocess, time, os, sys, struct

usbpcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
pcap_file = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_uvc_capture.pcap"
usbpcap_dev = r"\\.\USBPcap1"

print("=== Starting USBPcap capture ===")
cmd = [
    usbpcap_exe, "-d", usbpcap_dev, "-o", pcap_file,
    "-s", "1024", "-A", "--inject-descriptors"
]
print(f"Command: {' '.join(cmd)}")

proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, 
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
print(f"Capture started (PID={proc.pid})")
time.sleep(2)

print("\n=== Opening camera to trigger UVC initialization ===")
import cv2
for idx in range(3):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f"Opened camera index {idx}")
        for i in range(30):
            ret, frame = cap.read()
            if ret and i % 10 == 0:
                print(f"  Frame {i}: {frame.shape[1]}x{frame.shape[0]}")
            elif ret and i == 0:
                print(f"  Frame 0: {frame.shape[1]}x{frame.shape[0]}")
        cap.release()
        print("Camera released")
        break

print("\n=== Waiting 8 seconds for IMU data ===")
time.sleep(8)

print("\n=== Stopping capture ===")
proc.terminate()
try:
    proc.wait(timeout=3)
except:
    proc.kill()
time.sleep(1)

if not os.path.exists(pcap_file):
    print(f"ERROR: No capture file!")
    sys.exit(1)

size = os.path.getsize(pcap_file)
print(f"Capture file: {size} bytes")

if size <= 24:
    print("ERROR: Only pcap header, no packets captured!")
    sys.exit(1)

# ===== ANALYSIS =====
with open(pcap_file, 'rb') as f:
    data = f.read()

offset = 24
uvc_transfers = []
interrupt_packets = []
all_setup_packets = []
pkt_num = 0

while offset + 16 <= len(data):
    ts_sec, ts_usec, incl_len, orig_len = struct.unpack('<IIII', data[offset:offset+16])
    offset += 16
    if offset + incl_len > len(data):
        break
    
    pkt_data = data[offset:offset+incl_len]
    offset += incl_len
    pkt_num += 1
    
    if len(pkt_data) < 28:
        continue
    
    hdr_len = struct.unpack('<H', pkt_data[0:2])[0]
    if hdr_len < 28:
        continue
    
    irp_id = struct.unpack('<Q', pkt_data[2:10])[0]
    status = struct.unpack('<i', pkt_data[10:14])[0]
    usb_function = struct.unpack('<H', pkt_data[14:16])[0]
    info = pkt_data[16]
    bus = struct.unpack('<H', pkt_data[17:19])[0]
    dev = struct.unpack('<H', pkt_data[19:21])[0]
    ep_addr = pkt_data[21]
    transfer_type = pkt_data[22] & 0x03
    data_len = struct.unpack('<I', pkt_data[24:28])[0]
    
    payload = pkt_data[hdr_len:hdr_len + data_len] if data_len > 0 else b''
    
    # Focus on:
    # 1. Control transfers (transfer_type == 2) -> UVC init commands
    # 2. Interrupt transfers (transfer_type == 1) -> IMU data
    
    if transfer_type == 2 and len(payload) >= 8:  # Control transfer with setup packet
        bmReq = payload[0]
        bReq = payload[1]
        wValue = struct.unpack('<H', payload[2:4])[0]
        wIndex = struct.unpack('<H', payload[4:6])[0]
        wLength = struct.unpack('<H', payload[6:8])[0]
        
        req_type = (bmReq >> 5) & 0x03
        direction = "IN" if (bmReq & 0x80) else "OUT"
        recipient = bmReq & 0x1F
        
        # UVC class request: bits 5-6 = 01
        is_uvc = (req_type == 1)
        
        # Vendor request (e.g. XU): bits 5-6 = 10
        is_vendor = (req_type == 2)
        
        uvc_codes = {0x01:"SET_CUR", 0x81:"GET_CUR", 0x82:"GET_MIN",
                    0x83:"GET_MAX", 0x84:"GET_RES", 0x85:"GET_LEN", 0x86:"GET_INFO"}
        
        if is_uvc or is_vendor:
            extra = payload[8:8+wLength].hex(' ') if wLength > 0 and len(payload) > 8 else ''
            
            entry = {
                'pkt': pkt_num, 'dev': dev, 'ep': f'0x{ep_addr:02X}',
                'dir': direction, 'fn': f'0x{usb_function:04X}',
                'bmReq': f'0x{bmReq:02X}', 'bReq': f'0x{bReq:02X}',
                'bReqName': uvc_codes.get(bReq, ''),
                'wValue': f'0x{wValue:04X}', 'wIndex': f'0x{wIndex:04X}',
                'wLen': wLength, 'data': extra,
                'is_uvc': is_uvc, 'is_vendor': is_vendor,
                'recipient': ['Device','Interface','Endpoint'][recipient] if recipient < 3 else f'R{recipient}',
                'full_payload': payload.hex(' '),
            }
            uvc_transfers.append(entry)
        
        all_setup_packets.append({
            'pkt': pkt_num, 'dev': dev, 'ep': f'0x{ep_addr:02X}',
            'bmReq': f'0x{bmReq:02X}', 'bReq': f'0x{bReq:02X}',
            'wValue': f'0x{wValue:04X}', 'wIndex': f'0x{wIndex:04X}',
            'wLen': wLength, 'payload': payload.hex(' '),
        })
    
    elif transfer_type == 1:  # Interrupt = IMU data
        interrupt_packets.append({
            'pkt': pkt_num, 'dev': dev,
            'ep': f'0x{ep_addr:02X}',
            'len': data_len,
            'data': payload.hex(' '),
        })

# Print results
print(f"\n{'='*70}")
print(f"Total packets: {pkt_num}")
print(f"Devices seen: {sorted(set(uvc_transfers[-1]['dev'] for uvc_transfers in [uvc_transfers]) if uvc_transfers else [])}")
print(f"UVC/Vendor control transfers: {len(uvc_transfers)}")
print(f"Interrupt (IMU) packets: {len(interrupt_packets)}")

if uvc_transfers:
    print(f"\n=== UVC/VENDOR Control Transfers (potential XU commands) ===")
    for ct in uvc_transfers:
        kind = "UVC" if ct['is_uvc'] else "VENDOR"
        print(f"\n[{kind}] Pkt#{ct['pkt']} | Dev{ct['dev']} | {ct['dir']} | {ct['recipient']} | EP{ct['ep']}")
        print(f"  setup: {ct['bmReq']} {ct['bReq']} {ct['bReqName']} wVal={ct['wValue']} wIdx={ct['wIndex']} wLen={ct['wLen']}")
        if ct['data']:
            print(f"  data: {ct['data']}")

# Show all setup packets for completeness (including standard UVC)
print(f"\n=== ALL Setup/Control Packets (first 50) ===")
for sp in all_setup_packets[:50]:
    print(f"Pkt#{sp['pkt']} Dev{sp['dev']} {sp['bmReq']} bReq={sp['bReq']} wVal={sp['wValue']} wIdx={sp['wIndex']} wLen={sp['wLen']}")
    if len(sp['payload']) > 16:
        print(f"  data: {sp['payload'][:80]}...")

if interrupt_packets:
    print(f"\n=== IMU Interrupt Packets (first 20) ===")
    for ip in interrupt_packets[:20]:
        print(f"Pkt#{ip['pkt']} Dev{ip['dev']} EP{ip['ep']} {ip['len']}B: {ip['data']}")
    if len(interrupt_packets) > 20:
        print(f"... and {len(interrupt_packets)-20} more")
    
    # Quick IMU data analysis
    if interrupt_packets:
        samples = [ip['data'].split() for ip in interrupt_packets[:50]]
        print(f"\n  First sample (hex bytes): {samples[0]}")
else:
    print("\n  *** WARNING: No IMU interrupt packets captured ***")
    print("  Camera may not be on this USBPcap device or not streaming")

# Check if the capture file has useful data
status_ok = len(uvc_transfers) > 0
print(f"\n=== Summary: {'SUCCESS' if status_ok else 'NO UVC TRANSFERS CAPTURED'} ===")
