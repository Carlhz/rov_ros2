"""USBPcap capture with CORRECT offsets from analyze_pcap_v2.py."""
import subprocess, time, os, struct, sys

OUTPUT = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_fixed.pcap'
USPCAP = r'C:\Program Files\USBPcap\USBPcapCMD.exe'

if os.path.exists(OUTPUT):
    os.remove(OUTPUT)

print('Starting capture on \\\\.\\USBPcap1 (15 seconds)...')
proc = subprocess.Popen(
    [USPCAP, '-d', r'\\.\USBPcap1', '-o', OUTPUT, '-s', '4096', '-A'],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE)

time.sleep(1)
if proc.poll() is not None:
    out, err = proc.communicate()
    print(f'USBPcapCMD FAILED! stderr={err.decode(errors="replace")}')
    sys.exit(1)

print('Streaming camera for 12 seconds...')
import cv2
for idx in range(5):
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f'  Camera on index {idx}')
        for i in range(360):
            cap.read()
        cap.release()
        break
else:
    print('  Camera not found!')

time.sleep(2)
proc.terminate()
try:
    proc.wait(timeout=5)
except:
    proc.kill()

if not os.path.exists(OUTPUT):
    print('No capture file!')
    sys.exit(1)

size = os.path.getsize(OUTPUT)
print(f'\nCapture: {size/1024:.1f} KB')

# ===== VERIFIED CORRECT OFFSETS (from analyze_pcap_v2.py) =====
# USBPcap header (27 bytes minimum):
#   0-1: headerLen (USHORT)
#   2-9: irpId (QWORD)
#   10-13: status (ULONG)
#   14-15: urb_function (USHORT)
#   16: info (UCHAR)
#   17-18: bus (USHORT)
#   19-20: device (USHORT)
#   21: endpoint (UCHAR)
#   22: transfer (UCHAR)  - 0=ISO, 1=INTR, 2=CTRL, 3=BULK
#   23-26: dataLength (ULONG)

URB_CTRL = 0x0008
URB_BULK_INT = 0x0009
URB_ISOCH = 0x001A

with open(OUTPUT, 'rb') as f:
    data = f.read()

offset = 24  # skip pcap global header
pkt_num = 0
stats = {'ctrl': 0, 'intr': 0, 'iso': 0, 'bulk': 0, 'other': 0}
ep_summary = {}
all_ctrl = []
all_intr = []

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
    pkt_num += 1
    
    # CORRECT offsets
    header_len = struct.unpack_from('<H', pkt, 0)[0]
    if header_len < 27 or incl_len < header_len:
        continue
    
    irp_id = struct.unpack_from('<Q', pkt, 2)[0]
    status = struct.unpack_from('<I', pkt, 10)[0]
    urb_func = struct.unpack_from('<H', pkt, 14)[0]
    info = pkt[16]
    bus = struct.unpack_from('<H', pkt, 17)[0]
    dev = struct.unpack_from('<H', pkt, 19)[0]
    ep = pkt[21]
    transfer = pkt[22]
    data_len = struct.unpack_from('<I', pkt, 23)[0]
    
    key = (dev, ep)
    if key not in ep_summary:
        ep_summary[key] = {'count': 0, 'xfer': transfer, 'urb': urb_func}
    ep_summary[key]['count'] += 1
    
    xfer_names = {0: 'ISO', 1: 'INTR', 2: 'CTRL', 3: 'BULK'}
    if transfer in xfer_names:
        key2 = xfer_names[transfer].lower()
        stats[key2] += 1
    else:
        stats['other'] += 1
    
    # Payload
    pld_start = header_len
    pld = pkt[pld_start:pld_start + min(data_len, incl_len - pld_start)] if data_len > 0 else b''
    
    # Control transfers with URB_FUNCTION_CONTROL_TRANSFER
    if transfer == 2 and urb_func == URB_CTRL and header_len >= 35:
        setup = pkt[27:35]  # setup packet at offset 27 after USBPcap header
        bmReq = setup[0]
        bReq = setup[1]
        wValue = struct.unpack('<H', setup[2:4])[0]
        wIndex = struct.unpack('<H', setup[4:6])[0]
        wLength = struct.unpack('<H', setup[6:8])[0]
        ctrl_data = pkt[35:incl_len] if header_len > 35 else b''
        all_ctrl.append({
            'num': pkt_num, 'dev': dev, 'ep': ep,
            'bmReq': bmReq, 'bReq': bReq,
            'wValue': wValue, 'wIndex': wIndex, 'wLength': wLength,
            'type': (bmReq >> 5) & 0x03,
            'data': ctrl_data[:min(wLength, 64)], 'urb': urb_func
        })
    
    # Interrupt transfers
    if transfer == 1 and len(pld) > 0:
        all_intr.append({
            'num': pkt_num, 'dev': dev, 'ep': ep,
            'dlen': len(pld), 'data': pld[:32], 'urb': urb_func
        })

print(f'Packets: {pkt_num}')
print(f'CTRL={stats["ctrl"]}  INTR={stats["intr"]}  ISO={stats["iso"]}  BULK={stats["bulk"]}  OTHER={stats["other"]}')

print(f'\n=== Endpoint Distribution ===')
for (dev, ep), info in sorted(ep_summary.items()):
    xfer_names = {0: 'ISO', 1: 'INTR', 2: 'CTRL', 3: 'BULK'}
    urb_names = {0x0008: 'CTRL_XFER', 0x0009: 'BULK/INT', 0x001A: 'ISOCH'}
    print(f'  Dev{dev} EP{ep:02X}: {info["count"]} pkts, xfer={xfer_names.get(info["xfer"],str(info["xfer"]))}, urb={urb_names.get(info["urb"], f"0x{info['urb']:04X}")}')

if all_intr:
    print(f'\n*** FOUND {len(all_intr)} INTERRUPT PACKETS ***')
    for p in all_intr[:20]:
        print(f'  Dev{p["dev"]} EP{p["ep"]:02X} {p["dlen"]}B: {p["data"][:16].hex(" ")}')
    if len(all_intr) > 20:
        print(f'  ... and {len(all_intr)-20} more')
    
    # Parse first IMU data
    first = all_intr[0]
    if first['dlen'] >= 8:
        raw = first['data']
        hdr = struct.unpack('<H', raw[0:2])[0]
        x = struct.unpack('<h', raw[2:4])[0]
        y = struct.unpack('<h', raw[4:6])[0]
        z = struct.unpack('<h', raw[6:8])[0]
        print(f'\nFirst IMU: Header=0x{hdr:04X}  X={x}  Y={y}  Z={z}')
        print(f'  (12-bit): X={x&0x0FFF}  Y={y&0x0FFF}  Z={z&0x0FFF}')
else:
    print('\n*** NO INTERRUPT PACKETS ***')

if all_ctrl:
    type_names = {0: 'STD', 1: 'CLS', 2: 'VEN'}
    uvc_names = {0x01: 'SET_CUR', 0x81: 'GET_CUR', 0x82: 'GET_MIN',
                 0x83: 'GET_MAX', 0x84: 'GET_RES', 0x85: 'GET_LEN', 0x86: 'GET_INFO'}
    print(f'\n=== Control Transfers ({len(all_ctrl)}) ===')
    for c in all_ctrl[:40]:
        t = type_names.get(c['type'], str(c['type']))
        d = 'IN' if c['bmReq'] & 0x80 else 'OUT'
        name = uvc_names.get(c['bReq'], '')
        print(f'  [{t}] Dev{c["dev"]} EP{c["ep"]:02X} {d} bReq=0x{c["bReq"]:02X}({name:8s}) '
              f'wVal=0x{c["wValue"]:04X} wIdx=0x{c["wIndex"]:04X} wLen={c["wLength"]} '
              f'| {c["data"].hex(" ")[:60]}')

print(f'\nDone. File: {OUTPUT}')
