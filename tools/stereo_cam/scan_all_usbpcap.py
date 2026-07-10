"""Scan all USBPcap devices to find which has interrupt traffic."""
import subprocess, time, os, struct

USPCAP = r'C:\Program Files\USBPcap\USBPcapCMD.exe'
OUTDIR = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam'

for idx in range(1, 7):
    output = os.path.join(OUTDIR, f'scan_usbpcap{idx}.pcap')
    if os.path.exists(output):
        os.remove(output)
    
    device = f'\\\\.\\USBPcap{idx}'
    print(f'\n--- Testing {device} ---')
    
    proc = subprocess.Popen(
        [USPCAP, '-d', device, '-o', output, '-s', '4096', '-A'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    
    time.sleep(3)
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except:
        proc.kill()
    
    if not os.path.exists(output) or os.path.getsize(output) < 50:
        print(f'  No data (< 50 bytes)')
        continue
    
    # Quick parse
    with open(output, 'rb') as f:
        data = f.read()
    
    size = len(data)
    offset = 24
    pkt_num, ctrl, intr, iso, bulk = 0, 0, 0, 0, 0
    devices = set()
    ep_types = {}
    
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
        devices.add(dev)
        
        if xfer == 0: iso += 1
        elif xfer == 1: intr += 1
        elif xfer == 2: ctrl += 1
        elif xfer == 3: bulk += 1
    
    print(f'  {size/1024:.1f}KB, pkts={pkt_num}, devs={sorted(devices)}, '
          f'CTRL={ctrl} INTR={intr} ISO={iso} BULK={bulk}')
    
    if intr > 0:
        print(f'  *** FOUND INTERRUPT TRAFFIC ON {device}! ***')

print('\nDone scanning.')
