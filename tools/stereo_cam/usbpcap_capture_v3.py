#!/usr/bin/env python3
"""USBPcap capture V3 - properly handle file lifecycle and trigger camera"""
import subprocess, time, os, sys, struct

USBPcap_EXE = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
OUTPUT_DIR = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam"
USBPcap_DEV = r"\\.\USBPcap1"

def capture_and_analyze():
    # Use timestamped filename
    ts = int(time.time())
    pcap_file = os.path.join(OUTPUT_DIR, f"ylx_cap_{ts}.pcap")
    
    # Ensure no stale file
    try:
        os.remove(pcap_file)
    except:
        pass
    
    print(f"Output: {pcap_file}")
    print(f"Starting USBPcapCMD on {USBPcap_DEV}...")
    
    # Start capture process
    proc = subprocess.Popen(
        [USBPcap_EXE, "-d", USBPcap_DEV, "-o", pcap_file,
         "-s", "1024", "-A", "--inject-descriptors"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    print(f"PID={proc.pid}, waiting for init...")
    
    # Wait for capture to initialize
    time.sleep(3)
    
    # Check if process is still alive
    if proc.poll() is not None:
        stdout = proc.stdout.read().decode(errors='replace')
        stderr = proc.stderr.read().decode(errors='replace')
        print(f"ERROR: Process exited early! code={proc.returncode}")
        if stdout: print(f"stdout: {stdout}")
        if stderr: print(f"stderr: {stderr}")
        return
    
    # Trigger camera
    print("Opening camera...")
    try:
        import cv2
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if cap.isOpened():
            print("Camera opened, reading frames...")
            for i in range(20):
                ret, _ = cap.read()
                if ret and i == 0:
                    print("  First frame OK")
            cap.release()
            print("Camera released")
        else:
            print("WARNING: Could not open camera 0")
    except Exception as e:
        print(f"Camera error: {e}")
    
    # Continue capturing
    print("Capturing 8 more seconds...")
    time.sleep(8)
    
    # Stop
    print("Stopping capture...")
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except:
        proc.kill()
        proc.wait()
    
    time.sleep(1)
    
    # Check file
    if not os.path.exists(pcap_file):
        print(f"ERROR: No file created!")
        return
    
    size = os.path.getsize(pcap_file)
    print(f"File size: {size} bytes")
    
    if size <= 24:
        print("ERROR: Only pcap header")
        return
    
    # Analyze
    with open(pcap_file, 'rb') as f:
        data = f.read()
    
    offset = 24
    pkt_num = 0
    stats = {'control': 0, 'interrupt': 0, 'iso': 0, 'bulk': 0, 'other': 0}
    devices = set()
    setup_packets = []
    uvc_packets = []
    imu_packets = []
    
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
        
        if xfer == 2:
            stats['control'] += 1
        elif xfer == 1:
            stats['interrupt'] += 1
        elif xfer == 0:
            stats['iso'] += 1
        elif xfer == 3:
            stats['bulk'] += 1
        else:
            stats['other'] += 1
        
        payload = pkt[hdr_len:hdr_len+dlen] if dlen > 0 else b''
        
        # Control transfers with setup packet
        if xfer == 2 and dlen >= 8:
            bmReq = payload[0]
            bReq = payload[1]
            wValue = struct.unpack('<H', payload[2:4])[0]
            wIndex = struct.unpack('<H', payload[4:6])[0]
            wLength = struct.unpack('<H', payload[6:8])[0]
            req_type = (bmReq >> 5) & 0x03
            
            sp = (pkt_num, dev, ep, bmReq, bReq, wValue, wIndex, wLength,
                  payload[8:8+min(wLength, 64)].hex(' ') if wLength > 0 else '')
            setup_packets.append(sp)
            
            if req_type in (1, 2):  # Class or Vendor
                uvc_packets.append(sp)
        
        # Interrupt = IMU
        if xfer == 1:
            imu_packets.append((pkt_num, dev, ep, dlen, 
                              payload.hex(' ') if dlen <= 32 else payload[:16].hex(' ')+'...'))
    
    # Print summary
    print(f"\n{'='*70}")
    print(f"Total packets: {pkt_num}")
    print(f"Devices: {sorted(devices)}")
    print(f"Control: {stats['control']} | Interrupt: {stats['interrupt']} | Iso: {stats['iso']} | Bulk: {stats['bulk']}")
    
    if uvc_packets:
        print(f"\n=== UVC/Vendor Control Transfers ({len(uvc_packets)}) ===")
        uvc_codes = {0x01:"SET_CUR", 0x81:"GET_CUR", 0x82:"GET_MIN",
                    0x83:"GET_MAX", 0x84:"GET_RES", 0x85:"GET_LEN", 0x86:"GET_INFO"}
        for p in uvc_packets:
            num, dev, ep, bmReq, bReq, wVal, wIdx, wLen, d = p
            kind = "UVC" if ((bmReq>>5)&3)==1 else "VEN"
            dirs = "IN" if (bmReq&0x80) else "OUT"
            name = uvc_codes.get(bReq, '')
            print(f"  [{kind}] #{num} Dev{dev} {dirs} bReq=0x{bReq:02X}({name}) "
                  f"wVal=0x{wVal:04X} wIdx=0x{wIdx:04X} wLen={wLen} data={d}")
    
    if imu_packets:
        print(f"\n=== IMU Interrupt Packets ({len(imu_packets)}) ===")
        for p in imu_packets[:15]:
            num, dev, ep, dlen, d = p
            print(f"  #{num} Dev{dev} EP0x{ep:02X} {dlen}B: {d}")
        if len(imu_packets) > 15:
            print(f"  ... and {len(imu_packets)-15} more")
    
    # Show ALL setup packets (standard UVC included)
    if setup_packets:
        print(f"\n=== ALL Setup Packets (first 100) ===")
        for p in setup_packets[:100]:
            num, dev, ep, bmReq, bReq, wVal, wIdx, wLen, d = p
            req_type = (bmReq >> 5) & 0x03
            type_names = {0:'STD', 1:'CLS', 2:'VEN'}
            dirs = "IN" if (bmReq&0x80) else "OUT"
            print(f"  [{type_names.get(req_type,'?')}] #{num} Dev{dev} {dirs} "
                  f"bmReq=0x{bmReq:02X} bReq=0x{bReq:02X} wVal=0x{wVal:04X} wIdx=0x{wIdx:04X} wLen={wLen}")
    
    print(f"\n=== File saved: {pcap_file} ===")
    return pcap_file

if __name__ == "__main__":
    capture_and_analyze()
