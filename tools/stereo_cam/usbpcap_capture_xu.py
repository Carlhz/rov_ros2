#!/usr/bin/env python3
"""
USBPcap capture for UVC XU#4 control transfer analysis.
Focus: capture USB control transfers (SET CUR) targeting XU#4.
"""
import ctypes, ctypes.wintypes as w
import time, subprocess, os, struct, sys

# --- Step 1: Find USBPcap device ---
def find_usbpcap_devices():
    """List all USBPcap devices and their corresponding root hubs"""
    kernel32 = ctypes.windll.kernel32
    devices = []
    for i in range(10):
        path = f"\\\\.\\USBPcap{i}"
        h = kernel32.CreateFileW(path, 0x80000000, 0x3, None, 3, 0x80, None)
        if h and h != w.HANDLE(-1).value:
            devices.append(path)
            kernel32.CloseHandle(w.HANDLE(h))
    return devices

# --- Step 2: Get USB device tree ---
def get_camera_info():
    """Find camera device address and root hub"""
    import winreg
    
    # Find camera in device tree
    for bus_num in range(10):
        try:
            key_path = f"SYSTEM\\CurrentControlSet\\Enum\\USB\\VID_1BCF&PID_0B15"
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path)
            i = 0
            while True:
                try:
                    subkey = winreg.EnumKey(key, i)
                    full = f"{key_path}\\{subkey}"
                    sk = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, full)
                    try:
                        parent, _ = winreg.QueryValueEx(sk, "ParentIdPrefix")
                        print(f"  Camera: {full}")
                        print(f"  ParentIdPrefix: {parent}")
                    except:
                        pass
                    try:
                        svc, _ = winreg.QueryValueEx(sk, "Service")
                        print(f"  Service: {svc}")
                    except:
                        pass
                    winreg.CloseKey(sk)
                    i += 1
                except OSError:
                    break
            winreg.CloseKey(key)
        except:
            pass
    
    # Get bus/device address from sysfs-like info
    # Query device location
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
            "SYSTEM\\CurrentControlSet\\Enum\\USB\\VID_1BCF&PID_0B15\\01.00.00")
        addr, _ = winreg.QueryValueEx(key, "Address")
        print(f"  USB Address: {addr}")
        winreg.CloseKey(key)
    except:
        pass

# --- Step 3: Start capture ---
def start_capture(usbpcap_device, output_file, device_addr=None):
    """Start USBPcapCMD in background to capture USB traffic"""
    cmd = [
        r"C:\Program Files\USBPcap\USBPcapCMD.exe",
        "-d", usbpcap_device,
        "-o", output_file,
        "-s", "1024",  # snapshot length
        "-A",  # capture all devices on root hub
        "--inject-descriptors"
    ]
    
    print(f"Starting: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, 
                           stderr=subprocess.PIPE, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    return proc

# --- Step 4: Open camera to trigger UVC init ---
def trigger_camera():
    """Open camera via DirectShow to trigger UVC initialization.
    The IMU data starts automatically on Windows when camera is plugged in,
    but opening the camera ensures the UVC init sequence is captured."""
    # Use basic CreateFile approach - just open the camera device
    # This triggers the UVC driver probe/init
    try:
        # Try OpenCV
        import cv2
        for idx in range(3):
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if cap.isOpened():
                print(f"  Opened camera {idx}")
                for i in range(20):
                    ret, frame = cap.read()
                    if ret:
                        print(f"    Frame {i}: {frame.shape[1]}x{frame.shape[0]}")
                        break
                cap.release()
                print("  Camera triggered successfully")
                return True
        print("  No camera found at indices 0-2")
    except ImportError:
        print("  OpenCV not installed, using fallback...")
    except Exception as e:
        print(f"  OpenCV error: {e}")
    
    # Fallback: use Windows Media Foundation to enumerate and init cameras
    try:
        import winreg
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
            "SYSTEM\\CurrentControlSet\\Enum\\USB\\VID_1BCF&PID_0B15\\01.00.00")
        print("  Camera exists in registry (already detected by OS)")
        winreg.CloseKey(key)
    except:
        pass
    
    return False

# --- Step 5: Analyze pcap for UVC control transfers ---
def analyze_pcap_uvc(pcap_file):
    """Analyze pcap for UVC XU control transfers"""
    with open(pcap_file, 'rb') as f:
        data = f.read()
    
    # Parse pcap global header
    magic = struct.unpack('<I', data[0:4])[0]
    if magic == 0xA1B2C3D4:
        endian = '<'
    elif magic == 0xD4C3B2A1:
        endian = '>'
    else:
        print(f"Unknown pcap magic: {hex(magic)}")
        return
    
    offset = 24  # after global header
    pkt_num = 0
    control_transfers = []
    irp_packets = []
    
    while offset + 16 <= len(data):
        ts_sec, ts_usec, incl_len, orig_len = struct.unpack(f'{endian}IIII', data[offset:offset+16])
        offset += 16
        
        if offset + incl_len > len(data):
            break
        pkt_data = data[offset:offset+incl_len]
        offset += incl_len
        pkt_num += 1
        
        if len(pkt_data) < 28:
            continue
        
        # USB pcap header (classic pcap LINKTYPE_USB_2_0 = 220)
        # Header: header_len(2) + irp_id(8) + status(4) + function(2) + info(1)
        # + bus(2) + device(2) + ep(1) + transfer(1) + data_len(4)
        header_len = struct.unpack('<H', pkt_data[0:2])[0]
        if header_len < 28:
            continue
        
        irp_id = struct.unpack('<Q', pkt_data[2:10])[0]
        status = struct.unpack('<I', pkt_data[10:14])[0]
        usb_function = struct.unpack('<H', pkt_data[14:16])[0]  # URB_FUNCTION_*
        info = pkt_data[16]
        bus, dev = struct.unpack('<HH', pkt_data[17:21])
        ep_addr = pkt_data[21]
        transfer_type = pkt_data[22] & 0x03  # 0=iso, 1=interrupt, 2=control, 3=bulk
        data_len = struct.unpack('<I', pkt_data[24:28])[0]
        
        payload = pkt_data[header_len:header_len + data_len] if data_len > 0 else b''
        
        # URB_FUNCTION_CONTROL_TRANSFER = 0x0008
        # URB_FUNCTION_VENDOR_DEVICE = 0x0011
        # URB_FUNCTION_VENDOR_INTERFACE = 0x0012
        is_control = (transfer_type == 2)
        is_vendor = (usb_function in [0x0008, 0x0011, 0x0012])
        
        # Check for UVC-class requests (bmRequestType bit 5=class, bits 0-4=interface)
        if is_control and len(payload) >= 8:
            bmRequestType = payload[0] if len(payload) > 0 else 0
            bRequest = payload[1] if len(payload) > 1 else 0
            wValue = struct.unpack('<H', payload[2:4])[0] if len(payload) >= 4 else 0
            wIndex = struct.unpack('<H', payload[4:6])[0] if len(payload) >= 6 else 0
            wLength = struct.unpack('<H', payload[6:8])[0] if len(payload) >= 8 else 0
            
            # UVC class: bmRequestType bits 5-6 = 01 (class)
            req_type = (bmRequestType >> 5) & 0x03
            is_uvc_class = (req_type == 1)  # Class request
            is_uvc_vendor = (req_type == 2)  # Vendor request
            direction = "OUT" if (bmRequestType & 0x80) == 0 else "IN"
            recipient = bmRequestType & 0x1F
            recipient_name = ["Device","Interface","Endpoint","Other","Other",
                            "Other","Other","Other"][recipient] if recipient < 8 else f"Unknown({recipient})"
            
            # UVC control selectors: SET_CUR = 0x01, GET_CUR=0x81, GET_LEN=0x85, GET_INFO=0x86
            uvc_codes = {0x01: "SET_CUR", 0x81: "GET_CUR", 0x82: "GET_MIN",
                        0x83: "GET_MAX", 0x84: "GET_RES", 0x85: "GET_LEN", 0x86: "GET_INFO"}
            
            if is_uvc_class or bRequest in uvc_codes:
                control_transfers.append({
                    'pkt': pkt_num,
                    'bmRequestType': f'0x{bmRequestType:02X}',
                    'bRequest': f'0x{bRequest:02X} ({uvc_codes.get(bRequest, "?")})' if bRequest in uvc_codes else f'0x{bRequest:02X}',
                    'direction': direction,
                    'recipient': recipient_name,
                    'wValue': f'0x{wValue:04X}',
                    'wIndex': f'0x{wIndex:04X}',
                    'wLength': wLength,
                    'data': payload[8:8+wLength].hex(' ') if wLength > 0 and len(payload) > 8 else '',
                    'full_payload': payload.hex(' '),
                    'usb_function': f'0x{usb_function:04X}',
                    'transfer_type': transfer_type,
                    'dev': dev,
                    'ep': f'0x{ep_addr:02X}',
                    'len': data_len,
                })
        
        # Also track interrupt/IMU packets
        if transfer_type == 1:  # Interrupt
            irp_packets.append({
                'pkt': pkt_num,
                'ep': f'0x{ep_addr:02X}',
                'dev': dev,
                'len': data_len,
                'data': payload.hex(' ') if len(payload) <= 32 else payload[:16].hex(' ') + '...',
            })
    
    # Print results
    print(f"\n{'='*60}")
    print(f"Total packets: {pkt_num}")
    print(f"UVC control transfers: {len(control_transfers)}")
    print(f"Interrupt (IMU) packets: {len(irp_packets)}")
    
    if control_transfers:
        print(f"\n=== UVC Control Transfers (potential XU commands) ===")
        for ct in control_transfers:
            print(f"\nPkt#{ct['pkt']} | Dev#{ct['dev']} | {ct['direction']} | {ct['recipient']}")
            print(f"  bmReq={ct['bmRequestType']} bReq={ct['bRequest']}")
            print(f"  wValue={ct['wValue']} wIndex={ct['wIndex']} wLen={ct['wLength']}")
            print(f"  USB fn={ct['usb_function']} transfer={ct['transfer_type']}")
            if ct['data']:
                print(f"  Data: {ct['data']}")
            print(f"  Full: {ct['full_payload']}")
    
    if irp_packets:
        print(f"\n=== First 10 Interrupt (IMU) packets ===")
        for ip in irp_packets[:10]:
            print(f"Pkt#{ip['pkt']} | Dev#{ip['dev']} | EP{ip['ep']} | {ip['len']}B: {ip['data']}")
    
    return control_transfers, irp_packets

# --- Main ---
if __name__ == "__main__":
    print("=== Step 1: Find USBPcap devices ===")
    devices = find_usbpcap_devices()
    print(f"Found {len(devices)} USBPcap devices: {devices}")
    
    if not devices:
        print("ERROR: No USBPcap devices found. Did you install USBPcap?")
        print("Download from: https://desowin.org/usbpcap/")
        sys.exit(1)
    
    print("\n=== Step 2: Camera info ===")
    get_camera_info()
    
    # Start capture on first USBPcap device
    pcap_file = os.path.join(os.path.dirname(__file__), "ylx_uvc_capture.pcap")
    usbpcap_dev = devices[0]
    
    print(f"\n=== Step 3: Start capture ({usbpcap_dev}) ===")
    print(f"Output: {pcap_file}")
    
    proc = start_capture(usbpcap_dev, pcap_file)
    print(f"Capture process PID: {proc.pid}")
    
    # Wait a moment for capture to start
    time.sleep(2)
    
    print("\n=== Step 4: Trigger camera initialization ===")
    print("Opening camera to trigger UVC init sequence...")
    trigger_camera()
    
    # Let capture run for a few more seconds
    print("Capturing for 5 more seconds...")
    time.sleep(5)
    
    # Stop capture
    print("Stopping capture...")
    try:
        proc.terminate()
        proc.wait(timeout=5)
    except:
        proc.kill()
    
    time.sleep(1)
    
    # Analyze
    print(f"\n=== Step 5: Analyze {pcap_file} ===")
    if os.path.exists(pcap_file) and os.path.getsize(pcap_file) > 0:
        analyze_pcap_uvc(pcap_file)
    else:
        print(f"ERROR: Capture file empty or missing. Size: {os.path.getsize(pcap_file) if os.path.exists(pcap_file) else 'N/A'}")
