#!/usr/bin/env python3
"""
Send UVC Extension Unit queries to YLX camera on Windows via libusb/winusb,
while simultaneously capturing with USBPcap to verify control transfer format.
"""
import subprocess, os, sys, time, struct, ctypes
from ctypes import wintypes

OUTPUT_FILE = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture3.pcapng"

# XU#4 GUID: 63610682-5070-49ab-b8cc-b3855e8d221d
XU4_GUID = "{63610682-5070-49ab-b8cc-b3855e8d221d}"

def find_device_path():
    """Find the YLX camera USB device path for WinUSB."""
    import winreg
    import re
    
    vid_pid_pattern = re.compile(r'VID_1BCF&PID_0B15', re.IGNORECASE)
    
    # Enumerate USB devices
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, 
                            r'SYSTEM\CurrentControlSet\Enum\USB')
        idx = 0
        while True:
            try:
                subkey_name = winreg.EnumKey(key, idx)
                if vid_pid_pattern.search(subkey_name):
                    full_path = r'SYSTEM\CurrentControlSet\Enum\USB\\' + subkey_name
                    # Find MI_00 (camera interface)
                    try:
                        dev_key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, full_path)
                        mi_idx = 0
                        while True:
                            try:
                                mi_name = winreg.EnumKey(dev_key, mi_idx)
                                mi_path = full_path + '\\' + mi_name
                                try:
                                    mi_key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, mi_path)
                                    # Read device interface GUIDs
                                    # The device path is under Device Parameters
                                    try:
                                        param_key = winreg.OpenKey(mi_key, 'Device Parameters')
                                        symlink = winreg.QueryValueEx(param_key, 'SymbolicName')[0]
                                        print(f"Found: {mi_path}\n  SymLink: {symlink}")
                                        # Convert to device path
                                        # \\?\usb#vid_1bcf...
                                        return symlink
                                    except:
                                        pass
                                    winreg.CloseKey(mi_key)
                                except:
                                    pass
                                mi_idx += 1
                            except WindowsError:
                                break
                        winreg.CloseKey(dev_key)
                    except:
                        pass
                idx += 1
            except WindowsError:
                break
        winreg.CloseKey(key)
    except Exception as e:
        print(f"Registry error: {e}")
    
    return None

def find_winusb_path():
    """Find WinUSB device path for YLX camera."""
    import winreg
    import re
    
    vid_pid = re.compile(r'VID_1BCF.*PID_0B15|vid_1bcf.*pid_0b15', re.IGNORECASE)
    
    # Search in USB device interface GUIDs
    guid = '{88bae032-5a81-49f0-bc3d-a4ff138216d6}'  # GUID_DEVINTERFACE_USB_DEVICE
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                           fr'SYSTEM\CurrentControlSet\Control\DeviceClasses')
        # Too many subkeys, try different approach...
    except:
        pass
    
    # Use SetupAPI through devcon or pnputil
    # Simpler: list all USB devices via WMI
    print("Searching for YLX camera WinUSB path...")
    
    try:
        # Use SetupDiGetClassDevs via Python
        result = subprocess.run(
            ['pnputil', '/enum-devices', '/class', 'USB', '/connected'],
            capture_output=True, text=True, timeout=10
        )
        for line in result.stdout.split('\n'):
            if '1BCF' in line.upper() or '0B15' in line.upper():
                print(f"  {line.strip()}")
    except:
        pass
    
    # Look for the device interface path
    # On Windows, the path is typically:
    # \\?\USB#VID_1BCF&PID_0B15&MI_00#...
    # or \\?\usb#vid_1bcf&pid_0b15&mi_00#...
    
    # Search with listusb-like approach
    try:
        result = subprocess.run(
            ['wmic', 'path', 'Win32_USBControllerDevice', 'get', 'Dependent'],
            capture_output=True, text=True, timeout=10
        )
        for line in result.stdout.split('\n'):
            if '1BCF' in line.upper() or '0B15' in line.upper():
                print(f"  {line.strip()}")
    except:
        pass
    
    return None


def start_capture():
    """Start USBPcap capture in background."""
    USBPcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
    
    if os.path.exists(OUTPUT_FILE):
        os.remove(OUTPUT_FILE)
    
    # Find device
    devices = []
    for i in range(1, 5):
        try:
            handle = ctypes.windll.kernel32.CreateFileW(
                r"\\.\USBPcap" + str(i), 0x80000000, 3, None, 3, 0, None)
            if handle not in (-1, 0):
                ctypes.windll.kernel32.CloseHandle(handle)
                devices.append(i)
        except:
            continue
    
    device = r"\\.\USBPcap" + str(devices[0]) if devices else r"\\.\USBPcap1"
    print(f"USBPcap device: {device}")
    
    cmd = [USBPcap_exe, "-d", device, "-o", OUTPUT_FILE, "-A", "-s", "256"]
    proc = subprocess.Popen(cmd, cwd=r"C:\Program Files\USBPcap",
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(1)
    return proc


def stop_capture(proc):
    """Stop USBPcap capture."""
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except:
            proc.kill()
    time.sleep(1)
    
    if os.path.exists(OUTPUT_FILE):
        size_kb = os.path.getsize(OUTPUT_FILE) / 1024
        print(f"\nCapture file: {OUTPUT_FILE} ({size_kb:.1f} KB)")
        return OUTPUT_FILE
    return None


def try_pyusb_control():
    """Try sending UVC control transfers via PyUSB."""
    try:
        import usb.core
        import usb.util
        
        print("\n=== PyUSB Control Transfer Test ===")
        
        # Find YLX device
        dev = usb.core.find(idVendor=0x1BCF, idProduct=0x0B15)
        if dev is None:
            print("Device not found via PyUSB")
            return False
        
        print(f"Found device: {dev}")
        
        # Detach kernel driver if needed
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
                print("Detached kernel driver from interface 0")
        except Exception as e:
            print(f"  (driver detach: {e})")
        
        try:
            dev.set_configuration()
            print("Configuration set")
        except Exception as e:
            print(f"  (config: {e})")
        
        # Try GET_INFO on XU#4
        # bmRequestType: 0xA1 (D2H, class, interface)
        # bRequest: UVC_GET_INFO = 0x86
        # wValue: (control_selector << 8) | 0
        # wIndex: (entity_id << 8) | interface_number
        # wLength: 1
        
        print("\nTesting XU#4 queries...")
        
        for selector in range(0, 26):  # 25 controls
            try:
                data = dev.ctrl_transfer(
                    bmRequestType=0xA1,  # D2H, class, interface
                    bRequest=0x86,        # GET_INFO
                    wValue=(selector << 8),
                    wIndex=(4 << 8) | 0,  # entity 4, interface 0
                    data_or_wLength=1,
                    timeout=500
                )
                if data and len(data) > 0:
                    print(f"  XU#4 sel={selector}: GET_INFO = {data[0]:02X} (supported)" if data[0] else f"  XU#4 sel={selector}: GET_INFO = 0 (unsupported)")
            except Exception as e:
                pass  # Most will fail, that's expected
        
        return True
        
    except ImportError:
        print("PyUSB not installed. Installing...")
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'pyusb'], timeout=30)
        print("PyUSB installed. Please run this script again.")
        return False
    except Exception as e:
        print(f"PyUSB error: {e}")
        return False


def try_libwdi_list():
    """List USB devices using a simple approach."""
    print("\n=== USB Device Enumeration ===")
    
    # Use Python's wmi or just system commands
    try:
        result = subprocess.run(
            ['pnputil', '/enum-devices', '/connected'],
            capture_output=True, text=True, timeout=10
        )
        in_camera = False
        for line in result.stdout.split('\n'):
            if '1BCF' in line.upper() or '0B15' in line.upper():
                in_camera = True
                print(line.strip())
            elif in_camera and 'Device Description' in line:
                print(line.strip())
            elif in_camera and 'Hardware ID' in line:
                print(line.strip())
            elif in_camera and 'Instance ID' in line:
                instance_id = line.split(':', 1)[-1].strip()
                print(f"Instance ID: {instance_id}")
                in_camera = False
    except:
        pass


def main():
    print("=" * 60)
    print("YLX UVC Control Transfer Capture Test")
    print("=" * 60)
    
    # List devices
    try_libwdi_list()
    find_winusb_path()
    find_device_path()
    
    print("\n" + "=" * 60)
    print("Starting capture + PyUSB test...")
    print("=" * 60)
    
    # Start capture
    proc = start_capture()
    time.sleep(1)
    
    # Try PyUSB
    result = try_pyusb_control()
    
    if not result:
        print("\nPyUSB test failed. Capturing for 10 more seconds of interrupt data...")
        # Still capture interrupt data
        time.sleep(10)
    
    # Stop capture
    stop_capture(proc)
    
    print("\nDone! Analyze with: python analyze_pcap_v2.py")


if __name__ == '__main__':
    main()
