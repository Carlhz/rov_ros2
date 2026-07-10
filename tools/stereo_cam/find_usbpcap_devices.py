"""Find all USBPcap devices and identify which one has the YLX camera"""
import ctypes
from ctypes import wintypes
import os

kernel32 = ctypes.windll.kernel32
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 1
FILE_SHARE_WRITE = 2
OPEN_EXISTING = 3

print("=== Checking USBPcap devices ===")
for i in range(10):
    path = r'\\.\USBPcap' + str(i)
    h = kernel32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE, None, OPEN_EXISTING, 0x80, None)
    err = kernel32.GetLastError()
    if h and int(h) != -1:
        print(f"USBPcap{i}: FOUND (handle={int(h)})")
        kernel32.CloseHandle(h)
    else:
        print(f"USBPcap{i}: NOT AVAILABLE (err={err})")

# Check USBPcapCMD
print(f"\n=== USBPcapCMD ===")
usbpcap_path = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
print(f"Exists: {os.path.exists(usbpcap_path)}")

# Also check if Wireshark is available
for ws_path in [
    r"C:\Program Files\Wireshark\Wireshark.exe",
    r"C:\Program Files (x86)\Wireshark\Wireshark.exe",
]:
    if os.path.exists(ws_path):
        print(f"Wireshark: {ws_path}")
        # Check for usbpcap via Wireshark
        ws_dir = os.path.dirname(ws_path)
        for ext in ['extcap', 'usbpcap']:
            test = os.path.join(ws_dir, ext)
            if os.path.exists(test):
                print(f"  {ext}: exists")
