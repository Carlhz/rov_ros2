#!/usr/bin/env python3
"""
Query YLX UVC Extension Unit #4 controls via Windows USB Video Class IOCTL.

UVC XU control protocol:
  GET_CUR = 0x01, GET_MIN = 0x02, GET_MAX = 0x03, GET_RES = 0x04
  SET_CUR = 0x81
  
  Control selector index = control_id (1-based from UVC descriptor)
  
We use the low-level approach: find the camera's device path, then send
UVCIOC_CTRL_QUERY via DeviceIoControl.
"""

import ctypes
from ctypes import wintypes
import struct

# Windows types
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 1
FILE_SHARE_WRITE = 2
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value

# IOCTL codes for USB Video Class
# CTL_CODE(DeviceType, Function, Method, Access)
FILE_DEVICE_UNKNOWN = 0x22
METHOD_BUFFERED = 0
METHOD_NEITHER = 3
FILE_ANY_ACCESS = 0

def CTL_CODE(dev_type, func, method, access):
    return (dev_type << 16) | (access << 14) | (func << 2) | method

# USB Video Class IOCTLs
IOCTL_UVCHOST_GET_DEVICE_DESCRIPTOR = CTL_CODE(0x21, 0x900, METHOD_BUFFERED, FILE_ANY_ACCESS)
IOCTL_UVCHOST_GET_CLASS_DESCRIPTOR    = CTL_CODE(0x21, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS)
IOCTL_UVCHOST_SET_QUERY_POST_PROCESS  = CTL_CODE(0x21, 0x902, METHOD_BUFFERED, FILE_ANY_ACCESS)

kernel32 = ctypes.windll.kernel32
setupapi = ctypes.windll.setupapi

# Find camera device path using SetupAPI
DIGCF_PRESENT = 0x02
DIGCF_DEVICEINTERFACE = 0x10

class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", wintypes.BYTE * 8),
    ]

class SP_DEVINFO_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("ClassGuid", GUID),
        ("DevInst", wintypes.DWORD),
        ("Reserved", ctypes.c_void_p),
    ]

class SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
    _fields_ = [
        ("cbSize", wintypes.DWORD),
        ("InterfaceClassGuid", GUID),
        ("Flags", wintypes.DWORD),
        ("Reserved", ctypes.c_void_p),
    ]

# KSCATEGORY_VIDEO_CAMERA
CAMERA_GUID = GUID(0xe5323777, 0xf976, 0x4f5b, (0x9b, 0x55, 0xb9, 0x46, 0x99, 0xc4, 0x6e, 0x44))
# KSCATEGORY_VIDEO
VIDEO_GUID = GUID(0x6994AD05, 0x93EF, 0x11D0, (0xA3, 0xCC, 0x00, 0xA0, 0xC9, 0x22, 0x31, 0x96))


def find_camera_paths():
    """Find all UVC camera device paths"""
    paths = []
    
    hdevinfo = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(CAMERA_GUID),
        None, None,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    )
    
    if hdevinfo == -1:
        # Try KS video category
        hdevinfo = setupapi.SetupDiGetClassDevsW(
            ctypes.byref(VIDEO_GUID),
            None, None,
            DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
        )
    
    if hdevinfo == -1:
        print("SetupDiGetClassDevs failed")
        return paths
    
    idx = 0
    dev_interface = SP_DEVICE_INTERFACE_DATA()
    dev_interface.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)
    
    while setupapi.SetupDiEnumDeviceInterfaces(hdevinfo, None, ctypes.byref(CAMERA_GUID), idx, ctypes.byref(dev_interface)):
        # Get required buffer size
        required_size = wintypes.DWORD()
        setupapi.SetupDiGetDeviceInterfaceDetailW(
            hdevinfo, ctypes.byref(dev_interface),
            None, 0, ctypes.byref(required_size), None
        )
        
        detail_data = ctypes.create_string_buffer(required_size.value)
        detail = ctypes.cast(detail_data, ctypes.POINTER(wintypes.BYTE))
        
        pdetail = ctypes.cast(detail_data, ctypes.c_void_p)
        ctypes.memset(pdetail, 0, required_size.value)
        
        # Set cbSize manually
        detail_data_raw = bytearray(required_size.value)
        struct.pack_into('I', detail_data_raw, 0, ctypes.sizeof(SP_DEVICE_INTERFACE_DATA) if ctypes.sizeof(ctypes.c_void_p) == 4 else 8)
        
        pdetail = ctypes.cast(detail_data, ctypes.c_void_p)
        success = setupapi.SetupDiGetDeviceInterfaceDetailW(
            hdevinfo, ctypes.byref(dev_interface),
            ctypes.cast(detail_data, ctypes.c_void_p), required_size.value,
            ctypes.byref(required_size), None
        )
        
        if success:
            # Device path is at offset 4 (after cbSize DWORD)
            path_start = 4
            # The path is a wide string
            path_data = detail_data_raw[path_start:]
            # Find null terminator
            null_idx = path_data.find(b'\x00\x00')
            if null_idx >= 0:
                path = path_data[:null_idx].decode('utf-16-le')
                paths.append(path)
        
        idx += 1
    
    setupapi.SetupDiDestroyDeviceInfoList(hdevinfo)
    return paths


def list_all_camera_devices():
    """Alternative: enumerate by GUID_DEVCLASS_IMAGE"""
    GUID_DEVCLASS_IMAGE = GUID(0x6bdd1fc6, 0x810f, 0x11d0, (0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f))
    
    paths = []
    hdevinfo = setupapi.SetupDiGetClassDevsW(
        ctypes.byref(GUID_DEVCLASS_IMAGE), None, None,
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE
    )
    
    if hdevinfo == -1:
        return paths
    
    idx = 0
    dev_interface = SP_DEVICE_INTERFACE_DATA()
    dev_interface.cbSize = ctypes.sizeof(SP_DEVICE_INTERFACE_DATA)
    
    while setupapi.SetupDiEnumDeviceInterfaces(hdevinfo, None, ctypes.byref(GUID_DEVCLASS_IMAGE), idx, ctypes.byref(dev_interface)):
        required_size = wintypes.DWORD()
        setupapi.SetupDiGetDeviceInterfaceDetailW(hdevinfo, ctypes.byref(dev_interface), None, 0, ctypes.byref(required_size), None)
        
        detail_data = ctypes.create_string_buffer(required_size.value)
        cb_size_offset = 0
        if ctypes.sizeof(ctypes.c_void_p) == 4:
            struct.pack_into('I', detail_data, 0, 5)  # SPDRP_DEVICE_INTERFACE_DETAIL_DATA size
        else:
            struct.pack_into('I', detail_data, 0, 8)
        
        if setupapi.SetupDiGetDeviceInterfaceDetailW(hdevinfo, ctypes.byref(dev_interface), ctypes.cast(detail_data, ctypes.c_void_p), required_size.value, ctypes.byref(required_size), None):
            path = ctypes.wstring_at(ctypes.cast(detail_data, ctypes.c_void_p).value + 4)
            paths.append(path)
        
        idx += 1
    
    setupapi.SetupDiDestroyDeviceInfoList(hdevinfo)
    return paths


def find_ylx_device():
    """Find YLX camera (VID_1BCF) device path"""
    all_paths = find_camera_paths()
    if not all_paths:
        all_paths = list_all_camera_devices()
    
    print("=== Camera device paths ===")
    for p in all_paths:
        is_ylx = "1bcf" in p.lower() or "0b15" in p.lower()
        marker = " <-- YLX!" if is_ylx else ""
        print(f"  {p}{marker}")
    
    ylx = [p for p in all_paths if "1bcf" in p.lower() or "0b15" in p.lower()]
    if ylx:
        return ylx[0]
    
    # Fallback: search by VID/PID in device manager
    print("\nSearching via PNP device tree...")
    # Try common paths
    for path in all_paths:
        if "usb" in path.lower() and "video" in path.lower():
            return path
    
    return None


# UVC Extension Unit Control query structure
# This uses a KSPROPERTY-based approach through the UVC driver

class KSP_NODE(ctypes.Structure):
    _fields_ = [
        ("Property", GUID),
        ("NodeId", wintypes.ULONG),
        ("Reserved", wintypes.ULONG),
    ]

# PROPSETID_VIDCAP_EXTENSION_UNIT
EXTENSION_UNIT_GUID = GUID(0x2DC69E01, 0x474B, 0x4B15, (0x8E, 0x4B, 0x78, 0xBC, 0x83, 0xDA, 0x3C, 0x03))

KSPROPERTY_TYPE_GET = 0x01
KSPROPERTY_TYPE_SET = 0x02

def query_xu_control(device_path, node_id, control_id, query_type=KSPROPERTY_TYPE_GET, data=None):
    """
    Send UVCIOC_CTRL_QUERY to an Extension Unit control.
    
    This is complex on Windows because the UVC driver wraps this through
    the KS property system. We'll use a simplified approach.
    """
    # Open device
    handle = kernel32.CreateFileW(
        device_path,
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        None,
        OPEN_EXISTING,
        0,
        None
    )
    
    if handle == INVALID_HANDLE_VALUE:
        err = kernel32.GetLastError()
        print(f"  Cannot open device: error {err}")
        return None
    
    # Build KSPROPERTY request
    # For UVC XU, we use KSPROPERTY with PROPSETID_EXTENSION_UNIT
    # The request format: KSP_NODE + KSPROPERTY description + selector + data
    
    # Actually let's use a simpler IOCTL approach
    
    kernel32.CloseHandle(handle)
    return None


# Let's try the simpler approach: just list what we know and use subprocess
# to call a tool that can do this

if __name__ == "__main__":
    print("Finding YLX camera device...")
    path = find_ylx_device()
    
    if not path:
        print("\nYLX camera not found via SetupAPI. Checking USB devices...")
        import subprocess
        result = subprocess.run(["pnputil", "/enum-devices", "/class", "Image"], 
                              capture_output=True, text=True, timeout=10)
        for line in result.stdout.split("\n"):
            if "1bcf" in line.lower() or "0b15" in line.lower() or "ylx" in line.lower():
                print(f"  {line}")
        
        result = subprocess.run(["pnputil", "/enum-devices", "/class", "Camera"], 
                              capture_output=True, text=True, timeout=10)
        for line in result.stdout.split("\n"):
            if "1bcf" in line.lower() or "0b15" in line.lower() or "ylx" in line.lower():
                print(f"  {line}")
    
    if path:
        print(f"\nFound device path: {path}")
        print("\nTo query UVC XU controls, we need to use the UVC host driver IOCTLs.")
        print("The device path can be used with CreateFile + DeviceIoControl.")
        print(f"\nDevice path for reference: {path}")
