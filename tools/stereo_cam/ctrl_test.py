#!/usr/bin/env python3
"""
Mini test: send a known UVC control transfer and verify capture.
Uses PyUSB to send GET_INFO to XU#4, captures with USBPcap.
"""
import subprocess, os, time, sys, ctypes

def install_pyusb():
    """Install PyUSB if not available."""
    try:
        import usb
        return True
    except ImportError:
        print("Installing PyUSB...")
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'pyusb', '--quiet'], timeout=30)
        try:
            import usb
            return True
        except:
            return False

def capture_and_query():
    USBPcap_exe = r"C:\Program Files\USBPcap\USBPcapCMD.exe"
    OUTPUT = r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_ctrl_test.pcapng"

    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)

    print("=" * 60)
    print("Control Transfer Test: Capture + PyUSB GET_INFO")
    print("=" * 60)

    # Start capture
    print("\nStarting USBPcap...")
    cmd = [USBPcap_exe, "-d", r"\\.\USBPcap1", "-o", OUTPUT, "-A", "-s", "256"]
    proc = subprocess.Popen(cmd, cwd=r"C:\Program Files\USBPcap",
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2)
    
    if proc.poll() is not None:
        print(f"ERROR: USBPcapCMD failed (code={proc.returncode})")
        return

    # Try PyUSB control transfers
    try:
        import usb.core
        import usb.util

        print("PyUSB loaded. Searching for YLX camera (1BCF:0B15)...")
        
        devs = list(usb.core.find(find_all=True, idVendor=0x1BCF, idProduct=0x0B15))
        if not devs:
            print("YLX device NOT found via PyUSB!")
            print("Trying without ID filter...")
            devs = list(usb.core.find(find_all=True))
            for d in devs:
                try:
                    print(f"  USB: VID={d.idVendor:04X} PID={d.idProduct:04X}")
                except:
                    pass
        
        for dev in devs:
            try:
                print(f"\nFound: VID={dev.idVendor:04X} PID={dev.idProduct:04X}")
                
                # Try to detach kernel driver
                for cfg in dev:
                    for intf in cfg:
                        try:
                            if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                                dev.detach_kernel_driver(intf.bInterfaceNumber)
                                print(f"  Detached kernel driver from interface {intf.bInterfaceNumber}")
                        except Exception as e:
                            pass
                
                try:
                    dev.set_configuration()
                    print("  Configuration set")
                except Exception as e:
                    print(f"  Config error: {e}")
                
                # Test: send GET_INFO to XU#4 with various selectors
                print("\n  Testing XU#4 GET_INFO...")
                bmRT = 0xA1  # device-to-host, class, interface
                bReq = 0x86  # GET_INFO
                
                for selector in range(26):  # 25 controls
                    wVal = (selector << 8)
                    wIdx = (4 << 8) | 0  # entity 4, interface 0
                    
                    try:
                        data = dev.ctrl_transfer(bmRT, bReq, wVal, wIdx, 1, timeout=200)
                        if data and data[0] != 0:
                            print(f"    XU#4 sel={selector:2d}: GET_INFO = 0x{data[0]:02X} (SUPPORTED)")
                    except Exception as e:
                        pass  # expected to fail for many
                
                # Also try GET_CUR on supported selectors
                print("\n  Testing XU#4 GET_CUR...")
                bReq = 0x81  # GET_CUR
                
                for selector in range(26):
                    wVal = (selector << 8)
                    wIdx = (4 << 8) | 0
                    
                    try:
                        data = dev.ctrl_transfer(bmRT, bReq, wVal, wIdx, 8, timeout=200)
                        if data and len(data) > 0:
                            hex_str = ' '.join(f'{b:02X}' for b in data)
                            print(f"    XU#4 sel={selector:2d}: GET_CUR[{len(data)}] = {hex_str}")
                    except Exception as e:
                        str_e = str(e)[:80]
                        if 'pipe error' not in str_e.lower() and 'access' not in str_e.lower():
                            pass
                
                # Test standard UVC controls (brightness, etc.) to verify control transfer works
                print("\n  Testing standard UVC controls...")
                # PU_BRIGHTNESS_CONTROL on Processing Unit
                for entity in range(1, 12):
                    try:
                        data = dev.ctrl_transfer(0xA1, 0x86, 0x0200, (entity << 8) | 0, 1, timeout=200)
                        if data and data[0] != 0:
                            print(f"    Entity#{entity}: GET_INFO=0x{data[0]:02X}")
                    except:
                        pass

            except Exception as e:
                print(f"  Device access error: {e}")

    except ImportError:
        print("PyUSB import failed")
    except Exception as e:
        print(f"Error: {e}")

    # Wait a bit more for capture
    time.sleep(3)
    
    # Stop capture
    print("\nStopping capture...")
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except:
            proc.kill()
    
    if os.path.exists(OUTPUT):
        size = os.path.getsize(OUTPUT) / 1024
        print(f"Capture file: {OUTPUT} ({size:.1f} KB)")
    else:
        print("No capture file created")

if __name__ == '__main__':
    if not install_pyusb():
        print("Failed to install PyUSB")
    capture_and_query()
