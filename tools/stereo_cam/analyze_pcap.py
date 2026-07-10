#!/usr/bin/env python3
"""
分析 USBPcap 抓包文件，提取 UVC Extension Unit 控制传输命令
用于找出 YLX 驱动如何激活陀螺仪（XU#4，GUID: 63610682-5070-49ab-b8cc-b3855e8d221d）
"""

import struct
import sys
import os
from collections import defaultdict

# USBPcap 文件格式 (来自 Wireshark)
# pcapng 或 pcap 格式，都可以用 scapy 读

def read_pcapng_raw(filepath):
    """直接解析 USBPcap pcapng 文件，提取 USB 控制传输"""
    with open(filepath, 'rb') as f:
        data = f.read()
    
    # 查找所有的 USBPcap 包
    # USBPcap 使用 Enhanced Packet Block (EPB)
    # 在 pcapng 中，USB 包有一个特殊的 header:
    #   - Header length (2 bytes)
    #   - Header type (2 bytes) - 0x00 for USB
    #   - USB header data:
    #     - headerLen (2) - length of this header
    #     - irpId (8) - I/O Request Packet ID  
    #     - status (4) - USBD_STATUS
    #     - function (2) - URB function
    #     - info (1) - transfer flags
    #     - bus (2)
    #     - device (2)
    #     - endpoint (1)
    #     - transfer (1) - URB transfer type
    #       0 = isochronous, 1 = interrupt, 2 = control, 3 = bulk
    #     - dataLength (4)
    
    pos = 0
    packets = []
    
    while pos < len(data):
        # Look for Section Header Block (SHB) magic: 0x0A0D0D0A
        # or Enhanced Packet Block (EPB) magic: 0x00000006
        
        if pos + 8 > len(data):
            break
            
        # Check block type
        block_type = struct.unpack_from('<I', data, pos)[0]
        block_len = struct.unpack_from('<I', data, pos + 4)[0]
        
        if block_len == 0 or block_len > len(data) - pos:
            pos += 4
            continue
            
        if block_type == 6:  # Enhanced Packet Block
            # EPB header: type(4) + length(4) + interface_id(4) + timestamp(8) + cap_len(4) + orig_len(4)
            if block_len < 32:
                pos += block_len
                continue
                
            cap_len = struct.unpack_from('<I', data, pos + 24)[0]
            
            # Packet data starts at offset 32 in EPB
            pkt_offset = pos + 32
            if pkt_offset + 4 > len(data):
                break
                
            # First 2 bytes of packet data: USBPcap header length
            usb_header_len = struct.unpack_from('<H', data, pkt_offset)[0]
            usb_header_type = struct.unpack_from('<H', data, pkt_offset + 2)[0]
            
            if usb_header_type == 0 and usb_header_len >= 27:  # USB packet
                irp_id = struct.unpack_from('<Q', data, pkt_offset + 4)[0]
                status = struct.unpack_from('<I', data, pkt_offset + 12)[0]
                urb_function = struct.unpack_from('<H', data, pkt_offset + 16)[0]
                info = data[pkt_offset + 18]
                bus = struct.unpack_from('<H', data, pkt_offset + 19)[0]
                device = struct.unpack_from('<H', data, pkt_offset + 21)[0]
                endpoint = data[pkt_offset + 23]
                transfer_type = data[pkt_offset + 24]
                data_len = struct.unpack_from('<I', data, pkt_offset + 25)[0]
                
                # Only collect control transfers (URB_FUNCTION_CONTROL_TRANSFER = 0x0008)
                # and URB_FUNCTION_VENDOR/CLASS_INTERFACE
                pkt = {
                    'irp_id': irp_id,
                    'urb_function': urb_function,
                    'status': status,
                    'bus': bus,
                    'device': device,
                    'endpoint': endpoint,
                    'transfer_type': transfer_type,
                    'data_len': data_len,
                    'offset': pkt_offset + usb_header_len,
                }
                
                # Control transfer specific
                if urb_function == 0x0008:  # URB_FUNCTION_CONTROL_TRANSFER
                    ctrl_offset = pkt_offset + usb_header_len
                    if ctrl_offset + 8 <= len(data):
                        pkt['bmRequestType'] = data[ctrl_offset]
                        pkt['bRequest'] = data[ctrl_offset + 1]
                        pkt['wValue'] = struct.unpack_from('<H', data, ctrl_offset + 2)[0]
                        pkt['wIndex'] = struct.unpack_from('<H', data, ctrl_offset + 4)[0]
                        pkt['wLength'] = struct.unpack_from('<H', data, ctrl_offset + 6)[0]
                        pkt['ctrl_data'] = data[ctrl_offset + 8 : ctrl_offset + 8 + pkt['wLength']]
                        
                        # Decode UVC fields
                        req_type = pkt['bmRequestType']
                        req_code = pkt['bRequest']
                        wValue = pkt['wValue']
                        wIndex = pkt['wIndex']
                        
                        # UVC control requests: bmRequestType = 0x21 (host-to-device, class, interface)
                        #                        bmRequestType = 0xA1 (device-to-host, class, interface)
                        pkt['is_uvc_class'] = (req_type & 0x60) == 0x20  # class request
                        pkt['is_uvc_interface'] = (req_type & 0x03) == 0x01
                        pkt['uvc_entity_id'] = (wIndex >> 8) & 0xFF
                        pkt['uvc_interface'] = wIndex & 0xFF
                        pkt['uvc_control_selector'] = wValue >> 8
                        
                        # Standard UVC control selectors
                        if req_code in (0x01, 0x81):
                            pkt['uvc_control_name'] = 'SET_CUR' if req_code == 0x01 else 'GET_CUR'
                        elif req_code in (0x02, 0x82):
                            pkt['uvc_control_name'] = 'GET_MIN' if req_code == 0x82 else 'SET_MIN'
                        elif req_code in (0x03, 0x83):
                            pkt['uvc_control_name'] = 'GET_MAX' if req_code == 0x83 else 'SET_MAX'
                        elif req_code in (0x04, 0x84):
                            pkt['uvc_control_name'] = 'GET_RES' if req_code == 0x84 else 'SET_RES'
                        elif req_code in (0x05, 0x85):
                            pkt['uvc_control_name'] = 'GET_LEN'
                        elif req_code in (0x06, 0x86):
                            pkt['uvc_control_name'] = 'GET_INFO'
                        elif req_code in (0x07, 0x87):
                            pkt['uvc_control_name'] = 'GET_DEF'
                        else:
                            pkt['uvc_control_name'] = f'UNKNOWN(0x{req_code:02X})'
                
                packets.append(pkt)
        
        pos += block_len
    
    return packets


def try_pyshark(filepath):
    """尝试用 pyshark 读取 pcapng"""
    try:
        import pyshark
        cap = pyshark.FileCapture(filepath, display_filter='usb')
        packets = []
        for pkt in cap:
            try:
                usb = pkt.usb
                info = {
                    'src': usb.src,
                    'dst': usb.dst,
                    'transfer_type': getattr(usb, 'transfer_type', '?'),
                }
                if hasattr(usb, 'setup'):
                    info['bmRequestType'] = usb.setup.get_field('bmRequestType')
                    info['bRequest'] = usb.setup.get_field('bRequest')
                    info['wValue'] = usb.setup.get_field('wValue')
                    info['wIndex'] = usb.setup.get_field('wIndex')
                packets.append(info)
            except:
                pass
        cap.close()
        return packets
    except ImportError:
        return None
    except Exception as e:
        print(f"pyshark error: {e}")
        return None


def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_pcap.py <capture.pcapng>")
        print()
        print("从 USBPcap 抓包文件中提取 UVC Extension Unit 控制传输")
        sys.exit(1)
    
    filepath = sys.argv[1]
    if not os.path.exists(filepath):
        print(f"ERROR: File not found: {filepath}")
        sys.exit(1)
    
    print(f"Analyzing: {filepath}")
    print(f"Size: {os.path.getsize(filepath)} bytes")
    print()
    
    # Try raw parsing
    print("=== Parsing USB packets ===")
    packets = read_pcapng_raw(filepath)
    print(f"Total packets: {len(packets)}")
    
    # Filter: only UVC class-specific requests
    uvc_packets = [p for p in packets if p.get('is_uvc_class') and p.get('is_uvc_interface')]
    print(f"UVC class-specific requests: {len(uvc_packets)}")
    print()
    
    if not uvc_packets:
        print("No UVC class requests found. Trying raw control transfer listing...")
        ctrl_packets = [p for p in packets if p.get('bmRequestType') is not None]
        print(f"\nAll control transfers ({len(ctrl_packets)}):")
        for p in ctrl_packets[:50]:
            req_type = p['bmRequestType']
            req_dir = "H2D" if (req_type & 0x80) == 0 else "D2H"
            is_class = (req_type & 0x60) == 0x20
            req_name = p.get('uvc_control_name', '?')
            entity = p.get('uvc_entity_id', '?')
            selector = p.get('uvc_control_selector', 0)
            print(f"  [{req_dir}] bmReqType=0x{req_type:02X} bReq=0x{p['bRequest']:02X} "
                  f"wVal=0x{p['wValue']:04X} wIdx=0x{p['wIndex']:04X} "
                  f"wLen={p['wLength']} "
                  f"ent={entity} sel=0x{selector:02X} '{req_name}' "
                  f"class={is_class}")
            
            ctrl_data = p.get('ctrl_data', b'')
            if ctrl_data:
                hex_str = ' '.join(f'{b:02X}' for b in ctrl_data[:64])
                print(f"    Data[{len(ctrl_data)}]: {hex_str}")
        return
    
    # Group by entity ID
    by_entity = defaultdict(list)
    for p in uvc_packets:
        by_entity[p['uvc_entity_id']].append(p)
    
    print("=== UVC Extension Unit Commands ===")
    for entity_id in sorted(by_entity.keys()):
        pkts = by_entity[entity_id]
        
        # Deduplicate (same request often appears multiple times due to USBPcap capturing both submit and complete)
        unique = []
        seen = set()
        for p in pkts:
            key = (p['bmRequestType'], p['bRequest'], p['wValue'], p['wIndex'])
            ctrl_data = p.get('ctrl_data', b'')
            data_hash = hash(ctrl_data)
            full_key = (key, data_hash)
            if key not in seen:
                seen.add(key)
                unique.append(p)
        
        print(f"\n--- Entity #{entity_id} ({len(pkts)} raw, {len(unique)} unique) ---")
        for p in unique:
            req_type = p['bmRequestType']
            req_dir = "HOST→DEV" if (req_type & 0x80) == 0 else "DEV→HOST"
            req_name = p.get('uvc_control_name', '?')
            selector = p.get('uvc_control_selector', 0)
            
            print(f"  {req_dir} {req_name} sel=0x{selector:02X} "
                  f"bmReqType=0x{req_type:02X} bReq=0x{p['bRequest']:02X} "
                  f"wVal=0x{p['wValue']:04X} wIdx=0x{p['wIndex']:04X} "
                  f"wLen={p['wLength']}")
            
            ctrl_data = p.get('ctrl_data', b'')
            if ctrl_data:
                hex_str = ' '.join(f'{b:02X}' for b in ctrl_data[:128])
                print(f"    Payload[{len(ctrl_data)}]: {hex_str}")
    
    # Highlight entity #4 (gyroscope XU)
    x4_pkts = by_entity.get(4, [])
    if x4_pkts:
        print("\n" + "=" * 60)
        print("*** XU#4 (GYROSCOPE - 63610682-5070-49ab-b8cc-b3855e8d221d) ***")
        print("=" * 60)
        for p in x4_pkts:
            req_type = p['bmRequestType']
            req_dir = ">>> ACTIVATION CMD >>>" if (req_type & 0x80) == 0 else "<<< QUERY <<<"
            req_name = p.get('uvc_control_name', '?')
            selector = p.get('uvc_control_selector', 0)
            
            print(f"\n  {req_dir}")
            print(f"  {req_name} sel=0x{selector:02X} "
                  f"bmReqType=0x{req_type:02X} bReq=0x{p['bRequest']:02X} "
                  f"wVal=0x{p['wValue']:04X} wIdx=0x{p['wIndex']:04X} "
                  f"wLen={p['wLength']}")
            
            ctrl_data = p.get('ctrl_data', b'')
            if ctrl_data:
                hex_str = ' '.join(f'{b:02X}' for b in ctrl_data[:256])
                print(f"  Payload[{len(ctrl_data)}]: {hex_str}")
                
                # Also show as C array for use in code
                if len(ctrl_data) <= 64:
                    c_array = '{' + ', '.join(f'0x{b:02X}' for b in ctrl_data) + '}'
                    print(f"  C array: {c_array}")


if __name__ == '__main__':
    main()
