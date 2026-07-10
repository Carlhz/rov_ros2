"""
Parse USBPcap classic pcap format (linktype 249 = LINKTYPE_USB_2_0).
Extract UVC Extension Unit control transfers for YLX gyroscope activation.
"""
import struct
import sys
import os
from collections import defaultdict

# URB functions
URB_FUNCTION_CONTROL_TRANSFER = 0x0008
URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER = 0x0009
URB_FUNCTION_ISOCH_TRANSFER = 0x001A

# Transfer types
USB_TRANSFER_ISOCH = 0
USB_TRANSFER_INTERRUPT = 1
USB_TRANSFER_CONTROL = 2
USB_TRANSFER_BULK = 3

TRANSFER_NAMES = {0: 'ISOCH', 1: 'INT', 2: 'CONTROL', 3: 'BULK'}
URB_NAMES = {0x0008: 'CONTROL', 0x0009: 'BULK/INT', 0x001A: 'ISOCH'}


def parse_pcap(filepath):
    """Parse classic pcap with USBPcap headers."""
    with open(filepath, 'rb') as f:
        data = f.read()
    
    if len(data) < 24:
        return []
    
    # Global header (24 bytes)
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != 0xA1B2C3D4:
        print(f"ERROR: Not a valid pcap file (magic=0x{magic:08X})")
        return []
    
    ver_major = struct.unpack_from('<H', data, 4)[0]
    ver_minor = struct.unpack_from('<H', data, 6)[0]
    snaplen = struct.unpack_from('<I', data, 16)[0]
    linktype = struct.unpack_from('<I', data, 20)[0]
    
    print(f"pcap v{ver_major}.{ver_minor}, snaplen={snaplen}, linktype={linktype}")
    
    # USBPcap header struct (27 bytes):
    #   uint16 headerLen;     // includes this header + possible USBD header
    #   uint64 irpId;         // 8 bytes
    #   uint32 status;        // 4 bytes (USBD_STATUS)
    #   uint16 function;      // 2 bytes (URB_FUNCTION)
    #   uint8  info;          // 1 byte (transfer flags)
    #   uint16 bus;           // 2 bytes
    #   uint16 device;        // 2 bytes
    #   uint8  endpoint;      // 1 byte
    #   uint8  transfer;      // 1 byte (0=ISOCH,1=INT,2=CONTROL,3=BULK)
    #   uint32 dataLength;    // 4 bytes
    USBPcap_HDR_LEN = 27
    
    packets = []
    pos = 24  # skip global header
    
    while pos + 16 <= len(data):
        # Packet record header (16 bytes)
        ts_sec = struct.unpack_from('<I', data, pos)[0]
        ts_usec = struct.unpack_from('<I', data, pos + 4)[0]
        incl_len = struct.unpack_from('<I', data, pos + 8)[0]
        orig_len = struct.unpack_from('<I', data, pos + 12)[0]
        
        pkt_start = pos + 16
        pkt_end = pkt_start + incl_len
        
        if pkt_end > len(data) or incl_len < USBPcap_HDR_LEN:
            break
        
        pkt_data = data[pkt_start:pkt_end]
        
        # Parse USBPcap header
        header_len = struct.unpack_from('<H', pkt_data, 0)[0]
        irp_id = struct.unpack_from('<Q', pkt_data, 2)[0]
        status = struct.unpack_from('<I', pkt_data, 10)[0]
        urb_function = struct.unpack_from('<H', pkt_data, 14)[0]
        info = pkt_data[16]
        bus = struct.unpack_from('<H', pkt_data, 17)[0]
        device = struct.unpack_from('<H', pkt_data, 19)[0]
        endpoint = pkt_data[21]
        transfer = pkt_data[22]
        data_len = struct.unpack_from('<I', pkt_data, 23)[0]
        
        pkt = {
            'ts': ts_sec + ts_usec / 1000000.0,
            'irp_id': irp_id,
            'status': status,
            'urb_function': urb_function,
            'info': info,
            'bus': bus,
            'device': device,
            'endpoint': endpoint,
            'transfer': transfer,
            'data_len': data_len,
        }
        
        # Payload (data after USBPcap header)
        payload_offset = header_len  # headerLen includes control setup if present
        if payload_offset < incl_len:
            payload = pkt_data[payload_offset : incl_len]
            pkt['payload'] = payload
        else:
            payload = b''
        
        # For CONTROL transfers, extract setup packet from USBD_HEADER
        # When transfer == CONTROL:
        #   headerLen includes 8 bytes USBD setup before the actual data
        # USBD header is at the end of the USBPcap header (27 bytes),
        # then data follows
        if transfer == 2 and urb_function == URB_FUNCTION_CONTROL_TRANSFER:
            # USBD_PIPE_HANDLE/setup start at byte 27, before payload
            # Actually, the setup packet is before the data payload.
            # headerLen = 27 + 8 (setup) + payload_data_len
            # So setup is at offset 27 within pkt_data, and data is at offset 35
            if header_len >= 35:  # at least USBPcap header + setup packet
                setup_start = USBPcap_HDR_LEN
                if setup_start + 8 <= incl_len:
                    setup = pkt_data[setup_start:setup_start + 8]
                    pkt['bmRequestType'] = setup[0]
                    pkt['bRequest'] = setup[1]
                    pkt['wValue'] = struct.unpack_from('<H', setup, 2)[0]
                    pkt['wIndex'] = struct.unpack_from('<H', setup, 4)[0]
                    pkt['wLength'] = struct.unpack_from('<H', setup, 6)[0]
                    
                    # Actual control data is at offset 35 (27 + 8)
                    if header_len > 35:
                        ctrl_data_start = 35
                        ctrl_data = pkt_data[ctrl_data_start : incl_len]
                        pkt['ctrl_data'] = ctrl_data
                    else:
                        pkt['ctrl_data'] = b''
        
        packets.append(pkt)
        
        pos += 16 + incl_len
    
    return packets


def analyze(filepath):
    print(f"\nAnalyzing: {filepath}")
    print(f"Size: {os.path.getsize(filepath)} bytes\n")
    
    packets = parse_pcap(filepath)
    print(f"Total USB packets: {len(packets)}")
    
    # Show summary
    by_transfer = defaultdict(int)
    by_function = defaultdict(int)
    by_device = defaultdict(int)
    by_endpoint = defaultdict(int)
    
    for p in packets:
        by_transfer[p['transfer']] += 1
        by_function[p['urb_function']] += 1
        by_device[p['device']] += 1
        by_endpoint[f"{p['device']}:0x{p['endpoint']:02X}"] += 1
    
    print("\n--- Packets by transfer type ---")
    for ttype in sorted(by_transfer):
        print(f"  {TRANSFER_NAMES.get(ttype, ttype)}: {by_transfer[ttype]}")
    
    print("\n--- Packets by URB function ---")
    for func in sorted(by_function):
        name = URB_NAMES.get(func, f"0x{func:04X}")
        print(f"  {name}: {by_function[func]}")
    
    print("\n--- Packets by device ---")
    for dev in sorted(by_device):
        print(f"  Device {dev}: {by_device[dev]}")
    
    print("\n--- Packets by endpoint ---")
    for ep in sorted(by_endpoint):
        print(f"  {ep}: {by_endpoint[ep]}")
    
    # Filter: only CONTROL transfers
    control_pkts = [p for p in packets if p['transfer'] == 2]
    print(f"\n{'='*60}")
    print(f"CONTROL TRANSFERS: {len(control_pkts)}")
    print(f"{'='*60}")
    
    if not control_pkts:
        print("\n*** NO CONTROL TRANSFERS FOUND! ***")
        print("Possible reasons:")
        print("1. Camera software was not opened during capture")
        print("2. XU commands are sent at device enumeration, not during streaming")
        print("3. Need to re-plug USB camera before starting capture")
        print("\nShowing all packet types to understand what was happening:")
        
        for p in packets[:30]:
            ttype = TRANSFER_NAMES.get(p['transfer'], p['transfer'])
            func = URB_NAMES.get(p['urb_function'], f"0x{p['urb_function']:04X}")
            print(f"  [{ttype:7s}] dev={p['device']} ep=0x{p['endpoint']:02X} "
                  f"URB={func} dataLen={p['data_len']} status={p['status']}")
        
        return
    
    # Show control transfers
    for i, p in enumerate(control_pkts):
        direction = "H2D" if (p.get('bmRequestType', 0) & 0x80) == 0 else "D2H"
        bmRT = p.get('bmRequestType', 0)
        bReq = p.get('bRequest', 0)
        wVal = p.get('wValue', 0)
        wIdx = p.get('wIndex', 0)
        wLen = p.get('wLength', 0)
        
        entity_id = (wIdx >> 8) & 0xFF
        interface = wIdx & 0xFF
        selector = wVal >> 8
        
        is_class = (bmRT & 0x60) == 0x20
        is_interface = (bmRT & 0x03) == 0x01
        
        print(f"\n  [{i}] {direction} bmRT=0x{bmRT:02X} bReq=0x{bReq:02X} "
              f"wVal=0x{wVal:04X} wIdx=0x{wIdx:04X} wLen={wLen}")
        print(f"       EntityID={entity_id} Interface={interface} Selector=0x{selector:02X}")
        print(f"       Class={is_class} ToInterface={is_interface} "
              f"Dev={p['device']} EP=0x{p['endpoint']:02X}")
        
        cdata = p.get('ctrl_data', b'')
        if cdata:
            hex_str = ' '.join(f'{b:02X}' for b in cdata[:64])
            print(f"       Data[{len(cdata)}]: {hex_str}")
    
    # Focus on Entity #4 (gyroscope XU)
    x4_pkts = [p for p in control_pkts if ((p.get('wIndex', 0) >> 8) & 0xFF) == 4]
    if x4_pkts:
        print(f"\n{'='*60}")
        print(f"*** XU#4 GYROSCOPE COMMANDS ({len(x4_pkts)}) ***")
        print(f"{'='*60}")
        for p in x4_pkts:
            direction = "HOST->DEV" if (p.get('bmRequestType', 0) & 0x80) == 0 else "DEV->HOST"
            bReq = p.get('bRequest', 0)
            wVal = p.get('wValue', 0)
            selector = wVal >> 8
            cdata = p.get('ctrl_data', b'')
            
            req_names = {0x01: 'SET_CUR', 0x81: 'GET_CUR', 0x02: 'GET_MIN', 
                        0x03: 'GET_MAX', 0x04: 'GET_RES', 0x05: 'GET_LEN',
                        0x06: 'GET_INFO', 0x07: 'GET_DEF'}
            
            print(f"\n  {direction} {req_names.get(bReq, f'bReq=0x{bReq:02X}')}")
            print(f"  Selector=0x{selector:02X} bmRT=0x{p.get('bmRequestType',0):02X} "
                  f"wVal=0x{wVal:04X} wIdx=0x{p.get('wIndex',0):04X}")
            if cdata:
                hex_str = ' '.join(f'{b:02X}' for b in cdata[:64])
                c_array = '{' + ', '.join(f'0x{b:02X}' for b in cdata) + '}'
                print(f"  Payload[{len(cdata)}]: {hex_str}")
                print(f"  C/C++: {c_array}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        fp = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng'
    else:
        fp = sys.argv[1]
    
    if not os.path.exists(fp):
        print(f"ERROR: {fp} not found")
        sys.exit(1)
    
    analyze(fp)
