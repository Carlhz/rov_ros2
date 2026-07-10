"""Show all control transfers in the capture."""
import struct

PCAP = r'D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_fixed.pcap'

with open(PCAP, 'rb') as f:
    data = f.read()

offset = 24
ctrls = []

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
    
    header_len = struct.unpack_from('<H', pkt, 0)[0]
    urb_func = struct.unpack_from('<H', pkt, 14)[0]
    dev = struct.unpack_from('<H', pkt, 19)[0]
    ep = pkt[21]
    transfer = pkt[22]
    data_len = struct.unpack_from('<I', pkt, 23)[0]
    
    if transfer == 2 and header_len >= 35:  # CONTROL (any URB function)
        setup = pkt[27:35]
        bmReq = setup[0]
        bReq = setup[1]
        wValue = struct.unpack('<H', setup[2:4])[0]
        wIndex = struct.unpack('<H', setup[4:6])[0]
        wLength = struct.unpack('<H', setup[6:8])[0]
        req_type = (bmReq >> 5) & 0x03
        direction = 'IN' if bmReq & 0x80 else 'OUT'
        
        ctrl_data = b''
        if header_len > 35:
            ctrl_data = pkt[35:incl_len]
        
        ctrls.append({
            'num': len(ctrls) + 1, 'dev': dev, 'ep': ep,
            'dir': direction, 'type': req_type,
            'bReq': bReq, 'wValue': wValue, 'wIndex': wIndex,
            'wLength': wLength, 'data': ctrl_data
        })

print(f'Total control transfers: {len(ctrls)}')

# UVC control names
uvc_req = {0x01: 'SET_CUR', 0x81: 'GET_CUR', 0x82: 'GET_MIN',
           0x83: 'GET_MAX', 0x84: 'GET_RES', 0x85: 'GET_LEN', 0x86: 'GET_INFO', 0x87: 'GET_DEF'}
type_names = {0: 'STD', 1: 'CLS', 2: 'VEN'}

print(f'\n{"#":>3s} {"Dev":>3s} {"Dir":>3s} {"Typ":>3s} {"bReq":>8s} {"wValue":>8s} {"wIndex":>8s} {"wLen":>5s} data')
print('-' * 90)

for c in ctrls:
    t = type_names.get(c['type'], str(c['type']))
    name = uvc_req.get(c['bReq'], f'0x{c["bReq"]:02X}')
    data_hex = c['data'][:40].hex(' ') if c['data'] else '(none)'
    
    # Decode wIndex for UVC interface
    if c['type'] == 1:  # Class
        entity = (c['wIndex'] >> 8) & 0xFF
        iface = c['wIndex'] & 0xFF
        sel = (c['wValue'] >> 0) & 0xFF
        wIndex_desc = f'entity={entity} if={iface}'
        wValue_desc = f'sel={sel}'
    else:
        wIndex_desc = ''
        wValue_desc = ''
    
    print(f'{c["num"]:3d} {c["dev"]:3d} {c["dir"]:>3s} {t:>3s} {name:>8s} 0x{c["wValue"]:04X} 0x{c["wIndex"]:04X} {c["wLength"]:5d} {data_hex}')
    if wIndex_desc:
        print(f'     {"":>38s} {wValue_desc} {wIndex_desc}')
