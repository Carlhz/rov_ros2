#!/bin/bash
# ============================================================
#  一键部署: 在 VM 终端运行此脚本即可创建所有工具文件
#  用法: 把下面整段复制粘贴到 VM 终端后回车
# ============================================================

mkdir -p ~/stereo_cam_tools
cd ~/stereo_cam_tools

# ---- ylx_detect.sh ----
cat > ylx_detect.sh << 'DETECT_EOF'
#!/bin/bash
echo "====== YLX Camera Detection ======"
lsusb
echo ""
echo "[Video devices]"
ls -la /dev/video* 2>/dev/null || echo "  None"
echo ""
which v4l2-ctl > /dev/null 2>&1 || sudo apt-get install -y v4l-utils
echo "[v4l2-ctl list-devices]"
v4l2-ctl --list-devices 2>/dev/null
echo ""
for VDEV in $(ls /dev/video* 2>/dev/null); do
  echo "[${VDEV}]"
  v4l2-ctl -d ${VDEV} --list-formats-ext 2>/dev/null | head -15
  echo ""
done
echo "[USB descriptor Extension Unit]"
sudo lsusb -v -d 1bcf:0b15 2>/dev/null | grep -E "bDescriptor|bUnit|GUID|Extension|Interface|imu|gyro" | head -50 || echo "  (需要 sudo 或设备未连接)"
echo "====== Done ======"
DETECT_EOF

# ---- ylx_cam_view.py ----
cat > ylx_cam_view.py << 'CAM_EOF'
#!/usr/bin/env python3
"""YLX 双目摄像头查看器 - 按 Q 退出, S 保存"""
import cv2, numpy as np, time, os, subprocess, glob

def probe(idx):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(idx)
    if not cap.isOpened(): return None
    ret, fr = cap.read()
    if ret:
        print(f"  video{idx}: {fr.shape[1]}x{fr.shape[0]}  OK")
        return cap
    cap.release()
    return None

print("=== YLX 双目摄像头查看器 ===")
try:
    out = subprocess.check_output(["v4l2-ctl","--list-devices"],stderr=subprocess.DEVNULL,timeout=5).decode()
    print(out)
except: pass

caps = [(i, probe(i)) for i in range(8) if probe(i)]
# 重新扫描避免重复打开
caps = []
for i in range(8):
    c = probe(i)
    if c: caps.append((i, c))

if not caps:
    print("无可用摄像头！请检查 VMware USB 连接")
    print("Player > Removable Devices > YLX Camera > Connect")
    exit(1)

print(f"\n可用摄像头 index: {[c[0] for c in caps]}")
for _, c in caps:
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    c.set(cv2.CAP_PROP_FPS, 30)

cap_l = caps[0][1]
cap_r = caps[1][1] if len(caps) >= 2 else None
save_dir = os.path.expanduser("~/ylx_frames"); os.makedirs(save_dir, exist_ok=True)
save_n = 0; t0 = time.time(); fn = 0

print("\n[Q] 退出  [S] 保存截图")
while True:
    ret, fl = cap_l.read()
    if not ret: break
    fn += 1
    fps = fn / max(time.time()-t0, 0.001)
    if cap_r:
        _, fr = cap_r.read()
        mode = "Dual"
    else:
        h, w = fl.shape[:2]
        if w > h * 1.5:
            m = w//2; fr = fl[:,m:].copy(); fl = fl[:,:m].copy(); mode = "Split"
        else:
            fr = np.zeros_like(fl); mode = "Single"
    dw, dh = 640, 360
    dl = cv2.resize(fl,(dw,dh)); dr = cv2.resize(fr,(dw,dh))
    info = f"{mode} FPS:{fps:.0f} {fl.shape[1]}x{fl.shape[0]}"
    for img,lbl in [(dl,"LEFT"),(dr,"RIGHT")]:
        cv2.putText(img,lbl,(8,30),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,80),2)
        cv2.putText(img,info,(8,dh-8),cv2.FONT_HERSHEY_SIMPLEX,0.4,(180,180,180),1)
    cv2.imshow("YLX Stereo  Q=quit S=save", np.hstack([dl,dr]))
    k = cv2.waitKey(1)&0xFF
    if k in (ord('q'),27): break
    elif k == ord('s'):
        ts = int(time.time()*1000)
        cv2.imwrite(f"{save_dir}/L_{save_n:04d}.jpg", fl)
        cv2.imwrite(f"{save_dir}/R_{save_n:04d}.jpg", fr)
        save_n += 1; print(f"  保存 #{save_n}")

for _, c in caps: c.release()
cv2.destroyAllWindows()
print(f"完成，共保存 {save_n} 对到 {save_dir}")
CAM_EOF

# ---- ylx_imu_probe.py ----
cat > ylx_imu_probe.py << 'IMU_EOF'
#!/usr/bin/env python3
"""YLX 陀螺仪 UVC Extension Unit 探测器 - 需要 sudo"""
import os, sys, struct, ctypes, fcntl, glob, subprocess

UVCIOC_CTRL_QUERY = 0xC0186F21
UVC_GET_CUR, UVC_GET_LEN = 0x81, 0x85

class XU(ctypes.Structure):
    _fields_ = [('unit',ctypes.c_uint8),('selector',ctypes.c_uint8),
                ('query',ctypes.c_uint16),('size',ctypes.c_uint16),
                ('reserved',ctypes.c_uint16),('data',ctypes.c_void_p)]

def xu_query(fd, unit, sel, qtype, length=64):
    buf = (ctypes.c_uint8 * length)()
    xu = XU(unit=unit,selector=sel,query=qtype,size=length,reserved=0,
            data=ctypes.cast(buf,ctypes.c_void_p))
    try: fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, xu); return bytes(buf)
    except: return None

print("=== YLX 陀螺仪通道探测 ===")
if os.geteuid() != 0:
    print("⚠  建议 sudo 运行\n")

# USB 描述符
print("[1] USB Extension Unit:")
try:
    out = subprocess.check_output(["lsusb","-v","-d","1bcf:0b15"],
                                   stderr=subprocess.STDOUT,timeout=10).decode()
    in_xu=False; lines=out.splitlines()
    for i,l in enumerate(lines):
        if '0x06' in l and 'bDescriptorSubtype' in l:
            in_xu=True
        if in_xu:
            print(" ",l)
            if i > 5 and 'bDescriptorSubtype' in l and '0x06' not in l:
                in_xu=False
    if 'Extension' not in out and '0x06' not in out:
        print("  未找到 Extension Unit，打印接口列表:")
        for l in lines:
            if any(k in l for k in ('bInterfaceClass','bInterfaceSubClass','Interface Number')):
                print(" ",l)
except Exception as e:
    print(f"  lsusb 失败: {e}")

# V4L2 XU 扫描
print("\n[2] V4L2 XU 暴力扫描:")
for dev in sorted(glob.glob("/dev/video*"))[:4]:
    try:
        fd = os.open(dev, os.O_RDWR|os.O_NONBLOCK)
    except Exception as e:
        print(f"  {dev}: {e}"); continue
    found = False
    for unit in range(1,10):
        for sel in range(1,12):
            r = xu_query(fd, unit, sel, UVC_GET_LEN, 2)
            if r:
                try: ln = struct.unpack("<H",r[:2])[0]
                except: continue
                if 0 < ln < 512:
                    val = xu_query(fd, unit, sel, UVC_GET_CUR, ln)
                    if val:
                        hx = ' '.join(f'{b:02X}' for b in val[:min(ln,32)])
                        print(f"  {dev} Unit={unit} Sel={sel} Len={ln}  {hx}")
                        if ln >= 12:
                            try:
                                v = struct.unpack_from('<6h',val,0)
                                print(f"    int16x6: {v} (可能 gx,gy,gz,ax,ay,az)")
                            except: pass
                        found = True
    os.close(fd)
    if not found:
        print(f"  {dev}: 无 XU 响应")

# Metadata 设备
print("\n[3] Metadata 设备:")
for dev in sorted(glob.glob("/dev/video*")):
    try:
        out = subprocess.check_output(["v4l2-ctl","-d",dev,"--info"],
                                       stderr=subprocess.DEVNULL,timeout=3).decode()
        for l in out.splitlines():
            if any(k in l.lower() for k in ('meta','imu','gyro','card')):
                print(f"  {dev}: {l.strip()}")
    except: pass

print("\n=== 探测完成！请截图给上位机 ===")
IMU_EOF

chmod +x ylx_detect.sh
echo ""
echo "======================================"
echo "  工具已创建到 ~/stereo_cam_tools/"
echo "  运行以下命令开始检测："
echo ""
echo "  1. bash ylx_detect.sh           (检测设备)"
echo "  2. python3 ylx_cam_view.py       (查看摄像头)"
echo "  3. sudo python3 ylx_imu_probe.py (探测陀螺仪)"
echo "======================================"
