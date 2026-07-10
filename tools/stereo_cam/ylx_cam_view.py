#!/usr/bin/env python3
"""
YLX 双目摄像头验证 — 步骤2: 查看摄像头画面
=============================================
在 VM 终端运行:
  python3 ylx_cam_view.py

按 Q 退出, S 保存截图

如果出现 select() timeout:
  1. VMware 设 USB 3.0: Player > Removable Devices > YLX Camera 确认已 Connect
  2. 加载 UVC 内核 quirk: sudo modprobe -r uvcvideo && sudo modprobe uvcvideo quirks=2
"""

import cv2
import numpy as np
import time
import os
import subprocess
import glob
import sys

def run_cmd(cmd):
    """安全执行 shell 命令"""
    try:
        return subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, timeout=5).decode()
    except Exception as e:
        return str(e)

def list_video_devices():
    """列出所有 V4L2 设备"""
    print("\n--- v4l2-ctl --list-devices ---")
    print(run_cmd("v4l2-ctl --list-devices 2>/dev/null"))
    devs = sorted(glob.glob("/dev/video*"))
    print(f"/dev/video*: {devs}")
    return devs


def probe_device(path):
    """用不同方式尝试打开设备，返回 (cap, 宽, 高) 或 (None, None, None)"""
    idx = int(os.path.basename(path).replace("video", ""))

    # 先看 V4L2 格式信息
    fmt_out = run_cmd(f"v4l2-ctl -d {path} --list-formats-ext 2>/dev/null")
    if "MJPG" in fmt_out or "YUYV" in fmt_out:
        print(f"  {path}: 支持格式: ", end="")
        for line in fmt_out.split("\n"):
            line = line.strip()
            if "MJPG" in line or "YUYV" in line:
                print(line, end="  ")
        print()

    # 策略: 尝试多种打开方式
    strategies = [
        # (描述, 创建代码)
        ("CAP_V4L2 + explicit path", lambda p: cv2.VideoCapture(p, cv2.CAP_V4L2)),
        ("CAP_ANY + explicit path",  lambda p: cv2.VideoCapture(p)),
        ("CAP_DSHOW + explicit path (Windows fallback, skip)", None),  # Linux 不可用，跳过
    ]

    cap = None
    used_strategy = ""

    for label, factory in strategies:
        if factory is None:
            continue
        cap = factory(path)
        if cap is not None and cap.isOpened():
            used_strategy = label
            break
        if cap is not None:
            cap.release()

    # 最后试 index 方式
    if cap is None:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            used_strategy = "CAP_V4L2 + index %d" % idx
        else:
            cap.release()
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                used_strategy = "CAP_ANY + index %d" % idx

    if cap is None or not cap.isOpened():
        print(f"  {path}: ✗ 无法打开 (所有策略均失败)")
        if cap:
            cap.release()
        return None, None, None

    # 设置 MJPG 优先 (UVC 常见)
    fmt_4cc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fmt_4cc)

    # 尝试读一帧
    ok, frame = cap.read()
    if ok and frame is not None:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"  {path}: {w}x{h}  ✓ ({used_strategy})")
        return cap, w, h
    else:
        cap.release()
        print(f"  {path}: ✗ 打开成功但无法读取帧 ({used_strategy})")
        return None, None, None


def main():
    print("=" * 55)
    print("  YLX 双目摄像头图像查看器  v2")
    print("=" * 55)

    devs = list_video_devices()

    if not devs:
        print("\n❌ 未找到任何 /dev/video* 设备!")
        print("   请确认: VMware > Removable Devices > YLX Camera > Connect")
        return

    # 探测所有设备
    print("\n--- 探测摄像头 ---")
    caps = []
    for dev in devs[:8]:
        cap, w, h = probe_device(dev)
        if cap is not None:
            caps.append((dev, cap, w, h))

    if not caps:
        print("\n" + "=" * 55)
        print("  ❌ 没有找到可用的摄像头")
        print("=" * 55)
        print("""
可能原因和解决方案:

1. VMware USB 控制器类型不匹配:
   → VMware 虚拟机设置 → USB Controller → 选 USB 3.0
   （默认可能是 USB 2.0，但 YLX 是 USB 3.0 设备）

2. UVC 内核驱动 quirk:
   → sudo modprobe -r uvcvideo
   → sudo modprobe uvcvideo quirks=2

3. 摄像头被宿主机占用:
   → 先在 Windows 设备管理器里禁用该设备
   → 再在 VMware 里 Connect

4. 检查 lsusb -t 看是否挂在 USB 3.0 总线:
   → lsusb -t | grep -A2 1bcf

请先尝试以上方案，然后重新运行本脚本。
""")
        return

    print(f"\n可用摄像头: {[c[0] for c in caps]}")

    # 判断模式
    if len(caps) >= 2:
        print(f"\n模式: 双设备 (L={caps[0][0]}, R={caps[1][0]})")
        dev_l, cap_l, w_l, h_l = caps[0]
        dev_r, cap_r, w_r, h_r = caps[1]
    else:
        print(f"\n模式: 单设备 ({caps[0][0]})")
        dev_l, cap_l, w_l, h_l = caps[0]
        cap_r = None
        w_r = h_r = 0

    save_dir = os.path.expanduser("~/ylx_frames")
    os.makedirs(save_dir, exist_ok=True)
    save_count = 0
    t0 = time.time()
    frame_n = 0

    print(f"\n运行中... [Q] 退出  [S] 保存截图")
    print(f"截图保存到: {save_dir}\n")

    while True:
        ret_l, frame_l = cap_l.read()
        if not ret_l:
            print(f"左摄像头({dev_l})读取失败! 尝试重新打开...")
            cap_l.release()
            cap_l = cv2.VideoCapture(dev_l, cv2.CAP_V4L2)
            time.sleep(0.5)
            continue

        if cap_r is not None:
            ret_r, frame_r = cap_r.read()
            if not ret_r:
                print(f"右摄像头({dev_r})读取失败")
                cap_r = None
                frame_r = None

        frame_n += 1
        elapsed = time.time() - t0
        fps = frame_n / max(elapsed, 0.001)

        # 如果是单设备，判断是否宽帧（左右拼接）
        if cap_r is None:
            h_img, w_img = frame_l.shape[:2]
            if w_img > h_img * 1.5:
                mid = w_img // 2
                frame_r = frame_l[:, mid:].copy()
                frame_l = frame_l[:, :mid].copy()
                mode_str = "Wide-frame (auto-split)"
            else:
                frame_r = np.zeros_like(frame_l)
                cv2.putText(frame_r, "Right N/A", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (80, 80, 80), 2)
                mode_str = "Single camera"
        else:
            mode_str = "Dual cameras"

        # 统一显示尺寸
        DH, DW = 360, 640
        try:
            fl = cv2.resize(frame_l, (DW, DH))
            fr = cv2.resize(frame_r, (DW, DH))
        except Exception as e:
            print(f"Resize error: {e}")
            break

        # 添加 OSD 标签
        info = f"{mode_str}  FPS:{fps:.1f}"
        for img, lbl in [(fl, "LEFT"), (fr, "RIGHT")]:
            cv2.putText(img, lbl, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 100), 2)
            cv2.putText(img, info, (8, DH - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)

        combined = np.hstack([fl, fr])
        cv2.imshow("YLX Stereo Camera  [Q]=quit [S]=save", combined)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            break
        elif key == ord('s'):
            ts = int(time.time() * 1000)
            pl = f"{save_dir}/left_{save_count:04d}.jpg"
            pr = f"{save_dir}/right_{save_count:04d}.jpg"
            cv2.imwrite(pl, frame_l)
            cv2.imwrite(pr, frame_r if frame_r is not None else frame_l)
            save_count += 1
            print(f"  保存 #{save_count}: {pl}")

    cap_l.release()
    if cap_r is not None:
        cap_r.release()
    cv2.destroyAllWindows()
    print(f"\n完成. 共保存 {save_count} 对图片到 {save_dir}")


if __name__ == "__main__":
    main()
