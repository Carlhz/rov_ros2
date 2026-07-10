#!/usr/bin/env python3
"""
Step 2: Stereo camera image viewer
Tests both camera channels side by side.

Usage:
  pip3 install opencv-python
  python3 step2_stereo_viewer.py

Press Q to quit, S to save a frame pair.
"""

import cv2
import numpy as np
import time
import os

# ---- Config ---- 
# Adjust these if left/right are swapped or at different indices
LEFT_IDX  = 0
RIGHT_IDX = 1
TARGET_W  = 640
TARGET_H  = 480
FPS       = 30

def try_open(idx):
    """Try to open a video device, return cap or None."""
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  TARGET_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_H)
        cap.set(cv2.CAP_PROP_FPS, FPS)
        # Try MJPEG for USB3 bandwidth
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        print(f"  /dev/video{idx}: opened  "
              f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
              f"{int(cap.get(cv2.CAP_PROP_FPS))}fps")
        return cap
    print(f"  /dev/video{idx}: FAILED to open")
    return None


def scan_cameras():
    """Find all available video devices."""
    found = []
    for i in range(10):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                found.append(i)
                print(f"  /dev/video{i}: OK (readable)")
            else:
                print(f"  /dev/video{i}: opened but no frame")
            cap.release()
    return found


def main():
    print("=" * 50)
    print("  Stereo Camera Viewer")
    print("=" * 50)

    print("\n[Scanning for video devices...]")
    available = scan_cameras()
    print(f"  Available: {available}")

    if len(available) == 0:
        print("\nERROR: No video devices found.")
        print("Check: ls /dev/video*")
        return
    if len(available) == 1:
        print(f"\nOnly 1 camera found at index {available[0]}.")
        print("Trying to read a wide frame (some stereo cams output L+R side-by-side on one device)...")
        LEFT_IDX  = available[0]
        RIGHT_IDX = None
    else:
        LEFT_IDX  = available[0]
        RIGHT_IDX = available[1]

    print(f"\n[Opening cameras: left={LEFT_IDX}, right={RIGHT_IDX}]")
    cap_l = try_open(LEFT_IDX)
    cap_r = try_open(RIGHT_IDX) if RIGHT_IDX is not None else None

    if cap_l is None:
        print("Cannot open left camera. Exiting.")
        return

    save_dir = os.path.expanduser("~/stereo_frames")
    os.makedirs(save_dir, exist_ok=True)
    frame_count = 0
    t0 = time.time()
    fps_display = 0.0
    save_count = 0

    print("\n[Running — press Q to quit, S to save frame pair]")
    print(f"  Frames saved to: {save_dir}")

    while True:
        ret_l, frame_l = cap_l.read()
        if not ret_l:
            print("Left camera read failed!")
            break

        if cap_r is not None:
            ret_r, frame_r = cap_r.read()
            if not ret_r:
                print("Right camera read failed, showing left only")
                cap_r = None

        # FPS
        frame_count += 1
        elapsed = time.time() - t0
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            frame_count = 0
            t0 = time.time()

        # If single wide camera, split left/right
        if cap_r is None and frame_l.shape[1] > frame_l.shape[0] * 1.5:
            mid = frame_l.shape[1] // 2
            frame_r = frame_l[:, mid:]
            frame_l = frame_l[:, :mid]
        elif cap_r is None:
            frame_r = np.zeros_like(frame_l)
            cv2.putText(frame_r, "No right camera", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)

        # Resize to uniform size for display
        h, w = 360, 480
        disp_l = cv2.resize(frame_l, (w, h))
        disp_r = cv2.resize(frame_r, (w, h))

        # Overlay info
        info = f"FPS:{fps_display:.1f}  {frame_l.shape[1]}x{frame_l.shape[0]}  [S]save [Q]quit"
        for disp, label in [(disp_l, "LEFT"), (disp_r, "RIGHT")]:
            cv2.putText(disp, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(disp, info, (10, disp.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Combine side by side
        combined = np.hstack([disp_l, disp_r])
        cv2.imshow("Stereo Camera Viewer", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            ts = int(time.time() * 1000)
            path_l = f"{save_dir}/left_{save_count:04d}_{ts}.jpg"
            path_r = f"{save_dir}/right_{save_count:04d}_{ts}.jpg"
            cv2.imwrite(path_l, frame_l)
            cv2.imwrite(path_r, frame_r)
            save_count += 1
            print(f"  Saved pair #{save_count}: {path_l}")

    cap_l.release()
    if cap_r:
        cap_r.release()
    cv2.destroyAllWindows()
    print(f"\nDone. Saved {save_count} frame pairs to {save_dir}")


if __name__ == "__main__":
    main()
