#!/usr/bin/env python3
"""
Step 4: Combined stereo viewer + IMU display
After XU unit/selector are confirmed from step3, update XU_UNIT / XU_SELECTOR below.

Shows left/right image side-by-side with live IMU data overlay.

Usage:
  sudo python3 step4_stereo_imu.py
"""

import cv2
import numpy as np
import os
import time
import struct
import threading
import ctypes
import fcntl

# ============================================================
#  CONFIG — update after running step3_imu_probe.py
# ============================================================
LEFT_IDX   = 0
RIGHT_IDX  = 1           # Set to None if single wide-frame camera
XU_UNIT    = 3           # From step3 output — unit ID for IMU
XU_SELECTOR= 1           # From step3 output — selector for IMU data
XU_LEN     = 24          # Bytes: 3 gyro(float32) + 3 accel(float32) = 24B (typical)
# ============================================================

UVCIOC_CTRL_QUERY = 0xC0186F21
UVC_GET_CUR = 0x81


class IMUReader(threading.Thread):
    """Background thread: continuously reads IMU via UVC XU."""
    def __init__(self, dev="/dev/video0", unit=XU_UNIT, sel=XU_SELECTOR, length=XU_LEN):
        super().__init__(daemon=True)
        self.dev    = dev
        self.unit   = unit
        self.sel    = sel
        self.length = length
        self._lock  = threading.Lock()
        self._data  = {"gx": 0.0, "gy": 0.0, "gz": 0.0,
                       "ax": 0.0, "ay": 0.0, "az": 0.0,
                       "ts": 0, "ok": False, "rate": 0.0}
        self._fd    = None

    def _open(self):
        try:
            self._fd = os.open(self.dev, os.O_RDWR | os.O_NONBLOCK)
            return True
        except Exception as e:
            print(f"[IMU] Cannot open {self.dev}: {e}")
            return False

    def _read_imu(self):
        class XUQuery(ctypes.Structure):
            _fields_ = [
                ('unit',     ctypes.c_uint8),
                ('selector', ctypes.c_uint8),
                ('query',    ctypes.c_uint16),
                ('size',     ctypes.c_uint16),
                ('reserved', ctypes.c_uint16),
                ('data',     ctypes.c_void_p),
            ]
        buf = (ctypes.c_uint8 * self.length)()
        xu = XUQuery()
        xu.unit = self.unit; xu.selector = self.sel
        xu.query = UVC_GET_CUR; xu.size = self.length
        xu.data = ctypes.cast(buf, ctypes.c_void_p)
        try:
            fcntl.ioctl(self._fd, UVCIOC_CTRL_QUERY, xu)
            return bytes(buf)
        except Exception:
            return None

    def _parse(self, raw):
        """Default: 3 float32 gyro + 3 float32 accel = 24 bytes.
        Adjust if your camera uses a different layout."""
        if raw is None or len(raw) < 24:
            return None
        try:
            gx, gy, gz, ax, ay, az = struct.unpack_from("<ffffff", raw)
            return {"gx": gx, "gy": gy, "gz": gz,
                    "ax": ax, "ay": ay, "az": az}
        except Exception:
            return None

    def run(self):
        if not self._open():
            return
        count = 0
        t0 = time.time()
        while True:
            raw = self._read_imu()
            parsed = self._parse(raw)
            count += 1
            dt = time.time() - t0
            if dt >= 1.0:
                rate = count / dt
                count = 0
                t0 = time.time()
            else:
                rate = self._data["rate"]

            with self._lock:
                if parsed:
                    self._data.update(parsed)
                    self._data["ok"] = True
                else:
                    self._data["ok"] = False
                self._data["rate"] = rate
                self._data["ts"] = int(time.time() * 1000)
            time.sleep(0.005)  # 200 Hz max

    def get(self):
        with self._lock:
            return dict(self._data)


def draw_imu_panel(canvas, imu_data, x, y):
    """Draw IMU values as text overlay."""
    ok = imu_data.get("ok", False)
    color = (0, 255, 100) if ok else (100, 100, 100)
    status = f"IMU: {'OK' if ok else 'NO DATA'}  {imu_data.get('rate', 0):.0f}Hz"
    lines = [
        status,
        f"Gyro  X:{imu_data.get('gx', 0):+7.3f}",
        f"      Y:{imu_data.get('gy', 0):+7.3f}",
        f"      Z:{imu_data.get('gz', 0):+7.3f}  deg/s",
        f"Accel X:{imu_data.get('ax', 0):+7.3f}",
        f"      Y:{imu_data.get('ay', 0):+7.3f}",
        f"      Z:{imu_data.get('az', 0):+7.3f}  m/s²",
    ]
    # Background
    panel_h = len(lines) * 22 + 12
    panel_w = 280
    cv2.rectangle(canvas, (x, y), (x + panel_w, y + panel_h), (20, 20, 20), -1)
    cv2.rectangle(canvas, (x, y), (x + panel_w, y + panel_h), (60, 60, 60), 1)
    for i, line in enumerate(lines):
        lc = color if i == 0 else (200, 200, 200)
        cv2.putText(canvas, line, (x + 8, y + 20 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, lc, 1, cv2.LINE_AA)


def main():
    print("=" * 55)
    print("  Stereo Camera + IMU Viewer  (V1)")
    print(f"  Left: /dev/video{LEFT_IDX}  Right: /dev/video{RIGHT_IDX}")
    print(f"  IMU: XU unit={XU_UNIT} sel={XU_SELECTOR} len={XU_LEN}B")
    print("=" * 55)
    print("Keys: Q=quit  S=save  C=toggle IMU")

    # Start IMU reader thread
    imu = IMUReader("/dev/video0", XU_UNIT, XU_SELECTOR, XU_LEN)
    imu.start()

    # Open cameras
    cap_l = cv2.VideoCapture(LEFT_IDX, cv2.CAP_V4L2)
    cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_l.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap_r = None
    if RIGHT_IDX is not None:
        cap_r = cv2.VideoCapture(RIGHT_IDX, cv2.CAP_V4L2)
        cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap_r.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    if not cap_l.isOpened():
        print("ERROR: Cannot open left camera")
        return

    save_dir = os.path.expanduser("~/stereo_frames")
    os.makedirs(save_dir, exist_ok=True)

    show_imu = True
    save_count = 0
    t0 = time.time()
    fps_count = 0
    fps_disp = 0.0

    while True:
        ret_l, frame_l = cap_l.read()
        if not ret_l:
            print("Left camera failed")
            break

        if cap_r is not None:
            ret_r, frame_r = cap_r.read()
            if not ret_r:
                cap_r = None

        # FPS
        fps_count += 1
        dt = time.time() - t0
        if dt >= 1.0:
            fps_disp = fps_count / dt
            fps_count = 0
            t0 = time.time()

        # Single wide-frame auto-split
        if cap_r is None:
            if frame_l.shape[1] >= frame_l.shape[0] * 1.5:
                mid = frame_l.shape[1] // 2
                frame_r = frame_l[:, mid:].copy()
                frame_l = frame_l[:, :mid].copy()
            else:
                frame_r = np.zeros_like(frame_l)

        # Resize for display
        dw, dh = 540, 400
        disp_l = cv2.resize(frame_l, (dw, dh))
        disp_r = cv2.resize(frame_r, (dw, dh))

        # Labels
        cv2.putText(disp_l, f"LEFT  {frame_l.shape[1]}x{frame_l.shape[0]}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(disp_r, f"RIGHT {frame_r.shape[1]}x{frame_r.shape[0]}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # FPS
        cv2.putText(disp_l, f"FPS:{fps_disp:.1f}", (dw - 100, dh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Combine
        combined = np.hstack([disp_l, disp_r])

        # IMU overlay
        if show_imu:
            imu_data = imu.get()
            draw_imu_panel(combined, imu_data, 10, dh - 180)

        # Instructions
        cv2.putText(combined, "Q:quit  S:save  C:imu", (combined.shape[1] - 230, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        cv2.imshow("Stereo + IMU Viewer", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            ts = int(time.time() * 1000)
            cv2.imwrite(f"{save_dir}/L_{save_count:04d}_{ts}.jpg", frame_l)
            cv2.imwrite(f"{save_dir}/R_{save_count:04d}_{ts}.jpg", frame_r)
            save_count += 1
            print(f"  Saved frame pair #{save_count}")
        elif key == ord('c'):
            show_imu = not show_imu

    cap_l.release()
    if cap_r:
        cap_r.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
