"""camera_report.py — ตรวจกล้องทุก index แบบรันเองจบ (ไม่มีหน้าต่าง ไม่ถามอะไร) แล้วเขียนผลลง camera_report.txt + ภาพ cam_idx<N>_<backend>.png
ดับเบิลคลิก run_camera_check.bat หรือรัน:  python camera_report.py
ต่อ 1 กล้อง: ความละเอียดที่ได้, fps จริง (วัด 2 วินาที), สว่างเฉลี่ย, สัดส่วนพิกเซลที่ขาวจ้า/มืดสนิท, ภาพเป็นขาวดำจริงหรือไม่ (OV9281 = ขาวดำ)"""
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IS_WIN = sys.platform.startswith("win")
BACKENDS = [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)] if IS_WIN else [("V4L2", cv2.CAP_V4L2)]
out = []


def log(s=""):
    print(s, flush=True)
    out.append(s)


def probe(idx, name, be):
    cap = cv2.VideoCapture(idx, be)
    if not cap.isOpened():
        cap.release()
        return False
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 60)
    frame = None
    for _ in range(15):                                  # ทิ้งเฟรมแรกๆ
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
        time.sleep(0.03)
    if frame is None:
        log(f"index {idx} [{name}]: เปิดได้แต่อ่านภาพไม่ได้")
        cap.release()
        return True
    n, t0 = 0, time.time()
    while time.time() - t0 < 2.0:                        # วัด fps จริง
        ok, f = cap.read()
        if ok and f is not None:
            n += 1
            frame = f
    fps = n / (time.time() - t0)
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    mono = True
    if frame.ndim == 3:
        d = frame.astype(np.int16)
        mono = float(np.abs(d[..., 0] - d[..., 1]).mean() + np.abs(d[..., 1] - d[..., 2]).mean()) < 2.0
    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    log(f"index {idx} [{name}]: {w}x{h}  fps จริง {fps:.0f}  fourcc {''.join(chr((fcc >> (8 * i)) & 255) for i in range(4))!r}  "
        f"สว่างเฉลี่ย {gray.mean():.0f}/255  ขาวจ้า(>=250) {100 * (gray >= 250).mean():.1f}%  มืดสนิท(<=5) {100 * (gray <= 5).mean():.1f}%  "
        f"ความคม {sharp:.0f}  {'ขาวดำ (น่าจะเป็น OV9281)' if mono else 'สี (น่าจะเป็นเว็บแคมธรรมดา)'}")
    cv2.imwrite(os.path.join(HERE, f"cam_idx{idx}_{name}.png"), frame)
    cap.release()
    return True


log(f"OpenCV {cv2.__version__} | Python {sys.version.split()[0]} | {time.strftime('%Y-%m-%d %H:%M:%S')}")
found = 0
for idx in range(8):
    for name, be in BACKENDS:
        try:
            found += bool(probe(idx, name, be))
        except Exception as e:                           # noqa: BLE001
            log(f"index {idx} [{name}]: error {e!r}")
log(f"\nเจอกล้องที่เปิดได้ {found} รายการ (index เดียวกันอาจซ้ำต่างอัน backend) — ภาพบันทึกไว้ที่ {HERE}")
with open(os.path.join(HERE, "camera_report.txt"), "w", encoding="utf-8") as f:
    f.write("\n".join(out) + "\n")
