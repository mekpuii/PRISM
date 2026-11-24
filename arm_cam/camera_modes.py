"""camera_modes.py — ลองทุกชุด (backend x ความละเอียด x fourcc x ลำดับการตั้งค่า) วัด fps จริง แล้วลองตั้ง exposure ในชุดที่ดีที่สุด
รันเองจบ ไม่มีหน้าต่าง/ไม่ถามอะไร ผลลง camera_modes.txt   ใช้:  python camera_modes.py [index]   (ค่าเริ่มต้น index 0)"""
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IDX = int(sys.argv[1]) if len(sys.argv) > 1 else 0
IS_WIN = sys.platform.startswith("win")
BACKENDS = [("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)] if IS_WIN else [("V4L2", cv2.CAP_V4L2)]
SIZES = [(1280, 720), (1280, 800), (640, 480), (640, 400)]
FOURCCS = ["MJPG", "YUY2", "NV12"]
ORDERS = ["fourcc_first", "size_first"]
out = []


def log(s=""):
    print(s, flush=True)
    out.append(s)


def fcc_str(cap):
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join(chr((v >> (8 * i)) & 255) for i in range(4))


def open_mode(be, size, fcc, order, fps=120):
    cap = cv2.VideoCapture(IDX, be)
    if not cap.isOpened():
        cap.release()
        return None
    fourcc = cv2.VideoWriter_fourcc(*fcc)
    if order == "fourcc_first":
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    else:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def measure(cap, secs=1.5):
    for _ in range(8):                                   # ทิ้งเฟรมแรก
        cap.read()
    n, t0, last = 0, time.time(), None
    while time.time() - t0 < secs:
        ok, f = cap.read()
        if ok and f is not None:
            n += 1
            last = f
    return (n / (time.time() - t0), last)


results = []
log(f"OpenCV {cv2.__version__} index {IDX} {time.strftime('%Y-%m-%d %H:%M:%S')}")
for bname, be in BACKENDS:
    for size in SIZES:
        for fcc in FOURCCS:
            for order in ORDERS:
                try:
                    cap = open_mode(be, size, fcc, order)
                    if cap is None:
                        continue
                    fps, f = measure(cap)
                    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), fcc_str(cap))
                    cap.release()
                    if f is None:
                        continue
                    ok_fcc = "ตรง" if got[2] == fcc else f"ได้ {got[2]!r}"
                    log(f"{bname:5s} ขอ {size[0]}x{size[1]} {fcc} ({order:12s}) -> ได้ {got[0]}x{got[1]} fourcc {ok_fcc:14s} fps จริง {fps:5.1f}")
                    results.append((fps, bname, be, size, fcc, order, got))
                except Exception as e:                   # noqa: BLE001
                    log(f"{bname} {size} {fcc} {order}: error {e!r}")

if not results:
    log("ไม่มีชุดไหนอ่านภาพได้ (ปิดโปรแกรมอื่นที่ใช้กล้อง? สาย USB?)")
else:
    results.sort(key=lambda r: -r[0])
    log("\n=== 5 อันดับ fps สูงสุด ===")
    for r in results[:5]:
        log(f"{r[0]:5.1f} fps  {r[1]} {r[3][0]}x{r[3][1]} ขอ {r[4]} ({r[5]}) ได้ {r[6]}")
    # ชุดที่ดีที่สุดที่ความละเอียด >= 640 : ลอง exposure
    best = results[0]
    log(f"\n=== ทดสอบ exposure ในชุดที่เร็วสุด: {best[1]} {best[3]} {best[4]} ===")
    for auto_val in (0.25, 1, 0.75):                    # DSHOW: 0.25 = manual ; V4L2: 1 = manual ; 0.75 = auto
        row = []
        for exp in (-3, -5, -7, -9, -11):
            cap = open_mode(best[2], best[3], best[4], best[5])
            if cap is None:
                break
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, auto_val)
            cap.set(cv2.CAP_PROP_EXPOSURE, exp)
            time.sleep(0.4)
            fps, f = measure(cap, 1.0)
            readback = cap.get(cv2.CAP_PROP_EXPOSURE)
            cap.release()
            if f is not None:
                g = f if f.ndim == 2 else cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
                row.append(f"exp {exp:>3} (อ่านกลับ {readback:>6.1f}) สว่าง {g.mean():5.1f} fps {fps:4.1f}")
        log(f"auto_exposure={auto_val}:")
        for r in row:
            log("   " + r)
    log("\nถ้าสว่างเปลี่ยนตามค่า exp = ตั้ง exposure ได้ (เลือกค่าที่สว่างเฉลี่ย ~100-140) | ถ้าเท่ากันทุกค่า = driver ไม่รับ")
with open(os.path.join(HERE, "camera_modes.txt"), "w", encoding="utf-8") as fh:
    fh.write("\n".join(out) + "\n")
