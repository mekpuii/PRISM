"""measure_distance.py — ใช้กล้อง OV9281 วัดระยะถึงบล็อก 40x30 mm (ไม่ต้องใช้แขน ไม่ต้องทำ homography)

หลักการ (pinhole camera):  Z = f_px * ขนาดจริง / ขนาดในภาพ(px)
  - ใช้ขนาดเฉลี่ยเชิงเรขาคณิต sqrt(ยาว*กว้าง) ของบล็อก เทียบกับ sqrt(40*30) mm
  - ต้องรู้ f_px (ความยาวโฟกัสหน่วย pixel) มี 2 ทาง:
      (ก) มี intrinsics.npz จาก calib_intrinsics.py  -> ใช้เลย แม่นที่สุด
      (ข) ไม่มี -> สอบเทียบเร็วด้วยบล็อกที่ระยะรู้ค่า:  python measure_distance.py --calib 200
          (วางบล็อกห่างจากเลนส์ 200 mm วัดด้วยไม้บรรทัดจากเลนส์ถึงหน้าบนบล็อก) จะได้ focal.json

ใช้:  python measure_distance.py --calib 200     # ทำครั้งเดียว (ถ้าไม่มี intrinsics.npz)
      python measure_distance.py                  # วัดสด มีหน้าต่างภาพ (ESC = ออก)
      python measure_distance.py --nogui          # Pi ไม่มีจอ: พิมพ์ค่าอย่างเดียว

ข้อจำกัด: ถูกต้องเมื่อกล้องมองหน้าบนบล็อกตรงๆ (แกนกล้องตั้งฉากกับหน้า) ถ้าเอียงมาก ค่าจะคลาด
"""
import argparse
import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np

import camera
import config as C

FOCAL_FILE = "focal.json"
REAL_SIZE_MM = math.sqrt(C.BLOCK_LONG_MM * C.BLOCK_SHORT_MM)      # ขนาดเฉลี่ยเรขาคณิตของหน้า 40x30 (~34.6 mm)
RATIO_NOMINAL = C.BLOCK_LONG_MM / C.BLOCK_SHORT_MM                 # 1.333


def find_block_px(gray):
    """หาบล็อกในภาพ คืน dict(cx, cy, long_px, short_px, box) หรือ None
    ใช้ threshold + contour + minAreaRect แล้วกรองด้วยสัดส่วน ~1.33 (ยังไม่รู้ระยะจึงเช็กสัดส่วนแทนขนาด mm)"""
    # --- 1) จำกัดพื้นที่ค้นหา (ตัด jaw/ตัวแขนที่อาจโผล่ในภาพ) ---
    x0, y0, x1, y1 = C.DETECT_ROI
    mask = np.zeros_like(gray)
    mask[y0:y1, x0:x1] = 255

    # --- 2) Otsu threshold (กล้อง mono แยกด้วยความสว่างเท่านั้น) ---
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    mode = cv2.THRESH_BINARY if C.BLOCK_IS_BRIGHT else cv2.THRESH_BINARY_INV
    _, bw = cv2.threshold(blur, 0, 255, mode + cv2.THRESH_OTSU)
    bw = cv2.bitwise_and(bw, mask)

    # --- 3) ล้าง noise ---
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k)

    # --- 4) เลือก contour ที่เป็นสี่เหลี่ยมสัดส่วน ~1.33 และ "เต็มกรอบ" ที่สุด ---
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < C.MIN_CONTOUR_AREA_PX:
            continue
        (cx, cy), (w, h), _ang = cv2.minAreaRect(c)
        if min(w, h) < 1:
            continue
        long_px, short_px = max(w, h), min(w, h)
        ratio = long_px / short_px
        fill = area / (long_px * short_px)                   # สี่เหลี่ยมจริงควร fill ~1.0 (ตัดรูปทรงอื่นทิ้ง)
        if not (1.15 < ratio < 1.55) or fill < 0.85:
            continue
        if area > best_area:
            box = cv2.boxPoints(cv2.minAreaRect(c)).astype(np.int32)
            best, best_area = {"cx": cx, "cy": cy, "long_px": long_px, "short_px": short_px, "box": box}, area
    return best


def size_px(b):
    """ขนาดเฉลี่ยเรขาคณิตของบล็อกในภาพ (px) — ทนต่อการหมุนของบล็อก"""
    return math.sqrt(b["long_px"] * b["short_px"])


def load_intrinsics(w, h):
    """คืน (fx, fy, cx0, cy0) หรือ None ถ้ายังไม่มีข้อมูลสอบเทียบ"""
    if os.path.exists(C.INTRINSICS_FILE):                    # ภาพถูก undistort ด้วย K นี้แล้วใน camera.grab
        K = np.load(C.INTRINSICS_FILE)["K"]
        return float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    if os.path.exists(FOCAL_FILE):
        f = json.load(open(FOCAL_FILE))["f_px"]
        return f, f, w / 2.0, h / 2.0                        # สมมติจุดกลางภาพ = จุดหลัก
    return None


def measure(b, intr):
    """จากบล็อกในภาพ -> ระยะ Z ตามแกนกล้อง, ออฟเซต X/Y ด้านข้าง และระยะเส้นตรงถึงบล็อก (mm)"""
    fx, fy, cx0, cy0 = intr
    z = fx * REAL_SIZE_MM / size_px(b)                       # Z = f * real / pixel
    x = (b["cx"] - cx0) * z / fx                             # +x = ขวาของภาพ
    y = (b["cy"] - cy0) * z / fy                             # +y = ลงล่างของภาพ
    return {"Z": z, "X": x, "Y": y, "D": math.sqrt(x * x + y * y + z * z)}


def calibrate(cap, dist_mm, n=40):
    """สอบเทียบ f_px: วางบล็อกที่ระยะ dist_mm แล้วเฉลี่ยขนาดในภาพ n เฟรม -> บันทึก focal.json"""
    sizes = []
    for _ in range(n):
        g = camera.grab(cap)
        b = find_block_px(g) if g is not None else None
        if b:
            sizes.append(size_px(b))
    if len(sizes) < n * 0.7:
        raise SystemExit(f"เจอบล็อกแค่ {len(sizes)}/{n} เฟรม — ปรับแสง/BLOCK_IS_BRIGHT/DETECT_ROI ก่อน")
    s = float(np.median(sizes))
    f_px = s * dist_mm / REAL_SIZE_MM                        # ย้อนสูตร: f = pixel * Z / real
    json.dump({"f_px": f_px, "calib_dist_mm": dist_mm}, open(FOCAL_FILE, "w"))
    print(f"ขนาดในภาพ {s:.1f} px ที่ {dist_mm} mm -> f_px = {f_px:.1f}  (บันทึก {FOCAL_FILE})")
    print(f"ความนิ่ง: std = {np.std(sizes):.2f} px (ควรน้อยกว่า ~1 px)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", type=float, metavar="MM", help="สอบเทียบ f_px ด้วยบล็อกที่ห่างเลนส์ MM mm")
    ap.add_argument("--nogui", action="store_true", help="ไม่เปิดหน้าต่างภาพ (Pi ไม่มีจอ)")
    args = ap.parse_args()

    cap = camera.open_camera()
    try:
        if args.calib:
            calibrate(cap, args.calib)
            return
        g = camera.grab(cap)
        intr = load_intrinsics(g.shape[1], g.shape[0])
        if intr is None:
            raise SystemExit("ยังไม่มี intrinsics.npz / focal.json — รัน: python measure_distance.py --calib <ระยะ mm> ก่อน")
        hist = deque(maxlen=7)                               # median filter กันค่ากระโดด
        last_print = 0.0
        while True:
            g = camera.grab(cap)
            if g is None:
                continue
            b = find_block_px(g)
            if b:
                hist.append(measure(b, intr))
            elif hist:
                hist.popleft()                               # หายจากภาพ -> ค่อยๆ เคลียร์ ไม่ค้างค่าเก่า
            med = None
            if hist:
                med = {k: float(np.median([h[k] for h in hist])) for k in ("Z", "X", "Y", "D")}
            if med and time.time() - last_print > 0.2:       # พิมพ์ 5 ครั้ง/วินาที
                print(f"Z={med['Z']:6.1f} mm  X={med['X']:6.1f}  Y={med['Y']:6.1f}  ระยะตรง={med['D']:6.1f} mm")
                last_print = time.time()
            if not args.nogui:
                vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
                if b:
                    cv2.polylines(vis, [b["box"]], True, (0, 255, 0), 2)
                if med:
                    cv2.putText(vis, f"Z={med['Z']:.0f} mm", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imshow("distance (ESC = ออก)", vis)
                if cv2.waitKey(1) == 27:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
