"""calib_focal.py — วัดค่า focal length (px) ของกล้องข้อมือ ด้วยบล็อก 40x30 mm + ไม้บรรทัด
ใช้แปลง "ขนาดบล็อกในภาพ (px)" -> "ระยะกล้องถึงผิวบนบล็อก (mm)"  (grip_observe.py ใช้หาความสูงตอนหนีบ)

ทำไมได้: กล้องรูเข็ม  ระยะ Z = f * ขนาดจริง / ขนาดในภาพ   -> รู้ Z (ไม้บรรทัด) + ขนาดในภาพ = รู้ f
วัด 2-3 ระยะ (เช่น 80, 120, 160 mm) แล้วฟิตเส้นตรง จะได้ทั้ง f และ "ระยะชดเชย" (จุดศูนย์กลางเลนส์อยู่ลึกจากหน้าเลนส์กี่ mm)

ขั้นตอน (ไม่ใช้แขน ไม่เปิดพอร์ต Mega):
  1) python calib_focal.py
  2) วางบล็อกให้ "ผิวบนหันหน้าตรงเข้าหากล้อง" วัดระยะจากหน้าเลนส์ถึงผิวบล็อกด้วยไม้บรรทัด
  3) คลิกที่หน้าต่างภาพ พิมพ์ระยะ (mm) แล้วกด Enter -> เก็บค่า (ต้องเห็นกรอบเขียวบนบล็อก)
  4) ย้ายระยะ ทำซ้ำ 2-3 ครั้ง (บันทึก cam_focal.json อัตโนมัติทุกจุด และตอนออก)  |  Backspace = ลบตัวเลข  |  Esc = ออก
  5) (ครั้งเดียว) วัดระยะ "หน้าเลนส์ -> ปลาย jaw" ตามแนวที่กล้องมอง (jaw ชี้ลง: วัดแนวดิ่ง) พิมพ์ mm แล้วกด t
     -> grip_observe.py จะบอกได้ว่าตอนหนีบ ปลาย jaw ลึกจากผิวบนบล็อกกี่ mm / สูงจากสายพานกี่ mm
"""
import json
import math
import os
import time

import cv2
import numpy as np

import camera
import detect_block as D
from grip_observe import find_any_block

HERE = os.path.dirname(os.path.abspath(__file__))
FOCAL_JSON = os.path.join(HERE, "cam_focal.json")
BLOCK_EQ_MM = math.sqrt(40.0 * 30.0)      # ขนาด "เทียบเท่า" ของบล็อก (รากของพื้นที่) — ไม่ขึ้นกับว่าบล็อกหมุนกี่องศาในภาพ
SAMPLES = 15
CENTRE_ZONE = 0.2         # บล็อกต้องอยู่ในกรอบกลางภาพ (ตัดขอบ 20% ทุกด้าน) — กันจับถัง/ของที่โผล่ขอบภาพ


def eq_px(pose):
    return math.sqrt(pose["long_px"] * pose["short_px"])


def solve(points):
    """points: [(ระยะไม้บรรทัด mm, ขนาดเทียบเท่า px)] -> (f_px, offset_mm)
    ระยะไม้บรรทัด d = f * BLOCK_EQ_MM / px - offset  (1 จุด: offset = 0)"""
    d = np.array([p[0] for p in points], float)
    x = BLOCK_EQ_MM / np.array([p[1] for p in points], float)
    if len(points) == 1 or float(d.max() - d.min()) < 30.0:
        # จุดเดียว หรือทุกจุดระยะใกล้กัน (< 30 mm) -> หา offset ไม่ได้ (ได้ค่าเพี้ยน/ติดลบ): ใช้ offset = 0, f = ค่าเฉลี่ย
        return float(np.mean(d / x)), 0.0
    A = np.stack([x, -np.ones_like(x)], axis=1)
    (f, off), *_ = np.linalg.lstsq(A, d, rcond=None)
    if f <= 0 or not (-5.0 <= off <= 30.0):                 # offset ที่เป็นไปไม่ได้ (เลนส์จริงลึก 0-~15 mm) -> ใช้ offset = 0
        return float(np.mean(d / x)), 0.0
    return float(f), float(off)


def _read_json():
    try:
        with open(FOCAL_JSON, encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def _write_json(d):
    tmp = FOCAL_JSON + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(d, fh, ensure_ascii=False, indent=1)
    os.replace(tmp, FOCAL_JSON)


def save_focal(points, shape):
    """คำนวณ f จากจุดที่วัด แล้วบันทึก cam_focal.json (เก็บ cam_to_tip_mm เดิมไว้) -> คืนข้อความสั้นๆ"""
    f, off = solve(points)
    resid = [p[0] - (f * BLOCK_EQ_MM / p[1] - off) for p in points]
    d = _read_json()
    d.update({"f_px": f, "offset_mm": off, "block_eq_mm": BLOCK_EQ_MM, "points": points,
              "residual_mm": resid, "image_size": [shape[1], shape[0]],
              "time": time.strftime("%Y-%m-%d %H:%M:%S"),
              "note": "ระยะหน้าเลนส์->ผิวบนบล็อก (mm) = f_px * block_eq_mm / ขนาดบล็อกเทียบเท่า(px) - offset_mm"})
    _write_json(d)
    print(f"บันทึก {FOCAL_JSON}: f = {f:.0f} px, offset = {off:+.1f} mm, คลาดเคลื่อน {[round(r, 1) for r in resid]} mm")
    if len(points) < 2:
        print("  (มีจุดเดียว: offset = 0 — วัดเพิ่มอีก 1-2 ระยะจะแม่นขึ้น)")
    return f"SAVED f={f:.0f}px offset={off:+.1f}mm ({len(points)} points)"


def save_tip(tip):
    d = _read_json()
    d["cam_to_tip_mm"] = tip
    _write_json(d)
    print(f"บันทึกระยะหน้าเลนส์ -> ปลาย jaw = {tip:.0f} mm ลง {FOCAL_JSON}")
    return f"SAVED lens->jaw tip = {tip:.0f} mm"


def main():
    H = D.load_homography()
    cap = camera.open_camera()
    typed, points, collecting = "", [], None        # collecting = (ระยะ mm, [ขนาด px...])
    shape = (720, 1280)
    tip_mm = _read_json().get("cam_to_tip_mm")
    msg = "type mm + Enter = block distance (auto-saves) | mm + t = lens->jaw tip"
    win = "calib focal (mm + Enter = add point, t = jaw tip, Esc = quit)"
    try:
        while True:
            g = camera.grab(cap)
            if g is None:
                if cv2.waitKey(10) & 0xFF == 27:
                    break
                continue
            shape = g.shape
            pose = find_any_block(g, H, prefer_px=(g.shape[1] / 2.0, g.shape[0] / 2.0))
            off_centre = None
            if pose is not None and not (CENTRE_ZONE * g.shape[1] < pose["u"] < (1 - CENTRE_ZONE) * g.shape[1] and
                                         CENTRE_ZONE * g.shape[0] < pose["v"] < (1 - CENTRE_ZONE) * g.shape[0]):
                off_centre, pose = pose, None             # ไม่อยู่กลางภาพ = มักเป็นถัง/ของขอบภาพ ไม่ใช่บล็อกที่ถืออยู่ -> ไม่นับ
            if collecting is not None and pose is not None:
                collecting[1].append(eq_px(pose))
                if len(collecting[1]) >= SAMPLES:
                    px = float(np.median(collecting[1]))
                    points.append((collecting[0], px))
                    print(f"จุดที่ {len(points)}: ระยะ {collecting[0]:.0f} mm -> บล็อก {px:.1f} px")
                    msg = save_focal(points, shape) + " - move & add more, or Esc"   # บันทึกทันทีทุกจุด
                    collecting = None
            vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            if pose is not None:
                cv2.polylines(vis, [np.array(pose["box"], np.int32)], True, (0, 255, 0), 2)
                txt = f"block {eq_px(pose):.1f} px"
                if points:
                    f, off = solve(points)
                    _d = f * BLOCK_EQ_MM / eq_px(pose) - off
                    txt += f"  -> cam {_d:.0f} mm"
                    if tip_mm is not None:
                        txt += f" | jaw tip {_d - tip_mm:.0f} mm"
                cv2.putText(vis, txt, (int(pose["u"]) + 10, int(pose["v"])), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(vis, "no block seen - hold the block in the middle box", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if off_centre is not None:
                cv2.polylines(vis, [np.array(off_centre["box"], np.int32)], True, (0, 0, 255), 1)
            _h, _w = g.shape[:2]
            cv2.rectangle(vis, (int(CENTRE_ZONE * _w), int(CENTRE_ZONE * _h)), (int((1 - CENTRE_ZONE) * _w), int((1 - CENTRE_ZONE) * _h)), (255, 255, 0), 1)
            cv2.putText(vis, f"distance: {typed or '-'} mm", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            cv2.putText(vis, ("collecting..." if collecting else msg), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if ord("0") <= k <= ord("9") or k == ord("."):
                typed += chr(k)
            elif k == 8:
                typed = typed[:-1]
            elif k in (13, 10) and typed:
                try:
                    collecting = (float(typed), [])
                except ValueError:
                    msg = "bad number"
                typed = ""
            elif k == ord("t") and typed:
                try:
                    tip_mm = float(typed)
                    msg = save_tip(tip_mm)
                except ValueError:
                    msg = "bad number"
                typed = ""
            elif k == ord("s"):
                msg = save_focal(points, shape) if points else "no points yet"
    finally:
        if points:
            save_focal(points, shape)                   # ออกด้วย Esc / ปิดหน้าต่าง ก็ยังบันทึก
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
