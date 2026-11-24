"""grip_observe.py — "ดูอย่างเดียว" ว่าแขนหยิบบล็อกยังไง แล้วเรียนรู้ตำแหน่งหนีบจากกล้องข้อมือ

ใช้ตอนที่ระบบอื่น (Arduino Mega / PRISM ของเพื่อน) คุมแขนอยู่:
  * เปิด "กล้องอย่างเดียว" — ไม่เปิดพอร์ต Mega (COM) เลย จึงไม่แย่งพอร์ตกับโปรแกรมของเพื่อน
  * ทุกครั้งที่แขนหยิบบล็อก จะจับ "จังหวะที่ jaw ปิด" แล้วบันทึกว่าบล็อกอยู่ตรงไหนในภาพ (จุดกลาง/มุม/ขนาด px)
  * สะสมหลายครั้ง -> grip_learned.json = "ภาพของบล็อกตอนพร้อมหนีบ" (เป้าหมายสำหรับเล็งด้วยกล้องในอนาคต)

หลักการ (ไม่ต้องรู้ค่าข้อต่อของแขน):
  กล้องติดบนข้อมือ -> ถ้าบล็อกถูกหนีบอยู่ บล็อก "นิ่งในภาพ" ขณะที่ฉากหลัง "เลื่อน" (แขนกำลังยก/ย้าย)
    d_out = ภาพเปลี่ยนเท่าไรนอกกรอบบล็อก (ฉากหลังขยับ = กล้องขยับ)
    d_in  = ภาพเปลี่ยนเท่าไรในกรอบบล็อก
    HELD  = d_out สูง และ d_in ต่ำ ต่อเนื่อง HELD_MIN_S วินาที
  จังหวะหนีบ = เฟรมล่าสุด "ก่อน" เริ่ม HELD ที่กล้องนิ่ง (แขนหยุดลงแล้วปิด jaw) และยังเห็นบล็อก

ใช้:  python grip_observe.py            (มีหน้าต่างภาพ, Esc = ออก)
      python grip_observe.py --nogui    (ไม่มีหน้าต่าง)
      python grip_observe.py --trace    (บันทึกค่าทุก ~0.1 s ลง grip_trace.csv ไว้จูน threshold)
ผลลัพธ์: grip_observations.csv, grip_learned.json, ภาพใน grip_obs/
ค่า threshold ด้านล่างเป็นค่าเริ่มต้น — ดูตัวเลข d_out / d_in บนจอขณะแขนทำงานจริงแล้วปรับ
"""
import argparse
import collections
import csv
import json
import math
import os
import time

import cv2
import numpy as np

import camera
import config as C
import detect_block as D

HERE = os.path.dirname(os.path.abspath(__file__))
OBS_DIR = os.path.join(HERE, "grip_obs")
OBS_CSV = os.path.join(HERE, "grip_observations.csv")
LEARNED_JSON = os.path.join(HERE, "grip_learned.json")
TRACE_CSV = os.path.join(HERE, "grip_trace.csv")
FOCAL_JSON = os.path.join(HERE, "cam_focal.json")
BLOCK_EQ_MM = math.sqrt(C.BLOCK_LONG_MM * C.BLOCK_SHORT_MM)   # ขนาดเทียบเท่าของบล็อก (รากพื้นที่) ไม่ขึ้นกับการหมุน

# ---- ค่าตัดสิน (จูนจากตัวเลขบนจอ) ----
MOTION_SCALE = 4          # คำนวณการเปลี่ยนแปลงบนภาพย่อ 1/4 (เร็ว + ไม่ไวต่อ noise)
MOVE_THR = 6.0            # d_out (ระดับเทาเฉลี่ยที่เปลี่ยนต่อเฟรม) เกินนี้ = กล้อง/แขนกำลังขยับ
STILL_THR = 3.0           # d_out ต่ำกว่านี้ = กล้องนิ่ง
LOCK_RATIO = 0.35         # d_in < d_out * นี้ = บล็อกขยับไปกับกล้อง (ถูกหนีบ)
HELD_MIN_S = 0.4          # หลักฐาน "ถือบล็อก" ต้องมีต่อเนื่องเท่านี้
HELD_WINDOW_FRAC = 0.7    # ในช่วง HELD_MIN_S ต้องเป็นหลักฐานอย่างน้อยสัดส่วนนี้ของเฟรม
RELEASE_S = 0.5           # ตอน HELD: กรอบบล็อกเปลี่ยน (d_in > MOVE_THR) ต่อเนื่องเท่านี้ = ปล่อยแล้ว
HISTORY_S = 4.0           # เก็บภาพย้อนหลังเท่านี้ (ไว้ย้อนหาจังหวะหนีบ)
COOLDOWN_S = 1.0          # หลังปล่อย รอเท่านี้ก่อนนับการหยิบครั้งใหม่
GRIP_TIP_ABOVE_MAX_MM = 15.0   # ตอนหนีบ ปลาย jaw ต้องไม่สูงกว่าผิวบนบล็อกเกินนี้ (ไม่งั้นหนีบไม่ถึง = เงาสะท้อน/จับผิด) — ต้องรู้ cam_to_tip_mm
GRIP_CAM_MAX_MM = 150.0        # (ถ้ายังไม่ได้วัด cam_to_tip) กล้องห่างผิวบล็อกเกินนี้ตอนหนีบ = ไม่ใช่การหนีบจริง
HELD_LOST_S = 2.0              # ตอน HELD: มองไม่เห็นบล็อกเลยนานเท่านี้ -> กลับ WATCH (กันค้าง HELD)
KEEP_EVENTS = 200         # grip_learned.json เก็บรายการล่าสุดเท่านี้


def plausible_grip(pose):
    """เช็กทางฟิสิกส์ว่า "หนีบจริงได้ไหม" -> (True, "") หรือ (False, เหตุผล)
    ปลาย jaw ต้องลงไปถึงบล็อก: ถ้าปลาย jaw ยังสูงกว่าผิวบนบล็อกมาก = ของที่เห็นไม่ใช่บล็อกที่ถูกหนีบ (เช่นแสงสะท้อนที่เลื่อนตามกล้อง)"""
    d = dist_mm(eq_px(pose))
    if d is None:
        return True, ""
    dep = tip_info(d)[0]
    if dep is not None:
        if dep < -GRIP_TIP_ABOVE_MAX_MM:
            return False, f"ปลาย jaw อยู่สูงกว่าผิวบล็อก {-dep:.0f} mm (> {GRIP_TIP_ABOVE_MAX_MM:.0f}) หนีบไม่ถึง"
    elif d > GRIP_CAM_MAX_MM:
        return False, f"กล้องห่างบล็อก {d:.0f} mm (> {GRIP_CAM_MAX_MM:.0f}) ไกลเกินกว่าจะหนีบ"
    return True, ""


def block_pose_px(res):
    """จาก find_block -> ท่าบล็อกในภาพ: จุดกลาง (u,v), ด้านยาว/สั้น (px), มุมแกนยาวในภาพ (องศา 0-180)"""
    b = res["box_px"]
    e0, e1 = b[1] - b[0], b[2] - b[1]
    l0, l1 = float(np.hypot(*e0)), float(np.hypot(*e1))
    lv = e0 if l0 >= l1 else e1
    ang = math.degrees(math.atan2(float(lv[1]), float(lv[0]))) % 180.0
    c = b.mean(axis=0)
    return {"u": float(c[0]), "v": float(c[1]), "long_px": max(l0, l1), "short_px": min(l0, l1),
            "angle_img": ang, "gray": res.get("gray"), "tone": res.get("tone"), "box": b.tolist()}


def load_focal():
    """(f_px, offset_mm) จาก cam_focal.json (calib_focal.py) หรือ intrinsics.npz — None ถ้ายังไม่ได้วัด"""
    try:
        with open(FOCAL_JSON, encoding="utf-8") as f:
            d = json.load(f)
        return float(d["f_px"]), float(d.get("offset_mm", 0.0))
    except (OSError, ValueError, KeyError):
        pass
    if os.path.exists(C.INTRINSICS_FILE):
        try:
            return float(np.load(C.INTRINSICS_FILE)["K"][0, 0]), 0.0
        except Exception:
            pass
    return None


FOCAL = load_focal()


def load_cam_to_tip():
    """ระยะหน้าเลนส์ -> ปลาย jaw (mm) วัดด้วยไม้บรรทัดครั้งเดียว (calib_focal.py ปุ่ม t) — None ถ้ายังไม่ได้วัด"""
    try:
        with open(FOCAL_JSON, encoding="utf-8") as f:
            v = json.load(f).get("cam_to_tip_mm")
        return None if v is None else float(v)
    except (OSError, ValueError):
        return None


CAM_TO_TIP = load_cam_to_tip()


def tip_info(d_block):
    """จากระยะกล้อง->ผิวบนบล็อก -> (ปลาย jaw ลึกจากผิวบนบล็อกกี่ mm, ปลาย jaw สูงจากสายพานกี่ mm) หรือ (None, None)
    ลึก = (เลนส์->ปลาย jaw) - (เลนส์->ผิวบล็อก)   บวก = ปลาย jaw ต่ำกว่าผิวบนบล็อก (กินเข้าไปในบล็อก)"""
    if d_block is None or CAM_TO_TIP is None:
        return None, None
    depth = CAM_TO_TIP - d_block
    return depth, C.BLOCK_H_MM - depth


def eq_px(pose):
    return math.sqrt(pose["long_px"] * pose["short_px"])


def dist_mm(eqpx, focal=None):
    """ระยะหน้าเลนส์ -> ผิวบนบล็อก (mm) จากขนาดบล็อกในภาพ (กล้องรูเข็ม: Z = f * ขนาดจริง / ขนาดในภาพ)"""
    focal = focal or FOCAL
    if focal is None or not eqpx:
        return None
    return focal[0] * BLOCK_EQ_MM / eqpx - focal[1]


def find_any_block(g, H, prefer_px=None):
    """หาบล็อกโดย "ไม่เช็กขนาด mm" (แขนอยู่ท่าไหนก็ได้ homography ใช้ไม่ได้) — ใช้แค่รูปทรง/ความเรียบ/รอบข้าง"""
    h, w = g.shape[:2]
    res = D.find_block(g, H, roi=(0, 0, w, h), px_mode=True, prefer_px=prefer_px)
    return block_pose_px(res) if res is not None else None


def motion(prev_s, cur_s, box):
    """d_out, d_in: ความต่างเฉลี่ยของภาพย่อ นอก/ใน กรอบบล็อก (box = มุม 4 จุดใน px เต็ม)"""
    if prev_s is None:
        return 0.0, 0.0
    diff = cv2.absdiff(prev_s, cur_s)
    h, w = diff.shape
    m = np.zeros((h, w), np.uint8)
    if box is not None:
        cv2.fillPoly(m, [(np.array(box, np.float32) / MOTION_SCALE).astype(np.int32)], 255)
    mx, my = int(w * 0.1), int(h * 0.1)                       # ไม่นับขอบภาพ 10% (jaw/สายที่ติดกล้องอยู่)
    out = np.zeros_like(m)
    out[my:h - my, mx:w - mx] = 255
    out[m > 0] = 0
    d_out = float(diff[out > 0].mean()) if np.count_nonzero(out) else 0.0
    d_in = float(diff[m > 0].mean()) if np.count_nonzero(m) else 0.0
    return d_out, d_in


def circ_mean_180(angles):
    """ค่าเฉลี่ยมุมที่วน 180° (แกนยาว 10° กับ 170° อยู่ใกล้กัน)"""
    a = np.radians(np.asarray(angles, float) * 2.0)
    m = math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())) / 2.0 % 180.0
    r = float(np.hypot(np.sin(a).mean(), np.cos(a).mean()))
    sd = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(r, 1e-9))))) / 2.0
    return m, sd


def load_learned():
    try:
        with open(LEARNED_JSON, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"events": []}


def save_learned(data, pose=None, extra=None):
    """pose=None: ไม่เพิ่มรายการ แค่คำนวณสรุปใหม่ (เช่น เพิ่งวัด focal -> แปลงความสูงย้อนหลัง)"""
    if pose is not None:
        ev = dict(pose)
        ev.pop("box", None)
        ev["eq_px"] = eq_px(pose)
        ev.update(extra or {})
        data.setdefault("events", []).append(ev)
    data["events"] = data.get("events", [])[-KEEP_EVENTS:]
    evs = data["events"]
    if not evs:
        return None
    s = {"count": len(evs)}
    for k in ("u", "v", "long_px", "short_px"):
        vals = np.array([e[k] for e in evs], float)
        s[k] = {"median": float(np.median(vals)), "std": float(vals.std())}
    m, sd = circ_mean_180([e["angle_img"] for e in evs])
    s["angle_img"] = {"mean": m, "std": sd}
    # ความสูง: คำนวณใหม่ทุกครั้งจาก px ที่เก็บไว้ (วัด focal ทีหลังก็ได้ ค่าเก่าถูกแปลงให้ด้วย)
    ds = [dist_mm(e.get("eq_px") or math.sqrt(e["long_px"] * e["short_px"])) for e in evs]
    if FOCAL is not None and all(d is not None for d in ds):
        ds = np.array(ds, float)
        s["cam_to_block_top_mm"] = {"median": float(np.median(ds)), "std": float(ds.std())}
        s["cam_above_belt_mm"] = {"median": float(np.median(ds) + C.BLOCK_H_MM), "std": float(ds.std())}
        s["focal"] = {"f_px": FOCAL[0], "offset_mm": FOCAL[1]}
        for e, d in zip(evs, ds):
            e["cam_to_block_top_mm"] = float(d)
        if CAM_TO_TIP is not None:
            dep = CAM_TO_TIP - ds
            s["tip_depth_into_block_mm"] = {"median": float(np.median(dep)), "std": float(dep.std())}
            s["tip_above_belt_mm"] = {"median": float(C.BLOCK_H_MM - np.median(dep)), "std": float(dep.std())}
            s["cam_to_tip_mm"] = CAM_TO_TIP
            for e, d in zip(evs, dep):
                e["tip_depth_into_block_mm"] = float(d)
    data["summary"] = s
    data["note"] = ("ท่าบล็อกในภาพกล้องข้อมือ ณ จังหวะที่ jaw ปิด (เรียนจากการดูระบบอื่นหยิบ) — "
                    "u,v = จุดกลางบล็อก (px), long/short = ขนาด (px), angle_img = มุมแกนยาวในภาพ, "
                    "cam_to_block_top_mm = ระยะหน้าเลนส์ถึงผิวบนบล็อก (ต้องมี cam_focal.json จาก calib_focal.py), "
                    "cam_above_belt_mm = + ความสูงบล็อก (ถ้ากล้องมองลงตรงๆ)")
    tmp = LEARNED_JSON + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    os.replace(tmp, LEARNED_JSON)
    return s


def append_csv(row):
    if os.path.exists(OBS_CSV):                          # หัวตารางเปลี่ยน (เวอร์ชันเก่า) -> เก็บไฟล์เก่าไว้ เริ่มไฟล์ใหม่
        with open(OBS_CSV, encoding="utf-8") as f:
            head = f.readline().strip()
        if head != ",".join(row.keys()):
            os.replace(OBS_CSV, OBS_CSV.replace(".csv", time.strftime("_old_%Y%m%d_%H%M%S.csv")))
    new = not os.path.exists(OBS_CSV)
    with open(OBS_CSV, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)


def draw_pose(vis, pose, col, label=None):
    cv2.polylines(vis, [np.array(pose["box"], np.int32)], True, col, 2)
    cv2.drawMarker(vis, (int(pose["u"]), int(pose["v"])), col, cv2.MARKER_CROSS, 24, 2)
    if label:
        cv2.putText(vis, label, (int(pose["u"]) + 12, int(pose["v"]) - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)


def learned_box(summary):
    """กรอบ "เป้าหนีบที่เรียนได้" สำหรับวาดทับภาพ"""
    u, v = summary["u"]["median"], summary["v"]["median"]
    L, S = summary["long_px"]["median"], summary["short_px"]["median"]
    a = summary["angle_img"]["mean"]
    return cv2.boxPoints(((u, v), (L, S), a)).astype(np.int32)


def _panel_text(vis, lines, x, y, scale=0.75):
    """ข้อความหลายบรรทัดบนพื้นดำโปร่ง (อ่านง่ายบนภาพสว่าง)"""
    th = int(34 * scale / 0.75)
    w = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)[0][0] for t, _ in lines) + 20
    h = th * len(lines) + 12
    x = min(x, vis.shape[1] - w - 4)
    roi = vis[y:y + h, x:x + w]
    roi[:] = (roi * 0.35).astype(roi.dtype)
    for i, (t, col) in enumerate(lines):
        cv2.putText(vis, t, (x + 10, y + th * (i + 1) - 6), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 2)


def draw_height_panel(vis, pose, summary):
    """กล่องความสูงมุมขวาบน: ตอนนี้ (จากขนาดบล็อกในภาพ) + ค่าที่เรียนได้ตอนหนีบ"""
    lines = [("HEIGHT", (0, 255, 255))]
    if FOCAL is None:
        lines.append(("no cam_focal.json -> run calib_focal.py", (0, 0, 255)))
    elif pose is None:
        lines.append(("now: no block", (180, 180, 180)))
    else:
        d = dist_mm(eq_px(pose))
        lines.append((f"cam -> block top : {d:5.0f} mm", (0, 255, 0)))
        dep, above = tip_info(d)
        if dep is not None:
            lines.append((f"jaw tip -> block : {-dep:5.0f} mm" + ("  (IN block)" if dep > 0 else ""), (0, 255, 0)))
            lines.append((f"jaw tip -> belt  : {above:5.0f} mm", (0, 255, 0)))
        else:
            lines.append((f"cam above belt   : {d + C.BLOCK_H_MM:5.0f} mm", (0, 255, 0)))
    if summary and "cam_to_block_top_mm" in summary:
        n = summary["count"]
        if "tip_above_belt_mm" in summary:
            lines.append((f"learned grip ({n}x): tip {summary['tip_above_belt_mm']['median']:.0f}"
                          f"+-{summary['tip_above_belt_mm']['std']:.0f} mm above belt", (255, 0, 255)))
        else:
            lines.append((f"learned grip ({n}x): cam {summary['cam_to_block_top_mm']['median']:.0f} mm from block", (255, 0, 255)))
    _panel_text(vis, lines, vis.shape[1] - 470, 10)


def save_img(path, g, pose, col, text):
    vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if pose is not None:
        draw_pose(vis, pose, col)
    cv2.putText(vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.imwrite(path, vis)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nogui", action="store_true", help="ไม่เปิดหน้าต่างภาพ")
    ap.add_argument("--trace", action="store_true", help="บันทึก d_out/d_in/ท่าบล็อก ทุก ~0.1 s ลง grip_trace.csv")
    args = ap.parse_args()
    gui = not args.nogui

    os.makedirs(OBS_DIR, exist_ok=True)
    H = D.load_homography()
    cap = camera.open_camera()
    learned = load_learned()
    summary = learned.get("summary")
    if FOCAL is not None and learned.get("events") and (
            "cam_to_block_top_mm" not in (summary or {}) or (CAM_TO_TIP is not None and "tip_depth_into_block_mm" not in summary)):
        summary = save_learned(learned)                 # เพิ่งวัด focal: แปลงความสูงของการหนีบที่เก็บไว้ก่อนหน้าให้
    if FOCAL is None:
        print("ยังไม่รู้ focal length -> บันทึกแค่ขนาด px (รัน calib_focal.py เมื่อไรก็ได้ ความสูงจะถูกคำนวณย้อนหลัง)")
    else:
        print(f"focal = {FOCAL[0]:.0f} px (offset {FOCAL[1]:+.1f} mm) -> คำนวณความสูงจากขนาดบล็อกได้")
    print("กล้องอย่างเดียว (ไม่เปิดพอร์ต Mega) — ปล่อยให้ระบบของเพื่อนหยิบบล็อกได้เลย | Esc = ออก")
    if summary:
        print(f"เรียนมาแล้ว {summary['count']} ครั้ง: กลางบล็อก u={summary['u']['median']:.0f} v={summary['v']['median']:.0f} px, "
              f"มุม {summary['angle_img']['mean']:.0f}°")

    hist = collections.deque()                # (t, gray, pose, d_out)
    ev_hist = collections.deque()             # (t, หลักฐานถือบล็อก True/False)
    prev_s = None
    last_pose, last_pose_t = None, 0.0
    state = "WATCH"
    held_since, release_since, cooldown_until = None, None, 0.0
    grip = None                                # (t, gray, pose) ของจังหวะหนีบครั้งล่าสุด
    n_events = len(learned.get("events", []))
    last_msg = ""
    trace_f = trace_w = None
    if args.trace:
        newt = not os.path.exists(TRACE_CSV)
        trace_f = open(TRACE_CSV, "a", newline="", encoding="utf-8")
        trace_w = csv.writer(trace_f)
        if newt:
            trace_w.writerow(["t", "state", "d_out", "d_in", "seen", "u", "v", "long_px", "short_px", "angle_img"])
    last_trace = 0.0
    win = "grip observe (Esc = quit)"
    try:
        while True:
            D.watchdog_kick()
            g = camera.grab(cap)
            if g is None:
                if gui and cv2.waitKey(10) & 0xFF == 27:
                    break
                continue
            now = time.perf_counter()
            small = cv2.GaussianBlur(cv2.resize(g, (g.shape[1] // MOTION_SCALE, g.shape[0] // MOTION_SCALE),
                                                interpolation=cv2.INTER_AREA), (3, 3), 0)
            pose = find_any_block(g, H)
            if pose is not None:
                last_pose, last_pose_t = pose, now
            # กรอบที่ใช้วัด d_in: ท่าที่เห็นล่าสุด (ตอนถูกหนีบ jaw อาจบังจนหาไม่เจอ แต่บล็อกยังอยู่ที่เดิมในภาพ)
            ref = last_pose if (last_pose is not None and now - last_pose_t < 3.0) else None
            d_out, d_in = motion(prev_s, small, ref["box"] if ref else None)
            prev_s = small

            hist.append((now, g, pose, d_out))
            while hist and now - hist[0][0] > HISTORY_S:
                hist.popleft()

            evidence = ref is not None and d_out > MOVE_THR and d_in < d_out * LOCK_RATIO
            ev_hist.append((now, evidence))
            while ev_hist and now - ev_hist[0][0] > HELD_MIN_S:
                ev_hist.popleft()
            span = now - ev_hist[0][0] if ev_hist else 0.0
            frac = sum(e for _, e in ev_hist) / max(1, len(ev_hist))

            if state == "WATCH" and now >= cooldown_until and span >= HELD_MIN_S * 0.9 and frac >= HELD_WINDOW_FRAC:
                # ---- เริ่มถือบล็อก: ย้อนหาจังหวะหนีบ = เฟรมล่าสุดก่อนเริ่มขยับ ที่กล้องนิ่งและเห็นบล็อก ----
                onset = ev_hist[0][0]
                before = [h for h in hist if h[0] < onset and h[2] is not None]
                still = [h for h in before if h[3] < STILL_THR]
                pick = still[-1] if still else (before[-1] if before else None)
                state, held_since, release_since = "HELD", now, None
                ok_grip, why = plausible_grip(pick[2]) if pick is not None else (True, "")
                if pick is not None and not ok_grip:
                    stamp = time.strftime("%Y%m%d_%H%M%S")
                    rdir = os.path.join(OBS_DIR, "rejected")
                    os.makedirs(rdir, exist_ok=True)
                    save_img(os.path.join(rdir, f"{stamp}_grip.png"), pick[1], pick[2], (0, 0, 255), "REJECTED: " +
                             "".join(ch for ch in why if ord(ch) < 128))
                    p = pick[2]
                    append_csv({"time": stamp, "event": "rejected", "n": "", "u": round(p["u"], 1), "v": round(p["v"], 1),
                                "long_px": round(p["long_px"], 1), "short_px": round(p["short_px"], 1),
                                "angle_img": round(p["angle_img"], 1), "gray": p["gray"], "tone": p["tone"],
                                "still_before": bool(still), "lift_after_s": round(onset - pick[0], 2), "held_s": "",
                                "eq_px": round(eq_px(p), 1),
                                "cam_to_block_top_mm": "" if dist_mm(eq_px(p)) is None else round(dist_mm(eq_px(p)), 1),
                                "note": why})
                    print(f"ไม่นับ (ไม่ใช่การหนีบจริง): {why} — ภาพที่ grip_obs/rejected/")
                    last_msg = "rejected: " + why
                elif pick is not None:
                    grip = (pick[0], pick[1], pick[2])
                    n_events += 1
                    stamp = time.strftime("%Y%m%d_%H%M%S")
                    base = os.path.join(OBS_DIR, f"{stamp}_{n_events:03d}")
                    first = next((h for h in hist if h[2] is not None), None)
                    if first is not None:
                        save_img(base + "_approach.png", first[1], first[2], (255, 200, 0), "approach (first seen)")
                    _dg = dist_mm(eq_px(pick[2]))
                    _tg = tip_info(_dg)[1]
                    save_img(base + "_grip.png", pick[1], pick[2], (0, 255, 0), "GRIP moment (jaw closing)" +
                             (f"  cam->block {_dg:.0f} mm" if _dg else "") + (f"  tip->belt {_tg:.0f} mm" if _tg is not None else ""))
                    save_img(base + "_held.png", g, ref, (0, 140, 255), "HELD (lifting)")
                    p = pick[2]
                    summary = save_learned(learned, p, {"time": stamp, "still_before": bool(still),
                                                        "lift_after_s": round(onset - pick[0], 2)})
                    append_csv({"time": stamp, "event": "grip", "n": n_events, "u": round(p["u"], 1), "v": round(p["v"], 1),
                                "long_px": round(p["long_px"], 1), "short_px": round(p["short_px"], 1),
                                "angle_img": round(p["angle_img"], 1), "gray": p["gray"], "tone": p["tone"],
                                "still_before": bool(still), "lift_after_s": round(onset - pick[0], 2), "held_s": "",
                                "eq_px": round(eq_px(p), 1),
                                "cam_to_block_top_mm": "" if dist_mm(eq_px(p)) is None else round(dist_mm(eq_px(p)), 1),
                                "note": ""})
                    _d = dist_mm(eq_px(p))
                    last_msg = (f"#{n_events} หนีบที่ u={p['u']:.0f} v={p['v']:.0f} px มุม {p['angle_img']:.0f}° "
                                f"ขนาด {p['long_px']:.0f}x{p['short_px']:.0f} px" +
                                (f" | กล้อง->ผิวบล็อก {_d:.0f} mm" if _d else " | ยังไม่รู้ความสูง (รัน calib_focal.py)") +
                                (f" | ปลาย jaw ลึกจากผิวบน {tip_info(_d)[0]:.0f} mm (สูงจากสายพาน {tip_info(_d)[1]:.0f} mm)"
                                 if tip_info(_d)[0] is not None else (" | ยังไม่ได้วัดเลนส์->ปลาย jaw (calib_focal.py ปุ่ม t)" if _d else "")))
                    print(last_msg + f"  (เฉลี่ย {summary['count']} ครั้ง: u={summary['u']['median']:.0f} "
                                     f"v={summary['v']['median']:.0f} มุม {summary['angle_img']['mean']:.0f}°)")
                else:
                    print("เห็นว่าถือบล็อกอยู่ แต่ไม่มีภาพบล็อกก่อนหนีบในประวัติ — ไม่บันทึกครั้งนี้")
            elif state == "HELD" and (ref is None or now - held_since > 60.0):
                # มองไม่เห็นบล็อกนานแล้ว (หรือค้าง HELD นานผิดปกติ) -> กลับไปเฝ้าดูใหม่ ไม่ค้าง
                if ref is None and now - last_pose_t > HELD_LOST_S or now - held_since > 60.0:
                    print("   ไม่เห็นบล็อกแล้ว -> กลับ WATCH")
                    state, cooldown_until = "WATCH", now + COOLDOWN_S
                    last_pose, ev_hist = None, collections.deque()
            elif state == "HELD":
                # ปล่อยแล้ว = กรอบบล็อกในภาพเปลี่ยนต่อเนื่อง (บล็อกหลุด/ถูกวาง แล้วกล้องเห็นฉากอื่นตรงนั้น)
                if d_in > MOVE_THR and d_in >= d_out * LOCK_RATIO:
                    release_since = release_since or now
                    if now - release_since >= RELEASE_S:
                        held = now - held_since
                        print(f"   ปล่อยบล็อกแล้ว (ถืออยู่ {held:.1f} s)")
                        append_csv({"time": time.strftime("%Y%m%d_%H%M%S"), "event": "release", "n": n_events,
                                    "u": "", "v": "", "long_px": "", "short_px": "", "angle_img": "", "gray": "", "tone": "",
                                    "still_before": "", "lift_after_s": "", "held_s": round(held, 2),
                                    "eq_px": "", "cam_to_block_top_mm": "", "note": ""})
                        state, cooldown_until = "WATCH", now + COOLDOWN_S
                        last_pose, ev_hist = None, collections.deque()
                else:
                    release_since = None

            if trace_w is not None and now - last_trace >= 0.1:
                last_trace = now
                trace_w.writerow([round(now, 3), state, round(d_out, 2), round(d_in, 2), pose is not None] +
                                 ([round(pose[k], 1) for k in ("u", "v", "long_px", "short_px", "angle_img")] if pose else [""] * 5))

            if gui:
                vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
                if summary:
                    cv2.polylines(vis, [learned_box(summary)], True, (255, 0, 255), 1)       # ม่วง = เป้าหนีบที่เรียนได้
                if pose is not None:
                    _d = dist_mm(eq_px(pose))
                    _t = tip_info(_d)[0]
                    draw_pose(vis, pose, (0, 255, 0), f"{pose['angle_img']:.0f}deg" + (f"  cam {_d:.0f}mm" if _d else "") +
                              (f"  tip {'+' if _t >= 0 else ''}{_t:.0f}mm" if _t is not None else ""))
                elif ref is not None and state == "HELD":
                    draw_pose(vis, ref, (0, 140, 255))
                col = (0, 140, 255) if state == "HELD" else (255, 255, 0)
                cv2.putText(vis, f"{state}  grips learned: {n_events}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
                cv2.putText(vis, f"d_out {d_out:4.1f} (move>{MOVE_THR:.0f})  d_in {d_in:4.1f}  lock {frac:.0%}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                if grip is not None:
                    cv2.putText(vis, "last grip: " + "".join(ch for ch in last_msg if ord(ch) < 128), (10, 88),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                cv2.putText(vis, "camera only - arm port not used", (10, vis.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                draw_height_panel(vis, pose if pose is not None else (ref if state == "HELD" else None), summary)
                cv2.imshow(win, vis)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        D.watchdog_off()
        if trace_f is not None:
            trace_f.close()
        cap.release()
        cv2.destroyAllWindows()
        s = load_learned().get("summary")
        if s:
            print(f"\nเรียนได้ทั้งหมด {s['count']} ครั้ง -> {LEARNED_JSON}")
            print(f"  กลางบล็อกตอนหนีบ: u={s['u']['median']:.0f}±{s['u']['std']:.0f}  v={s['v']['median']:.0f}±{s['v']['std']:.0f} px")
            print(f"  ขนาดในภาพ: {s['long_px']['median']:.0f} x {s['short_px']['median']:.0f} px   มุม {s['angle_img']['mean']:.0f}±{s['angle_img']['std']:.0f}°")
            if "cam_to_block_top_mm" in s:
                print(f"  ความสูงตอนหนีบ: กล้อง->ผิวบนบล็อก {s['cam_to_block_top_mm']['median']:.0f}±{s['cam_to_block_top_mm']['std']:.0f} mm "
                      f"(กล้องสูงจากสายพาน ~{s['cam_above_belt_mm']['median']:.0f} mm)")
                if "tip_depth_into_block_mm" in s:
                    print(f"  ปลาย jaw ตอนหนีบ: ลึกจากผิวบนบล็อก {s['tip_depth_into_block_mm']['median']:.0f}±{s['tip_depth_into_block_mm']['std']:.0f} mm "
                          f"= สูงจากสายพาน {s['tip_above_belt_mm']['median']:.0f} mm")
                else:
                    print("  ปลาย jaw: ยังไม่ได้วัดระยะเลนส์->ปลาย jaw -> calib_focal.py พิมพ์ mm แล้วกด t")
            else:
                print("  ความสูง: ยังไม่มี cam_focal.json -> รัน python calib_focal.py (ค่าที่เก็บไว้จะถูกแปลงย้อนหลังให้)")


if __name__ == "__main__":
    main()
