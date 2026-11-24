"""visual_grab.py — ตามบล็อกด้วยกล้องข้อมือ -> บล็อกนิ่ง 2 วินาที -> ลงไปหยิบ (ใช้ "ขนาดบล็อกในภาพ" บอกความสูง)

ไม่ใช้ IK/TABLE_PLANE/homography ในการหาตำแหน่งหยิบ (ค่าพวกนั้นยังไม่แม่น) — ใช้ภาพปิดวงควบคุมตลอดทาง:
  1) FOLLOW   : ที่ความสูงท่า scan ขยับปลายแขนให้บล็อกอยู่ที่ "จุดเป้าในภาพ"
  2) นิ่ง HOLD_S วินาที (บล็อกอยู่ที่เป้า + ไม่ขยับ) -> DESCEND
  3) DESCEND  : ค่อยๆ ลดความสูง พร้อมเลื่อนให้บล็อกอยู่ที่เป้า + หมุน roll ให้มุมบล็อกตรงกับตอนสอน + ปรับก้ม (pitch) ไปเท่าตอนสอน
                จนบล็อกในภาพ "ใหญ่เท่าตอนสอน" = ถึงจุดก่อนหยิบ (pre-grasp)
  4) FINAL    : ลงตรงๆ (แกน z ของโลก) เท่ากับระยะที่สอนไว้ -> ปิด gripper -> ยกขึ้น
จุดเป้าในภาพ / ขนาดบล็อก / ระยะลงสุดท้าย ได้จากการ "สอนครั้งเดียว" (--teach):
  ประคอง jaw (ที่อ้าอยู่) ครอบบล็อกที่ความสูงที่อยากหนีบ -> กด Enter -> โปรแกรมยกขึ้นตรงๆ แล้วจดว่าบล็อกอยู่ตรงไหน/ใหญ่เท่าไรในภาพ

ใช้:
  python visual_grab.py --teach        สอนครั้งเดียว (ต้องประคองแขน — torque ข้อ 1-5 จะถูกปิด)
  python visual_grab.py --dry          ไม่ขยับแขน: ดูการตรวจจับ/เป้า/ตัวนับ 2 วินาที
  python visual_grab.py                ตาม + หยิบจริง (ถึงจุดก่อนหยิบแล้วรอกด g ก่อนลง)
  python visual_grab.py --auto         ไม่ต้องกด g (หยิบเองทั้งหมด) — ใช้หลังลองแบบกด g จนมั่นใจแล้ว
ปุ่ม (คลิกหน้าต่างภาพก่อน): SPACE = ลงหยิบเดี๋ยวนี้ (ไม่ต้องรอนิ่ง ไม่ต้องกด g) | Esc = หยุด (แขนค้างที่เดิม)
                            g = ลงหยิบ (ตอนรอที่ pre-grasp) | o = เปิด gripper | r = กลับท่า scan
  python visual_grab.py --hold 0     ตามบล็อกอย่างเดียว ลงหยิบเมื่อกด SPACE เท่านั้น
ต้องปิดโปรแกรมอื่นที่ใช้พอร์ต Mega ก่อน (autoen / PRISM / Arduino Serial Monitor)
"""
import argparse
import json
import math
import os
import time

import cv2
import numpy as np

import camera
import config as C
import detect_block as D
import kinematics as K
from grip_observe import FOCAL, dist_mm, eq_px, find_any_block

HERE = os.path.dirname(os.path.abspath(__file__))
TEACH_FILE = os.path.join(HERE, "grasp_teach.json")
WIN = "visual grab (Esc = stop, g = grip, o = open, r = scan)"

# ---- พารามิเตอร์ ----
HOLD_S = 2.0              # บล็อกต้องอยู่ที่เป้าและนิ่งนานเท่านี้ก่อนลง
CENTER_PX = 25            # "อยู่ที่เป้า" = ห่างจุดเป้าไม่เกินนี้ (px)
JITTER_PX = 16            # ระหว่างนับเวลา บล็อกขยับได้ไม่เกินนี้ (px) ไม่งั้นนับใหม่
PRE_TOL_PX = 35           # ถึง pre-grasp: ห่างเป้าไม่เกินนี้
PRE_SIZE_TOL = 0.08       # ถึง pre-grasp: ขนาดบล็อกต่างจากตอนสอนไม่เกินสัดส่วนนี้ (ทั้งใหญ่ไปและเล็กไป)
PRE_SETTLE_S = 0.3        # ทุกเงื่อนไข pre-grasp ต้องผ่านต่อเนื่องเท่านี้
KZ = 0.5                  # ปรับความสูงตามขนาด: ระยะกล้องคลาด 10 mm -> สั่งขยับ 5 mm (ต่อรอบ จำกัดด้วย DESCEND_MM_S)
PITCH_RATE_DEG = 0.4      # เปลี่ยนมุมก้มได้ไม่เกินนี้ต่อรอบ (ก้ม 1° = ภาพเลื่อน ~16 px)
EDGE_PAUSE_PX = 60        # บล็อกเข้าใกล้ขอบภาพเท่านี้ -> หยุดลง/หยุดเปลี่ยนก้ม ชั่วคราว แก้แนวราบอย่างเดียว
FINAL_Z_TOL_MM = 5.0      # ก่อนลงสุดท้าย: ท่าหนีบที่จะไปต้องไม่ต่ำกว่าท่าหนีบที่สอนเกินนี้ (กันลงลึกจนชนสายพาน)
FINAL_PITCH_TOL_DEG = 6.0
PRE_PITCH_TOL_DEG = 6.0
PRE_TIMEOUT_S = 6.0       # ลงอยู่นานเกินนี้ แต่ "ใกล้พอ" (ขนาด ±PRE_LOOSE_SIZE, ห่าง <= PRE_LOOSE_PX) -> ไปหนีบเลย ไม่ค้าง
PRE_LOOSE_SIZE = 0.15
PRE_LOOSE_PX = 60
EXTRA_DEPTH_MM = 5.0      # ลงลึกกว่าท่าที่สอนอีกเท่านี้ก่อนหนีบ (--deeper)
LEAD_MAX_TICKS = 60       # เป้าสะสมนำตำแหน่งจริงได้ไม่เกินนี้ (servo แบกน้ำหนักค้างห่างเป้า ~30-40 tick)
STUCK_TICKS = 2
PRE_ANGLE_TOL_DEG = 6.0
KP_XY = 0.3               # สั่งแก้ 30% ของคลาดแนวราบต่อรอบ (สูงกว่านี้แกว่งเพราะภาพหน่วงกว่าแขน)
DEADBAND_PX = 12          # คลาดน้อยกว่านี้ไม่ขยับ (ให้แขนหยุดนิ่งจริงตอนรอ 2 วินาที)
KP_ROLL = 0.5
DESCEND_MM_S = 25.0       # ความเร็วลง (ช่วง DESCEND)
EXTRA_MM_S = 10.0         # ความเร็วช่วงลงเพิ่ม (--deeper)
FINAL_MM_S = 20.0         # ความเร็วลงช่วงสุดท้าย (มองไม่เห็นแล้ว)
LIFT_AFTER_MM = 60.0      # หนีบแล้วยกขึ้นเท่านี้
MAX_DESCENT_MM = 180.0    # ลงต่ำกว่าท่า scan ได้ไม่เกินนี้ (กันลงไม่หยุด)
PRE_Z_MARGIN_MM = 30.0    # และห้ามต่ำกว่าความสูงจุดก่อนหยิบที่สอนไว้เกินนี้ (กันชนสายพานถ้าวัดขนาดบล็อกพลาด)
STEP_XY_MM = 12.0         # ต่อรอบ เลื่อนแนวราบไม่เกินนี้ (move_cartesian)
FOLLOW_STEP_MM = 4.0      # ต่อรอบตอนตาม/ลง (~50 mm/s ที่ 12 Hz) — เร็วกว่านี้แล้วภาพตามไม่ทัน แกว่งจนบล็อกหลุดภาพ
SEARCH_AFTER_S = 1.0      # บล็อกหายนานกว่านี้ถึงจะเริ่มกลับท่า scan (เฟรมพลาด 1-2 เฟรมไม่นับ)
RETURN_FRAC = 0.04        # บล็อกหายตอน FOLLOW: ค่อยๆ กลับท่า scan ทีละเท่านี้ของระยะต่อรอบ (ไปหาที่เคยเห็น)
STEP_Z_MM = 8.0
STEP_PITCH_DEG = 3.0
MAX_STEP_TICKS = 40
SEND_HZ = 12.0
MIN_SPEED, MAX_SPEED = C.MEGA_MIN_SPEED, 300
LOST_S = 0.7              # ตอน DESCEND มองไม่เห็นบล็อกนานเท่านี้ -> หยุดค้าง (รอเห็นใหม่)
FB_LOST_MAX_S = 2.0
FOLLOW_RANGE = {"shoulder_pan": 350, "shoulder_lift": 120, "elbow_flex": 120, "wrist_flex": 120}   # ช่วง FOLLOW จากท่า scan
TEACH_LIFT_MM = 50.0      # ตอนสอน: ยกจากท่าหนีบขึ้นเท่านี้ (ถ้ายังเห็นบล็อกไม่ครบ จะยกเพิ่มทีละ 20 mm)
TEACH_LIFT_MAX_MM = 150.0
TEACH_MARGIN_PX = 110     # ตอนสอน: ยกจนบล็อกห่างขอบภาพอย่างน้อยเท่านี้ (เป้าใกล้ขอบเกินไป = ตอนใช้งานบล็อกหลุดภาพง่าย)
ROLL_PROBE_TICKS = 80
JOINTS4 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]


# ------------------------------------------------------------ เรขาคณิต
def wrap180(a):
    """มุมแกนยาว (วน 180°) -> ส่วนต่างในช่วง [-90, 90)"""
    return (a + 90.0) % 180.0 - 90.0


def state4(j):
    """(x, y, z, pitch) ของจุดกึ่งกลาง jaw"""
    x, y, z, p, _ = K.fk(j, "grasp")
    return np.array([x, y, z, p])


def rot(j):
    return K.gripper_frame(K.motor_to_q(j))[:3, :3]


def jac4(j, d=8):
    base = state4(j)
    J = np.zeros((4, 4))
    for k, n in enumerate(JOINTS4):
        m = dict(j)
        m[n] += d
        J[:, k] = (state4(m) - base) / d
    return J


def img_map_at_scan(H, scan, center):
    """ทิศในเฟรม gripper ของ "1 px แนว u" และ "1 px แนว v" (mm ที่ระยะท่า scan) — จาก homography ของท่า scan
    ใช้แปลงคลาดในภาพ -> การเลื่อนในโลก โดยหมุนตามท่าปัจจุบันของ gripper (pan/roll/pitch เปลี่ยนก็ยังถูกทิศ)"""
    c = np.array(center, np.float32)
    pts = np.array([[c, c + (1, 0), c + (0, 1)]], np.float32)
    w = cv2.perspectiveTransform(pts, H)[0].astype(float)
    Rs = rot(scan)
    gu = Rs.T @ np.array([w[1, 0] - w[0, 0], w[1, 1] - w[0, 1], 0.0])
    gv = Rs.T @ np.array([w[2, 0] - w[0, 0], w[2, 1] - w[0, 1], 0.0])
    return gu, gv


def pixel_err_to_world(e_px, j, gu, gv, eq_ref, eq_now):
    """คลาดในภาพ (px) -> การเลื่อนแนวราบในโลก (mm) ที่ทำให้บล็อกเข้าหาเป้า"""
    s = eq_ref / max(eq_now, 1e-6)                          # ใกล้ขึ้น = 1 px เท่ากับ mm น้อยลง
    g = (e_px[0] * gu + e_px[1] * gv) * s
    w = rot(j) @ g
    return w[:2]


def target_px(teach, eq_now, shape):
    """จุดเป้าในภาพ: ที่ขนาดตอนสอนคือจุดที่สอน, ไกลกว่านั้นค่อยๆ เข้าหากลางภาพ (พารัลแลกซ์ของกล้องที่เยื้องจาก jaw)"""
    c0 = np.array([shape[1] / 2.0, shape[0] / 2.0])
    t = np.array([teach["u"], teach["v"]])
    k = min(1.0, eq_now / teach["eq_px"]) if eq_now else 0.0
    return c0 + (t - c0) * k


# ------------------------------------------------------------ สั่งแขน
def read_joints_nb(arm, max_age=1.0):
    out = {}
    for j in K.JOINTS5:
        p = arm.link.servo_pos(arm.ids[j], max_age)
        if p is None:
            return None
        out[j] = p
    return out


def plan_step(j, goal4, roll_goal=None, ranges=None, ref=None, pitch_weight=1.0, step_xy=None):
    """คำสั่งหนึ่งรอบให้ (x,y,z,pitch) เข้าหา goal4 (+ roll เข้าหา roll_goal) — คืน {joint: (tick, speed)}
    pitch_weight < 1: ยอมให้มุมก้มเพี้ยนได้ (ใช้ตอนยก/ลงที่ข้อมือใกล้ขอบ)
    ข้อที่จะชนขอบ -> "ล็อกข้อนั้น" แล้วแก้ใหม่ด้วยข้อที่เหลือ (ไม่ใช่หยุดทั้งแขน)"""
    cur = state4(j)
    e = np.array(goal4, float) - cur
    n = np.linalg.norm(e[:2])
    lim = STEP_XY_MM if step_xy is None else step_xy
    if n > lim:
        e[:2] *= lim / n
    e[2] = float(np.clip(e[2], -STEP_Z_MM, STEP_Z_MM))
    e[3] = float(np.clip(e[3], -STEP_PITCH_DEG, STEP_PITCH_DEG))
    Wt = np.diag([1.0, 1.0, 1.0, pitch_weight])
    J = jac4(j)

    def bounds(n_):
        lo, hi = C.JOINT_LIMITS[n_][0] + C.LIMIT_MARGIN, C.JOINT_LIMITS[n_][1] - C.LIMIT_MARGIN
        if ranges and n_ in ranges and ref is not None:
            lo, hi = max(lo, ref[n_] - ranges[n_]), min(hi, ref[n_] + ranges[n_])
        return lo, hi

    free = list(range(4))
    dq = np.zeros(4)
    for _ in range(4):
        if not free:
            break
        sol = np.linalg.lstsq(Wt @ J[:, free], Wt @ e, rcond=1e-3)[0]
        dq = np.zeros(4)
        dq[free] = sol
        big = float(np.max(np.abs(dq)))
        if big > MAX_STEP_TICKS:
            dq *= MAX_STEP_TICKS / big
        hit = []
        for k_ in free:
            lo, hi = bounds(JOINTS4[k_])
            g = j[JOINTS4[k_]] + dq[k_]
            if (dq[k_] > 0 and g > hi) or (dq[k_] < 0 and g < lo):
                hit.append(k_)
        if not hit:
            break
        free = [k_ for k_ in free if k_ not in hit]
    goals = {}
    for k_, n_ in enumerate(JOINTS4):
        lo, hi = bounds(n_)
        goals[n_] = float(np.clip(j[n_] + dq[k_], min(lo, j[n_]), max(hi, j[n_])))
    if roll_goal is not None:
        lo, hi = bounds("wrist_roll")
        goals["wrist_roll"] = float(np.clip(j["wrist_roll"] + np.clip(roll_goal - j["wrist_roll"], -MAX_STEP_TICKS, MAX_STEP_TICKS),
                                            min(lo, j["wrist_roll"]), max(hi, j["wrist_roll"])))
    cmds = {}
    for n_, g in goals.items():
        if abs(g - j[n_]) < 1:
            continue
        sp = min(max(abs(g - j[n_]) * SEND_HZ * 1.5, MIN_SPEED), MAX_SPEED)
        cmds[n_] = (int(round(g)), int(sp))
    return cmds


def integrate(cmds, j, state):
    """ชนะการยุบของ servo ที่แบกน้ำหนัก โดยไม่แกว่ง (แบบเดียวกับ follow_block):
    ข้อที่ "ติด" (ขยับ <= STUCK_TICKS จากรอบก่อน) และยังต้องไปทางเดิม -> ดันเป้าเพิ่มสะสม; กลับทิศ -> ทิ้งที่สะสม"""
    out = {}
    for n_, (g, sp) in cmds.items():
        d = g - j[n_]
        prev = state.get(("pos", n_))
        lead = state.get(n_, j[n_]) - j[n_]
        if d * lead < 0:
            lead = 0.0
        if prev is not None and abs(j[n_] - prev) <= STUCK_TICKS and d:
            lead += d
        g2 = j[n_] + (d if abs(d) > abs(lead) else lead) if lead else j[n_] + d
        g2 = min(max(g2, j[n_] - LEAD_MAX_TICKS), j[n_] + LEAD_MAX_TICKS)
        lo, hi = C.JOINT_LIMITS[n_][0] + C.LIMIT_MARGIN, C.JOINT_LIMITS[n_][1] - C.LIMIT_MARGIN
        g2 = min(max(g2, lo), hi)
        state[n_], state[("pos", n_)] = g2, j[n_]
        out[n_] = (int(round(g2)), sp)
    return out


def send(arm, cmds):
    for n_, (g, sp) in cmds.items():
        arm._check_limits({n_: g})
        arm._moved.add(arm.ids[n_])
        arm._p(n_, g, sp)


def move_cartesian(arm, goal4, timeout=8.0, tol_mm=1.5, ui=None, speed_mm_s=None, pitch_weight=1.0):
    """เลื่อนจุดกึ่งกลาง jaw ไป goal4 = (x,y,z,pitch) แบบค่อยๆ (เส้นตรงในโลกโดยประมาณ) — คืน True ถ้าถึง
    ui(): เรียกทุกรอบ (ปั๊มหน้าต่าง) ถ้าคืน False = ผู้ใช้สั่งหยุด"""
    j = read_joints_nb(arm)
    if j is None:
        raise RuntimeError("ไม่มีค่าตำแหน่งแขน")
    start = state4(j)
    goal4 = np.array(goal4, float)
    t0 = time.perf_counter()
    dist = float(np.linalg.norm(goal4[:3] - start[:3]))
    dur = dist / speed_mm_s if speed_mm_s else 0.0
    last = 0.0
    while time.perf_counter() - t0 < timeout + dur:
        if ui is not None and ui() is False:
            return False
        now = time.perf_counter()
        if now - last < 1.0 / SEND_HZ:
            time.sleep(0.005)
            continue
        last = now
        j = read_joints_nb(arm)
        if j is None:
            continue
        a = min(1.0, (now - t0) / dur) if dur > 0 else 1.0     # เป้าเคลื่อนตามเวลา (ความเร็วคงที่) ไม่กระโดด
        sub = start + (goal4 - start) * a
        cur = state4(j)
        if a >= 1.0 and np.linalg.norm(goal4[:3] - cur[:3]) < tol_mm and (pitch_weight < 1.0 or abs(goal4[3] - cur[3]) < 2.0):
            return True
        send(arm, plan_step(j, sub, pitch_weight=pitch_weight))
    return False


# ------------------------------------------------------------ ภาพ
def detect(g, H, prefer=None):
    p = find_any_block(g, H, prefer_px=prefer)
    if p is not None:
        p["eq"] = eq_px(p)
    return p


def stable_pose(cap, H, n=10, timeout=3.0, ui=None):
    """เฉลี่ยท่าบล็อกหลายเฟรม (แขนนิ่ง) -> dict หรือ None"""
    got, t0 = [], time.perf_counter()
    while len(got) < n and time.perf_counter() - t0 < timeout:
        g = camera.grab(cap)
        if ui is not None:
            ui(g)
        if g is None:
            continue
        p = detect(g, H, prefer=(got[-1]["u"], got[-1]["v"]) if got else None)
        if p is not None:
            got.append(p)
    if len(got) < max(3, n // 2):
        return None
    out = {k: float(np.median([p[k] for p in got])) for k in ("u", "v", "eq", "long_px", "short_px")}
    a0 = got[0]["angle_img"]
    out["angle_img"] = (a0 + float(np.median([wrap180(p["angle_img"] - a0) for p in got]))) % 180.0
    out["box"] = got[-1]["box"]
    return out


def draw(vis, pose, tgt, lines, col=(0, 255, 255)):
    if tgt is not None:
        cv2.drawMarker(vis, (int(tgt[0]), int(tgt[1])), (255, 0, 255), cv2.MARKER_CROSS, 40, 2)
        cv2.circle(vis, (int(tgt[0]), int(tgt[1])), CENTER_PX, (255, 0, 255), 1)
    if pose is not None:
        cv2.polylines(vis, [np.array(pose["box"], np.int32)], True, (0, 255, 0), 2)
        cv2.drawMarker(vis, (int(pose["u"]), int(pose["v"])), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
    for i, t in enumerate(lines):
        cv2.putText(vis, t, (10, 32 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col if i == 0 else (0, 255, 0), 2)


# ------------------------------------------------------------ สอน
def teach(arm, cap, H, wait_user=None):
    """สอนครั้งเดียว: ประคอง jaw ครอบบล็อก -> Enter -> ยกขึ้นตรงๆ -> จดภาพบล็อก + ทิศหมุน roll"""
    state = {"key": -1}

    def ui(g=None, text="TEACH"):
        if g is not None:
            vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            p = detect(g, H)
            draw(vis, p, None, [text] + ([f"block {p['eq']:.0f} px  angle {p['angle_img']:.0f}"] if p else ["no block"]))
            cv2.imshow(WIN, vis)
        state["key"] = cv2.waitKey(1) & 0xFF
        return state["key"] != 27

    print("\n=== สอนท่าหยิบ (ครั้งเดียว) ===")
    print("1) วางบล็อกบนสายพาน   2) เปิด gripper ให้อ้า   3) ประคองแขนไว้ — torque ข้อ 1-5 จะถูกปิด (แขนจะอ่อน)")
    arm.open_gripper()
    if wait_user is None:
        if input("ประคองแขนแล้ว? พิมพ์ y แล้ว Enter: ").strip().lower() != "y":
            raise SystemExit("ยกเลิก")
    arm.torque_off(ids=(1, 2, 3, 4, 5))
    print("ขยับ jaw ที่อ้าอยู่ให้ครอบบล็อก ที่ความสูงที่อยากให้หนีบ (ปลาย jaw ต่ำกว่าผิวบนบล็อกพอประมาณ) แล้วกด Enter ที่หน้าต่างภาพ")
    if wait_user is not None:
        wait_user()
    else:
        while True:
            if not ui(camera.grab(cap), "TEACH: put open jaws around the block, press Enter"):
                arm.torque_on(ids=(1, 2, 3, 4, 5))
                raise SystemExit("ยกเลิก")
            if state["key"] in (13, 10):
                break
    arm.torque_on(ids=(1, 2, 3, 4, 5))
    time.sleep(0.5)
    jg = read_joints_nb(arm)
    if jg is None:
        raise RuntimeError("อ่านตำแหน่งแขนไม่ได้")
    sg = state4(jg)
    print(f"ท่าหนีบ: jaw กลางที่ x={sg[0]:.0f} y={sg[1]:.0f} z={sg[2]:.0f} mm  ก้ม {sg[3]:.0f}°")
    def margin(p):
        if p is None:
            return -1.0
        b = np.array(p["box"])
        return float(min(b[:, 0].min(), b[:, 1].min(), cap_w(cap) - b[:, 0].max(), cap_h(cap) - b[:, 1].max()))

    # ยกขึ้นทีละขั้น: หยุดที่ขั้นที่บล็อก "ห่างขอบภาพมากที่สุด" (ยกต่อเมื่อยังห่างขอบเพิ่มขึ้น) ต้องห่างอย่างน้อย 30 px
    lift, pose, best = TEACH_LIFT_MM, None, None
    while lift <= TEACH_LIFT_MAX_MM:
        print(f"ยกขึ้นตรงๆ {lift:.0f} mm ...")
        if not move_cartesian(arm, [sg[0], sg[1], sg[2] + lift, sg[3]], timeout=10.0, pitch_weight=0.15,
                              ui=lambda: ui(camera.grab(cap), "TEACH: lifting")):
            print("  ไปไม่ถึงเป้า (ใกล้ขอบข้อต่อ?) — ใช้ตำแหน่งที่ไปได้")
        time.sleep(0.6)
        p_ = stable_pose(cap, H, ui=lambda g: ui(g, "TEACH: measuring"))
        m_ = margin(p_)
        print(f"  บล็อกห่างขอบภาพ {m_:.0f} px" if p_ else "  ยังไม่เห็นบล็อกครบ")
        if best is not None and best[0] >= 30 and m_ < best[0] - 5:
            print(f"  ห่างขอบลดลง -> กลับไปขั้นก่อน ({best[1]:.0f} mm)")
            move_cartesian(arm, [sg[0], sg[1], sg[2] + best[1], sg[3]], timeout=10.0, pitch_weight=0.15,
                           ui=lambda: ui(camera.grab(cap), "TEACH: back"))
            time.sleep(0.6)
            pose = stable_pose(cap, H, ui=lambda g: ui(g, "TEACH: measuring"))
            break
        if best is None or m_ > best[0]:
            best = (m_, lift)
        pose = p_ if m_ >= 30 else None
        if m_ >= TEACH_MARGIN_PX:
            break
        lift += 20.0
    if pose is None or margin(pose) < 30:
        raise RuntimeError("ยกสูงสุดแล้วยังไม่เห็นบล็อกครบ — บล็อกอยู่ในภาพตอนท่าหนีบไหม?")
    jp = read_joints_nb(arm)
    lift_real = float(state4(jp)[2] - sg[2])
    # ---- หมุน roll ลองดู: ภาพหมุนกี่องศาต่อ tick (และทิศ) ----
    deg_per_tick = 0.0
    r0 = jp["wrist_roll"]
    send(arm, {"wrist_roll": (int(r0 + ROLL_PROBE_TICKS), 150)})
    time.sleep(1.2)
    p2 = stable_pose(cap, H, ui=lambda g: ui(g, "TEACH: roll probe"))
    send(arm, {"wrist_roll": (int(r0), 150)})
    time.sleep(1.2)
    if p2 is not None:
        deg_per_tick = wrap180(p2["angle_img"] - pose["angle_img"]) / ROLL_PROBE_TICKS
    if abs(deg_per_tick) < 0.02:
        print("  หมุน roll แล้วมุมบล็อกในภาพแทบไม่เปลี่ยน -> ปิดการหมุน roll ให้ตรงมุม (กล้องไม่ได้หมุนตาม roll?)")
        deg_per_tick = 0.0
    data = {"u": pose["u"], "v": pose["v"], "eq_px": pose["eq"], "angle_img": pose["angle_img"],
            "long_px": pose["long_px"], "short_px": pose["short_px"],
            "lift_mm": lift_real, "pitch_deg": float(state4(jp)[3]), "grip_pitch_deg": float(sg[3]),
            "roll_deg_per_tick": deg_per_tick,
            "delta_joints": {n: float(jg[n] - jp[n]) for n in JOINTS4},
            "grip_joints": jg, "pre_joints": jp,
            "cam_to_block_mm": dist_mm(pose["eq"]) if FOCAL else None,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": "u,v/eq_px/angle_img/pitch_deg = บล็อกในภาพ+มุมก้มที่จุดก่อนหยิบ, delta_joints = ข้อต่อที่ต้องเปลี่ยนจากจุดก่อนหยิบถึงท่าหนีบ (ทำซ้ำตามที่สอน), lift_mm = ความสูงที่ต่างกัน"}
    with open(TEACH_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    print(f"บันทึก {TEACH_FILE}: เป้า u={data['u']:.0f} v={data['v']:.0f} px, บล็อก {data['eq_px']:.0f} px, "
          f"มุม {data['angle_img']:.0f}°, ลงสุดท้าย {lift_real:.0f} mm, roll {deg_per_tick:+.3f}°/tick")
    print("แขนค้างอยู่ที่จุดก่อนหยิบ -> รัน python go_scan.py แล้วลอง python visual_grab.py --dry")
    return data


def cap_w(cap):
    return 1280 if not hasattr(cap, "get") or not cap.get(cv2.CAP_PROP_FRAME_WIDTH) else int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))


def cap_h(cap):
    return 720 if not hasattr(cap, "get") or not cap.get(cv2.CAP_PROP_FRAME_HEIGHT) else int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


# ------------------------------------------------------------ ทำงานจริง
def run(arm, cap, H, teach_d, args, max_time=None):
    scan = {j: C.SCAN_POSE[j] for j in K.JOINTS5}
    s_scan = state4(scan)
    gu = gv = None
    eq_ref = None                     # ขนาดบล็อกตอนอยู่ระดับท่า scan (ใช้ปรับสเกล px -> mm ตามระยะ)
    phase = "FOLLOW"
    still_since, still_hist = None, []
    z_goal, pitch_goal = s_scan[2], s_scan[3]
    z_floor = s_scan[2] - MAX_DESCENT_MM
    if teach_d.get("pre_joints"):
        z_floor = max(z_floor, state4(teach_d["pre_joints"])[2] - PRE_Z_MARGIN_MM)
    roll_goal = None
    last_seen, last_send, fb_lost = 0.0, 0.0, None
    pre_since = None
    cmd_state = {}                    # เป้าสะสมต่อข้อ (ชดเชยการยุบ)
    descend_since = None
    pose, last_pose = None, None
    result = None
    t_start = time.perf_counter()
    if arm is not None:
        arm.open_gripper()
    while True:
        if max_time is not None and time.perf_counter() - t_start > max_time:
            break
        D.watchdog_kick()
        g = camera.grab(cap)
        if g is None:
            if cv2.waitKey(10) & 0xFF == 27:
                break
            continue
        now = time.perf_counter()
        if gu is None:
            gu, gv = img_map_at_scan(H, scan, (g.shape[1] / 2.0, g.shape[0] / 2.0))
        pose = detect(g, H, prefer=(last_pose["u"], last_pose["v"]) if last_pose else None)
        if pose is not None:
            last_pose, last_seen = pose, now
        j = (read_joints_nb(arm) if arm is not None else dict(scan))
        if j is None:
            fb_lost = fb_lost or now
            if now - fb_lost > FB_LOST_MAX_S:
                print("\nหยุด: ไม่ได้ค่าตำแหน่งแขน — Mega/สาย/ไฟ?")
                break
        else:
            fb_lost = None
        tgt = target_px(teach_d, pose["eq"] if pose else 0.0, g.shape) if pose else None
        e_px = np.array([pose["u"] - tgt[0], pose["v"] - tgt[1]]) if pose else None
        if pose is not None and phase == "FOLLOW" and j is not None and \
                max(abs(j[n] - scan[n]) for n in JOINTS4) < 200 and abs(state4(j)[2] - s_scan[2]) < 10:
            eq_ref = pose["eq"] if eq_ref is None else 0.9 * eq_ref + 0.1 * pose["eq"]
        cur = state4(j) if j is not None else None
        ang_err = wrap180(pose["angle_img"] - teach_d["angle_img"]) if pose else None
        lines = []
        due = now - last_send >= 1.0 / SEND_HZ

        # ---------------- FOLLOW ----------------
        if phase == "FOLLOW":
            lines.append("FOLLOW" + ("  DRY" if arm is None else ""))
            if pose is not None and eq_ref:
                err = float(np.hypot(*e_px))
                if err <= CENTER_PX:
                    if still_since is None:
                        still_since, still_hist = now, []
                    still_hist.append((pose["u"], pose["v"]))
                    us, vs = zip(*still_hist)
                    if max(us) - min(us) > JITTER_PX or max(vs) - min(vs) > JITTER_PX:
                        still_since, still_hist = now, [still_hist[-1]]
                    if HOLD_S > 0:
                        lines.append(f"STILL {now - still_since:.1f} / {HOLD_S:.1f} s   (SPACE = grip now)")
                    if HOLD_S > 0 and now - still_since >= HOLD_S:
                        print(f"\nบล็อกนิ่ง {HOLD_S:.0f} s -> ลงไปหยิบ")
                        if arm is None:
                            print("--dry: ไม่ลงจริง")
                            still_since = None
                        else:
                            phase, z_goal, pitch_goal = "DESCEND", cur[2], cur[3]
                            roll_goal = j["wrist_roll"]
                            descend_since = now
                            cmd_state.clear()
                else:
                    still_since = None
                if due and arm is not None and j is not None and phase == "FOLLOW" and err > DEADBAND_PX:
                    w = pixel_err_to_world(e_px, j, gu, gv, eq_ref, pose["eq"]) * KP_XY
                    send(arm, plan_step(j, [cur[0] + w[0], cur[1] + w[1], s_scan[2], s_scan[3]],
                                        ranges=FOLLOW_RANGE, ref=scan, step_xy=FOLLOW_STEP_MM))
                    last_send = now
                lines.append(f"err {e_px[0]:+.0f},{e_px[1]:+.0f} px")
            else:
                still_since = None
                lines.append("no block")
            if pose is not None and HOLD_S <= 0:
                lines.append("press SPACE to grip")
                # หายไปจากภาพ (เช่นเลยเป้าจนหลุดขอบ): ค่อยๆ กลับท่า scan ซึ่งเป็นมุมมองที่ใช้ค้นหา
                if due and arm is not None and j is not None and now - last_seen > SEARCH_AFTER_S and \
                        max(abs(j[n] - scan[n]) for n in K.JOINTS5) > 5:
                    send(arm, {n: (int(round(j[n] + (scan[n] - j[n]) * max(RETURN_FRAC, min(1.0, 3.0 / max(1.0, abs(scan[n] - j[n])))))), 120)
                               for n in K.JOINTS5 if abs(scan[n] - j[n]) >= 2})
                    last_send = now
                    lines.append("searching: back to scan")

        # ---------------- DESCEND ----------------
        elif phase == "DESCEND":
            lines.append("DESCEND")
            if pose is None:
                lines.append(f"block lost {now - last_seen:.1f}s - rising to find it")
                # หลุดภาพตอนลง: ค่อยๆ ยกขึ้น (มุมมองกว้างขึ้น) จนเห็นใหม่ — ไม่สูงกว่าท่า scan
                if due and j is not None and now - last_seen > 0.3:
                    z_goal = min(cur[2] + DESCEND_MM_S / SEND_HZ, s_scan[2])
                    send(arm, plan_step(j, [cur[0], cur[1], z_goal, cur[3]], step_xy=FOLLOW_STEP_MM))
                    last_send = now
                if now - last_seen > 8.0:
                    print("\nมองไม่เห็นบล็อก 8 s -> หยุด (แขนค้าง)")
                    break
            elif j is not None:
                eq_t = teach_d["eq_px"]
                d_star = teach_d.get("cam_to_block_mm") or 120.0
                d_now = d_star * eq_t / pose["eq"]                 # ระยะกล้อง->บล็อก (สัดส่วนกับ 1/ขนาด)
                size_ok = abs(pose["eq"] / eq_t - 1.0) <= PRE_SIZE_TOL
                b = np.array(pose["box"])
                at_edge = (b[:, 0].min() < EDGE_PAUSE_PX or b[:, 1].min() < EDGE_PAUSE_PX or
                           b[:, 0].max() > g.shape[1] - EDGE_PAUSE_PX or b[:, 1].max() > g.shape[0] - EDGE_PAUSE_PX)
                at_edge = at_edge and float(np.hypot(*e_px)) > 2 * PRE_TOL_PX   # อยู่ที่เป้าแล้ว (แม้เป้าใกล้ขอบ) = ไม่ต้องหยุด
                if due:
                    dt = 1.0 / SEND_HZ
                    if at_edge:
                        z_goal = cur[2]                          # ใกล้ขอบภาพ: แก้แนวราบก่อน
                    else:
                        dz = float(np.clip(-(d_now - d_star) * KZ, -DESCEND_MM_S * dt, DESCEND_MM_S * dt))
                        z_goal = max(cur[2] + dz, z_floor)
                        if z_goal <= z_floor + 0.5 and d_now > d_star * (1 + PRE_SIZE_TOL):
                            print("\nลงถึงขีดจำกัดแล้วบล็อกยังเล็กกว่าตอนสอน -> หยุด")
                            break
                        pitch_goal += float(np.clip(teach_d["pitch_deg"] - pitch_goal, -PITCH_RATE_DEG, PITCH_RATE_DEG))
                    w = pixel_err_to_world(e_px, j, gu, gv, eq_ref or pose["eq"], pose["eq"]) * KP_XY
                    if teach_d.get("roll_deg_per_tick") and not at_edge:
                        roll_goal = j["wrist_roll"] - KP_ROLL * ang_err / teach_d["roll_deg_per_tick"]
                    send(arm, integrate(plan_step(j, [cur[0] + w[0], cur[1] + w[1], z_goal, pitch_goal], roll_goal=roll_goal,
                                                  step_xy=FOLLOW_STEP_MM), j, cmd_state))
                    last_send = now
                ok_pos = float(np.hypot(*e_px)) <= PRE_TOL_PX
                ok_pitch = abs(cur[3] - teach_d["pitch_deg"]) <= PRE_PITCH_TOL_DEG
                ok_ang = (not teach_d.get("roll_deg_per_tick")) or abs(ang_err) <= PRE_ANGLE_TOL_DEG
                lines.append(f"size {pose['eq']:.0f}/{eq_t:.0f}px  err {e_px[0]:+.0f},{e_px[1]:+.0f}  "
                             f"pitch {cur[3]:.0f}/{teach_d['pitch_deg']:.0f}  ang {ang_err:+.0f}" + ("  EDGE" if at_edge else ""))
                loose = (descend_since is not None and now - descend_since > PRE_TIMEOUT_S and
                         abs(pose["eq"] / eq_t - 1.0) <= PRE_LOOSE_SIZE and float(np.hypot(*e_px)) <= PRE_LOOSE_PX and ok_pitch)
                if loose and not (size_ok and ok_pos) and pre_since is None:
                    print(f"\nลงมา {now - descend_since:.0f} s แล้ว ใกล้พอ (ขนาด {pose['eq'] / eq_t:.2f}x, ห่าง {np.hypot(*e_px):.0f} px) -> ไปหนีบเลย")
                    pre_since = now - PRE_SETTLE_S
                if (size_ok and ok_pos and ok_pitch and ok_ang) or loose:
                    pre_since = pre_since or now
                    if now - pre_since >= PRE_SETTLE_S:
                        phase = "PRE"
                        _go = args.auto or getattr(args, "_auto_go", False)
                        print("\nถึงจุดก่อนหยิบ" + (" -> ลงหนีบ" if _go else " — กด g ที่หน้าต่างภาพเพื่อลงหยิบ (Esc = ยกเลิก)"))
                else:
                    pre_since = None

        # ---------------- PRE (รอยืนยัน) ----------------
        elif phase == "PRE":
            lines.append("PRE-GRASP reached" + ("" if args.auto else " - press g to grip"))

        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if cur is not None:
            lines.append(f"z {cur[2] - s_scan[2]:+.0f} mm from scan" + (f"   cam->block {dist_mm(pose['eq']):.0f} mm" if (pose and FOCAL) else ""))
        draw(vis, pose, tgt, lines, (0, 140, 255) if phase != "FOLLOW" else (255, 255, 0))
        cv2.imshow(WIN, vis)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            print("\nหยุดโดยผู้ใช้ (แขนค้างที่เดิม)")
            break
        if k == 32 and phase == "FOLLOW":                    # SPACE = ลงหยิบเดี๋ยวนี้ (ไม่ต้องรอนิ่ง) + ถึงจุดก่อนหยิบแล้วหนีบเลย
            if pose is None or j is None or not eq_ref:
                print("\nกด SPACE แล้ว แต่ยังไม่เห็นบล็อก / ยังไม่พร้อม — รอให้เห็นกรอบเขียวก่อน")
            elif arm is None:
                print("\n--dry: กด SPACE แล้ว (ไม่ลงจริง)")
            else:
                print("\nกด SPACE -> ลงไปหยิบ")
                phase, z_goal, pitch_goal = "DESCEND", cur[2], cur[3]
                roll_goal = j["wrist_roll"]
                args._auto_go = True
                descend_since = now
                cmd_state.clear()
        if k == ord("o") and arm is not None:
            arm.open_gripper()
        if k == ord("r") and arm is not None:
            print("\nกลับท่า scan ...")
            arm.move_smooth(scan, duration=3.0)
            phase, still_since = "FOLLOW", None
        if phase == "PRE" and (args.auto or k == ord("g") or getattr(args, "_auto_go", False)):
            result = final_grip(arm, cap, teach_d, g)
            break
    return result


def final_grip(arm, cap, teach_d, g_last):
    """ทำซ้ำท่าที่สอน: จากจุดก่อนหยิบเปลี่ยนข้อต่อเท่ากับตอนสอน (ลงถึงท่าหนีบ) -> ปิด gripper -> ย้อนกลับขึ้นทางเดิม + ยกเพิ่ม"""
    j = read_joints_nb(arm)
    dlt = teach_d["delta_joints"]
    # แก้ความสูงจุดก่อนหยิบให้เท่าที่สอนก่อน (ลงมาต่ำ/สูงเกินจากการดูขนาดบล็อก) -> ทำซ้ำท่าที่สอนแล้วถึงความสูงหนีบจริง
    if teach_d.get("pre_joints"):
        z_pre_t = state4(teach_d["pre_joints"])[2]
        for _ in range(3):
            s_now = state4(j)
            dz = z_pre_t - s_now[2]
            if abs(dz) <= 2.0:
                break
            print(f"ปรับความสูงก่อนหนีบ {dz:+.0f} mm (ให้เท่าตอนสอน z={z_pre_t:.0f}) ...")
            move_cartesian(arm, [s_now[0], s_now[1], z_pre_t, s_now[3]], timeout=6.0, speed_mm_s=FINAL_MM_S, tol_mm=1.5)
            time.sleep(0.3)
            j = read_joints_nb(arm)
    down = {n: j[n] + dlt[n] for n in JOINTS4}
    for n, v in down.items():
        lo, hi = C.JOINT_LIMITS[n]
        if not lo <= v <= hi:
            raise RuntimeError(f"ท่าลงหยิบเกินขอบเขต {n}={v:.0f} — สอนใหม่ใกล้ตำแหน่งนี้")
    # ความปลอดภัย: ท่าหนีบที่จะไป (จาก FK) ต้องไม่ต่ำ/ก้มต่างจากท่าหนีบที่สอนเกินกำหนด
    full = dict(j)
    full.update(down)
    s_dn, s_tg = state4(full), state4(teach_d["grip_joints"])
    if s_dn[2] < s_tg[2] - FINAL_Z_TOL_MM or abs(s_dn[3] - s_tg[3]) > FINAL_PITCH_TOL_DEG:
        print(f"ไม่ลง: ท่าหนีบที่คำนวณได้ z={s_dn[2]:.0f} mm ก้ม {s_dn[3]:.0f}° แต่ที่สอน z={s_tg[2]:.0f} ก้ม {s_tg[3]:.0f}° "
              f"(ต่างเกิน {FINAL_Z_TOL_MM:.0f} mm / {FINAL_PITCH_TOL_DEG:.0f}°) — กันชนสายพาน")
        return {"gripper_tick": None, "verdict": "refused (unsafe depth)"}
    dur = max(2.0, abs(teach_d["lift_mm"]) / FINAL_MM_S)
    print(f"ลงหยิบ (ทำซ้ำท่าที่สอน ~{teach_d['lift_mm']:.0f} mm, {dur:.1f} s) ...")
    arm.move_smooth(down, duration=dur, fine=True)          # fine = ชดเชยการยุบของ servo ให้ถึงท่าจริง
    if EXTRA_DEPTH_MM > 0:
        sd = state4(read_joints_nb(arm))
        print(f"ลงเพิ่มอีก {EXTRA_DEPTH_MM:.0f} mm ...")
        move_cartesian(arm, [sd[0], sd[1], sd[2] - EXTRA_DEPTH_MM, sd[3]], timeout=8.0, speed_mm_s=EXTRA_MM_S, tol_mm=1.5)
    print("ปิด gripper ...")
    pos = arm.close_gripper()
    time.sleep(0.4)
    print("ยกขึ้นทางเดิม ...")
    arm.move_smooth({n: j[n] for n in JOINTS4}, duration=dur, fine=True)
    jj = read_joints_nb(arm)
    s2 = state4(jj)
    move_cartesian(arm, [s2[0], s2[1], s2[2] + max(0.0, LIFT_AFTER_MM - teach_d["lift_mm"]), s2[3]],
                   timeout=5.0, speed_mm_s=30.0, pitch_weight=0.15)
    rng = C.GRIP_TICK_RANGE if getattr(C, "GRIP_TICK_RANGE", None) else None
    verdict = ("ได้บล็อก" if rng[0] <= pos <= rng[1] else "อาจหนีบพลาด") if rng else "ยังไม่รู้ (ยังไม่ได้ตั้ง GRIP_TICK_RANGE)"
    print(f"หยิบเสร็จ: gripper หยุดที่ {pos} tick -> {verdict}   (แขนถือค้างไว้ — รัน go_scan.py / เปิด gripper เอง)")
    return {"gripper_tick": pos, "verdict": verdict}


def main():
    global HOLD_S, EXTRA_DEPTH_MM, DESCEND_MM_S, FINAL_MM_S, EXTRA_MM_S
    ap = argparse.ArgumentParser()
    ap.add_argument("--teach", action="store_true", help="สอนจุดหยิบครั้งเดียว")
    ap.add_argument("--dry", action="store_true", help="ไม่ขยับแขน")
    ap.add_argument("--auto", action="store_true", help="ถึงจุดก่อนหยิบแล้วลงเลย ไม่ต้องกด g")
    ap.add_argument("--slow", action="store_true", help="ทดสอบครั้งแรก: ลง/หนีบ/ยก ช้าลง ~2.5 เท่า")
    ap.add_argument("--deeper", type=float, default=EXTRA_DEPTH_MM, help="ลงลึกกว่าท่าที่สอนอีกกี่ mm ก่อนหนีบ (ค่าเริ่มต้น 5)")
    ap.add_argument("--hold", type=float, default=HOLD_S, help="วินาทีที่บล็อกต้องนิ่งก่อนลงเอง (ค่าเริ่มต้น 2; 0 = ไม่ลงเอง ใช้ SPACE เท่านั้น)")
    args = ap.parse_args()
    HOLD_S = args.hold
    EXTRA_DEPTH_MM = max(0.0, min(args.deeper, 15.0))       # จำกัด 15 mm กันชนสายพาน
    if args.slow:
        DESCEND_MM_S, FINAL_MM_S, EXTRA_MM_S = 10.0, 8.0, 4.0
        print(f"โหมดช้า: ลง {DESCEND_MM_S:.0f} mm/s, ลงหนีบ {FINAL_MM_S:.0f} mm/s, ลงเพิ่ม {EXTRA_DEPTH_MM:.0f} mm ที่ {EXTRA_MM_S:.0f} mm/s")
    H = D.load_homography()
    cap = camera.open_camera()
    arm = None
    try:
        if not args.dry:
            from arm_select import make_arm
            arm = make_arm()
            arm.connect()
            try:
                import arm_mega as _am
                _am.IDLE_HOOK = lambda: cv2.waitKey(1)
            except ImportError:
                pass
        if args.teach:
            if arm is None:
                raise SystemExit("--teach ต้องใช้แขน (ไม่ใส่ --dry)")
            teach(arm, cap, H)
            return
        if not os.path.exists(TEACH_FILE):
            raise SystemExit("ยังไม่ได้สอนจุดหยิบ -> python visual_grab.py --teach")
        with open(TEACH_FILE, encoding="utf-8") as f:
            teach_d = json.load(f)
        if arm is not None:
            j = {n: arm.read_joint(n) for n in K.JOINTS5}
            off = max(abs(j[n] - C.SCAN_POSE[n]) for n in K.JOINTS5)
            if off > 40:
                raise SystemExit(f"แขนไม่ได้อยู่ท่า scan (คลาด {off:.0f} tick) -> python go_scan.py ก่อน")
        print("SPACE = ลงหยิบเดี๋ยวนี้ | Esc = หยุด" + (" | --hold 0: ไม่ลงเอง" if HOLD_S <= 0 else f" | หรือบล็อกนิ่ง {HOLD_S:.0f} s แล้วลงเอง"))
        print(f"เป้า: บล็อกที่ u={teach_d['u']:.0f} v={teach_d['v']:.0f} px ขนาด {teach_d['eq_px']:.0f} px "
              f"| นิ่ง {HOLD_S:.0f} s แล้วลง | ลงสุดท้าย {teach_d['lift_mm']:.0f} mm" + ("" if args.auto else " | ถึงจุดก่อนหยิบแล้วกด g"))
        run(arm, cap, H, teach_d, args)
    except KeyboardInterrupt:
        print("\nหยุด (Ctrl+C) — แขนค้างที่เดิม")
    finally:
        D.watchdog_off()
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        if arm is not None:
            arm.disconnect(hold=True)
        cap.release()


if __name__ == "__main__":
    main()
