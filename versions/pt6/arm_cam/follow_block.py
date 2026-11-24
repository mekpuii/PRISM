"""follow_block.py — แขน "ตามบล็อก" ที่เคลื่อนที่บนสายพาน โดยหมุนฐาน (pan ข้อ 1) ให้บล็อกอยู่กลางภาพ (visual servo 1 แกน)

หลักการ (ทำไมหมุนแค่ pan): สายพานวิ่งตามแนวแกน y ของเฟรมแขน ปลายแขนที่ระยะ ~170 mm หมุน pan แล้วเคลื่อนไปตามแนวสายพานพอดี
  ส่วนเลน/ความสูง/roll ไม่ต้องขยับ -> ปลอดภัยกว่าสั่งหลายข้อพร้อมกัน
วงจรควบคุม ทุกเฟรม:
  1) BlockTracker: ค้นหาทั้งภาพ -> เจอแล้วหดกรอบตามบล็อก (detect_block.py)
  2) คลาดในภาพ = บล็อก - จุดเป้า (กลางภาพตามแนวนอน) แปลงเป็น mm ด้วย homography ของท่า scan
  3) แปลง mm -> tick ของ pan ด้วย FK (ทิศ/สเกลมาจาก FK จริง จึงไม่ต้องเดาเครื่องหมาย)
  4) สั่ง P,1 ไปที่ตำแหน่งปัจจุบัน + Kp*คลาด จำกัดความเร็วและช่วงหมุน ส่งซ้ำถี่กว่า 500 ms (เฟิร์มแวร์ freeze เป้าถ้าเงียบ)
ความปลอดภัย: ต้องอยู่ท่า scan ก่อน | หมุนได้เฉพาะ ±PAN_RANGE_TICKS จากท่า scan | ความเร็วจำกัด | บล็อกหาย = หยุดค้างที่เดิม
            Esc/Ctrl+C = ค้างแขนที่ตำแหน่งปัจจุบัน | --dry = ไม่ส่งคำสั่งแขน (พิมพ์สิ่งที่จะสั่ง)
ข้อจำกัด: homography ใช้ท่า scan จึงแม่นเมื่อหมุนไม่มาก (±~20°); สายพานเร็วเกินความเร็วสูงสุดของ pan จะตามไม่ทัน (ดู MAX_SPEED)

ใช้:  python follow_block.py --dry     # ลองก่อน ไม่ขยับแขน (ลากบล็อกด้วยมือดูตัวเลข)
      python follow_block.py           # ตามจริง
"""
import faulthandler as _fh
import os as _os
_FH = open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "crash_native.txt"), "w")
_fh.enable(file=_FH, all_threads=True)      # ถ้า Python ตายแบบไม่มี traceback (เช่น OpenCV/กล้อง/พอร์ตอนุกรมล่ม) จะเขียนว่าตายตรงไหนลงไฟล์นี้

import argparse
import math
import time

import cv2
import numpy as np

import camera
import config as C
import detect_block as D
import kinematics as K
try:
    import grip_observe as _HP                        # กล่อง HEIGHT (กล้อง->บล็อก / ปลาย jaw) — ไม่มีไฟล์นี้ก็ทำงานได้ปกติ
except Exception:
    _HP = None

# ---- พารามิเตอร์ (ค่าเริ่มต้นระมัดระวัง ปรับทีหลังได้) ----
KP = 0.4                 # สั่งแก้ 40% ของคลาดต่อรอบ (ต่ำ = นุ่ม ไม่แกว่ง; 0.6 แกว่งขึ้นลงบนแขนจริง)
MAX_SPEED = 300          # tick/s เพดานความเร็ว pan (ตาม belt ~ r*ω: 300 tick/s ≈ 0.46 rad/s ≈ 78 mm/s ที่ r=170)
MIN_SPEED = 90           # ค่าต่ำสุดที่เฟิร์มแวร์รับ (C.MEGA_MIN_SPEED)
PAN_RANGE_TICKS = 350    # หมุนได้ ±350 tick (~31°, ≈ ±96 mm ที่ปลายแขน) จากท่า scan เท่านั้น
DEADBAND_TICKS = 3       # คลาดน้อยกว่านี้ไม่ขยับ
SHOW_HZ = 30.0           # ความถี่วาดภาพ/พิมพ์ (ตรวจจับ+ควบคุมยังทำทุกเฟรมตาม SEND_HZ)
SEND_HZ = 12.0           # ความถี่ส่งคำสั่ง (ต้องถี่กว่า 2 Hz เพื่อไม่ให้เป้าโดน freeze)
POSE_TOL = 40            # tick: ต้องอยู่ใกล้ท่า scan ก่อนเริ่ม

# ---- โหมด 2 แกน: ให้บล็อกอยู่ "กลางภาพ" ทั้งแนวนอนและแนวตั้ง ----
# ขยับปลายแขน (กล้องติดข้อมือ) ในระนาบ x-y โดย "คงความสูง z และมุมก้ม pitch เดิม" -> กล้องแค่เลื่อน ไม่เงยไม่ก้ม ไม่ลอยสูงขึ้น
# ใช้ Jacobian เชิงตัวเลขของ FK (pan, lift, elbow, wrist_flex -> x, y, z, pitch) แก้ 4 สมการ 4 ตัวแปร; roll/gripper ไม่แตะ
TRACK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
RANGE_TICKS = {"shoulder_pan": 350, "shoulder_lift": 140, "elbow_flex": 140, "wrist_flex": 140}   # ขยับได้ไกลสุดจากท่า scan
ABBR = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elb", "wrist_flex": "wf"}   # ชื่อย่อโชว์ใน HUD
Z_ABORT_MM = 20.0        # ความสูงปลายแขนเพี้ยนจากตอนเริ่มตามเกินนี้ (ต่อเนื่อง Z_ABORT_S) -> หยุด (กันจมลงสายพาน)
Z_ABORT_S = 0.3
FB_LOST_MAX_S = 2.0       # ฟีดแบ็กแขนขาดเกินนี้ -> หยุดโปรแกรม (ระหว่างนั้นไม่สั่งแขน ค้างที่เดิม)
CENTER_PX = 15           # บล็อกอยู่ในรัศมีนี้จากกากบาท (px) ถือว่า "ถึงกลาง"
HOLD_S = 1.0             # ต้องอยู่กลางนิ่งต่อเนื่องเท่านี้ (วินาที) ก่อนหยิบ
JITTER_PX = 8            # ระหว่างรอ ตำแหน่งบล็อกในภาพขยับได้ไม่เกินนี้ (ถ้าบล็อกยังวิ่งอยู่ = ไม่นิ่ง ไม่หยิบ)
BG_VALID_TICKS = 15      # ใช้ background ต่อเมื่อทุกข้อห่างท่า scan ไม่เกินนี้ (กล้องแทบไม่ขยับ)
MAX_STEP_TICKS = 40      # ต่อรอบ ข้อไหนเกินนี้ย่อทุกข้อลงตามสัดส่วน (กันกระโดดตอนบล็อกอยู่ไกลจากกลางภาพ)


def read_joints_nb(arm, max_age=1.0):
    """อ่านตำแหน่งข้อ 1-5 "ไม่รอ": คืน dict หรือ None ถ้ามีข้อไหนไม่มีค่าสด
    (arm.read_joint() รอได้ถึง 3 วินาทีต่อข้อ -> ลูปค้างหลายวินาที หน้าต่าง Not Responding แล้วแขนไม่ได้รับคำสั่งเลย)"""
    out = {}
    for j in K.JOINTS5:
        p = arm.link.servo_pos(arm.ids[j], max_age)
        if p is None:
            return None
        out[j] = p
    return out


def tip_state(joints):
    x, y, z, pitch, _ = K.fk(joints, "touch")
    return np.array([x, y, z, pitch])


def jacobian(joints, d=10):
    """J[i, k] = การเปลี่ยนของ (x, y, z, pitch) ต่อ 1 tick ของ TRACK_JOINTS[k] — จาก FK จริง"""
    base = tip_state(joints)
    J = np.zeros((4, 4))
    for k, j in enumerate(TRACK_JOINTS):
        m = dict(joints)
        m[j] = m[j] + d
        J[:, k] = (tip_state(m) - base) / d
    return J


def cam_rotation(joints, scan):
    """มุม (rad) ที่กล้องหมุนรอบแกน z จากท่า scan เพราะ pan หมุนไป — หาจากทิศ tangent ของ FK (ไม่ต้องเดาเครื่องหมาย)
    homography แปลงพิกเซลเป็น mm ในเฟรมแขน "ตอนท่า scan" ถ้า pan หมุนแล้วต้องหมุนเวกเตอร์ตามด้วย"""
    t0, _ = pan_tangent(scan)
    t1, _ = pan_tangent(joints)
    return float(np.arctan2(t0[0] * t1[1] - t0[1] * t1[0], t0 @ t1))


def block_error_mm(H, block_px, target_px):
    """เวกเตอร์ mm (เฟรมแขนท่า scan) จากจุดเป้ากลางภาพไปหาบล็อก = ปลายแขนต้องเลื่อนไปทางนี้"""
    pb = cv2.perspectiveTransform(np.array([[block_px]], np.float32), H)[0, 0]
    pt = cv2.perspectiveTransform(np.array([[target_px]], np.float32), H)[0, 0]
    return (pb - pt).astype(float)


def block_world(H, c_px, long_deg, joints, scan):
    """ตำแหน่งบล็อกจริงในเฟรมแขน (mm) + ทิศแกนยาว — แม้กล้องขยับออกจากท่า scan แล้ว
    H ใช้ได้เฉพาะท่า scan จึงแปลงเป็น mm ในเฟรมของท่า scan ก่อน แล้วเลื่อน/หมุนตามการเคลื่อนของข้อมือ (rigid):
      p_now = tip_now + R(theta) @ (p_scan - tip_scan)   (theta = มุมที่ pan หมุนไป หาจาก FK; z และ pitch คงที่ตามที่ควบคุมไว้)"""
    pc = cv2.perspectiveTransform(np.array([[c_px]], np.float32), H)[0, 0].astype(float)
    th = cam_rotation(joints, scan)
    c, s = np.cos(th), np.sin(th)
    d = pc - tip_state(scan)[:2]
    p = tip_state(joints)[:2] + np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    return p, (long_deg + np.degrees(th)) % 180.0


LEAD_MAX_TICKS = 60       # เป้าที่สะสมนำตำแหน่งจริงได้ไม่เกินนี้ (กัน windup) — แขนยุบจากน้ำหนัก ~30-40 tick


STUCK_TICKS = 2           # ข้อที่ขยับไม่เกินนี้ระหว่างรอบ = "ติด" (แบกน้ำหนัก) -> ค่อยสะสมเป้า
HOLD_PX = 12              # บล็อกห่างเป้าไม่เกินนี้ -> ไม่สั่งขยับ (กันแกว่งรอบเป้า) แต่คงแรงชดเชยไว้


def integrate_goals(cmds, joints, scan, cmd_goal):
    """แปลงคำสั่ง "ตำแหน่งจริง + Δ" ให้ชนะการยุบของ servo ที่แบกน้ำหนัก (หยุดห่างเป้า ~30-40 tick) โดยไม่แกว่ง:
      * ข้อที่ "ติด" (ขยับไม่เกิน STUCK_TICKS จากรอบก่อน) และยังต้องไปทางเดิม -> เป้า = เป้าเดิม + Δ (สะสม)
      * ข้อที่กำลังขยับอยู่ -> ไม่สะสมเพิ่ม (เป้า = ตำแหน่งจริง + Δ + แรงชดเชยที่มีอยู่ ไม่โต)
      * ต้องกลับทิศ -> ล้างที่สะสมทิ้งทันที (ไม่ดันเลยเป้า = ไม่แกว่งขึ้นลง)
      เป้านำตำแหน่งจริงได้ไม่เกิน LEAD_MAX_TICKS และอยู่ในช่วง RANGE/SAFE"""
    out = {}
    for j, (g, sp) in cmds.items():
        d = g - joints[j]                                   # Δ ที่ต้องการรอบนี้
        prev_pos = cmd_goal.get(("pos", j))
        lead = cmd_goal.get(j, joints[j]) - joints[j]       # แรงชดเชยที่สะสมอยู่ (tick)
        if d * lead < 0:
            lead = 0.0                                      # กลับทิศ: ทิ้งที่สะสม
        stuck = prev_pos is not None and abs(joints[j] - prev_pos) <= STUCK_TICKS
        if stuck and abs(d) > 0:
            lead = lead + d                                 # ติดอยู่ และยังต้องไปทางนี้: ดันเพิ่ม
        g2 = joints[j] + (d if abs(d) > abs(lead) else lead) if lead else joints[j] + d
        g2 = min(max(g2, joints[j] - LEAD_MAX_TICKS), joints[j] + LEAD_MAX_TICKS)
        lo = max(C.JOINT_LIMITS[j][0] + C.LIMIT_MARGIN, scan[j] - RANGE_TICKS.get(j, 4096) - LEAD_MAX_TICKS)
        hi = min(C.JOINT_LIMITS[j][1] - C.LIMIT_MARGIN, scan[j] + RANGE_TICKS.get(j, 4096) + LEAD_MAX_TICKS)
        g2 = min(max(g2, lo), hi)
        cmd_goal[j] = g2
        cmd_goal[("pos", j)] = joints[j]
        out[j] = (int(round(g2)), sp)
    return out


def plan_follow(joints, scan, e_mm, pan_only=False, ref_zp=None):
    """คืน (dq, cmds): dq = tick ที่ควรเปลี่ยนต่อข้อ (ไว้โชว์), cmds = {joint: (goal, speed)} หรือ {} ถ้าไม่ต้องขยับ"""
    th = cam_rotation(joints, scan)
    c, s = np.cos(th), np.sin(th)
    d = KP * np.array([c * e_mm[0] - s * e_mm[1], s * e_mm[0] + c * e_mm[1]])      # mm ในเฟรมแขนปัจจุบัน
    if pan_only:
        t, mm_per_tick = pan_tangent(joints)
        dq = {"shoulder_pan": float(d @ t) / mm_per_tick}
    else:
        # z และ pitch ต้อง "กลับไปเท่าท่า scan" (ค่าสัมบูรณ์) ไม่ใช่แค่ "ไม่เปลี่ยนจากตอนนี้" — ไม่งั้นความคลาดสะสมแล้วแขนค่อยๆ จมลง
        t_scan, t_now = tip_state(scan), tip_state(joints)
        z_ref, p_ref = (t_scan[2], t_scan[3]) if ref_zp is None else ref_zp   # ความสูง/ก้มตอน "เริ่มตาม" (แขนจริงยุบจากท่า scan ได้)
        dz = float(np.clip(z_ref - t_now[2], -5, 5))
        dp = float(np.clip(p_ref - t_now[3], -4, 4))
        sol = np.linalg.lstsq(jacobian(joints), np.array([d[0], d[1], dz, dp]), rcond=1e-3)[0]
        dq = dict(zip(TRACK_JOINTS, map(float, sol)))
    big = max(abs(v) for v in dq.values())
    if big < DEADBAND_TICKS:
        return dq, {}
    if big > MAX_STEP_TICKS:
        dq = {j: v * MAX_STEP_TICKS / big for j, v in dq.items()}
    # ชนขอบช่วง: ย่อ "ทุกข้อ" ด้วยสัดส่วนเดียวกัน (ไม่ตัดทีละข้อ) เพื่อให้ทิศการเคลื่อนและความสูง z ยังถูกต้อง
    k = 1.0
    for j, v in dq.items():
        lo, hi = scan[j] - RANGE_TICKS[j], scan[j] + RANGE_TICKS[j]
        room = (hi - joints[j]) if v > 0 else (joints[j] - lo)
        if abs(v) > 1e-9:
            k = min(k, max(room, 0.0) / abs(v))
    dq = {j: v * k for j, v in dq.items()}
    if not pan_only and k < 0.2:
        # ชนขอบช่วง: เลื่อนแนวราบต่อไม่ได้ แต่ "แก้ความสูง/มุมก้ม" ยังต้องทำได้ (ไม่งั้นค้างต่ำจนระบบป้องกันหยุด)
        t_now = tip_state(joints)
        z_ref, p_ref = (tip_state(scan)[2], tip_state(scan)[3]) if ref_zp is None else ref_zp
        sol = np.linalg.lstsq(jacobian(joints), np.array([0.0, 0.0, float(np.clip(z_ref - t_now[2], -5, 5)),
                                                          float(np.clip(p_ref - t_now[3], -4, 4))]), rcond=1e-3)[0]
        zq = dict(zip(TRACK_JOINTS, map(float, sol)))
        bz = max(abs(v) for v in zq.values())
        if bz > MAX_STEP_TICKS:
            zq = {j: v * MAX_STEP_TICKS / bz for j, v in zq.items()}
        kz = 1.0
        for j, v in zq.items():
            lo, hi = scan[j] - RANGE_TICKS[j] - 30, scan[j] + RANGE_TICKS[j] + 30   # เผื่อ 30 tick สำหรับการแก้ความสูงเท่านั้น
            room = (hi - joints[j]) if v > 0 else (joints[j] - lo)
            if abs(v) > 1e-9:
                kz = min(kz, max(room, 0.0) / abs(v))
        dq = {j: dq[j] + zq[j] * kz for j in dq}
    cmds = {}
    for j, v in dq.items():
        goal = joints[j] + v
        if abs(goal - joints[j]) < 1:
            continue
        speed = min(max(abs(goal - joints[j]) * SEND_HZ * 1.5, MIN_SPEED), MAX_SPEED)
        cmds[j] = (int(round(goal)), int(speed))
    return dq, cmds


def pan_tangent(joints, dtick=20):
    """ทิศ (หน่วยเวกเตอร์ในเฟรมแขน) และ mm ต่อ 1 tick ที่ปลายแขนเคลื่อนเมื่อ pan เพิ่ม 1 tick — จาก FK"""
    m0 = dict(joints)
    m1 = dict(joints)
    m1["shoulder_pan"] = m0["shoulder_pan"] + dtick
    p0 = np.array(K.fk(m0, "touch")[:2], dtype=float)
    p1 = np.array(K.fk(m1, "touch")[:2], dtype=float)
    d = p1 - p0
    n = float(np.linalg.norm(d))
    return d / n, n / dtick


def correction_ticks(H, block_px, target_u, joints):
    """pan ต้อง "เพิ่ม" กี่ tick เพื่อให้บล็อกมาอยู่ที่ u = target_u (ตามแนวนอนของภาพ)"""
    pb = cv2.perspectiveTransform(np.array([[block_px]], np.float32), H)[0, 0]
    pt = cv2.perspectiveTransform(np.array([[(target_u, block_px[1])]], np.float32), H)[0, 0]
    e = pb - pt                                      # mm ที่ต้องเลื่อนไปหา (เฟรมแขน)
    t_hat, mm_per_tick = pan_tangent(joints)
    return float(e @ t_hat) / mm_per_tick


def plan_command(cur, dtk, scan_pan):
    """คืน (goal_tick, speed) หรือ None ถ้าไม่ต้องขยับ"""
    step = KP * dtk
    if abs(step) < DEADBAND_TICKS:
        return None
    goal = cur + step
    lo, hi = scan_pan - PAN_RANGE_TICKS, scan_pan + PAN_RANGE_TICKS
    goal = min(max(goal, lo), hi)
    if abs(goal - cur) < 1:
        return None                                  # ชนขอบช่วงหมุน
    speed = min(max(abs(goal - cur) * SEND_HZ * 1.5, MIN_SPEED), MAX_SPEED)
    return int(round(goal)), int(speed)


def wait_enter_live(msg, cap):
    """รอ Enter โดยโชว์ภาพกล้องสดไปด้วย (กด Enter ที่ terminal หรือที่หน้าต่างภาพ) — เห็นเลนตอนเตรียมถ่ายพื้นหลัง"""
    import pi_env
    print(msg, end="", flush=True)
    while True:
        g = camera.grab(cap)
        if g is not None:
            vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            cv2.putText(vis, "EMPTY LANE? press Enter to capture background", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("follow block (Esc = quit)", vis)
        k = cv2.waitKey(30) & 0xFF
        if k in (13, 10) or pi_env.enter_pressed():
            break
    print()


GRIP_FILE = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "follow_grip.json")
GRIP_J = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
GRIP_MM_S = 20.0          # ความเร็วลงหนีบ/ยกกลับ (--slow = 8)
EXTRA_MM_S = 10.0         # ความเร็วลงเพิ่ม (--slow = 4)
EXTRA_DEPTH_MM = 0.0      # ลงลึกกว่าท่าที่สอนอีกเท่านี้ก่อนหนีบ (--deeper)
GRIP_Z_TOL_MM = 10.0      # ท่าหนีบที่คำนวณได้ต่ำกว่าที่สอนเกินนี้ -> ไม่ลง (กันชนสายพาน)


def _show_text(cap, win, lines):
    g = camera.grab(cap)
    if g is not None:
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        for i, t in enumerate(lines):
            cv2.putText(vis, t, (10, 36 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.imshow(win, vis)
    return cv2.waitKey(15) & 0xFF


def _live_hook(cap, win, text):
    """ระหว่างแขนขยับนาน (หนีบ/ยก): วาดภาพกล้องสดต่อ หน้าต่างไม่ค้าง"""
    st = {"t": 0.0}

    def hook():
        D.watchdog_kick()
        now = time.perf_counter()
        if now - st["t"] >= 1 / 25:
            st["t"] = now
            _show_text(cap, win, [text[0]])
        else:
            cv2.waitKey(1)
    return hook


def _am_hook():
    try:
        import arm_mega as _am
        if _am.IDLE_HOOK:
            _am.IDLE_HOOK()
    except ImportError:
        cv2.waitKey(1)


def _set_hook(h):
    try:
        import arm_mega as _am
        old = _am.IDLE_HOOK
        _am.IDLE_HOOK = h
        return old
    except ImportError:
        return None


def teach_grip(arm, cap, win):
    """t: (บล็อกอยู่กลางเป้าแล้ว) ปิด torque ข้อ 1-5 -> ประคอง jaw ที่อ้าให้ครอบบล็อกที่ความสูงที่จะหนีบ -> Enter
    จดว่าจากท่าตาม (ความสูง scan) ไปท่าหนีบ ข้อต่อเปลี่ยนเท่าไร -> follow_grip.json แล้วกลับท่าตามเดิม"""
    import json
    j0 = read_joints_nb(arm)
    if j0 is None:
        print("\nสอนไม่ได้: อ่านตำแหน่งแขนไม่ได้")
        return
    print("\n=== สอนท่าหนีบ: ประคองแขนไว้ — torque ข้อ 1-5 จะปิด (แขนอ่อน) ===")
    arm.open_gripper()
    arm.torque_off(ids=(1, 2, 3, 4, 5))
    print("ดัน jaw ที่อ้าอยู่ลงให้ครอบบล็อก ที่ความสูงที่อยากหนีบ แล้วกด Enter ที่หน้าต่างภาพ (Esc = ยกเลิก)")
    while True:
        k = _show_text(cap, win, ["TEACH GRIP: guide open jaws around block", "Enter = save   Esc = cancel"])
        if k in (13, 10, 27):
            break
    arm.torque_on(ids=(1, 2, 3, 4, 5))
    time.sleep(0.5)
    jg = read_joints_nb(arm)
    if k != 27 and jg is not None:
        delta = {n: int(jg[n] - j0[n]) for n in GRIP_J}
        sz0, szg = tip_state(j0), tip_state(jg)
        with open(GRIP_FILE, "w", encoding="utf-8") as f:
            json.dump({"delta_joints": delta, "follow_joints": {n: int(j0[n]) for n in GRIP_J},
                       "grip_joints": {n: int(jg[n]) for n in GRIP_J}, "grip_z_mm": float(szg[2]),
                       "drop_mm": float(sz0[2] - szg[2]), "time": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)
        print(f"บันทึก follow_grip.json: ลง {sz0[2] - szg[2]:.0f} mm, ข้อเปลี่ยน {delta}")
    else:
        print("ยกเลิกการสอน")
    print("กลับท่าตาม ...")
    arm.move_smooth({n: j0[n] for n in GRIP_J}, duration=3.0, fine=True)


def grip_now(arm, cap, win):
    """SPACE: จากท่าตามตอนนี้ ทำซ้ำท่าที่สอน (ลง) -> ลงเพิ่ม EXTRA_DEPTH_MM -> ปิด gripper -> ยกกลับท่าตาม. คืน True ถ้าหนีบแล้ว"""
    import json
    if not _os.path.exists(GRIP_FILE):
        print("\nยังไม่ได้สอนท่าหนีบ: ให้บล็อกอยู่กลางเป้าแล้วกด t ก่อน")
        return False
    with open(GRIP_FILE, encoding="utf-8") as f:
        td = json.load(f)
    j0 = read_joints_nb(arm)
    if j0 is None:
        print("\nไม่หนีบ: อ่านตำแหน่งแขนไม่ได้")
        return False
    down = {n: j0[n] + td["delta_joints"][n] for n in GRIP_J}
    for n, v in down.items():
        lo, hi = C.JOINT_LIMITS[n]
        if not lo <= v <= hi:
            print(f"\nไม่หนีบ: ท่าลงเกินขอบเขต {n}={v:.0f}")
            return False
    z_dn = float(tip_state({**j0, **down})[2])
    if z_dn < td["grip_z_mm"] - GRIP_Z_TOL_MM:
        print(f"\nไม่หนีบ: ท่าหนีบที่คำนวณได้ z={z_dn:.0f} mm ต่ำกว่าที่สอน ({td['grip_z_mm']:.0f}) เกิน {GRIP_Z_TOL_MM:.0f} mm — กันชนสายพาน")
        return False
    dur = max(2.5, abs(td["drop_mm"]) / GRIP_MM_S)
    phase = ["GRIP: going down"]
    old_hook = _set_hook(_live_hook(cap, win, phase))
    try:
        pos = _grip_motion(arm, td, j0, down, dur, phase)
    finally:
        _set_hook(old_hook)
    rng = getattr(C, "GRIP_TICK_RANGE", None)
    verdict = ("ได้บล็อก" if rng[0] <= pos <= rng[1] else "อาจหนีบพลาด") if rng else "ดูเอาว่าติดไหม (ยังไม่ได้ตั้ง GRIP_TICK_RANGE)"
    print(f"หนีบเสร็จ: gripper หยุดที่ {pos} tick -> {verdict}")
    print("ถือบล็อกไว้: r = ปล่อยบล็อก แล้วตามต่อ | Esc = ออก (แขนถือค้างไว้)")
    while True:
        D.watchdog_kick()
        k = _show_text(cap, win, [f"HOLDING (gripper {pos})", "r = release + follow again   Esc = quit (keep holding)"])
        if k == 27:
            return True
        if k == ord("r"):
            print("ปล่อยบล็อก -> ตามต่อ")
            arm.open_gripper()
            return False


def error_hold(arm, cap, win, err):
    """หนีบ/ยกล้มกลางทาง: กล้องยังเปิด แขนค้างที่เดิม (gripper ยังหนีบอยู่)
    s = ลองกลับท่า scan (ถือบล็อกไว้) | r = เปิด gripper แล้วตามต่อ | Esc = ออก"""
    import traceback
    traceback.print_exc()
    print(f"\nหนีบ/ยกไม่สำเร็จ: {err}\n  s = ลองกลับท่า scan (ยังถือบล็อก) | r = ปล่อยบล็อกแล้วตามต่อ | Esc = ออก (แขนค้างไว้)")
    while True:
        D.watchdog_kick()
        k = _show_text(cap, win, ["ERROR: " + str(err)[:60], "s = back to scan (keep block)  r = release + follow  Esc = quit"])
        try:
            if k == 27:
                return True
            if k == ord("s"):
                print("กลับท่า scan (ถือบล็อกไว้) ...")
                old = _set_hook(_live_hook(cap, win, ["back to scan (holding block)"]))
                try:
                    arm.move_smooth({n: C.SCAN_POSE[n] for n in GRIP_J}, duration=3.0, fine=True)
                finally:
                    _set_hook(old)
                print("ถึงท่า scan แล้ว: r = ปล่อยบล็อกแล้วตามต่อ | Esc = ออก")
            if k == ord("r"):
                arm.open_gripper()
                print("ปล่อยบล็อก -> ตามต่อ")
                return False
        except Exception as e2:
            print(f"ยังไม่ได้ ({e2}) — เช็กสาย/ไฟเลี้ยง แล้วกด s อีกครั้ง")


def _grip_motion(arm, td, j0, down, dur, phase):
    print(f"\nหนีบ: ลง {td['drop_mm']:.0f} mm ({dur:.1f} s) ...")
    arm.open_gripper()
    arm.move_smooth(down, duration=dur, fine=True)
    if EXTRA_DEPTH_MM > 0:
        phase[0] = f"GRIP: {EXTRA_DEPTH_MM:.0f} mm deeper"
        import visual_grab as V
        sd = V.state4(V.read_joints_nb(arm))
        print(f"ลงเพิ่มอีก {EXTRA_DEPTH_MM:.0f} mm ...")
        V.move_cartesian(arm, [sd[0], sd[1], sd[2] - EXTRA_DEPTH_MM, sd[3]], timeout=8.0, speed_mm_s=EXTRA_MM_S, tol_mm=1.5,
                         ui=lambda: (_am_hook(), True)[1])
    phase[0] = "GRIP: closing"
    print("ปิด gripper ...")
    pos = arm.close_gripper()
    time.sleep(0.4)
    phase[0] = "GRIP: lifting"
    print("ยกขึ้น ...")
    arm.move_smooth({n: j0[n] for n in GRIP_J}, duration=dur, fine=True)
    phase[0] = "GRIP: back to scan (holding block)"
    print("กลับท่า scan (ถือบล็อกไว้) ...")
    arm.move_smooth({n: C.SCAN_POSE[n] for n in GRIP_J}, duration=3.0, fine=True)   # ไม่ส่ง gripper = ยังหนีบค้าง
    return pos


def main():
    global GRIP_MM_S, EXTRA_MM_S, EXTRA_DEPTH_MM
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="ไม่ส่งคำสั่งแขน")
    ap.add_argument("--slow", action="store_true", help="หนีบ/ยก ช้าลง (ทดสอบครั้งแรก)")
    ap.add_argument("--deeper", type=float, default=EXTRA_DEPTH_MM, help="ลงลึกกว่าท่าที่สอนอีกกี่ mm ก่อนหนีบ (ค่าเริ่มต้น 0 = ลึกเท่าที่สอน, สูงสุด 15)")
    ap.add_argument("--record", default=None, help="บันทึกวิดีโอหน้าต่าง (เช่น --record follow.mp4) ไว้ดูย้อนหลัง")
    ap.add_argument("--target-u", type=float, default=None, help="ตำแหน่งแนวนอนในภาพที่ต้องการให้บล็อกอยู่ (ค่าเริ่มต้น = กลางภาพ)")
    ap.add_argument("--no-return", action="store_true", help="ตอนออกให้ค้างแขนที่ตำแหน่งปัจจุบัน (ค่าเริ่มต้น: กลับท่า scan เอง)")
    ap.add_argument("--grab", action="store_true", help="บล็อกถึงกลางภาพและนิ่ง HOLD_S วินาที -> ลงไปหยิบ (ใช้ --dry เพื่อดูแผนโดยไม่ลง)")
    ap.add_argument("--vision-strict", action="store_true", help="กล้องฟันธงว่าบล็อกยังอยู่ที่เดิม (not_held) = ถือว่าหนีบพลาด ปล่อยแล้วหยุด (ค่าเริ่มต้น: แค่แสดง/บันทึก)")
    ap.add_argument("--yes", action="store_true", help="ข้ามการถามยืนยันก่อนเริ่ม (--grab)")
    ap.add_argument("--hold", type=float, default=HOLD_S, help="วินาทีที่ต้องนิ่งกลางภาพก่อนหยิบ (ค่าเริ่มต้น 1.0)")
    ap.add_argument("--pan-only", action="store_true", help="ควบคุมแค่ pan (แนวนอนของภาพ) ไม่ขยับ lift/elbow/wrist_flex")
    ap.add_argument("--target-v", type=float, default=None, help="ตำแหน่งแนวตั้งในภาพที่ต้องการให้บล็อกอยู่ (ค่าเริ่มต้น = กลางภาพ)")
    args = ap.parse_args()
    EXTRA_DEPTH_MM = max(0.0, min(args.deeper, 15.0))
    if args.slow:
        GRIP_MM_S, EXTRA_MM_S = 8.0, 4.0
    print("t = สอนท่าหนีบ (ครั้งเดียว)  |  SPACE = หนีบเดี๋ยวนี้  |  Esc = ออก" +
          (f"  | ช้า: ลง {GRIP_MM_S:.0f} mm/s" if args.slow else "") + f"  | ลงเพิ่ม {EXTRA_DEPTH_MM:.0f} mm")

    if not C.CALIBRATED["SCAN_POSE"]:
        raise SystemExit("ยังไม่ได้สอนท่า scan -> teach_scan.py")
    H = D.load_homography()
    cap = camera.open_camera()
    arm = None
    writer = None
    if not args.dry:
        from arm_select import make_arm
        arm = make_arm()
        arm.connect()
        try:
            import arm_mega as _am
            _am.IDLE_HOOK = lambda: cv2.waitKey(1)   # ระหว่างแขนขยับนานๆ (หยิบ/กลับท่า) ให้หน้าต่างยังตอบสนอง
        except ImportError:
            pass
    else:
        print("--dry: ไม่ส่งคำสั่งแขน (ใช้ค่าท่า scan เป็นตำแหน่งแขน)")
    scan = {j: C.SCAN_POSE[j] for j in K.JOINTS5}
    picker = None
    grabbed = False
    no_auto_return = False        # True = ผู้ใช้กด Ctrl+C / เริ่มหยิบแล้ว: ห้ามสั่งแขนกลับท่า scan เอง (ค้างตามที่อยู่)
    if args.grab:
        from picker import Picker, PickError
        picker = Picker(use_arm=False, cap=cap)               # ใช้กล้อง/H ที่เปิดไว้แล้ว ไม่เปิดซ้ำ
        picker.arm = arm
        from grip_vision import GripWatch
        picker.watch = GripWatch(cap, window="follow block (Esc = quit)", strict=args.vision_strict)   # กล้องเทียบภาพก่อนลง/หลังยก
        if arm is not None and not args.yes:
            if input("โหมดหยิบ: เมื่อบล็อกถึงกลางภาพและนิ่ง แขนจะ \"ลงไปหยิบเอง\" — มือและสายพ้นพื้นที่แล้ว? พิมพ์ y: ").strip().lower() != "y":
                raise SystemExit("ยกเลิก")
    try:
        if arm is not None:
            cur = {j: arm.read_joint(j) for j in K.JOINTS5}
            off = {j: cur[j] - scan[j] for j in K.JOINTS5}
            if max(abs(v) for v in off.values()) > POSE_TOL:
                raise SystemExit(f"แขนไม่ได้อยู่ท่า scan (คลาด {off}) -> รัน python go_scan.py ก่อน (หรือ read_joints.py ย้ายด้วยมือ)")
        wait_enter_live("เอาบล็อกและมือออกจากเลนให้ว่าง แล้วกด Enter เพื่อถ่ายพื้นหลัง ", cap)
        trk = D.BlockTracker(H)
        trk.set_background([f for f in (camera.grab(cap) for _ in range(20)) if f is not None])
        bg_saved = trk.bg                       # พื้นหลังถูกต้องเฉพาะ "ตอนกล้องอยู่ท่า scan" — พอแขนขยับภาพเลื่อน/หมุน ต้องปิดใช้
        print("พื้นหลังพร้อม — ใส่บล็อกในเลน / เลื่อนบล็อก (Esc = ออก, แขนจะค้างตำแหน่งปัจจุบัน)")
        tu = args.target_u
        tv = args.target_v
        _tfile = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "follow_target.json")
        if tu is None and tv is None and _os.path.exists(_tfile):      # จุดเป้าที่คลิกเลือกไว้ครั้งก่อน
            try:
                import json as _json
                with open(_tfile, encoding="utf-8") as _f:
                    _t = _json.load(_f)
                if "u" in _t:
                    tu, tv = float(_t["u"]), float(_t["v"])
                print(f"จุดเป้า (จาก follow_target.json): u={tu:.0f} v={tv:.0f} px  — คลิกซ้ายบนภาพเพื่อเปลี่ยน, คลิกขวา = กลางภาพ")
            except (OSError, ValueError, KeyError):
                pass
        else:
            print("จุดเป้า = กลางภาพ — คลิกซ้ายบนภาพตรง \"ช่องระหว่างนิ้ว gripper\" เพื่อตั้งเป้าใหม่ (จำไว้ใน follow_target.json)")
        _click = {}
        _aux = {"claw": None, "gripper": None, "white": None, "orange": None}   # ป้ายนิ้ว gripper + ค่าเทาของบล็อกแต่ละสี
        try:
            import json as _json
            with open(_tfile, encoding="utf-8") as _f:
                _t = _json.load(_f)
            for _k2 in _aux:
                if _t.get(_k2) is not None:
                    _aux[_k2] = _t[_k2]
        except (OSError, ValueError):
            pass

        def _save_target():
            import json as _json
            d = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), **{k_: v_ for k_, v_ in _aux.items() if v_ is not None}}
            if tu is not None and tv is not None and (abs(tu - 640) > 0.5 or abs(tv - 360) > 0.5):
                d.update(u=tu, v=tv)
            with open(_tfile, "w", encoding="utf-8") as _f:
                _json.dump(d, _f)

        def _apply_tones():
            if _aux["white"] is not None and _aux["orange"] is not None:
                D.BLOCK_WHITE_GRAY = (float(_aux["white"]) + float(_aux["orange"])) / 2.0   # ตัดสินสีจากค่ากลางของสองสีที่สอน
        _apply_tones()

        def _on_mouse(ev, x, y, flags, param):
            if ev == cv2.EVENT_LBUTTONDOWN:
                if flags & cv2.EVENT_FLAG_CTRLKEY:
                    _click["claw"] = (float(x), float(y))
                elif flags & cv2.EVENT_FLAG_SHIFTKEY:
                    _click["gripper"] = (float(x), float(y))
                else:
                    _click["set"] = (float(x), float(y))
            elif ev == cv2.EVENT_RBUTTONDOWN:
                _click["reset"] = True

        cv2.namedWindow("follow block (Esc = quit)")
        cv2.setMouseCallback("follow block (Esc = quit)", _on_mouse)
        last_send = 0.0
        last_show = 0.0
        fb_lost = None
        center_since, center_hist = None, []
        ref_zp, z_bad_since = None, None
        cmd_goal = {}                             # เป้าที่สั่งสะสมไว้ต่อข้อ (ล้างเมื่อหาบล็อกไม่เจอ)          # ความสูงอ้างอิงตอนเริ่มตาม / เวลาที่เริ่มเพี้ยน
        show_rej = False                          # กรอบแดงของที่ถูกปัดตก ซ่อนไว้ก่อน (กด d เพื่อดู)
        last_state = None
        hud = {"u_err": None, "v_err": None, "dq": None, "cmd": None}
        while True:
            D.watchdog_kick()
            g = camera.grab(cap)
            if g is None:                              # ยังไม่มีเฟรมใหม่ (กล้องสะดุด): ปั๊มหน้าต่างต่อ ไม่งั้น Windows ขึ้น Not Responding
                if cv2.waitKey(10) & 0xFF == 27:
                    break
                if getattr(cap, "age_s", 0) > 1.0 and time.perf_counter() - last_show > 0.5:
                    print(f"\r[กล้องไม่ส่งภาพ {cap.age_s:.1f}s — รอ/เปิดใหม่อัตโนมัติ]".ljust(80), end="", flush=True)
                    last_show = time.perf_counter()
                continue
            if tu is None:
                tu = g.shape[1] / 2.0
            if tv is None:
                tv = g.shape[0] / 2.0
            if _click:                                  # คลิกบนภาพ = ตั้งจุดเป้าใหม่ (บันทึกไว้ใช้ครั้งต่อไป)
                if _click.pop("reset", None):
                    tu, tv = g.shape[1] / 2.0, g.shape[0] / 2.0
                    try:
                        _os.remove(_tfile)
                    except OSError:
                        pass
                    print(f"\nจุดเป้า = กลางภาพ ({tu:.0f}, {tv:.0f})", flush=True)
                for _nm in ("claw", "gripper"):
                    if _nm in _click:
                        _aux[_nm] = list(_click.pop(_nm))
                        _save_target()
                        print(f"\nป้าย {'grip claw' if _nm == 'claw' else 'gripper'} ที่ ({_aux[_nm][0]:.0f}, {_aux[_nm][1]:.0f})", flush=True)
                if "set" in _click:
                    tu, tv = _click.pop("set")
                    _save_target()
                    print(f"\nจุดเป้าใหม่: u={tu:.0f} v={tv:.0f} px (บันทึก follow_target.json)", flush=True)
                cmd_goal.clear()
                center_since, center_hist = None, []
            res, state, roi = trk.update(g)
            if state != last_state:
                cmd_goal.clear()
                print(f"== {state} ==")
                last_state = state
            now = time.perf_counter()
            due = arm is None or (now - last_send >= 1.0 / SEND_HZ)         # ส่ง/คำนวณคำสั่งที่ SEND_HZ ไม่ใช่ทุกเฟรม
            joints = (read_joints_nb(arm) if arm is not None else dict(scan)) if due else None
            if due and arm is not None:
                if joints is None:                              # ฟีดแบ็กแขนขาด: ไม่สั่งอะไร ค้างตำแหน่งเดิม แต่ยังให้หน้าต่างทำงานต่อ
                    fb_lost = fb_lost or now
                    if now - fb_lost > FB_LOST_MAX_S:
                        print(f"\nหยุด: ไม่ได้ค่าตำแหน่งแขนนาน {FB_LOST_MAX_S:.0f}s (Mega/สาย/ไฟเลี้ยง/โปรแกรมอื่นใช้พอร์ต?) — ตรวจ check_arm.py", flush=True)
                        break
                else:
                    fb_lost = None
            line = ""
            show_due = (now - last_show) >= 1.0 / SHOW_HZ      # วาด/โชว์ภาพ/พิมพ์ที่ SHOW_HZ ไม่ใช่ทุกเฟรมกล้อง (กล้อง ~115 fps)
            if joints is not None:
                moved = max(abs(joints[j] - scan[j]) for j in TRACK_JOINTS)
                trk.bg = bg_saved if moved <= BG_VALID_TICKS else None   # แขนออกจากท่า scan -> ไม่ใช้ background (ใช้ open ใหญ่ + กัน glare แทน)
            centered = False
            if res is not None and state == "TRACK" and joints is not None:
                c = res["box_px"].mean(axis=0)
                e = block_error_mm(H, (float(c[0]), float(c[1])), (tu, tv))
                if ref_zp is None:                              # ความสูงอ้างอิง = ตอนเริ่มตามจริง (ไม่ใช่ท่า scan ในทฤษฎี: แขนยุบจากน้ำหนักได้ ~10 mm)
                    _ts = tip_state(joints)
                    ref_zp = (float(_ts[2]), float(_ts[3]))
                    print(f"\nความสูงอ้างอิง: ปลายแขน z={ref_zp[0]:.0f} mm (ท่า scan ในทฤษฎี {tip_state(scan)[2]:.0f} mm)", flush=True)
                dq, cmds = plan_follow(joints, scan, e, pan_only=args.pan_only, ref_zp=ref_zp)
                dz_now = float(tip_state(joints)[2] - ref_zp[0])
                if abs(dz_now) > Z_ABORT_MM and not args.pan_only:      # เพี้ยน "ต่อเนื่อง" เกิน Z_ABORT_S เท่านั้น (ค่าตำแหน่งพลาดครั้งเดียวไม่หยุด)
                    z_bad_since = z_bad_since or now
                    cmds = {}                                   # ระหว่างนี้ไม่สั่งขยับเพิ่ม
                    if now - z_bad_since >= Z_ABORT_S:
                        print(f"\nหยุด: ปลายแขนสูงเพี้ยน {dz_now:+.0f} mm จากตอนเริ่มตาม นาน {Z_ABORT_S:.1f}s (>{Z_ABORT_MM} mm) — ใช้ --pan-only หรือ go_scan.py", flush=True)
                        break
                else:
                    z_bad_since = None
                if math.hypot(c[0] - tu, (0 if args.pan_only else c[1] - tv)) <= HOLD_PX:
                    cmds = {}                                   # ใกล้เป้าแล้ว: ไม่ขยับ (คงเป้าสะสมเดิมไว้ให้ servo ยึด)
                else:
                    cmds = integrate_goals(cmds, joints, scan, cmd_goal)   # ชนะการยุบจากน้ำหนัก โดยไม่แกว่ง
                hud.update(u_err=float(c[0] - tu), v_err=float(c[1] - tv), dq=dq, cmd=cmds or None)
                centered = abs(c[0] - tu) <= CENTER_PX and (args.pan_only or abs(c[1] - tv) <= CENTER_PX)
                line = f"err=({c[0] - tu:+5.0f},{c[1] - tv:+5.0f})px  " + " ".join(f"{ABBR[j]}{v:+5.0f}" for j, v in dq.items())
                if cmds:
                    line += "  -> " + " ".join(f"{ABBR[j]}={g_}@{sp}" for j, (g_, sp) in cmds.items())
                    if arm is not None:
                        for j, (goal, speed) in cmds.items():
                            arm._check_limits({j: goal})
                            arm._moved.add(arm.ids[j])
                            arm._p(j, goal, speed)
                    last_send = time.perf_counter()
                if trk.stationary:
                    line += "  [บล็อกนิ่ง]"
                if show_due:
                    print(line.ljust(130), end="\r")
            # ---- ถึงกลางแล้วนิ่งครบเวลา -> หยิบ ----
            if args.grab and res is not None and joints is not None:
                if centered:
                    c0 = res["box_px"].mean(axis=0)
                    if center_since is None:
                        center_since, center_hist = now, []
                    center_hist.append((float(c0[0]), float(c0[1])))
                    xs_, ys_ = zip(*center_hist)
                    if max(xs_) - min(xs_) > JITTER_PX or max(ys_) - min(ys_) > JITTER_PX:
                        center_since, center_hist = now, [center_hist[-1]]     # ยังขยับอยู่ -> เริ่มนับใหม่
                    elif now - center_since >= args.hold:
                        B, ldeg = block_world(H, (float(c0[0]), float(c0[1])), res["long_deg"], joints, scan)
                        blk = {"x": float(B[0]), "y": float(B[1]), "long_deg": float(ldeg)}
                        print(f"\nนิ่งกลางภาพครบ {args.hold:.1f}s -> บล็อกที่ x={blk['x']:.1f} y={blk['y']:.1f} แกนยาว={blk['long_deg']:.0f}°")
                        try:
                            plan = picker.plan(blk)
                        except K.Unreachable as e:
                            print(f"หยิบไม่ได้ (เอื้อมไม่ถึง/ชนราง): {e}")
                            break
                        print("  hover:", {j: round(v) for j, v in plan["hover"].items()})
                        print("  grasp:", {j: round(v) for j, v in plan["down"][-1].items()})
                        if arm is None:
                            print("--dry: ไม่ลงไปหยิบ")
                            break
                        _v = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
                        cv2.putText(_v, "GRABBING ... (window pauses)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                        cv2.imshow("follow block (Esc = quit)", _v)
                        cv2.waitKey(1)
                        no_auto_return = True
                        try:
                            arm.open_gripper()
                            stall = picker.grasp(plan)
                            v = picker.vision or {}
                            print(f"หยิบเสร็จ — gripper หยุดที่ {stall} tick | กล้อง: {v.get('verdict', '-')} (เปลี่ยน {v.get('changed_frac', '-')}, "
                                  f"เห็นบล็อก {v.get('visible', '-')}) | กระแส gripper {picker.grip_current} | ภาพก่อน/หลังที่ grip_frames/")
                            if v.get("verdict") == "not_held":
                                print("  ! กล้องเห็นว่าบล็อกยังอยู่ที่เดิม — อาจหนีบพลาด (หรือค่าตัดสินของกล้องยังไม่ปรับ ดูภาพใน grip_frames/)")
                            grabbed = True
                        except PickError as e:
                            print("หยิบพลาด:", e)
                        break
                else:
                    center_since, center_hist = None, []
            if not show_due:
                continue
            last_show = now
            vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            col = (255, 255, 0) if state == "SEARCH" else (0, 140, 255)
            cv2.rectangle(vis, (roi[0], roi[1]), (roi[2] - 1, roi[3] - 1), col, 2)
            # crosshair เป้า: เส้นตั้ง = แนวที่ควบคุมจริง (u) / เส้นนอน = แนวตั้ง v ที่ควบคุมด้วย (ยกเว้น --pan-only)
            _h, _w = vis.shape[:2]
            _cx, _cy = int(tu), int(tv if tv is not None else _h // 2)
            cv2.line(vis, (_cx, 0), (_cx, _h), (0, 255, 255), 1)                         # เส้นตั้ง (แกนที่ควบคุม)
            cv2.line(vis, (0, _cy), (_w, _cy), (0, 255, 255), 1)                         # เส้นนอน
            cv2.circle(vis, (_cx, _cy), 40, (0, 255, 255), 1)                            # วงกลมรอบจุดตัด
            cv2.circle(vis, (_cx, _cy), 3, (0, 255, 255), -1)                            # จุดกลาง
            for rb, rl, rs in (D.LAST_REJECTS if show_rej else []):                 # กรอบแดง = เจอวัตถุแต่ขนาดไม่ใช่บล็อก (บอก mm ให้ดูว่าเพี้ยนเท่าไร)
                cv2.polylines(vis, [rb.astype(np.int32)], True, (0, 0, 255), 1)
                cv2.putText(vis, (f"con{rl:.0f}" if rs == -2 else f"tex{-rl:.0f}" if rl < 0 else f"{rl:.0f}x{rs:.0f}"), tuple(rb[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            if res is not None:
                cv2.polylines(vis, [res["box_px"].astype(np.int32)], True, (0, 255, 0), 2)
                bc = res["box_px"].mean(axis=0).astype(int)   # กากบาทเขียว = กลางบล็อกที่ตรวจเจอ -> แขนจะพากากบาทเหลืองมาทับจุดนี้
                cv2.drawMarker(vis, (int(bc[0]), int(bc[1])), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
                if res.get("gray") is not None:
                    cv2.putText(vis, f"gray {res['gray']:.0f} {res.get('tone') or ''}", (int(bc[0]) + 14, int(bc[1]) - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.line(vis, (int(bc[0]), int(bc[1])), (int(tu), int(tv)), (0, 255, 0), 1)
            cv2.putText(vis, f"{state}  {'DRY' if args.dry else 'LIVE'}  {'STATIONARY' if trk.stationary else ''}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
            if res is None:                                   # ไม่เห็นบล็อก: ล้างค่า HUD ให้เห็นว่าหยุดแล้ว
                hud.update(u_err=None, v_err=None, dq=None, cmd=None)
            rows = [f"err u,v: {hud['u_err']:+.0f}, {hud['v_err']:+.0f} px" if hud["u_err"] is not None else "err u,v: -",
                    ("fix: " + " ".join(f"{ABBR[j]}{v:+.0f}" for j, v in hud["dq"].items())) if hud["dq"] else "fix: -",
                    ("cmd: " + " ".join(f"{ABBR[j]}={g_}" for j, (g_, sp) in hud["cmd"].items())) if hud["cmd"] else "cmd: hold"]
            for i, t in enumerate(rows):
                cv2.putText(vis, t, (10, 62 + 28 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if args.grab and center_since is not None:
                cv2.putText(vis, f"CENTERED  hold {now - center_since:.1f}/{args.hold:.1f}s", (10, 62 + 28 * (len(rows) + 1)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            if fb_lost is not None:
                cv2.putText(vis, f"ARM FEEDBACK LOST {now - fb_lost:.1f}s", (10, 62 + 28 * len(rows)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if _HP is not None:                               # กล่องความสูง (จากขนาดบล็อกในภาพ + cam_focal.json)
                try:
                    _HP.draw_height_panel(vis, _HP.block_pose_px(res) if res is not None else None, None)
                except Exception:
                    pass
            if args.record:
                if writer is None:
                    writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (vis.shape[1], vis.shape[0]))
                writer.write(vis)
            for _nm, _lab in (("claw", "grip claw"), ("gripper", "gripper")):     # ป้ายนิ้ว gripper ที่คลิกไว้
                if _aux[_nm] is not None:
                    _p = (int(_aux[_nm][0]), int(_aux[_nm][1]))
                    cv2.circle(vis, _p, 9, (255, 0, 255), 2)
                    cv2.putText(vis, _lab, (_p[0] + 12, _p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5)
                    cv2.putText(vis, _lab, (_p[0] + 12, _p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
            if _aux["claw"] is not None and _aux["gripper"] is not None:
                _a, _b = np.array(_aux["claw"]), np.array(_aux["gripper"])
                _mid = ((_a + _b) / 2).astype(int)
                cv2.line(vis, tuple(_a.astype(int)), tuple(_b.astype(int)), (255, 0, 255), 1)
                cv2.drawMarker(vis, (int(_mid[0]), int(_mid[1])), (255, 0, 255), cv2.MARKER_DIAMOND, 18, 2)
                cv2.putText(vis, "between jaws (m = use as target)", (int(_mid[0]) + 12, int(_mid[1]) + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
            cv2.putText(vis, "t=teach grip  SPACE=grip  click=target  ctrl/shift+click=claw/gripper  m=target between jaws  w/o=white/orange",
                        (10, vis.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.imshow("follow block (Esc = quit)", vis)
            _k = cv2.waitKey(1) & 0xFF
            if _k == 27:
                break
            if _k in (ord("t"), ord(" ")) and arm is None:
                print("\n--dry: ไม่ได้ต่อแขน สอน/หนีบไม่ได้")
            elif _k == ord("t"):
                teach_grip(arm, cap, "follow block (Esc = quit)")
                cmd_goal.clear()
                center_since, center_hist = None, []
            elif _k == ord(" "):
                if res is None:
                    print("\nไม่หนีบ: ไม่เห็นบล็อก")
                else:
                    no_auto_return = True                 # เริ่มหนีบแล้ว: ถ้าล้มกลางทางห้ามกลับเอง
                    try:
                        _gr = grip_now(arm, cap, "follow block (Esc = quit)")
                    except Exception as _e:              # หนีบล้มกลางทาง: ไม่ปิดโปรแกรม/กล้อง
                        _gr = error_hold(arm, cap, "follow block (Esc = quit)", _e)
                    if _gr:
                        grabbed = True
                        break
                    no_auto_return = False
                    cmd_goal.clear()
                    center_since, center_hist = None, []
            if _k == ord("m") and _aux["claw"] is not None and _aux["gripper"] is not None:
                tu, tv = (_aux["claw"][0] + _aux["gripper"][0]) / 2.0, (_aux["claw"][1] + _aux["gripper"][1]) / 2.0
                _save_target()
                cmd_goal.clear()
                center_since, center_hist = None, []
                print(f"\nจุดเป้า = กึ่งกลางระหว่างนิ้ว gripper ({tu:.0f}, {tv:.0f})", flush=True)
            if _k in (ord("w"), ord("o")) and res is not None and res.get("gray") is not None:
                _aux["white" if _k == ord("w") else "orange"] = float(res["gray"])
                _apply_tones()
                _save_target()
                print(f"\nสอนสี: บล็อก{'ขาว' if _k == ord('w') else 'ส้ม'} = เทา {res['gray']:.0f}" +
                      (f" -> เกณฑ์ขาว/ส้ม = {D.BLOCK_WHITE_GRAY:.0f}" if _aux['white'] is not None and _aux['orange'] is not None
                       else " (สอนอีกสีด้วย)"), flush=True)
            if _k == ord("d"):                                  # d = เปิด/ปิดกรอบแดง (ของที่ถูกปัดตก: มือ/ราง/ผนัง) ไว้ debug
                show_rej = not show_rej
    except KeyboardInterrupt:
        no_auto_return = True                          # ผู้ใช้สั่งหยุดเอง: แขนค้างที่เดิม ไม่ขยับต่อ
        print("\nหยุดโดยผู้ใช้ — แขนค้างที่ตำแหน่งปัจจุบัน (ไม่กลับท่า scan เอง; รัน go_scan.py เมื่อพร้อม)")
    except Exception:
        no_auto_return = True                          # ล้มกลางทาง: ไม่สั่งแขนต่อเอง
        import os
        import traceback
        _tb = traceback.format_exc()
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "crash_log.txt"), "w", encoding="utf-8") as _f:
            _f.write(_tb)                                  # เก็บ traceback ไว้ในไฟล์ แม้หน้าต่างปิดไปแล้ว
        print("\n" + _tb)
        print("บันทึกข้อผิดพลาดที่ crash_log.txt แล้ว — ส่งไฟล์นี้ให้ดู")
    finally:
        D.watchdog_off()
        print()
        try:
            cv2.destroyAllWindows()                # ปิดหน้าต่างก่อนสั่งแขนกลับ (การกลับใช้เวลาหลายวินาที ถ้าหน้าต่างยังเปิดจะขึ้น Not Responding)
            cv2.waitKey(1)
        except Exception:
            pass
        try:
            if writer is not None:
                writer.release()
            if arm is not None:
                if not args.no_return and not grabbed and not no_auto_return:
                    # ออกจากโปรแกรมแล้วต้อง "กลับท่า scan" เสมอ: ไม่งั้นแขนค้างท่าที่ตามไป (ต่ำ/หมุน) แล้วสคริปต์ถัดไป (detect/homography) ผิดหมด
                    try:
                        print("กลับท่า scan ...")
                        arm.move_smooth({j: scan[j] for j in K.JOINTS5}, duration=3.0, fine=True)   # fine = ชดเชยการยุบเหมือน go_scan
                    except Exception as e:
                        print(f"กลับท่า scan ไม่สำเร็จ ({e}) -> ค้างแขนไว้ รัน go_scan.py เอง")
                arm.disconnect(hold=True)             # ค้างแขน
        finally:
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
