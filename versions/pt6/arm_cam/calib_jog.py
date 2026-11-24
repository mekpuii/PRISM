"""calib_jog.py — สร้าง homography.npy ใหม่ "โดยให้แขนขยับ" แทนการย้ายบล็อก (ใช้แทน calib_touch.py ได้เมื่อเลนแคบ)

ปัญหาของ calib_touch.py บนสายพาน: บล็อกอยู่ในเลนแคบ (เหลือที่ข้างละ ~2 mm) จุดกลางบล็อกทุกจุดเลยเรียงเป็นเส้นเดียว
-> homography เพี้ยนในแนวขวางเลน (RMS สูง, ill-conditioned)
วิธีนี้: วางบล็อก "จุดเดียว" นิ่งๆ ที่ท่า scan -> แขนเลื่อนปลาย ±STEP_MM ในแกน x และ y ของแขน (ความสูง/มุมก้มเท่าเดิม)
-> ดูว่าบล็อกในภาพเลื่อนกี่ px ไปทางไหน -> ได้ความสัมพันธ์ pixel <-> mm ครบ 2 แกนที่ท่า scan (ไม่ขึ้นกับความกว้างเลน)
   กล้องเลื่อน +d  = บล็อกเลื่อน -d เมื่อมองจากกล้อง  =>  Δpx = -A·Δtip,  homography = A⁻¹ (เชิงเส้น รอบกลางภาพ)
ใช้ได้ดีกับ follow_block.py (ต้องการแค่ "ทิศ/ระยะ" ของความคลาด) และการเช็กขนาดบล็อก (40x30 mm)
พิกัด x,y สัมบูรณ์เป็นค่าประมาณ (ยึดว่ากลางภาพ = ตำแหน่งปลาย jaw ตอนท่า scan) — ไม่ใช่สำหรับหยิบด้วย IK

ใช้:  python go_scan.py   แล้ว   python calib_jog.py         (แขนขยับจริง ±15 mm — มือพ้นแขน)
      python calib_jog.py --step 20                         (เลื่อนมากขึ้น = แม่นขึ้นถ้าบล็อกยังอยู่ในภาพ)
"""
import argparse
import json
import math
import os
import shutil
import time

import cv2
import numpy as np

import camera
import config as C
import detect_block as D
import kinematics as K
import visual_grab as V

HERE = os.path.dirname(os.path.abspath(__file__))


def measure(cap, H, label):
    p = V.stable_pose(cap, H, n=12, timeout=4.0, ui=lambda g: show(g, label))
    if p is None:
        raise RuntimeError(f"มองไม่เห็นบล็อก ({label}) — บล็อกหลุดภาพ? ลด --step")
    return p


_H = [np.eye(3)]


def show(g, text):
    if g is None:
        cv2.waitKey(1)
        return
    vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    p = V.detect(g, _H[0])
    if p is not None:
        cv2.polylines(vis, [np.array(p["box"], np.int32)], True, (0, 255, 0), 2)
    cv2.putText(vis, text, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow("calib jog", vis)
    cv2.waitKey(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=15.0, help="เลื่อนปลายแขนข้างละกี่ mm (ค่าเริ่มต้น 15)")
    args = ap.parse_args()

    from arm_select import make_arm
    Hold = D.load_homography() if os.path.exists(C.HOMOGRAPHY_FILE) else np.eye(3)
    _H[0] = Hold
    cap = camera.open_camera()
    arm = make_arm()
    arm.connect()
    try:
        import arm_mega as _am
        _am.IDLE_HOOK = lambda: cv2.waitKey(1)
    except ImportError:
        pass
    try:
        scan = {n: C.SCAN_POSE[n] for n in K.JOINTS5}
        j = {n: arm.read_joint(n) for n in K.JOINTS5}
        off = max(abs(j[n] - scan[n]) for n in K.JOINTS5)
        if off > 40:
            raise SystemExit(f"แขนไม่ได้อยู่ท่า scan (คลาด {off:.0f} tick) -> python go_scan.py ก่อน")
        input("วางบล็อก 1 ก้อนในเลน ให้อยู่แถวกลางภาพ ห้ามขยับระหว่างวัด แล้วกด Enter (แขนจะขยับ ±%.0f mm) " % args.step)
        j0 = V.read_joints_nb(arm)
        s0 = V.state4(j0)
        p0 = measure(cap, Hold, "measuring at scan pose")
        print(f"ที่ท่า scan: บล็อกที่ u={p0['u']:.0f} v={p0['v']:.0f} px, ขนาด {p0['long_px']:.0f}x{p0['short_px']:.0f} px")
        dtip, dpx = [], []
        for ax, name in ((0, "x"), (1, "y")):
            for sgn in (+1, -1):
                goal = s0.copy()
                goal[ax] += sgn * args.step
                print(f"เลื่อนปลายแขน {name} {sgn * args.step:+.0f} mm ...")
                V.move_cartesian(arm, goal, timeout=6.0, speed_mm_s=20.0, ui=lambda: (show(camera.grab(cap), f"move {name} {sgn * args.step:+.0f} mm"), True)[1])
                time.sleep(0.6)
                jm = V.read_joints_nb(arm)
                sm = V.state4(jm)
                pm = measure(cap, Hold, f"measuring {name}{sgn:+d}")
                dtip.append(sm[:2] - s0[:2])
                dpx.append(np.array([pm["u"] - p0["u"], pm["v"] - p0["v"]]))
                print(f"   ปลายแขนเลื่อนจริง ({dtip[-1][0]:+.1f}, {dtip[-1][1]:+.1f}) mm -> บล็อกในภาพเลื่อน ({dpx[-1][0]:+.0f}, {dpx[-1][1]:+.0f}) px"
                      f"  | z เพี้ยน {sm[2] - s0[2]:+.1f} mm")
                V.move_cartesian(arm, s0, timeout=6.0, speed_mm_s=20.0, ui=lambda: (show(camera.grab(cap), "back to scan"), True)[1])
                time.sleep(0.4)
        # ---- Δpx = -A·Δtip  ->  แก้ A (2x2) แบบ least squares จาก 4 การเลื่อน ----
        T = np.array(dtip)                      # 4x2
        P = np.array(dpx)                       # 4x2
        if float(np.abs(P).max()) < 5.0:
            print("บล็อกในภาพแทบไม่เลื่อนเลยตอนแขนขยับ — แขนขยับจริงไหม? (torque ปิด / ชนขอบ) หรือกล้องไม่ได้ติดกับแขน — ไม่บันทึก")
            return
        if abs(float(np.linalg.det(T.T @ T))) < 1e-6:
            print("ปลายแขนไม่ได้เลื่อนครบทั้งแกน x และ y — ไม่บันทึก")
            return
        At, *_ = np.linalg.lstsq(T, -P, rcond=None)   # P = -T @ A.T
        A = At.T                                # px ต่อ mm
        pred = -T @ A.T
        resid = np.linalg.norm(P - pred, axis=1)
        if abs(float(np.linalg.det(A))) < 1e-6:
            print("คำนวณไม่ได้ (การเลื่อนในภาพอยู่แนวเดียว) — ไม่บันทึก")
            return
        Ainv = np.linalg.inv(A)                 # mm ต่อ px
        sv = np.linalg.svd(A, compute_uv=False)
        print(f"\nสเกล: {sv[0]:.2f} / {sv[1]:.2f} px ต่อ mm (สองแกน) | คลาดของโมเดลต่อการเลื่อน {resid.round(1).tolist()} px")
        # ตรวจ: ขนาดบล็อก (mm) จากโมเดลใหม่ ต้องใกล้ 40x30
        box = np.array(p0["box"], float)
        e0, e1 = Ainv @ (box[1] - box[0]), Ainv @ (box[2] - box[1])
        L, S = sorted([float(np.linalg.norm(e0)), float(np.linalg.norm(e1))], reverse=True)
        print(f"ตรวจ: ขนาดบล็อกจากโมเดลใหม่ = {L:.1f} x {S:.1f} mm (จริง 40 x 30)")
        # ---- homography (affine): กลางภาพ -> ตำแหน่งปลาย jaw ตอนท่า scan (ค่าสัมบูรณ์โดยประมาณ) ----
        h, w = 720, 1280
        c = np.array([w / 2.0, h / 2.0])
        t = s0[:2] - Ainv @ c
        Hn = np.array([[Ainv[0, 0], Ainv[0, 1], t[0]], [Ainv[1, 0], Ainv[1, 1], t[1]], [0.0, 0.0, 1.0]])
        ok = abs(L - C.BLOCK_LONG_MM) < 8 and abs(S - C.BLOCK_SHORT_MM) < 8 and float(resid.max()) < 25
        if not ok:
            print("ผลไม่น่าเชื่อ (ขนาดบล็อกห่าง 40x30 เกิน 8 mm หรือคลาดเกิน 25 px) — ไม่บันทึก. บล็อกขยับระหว่างวัด? ลอง --step 10")
            return
        if os.path.exists(C.HOMOGRAPHY_FILE):
            shutil.copy(C.HOMOGRAPHY_FILE, os.path.join(HERE, time.strftime("homography_before_jog_%Y%m%d_%H%M%S.npy")))
        np.save(C.HOMOGRAPHY_FILE, Hn)
        with open(os.path.join(HERE, "calib_jog.json"), "w", encoding="utf-8") as f:
            json.dump({"A_px_per_mm": A.tolist(), "moves_mm": T.tolist(), "moves_px": P.tolist(), "resid_px": resid.tolist(),
                       "block_mm": [L, S], "scan_pose": scan, "step_mm": args.step, "time": time.strftime("%Y-%m-%d %H:%M:%S")},
                      f, ensure_ascii=False, indent=1)
        print(f"บันทึก {C.HOMOGRAPHY_FILE} แล้ว (ของเดิมสำรองเป็น homography_before_jog_*.npy)")
        print("ต่อไป: python follow_block.py  (กด Enter ถ่ายพื้นหลังใหม่ตอนเลนว่าง)")
    finally:
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        arm.disconnect(hold=True)
        cap.release()


if __name__ == "__main__":
    main()
