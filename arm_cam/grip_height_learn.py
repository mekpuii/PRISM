"""grip_height_learn.py — ให้ gripper "เรียนความสูงที่หนีบ" เอง โดยไม่ใช้ IK / TABLE_PLANE / homography

ขั้นตอน (แขนขยับจริง):
  1) คุณประคอง jaw (อ้าอยู่) ให้อยู่ "เหนือ" บล็อกบนสายพาน ~2-3 cm (ให้บล็อกอยู่ระหว่าง jaw สองข้าง) แล้วกด Enter
  2) ยกขึ้น 30 mm แล้ว "หนีบอากาศ" 1 ครั้ง -> tick_air (jaw ชนกันเอง) ใช้แยกหนีบอากาศ/หนีบบล็อก
  3) ลงทีละ STEP mm จากจุดเริ่ม: ทุกระดับ = หนีบ -> ยก 25 mm -> ตรวจว่าถือบล็อกไหม
        ติด = tick ที่ gripper หยุด ห่าง tick_air เกิน AIR_MARGIN  และ  บล็อกในภาพ "ไม่เล็กลง" ตอนยก (ยกไปด้วยกับกล้อง)
        (ถ้าไม่ได้ถือ บล็อกค้างบนสายพาน กล้องยกออก -> บล็อกในภาพเล็กลง ~20%)
     ติด -> วางคืน (ลงกลับ + เปิด) ; ไม่ติด -> เปิด แล้วลงต่ออีก STEP
  4) ระดับแรกที่ติด -> ลองซ้ำอีก 2 ครั้งที่ระดับเดียวกัน -> สรุป:
        grip_height.json : ลงจากจุดเริ่มกี่ mm, ปลาย jaw ลึกจากผิวบนบล็อกกี่ mm (ถ้ามี cam_focal.json), ท่าข้อต่อตอนหนีบ
        GRIP_TICK_RANGE  : ช่วง tick ตอนหนีบบล็อกจริง -> ถามก่อนบันทึกลง calib.json (visual_grab ใช้บอกว่า "ได้บล็อก")
ความปลอดภัย: ลงไม่เกิน --max-down mm จากจุดเริ่ม (ค่าเริ่มต้น 30 — จุดเริ่มต้องสูงกว่าผิวบล็อก 2-3 cm)
             ลงไม่ถึงเป้า (jaw ชนอะไร) = หนีบตรงนั้นแล้วไม่ลงต่ำกว่านั้นอีก และถ้ารู้ระยะเลนส์->ปลาย jaw จะไม่ให้ปลาย jaw ต่ำกว่าสายพาน + 3 mm
             Esc ที่หน้าต่างภาพ / Ctrl+C = หยุด (แขนค้างที่เดิม)

ใช้:  python grip_height_learn.py              (ปิดโปรแกรมอื่นที่ใช้พอร์ต Mega ก่อน)
      python grip_height_learn.py --step 1.5 --max-down 30
"""
import argparse
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
import kinematics as K
import visual_grab as V
from grip_observe import CAM_TO_TIP, FOCAL, dist_mm

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_JSON = os.path.join(HERE, "grip_height.json")
LOG_CSV = os.path.join(HERE, "grip_height_log.csv")

AIR_LIFT_MM = 30.0        # หนีบอากาศที่ความสูงนี้เหนือจุดเริ่ม
CHECK_LIFT_MM = 25.0      # หนีบแล้วยกเท่านี้เพื่อตรวจ
AIR_MARGIN = 60           # tick ที่หยุดห่าง tick_air อย่างน้อยเท่านี้ = มีของอยู่ระหว่าง jaw
HELD_SIZE_RATIO = 0.92    # หลังยก บล็อกในภาพต้องใหญ่ >= สัดส่วนนี้ของก่อนยก (ไม่ได้ถือ ~0.8)
CONFIRM = 2               # ระดับที่ติด ต้องติดซ้ำอีกกี่ครั้ง
BELT_CLEAR_MM = 3.0       # ปลาย jaw ต้องสูงกว่าสายพานอย่างน้อยเท่านี้ (ใช้ได้เมื่อรู้ cam_to_tip)
SPEED = 10.0              # mm/s ตอนขึ้นลง

_H = [None]
_stop = [False]
_key = [-1]


def ui(g=None, text=""):
    if g is not None:
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        p = V.detect(g, _H[0])
        if p is not None:
            cv2.polylines(vis, [np.array(p["box"], np.int32)], True, (0, 255, 0), 2)
            d = dist_mm(p["eq"])
            cv2.putText(vis, f"block {p['eq']:.0f}px" + (f"  cam->block {d:.0f}mm" if d else ""), (10, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis, text, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("grip height learn (Esc = stop)", vis)
    _key[0] = cv2.waitKey(1) & 0xFF
    if _key[0] == 27:
        _stop[0] = True
    return not _stop[0]


def live(cap, text):
    return lambda: ui(camera.grab(cap), text)


def block_size(cap, text, n=8):
    p = V.stable_pose(cap, _H[0], n=n, timeout=3.0, ui=lambda g: ui(g, text))
    return None if p is None else p["eq"]


def move_z(arm, cap, s_ref, dz, text):
    """เลื่อนจุดกึ่งกลาง jaw ไปที่ความสูง s_ref.z + dz (x, y, มุมก้ม เท่าจุดเริ่ม) — คืน True ถ้าถึง"""
    goal = np.array([s_ref[0], s_ref[1], s_ref[2] + dz, s_ref[3]])
    ok = V.move_cartesian(arm, goal, timeout=6.0, speed_mm_s=SPEED, ui=live(cap, text))
    if _stop[0]:
        raise KeyboardInterrupt
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", type=float, default=2.0, help="ลงทีละกี่ mm (ค่าเริ่มต้น 2)")
    ap.add_argument("--max-down", type=float, default=30.0, help="ลงจากจุดเริ่มได้ไม่เกินกี่ mm (ค่าเริ่มต้น 30)")
    args = ap.parse_args()

    from arm_select import make_arm
    _H[0] = D.load_homography() if os.path.exists(C.HOMOGRAPHY_FILE) else np.eye(3)
    cap = camera.open_camera()
    arm = make_arm()
    arm.connect()
    try:
        import arm_mega as _am
        _am.IDLE_HOOK = lambda: cv2.waitKey(1)
    except ImportError:
        pass
    rows = []
    try:
        arm.open_gripper()
        print("\n=== เรียนความสูงที่หนีบ ===")
        print("วางบล็อกบนสายพาน แล้วประคองแขน — torque ข้อ 1-5 จะถูกปิด (แขนจะอ่อน)")
        if input("ประคองแขนแล้ว? พิมพ์ y แล้ว Enter: ").strip().lower() != "y":
            return
        arm.torque_off(ids=(1, 2, 3, 4, 5))
        print("ขยับ jaw ที่อ้าอยู่ให้อยู่เหนือบล็อก ~2-3 cm (บล็อกอยู่ระหว่าง jaw สองข้าง) แล้วกด Enter ที่หน้าต่างภาพ")
        while True:
            ui(camera.grab(cap), "put OPEN jaws 2-3 cm ABOVE the block, press Enter")
            k = _key[0]
            if _stop[0]:
                arm.torque_on(ids=(1, 2, 3, 4, 5))
                return
            if k in (13, 10):
                break
        arm.torque_on(ids=(1, 2, 3, 4, 5))
        time.sleep(0.5)
        j0 = V.read_joints_nb(arm)
        s0 = V.state4(j0)
        eq0 = block_size(cap, "measuring start")
        d0 = dist_mm(eq0) if eq0 else None
        print(f"จุดเริ่ม: jaw กลางที่ z={s0[2]:.0f} mm ก้ม {s0[3]:.0f}°" + (f" | กล้อง->ผิวบล็อก {d0:.0f} mm" if d0 else ""))
        max_down = args.max_down
        if d0 is not None and CAM_TO_TIP is not None:
            tip_above_top = d0 - CAM_TO_TIP                          # ปลาย jaw สูงกว่าผิวบนบล็อกเท่านี้ (ตามที่วัด)
            lim = tip_above_top + C.BLOCK_H_MM - BELT_CLEAR_MM       # ลงได้ถึงสายพาน + 3 mm
            print(f"   ปลาย jaw สูงกว่าผิวบนบล็อก ~{tip_above_top:.0f} mm -> ลงได้ไม่เกิน {lim:.0f} mm (กันชนสายพาน)")
            if lim < 2:
                print("   คำนวณได้ว่าปลาย jaw ใกล้/ต่ำกว่าผิวบล็อกแล้ว — ตรวจ cam_to_tip_mm ใน cam_focal.json; ใช้แค่ --max-down")
            else:
                max_down = min(max_down, lim)
        # ---- 0) หนีบอากาศ ----
        move_z(arm, cap, s0, AIR_LIFT_MM, "lifting for air grip")
        tick_air = arm.close_gripper()
        arm.open_gripper()
        time.sleep(0.5)
        print(f"หนีบอากาศ: gripper หยุดที่ {tick_air} tick")
        move_z(arm, cap, s0, 0.0, "back to start")

        blocked = [False]

        def attempt(down):
            if not move_z(arm, cap, s0, -down, f"down {down:.1f} mm"):
                print(f"   ลง {down:.1f} mm ไม่ถึง (jaw ชนอะไร?) — หนีบที่ตำแหน่งนี้ และจะไม่ลงต่ำกว่านี้")
                blocked[0] = True
            eq_low = block_size(cap, f"at {down:.1f} mm", n=6)
            tick = arm.close_gripper()
            time.sleep(0.3)
            move_z(arm, cap, s0, -down + CHECK_LIFT_MM, "lift to check")
            time.sleep(0.4)
            eq_up = block_size(cap, "checking", n=6)
            has_obj = abs(tick - tick_air) >= AIR_MARGIN
            if eq_low and eq_up:
                ratio = eq_up / eq_low
                cam_ok = ratio >= HELD_SIZE_RATIO
            else:
                ratio, cam_ok = None, None                            # มองไม่เห็นตอนใดตอนหนึ่ง: ตัดสินด้วย tick อย่างเดียว
            held = has_obj and cam_ok is not False
            r = {"down_mm": round(down, 1), "tick": tick, "tick_air": tick_air, "size_ratio": None if ratio is None else round(ratio, 3),
                 "held": held, "time": time.strftime("%H:%M:%S")}
            rows.append(r)
            print(f"   ลง {down:4.1f} mm: gripper {tick} tick (อากาศ {tick_air})" +
                  (f", บล็อกในภาพตอนยก {ratio:.2f}x" if ratio else ", กล้องไม่เห็นบล็อก") + f" -> {'ติด' if held else 'ไม่ติด'}")
            # วางคืน / ปล่อย
            if held:
                move_z(arm, cap, s0, -down + 1.0, "putting back")
            arm.open_gripper()
            time.sleep(0.4)
            move_z(arm, cap, s0, 0.0, "back to start")
            return held

        found = None
        down = args.step
        while down <= max_down + 1e-6:
            if attempt(down):
                found = down
                break
            if blocked[0]:
                print("\nหยุดลง: jaw ลงต่อไม่ได้ (ชนบล็อก/สายพาน?) — ตรวจว่าบล็อกอยู่ระหว่าง jaw สองข้าง แล้วเริ่มใหม่")
                return
            down += args.step
        if found is None:
            print(f"\nลงถึง {max_down:.0f} mm แล้วยังหนีบไม่ติด — บล็อกอยู่ระหว่าง jaw ไหม? ลองเริ่มให้ตรงขึ้น หรือเพิ่ม --max-down")
            return
        ok_n = sum(attempt(found) for _ in range(CONFIRM))
        print(f"\nยืนยันที่ {found:.1f} mm: ติด {ok_n}/{CONFIRM}")
        if ok_n < CONFIRM:
            print("ไม่ติดทุกครั้ง -> ลองลงอีก 1 ขั้น")
            found2 = found + args.step
            if found2 <= max_down + 1e-6 and sum(attempt(found2) for _ in range(CONFIRM)) == CONFIRM:
                found = found2
            else:
                print("ยังไม่นิ่ง — ไม่บันทึก (ดู grip_height_log.csv)")
                return
        held_ticks = [r["tick"] for r in rows if r["held"] and abs(r["down_mm"] - found) < 1e-6]
        lo, hi = min(held_ticks) - 15, max(held_ticks) + 15
        tip_depth = None
        if d0 is not None and CAM_TO_TIP is not None:
            tip_depth = found - (d0 - CAM_TO_TIP)                      # ปลาย jaw ต่ำกว่าผิวบนบล็อกกี่ mm ตอนหนีบ
        # ท่าข้อต่อตอนหนีบ (คำนวณจากจุดเริ่ม - found)
        out = {"down_from_start_mm": found, "start_joints": j0, "start_state": [float(v) for v in s0],
               "grip_z_mm": float(s0[2] - found), "pitch_deg": float(s0[3]), "tick_air": tick_air,
               "grip_tick_range": [lo, hi], "tip_depth_into_block_mm": tip_depth,
               "cam_to_block_at_start_mm": d0, "step_mm": args.step, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
               "note": "ความสูงหนีบที่เรียนได้: ลงจากจุดเริ่ม down_from_start_mm; grip_z_mm = ความสูงจุดกึ่งกลาง jaw (FK) ตอนหนีบ"}
        with open(OUT_JSON, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print(f"\nเรียนได้: หนีบที่ลงจากจุดเริ่ม {found:.1f} mm (jaw กลางที่ z={s0[2] - found:.0f} mm)" +
              (f", ปลาย jaw ลึกจากผิวบนบล็อก ~{tip_depth:.0f} mm" if tip_depth is not None else ""))
        print(f"tick ตอนหนีบบล็อก: {held_ticks} -> GRIP_TICK_RANGE = [{lo}, {hi}]   (อากาศ = {tick_air})")
        print(f"บันทึก {OUT_JSON}")
        if input("บันทึก GRIP_TICK_RANGE ลง calib.json ด้วยไหม? (y/n): ").strip().lower() == "y":
            C.save_calib(GRIP_TICK_RANGE=[lo, hi])
            print("บันทึกแล้ว")
    except KeyboardInterrupt:
        print("\nหยุด — แขนค้างที่เดิม")
    finally:
        if rows:
            new = not os.path.exists(LOG_CSV)
            with open(LOG_CSV, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                if new:
                    w.writeheader()
                w.writerows(rows)
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        arm.disconnect(hold=True)
        cap.release()


if __name__ == "__main__":
    main()
