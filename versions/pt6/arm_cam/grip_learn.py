"""grip_learn.py — เรียนรู้ "ความสูงที่หนีบ" (Z_GRASP_MM) และ "tick ที่ gripper หยุดเมื่อหนีบบล็อกจริง" (GRIP_TICK_RANGE) เอง
โดยให้กล้องช่วยตัดสินว่าหนีบติดไหม (grip_vision.py)

วิธี (ปลอดภัยไว้ก่อน: เริ่มจาก "สูง" ลงมาทีละ STEP mm ไม่เคยลงต่ำกว่า Z_MIN_MM):
  0) วัด tick ตอนหนีบ "อากาศ" (stall_air) ที่ท่า scan  -> ค่านี้ = jaw ชนกันเอง ใช้แยกหนีบอากาศออกจากหนีบบล็อก
  1) แต่ละความสูงในบันได: scan -> หา -> ลง -> หนีบ -> ยกกลับ hover -> กล้องเทียบภาพก่อน/หลัง -> ถ้าติด วางคืนที่เดิม
        "ติด" = tick ที่หยุดสูงกว่า stall_air + AIR_MARGIN (หนีบโดนของ) และกล้องไม่ฟันธงว่า not_held
  2) เจอความสูงแรกที่ติด (z1) -> ลองต่ำลงอีก 2 mm สองรอบ (z1-2) ถ้าติดทั้งคู่ใช้ z1-2 ไม่งั้นใช้ z1  (ต่ำกว่า = หนีบกินตัวบล็อกมากกว่า)
  3) แสดงตาราง -> ถามก่อนบันทึก Z_GRASP_MM + GRIP_TICK_RANGE ลง calib.json   (บันทึกทุกรอบที่ grip_log.csv, ภาพกล้องที่ grip_frames/)
ใช้:  python grip_learn.py            ต้องวางบล็อกบนที่โล่ง (ไม่ใช่ในราง) ห่างราง/ผนัง; แขนต้องผ่าน go_scan.py แล้ว
"""
import csv
import time

import config as C
from grip_vision import GripWatch
from picker import Picker

LADDER_TOP_MM = 18.0       # จุดเริ่ม (grasp point สูงจากโต๊ะ) — สูงพอที่ jaw จับแค่ส่วนบนของบล็อก 20 mm
STEP_MM = 2.0
AIR_MARGIN_TICKS = 100     # tick ที่หยุด <= stall_air + นี้ = หนีบอากาศ
TICK_PAD = 15              # ขอบเผื่อเมื่อสรุปเป็น GRIP_TICK_RANGE
MAX_ATTEMPTS = 12


def ladder(z_min=None):
    z_min = C.Z_MIN_MM if z_min is None else z_min
    out, z = [], LADDER_TOP_MM
    while z >= z_min - 1e-9:
        out.append(round(z, 1))
        z -= STEP_MM
    if out[-1] > z_min + 1e-9:
        out.append(float(z_min))
    return out


def is_ok(row):
    """รอบนี้หนีบติดไหม (ตามนิยามด้านบน)"""
    return bool(row["grasped"]) and not row["air"] and row["vision"] != "not_held"


def decide(rows, z_min=None):
    """รับผลทุกรอบเรียงตามเวลา -> (z ที่เลือก หรือ None, tick_range หรือ None, เหตุผล)
    ใช้ฟังก์ชันนี้ทดสอบแยกได้ (ไม่แตะฮาร์ดแวร์)"""
    z_min = C.Z_MIN_MM if z_min is None else z_min
    ok = [r for r in rows if is_ok(r)]
    if not ok:
        return None, None, "ไม่มีรอบไหนหนีบติดเลย (ลองดูภาพ grip_frames/ และ grip_log.csv; ตรวจว่าตำแหน่งหยิบตรงบล็อกไหม)"
    z1 = ok[0]["z"]                                        # ความสูงแรก (สูงสุด) ที่ติด
    z2 = max(z1 - STEP_MM, z_min)
    at_z2 = [r for r in rows if abs(r["z"] - z2) < 1e-6]
    chosen = z1
    why = f"ติดครั้งแรกที่ z={z1:g}"
    if z2 < z1 - 1e-6 and len(at_z2) >= 2 and all(is_ok(r) for r in at_z2[:2]):
        chosen = z2
        why += f" และติดซ้ำ 2/2 ที่ z={z2:g} -> เลือกต่ำลง 1 ขั้น (กินตัวบล็อกมากขึ้น)"
    elif z2 < z1 - 1e-6:
        why += f" แต่ z={z2:g} ไม่ติดครบ 2/2 -> ใช้ z={z1:g}"
    rng = None
    if len(ok) >= 2:
        s = [r["stall"] for r in ok]
        rng = [int(min(s)) - TICK_PAD, int(max(s)) + TICK_PAD]
    return chosen, rng, why


def main():
    if C.ARM_BACKEND != "mega":
        raise SystemExit("ใช้กับ ARM_BACKEND='mega' เท่านั้น")
    print(__doc__)
    if input("วางบล็อกบนที่โล่ง ห่างราง มือพ้นพื้นที่แล้ว? แขนจะลงหนีบหลายรอบ พิมพ์ y: ").strip().lower() != "y":
        raise SystemExit("ยกเลิก")
    p = Picker()
    import cv2
    win = "grip learn (camera)"
    p.watch = GripWatch(p.cap, window=win, strict=False)      # strict=False: แค่ดู ไม่ให้กล้องตัดสินแทนระหว่างเรียนรู้
    rows = []
    try:
        blk = p.scan()
        if blk is None:
            raise SystemExit("ไม่เจอบล็อกที่ท่า scan — ดู detect_block.py")
        # 0) หนีบอากาศ: วัด stall_air
        print("วัดค่าหนีบอากาศ (ที่ท่า scan ไม่มีอะไรอยู่ใน jaw) ...")
        stall_air = p.arm.close_gripper()
        p.arm.open_gripper()
        print(f"  stall_air = {stall_air} tick (jaw ชนกันเอง)")
        zs = ladder()
        print("บันไดความสูง (mm เหนือโต๊ะ):", zs)
        queue = list(zs)
        confirming = False
        attempts = 0
        while queue and attempts < MAX_ATTEMPTS:
            z = queue.pop(0)
            attempts += 1
            blk = p.scan()
            if blk is None:
                print("ไม่เห็นบล็อกแล้ว — หยุด")
                break
            row = {"t": time.strftime("%H:%M:%S"), "z": z, "x": round(blk["x"], 1), "y": round(blk["y"], 1),
                   "grasped": 0, "stall": None, "air": None, "vision": "", "changed": None, "current": None, "note": ""}
            try:
                plan = p.plan(blk, z_grasp_mm=z)
                stall = p.grasp(plan)
                row.update(grasped=1, stall=stall, air=int(stall <= stall_air + AIR_MARGIN_TICKS),
                           vision=(p.vision or {}).get("verdict", ""), changed=(p.vision or {}).get("changed_frac"),
                           current=p.grip_current)
            except Exception as e:
                row["note"] = f"{type(e).__name__}: {e}"
                if isinstance(e, KeyboardInterrupt):
                    raise
            rows.append(row)
            ok = is_ok(row)
            print(f"  z={z:>4g}  stall={row['stall']}  air={row['air']}  camera={row['vision'] or '-'}"
                  f" (changed {row['changed']})  current={row['current']}  -> {'ติด' if ok else 'ไม่ติด'} {row['note']}")
            if row["grasped"]:
                if ok:
                    time.sleep(0.4)
                    p.put_back(plan)                       # ถือบล็อกอยู่: ลงวางที่เดิม ปล่อย แล้วยก
                else:
                    p.arm.open_gripper()                   # หนีบอากาศ/ไม่ติด: เปิดที่ hover (บล็อกยังอยู่บนโต๊ะ)
            # ตัดสินใจรอบถัดไป: ลงบันไดไปเรื่อยๆ จนเจอที่ติดครั้งแรก (z1) แล้วทดสอบ z1-2 สองรอบ แล้วจบ
            if row["grasped"] and ok and not confirming:
                z2 = max(z - STEP_MM, C.Z_MIN_MM)
                confirming = True
                queue = [z2, z2] if z2 < z - 1e-6 else []     # ติดที่ขั้นต่ำสุดแล้ว = ไม่มีขั้นต่ำกว่า
            if confirming and not queue:
                break
    except KeyboardInterrupt:
        print("\nหยุดโดยผู้ใช้ — แขนค้างตำแหน่งปัจจุบัน")
    finally:
        cv2.destroyAllWindows()
        p.close()
    if not rows:
        raise SystemExit("ไม่มีข้อมูล")
    with open("grip_log.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    z, rng, why = decide(rows)
    print("\n" + "=" * 70)
    print("สรุป:", why)
    if z is None:
        return
    print(f"  Z_GRASP_MM = {z:g}  (เดิม {C.Z_GRASP_MM:g})")
    print(f"  GRIP_TICK_RANGE = {rng}  (จาก tick ที่หนีบติดจริง {len([r for r in rows if is_ok(r)])} รอบ)" if rng
          else "  GRIP_TICK_RANGE: ข้อมูลไม่พอ (ต้องติด >= 2 รอบ) — ไม่บันทึก")
    if input("บันทึกลง calib.json? พิมพ์ y: ").strip().lower() == "y":
        kv = {"Z_GRASP_MM": float(z)}
        if rng:
            kv["GRIP_TICK_RANGE"] = rng
        C.save_calib(**kv)
        print("บันทึกแล้ว ->", C.CALIB_FILE)


if __name__ == "__main__":
    main()
