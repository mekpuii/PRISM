"""pick_moving.py — หยิบบล็อกขณะสายพานเดิน (ดู MOVING.md)

ใช้:  python pick_moving.py --measure      วัดความเร็ว/ทิศของบล็อกอย่างเดียว (แขนแค่ไปท่า scan) — ทำอันนี้ก่อนเสมอ
      python pick_moving.py --dry          ซ้อม: ไปดักที่ความสูง hover ปิด jaw ในอากาศตรงเวลา (ไม่ลง ไม่แตะบล็อก)
      python pick_moving.py                หยิบจริง
      python pick_moving.py --place        หยิบจริงแล้วไปวางที่ท่าวาง
      python pick_moving.py --loop         ทำซ้ำไปเรื่อยๆ (Ctrl-C หยุด)
      --yes = ไม่ถามยืนยันก่อนขยับ   --no-rail = ยืนยันว่าไม่มีรางนำ (ไม่ได้สอนด้วย teach_rail.py)
"""
import argparse

import config as C
from intercept import Interceptor

ap = argparse.ArgumentParser()
for f in ("--measure", "--dry", "--place", "--loop", "--yes", "--no-rail"):
    ap.add_argument(f, action="store_true")
args = ap.parse_args()

if not all(C.CALIBRATED.values()):                       # ใช้ค่าสอบเทียบชุดเดียวกับหยิบนิ่ง
    raise SystemExit(f"ยังสอบเทียบไม่ครบ {C.CALIBRATED} — ทำขั้น 1-6 ใน GUIDE.html ให้ผ่านก่อน")
if not args.measure and not C.GRIP_TICK_RANGE:
    raise SystemExit("ยังไม่มี GRIP_TICK_RANGE — รัน repeat_test.py (หยิบนิ่ง) ให้ผ่านก่อน")
if not args.measure and not C.RAIL and not args.no_rail:
    raise SystemExit("ยังไม่ได้สอนรางนำ (teach_rail.py) — โปรแกรมจะไม่รู้ว่าตรงไหนมีราง  ถ้าไม่มีรางจริงๆ ให้เพิ่ม --no-rail")
if not args.yes and input("แขนจะขยับ: พื้นที่โล่ง มือพร้อมที่ปุ่มหยุด/ปลั๊ก? พิมพ์ y: ").strip().lower() != "y":
    raise SystemExit("ยกเลิก")

p = Interceptor()
try:
    while True:
        if args.measure:
            p.goto_scan()
            print("รอบล็อกเข้ามาในเฟรม ...")
            trk = p.track() if p.wait_block(60.0) else None
            if trk is None:
                print("  ไม่เห็นบล็อก/เห็นไม่พอ")
            else:
                vx, vy = trk["v"]
                print(f"  ความเร็ว {trk['speed']:.1f} mm/s  (vx {vx:+.1f}, vy {vy:+.1f})  "
                      f"fit คลาด {trk['rms']:.2f} mm จาก {trk['n']} เฟรมใน {trk['span_s']:.2f} s")
                print(f"  ระยะที่บล็อกวิ่งระหว่างแขนไปดัก ({C.INTERCEPT_ARM_TIME_S} s) = {trk['speed'] * C.INTERCEPT_ARM_TIME_S:.0f} mm"
                      f"  | ยอมคลาดเวลาได้ ~±{13.0 / max(trk['speed'], 1e-6):.2f} s")
        else:
            print("รอบล็อกเข้ามาในเฟรม ...")
            r = p.pick_moving(dry=args.dry)
            print("ผล:", "สำเร็จ" if r["ok"] else "ไม่สำเร็จ", "|", r.get("reason", ""))
            for k in ("speed", "rms", "x", "y", "t_hit_in", "slack_s", "late_s", "stall"):
                if r.get(k) is not None:
                    print(f"   {k:9s} = {r[k]:.3f}" if isinstance(r[k], float) else f"   {k:9s} = {r[k]}")
            if r["ok"] and not args.dry:
                if args.place:
                    p.place()
                else:
                    input("ถือบล็อกอยู่ — รับบล็อกแล้วกด Enter เพื่อปล่อย ")
                    p.arm.open_gripper()
        if not args.loop:
            break
finally:
    p.close()
