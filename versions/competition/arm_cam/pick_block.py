"""pick_block.py — โปรแกรมหลัก: ไปท่า scan -> เห็นบล็อกในเฟรม -> ลงไปหยิบ -> ยก -> วางคืน/วางที่ท่าวาง
ต้องทำมาก่อน: calib_zero.py -> teach_scan.py -> calib_touch.py   (ผลอยู่ใน calib.json + homography.npy)

ใช้:  python pick_block.py --dry-run     # ตรวจจับ + คำนวณ IK แล้วพิมพ์แผน ไม่ลงไปหยิบ (ปลอดภัย)
      python pick_block.py               # หยิบจริง (ถามยืนยันก่อนลง) แล้ววางคืนที่เดิม
      python pick_block.py --place       # หยิบแล้วไปวางที่ท่าวาง (สอนด้วย teach_place.py)
      python pick_block.py --keep        # หยิบแล้วถือค้าง
      python pick_block.py --loop        # วนรอ: เห็นบล็อกเมื่อไรก็หยิบ (Ctrl+C ออก)
      python pick_block.py --yes         # ข้ามการถามยืนยัน (ใช้เมื่อมั่นใจแล้วเท่านั้น)
      python pick_block.py --save-debug  # บันทึก debug.png (เหมาะกับ Pi ที่ไม่มีจอ)
      python pick_block.py --no-arm --dry-run   # ทดสอบกล้อง+IK โดยไม่ต่อแขน (แขนต้องอยู่ท่า scan เอง)
"""
import argparse
import time

import cv2

import camera
import config as C
import detect_block as D
import kinematics as K
from picker import Picker, PickError

ap = argparse.ArgumentParser()
ap.add_argument("--dry-run", action="store_true", help="ไม่ลงไปหยิบ")
ap.add_argument("--no-arm", action="store_true", help="ไม่ต่อแขนเลย (ใช้คู่ --dry-run)")
ap.add_argument("--keep", action="store_true", help="ถือบล็อกค้าง")
ap.add_argument("--place", action="store_true", help="วางที่ท่าวาง (PLACE_POSE)")
ap.add_argument("--loop", action="store_true", help="วนหยิบเรื่อยๆ")
ap.add_argument("--yes", action="store_true", help="ข้ามการถามยืนยัน")
ap.add_argument("--save-debug", action="store_true", help="บันทึก debug.png")
args = ap.parse_args()
if args.no_arm and not args.dry_run:
    raise SystemExit("--no-arm ใช้ได้กับ --dry-run เท่านั้น")
if args.place and not C.PLACE_POSE:
    raise SystemExit("ยังไม่ได้สอนท่าวาง — รัน teach_place.py ก่อน")

missing = [k for k, ok in C.CALIBRATED.items() if not ok]
if missing:
    print(f"! ยังไม่ได้สอบเทียบ: {missing} — ผลจะไม่แม่น (ลำดับ: calib_zero.py -> teach_scan.py -> calib_touch.py)")
    if not args.dry_run:
        raise SystemExit("ไม่หยิบจริงจนกว่าจะสอบเทียบครบ (ใช้ --dry-run ดูแผนได้)")

p = Picker(use_arm=not args.no_arm)


def once():
    """หนึ่งรอบ  คืน True ถ้าเจอบล็อก (ไม่ว่าหยิบได้หรือไม่), False ถ้าไม่เจอ"""
    block = p.scan()
    if args.save_debug:
        _, vis, _ = D.find_block(camera.grab(p.cap), p.H, debug=True)
        cv2.imwrite("debug.png", vis)
    if block is None:
        print("ไม่เจอบล็อกในเฟรม (หรือค่าสั่นเกิน) — ดูด้วย detect_block.py / set_roi.py")
        return False
    try:
        plan = p.plan(block)
    except K.Unreachable as e:
        print(f"เห็นบล็อกที่ x={block['x']:.0f} y={block['y']:.0f} แต่หยิบไม่ได้ (แขนยังไม่ขยับลง): {e}")
        return True
    print(f"บล็อกที่ x={block['x']:.1f} y={block['y']:.1f} mm | แกนยาว={block['long_deg']:.1f}° | jaw ปิดที่ {plan['closing']:.1f}° "
          f"| ภาพแกว่ง {block['std_mm']:.2f} mm | โต๊ะ z={K.table_z(block['x'], block['y']):+.1f}")
    print("  hover:", {j: round(v) for j, v in plan["hover"].items()})
    print("  grasp:", {j: round(v) for j, v in plan["down"][-1].items()})
    if args.dry_run:
        return True
    if not args.yes and input("แขนจะลงไปหยิบ — มือพ้นพื้นที่แล้ว? พิมพ์ y: ").strip().lower() != "y":
        raise SystemExit("ยกเลิก")
    try:
        stall = p.grasp(plan)
    except PickError as e:
        print("  หยิบพลาด:", e)
        return True
    print(f"  หยิบได้ — gripper หยุดที่ {stall} tick | ค่าคลาดที่จุดหนีบ (tick): {p.grasp_error}")
    if args.place:
        p.place()
        print("  วางที่ท่าวางแล้ว")
    elif not args.keep:
        time.sleep(0.8)
        p.put_back(plan)
        print("  วางคืนที่เดิมแล้ว")
    return True


try:
    while True:
        found = once()
        if not args.loop or (args.keep and found and not args.dry_run):
            break
        time.sleep(1.0)
except KeyboardInterrupt:
    print("\nหยุดโดยผู้ใช้")
finally:
    p.close()
