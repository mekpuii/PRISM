"""teach_rail.py — สอนตำแหน่ง "รางนำสายพาน" ให้โปรแกรมรู้ (แตะ 3 จุดด้วยปลาย jaw) + บันทึกระยะอ้าของ gripper

ทำหลัง calib_touch.py (ต้องมี TABLE_PLANE ที่วัดบนผิวสายพาน)  ใช้:  python teach_rail.py
แตะด้วย "มุมด้านในของปลาย jaw ตัวที่ไม่ขยับ" (จุดเดียวกับ calib_touch) นิ้วชี้ลงดิ่ง:
  จุด 1 = มุมบนด้านในของผนังราง "ฝั่งใกล้แขน" ตรงปลายราง (ปลายที่บล็อกวิ่งออก)
  จุด 2 = มุมบนด้านในของผนังราง "ฝั่งไกลแขน" ตรงปลายราง
  จุด 3 = บนขอบบนด้านในของผนังฝั่งใกล้แขน ลึกเข้าไปในราง 6-10 cm จากจุด 1
ไม่มีราง (ทดสอบบนโต๊ะโล่ง): ไม่ต้องรันไฟล์นี้  |  ลบรางที่สอนไว้: python teach_rail.py --clear
"""
import statistics
import sys
import time

import numpy as np

import config as C
import kinematics as K
from arm_select import make_arm

if "--clear" in sys.argv:
    C.save_calib(RAIL=None)
    raise SystemExit("ลบ RAIL แล้ว (โปรแกรมจะไม่ตรวจชนราง)")
if not C.CALIBRATED["TABLE_PLANE"]:
    raise SystemExit("ต้องทำ calib_touch.py ก่อน (ใช้ระดับผิวสายพานเป็นฐานวัดความสูงผนัง)")

# ---------- ระยะอ้าของ gripper (วัดด้วยไม้บรรทัด) ----------
cur = C.GRIPPER_OPEN_MM
txt = input(f"ระยะระหว่างผิวแผ่น jaw สองข้างที่ปลาย ตอน gripper เปิดสุดของโปรแกรม (รัน test_gripper.py เพื่อเปิด) หน่วย mm "
            f"[ค่าเดิม {cur}] Enter = ข้าม: ").strip()
if txt:
    v = float(txt)
    if not 35.0 <= v <= 110.0:
        raise SystemExit(f"{v} mm ไม่สมเหตุผล (คาด 45-80) — วัดใหม่")
    C.save_calib(GRIPPER_OPEN_MM=v)
    print(f"บันทึก GRIPPER_OPEN_MM = {v}")

arm = make_arm()
arm.connect()


def read_touch():
    """ค่ากลางของ 15 ตัวอย่างทุกข้อ -> ตำแหน่งจุดแตะ (x, y, z) mm"""
    m = {}
    for j in K.JOINTS5:
        vals = []
        for _ in range(15):
            vals.append(arm.read_joint(j))
            time.sleep(0.02)
        m[j] = statistics.median(vals)
    return np.array(K.fk(m, "touch")[:3], float)


names = ["1: มุมบนด้านในผนัง 'ฝั่งใกล้แขน' ที่ปลายราง", "2: มุมบนด้านในผนัง 'ฝั่งไกลแขน' ที่ปลายราง",
         "3: ขอบบนด้านในผนังฝั่งใกล้แขน ลึกเข้าราง 6-10 cm"]
P = []
try:
    input("ประคองแขน แล้วกด Enter (ข้อ 1-5 จะอ่อน) ")
    arm.torque_off(ids=(1, 2, 3, 4, 5))
    for n in names:
        input(f"แตะจุด {n}  ถือนิ่ง แล้วกด Enter ")
        p = read_touch()
        print(f"   x={p[0]:.1f} y={p[1]:.1f} z={p[2]:.1f}  (สูงจากผิวสายพาน {p[2] - K.table_z(p[0], p[1]):.1f} mm)")
        P.append(p)
    input("ยกแขนขึ้นให้พ้นราง ~10 cm ถือไว้ แล้วกด Enter (torque จะกลับมา) ")
finally:
    try:
        arm.torque_on(ids=(1, 2, 3, 4, 5))
    finally:
        arm.disconnect(hold=False)

p1, p2, p3 = P
w = p2[:2] - p1[:2]
gap = float(np.linalg.norm(w))
w /= gap
d3 = p3[:2] - p1[:2]
along = d3 - (d3 @ w) * w                                  # ส่วนของ (จุด3 - จุด1) ที่ตั้งฉากกับแนวขวางช่อง = ทิศเข้าหาตัวราง
depth = float(np.linalg.norm(along))
h = [float(p[2] - K.table_z(p[0], p[1])) for p in P]
z_top = max(h)
print(f"\nช่องกว้าง {gap:.1f} mm (CAD 34) | ผนังสูง {h[0]:.1f} / {h[1]:.1f} / {h[2]:.1f} mm (CAD 10.6) | จุด 3 ลึก {depth:.0f} mm, เยื้องขวาง {abs(float(d3 @ w)):.1f} mm")
bad = []
if not 28.0 <= gap <= 45.0:
    bad.append("ช่องกว้างไม่ใกล้ 34 mm")
if not 5.0 <= z_top <= 25.0 or max(h) - min(h) > 4.0:
    bad.append("ความสูงผนังไม่ใกล้ 10.6 mm หรือสามจุดสูงไม่เท่ากัน")
if depth < 40.0:
    bad.append("จุด 3 ใกล้จุด 1 เกินไป (ต้องลึกเข้าราง >= 4 cm)")
if abs(float(d3 @ w)) > 6.0:
    bad.append("จุด 3 ไม่ได้อยู่บนผนังเดียวกับจุด 1")
if bad:
    raise SystemExit("ไม่บันทึก: " + "; ".join(bad) + " — แตะใหม่")
rail = {"p1": [float(p1[0]), float(p1[1])], "p2": [float(p2[0]), float(p2[1])],
        "up": [float(along[0] / depth), float(along[1] / depth)], "z_top": z_top}
print("RAIL =", rail)
if input("บันทึกลง calib.json? พิมพ์ y: ").strip().lower() == "y":
    C.save_calib(RAIL=rail)
    print("บันทึกแล้ว — pick_block.py / pick_moving.py จะเลี่ยงรางเอง")
