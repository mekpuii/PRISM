"""calib_zero.py — วัด JOINT_ZERO (tick ที่ "ท่า L") และ JOINT_DIR (ทิศที่ tick เพิ่ม) ของแขนตัวจริง แล้วบันทึกลง calib.json

ท่า L = q=0 ทุกข้อของ CAD (ดูรูป lpose.png):
  * ท่อนบน (ไหล่->ศอก) "ตั้งฉากกับโต๊ะ"      -> ใช้ไม้ฉากทาบสันหลังของท่อนบน
  * ท่อนล่าง (ศอก->ข้อมือ) "ขนานกับโต๊ะ" ชี้ไปข้างหน้า -> วัดความสูงหัว-ท้ายท่อนให้เท่ากัน
  * gripper ต่อเป็นแนวเดียวกับท่อนล่าง ชี้ไปข้างหน้า ขนานโต๊ะ
  * jaw สองข้างอยู่ "บน-ล่าง": jaw ตัวที่ขยับได้อยู่ด้านบน ตัวที่ไม่ขยับอยู่ด้านล่าง
  * ทั้งแขนชี้ตรงไปข้างหน้า ตั้งฉากกับขอบหน้าของฐาน
วัด 1 องศาผิด = ปลายแขนเพี้ยน ~5 mm -> ใช้ไม้ฉาก/ไม้บรรทัดจริงจัง

ใช้:  python calib_zero.py            (ทุกขั้น)
      python calib_zero.py --zero     (เฉพาะท่า L ไม่ทดสอบทิศ)
ต้องปิด autoen/dashboard ของ PRISM ก่อน และประคองแขนตลอด (torque ปิด)
"""
import statistics
import sys
import time

import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")

J5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# ทิศ "บวกของ URDF" ของแต่ละข้อ เมื่อเริ่มจากท่า L
PLUS = {
    "shoulder_pan": "หมุนฐานไปทาง 'ขวา' (ตามเข็มนาฬิกาเมื่อมองจากข้างบน)",
    "shoulder_lift": "โน้มท่อนบน 'ไปข้างหน้า' (ศอกเคลื่อนไปข้างหน้าและต่ำลง)",
    "elbow_flex": "กดปลายท่อนล่าง 'ลง' (ข้อมือต่ำลง)",
    "wrist_flex": "กดปลาย gripper 'ลง'",
    "wrist_roll": "บิด gripper 'ทวนเข็ม' เมื่อมองจากข้อมือไปทางปลาย jaw (ด้านบนของ gripper ไปทางซ้าย)",
}

arm = make_arm()
arm.connect()


def sample(joint, n=15):
    """ค่ากลางของ n ตัวอย่าง + ช่วงแกว่ง (ดูว่ามือนิ่งไหม)"""
    vals = []
    for _ in range(n):
        vals.append(arm.read_joint(joint))
        time.sleep(0.05)
    return statistics.median(vals), max(vals) - min(vals)


zero, direction = {}, dict(C.JOINT_DIR)
try:
    input("แขนจะถูกปล่อย torque (อ่อน) — ประคองแขนไว้ แล้วกด Enter ")
    arm.torque_off()
    time.sleep(0.5)

    # ---------- 1) ท่า L: อ่านทุกข้อพร้อมกัน ----------
    input("\n1) วางแขนใน 'ท่า L' ตามรูป (ไม้ฉากทาบ) ถือให้นิ่ง แล้วกด Enter ")
    for j in J5:
        z, spread = sample(j)
        zero[j] = float(z)
        dev = (z - 2048) / C.UNITS_PER_DEG
        flag = "" if abs(dev) < 8 else "   <-- ห่างจาก 2048 มาก: ตรวจท่าอีกครั้ง"
        print(f"   {j:14s} tick={z:6.0f}  (ห่างจาก 2048 = {dev:+.1f}°, มือแกว่ง {spread} tick){flag}")

    # ---------- 2) ทิศของแต่ละข้อ ----------
    if "--zero" not in sys.argv:
        print("\n2) ทดสอบทิศ: ขยับ 'ข้อเดียว' ไปตามที่บอก ~30° จากท่า L แล้วกด Enter (ข้ออื่นปล่อยตามสบายได้)")
        for j in J5:
            input(f"   {j}: {PLUS[j]}  แล้วกด Enter ")
            t, _ = sample(j)
            d = t - zero[j]
            if abs(d) < 120:
                print(f"      ! เปลี่ยนแค่ {d:+.0f} tick (30° ≈ 341) — ใช้ทิศเดิมจาก config ({direction[j]:+d}); รันใหม่ถ้าไม่แน่ใจ")
            else:
                direction[j] = 1 if d > 0 else -1
                print(f"      DIR = {direction[j]:+d}  (เปลี่ยน {d:+.0f} tick ≈ {abs(d) / C.UNITS_PER_DEG:.0f}°)")
            input("      คืนข้อนี้กลับท่า L แล้วกด Enter ")
finally:
    try:
        arm.torque_on()                    # ค้างตำแหน่งปัจจุบัน (ไม่กระโดด)
    finally:
        arm.disconnect(hold=False)

print("\nผล:")
print("  JOINT_ZERO =", {j: round(v) for j, v in zero.items()})
print("  JOINT_DIR  =", direction)
if input("บันทึกลง calib.json? พิมพ์ y: ").strip().lower() == "y":
    C.save_calib(JOINT_ZERO=zero, JOINT_DIR=direction)
    print("บันทึกแล้ว ->", C.CALIB_FILE)
