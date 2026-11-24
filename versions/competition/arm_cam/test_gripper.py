"""test_gripper.py — ทดสอบ gripper (ข้อ 6) ข้อเดียว: เปิด torque ข้อ 6 แล้วขยับไปทีละสเต็ปเล็กๆ ช้าๆ
ไม่แตะข้อ 1-5 เลย  ใช้ตรวจว่า gripper รับคำสั่ง P หรือไม่ (ครั้งก่อนมันไม่ขยับ)
ใช้:  python test_gripper.py
"""
import time
import config as C
from arm_select import make_arm

arm = make_arm()
arm.connect()
try:
    cur = arm.read()["gripper"]
    lo, hi = C.JOINT_LIMITS["gripper"]
    print(f"gripper ตอนนี้ = {cur}  (ช่วง {lo}-{hi})")
    arm.link.write("Q,6,1")                     # เปิด torque ข้อ 6 ที่ตำแหน่งปัจจุบัน (ไม่กระโดด)
    time.sleep(1.0)
    for tgt in (cur + 150, cur + 300, 2484):    # ขยับทีละนิดขึ้นไปจนถึงค่า start ของ PRISM
        tgt = max(lo, min(hi, int(tgt)))
        input(f"\nEnter = ไป {tgt} (Ctrl+C หยุด): ")
        arm._drive({"gripper": tgt}, {"gripper": 150}, 12)
        print("  ถึง", arm.read()["gripper"])
finally:
    arm.disconnect()                           # H ค้างเฉพาะข้อที่สั่ง (gripper ไม่อยู่ใน H)
