"""go_scan.py — พาแขนจากท่าปัจจุบันไปท่า scan (SCAN_POSE) ช้าๆ อย่างปลอดภัย
ใช้เมื่อแขนหย่อน/ถูกขยับ/torque หลุด (เช่น ไฟเลี้ยงสะดุด) แล้วจะรัน calib_touch.py / pick_block.py ต่อ
  1) เปิด torque ข้อ 1-6 "ที่ตำแหน่งปัจจุบัน" (ไม่กระโดด)  2) แสดงตำแหน่งปัจจุบันเทียบท่า scan  3) ถามยืนยัน  4) ไปช้าๆ 5 วินาที
ใช้:  python go_scan.py
"""
import time

import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")
if not C.CALIBRATED["SCAN_POSE"]:
    raise SystemExit("ยังไม่ได้สอนท่า scan -> teach_scan.py")

arm = make_arm()
arm.connect()
try:
    input("ประคองแขนไว้ — จะเปิด torque ที่ตำแหน่งปัจจุบัน แล้วกด Enter ")
    arm.torque_on(ids=(1, 2, 3, 4, 5, 6))        # ค้างที่ตำแหน่งปัจจุบัน (ถ้าก่อนหน้านี้ torque หลุดอยู่ จะกลับมาจับ)
    time.sleep(0.6)
    cur = arm.read()
    print(f"{'joint':14s}{'ตอนนี้':>8s}{'ท่า scan':>10s}{'ต่าง':>8s}")
    for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"):
        print(f"{j:14s}{cur[j]:8d}{C.SCAN_POSE[j]:10.0f}{C.SCAN_POSE[j] - cur[j]:+8.0f}")
    if input("ไปท่า scan ช้าๆ (5 วินาที) ไหม? พิมพ์ y: ").strip().lower() != "y":
        raise SystemExit("ยกเลิก (แขนค้างท่าปัจจุบัน)")
    arm.move_smooth(C.SCAN_POSE, duration=5.0, fine=True)
    print("ถึงท่า scan แล้ว:", arm.read())
finally:
    arm.disconnect(hold=False)
