"""check_arm.py — ตรวจสถานะแขนแบบ "อ่านอย่างเดียว" (ไม่ส่งคำสั่งขยับ ไม่เปิด/ปิด torque ไม่ส่ง H ตอนจบ)

พิมพ์ตำแหน่ง tick ของแต่ละข้อต่อเทียบขอบเขต SAFE_MIN/MAX และระยะห่างจากปลายช่วง
ใช้ก่อนทุกครั้งที่จะสั่งแขนขยับ: ถ้ามีข้อต่อ "นอกช่วง" แปลว่าแขนอยู่ท่าพัก/cradle — ยังห้ามสั่งขยับ

ใช้:  python check_arm.py
"""
import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")

arm = make_arm()
arm.connect()
try:
    rows = arm.status()
finally:
    arm.disconnect(hold=False)           # อ่านอย่างเดียว: ไม่แตะแขนตอนปิด

print(f"{'joint':14s} {'tick':>5s} {'min':>5s} {'max':>5s}  เหลือถึงปลายช่วง   สถานะ")
bad = False
for j, v, lo, hi, ok in rows:
    room = min(v - lo, hi - v)
    print(f"{j:14s} {v:5d} {lo:5d} {hi:5d}  {room:+6d} tick ({room / C.UNITS_PER_DEG if C.UNITS_PER_DEG != 1 else room:+.0f}°)"
          f"   {'ok' + (' (ใกล้ปลายช่วง)' if room < 100 else '') if ok else 'นอกช่วง!'}")
    bad |= not ok
if bad:
    print("\n! มีข้อต่อนอกช่วง = น่าจะอยู่ท่าพัก/cradle: รัน  python wake_from_rest.py  ให้เข้าช่วงก่อน (arm_mega จะไม่ยอมขยับให้)")
else:
    print("\nทุกข้อต่ออยู่ในช่วง — พร้อมสำหรับขั้นถัดไป")
