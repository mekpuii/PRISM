"""wake_from_rest.py — พาแขนออกจากท่าพัก (cradle) เข้าช่วง SAFE ด้วยลำดับ wake ที่เพื่อนสอนไว้ใน PRISM

ทำไมไม่ขยายช่วง limit: เฟิร์มแวร์ Mega บีบค่าเกิน SAFE_MIN/MAX เงียบๆ -> ขยายที่ Python อย่างเดียวไม่ได้ผล
และท่าพักอยู่ใกล้ปลายกลไก; ทางที่ถูกคือเดินตามลำดับที่เคยสอนไว้ (ข้อ 2 ขยับก่อนและลำพัง กันชนโครง)

ก่อนรัน: ประคองแขนด้วยมือ (torque ข้อ 2,3,5 จะถูกเปิดที่ตำแหน่งปัจจุบัน แขนอาจทรุดเล็กน้อย)
          เคลียร์พื้นที่รอบแขน ปิด autoen/dashboard ของ PRISM
ทุกสเต็ปถามยืนยันก่อน (Enter = ไป, q = หยุด) ความเร็ว 120 tick/s ~ 10 องศา/วินาที
ใช้:  python wake_from_rest.py
"""
import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")

arm = make_arm()
arm.connect()
try:
    # ---- แสดงสถานะก่อนเริ่ม ----
    for j, v, lo, hi, ok in arm.status():
        print(f"  {j:14s} {v:5d}  ช่วง {lo}-{hi}  {'ok' if ok else 'นอกช่วง'}")
    if input("\nประคองแขนแล้ว พื้นที่โล่ง พร้อมปลุกแขน? พิมพ์ y: ").strip().lower() != "y":
        raise SystemExit("ยกเลิก")

    def confirm(name, tgt):
        """ถามก่อนทุกสเต็ป แล้วคืน True/False"""
        print(f"\nสเต็ป {name}: ไปที่ {tgt}")
        return input("  Enter = ไป, q = หยุด: ").strip().lower() != "q"

    arm.wake(confirm)                       # torque ON ข้อ 2,3,5 -> unstep4..unstep1
    print("\nเสร็จ — แขนอยู่ท่า start ของ PRISM (ในช่วง SAFE) สถานะตอนนี้:")
    for j, v, lo, hi, ok in arm.status():
        print(f"  {j:14s} {v:5d}  {'ok' if ok else 'นอกช่วง!'}")
finally:
    arm.disconnect()                        # H ค้างเฉพาะข้อที่เราสั่งขยับ
