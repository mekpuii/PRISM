"""teach_place.py — สอน "ท่าวาง" (จุดที่จะปล่อยบล็อก เช่น เหนือกล่อง/ถาด) แล้วบันทึกลง calib.json
ท่าวางเป็นท่าข้อต่อที่สอนด้วยมือ (แบบเดียวกับ bin ของ PRISM) จึงวางนอกโซนที่หยิบแนวดิ่งได้

ข้อควรรู้: ตอนใช้งาน แขนจะเคลื่อนจาก "เหนือจุดหยิบ" มาท่านี้เป็นเส้นตรงในปริภูมิข้อต่อ
-> สอนให้ปลาย jaw อยู่สูงพอ (>= 6-8 cm เหนือโต๊ะ/ขอบกล่อง) และไม่มีสิ่งกีดขวางระหว่างทาง
ใช้:  python teach_place.py      แล้วทดสอบด้วย  python pick_block.py --place
"""
import config as C
import kinematics as K
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")
MARGIN = 40                                    # ต้องห่างขอบ SAFE อย่างน้อยเท่านี้ (tick)

arm = make_arm()
arm.connect()
saved = None
try:
    input("ข้อ 1-5 จะอ่อน — ประคองแขนไว้ แล้วกด Enter ")
    arm.torque_off(ids=(1, 2, 3, 4, 5))
    while True:
        input("จับแขนไปท่าที่ต้องการปล่อยบล็อก ถือนิ่ง แล้วกด Enter ")
        pos = {j: int(arm.read_joint(j)) for j in K.JOINTS5}
        x, y, z, pitch, _ = K.fk(pos, "touch")
        bad = [j for j in K.JOINTS5 if not (C.JOINT_LIMITS[j][0] + MARGIN <= pos[j] <= C.JOINT_LIMITS[j][1] - MARGIN)]
        print(f"  ท่า: {pos}\n  ปลาย jaw อยู่ที่ x={x:.0f} y={y:.0f} z={z - K.table_z(x, y):.0f} mm เหนือโต๊ะ (ก้ม {pitch:.0f}°)")
        if bad:
            print("  ! นอก/ชิดขอบช่วง:", bad, "— ขยับแล้วลองใหม่")
            continue
        if input("  ใช้ท่านี้? y = บันทึก, Enter = ลองใหม่: ").strip().lower() == "y":
            saved = pos
            break
finally:
    try:
        arm.torque_on(ids=(1, 2, 3, 4, 5))     # ค้างท่าปัจจุบัน (ไม่กระโดด)
    finally:
        arm.disconnect(hold=False)

if saved:
    C.save_calib(PLACE_POSE=saved)
    print("บันทึก PLACE_POSE แล้ว")
