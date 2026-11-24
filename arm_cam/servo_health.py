"""servo_health.py — ดูอุณหภูมิ/แรงดัน/กระแสของ servo ทุกตัว และ "รีเซ็ต torque" ข้อที่ค้าง (ไม่ขยับทั้งที่สั่ง)

ทำไม: ถ้า servo เข้าโหมดป้องกัน (ร้อน/โอเวอร์โหลด) มันตัด torque เอง แต่เฟิร์มแวร์ Mega ยังคิดว่า torque เปิดอยู่
      -> Q,id,1 ไม่ทำอะไร (มันทำเฉพาะตอนที่ตัวเองจำว่าปิดอยู่) และ P ถูกส่งแต่ servo ไม่ตาม
      ทางแก้: สั่ง "ปิดแล้วเปิดใหม่" (Q,id,0 -> Q,id,1) ให้เฟิร์มแวร์เขียน torque ใหม่ที่ตำแหน่งปัจจุบัน

ใช้:  python servo_health.py                  (อ่านอย่างเดียว ~8 วินาที)
      python servo_health.py --retorque 4     (ปิด-เปิด torque ข้อ 4 — ประคองแขนไว้! ข้อนั้นจะหย่อนชั่วครู่)
"""
import sys
import time

import config as C
from arm_select import make_arm

arm = make_arm()
arm.connect()
try:
    print("รอค่าอุณหภูมิ/แรงดัน (ส่งมาช้า ~ทุกไม่กี่วินาที) ...")
    time.sleep(8)
    print(f"{'ID':>3s} {'joint':14s}{'tick':>6s}{'อุณหภูมิ':>10s}{'แรงดัน V':>10s}{'กระแส mA~':>11s}")
    names = {v: k for k, v in C.MEGA_JOINT_ID.items()}
    for sid in range(1, 7):
        st = arm.link.servo[sid]
        print(f"{sid:3d} {names[sid]:14s}{str(st.pos):>6s}{(str(st.temp_c) + ' C') if st.temp_c is not None else '-':>10s}"
              f"{(f'{st.volt_dv / 10:.1f}') if st.volt_dv is not None else '-':>10s}{abs(st.current_raw) * 6.5:11.0f}")
    print("\nแนวทางอ่าน: อุณหภูมิ > ~65 C = ใกล้เข้าโหมดป้องกัน (พักให้เย็น) | แรงดันต่ำกว่า ~11 V ตอนมีโหลด = ไฟเลี้ยงไม่พอ")
    if "--retorque" in sys.argv:
        sid = int(sys.argv[sys.argv.index("--retorque") + 1])
        j = names[sid]
        if input(f"ประคองแขนไว้! จะปิด-เปิด torque ข้อ {sid} ({j}) พิมพ์ y: ").strip().lower() == "y":
            arm.torque_off(ids=(sid,))
            time.sleep(0.5)
            arm.torque_on(ids=(sid,))
            time.sleep(0.8)
            print("เสร็จ — ตำแหน่งตอนนี้:", arm.read_joint(j), "| แล้วรัน go_scan.py ต่อ")
finally:
    arm.disconnect(hold=False)
