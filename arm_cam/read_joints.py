"""read_joints.py — ปล่อย torque ข้อ 1-5 แล้วพิมพ์ค่า tick + ตำแหน่งปลาย jaw (FK) สดๆ ขณะจับแขนขยับด้วยมือ
ใช้ตรวจว่า FK ตรงกับของจริงไหม: เอาปลาย jaw ตายตัวแตะจุดที่รู้ระยะ (เช่น 200 mm หน้าแกนฐาน) แล้วดู x,y,z
กด Ctrl+C เพื่อออก (torque กลับมาค้างท่าปัจจุบัน)
"""
import time
import config as C
import kinematics as K
from arm_select import make_arm

arm = make_arm()
arm.connect()
input("ข้อ 1-5 จะอ่อน — ประคองแขนไว้ แล้วกด Enter ")
arm.torque_off(ids=(1, 2, 3, 4, 5))
print("torque OFF — จับแขนขยับได้เลย (Ctrl+C เพื่อออก)\n")
try:
    while True:
        m = {j: arm.read_joint(j) for j in K.JOINTS5}
        x, y, z, pitch, _ = K.fk(m, "touch")
        print("  ".join(f"{j[:8]}={m[j]:5d}" for j in K.JOINTS5) +
              f"  | ปลาย jaw x={x:6.1f} y={y:6.1f} z={z:6.1f} ก้ม={pitch:4.0f}°   ", end="\r")
        time.sleep(0.2)
except KeyboardInterrupt:
    print()
finally:
    try:
        arm.torque_on(ids=(1, 2, 3, 4, 5))   # เปิด torque กลับที่ตำแหน่งปัจจุบัน (ไม่กระโดด)
    except Exception as e:
        print(f"\nเปิด torque กลับไม่สำเร็จ ({e}) — ประคองแขนไว้!")
    arm.disconnect(hold=False)
