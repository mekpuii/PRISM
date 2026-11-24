"""ik_test.py — ทดสอบ IK โดยไม่ใช้กล้อง: สั่งจุดหนีบไปที่ (x, y) เหนือโต๊ะ แล้วดูว่าปลาย jaw ลงตรงมาร์กไหม

ใช้:  python ik_test.py 180 0                      # พิมพ์ผล IK อย่างเดียว (ปลอดภัย)
      python ik_test.py 180 50 --z 40 --yaw 30 --go   # สั่งแขนไปจริง (ถามยืนยันก่อน)
z = ความสูงจุดหนีบเหนือโต๊ะ (mm; ปลาย jaw อยู่ต่ำกว่านี้ 6 mm), yaw = ทิศที่ jaw ปิด (องศา ในกรอบหุ่น)
"""
import argparse
import config as C
import kinematics as K

ap = argparse.ArgumentParser()
ap.add_argument("x", type=float, help="mm ในกรอบหุ่น (+x ข้างหน้า)")
ap.add_argument("y", type=float, help="mm ในกรอบหุ่น (+y ซ้าย)")
ap.add_argument("--z", type=float, default=C.Z_HOVER_MM, help="ความสูงจุดหนีบเหนือโต๊ะ (mm)")
ap.add_argument("--yaw", type=float, default=90.0, help="ทิศที่ jaw ปิด (องศา)")
ap.add_argument("--go", action="store_true", help="สั่งแขนไปจริง")
a = ap.parse_args()
if a.z < C.Z_MIN_MM:
    raise SystemExit(f"z ต่ำกว่า Z_MIN_MM={C.Z_MIN_MM}")

# ---- คำนวณ IK แล้วเช็กย้อนด้วย FK ----
z_cmd = K.table_z(a.x, a.y) + a.z
motor = K.ik(a.x, a.y, z_cmd, a.yaw)
fx, fy, fz, pitch, closing = K.fk(motor)
print("IK ->", {j: round(v) for j, v in motor.items()})
print(f"FK check -> x={fx:.1f} y={fy:.1f} z={fz:.1f} | นิ้วก้ม {pitch:.1f}° (ควร -90) | jaw ปิดที่ {closing:.1f}°")

if a.go:
    from arm_select import make_arm
    if input("แขนจะขยับ — พร้อมไหม? พิมพ์ y: ").strip().lower() != "y":
        raise SystemExit("ยกเลิก")
    arm = make_arm()
    arm.connect()
    try:
        arm.open_gripper()
        arm.move_smooth(motor, duration=3.0, fine=True)
        print("ค่าคลาดสุดท้าย (tick):", arm.last_error)
        input("ถึงแล้ว — ดูตำแหน่งจริงเทียบมาร์ก แล้วกด Enter เพื่อจบ")
    finally:
        arm.disconnect()
