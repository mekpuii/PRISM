"""calib_limits.py — วัดช่วงขยับจริงของข้อ 1-5 (จับแขนขยับด้วยมือ) แล้วเสนอ/บันทึกขอบเขต JOINT_LIMITS ใหม่

ทำไมต้องมี: ขอบเขต SAFE_MIN/MAX ตอนนี้เป็นของเพื่อน (เฟิร์มแวร์ Mega) ไม่ได้วัดจากแขนตัวนี้ในท่าที่เราใช้
  เช่น ท่า L ของ wrist_roll = 145 แต่ SAFE_MIN = 298 -> ขึ้น "นอกขอบเขต" ตลอด

ขอบเขตอยู่ 2 ที่ และต้องไม่ขัดกัน:
  1) ฝั่ง Python (calib.json: JOINT_LIMITS)  — โปรแกรมของเราตรวจก่อนส่งคำสั่ง
  2) เฟิร์มแวร์ Mega (SAFE_MIN/SAFE_MAX ใน .ino) — "บีบค่าเงียบๆ" ถ้าเกิน
  สคริปต์นี้บันทึกฝั่ง Python ให้ "ไม่กว้างกว่าเฟิร์มแวร์" เท่านั้น (กว้างกว่า = แขนไม่ไปตามที่สั่งโดยไม่มีแจ้ง)
  ส่วนที่กว้างกว่าเฟิร์มแวร์ จะพิมพ์บรรทัด .ino ให้ + เขียนลง limits_proposal.txt ต้อง flash Mega เองแล้วค่อยรัน
  python calib_limits.py --fw-flashed  เพื่อบันทึกค่ากว้าง

ใช้:  python calib_limits.py                (วัด+เสนอ)
      python calib_limits.py --fw-flashed   (วัดแล้วบันทึกเต็มช่วง หลัง flash Mega ด้วยค่าใหม่แล้ว)
วิธีวัด: ทีละข้อ ค่อยๆ ขยับไปสุดทั้งสองด้านที่ "จะให้โปรแกรมใช้" แล้วกด Ctrl+C
  * อย่าฝืนเกินจุดที่รู้สึกชนทางกล / สายตึง   * อย่ารวมท่าพักบน cradle (เพื่อให้ guard ท่าพักยังทำงาน)
  * wrist_roll หมุนได้เกือบรอบ: หมุนเฉพาะช่วงที่ต้องใช้ และอย่าข้ามค่า 0/4095
ต้องปิด autoen/dashboard ของ PRISM ก่อน
"""
import sys
import time

import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")

J5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
FW_FLASHED = "--fw-flashed" in sys.argv
MARGIN = {"shoulder_pan": 30, "shoulder_lift": 30, "elbow_flex": 30, "wrist_flex": 30, "wrist_roll": 0}
# ความสูงของคำแนะนำต่อข้อ
HINT = {
    "shoulder_pan": "หมุนฐานไปสุดซ้าย-ขวาที่ต้องใช้ (คลุมสายพาน + ช่องปล่อย)",
    "shoulder_lift": "โน้มท่อนบนไปหน้าสุด-หลังสุดที่ต้องใช้ (ไม่รวมท่าพักบน cradle)",
    "elbow_flex": "พับ-เหยียดศอกสุดทั้งสองด้านที่ต้องใช้",
    "wrist_flex": "กดและแหงนข้อมือสุดทั้งสองด้านที่ต้องใช้",
    "wrist_roll": "หมุน gripper ผ่านช่วงที่ต้องใช้: ตั้งแต่ท่า L (jaw ขยับอยู่บน) ไปจนเลยท่า +180° (อย่าข้าม 0/4095)",
}

arm = make_arm()
arm.connect()
input("ข้อ 1-5 จะอ่อน — ประคองแขนไว้ แล้วกด Enter ")
arm.torque_off(ids=(1, 2, 3, 4, 5))

meas = {}
try:
    for j in J5:
        print(f"\n=== {j} ===\n  {HINT[j]}\n  ขยับให้สุดทั้งสองด้าน แล้วกด Ctrl+C")
        prev = arm.read_joint(j)
        lo = hi = prev
        wrapped = False
        try:
            while True:
                v = arm.read_joint(j)
                if abs(v - prev) > 2000:
                    wrapped = True        # ข้ามขอบ 0/4095
                prev = v
                lo, hi = min(lo, v), max(hi, v)
                print(f"  ตอนนี้={v:5d}   min={lo:5d}   max={hi:5d}   ", end="\r")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print()
        if wrapped:
            print(f"  !! {j} ข้ามขอบ 0/4095 — ค่า min/max ใช้ไม่ได้ ให้รันใหม่โดยหมุนไม่ข้ามขอบ")
        meas[j] = (lo, hi, wrapped)
finally:
    try:
        arm.torque_on(ids=(1, 2, 3, 4, 5))    # ค้างท่าปัจจุบัน ไม่กระโดด
    except Exception as e:
        print(f"เปิด torque กลับไม่สำเร็จ ({e}) — ประคองแขนไว้!")
    arm.disconnect(hold=False)

# ---------- สรุป ----------
print("\n" + "=" * 78)
print(f"{'ข้อ':14s}{'วัดได้':>14s}{'ข้อเสนอ':>16s}{'เฟิร์มแวร์เดิม':>18s}   สถานะ")
prop, safe_py, need_fw = {}, {}, []
for j in J5:
    lo, hi, wr = meas[j]
    m = MARGIN[j]
    p = (lo + m, hi - m)
    fw = C.MEGA_SAFE[j]
    prop[j] = p
    note = "ข้ามขอบ -> ไม่ใช้" if wr else ""
    if wr:
        safe_py[j] = tuple(C.JOINT_LIMITS[j])
    else:
        # ฝั่ง Python ต้องไม่กว้างกว่าเฟิร์มแวร์ (เว้นแต่ flash แล้ว)
        sp = p if FW_FLASHED else (max(p[0], fw[0]), min(p[1], fw[1]))
        safe_py[j] = sp
        if sp != p:
            need_fw.append(j)
            note = "กว้างกว่าเฟิร์มแวร์ -> ต้อง flash Mega"
        else:
            note = "OK" if p != tuple(fw) else "เท่าเดิม"
    print(f"{j:14s}{f'{lo}-{hi}':>14s}{f'{p[0]}-{p[1]}':>16s}{f'{fw[0]}-{fw[1]}':>18s}   {note}")
safe_py["gripper"] = tuple(C.JOINT_LIMITS["gripper"])

# ท่า L / ท่า scan ต้องอยู่ในช่วงใหม่ (เตือนถ้าไม่)
for j in J5:
    z, s = C.JOINT_ZERO[j], C.SCAN_POSE.get(j)
    for name, v in (("JOINT_ZERO(ท่า L)", z), ("SCAN_POSE", s)):
        if v is not None and not (safe_py[j][0] <= v <= safe_py[j][1]):
            print(f"  หมายเหตุ: {name}[{j}]={v:.0f} อยู่นอกช่วงที่จะบันทึก (ปกติสำหรับ roll: IK ใช้ twin +180° เอง)")

if need_fw:
    fmin = [0] + [prop[j][0] if j in need_fw else C.MEGA_SAFE[j][0] for j in J5] + [C.MEGA_SAFE["gripper"][0]]
    fmax = [4095] + [prop[j][1] if j in need_fw else C.MEGA_SAFE[j][1] for j in J5] + [C.MEGA_SAFE["gripper"][1]]
    ino = ("const int16_t SAFE_MIN[7] = {%s};\nconst int16_t SAFE_MAX[7] = {%s};\n"
           % (", ".join(map(str, fmin)), ", ".join(map(str, fmax))))
    with open("limits_proposal.txt", "w", encoding="utf-8") as f:
        f.write("// แทนที่สองบรรทัดนี้ใน so101_mega_controller.ino แล้ว flash Mega (เฟิร์มแวร์ใช้ร่วมกับ PRISM ของเพื่อน — คุยกันก่อน)\n" + ino)
    print("\nข้อที่กว้างกว่าเฟิร์มแวร์:", ", ".join(need_fw))
    print("บรรทัดสำหรับ .ino (เก็บไว้ที่ limits_proposal.txt ด้วย):\n" + ino)

if input("บันทึกฝั่ง Python (calib.json) ตามตารางสถานะ? พิมพ์ y: ").strip().lower() == "y":
    kv = {"JOINT_LIMITS": {j: list(v) for j, v in safe_py.items()}}
    if FW_FLASHED:
        kv["MEGA_SAFE"] = {j: list(v) for j, v in safe_py.items()}   # เฟิร์มแวร์ตรงกับค่านี้แล้ว
    C.save_calib(**kv)
    print("บันทึกแล้ว ->", C.CALIB_FILE)
    if need_fw:
        print("ข้อที่ยังกว้างกว่าเฟิร์มแวร์ถูกจำกัดตามเฟิร์มแวร์ไว้ก่อน จนกว่าจะ flash แล้วรันด้วย --fw-flashed")
