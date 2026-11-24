"""teach_scan.py — สอน "ท่า scan" (ท่าที่กล้องบนข้อมือมองโต๊ะ) แล้วบันทึกลง calib.json

ทำไมสำคัญ: กล้องติดข้อมือ -> ภาพเปลี่ยนตามท่าแขน  การแปลง pixel->mm (homography) จึงใช้ได้เฉพาะท่านี้ท่าเดียว
ท่าที่ดี:  * กล้องมองลงโต๊ะ "ใกล้ดิ่ง" ที่สุดเท่าที่ทำได้ (เอียงมาก = ความแม่นยำที่ขอบภาพตก)
           * เห็นพื้นที่วางบล็อกทั้งหมด (โซนที่แขนหยิบถึง: ห่างแกนฐาน ~100-230 mm)
           * jaw ไม่บังกลางภาพ, ภาพคมชัดที่ระยะนี้ (ถ้าเบลอให้หมุนเลนส์ปรับโฟกัสตอนนี้เลย)
ขั้นตอน: สคริปต์เปิด gripper -> ปล่อย torque ข้อ 1-5 -> คุณจับแขนจัดท่าโดยดูภาพสด -> กด s = บันทึก (torque กลับมาค้างท่านั้น)

ใช้:  python teach_scan.py        (ต้องมีหน้าจอ: รันบน PC)
"""
import cv2
import camera
import config as C
from arm_select import make_arm

if C.ARM_BACKEND != "mega":
    raise SystemExit("สคริปต์นี้สำหรับ ARM_BACKEND='mega' เท่านั้น")

J5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MARGIN = 40                                   # ท่า scan ต้องห่างขอบ SAFE อย่างน้อยเท่านี้ (tick)

cap = camera.open_camera()
arm = make_arm()
arm.connect()
saved = None
try:
    arm.open_gripper()                        # ให้ภาพตอนสอน = ภาพตอนใช้งานจริง (jaw อ้า)
    input("แขนข้อ 1-5 จะถูกปล่อย torque (อ่อน) — ประคองแขนไว้ แล้วกด Enter ")
    arm.torque_off(ids=(1, 2, 3, 4, 5))
    print("จัดท่าโดยดูหน้าต่างภาพ:  s = บันทึกท่านี้   q/ESC = ออกโดยไม่บันทึก")
    while True:
        g = camera.grab(cap)
        if g is None:
            continue
        pos = {j: arm.read_joint(j) for j in J5}
        bad = [j for j in J5 if not (C.JOINT_LIMITS[j][0] + MARGIN <= pos[j] <= C.JOINT_LIMITS[j][1] - MARGIN)]
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        h, w = g.shape[:2]
        cv2.line(vis, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)          # กากบาทกลางภาพ
        cv2.line(vis, (0, h // 2), (w, h // 2), (0, 255, 255), 1)
        txt = "  ".join(f"{j[:8]}={pos[j]}" for j in J5)
        cv2.putText(vis, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(vis, "OUT OF RANGE: " + ",".join(bad) if bad else "in range - press s to save",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if bad else (0, 255, 0), 2)
        cv2.imshow("teach scan pose", vis)
        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("s"):
            if bad:
                print("ยังบันทึกไม่ได้: นอก/ชิดขอบช่วง ->", bad)
                continue
            saved = {j: int(pos[j]) for j in J5}
            saved["gripper"] = int(C.GRIPPER_OPEN)
            cv2.imwrite("scan_view.png", g)   # เก็บภาพไว้ดูย้อนหลัง/ตั้ง DETECT_ROI
            break
finally:
    try:
        arm.torque_on(ids=(1, 2, 3, 4, 5))    # ค้างท่าปัจจุบัน (ไม่กระโดด)
    finally:
        arm.disconnect(hold=False)
        cap.release()
        cv2.destroyAllWindows()

if saved:
    C.save_calib(SCAN_POSE=saved)
    print("บันทึก SCAN_POSE ->", saved)
    print("ภาพมุมมองเก็บที่ scan_view.png  | ถ้าเคยทำ calib_touch.py มาก่อน ต้องทำใหม่ (ท่า scan เปลี่ยน = homography เดิมใช้ไม่ได้)")
else:
    print("ไม่ได้บันทึก")
