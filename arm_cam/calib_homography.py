"""calib_homography.py — ทำ homography: pixel -> mm ในกรอบหุ่นยนต์ (ที่ท่า SCAN_POSE)

เงื่อนไขสำคัญ (กล้องติดข้อมือ):
  * ต้องทำตอนแขน "อยู่ที่ SCAN_POSE" เท่านั้น และตอนหยิบจริงต้องกลับมาท่าเดิม ถึงจะใช้ H เดิมได้
  * มาร์กสอบเทียบต้องอยู่บนระนาบสูง CALIBRATION_PLANE_Z_MM (=20 mm = หน้าบนบล็อก)
    ทำได้โดยวางกระดาษมาร์กบนแผ่นเรียบหนา 20 mm  (หรือซ้อนของให้ได้ 20 mm)
  * วางมาร์ก >= 4 จุดกระจายให้ครอบพื้นที่ที่บล็อกอาจอยู่ วัดพิกัด (x, y) แต่ละจุดเทียบแกน shoulder_pan
    (x ข้างหน้า, y ซ้าย) ด้วยไม้บรรทัด ยิ่งวัดแม่น ยิ่งหยิบแม่น

ใช้:  python calib_homography.py --move     # พาแขนไป SCAN_POSE ก่อนถ่ายภาพ (ต่อแขนอยู่)
      python calib_homography.py            # ถ้าวางแขนที่ SCAN_POSE ด้วยมือเองแล้ว / ทดสอบกล้องอย่างเดียว
ต้องใช้หน้าต่างภาพ -> รันบน PC (ไฟล์ homography.npy คัดลอกไป Pi ได้)
"""
import sys
import cv2
import numpy as np
import camera
import config as C

# ---- (ตัวเลือก) พาแขนไปท่า scan ----
if "--move" in sys.argv:
    from arm_select import make_arm
    arm = make_arm()
    arm.connect()
    arm.move_smooth(C.SCAN_POSE, duration=3.0)
else:
    arm = None

# ---- ถ่ายภาพ (เฉลี่ย 5 เฟรมลด noise) ----
# ยังไม่ disconnect แขนจนกว่าจะถ่ายเสร็จ: disconnect ของ LeRobot อาจปล่อย torque ทำให้แขนตกจากท่า scan
cap = camera.open_camera()
frames = [camera.grab(cap) for _ in range(5)]
cap.release()
if arm is not None:
    arm.disconnect()
img = np.mean([f for f in frames if f is not None], axis=0).astype(np.uint8)

# ---- ให้คลิกมาร์กตามลำดับ ----
clicks = []
vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def on_mouse(event, mx, my, flags, param):
    """คลิกซ้าย = เพิ่มจุด แล้ววาดวงกลมพร้อมเลขลำดับ"""
    if event == cv2.EVENT_LBUTTONDOWN:
        clicks.append((mx, my))
        cv2.circle(vis, (mx, my), 5, (0, 255, 0), -1)
        cv2.putText(vis, str(len(clicks)), (mx + 8, my - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


cv2.namedWindow("calib")
cv2.setMouseCallback("calib", on_mouse)
print("คลิกกลางมาร์กแต่ละจุดตามลำดับ (>= 4) แล้วกด Enter")
while True:
    cv2.imshow("calib", vis)
    if cv2.waitKey(20) in (13, 10):                     # Enter
        break
cv2.destroyAllWindows()
if len(clicks) < 4:
    raise SystemExit("ต้องมีอย่างน้อย 4 จุด")

# ---- กรอกพิกัดจริง (mm) ของแต่ละจุดที่คลิก ----
robot_pts = []
for i, (px, py) in enumerate(clicks, 1):
    x, y = map(float, input(f"จุด {i}: พิมพ์ x y (mm) เว้นวรรค: ").split())
    robot_pts.append((x, y))

pix = np.array(clicks, dtype=np.float32)
rob = np.array(robot_pts, dtype=np.float32)
H, _ = cv2.findHomography(pix, rob, method=0)           # least-squares ใช้ทุกจุด

# ---- ตรวจความคลาดเคลื่อน: map pixel กลับเป็น mm เทียบค่าที่วัด ----
pred = cv2.perspectiveTransform(pix[None], H)[0]
err = np.linalg.norm(pred - rob, axis=1)
print("ความคลาดเคลื่อนรายจุด (mm):", np.round(err, 2))
print(f"RMS = {np.sqrt((err ** 2).mean()):.2f} mm   (เป้าหมาย < 2 mm; ถ้ามากให้วัดพิกัดใหม่/เพิ่มจุด)")
np.save(C.HOMOGRAPHY_FILE, H)
print("บันทึก", C.HOMOGRAPHY_FILE)
