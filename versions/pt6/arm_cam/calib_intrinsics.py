"""calib_intrinsics.py — คาลิเบรตเลนส์กล้อง (แก้ distortion) ด้วยกระดานหมากรุก
ทำบน PC ครั้งเดียว แล้วคัดลอก intrinsics.npz ไปไว้บน Pi ได้เลย (ใช้กล้องตัวเดิม ความละเอียดเดิม)

ขั้นตอน: พิมพ์กระดานหมากรุก (ขนาดตาม config.py) ติดบนแผ่นเรียบ ถือให้กล้องเห็นครบหลายมุม/ระยะ
กด SPACE เมื่อขึ้นเส้นสีรุ้ง (เจอกระดาน) ให้ได้ 15-20 ภาพ แล้วกด q เพื่อคำนวณ
"""
import cv2
import numpy as np
import camera
import config as C

pattern = (C.CHECKER_COLS, C.CHECKER_ROWS)
# พิกัด 3 มิติของมุมกระดาน (z=0) หน่วย mm
objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * C.CHECKER_SQUARE_MM

obj_pts, img_pts = [], []
cap = camera.open_camera()
size = None
print("SPACE = เก็บภาพ | q = คำนวณ")
while True:
    # ใช้ภาพดิบ (ไม่ undistort) เพราะกำลังหา intrinsics เอง
    cap.grab()
    ok, frame = cap.read()
    if not ok:
        continue
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    size = gray.shape[::-1]
    found, corners = cv2.findChessboardCorners(gray, pattern)
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if found:
        cv2.drawChessboardCorners(vis, pattern, corners, found)
    cv2.putText(vis, f"captured: {len(obj_pts)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.imshow("intrinsics", vis)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(" ") and found:
        # ปรับมุมให้แม่นระดับ sub-pixel ก่อนเก็บ
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        obj_pts.append(objp)
        img_pts.append(corners)
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
if len(obj_pts) < 10:
    raise SystemExit("ภาพน้อยเกินไป (ต้อง >= 10)")

rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
print(f"reprojection RMS = {rms:.3f} px  (ดี < 0.5; ถ้าเกิน 1.0 ให้ถ่ายใหม่ให้หลากหลายมุมขึ้น)")
np.savez(C.INTRINSICS_FILE, K=K, D=D)
print("บันทึก", C.INTRINSICS_FILE)
