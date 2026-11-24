"""set_roi.py — กำหนดบริเวณในภาพที่ใช้หาบล็อก (DETECT_ROI) โดยลากเมาส์ แล้วบันทึกลง calib.json
ทำไมต้องมี: กล้องติดข้อมือ jaw/ตัวแขนจะโผล่ในเฟรมเสมอ ถ้าไม่ตัดออก ตัวตรวจจับอาจจับ jaw แทนบล็อก
ใช้:  python set_roi.py            (แขนต้องอยู่ท่า scan — สคริปต์พาไปให้ถ้าต่อแขนอยู่; --no-arm = ไม่ขยับแขน)
คีย์:  ลากเมาส์ = วาดกรอบ | s = บันทึก | b = สลับ บล็อกสว่าง/มืดกว่าพื้น | r = ล้างกรอบ (ทั้งภาพ) | q = ออก
หน้าต่างขวาแสดงภาพขาวดำหลัง threshold: บล็อกควรเป็นก้อนขาวก้อนเดียวชัดๆ ในกรอบ
"""
import sys

import cv2
import numpy as np

import camera
import config as C
import detect_block as D

arm = None
if "--no-arm" not in sys.argv:
    from arm_select import make_arm
    arm = make_arm()
    arm.connect()
    arm.open_gripper()
    arm.move_smooth(C.SCAN_POSE, duration=3.0, fine=True)

cap = camera.open_camera()
roi = list(C.DETECT_ROI)
bright = bool(C.BLOCK_IS_BRIGHT)
drag = {}


def on_mouse(event, mx, my, flags, param):
    """ลากซ้าย = กำหนดกรอบใหม่"""
    if event == cv2.EVENT_LBUTTONDOWN:
        drag["p0"] = (mx, my)
    elif event == cv2.EVENT_MOUSEMOVE and "p0" in drag:
        x0, y0 = drag["p0"]
        roi[:] = [min(x0, mx), min(y0, my), max(x0, mx), max(y0, my)]
    elif event == cv2.EVENT_LBUTTONUP:
        drag.pop("p0", None)


cv2.namedWindow("roi")
cv2.setMouseCallback("roi", on_mouse)
saved = False
try:
    while True:
        g = camera.grab(cap)
        if g is None:
            continue
        C.DETECT_ROI, C.BLOCK_IS_BRIGHT = tuple(roi), bright     # ให้ตัวตรวจจับใช้ค่าที่กำลังลอง
        blob = D.find_blob_px(g) if (roi[2] - roi[0] > 20 and roi[3] - roi[1] > 20) else None
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (roi[0], roi[1]), (roi[2], roi[3]), (255, 128, 0), 2)
        if blob is not None:
            cv2.polylines(vis, [blob[2].astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(vis, f"block {'BRIGHT' if bright else 'DARK'} | {'FOUND' if blob else 'not found'} | s=save b=toggle r=reset q=quit",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if blob else (0, 0, 255), 2)
        cv2.imshow("roi", vis)
        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("b"):
            bright = not bright
        if k == ord("r"):
            roi[:] = [0, 0, g.shape[1], g.shape[0]]
        if k == ord("s"):
            C.save_calib(DETECT_ROI=[int(v) for v in roi], BLOCK_IS_BRIGHT=bright)
            saved = True
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    if arm is not None:
        arm.disconnect()
print("บันทึก DETECT_ROI =" if saved else "ไม่ได้บันทึก", roi if saved else "")
