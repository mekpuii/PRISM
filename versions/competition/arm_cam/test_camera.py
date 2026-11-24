"""test_camera.py — ทดสอบเปิดกล้อง OV9281 (ไฟล์เดี่ยว ไม่พึ่ง config.py รันได้เลย)

ใช้:  python test_camera.py --scan               # หาว่ากล้องอยู่ index ไหน (ลองทุก backend)
      python test_camera.py --index 1            # เปิดกล้อง index 1: แสดงค่าที่ต่อรองได้ + วัด fps จริง + preview
      python test_camera.py --index 1 --nogui    # ไม่เปิดหน้าต่าง (Pi ไม่มีจอ) บันทึก snapshot.png แทน
      python test_camera.py --index 1 --fps 120 --w 1280 --h 720 --fourcc MJPG
ในหน้าต่าง preview: s = บันทึกภาพ, ESC หรือ q = ออก
"""
import argparse
import sys
import time

import cv2
import numpy as np

IS_WINDOWS = sys.platform.startswith("win")
# Windows ลอง MSMF ก่อน (กล้อง OV9281 ตัวนี้ได้ ~115 fps บน MSMF แต่ DSHOW ติดที่ 10 fps) | Linux/Pi ใช้ V4L2
BACKENDS = ([("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW)] if IS_WINDOWS
            else [("V4L2", cv2.CAP_V4L2)])


def fourcc_to_str(v):
    """แปลงค่า FOURCC (ตัวเลข) เป็นตัวอักษร เช่น 'MJPG'"""
    v = int(v)
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def scan(max_index=10):
    """ลองเปิดทุก index x ทุก backend รายงานตัวที่เปิดได้และอ่านเฟรมได้จริง"""
    print(f"สแกน index 0-{max_index - 1} ...")
    found = 0
    for idx in range(max_index):
        for name, be in BACKENDS:
            cap = cv2.VideoCapture(idx, be)
            ok = cap.isOpened()
            frame = None
            if ok:
                for _ in range(5):                    # อ่านทิ้งเฟรมแรกๆ ที่มักว่าง/มืด
                    ok, frame = cap.read()
                    time.sleep(0.05)
            if ok and frame is not None:
                h, w = frame.shape[:2]
                print(f"  [พบ] index={idx} backend={name} ภาพ {w}x{h} ช่อง={1 if frame.ndim == 2 else frame.shape[2]}")
                found += 1
                cap.release()
                break                                 # เจอแล้วไม่ต้องลอง backend ถัดไปของ index นี้
            cap.release()
    if not found:
        print("ไม่พบกล้องเลย — เช็กสาย USB/พอร์ต, Device Manager (Windows) หรือ `v4l2-ctl --list-devices` (Pi),"
              " และปิดโปรแกรมอื่นที่ใช้กล้องอยู่ (Zoom, Teams, Camera app)")
    else:
        print("\nเลือก index ที่ใช่ (OV9281 มักเป็นตัวที่ภาพเป็นขาวดำ) แล้วรัน: python test_camera.py --index <เลข>")


def open_cam(idx, w, h, fps, fourcc):
    """เปิดกล้องด้วยค่าที่ขอ ลองทีละ backend คืน (cap, ชื่อ backend) หรือ (None, None)"""
    for name, be in BACKENDS:
        cap = cv2.VideoCapture(idx, be)
        if not cap.isOpened():
            cap.release()
            continue
        # ลำดับสำคัญ: ตั้ง FOURCC ก่อนความละเอียด/fps ไม่งั้น driver บางตัวเมินค่า
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok = False
        for _ in range(20):                           # warm-up จน อ่านเฟรมได้จริง
            ok, _f = cap.read()
            if ok:
                break
            time.sleep(0.05)
        if ok:
            return cap, name
        cap.release()
    return None, None


def to_gray(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan", action="store_true", help="สแกนหา index ของกล้อง")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--fourcc", default="MJPG", help="MJPG (แนะนำ) หรือ YUY2")
    ap.add_argument("--nogui", action="store_true")
    a = ap.parse_args()

    if a.scan:
        scan()
        return

    cap, be = open_cam(a.index, a.w, a.h, a.fps, a.fourcc)
    if cap is None:
        raise SystemExit(f"เปิดกล้อง index {a.index} ไม่ได้ — รัน --scan หา index ที่ถูกก่อน")

    # ---- 1) ค่าที่ต่อรองได้จริง (driver อาจไม่ให้ตามที่ขอ) ----
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    got_fps = cap.get(cv2.CAP_PROP_FPS)
    got_fcc = fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
    print(f"backend={be}")
    print(f"ขอ     : {a.w}x{a.h} @{a.fps} {a.fourcc}")
    print(f"ได้จริง: {got_w}x{got_h} @{got_fps:.0f} {got_fcc}   (ถ้าไม่ตรงที่ขอ = driver ไม่รองรับโหมดนั้น)")

    # ---- 2) วัด fps จริงจากการอ่านเฟรม 3 วินาที (ค่า CAP_PROP_FPS โกหกได้) ----
    n, t0, dropped, last = 0, time.time(), 0, None
    while time.time() - t0 < 3.0:
        ok, fr = cap.read()
        if not ok:
            dropped += 1
            continue
        n += 1
        last = fr
    el = time.time() - t0
    print(f"fps จริง: {n / el:.1f}  (อ่านได้ {n} เฟรมใน {el:.1f}s, อ่านพลาด {dropped})")
    if last is None:
        raise SystemExit("อ่านเฟรมไม่ได้เลยระหว่างวัด fps")

    # ---- 3) ตรวจคุณภาพภาพ: ขาวดำจริงไหม, สว่างพอไหม, ไม่ไหม้/มืดเกิน ----
    g = to_gray(last)
    is_mono_bgr = last.ndim == 3 and np.allclose(last[..., 0], last[..., 1]) and np.allclose(last[..., 1], last[..., 2])
    print(f"ภาพ: shape={last.shape} {'(mono แต่ส่งมาเป็น 3 ช่อง ปกติ)' if is_mono_bgr else ''}")
    print(f"ความสว่าง: mean={g.mean():.0f}/255 min={g.min()} max={g.max()}  "
          f"พิกเซลไหม้(>=250)={100 * (g >= 250).mean():.1f}%  มืดสนิท(<=5)={100 * (g <= 5).mean():.1f}%")
    if g.mean() < 25:
        print("  ! ภาพมืดมาก — เพิ่มแสงหรือเพิ่ม exposure")
    elif g.mean() > 220 or (g >= 250).mean() > 0.2:
        print("  ! ภาพสว่างเกิน — ลด exposure/แสง")
    # ความคมชัด (variance of Laplacian): ยิ่งสูงยิ่งคม ใช้เทียบก่อน-หลังปรับโฟกัส/ระยะ
    sharp = cv2.Laplacian(g, cv2.CV_64F).var()
    print(f"ความคม (Laplacian var): {sharp:.0f}  (เทียบค่านี้ตอนปรับระยะ — ยิ่งสูงยิ่งโฟกัสชัด)")

    cv2.imwrite("snapshot.png", g)
    print("บันทึก snapshot.png แล้ว")

    # ---- 4) preview สด ----
    if a.nogui:
        cap.release()
        return
    print("preview: s = บันทึกภาพ, ESC/q = ออก")
    cnt, t_fps, fps_show = 0, time.time(), 0.0
    while True:
        ok, fr = cap.read()
        if not ok:
            continue
        cnt += 1
        if time.time() - t_fps >= 1.0:                # อัปเดตตัวเลข fps ทุก 1 วินาที
            fps_show, cnt, t_fps = cnt / (time.time() - t_fps), 0, time.time()
        vis = fr if fr.ndim == 3 else cv2.cvtColor(fr, cv2.COLOR_GRAY2BGR)
        cv2.putText(vis, f"{vis.shape[1]}x{vis.shape[0]}  {fps_show:.0f} fps", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow("test_camera", vis)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("s"):
            name = f"snap_{int(time.time())}.png"
            cv2.imwrite(name, to_gray(fr))
            print("บันทึก", name)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
