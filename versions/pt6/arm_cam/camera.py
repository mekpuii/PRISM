"""camera.py — เปิดกล้อง OV9281 และอ่านภาพ gray (ใช้ร่วมกันทุกสคริปต์)"""
import os
import time
import cv2
import numpy as np
import config as C
import pi_env

# ---- โหลดค่า intrinsic ถ้ามี (ใช้แก้ distortion ของเลนส์) ----
_K = _D = None
if os.path.exists(C.INTRINSICS_FILE):
    _d = np.load(C.INTRINSICS_FILE)
    _K, _D = _d["K"], _d["D"]


CAM_MEAN_WARN = 100                        # สว่างเฉลี่ยเกินนี้ -> เตือน


def _open_raw():
    """เปิดกล้องด้วย MJPG, ล็อก exposure, ทิ้งเฟรมแรกๆ ที่ยังไม่นิ่ง"""
    if C.CAM_INDEX is None:                     # Pi: the wrist camera was not found by name
        raise RuntimeError(pi_env.camera_help())
    cap = cv2.VideoCapture(C.CAM_INDEX, C.CAM_BACKEND)
    if not cap.isOpened():
        busy = pi_env.holders(C.CAM_INDEX) if isinstance(C.CAM_INDEX, str) else []
        who = (" — in use by " + "; ".join(f"pid {p}: {c[:80]}" for p, c in busy)) if busy else ""
        raise RuntimeError(f"เปิดกล้อง {C.CAM_INDEX} ไม่ได้ — ตรวจ CAM_INDEX ใน config.py{who}")

    # MJPG ประหยัด bandwidth USB 2.0 ทำให้ได้ 720p@60 ได้จริง
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAM_H)
    cap.set(cv2.CAP_PROP_FPS, C.CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)        # ลดคิวภาพเก่า (ไม่งั้นได้ภาพหน่วง)

    if C.CAM_MANUAL_EXPOSURE:                   # บาง driver ไม่รองรับ จะถูกเมินเฉยๆ (ดูหมายเหตุใน config.py)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, C.CAM_AUTO_EXPOSURE_MANUAL_VAL)
        cap.set(cv2.CAP_PROP_EXPOSURE, C.CAM_EXPOSURE_VAL)

    for _ in range(15):                         # warm-up
        cap.read()
        time.sleep(0.01)
    ok, fr = cap.read()                         # เช็กความสว่าง: ถ้าสว่างกว่าที่ตั้งไว้มาก = driver ไม่ยอมล็อก exposure -> Otsu/ตรวจจับจะเพี้ยน
    if ok:
        mean = float(fr.mean())
        if mean > CAM_MEAN_WARN:
            print(f"[camera] เตือน: ภาพสว่างเฉลี่ย {mean:.0f}/255 (ปกติตอนตั้ง exposure {C.CAM_EXPOSURE_VAL} ~ 20-60) "
                  "— exposure ไม่ถูกล็อก/แสงเปลี่ยน; ปิดโปรแกรมที่ใช้กล้องอยู่ แล้วเปิดใหม่ หรือลด CAM_EXPOSURE_VAL")
    return cap


class LatestCamera:
    """อ่านกล้องใน thread แยก เก็บไว้แค่ "เฟรมล่าสุด" -> ลูปหลักไม่ต้องรอกล้องเลย (กัน Not Responding)
    - cap.read()/grab() ของ OpenCV บน Windows (MSMF) ค้างได้หลายวินาทีเวลา USB สะดุด: ตอนนี้ค้างแค่ใน thread นี้
    - ไม่มีเฟรมใหม่เกิน REOPEN_S วินาที -> ปิดแล้วเปิดกล้องใหม่เอง
    ใช้แทน cv2.VideoCapture ได้ (มี read / grab / release / isOpened / set / get)"""
    REOPEN_S = 3.0

    def __init__(self):
        import threading
        self._cv = threading.Condition()
        self._frame, self._seq, self._t = None, 0, time.perf_counter()
        self._running = True
        self.reopens = 0
        self._cap = _open_raw()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        fails = 0
        while self._running:
            cap = self._cap
            ok, fr = (False, None)
            try:
                ok, fr = cap.read() if cap is not None else (False, None)
            except Exception:
                ok = False
            if ok and fr is not None:
                fails = 0
                with self._cv:
                    self._frame, self._seq, self._t = fr, self._seq + 1, time.perf_counter()
                    self._cv.notify_all()
                continue
            fails += 1
            time.sleep(0.01)
            if self._running and time.perf_counter() - self._t > self.REOPEN_S:
                print(f"[camera] ไม่มีภาพใหม่ {self.REOPEN_S:.0f}s -> เปิดกล้องใหม่", flush=True)
                try:
                    if cap is not None:
                        cap.release()
                except Exception:
                    pass
                try:
                    self._cap = _open_raw()
                    self.reopens += 1
                except Exception as e:
                    print(f"[camera] เปิดใหม่ไม่ได้: {e}", flush=True)
                    self._cap = None
                    time.sleep(1.0)
                self._t = time.perf_counter()

    def latest(self, after_seq=None, timeout=0.25):
        """คืน (seq, frame) ของเฟรมที่ใหม่กว่า after_seq (รอไม่เกิน timeout) หรือ (seq, None) ถ้ายังไม่มี"""
        end = time.perf_counter() + timeout
        with self._cv:
            while self._frame is None or (after_seq is not None and self._seq <= after_seq):
                left = end - time.perf_counter()
                if left <= 0 or not self._running:
                    return self._seq, None
                self._cv.wait(left)
            return self._seq, self._frame

    @property
    def age_s(self):
        """กี่วินาทีแล้วที่ไม่มีเฟรมใหม่"""
        return time.perf_counter() - self._t

    # ---- ให้ใช้แทน cv2.VideoCapture ได้ ----
    _last_seq = None

    def read(self):
        seq, fr = self.latest(self._last_seq)
        if fr is None:
            return False, None
        self._last_seq = seq
        return True, fr

    def grab(self):
        return True

    def isOpened(self):
        return self._cap is not None and self._running

    def set(self, prop, val):
        return self._cap.set(prop, val) if self._cap is not None else False

    def get(self, prop):
        return self._cap.get(prop) if self._cap is not None else 0.0

    def release(self):
        self._running = False
        with self._cv:
            self._cv.notify_all()
        self._th.join(timeout=2.0)
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass


def open_camera():
    """เปิดกล้องแบบอ่านใน thread แยก (ดู LatestCamera)"""
    return LatestCamera()


def grab(cap):
    """อ่าน 1 เฟรม "ใหม่" -> ภาพ gray (undistort แล้วถ้ามี intrinsics) หรือ None ถ้ายังไม่มีเฟรมใหม่ภายใน ~0.25 s
    (ไม่ค้าง: ผู้เรียกควรวน waitKey ต่อแม้ได้ None)"""
    if isinstance(cap, LatestCamera):
        ok, frame = cap.read()
    else:
        cap.grab()                              # VideoCapture ธรรมดา: ทิ้งเฟรมค้างในบัฟเฟอร์ 1 เฟรม
        ok, frame = cap.read()
    if not ok or frame is None:
        return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    if _K is not None:
        gray = cv2.undistort(gray, _K, _D)
    return gray
