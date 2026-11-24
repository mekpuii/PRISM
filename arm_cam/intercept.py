"""intercept.py — หยิบบล็อก "ขณะกำลังเคลื่อนที่" บนสายพาน (ตัวเลือก B: คาดการณ์ตำแหน่ง แล้วไปดัก)

ลำดับงาน (ทุกอย่างอิงนาฬิกาเดียว time.perf_counter):
  1) แขนอยู่ท่า scan (homography ใช้ได้เฉพาะท่านี้) -> ตามดูบล็อกหลายเฟรม -> fit เส้นตรง p(t) = p0 + v*(t - t0)
  2) เลือก "จุดดัก" = ตำแหน่งที่บล็อกจะอยู่ ณ เวลา t_hit ซึ่งแขนไปถึงทันและ IK ไปได้
  3) แขนไปท่าดัก: jaw คร่อมทางวิ่ง (แกนปิด jaw ตั้งฉากกับทิศวิ่ง) ที่ความสูงหนีบ, gripper หุบรอไว้เกือบชิดบล็อก
     -> บล็อกวิ่งเข้ามาในช่องระหว่าง jaw เอง
  4) สั่งปิด jaw (C,6) ล่วงหน้าเท่ากับเวลาที่ jaw ใช้หุบ ให้ jaw แตะบล็อกตอนบล็อกถึงจุดดักพอดี -> ยก
ข้อจำกัดที่ต้องรู้:
  * หลังออกจากท่า scan กล้อง (ติดข้อมือ) ใช้วัดตำแหน่งไม่ได้แล้ว = ช่วงท้ายเป็น open-loop: ความเร็วสายพานต้องคงที่
  * ค่าเผื่อตามแนววิ่ง ~ ±13 mm (แผ่น jaw กว้าง 13 mm บนบล็อกยาว 40 mm) = ยอมคลาดเวลาได้ ~13/ความเร็ว วินาที
  * ต้องผ่านการหยิบแบบนิ่งก่อน (calib ครบ + GRIP_TICK_RANGE จาก repeat_test.py) เพราะใช้ค่าสอบเทียบชุดเดียวกัน
"""
import math
import time
from contextlib import contextmanager

import numpy as np

import camera
import config as C
import detect_block as D
import kinematics as K
from picker import Picker, PickError


@contextmanager
def _clearance(mm):
    """เปลี่ยน GRASP_CLEARANCE_MM ชั่วคราว (IK อ่านค่านี้ตอนคำนวณจุดหนีบ)"""
    old = C.GRASP_CLEARANCE_MM
    C.GRASP_CLEARANCE_MM = mm
    try:
        yield
    finally:
        C.GRASP_CLEARANCE_MM = old


_now = time.perf_counter      # นาฬิกาละเอียด (Windows + Python ก่อน 3.13: นาฬิกา monotonic ละเอียดแค่ ~16 ms ไม่พอจับเวลาเฟรม)


def _sleep_until(t):
    """รอจนถึงเวลา t (นาฬิกา _now) — หลับเป็นช่วงจนเหลือ 20 ms แล้ววนเปล่าให้ตรงเวลา (sleep ของ OS คลาดได้ถึง ~15 ms)"""
    while True:
        dt = t - _now()
        if dt <= 0:
            return
        if dt > 0.02:
            time.sleep(dt - 0.02)


class Interceptor(Picker):
    # ------------------------------------------------------------ มอง: ตามดูและวัดความเร็ว
    def goto_scan(self):
        """ไปท่า scan + เปิด gripper (ไม่ถ่ายภาพ)"""
        if self.arm is not None:
            self.arm.open_gripper()
            self.arm.move_smooth(C.SCAN_POSE, duration=2.5, fine=True)
            time.sleep(0.6)                                # รอสั่นหมด

    def wait_block(self, timeout=30.0, need=3):
        """รอจนเห็นบล็อก (ครบทั้งตัวใน ROI) ติดกัน need เฟรม  คืน True/False"""
        end, run = _now() + timeout, 0
        while _now() < end:
            g = camera.grab(self.cap)
            run = run + 1 if (g is not None and D.find_block(g, self.H)) else 0
            if run >= need:
                return True
        return False

    def track(self, duration=None):
        """ตามดูบล็อก duration วินาที แล้ว fit เส้นตรง  คืน dict หรือ None ถ้าเห็นไม่พอ
        t0 = เวลาของเฟรมสุดท้าย, p0 = ตำแหน่ง (mm) ณ t0, v = ความเร็ว (mm/s), rms = ค่าคลาดของการ fit (mm)"""
        duration = C.INTERCEPT_TRACK_S if duration is None else duration
        ts, ps, angs = [], [], []
        end = _now() + duration
        while _now() < end:
            g = camera.grab(self.cap)
            t = _now() - C.CAM_LATENCY_S         # เวลาที่ภาพถูกถ่ายจริง (ประมาณ)
            r = D.find_block(g, self.H) if g is not None else None
            if r:
                ts.append(t); ps.append((r["x"], r["y"])); angs.append(r["long_deg"])
        if len(ts) < C.INTERCEPT_MIN_HITS:
            return None
        ts, ps = np.array(ts), np.array(ps)
        t0 = float(ts[-1])
        A = np.column_stack([np.ones_like(ts), ts - t0])   # p = p0 + v*(t - t0) แยก fit แกน x และ y
        coef, *_ = np.linalg.lstsq(A, ps, rcond=None)
        rms = float(np.sqrt(((A @ coef - ps) ** 2).sum(axis=1).mean()))
        a2 = np.radians(2 * np.array(angs))                # เฉลี่ยมุมแบบวงกลม (มุมวนทุก 180°)
        long_deg = (math.degrees(math.atan2(np.sin(a2).mean(), np.cos(a2).mean())) / 2.0) % 180.0
        v = coef[1]
        return {"t0": t0, "p0": coef[0], "v": v, "speed": float(np.hypot(*v)), "rms": rms,
                "long_deg": long_deg, "n": len(ts), "span_s": float(ts[-1] - ts[0])}

    # ------------------------------------------------------------ วางแผนดัก
    def plan_intercept(self, trk, t_earliest):
        """หาจุดดักแรกที่ (ก) เวลา >= t_earliest (ข) อยู่ใน INTERCEPT_ZONE (ค) IK ไปได้ทั้งทางลง
        คืน dict(plan, t_hit, x, y, closing)  raise PickError ถ้าไม่มี"""
        ux, uy = trk["v"] / trk["speed"]
        closing = (math.degrees(math.atan2(uy, ux)) + 90.0) % 180.0      # jaw ปิด "ขวาง" ทางวิ่ง
        d = math.radians(trk["long_deg"] - closing)
        across = abs(C.BLOCK_LONG_MM * math.cos(d)) + abs(C.BLOCK_SHORT_MM * math.sin(d))   # ความกว้างบล็อกในแนวขวางทางวิ่ง
        if across > C.BLOCK_SHORT_MM + 6.0:
            raise PickError(f"บล็อกวางขวาง/เฉียงกับทางวิ่ง (กว้าง {across:.0f} mm ในแนว jaw) — ต้องให้ด้านยาว 40 mm ขนานทางวิ่ง")
        x0, x1, y0, y1 = C.INTERCEPT_ZONE
        why = "อยู่นอก INTERCEPT_ZONE ตลอดช่วงเวลาที่รอได้"
        for k in range(int(C.INTERCEPT_MAX_WAIT_S / 0.1) + 1):
            t_hit = t_earliest + 0.1 * k
            x, y = trk["p0"] + trk["v"] * (t_hit - trk["t0"])
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            try:
                with _clearance(C.INTERCEPT_CLEARANCE_MM):
                    plan = self.plan({"x": float(x), "y": float(y), "long_deg": (closing - 90.0) % 180.0}, allow_long=False)
            except K.Unreachable as e:
                why = f"ไปไม่ได้: {e}"                       # เอื้อมไม่ถึง หรือ jaw จะชนรางนำ (ดักได้เฉพาะหลังปลายราง)
                continue
            return {"plan": plan, "t_hit": t_hit, "x": float(x), "y": float(y), "closing": closing}
        raise PickError(f"ไม่มีจุดดักที่ไปได้ ({why}) — บล็อกเร็วไป/เห็นช้าไป หรือโซนแคบไป")

    def _grip_ticks(self):
        """(tick ที่หุบรอ, เวลาที่ C,6 ใช้หุบจากจุดนั้นจนแตะบล็อก)"""
        rng = C.GRIP_TICK_RANGE
        if not rng:
            raise PickError("ยังไม่มี GRIP_TICK_RANGE — รัน repeat_test.py (หยิบนิ่ง) ให้ผ่านก่อน เพื่อรู้ tick ตอนหนีบบล็อก")
        contact = 0.5 * (rng[0] + rng[1])
        pre = min(float(C.GRIPPER_OPEN), contact + C.INTERCEPT_PREGAP_TICKS)
        return pre, (pre - contact) / C.GRIP_CRAWL_TICKS_PER_S

    # ------------------------------------------------------------ ครบรอบ
    def pick_moving(self, dry=False, wait_s=30.0):
        """scan -> รอบล็อก -> วัดความเร็ว -> ไปดัก -> ปิด jaw ตรงเวลา -> ยก
        dry=True: ทำทุกอย่างแต่ "ไม่ลง" ค้างที่ความสูง hover และปิด jaw ในอากาศตรงเวลา (ดูด้วยตาว่า jaw อยู่เหนือบล็อกพอดีไหม)
        คืน dict(ok, reason, speed, v, rms, t_hit_in, slack_s, late_s, x, y, stall)"""
        out = {"ok": False, "reason": "", "stall": None}
        a = self.arm
        self.goto_scan()
        if not self.wait_block(wait_s):
            out["reason"] = "ไม่เห็นบล็อกเข้ามาในเฟรม"
            return out
        trk = self.track()
        if trk is None:
            out["reason"] = "เห็นบล็อกไม่ต่อเนื่องพอจะวัดความเร็ว (ROI แคบไป/แสง/บล็อกออกนอกเฟรมเร็วไป)"
            return out
        out.update(speed=trk["speed"], v=[float(trk["v"][0]), float(trk["v"][1])], rms=trk["rms"], n=trk["n"])
        if trk["speed"] < C.INTERCEPT_MIN_SPEED:
            out["reason"] = f"บล็อกนิ่ง ({trk['speed']:.1f} mm/s) — ใช้ pick_block.py"
            return out
        if trk["speed"] > C.INTERCEPT_MAX_SPEED or trk["rms"] > C.INTERCEPT_MAX_RMS_MM:
            out["reason"] = f"ค่าวัดไม่น่าเชื่อ: {trk['speed']:.0f} mm/s, fit คลาด {trk['rms']:.1f} mm"
            return out
        try:
            pre, t_close = self._grip_ticks()
            hit = self.plan_intercept(trk, _now() + C.INTERCEPT_ARM_TIME_S)
        except PickError as e:
            out["reason"] = str(e)
            return out
        plan, t_hit = hit["plan"], hit["t_hit"]
        t_send = t_hit - t_close + C.INTERCEPT_TIME_BIAS_S     # เวลาที่ต้องส่ง C,6
        out.update(x=hit["x"], y=hit["y"], t_hit_in=t_hit - _now())

        # ---- แขนไปท่าดัก (gripper หุบรอไปพร้อมกัน) ----
        a.move_smooth({**plan["hover"], "gripper": pre}, duration=1.2)
        if not dry:
            for wp in plan["down"][:-1]:
                a.move_smooth(wp, duration=0.25)
            a.move_smooth(plan["down"][-1], duration=0.4, fine=True)
        slack = t_send - _now()                   # เวลาที่เหลือก่อนต้องปิด jaw (+ = ทัน)
        out["slack_s"] = slack
        if slack < 0.05:                                    # มาไม่ทัน: บล็อกผ่าน/กำลังผ่าน -> ถอยขึ้น ไม่หนีบ
            self._retreat(plan, dry)
            out["reason"] = f"แขนถึงท่าดักช้าไป {-slack:.2f} s — เพิ่ม INTERCEPT_ARM_TIME_S"
            return out

        # ---- รอ แล้วปิด jaw ตรงเวลา ----
        _sleep_until(t_send)
        out["late_s"] = _now() - t_send
        try:
            stall = a.close_gripper()
        except Exception as e:
            if type(e).__name__ != "GripError":
                raise
            a.open_gripper()
            self._retreat(plan, dry)
            out["reason"] = "ทดสอบแบบไม่ลง (dry): ปิด jaw ในอากาศตรงเวลาแล้ว" if dry else f"หนีบไม่ติด: {e}"
            out["ok"] = dry
            return out
        out["stall"] = stall
        rng = C.GRIP_TICK_RANGE
        if dry or (stall is not None and not (rng[0] <= stall <= rng[1])):
            a.open_gripper()
            self._retreat(plan, dry)
            out["reason"] = "dry" if dry else f"gripper หยุดที่ {stall} tick นอกช่วงบล็อก {list(rng)} — หนีบพลาด"
            out["ok"] = dry
            return out
        self._retreat(plan, dry)
        out["ok"] = True
        return out

    def _retreat(self, plan, dry):
        if not dry:
            self._up(plan)
