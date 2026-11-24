"""grip_vision.py — ให้กล้องข้อมือ "เห็น" ว่าตอนหนีบบล็อกติดจริงไหม (ใช้ร่วมกับ picker.py)

หลักการ: เฟิร์มแวร์บอกได้แค่ "jaw หยุดแล้ว" (หนีบอากาศก็หยุด) จึงใช้กล้องเป็นตัวตัดสินเสริม
  1) snap_pre()  : แขนอยู่ท่า hover เหนือบล็อก (ก่อนลง) -> ถ่ายภาพอ้างอิง
  2) แขนลง -> หนีบ -> ยกกลับ "ท่า hover เดิมเป๊ะ"
  3) verify()    : ถ่ายภาพอีกใบที่ท่าเดิม แล้วเทียบ
        บล็อกถูกยกขึ้นมากับ jaw  -> ที่เดิมในภาพเปลี่ยนมาก (บล็อกหายจากโต๊ะ / ลอยมาอยู่ที่ปลาย jaw ใกล้กล้องขึ้น)
        หนีบพลาด บล็อกยังอยู่บนโต๊ะ -> ภาพสองใบเหมือนกันแทบเป๊ะ
  ไม่ต้องรู้ว่ากล้องติดตรงไหน/มุมไหน — เทียบ "ภาพที่ท่าเดิม" เท่านั้น
ข้อจำกัด: ถ้าบล็อกไม่อยู่ในภาพตอน hover (ถูกบังด้วย jaw ฯลฯ) จะตอบ "unknown" ไม่เดา
ค่าตัดสินด้านล่างเป็นค่าเริ่มต้นที่ "ยังไม่ได้ปรับกับกล้องจริง": ดูตัวเลขที่พิมพ์ + ภาพใน grip_frames/ แล้วปรับ
"""
import os
import time

import cv2
import numpy as np

import camera

CENTER_FRAC = 0.7          # ดูเฉพาะกลางภาพ 70% (ขอบภาพมีผนัง/สายที่ขยับเพราะแขนสั่น)
PIX_DELTA = 40             # พิกเซลที่ต่างกันเกินนี้ (ระดับเทา 0-255) ถือว่า "เปลี่ยน"
CHANGED_FRAC_MIN = 0.04    # สัดส่วนพิกเซลที่เปลี่ยน >= นี้ -> "held" (บล็อกถูกยกออกจากโต๊ะ)
VISIBLE_FRAC = (0.02, 0.70)  # สัดส่วนพิกเซลสว่าง (Otsu) ตอน pre ต้องอยู่ในช่วงนี้ จึงถือว่า "เห็นบล็อก"
MIN_CONTRAST = 40          # และกลุ่มสว่างต้องสว่างกว่ากลุ่มมืดอย่างน้อยนี้ (ไม่งั้นเป็นแค่ noise ที่ Otsu ผ่าครึ่ง)
SETTLE_S = 0.5             # รอแขนนิ่งก่อนถ่าย (กันภาพเบลอจากการสั่น)
AVG_FRAMES = 4             # เฉลี่ยกี่เฟรมต่อภาพ (ลด noise)


class GripWatch:
    def __init__(self, cap, save_dir="grip_frames", window=None, strict=False):
        """cap: กล้องที่เปิดไว้แล้ว | window: ชื่อหน้าต่าง cv2 ที่จะโชว์ภาพสดระหว่างถ่าย (None = ไม่โชว์)
        strict: True = ถ้ากล้องฟันธงว่า not_held ให้ picker ถือว่าหนีบพลาด (ปล่อย+raise); False = แค่บันทึก/แสดง"""
        self.cap, self.window, self.strict = cap, window, strict
        self.save_dir = None if save_dir is None else os.path.join(os.path.dirname(os.path.abspath(__file__)), save_dir)
        self.pre = None
        self.last = None

    # ---------- ถ่าย ----------
    def _show(self, g, text, col=(0, 255, 255)):
        if self.window is None or g is None:
            return
        v = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        cv2.putText(v, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 3)
        cv2.imshow(self.window, v)
        cv2.waitKey(1)

    def _avg(self, label):
        t_end = time.monotonic() + SETTLE_S
        g = None
        while time.monotonic() < t_end:                    # รอนิ่งแต่ยังโชว์ภาพสด (หน้าต่างไม่ค้าง)
            g = camera.grab(self.cap)
            self._show(g, label)
        fr = [f for f in (camera.grab(self.cap) for _ in range(AVG_FRAMES)) if f is not None]
        if not fr:
            return None
        out = np.mean(np.stack(fr).astype(np.float32), axis=0).astype(np.uint8)
        self._show(out, label)
        return out

    def snap_pre(self):
        """ท่า hover ก่อนลง"""
        self.pre = self._avg("GRIP: before descent")
        return self.pre is not None

    # ---------- ตัดสิน ----------
    @staticmethod
    def compare(pre, post):
        """คืน dict(verdict, visible, vis_frac, changed_frac)  verdict: held / not_held / unknown"""
        h, w = pre.shape[:2]
        my, mx = int(h * (1 - CENTER_FRAC) / 2), int(w * (1 - CENTER_FRAC) / 2)
        a = cv2.GaussianBlur(pre[my:h - my, mx:w - mx], (9, 9), 0)
        b = cv2.GaussianBlur(post[my:h - my, mx:w - mx], (9, 9), 0)
        thr, _ = cv2.threshold(a, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        hi, lo = a[a > thr], a[a <= thr]
        contrast = float(hi.mean() - lo.mean()) if hi.size and lo.size else 0.0
        vis_frac = float(np.mean(a > thr))
        visible = VISIBLE_FRAC[0] <= vis_frac <= VISIBLE_FRAC[1] and contrast >= MIN_CONTRAST
        changed = float(np.mean(cv2.absdiff(a, b) > PIX_DELTA))
        if not visible:
            verdict = "unknown"
        else:
            verdict = "held" if changed >= CHANGED_FRAC_MIN else "not_held"
        return {"verdict": verdict, "visible": visible, "vis_frac": round(vis_frac, 3), "contrast": round(contrast), "changed_frac": round(changed, 3)}

    def verify(self):
        """ท่า hover หลังยก: เทียบกับ pre (ต้อง snap_pre มาก่อน)"""
        if self.pre is None:
            self.last = {"verdict": "unknown", "visible": False, "vis_frac": 0.0, "changed_frac": 0.0}
            return self.last
        post = self._avg("GRIP: after lift")
        if post is None:
            self.last = {"verdict": "unknown", "visible": False, "vis_frac": 0.0, "changed_frac": 0.0}
            return self.last
        r = self.compare(self.pre, post)
        self.last = r
        col = {"held": (0, 255, 0), "not_held": (0, 0, 255), "unknown": (0, 200, 255)}[r["verdict"]]
        self._show(post, f"CAMERA: {r['verdict'].upper()}  changed={r['changed_frac']:.2f}", col)
        if self.save_dir is not None:
            try:
                os.makedirs(self.save_dir, exist_ok=True)
                d = cv2.convertScaleAbs(cv2.absdiff(self.pre, post), alpha=3.0)         # ขยายความต่างให้เห็นชัด
                strip = np.hstack([self.pre, post, d])
                cv2.imwrite(os.path.join(self.save_dir, time.strftime("%Y%m%d_%H%M%S") + f"_{r['verdict']}.png"),
                            cv2.resize(strip, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
            except Exception:
                pass                                                                    # บันทึกภาพไม่ได้ไม่ควรทำให้การหยิบล้ม
        time.sleep(0.8 if self.window else 0)                                           # ให้เห็นผลบนหน้าจอสักครู่
        return r
