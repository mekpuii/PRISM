"""picker.py — ตรรกะ "เห็นบล็อก -> หยิบ -> วาง" เป็นโมดูลเดียว ใช้ร่วมกันทุกโปรแกรม
(pick_block.py, repeat_test.py และโค้ดของเพื่อนบน Pi เรียกคลาส Picker ตัวนี้)

ตัวอย่างใช้จากโค้ดอื่น (เช่น arm_node ของ PRISM ที่ถือ MegaLink อยู่แล้ว):
    from picker import Picker
    p = Picker(link=my_megalink)          # ใช้พอร์ตร่วมกัน (ไม่เปิดซ้ำ)  หรือ Picker() ให้เปิดเอง
    r = p.pick_visible()                  # ไปท่า scan -> หา -> หยิบ -> ยก ; คืน dict ผล
    if r["ok"]: p.place()                 # ไปท่าวางที่สอนไว้ แล้วปล่อย
    p.close()
ทุกเมธอด "ไม่ถามผู้ใช้" (ไม่มี input) จึงรันแบบ headless ได้ — การยืนยันเป็นหน้าที่ของสคริปต์ที่เรียก
"""
import time

import camera
import config as C
import detect_block as D
import kinematics as K
import obstacles as O
from arm_select import make_arm


class PickError(RuntimeError):
    """หยิบไม่สำเร็จด้วยเหตุที่รู้ (ไม่เจอบล็อก / เอื้อมไม่ถึง / หนีบไม่ติด) — แขนอยู่ในสภาพปลอดภัย"""


class Picker:
    def __init__(self, link=None, use_arm=True, cap=None):
        self.H = D.load_homography()
        self.cap = cap if cap is not None else camera.open_camera()
        self._own_cap = cap is None
        self.arm = None
        self.watch = None                                  # GripWatch (grip_vision.py) ถ้าตั้ง กล้องจะเช็กว่าหนีบติดไหม
        self.vision = None                                 # ผลของกล้องรอบล่าสุด
        self.grip_current = None                           # กระแส gripper ตอนหนีบ (ดิบ จาก telemetry)
        if use_arm:
            self.arm = make_arm(link) if link is not None else make_arm()
            self.arm.connect()

    def close(self):
        if self._own_cap:
            self.cap.release()
        if self.arm is not None:
            self.arm.disconnect()

    # ------------------------------------------------------------ มอง
    def scan(self, open_gripper=True):
        """ไปท่า scan (โหมดแม่นยำ: ต้องเป็นท่าเดียวกับตอน calib_touch) แล้วคืนบล็อกที่เห็น หรือ None"""
        if self.arm is not None:
            if open_gripper:
                self.arm.open_gripper()
            self.arm.move_smooth(C.SCAN_POSE, duration=3.0, fine=True)
            time.sleep(0.6)                                # รอสั่นหมดก่อนถ่าย
        return D.detect_stable(self.cap, self.H)

    def plan(self, block, allow_long=True, z_grasp_mm=None):
        """บล็อก -> dict(closing, hover, down[...], width, side)  raise K.Unreachable ถ้าไปไม่ได้ (ยังไม่ขยับแขน)
        ลองหนีบ 2 แบบตามลำดับ แล้วใช้แบบแรกที่ (ก) IK ไปได้ทั้งทางลง (ข) jaw ไม่ชนรางนำ (obstacles.py):
          1) "short": jaw ปิดขวางด้านสั้น 30 mm  (ค่าเดิม ใช้เมื่อบล็อกอยู่ที่โล่ง)
          2) "long" : jaw ปิดขวางด้านยาว 40 mm   (ใช้เมื่อบล็อกอยู่ในรางนำ: jaw อยู่หน้า-หลังบล็อกตามแนวราง)
        allow_long=False: ลองเฉพาะแบบ 1 (การดักบล็อกเคลื่อนที่ต้องให้ jaw คร่อมทางวิ่งเท่านั้น)"""
        x, y = block["x"], block["y"]
        tz = K.table_z(x, y)                               # ระดับโต๊ะตามที่แขนวัดเอง
        zg = C.Z_GRASP_MM if z_grasp_mm is None else z_grasp_mm     # z_grasp_mm: ใช้ตอนเรียนรู้ความสูง (grip_learn.py)
        z_hover, z_grasp = tz + C.Z_HOVER_MM, tz + max(zg, C.Z_MIN_MM)
        cands = [("short", (block["long_deg"] + 90.0) % 180.0, C.BLOCK_SHORT_MM)]
        if allow_long:
            cands.append(("long", block["long_deg"] % 180.0, C.BLOCK_LONG_MM))
        open_mm, why = C.GRIPPER_OPEN_MM, []
        for side, closing, width in cands:
            if side == "long" and open_mm is None:
                why.append("long: ยังไม่ได้วัด GRIPPER_OPEN_MM (teach_rail.py)")
                continue
            if open_mm is not None and open_mm < width + C.GRASP_CLEARANCE_MM + 3.0:
                why.append(f"{side}: gripper อ้า {open_mm:.0f} mm ไม่พอสำหรับด้าน {width:.0f} mm")
                continue
            try:
                hover = K.ik(x, y, z_hover, closing, half_width=width / 2.0)
                down = K.descend(x, y, z_hover, z_grasp, closing, near=hover, half_width=width / 2.0)   # ลงดิ่ง ซอยช่วงละ 15 mm
            except K.Unreachable as e:
                why.append(f"{side}: {e}")
                continue
            seq = [hover] + down
            if max(abs(a["wrist_roll"] - b["wrist_roll"]) for a, b in zip(seq, seq[1:])) > 300:
                why.append(f"{side}: ทางลงต้องพลิกข้อมือ 180° กลางทาง (ชิดขอบ wrist_roll)")
                continue
            hit = O.check_path(seq, open_mm if open_mm is not None else 60.0)
            if hit:
                why.append(f"{side}: {hit}")
                continue
            return {"closing": closing, "hover": hover, "down": down, "width": width, "side": side}
        raise K.Unreachable(" | ".join(why))

    # ------------------------------------------------------------ หยิบ / วาง
    def grasp(self, plan):
        """ทำตามแผน: เหนือบล็อก -> ลงดิ่ง -> หนีบ -> ยกดิ่ง  คืน tick ที่ gripper หยุด
        raise PickError ถ้าเฟิร์มแวร์บอกว่าหนีบไม่ติด หรือ tick ที่หยุดไม่ใช่ความกว้างบล็อก (แขนถอยขึ้นแล้ว)"""
        a, down = self.arm, plan["down"]
        a.move_smooth(plan["hover"], duration=2.5, fine=True)
        self.vision = None
        if self.watch is not None:
            self.watch.snap_pre()                          # ภาพอ้างอิงที่ท่า hover ก่อนลง
        for wp in down[:-1]:
            a.move_smooth(wp, duration=0.5)
        a.move_smooth(down[-1], duration=0.6, fine=True)
        self.grasp_error = dict(a.last_error)              # ค่าคลาด tick ที่จุดหนีบ (ไว้บันทึก)
        try:
            stall = a.close_gripper()
        except Exception as e:
            if type(e).__name__ != "GripError":            # GripError = เฟิร์มแวร์ตอบ G,ERROR (อ่านตำแหน่ง gripper ไม่ได้)
                raise
            a.open_gripper()
            self._up(plan)
            raise PickError(f"หนีบไม่ติด: {e}")
        time.sleep(0.4)                                    # ให้แรงหนีบนิ่ง
        try:                                               # กระแสตอนหนีบ (ใช้ดูแรงหนีบ/แยกหนีบอากาศ) ไม่มีค่า = ข้าม
            self.grip_current = a.link.servo[a.ids["gripper"]].current_raw
        except Exception:
            self.grip_current = None
        long_side = plan.get("side") == "long"
        rng = C.GRIP_TICK_RANGE_LONG if long_side else C.GRIP_TICK_RANGE
        if rng and stall is not None and not (rng[0] <= stall <= rng[1]):
            a.open_gripper()                               # หนีบได้อย่างอื่น/หนีบอากาศ: ปล่อยแล้วถอยขึ้น
            self._up(plan)
            raise PickError(f"gripper หยุดที่ {stall} tick นอกช่วงบล็อก {'40' if long_side else '30'} mm {list(rng)} — ไม่ใช่บล็อก/หนีบพลาด")
        self._up(plan, fine=self.watch is not None)        # กลับท่า hover เดิมให้แม่นเพื่อให้ภาพก่อน/หลังเทียบกันได้
        if self.watch is not None:
            self.vision = self.watch.verify()
            if self.watch.strict and self.vision["verdict"] == "not_held":
                a.open_gripper()
                raise PickError(f"กล้องเห็นว่าบล็อกยังอยู่ที่เดิม (เปลี่ยนแค่ {self.vision['changed_frac']:.3f}) — หนีบไม่ติด")
        return stall

    def _up(self, plan, fine=False):
        for wp in reversed(plan["down"][:-1]):
            self.arm.move_smooth(wp, duration=0.5)
        self.arm.move_smooth(plan["hover"], duration=0.8, fine=fine)

    def put_back(self, plan):
        """วางคืนที่เดิม (ไว้ทดสอบซ้ำ)"""
        for wp in plan["down"]:
            self.arm.move_smooth(wp, duration=0.5)
        self.arm.open_gripper()
        self._up(plan)

    def place(self):
        """ไปท่าวางที่สอนไว้ (teach_place.py) แล้วปล่อยบล็อก — เคลื่อนเป็นเส้นตรงในปริภูมิข้อต่อจากเหนือจุดหยิบ"""
        if not C.PLACE_POSE:
            raise PickError("ยังไม่ได้สอนท่าวาง — รัน teach_place.py")
        pose = {j: v for j, v in C.PLACE_POSE.items() if j != "gripper"}
        self.arm.move_smooth(pose, duration=3.0)
        self.arm.open_gripper()
        time.sleep(0.3)

    def pick_visible(self):
        """ครบรอบ: scan -> plan -> grasp  คืน dict(ok, reason, block, stall)  (ไม่ raise สำหรับเหตุที่รู้)"""
        out = {"ok": False, "reason": "", "block": None, "stall": None}
        block = self.scan()
        if block is None:
            out["reason"] = "ไม่เจอบล็อกในเฟรม"
            return out
        out["block"] = block
        try:
            plan = self.plan(block)
        except K.Unreachable as e:
            out["reason"] = f"เอื้อมไม่ถึง: {e}"
            return out
        out["plan"] = plan
        try:
            out["stall"] = self.grasp(plan)
        except PickError as e:
            out["reason"] = str(e)
            return out
        out["ok"] = True
        return out
