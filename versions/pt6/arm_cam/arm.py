"""arm.py — ตัวห่อ (wrapper) บางๆ รอบ LeRobot SO101Follower
แยกไว้ไฟล์เดียว: ถ้าบน Pi ไม่อยากลง LeRobot (ลาก torch มาด้วย) ก็เขียนคลาส Arm นี้ใหม่ด้วย
feetech-servo-sdk โดย interface เท่าเดิม (connect/read/send/move_smooth/torque_off/disconnect) โค้ดที่เหลือไม่ต้องแก้
"""
import time
import config as C

# ชื่อ module ของ LeRobot เปลี่ยนตามเวอร์ชัน ลองทั้งแบบใหม่และแบบเก่า
try:
    from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
except ImportError:
    from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


class Arm:
    def __init__(self):
        # use_degrees=True: ข้อต่อรับ/ส่งเป็นองศา, max_relative_target: จำกัดการขยับต่อคำสั่ง กันกระชาก
        cfg = SO101FollowerConfig(port=C.ARM_PORT, id=C.ARM_ID, use_degrees=True,
                                  max_relative_target=C.MAX_STEP_DEG)
        self.robot = SO101Follower(cfg)

    def connect(self):
        self.robot.connect()          # โหลดไฟล์ calibration ตาม ARM_ID (ถ้าไม่มีจะเข้าโหมด calibrate)

    def disconnect(self):
        self.robot.disconnect()

    def torque_off(self):
        """ปล่อย motor ให้จับแขนขยับด้วยมือได้ (ใช้ตอนบันทึกท่า/วัด offset)"""
        self.robot.bus.disable_torque()

    def read(self):
        """อ่านตำแหน่งข้อต่อปัจจุบัน -> dict ชื่อข้อต่อ: ค่า"""
        obs = self.robot.get_observation()
        return {j: float(obs[f"{j}.pos"]) for j in JOINTS}

    def send(self, target):
        """ส่งเป้าหมายทันที 1 ครั้ง (ข้อต่อไหนไม่ระบุ = ค้างค่าเดิม)"""
        cur = self.read()
        cur.update(target)
        self.robot.send_action({f"{j}.pos": float(v) for j, v in cur.items()})

    def move_smooth(self, target, duration=2.0, fine=False):   # fine: มีไว้ให้ interface ตรงกับ arm_mega (ไม่ใช้)
        """ขยับจากตำแหน่งปัจจุบันไปเป้าหมายแบบนุ่ม (smoothstep) แทนการ sleep เฉยๆ
        ความเร็วเริ่ม/จบเป็นศูนย์ -> ลดแรงกระชากบนเกียร์ STS3215"""
        start = self.read()
        goal = dict(start)
        goal.update(target)
        steps = max(1, int(duration * C.MOVE_HZ))
        for i in range(1, steps + 1):
            t = i / steps
            s = t * t * (3 - 2 * t)                        # smoothstep 0->1
            cmd = {j: start[j] + (goal[j] - start[j]) * s for j in JOINTS}
            self.robot.send_action({f"{j}.pos": v for j, v in cmd.items()})
            time.sleep(1.0 / C.MOVE_HZ)

    # --- interface เดียวกับ arm_mega.py: pick_block.py เรียกสองตัวนี้โดยไม่สนว่า backend ไหน ---
    def open_gripper(self):
        self.move_smooth({"gripper": C.GRIPPER_OPEN}, duration=1.0)

    def close_gripper(self):
        self.move_smooth({"gripper": C.GRIPPER_CLOSE}, duration=1.0)

    def torque_on(self):
        self.robot.bus.enable_torque()
