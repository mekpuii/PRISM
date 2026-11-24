"""arm_select.py — เลือก backend ของแขนตาม config.ARM_BACKEND ("mega" หรือ "lerobot")
สคริปต์อื่นเรียก make_arm() ที่เดียว ไม่ต้องรู้ว่าคุยกับแขนทางไหน (import เฉพาะ backend ที่ใช้ ไม่ลาก LeRobot มาถ้าไม่ได้ใช้)
"""
import config as C

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def make_arm(link=None):
    """link: (เฉพาะ mega) MegaLink ที่เปิดอยู่แล้ว เช่นของ PRISM — ใช้พอร์ตร่วมกันแทนการเปิดซ้ำ"""
    if C.ARM_BACKEND == "mega":
        from arm_mega import Arm
        return Arm(link)
    elif C.ARM_BACKEND == "lerobot":
        from arm import Arm
    else:
        raise ValueError(f"ARM_BACKEND ต้องเป็น 'mega' หรือ 'lerobot' (ได้ '{C.ARM_BACKEND}')")
    return Arm()
