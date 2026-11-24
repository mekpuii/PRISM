"""apply_limits.py — ตั้ง JOINT_LIMITS ใหม่ (ค่าที่วัดด้วย calib_limits.py 2026-11-11) โดยเก็บค่าเก่าแยกไว้

  JOINT_LIMITS           = ค่า "ใช้งานจริง" (ใหม่) — ไม่กว้างกว่าเฟิร์มแวร์ จึงไม่โดนบีบเงียบๆ
  JOINT_LIMITS_OLD       = ค่าเดิมก่อนแก้ (เก็บไว้ย้อนกลับ)
  JOINT_LIMITS_MEASURED  = ค่าที่วัดจริงจากมือ (รวมส่วนที่กว้างกว่าเฟิร์มแวร์ — ใช้ถ้าจะ flash Mega ภายหลัง)
  MEGA_SAFE              = สำเนาเฟิร์มแวร์ (ไม่แตะ)
  + สำรองไฟล์ calib.json ก่อนแก้เป็น calib_backup_<เวลา>.json
ย้อนกลับ:  python apply_limits.py --revert
"""
import shutil
import sys
import time

import config as C

NEW = {"shoulder_pan": [814, 3254], "shoulder_lift": [1055, 2998],
       "elbow_flex": [996, 3056], "wrist_flex": [813, 3201],      # 2 ข้อนี้วัดได้กว้างกว่าเฟิร์มแวร์ -> ใช้ตามเฟิร์มแวร์
       "wrist_roll": [298, 3859], "gripper": [1768, 3116]}        # roll หมุนข้ามขอบ 0/4095 -> คงเดิม
MEASURED = {"shoulder_pan": [784, 3284], "shoulder_lift": [1025, 3028], "elbow_flex": [873, 3098],
            "wrist_flex": [749, 3249], "wrist_roll": None, "gripper": None}

if "--revert" in sys.argv:
    old = C._load_calib().get("JOINT_LIMITS_OLD")
    if not old:
        raise SystemExit("ไม่มี JOINT_LIMITS_OLD ใน calib.json")
    C.save_calib(JOINT_LIMITS=old)
    raise SystemExit("ย้อนกลับเป็นค่าเก่าแล้ว")

if C.CALIB_FILE and __import__("os").path.exists(C.CALIB_FILE):
    bak = C.CALIB_FILE.replace("calib.json", time.strftime("calib_backup_%Y%m%d_%H%M%S.json"))
    shutil.copy2(C.CALIB_FILE, bak)
    print("สำรอง ->", bak)
saved_old = C._load_calib().get("JOINT_LIMITS_OLD")
old = saved_old or {j: list(v) for j, v in C.MEGA_SAFE.items()}   # ครั้งแรก: เก่า = ค่าเฟิร์มแวร์เดิม (ไม่ทับถ้ารันซ้ำ)
C.save_calib(JOINT_LIMITS=NEW, JOINT_LIMITS_OLD=old, JOINT_LIMITS_MEASURED=MEASURED)
print(f"{'ข้อ':14s}{'เก่า':>14s}{'ใหม่ (ใช้จริง)':>18s}")
for j in NEW:
    print(f"{j:14s}{str(tuple(old[j])):>14s}{str(tuple(NEW[j])):>18s}")
print("เสร็จ — รัน python doctor.py เพื่อตรวจ")
