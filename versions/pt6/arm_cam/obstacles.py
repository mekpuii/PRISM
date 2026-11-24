"""obstacles.py — ตรวจว่า jaw จะชน "รางนำสายพาน" (M17) หรือไม่ ก่อนขยับแขนจริง

รางนำเป็นผนังสองข้างของช่องวิ่ง (CAD: ช่องกว้าง 34 mm ผนังสูง 10.6 mm เหนือสายพาน ยาวจากต้นทางถึง "ปลายราง")
แบบจำลอง (ค่าจาก teach_rail.py เก็บใน C.RAIL — วัดด้วยแขนตัวเอง จึงอยู่ในพิกัดเดียวกับที่แขนใช้หยิบ):
    p1, p2 = มุมบนด้านในของผนังสองข้าง ณ ปลายราง     up = ทิศชี้เข้าหาตัวราง     z_top = ความสูงผนังเหนือผิวสายพาน
จุดใดๆ ถือว่า "ชน" เมื่อครบ 3 ข้อ (แต่ละข้อเผื่อ RAIL_MARGIN_MM):
    1) อยู่เลยปลายรางเข้าไปทางตัวราง     2) อยู่นอกช่องระหว่างผนัง (ทุกอย่างนอกช่องถือเป็นของแข็ง)     3) ต่ำกว่ายอดผนัง
ตรวจเฉพาะ jaw สองข้าง (ช่วง 45 mm จากปลาย) ที่ทุกจุดผ่านของทางลง — ตัวแขนส่วนอื่นอยู่สูงกว่าผนังมากตอนนิ้วชี้ลง
ไม่ครอบคลุม: สิ่งกีดขวางอื่น (เสากล้อง ถาด เซนเซอร์ IR) และการเคลื่อนระหว่างท่า scan <-> เหนือจุดหยิบ (สูง >= Z_HOVER_MM)
"""
import numpy as np

import config as C
import kinematics as K


def rail_hit(points):
    """points: array N x 3 (mm กรอบหุ่น) -> ข้อความอธิบายจุดแรกที่ชน หรือ None"""
    R = C.RAIL
    if not R:
        return None
    p1, p2, up = np.array(R["p1"], float), np.array(R["p2"], float), np.array(R["up"], float)
    w = p2 - p1
    gap = float(np.linalg.norm(w))                         # ความกว้างช่องระหว่างผนัง
    w /= gap
    m = C.RAIL_MARGIN_MM
    for q in np.asarray(points, float):
        if q[2] - K.table_z(q[0], q[1]) >= R["z_top"] + m:
            continue                                       # สูงกว่ายอดผนัง
        d = q[:2] - p1
        s, lat = float(d @ up), float(d @ w)               # s > 0 = เข้าไปในตัวราง ; lat = ตำแหน่งขวางช่อง (0..gap = ในช่อง)
        if s > -m and (lat < m or lat > gap - m):
            side = "ผนังฝั่ง p1" if lat < m else "ผนังฝั่ง p2"
            return f"jaw จะชนรางนำ ({side}: ลึกเข้าราง {s:.0f} mm, ห่างขอบช่อง {min(lat, gap - lat):.0f} mm)"
    return None


def check_path(motors, open_mm):
    """motors: list ของ dict ท่า (hover + ทางลง) ; open_mm: ระยะอ้าของ jaw -> ข้อความชน หรือ None"""
    if not C.RAIL:
        return None
    for mtr in motors:
        hit = rail_hit(K.jaw_points(mtr, open_mm))
        if hit:
            return hit
    return None
