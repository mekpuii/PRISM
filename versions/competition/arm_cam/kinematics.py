"""kinematics.py — FK/IK ของ SO101 จาก "เรขาคณิตจริงใน CAD" (URDF so101_new_calib ของ TheRobotStudio/SO-ARM100)

ทำไมเขียนใหม่: รุ่นเดิมสมมติแขนเป็น 2 ลิงก์ตรงๆ และเดา TOOL=100 mm  แต่ CAD จริง
  * ท่อนบนมีมุมหัก ~14° ระหว่างตัวแขนกับเส้นเพลาไหล่->ศอก, ท่อนล่างหัก ~2°
  * ข้อมือ->ปลาย jaw ยาว ~159 mm และผิว jaw ตายตัวเยื้องจากแกน roll 7.9 mm
จึงใช้ FK เต็มรูปจาก URDF (คูณเมทริกซ์ทีละข้อ) แล้วแก้ IK แบบตัวเลข (Gauss-Newton) จากจุดตั้งต้นเชิงวิเคราะห์

กรอบหุ่น (mm): จุดกำเนิดอยู่ใต้แกน shoulder_pan ที่ระดับโต๊ะ, +x ข้างหน้า, +y ซ้าย, +z ขึ้น
มุมข้อต่อ q (เรเดียน) = นิยามเดียวกับ URDF: q=0 ทุกข้อ = "ท่า L" (ท่อนบนตั้ง ท่อนล่าง+gripper ราบชี้หน้า)
ค่า motor (tick):  tick = JOINT_ZERO + JOINT_DIR * q * UNITS_PER_RAD      (ดู config.py)

จุดบน gripper ที่ใช้ (พิกัดในเฟรม gripper_link ของ URDF, mm — นิ้วชี้ไปทาง -z, jaw เปิดไปทาง +x):
  TOUCH = ปลายสุดของ jaw ตายตัวด้านใน (จุดที่ jaw สองข้างประกบกันตอนปิด)  ใช้ตอนคาลิเบรตแบบ "เอาปลายแตะมาร์ก"
  GRASP = จุดกึ่งกลางบล็อกตอนหนีบ = ห่างผิว jaw ตายตัวเข้ามา (ครึ่งความกว้างบล็อก + ระยะเผื่อ) สูงจากปลาย jaw GRASP_UP_MM
"""
import math
import numpy as np
import config as C

JOINTS5 = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class Unreachable(Exception):
    """เป้าหมายอยู่นอก workspace หรือเกินขอบเขตข้อต่อ"""


# ------------------------------------------------------------ โซ่จลนศาสตร์จาก URDF
def _rpy(r, p, y):
    """เมทริกซ์หมุนแบบ URDF: R = Rz(y) Ry(p) Rx(r)"""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _T(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = _rpy(*rpy)
    m[:3, 3] = xyz
    return m


_PI = math.pi
# (xyz mm, rpy rad) ของ origin แต่ละ joint เทียบลิงก์แม่ — คัดจาก Simulation/SO101/so101_new_calib.urdf ทุกข้อหมุนรอบแกน z ของตัวเอง
_PAN_X = 38.8353                                           # แกน pan อยู่หน้า origin ของ base_link เท่านี้ (เราย้ายจุดกำเนิดมาไว้ที่แกน pan)
_CHAIN = [
    _T((0.0, 0.0, 62.4), (_PI, 0.0, -_PI)),                        # base -> shoulder_pan
    _T((-30.3992, -18.2778, -54.2), (-_PI / 2, -_PI / 2, 0.0)),    # -> shoulder_lift
    _T((-112.57, -28.0, 0.0), (0.0, 0.0, _PI / 2)),                # -> elbow_flex
    _T((-134.9, 5.2, 0.0), (0.0, 0.0, -_PI / 2)),                  # -> wrist_flex
    _T((0.0, -61.1, 18.1), (_PI / 2, 0.0486795, _PI)),             # -> wrist_roll (= gripper_link)
]


def _rotz(a):
    m = np.eye(4)
    c, s = math.cos(a), math.sin(a)
    m[0, 0], m[0, 1], m[1, 0], m[1, 1] = c, -s, s, c
    return m


def gripper_frame(q):
    """q: list/array 5 ค่า (rad ตาม URDF) -> เมทริกซ์ 4x4 ของ gripper_link ในกรอบหุ่น (mm)"""
    m = np.eye(4)
    m[2, 3] = C.BASE_Z_MM                                  # ฐานแขนสูงจากระนาบโต๊ะเท่าไร
    for Tj, qi in zip(_CHAIN, q):
        m = m @ Tj @ _rotz(qi)
    return m


def touch_point():
    """ปลาย jaw ตายตัวด้านใน ในเฟรม gripper_link (mm)"""
    return np.array([C.JAW_FACE_X_MM + C.JAW_PAD_MM, 0.0, -(C.JAW_TIP_LEN_MM), 1.0])


def grasp_point(half_width=None):
    """จุดกึ่งกลางบล็อกตอนหนีบ ในเฟรม gripper_link (mm)"""
    hw = C.BLOCK_SHORT_MM / 2.0 if half_width is None else half_width
    return np.array([C.JAW_FACE_X_MM + C.JAW_PAD_MM + hw + C.GRASP_CLEARANCE_MM, 0.0,
                     -(C.JAW_TIP_LEN_MM - C.GRASP_UP_MM), 1.0])


# ------------------------------------------------------------ tick <-> q
def q_to_motor(q):
    """q (rad, 5 ข้อ) -> dict ค่า motor (tick ใน backend mega)"""
    return {j: C.JOINT_ZERO[j] + C.JOINT_DIR[j] * qi * C.UNITS_PER_RAD for j, qi in zip(JOINTS5, q)}


def motor_to_q(motor):
    """dict ค่า motor -> q (rad, 5 ข้อ)"""
    return [(motor[j] - C.JOINT_ZERO[j]) / (C.JOINT_DIR[j] * C.UNITS_PER_RAD) for j in JOINTS5]


# ------------------------------------------------------------ FK
def fk(motor, point="grasp"):
    """ค่า motor -> (x, y, z, pitch_deg, closing_deg)
    x,y,z   : ตำแหน่งจุด point ("grasp" หรือ "touch") ในกรอบหุ่น (mm)
    pitch   : มุมก้มของนิ้ว (-90 = ชี้ลงดิ่ง, 0 = ราบ)
    closing : ทิศที่ jaw ปิด ในระนาบโต๊ะ (องศา 0-180)"""
    G = gripper_frame(motor_to_q(motor))
    p = G @ (grasp_point() if point == "grasp" else touch_point())
    finger = -G[:3, 2]                                     # นิ้วชี้ไปทาง -z ของ gripper_link
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, finger[2]))))
    closing = math.degrees(math.atan2(G[1, 0], G[0, 0])) % 180.0
    return float(p[0]), float(p[1]), float(p[2]), pitch, closing


# ------------------------------------------------------------ IK (หยิบจากด้านบน นิ้วชี้ลงดิ่ง)
def _seed(x, y, z):
    """จุดตั้งต้นเชิงวิเคราะห์: มองแขนเป็น 2 ลิงก์ในระนาบแนวตั้ง (ไหล่->ศอก->ข้อมือ) โดยข้อมืออยู่เหนือเป้า"""
    S = np.array([30.3992, 62.4 + 54.2 + C.BASE_Z_MM])     # เพลาไหล่ในระนาบ (r, z)
    v1, v2 = np.array([28.0, 112.57]), np.array([134.9, 5.2])     # ไหล่->ศอก, ศอก->ข้อมือ ที่ q=0
    l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
    f10, f20 = math.atan2(v1[1], v1[0]), math.atan2(v2[1], v2[0])
    r = math.hypot(x, y)
    W = np.array([r, z + 61.1 + C.JAW_TIP_LEN_MM - C.GRASP_UP_MM]) - S      # ข้อมืออยู่เหนือจุดหนีบ
    d = np.linalg.norm(W)
    if d > l1 + l2 - 1.0 or d < abs(l1 - l2) + 1.0:
        raise Unreachable(f"เอื้อมไม่ถึง (ไหล่->ข้อมือ {d:.0f} mm, ทำได้ {abs(l1 - l2):.0f}-{l1 + l2:.0f})")
    cosb = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    b = -math.acos(max(-1.0, min(1.0, cosb)))              # ลบ = ศอกอยู่ด้านบน (elbow-up)
    f1 = math.atan2(W[1], W[0]) - math.atan2(l2 * math.sin(b), l1 + l2 * math.cos(b))
    f2 = f1 + b
    q_lift = f10 - f1                                      # URDF: + = ก้มไปข้างหน้า (หมุนตามเข็มในระนาบ r-z)
    q_elbow = (f20 - f2) - q_lift
    q_wrist = math.pi / 2 - q_lift - q_elbow               # ผลรวม = 90° -> นิ้วชี้ลงดิ่ง
    q_pan = -math.atan2(y, x)                              # URDF: + = หมุนตามเข็มมองจากบน
    return [q_pan, q_lift, q_elbow, q_wrist]


def _residual(q, target, d):
    G = gripper_frame(q)
    p = (G @ target[1])[:3] - target[0]
    zg, xg = G[:3, 2], G[:3, 0]
    # นิ้วลงดิ่ง: แกน z ของ gripper ต้องชี้ +z โลก (x,y = 0) | jaw ปิดตามทิศ d (ขนานกัน ทิศไหนก็ได้)
    return np.array([p[0], p[1], p[2], 100.0 * zg[0], 100.0 * zg[1], 100.0 * (xg[0] * d[1] - xg[1] * d[0])])


def _solve(q0, target, d):
    q = np.array(q0, dtype=float)
    for _ in range(40):
        r = _residual(q, target, d)
        if np.abs(r).max() < 1e-6:
            break
        J = np.zeros((6, 5))
        for i in range(5):                                 # Jacobian เชิงตัวเลข (5 ตัวแปร ถูกมาก)
            dq = q.copy()
            dq[i] += 1e-6
            J[:, i] = (_residual(dq, target, d) - r) / 1e-6
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        n = np.abs(step).max()
        q = q + (step if n < 0.5 else step * (0.5 / n))    # จำกัดก้าว กันกระโดดข้ามกิ่งคำตอบ
    r = _residual(q, target, d)
    return q, float(np.abs(r).max())


def ik(x, y, z, closing_deg, point="grasp", near=None, half_width=None):
    """ให้จุด point ("grasp"/"touch") อยู่ที่ (x, y, z) mm โดยนิ้วชี้ลงดิ่ง และ jaw ปิดตามทิศ closing_deg (กรอบหุ่น)
    สำหรับบล็อก: closing_deg = ทิศแกนยาว + 90 (หนีบขวางด้านสั้น 30 mm)
    near: dict motor ของท่าก่อนหน้า (ถ้ามี) — เลือกคำตอบที่ wrist_roll ใกล้ท่านั้นที่สุด
          (jaw หนีบได้ 2 ทิศต่างกัน 180°; ถ้าไม่ล็อกกิ่งคำตอบ ข้อมืออาจหมุนกลับ 180° กลางทางลง)
    คืน dict ค่า motor 5 ข้อต่อ (ไม่รวม gripper) หรือ raise Unreachable"""
    pt = grasp_point(half_width) if point == "grasp" else touch_point()   # half_width: ครึ่งความกว้างของด้านที่หนีบ (None = ด้านสั้น)
    target = (np.array([x, y, z], dtype=float), pt)
    a = math.radians(closing_deg)
    d = (math.cos(a), math.sin(a))
    base = _seed(x, y, z)
    # --- หาคำตอบทุกกิ่ง: jaw หนีบได้ 2 ทิศ (roll ต่างกัน 180°) จึงมีได้ 2 คำตอบ ---
    sols = []

    def add(q0):
        q, err = _solve(q0, target, d)
        if err > 1e-3 or gripper_frame(q)[2, 2] < 0.99:    # ไม่ลู่เข้า หรือ นิ้วชี้ขึ้น (คำตอบกลับหัว)
            return
        q[4] = (q[4] + math.pi) % (2 * math.pi) - math.pi
        if not any(abs(q[4] - s[4]) < 0.05 for s in sols):
            sols.append(q)

    for roll0 in (0.0, math.pi / 2, -math.pi / 2, math.pi):
        add(base + [roll0])
    for q in list(sols):                                   # คู่แฝดของทุกคำตอบ: บิด roll อีก 180° แล้วขัดเกลาใหม่
        for s in (math.pi, -math.pi):
            add(list(q[:4]) + [q[4] + s])
    best, why = None, "แก้ IK ไม่ลู่เข้า (ใกล้สุดระยะเอื้อม?)"
    for q in sols:
        motor = q_to_motor(q)
        bad = [f"{j}={v:.0f} นอก [{C.JOINT_LIMITS[j][0]}, {C.JOINT_LIMITS[j][1]}]" for j, v in motor.items()
               if not (C.JOINT_LIMITS[j][0] + C.LIMIT_MARGIN <= v <= C.JOINT_LIMITS[j][1] - C.LIMIT_MARGIN)]
        if bad:
            why = "เกินขอบเขตข้อต่อ: " + ", ".join(bad)
            continue
        if near is None:
            cost = abs(q[4])                               # ท่าแรก: เลือกคำตอบที่บิดข้อมือน้อยสุด
        else:
            cost = abs(motor["wrist_roll"] - near["wrist_roll"])   # ท่าถัดไป: อยู่กิ่งเดียวกับท่าก่อนหน้า
        if best is None or cost < best[0]:
            best = (cost, motor)
    if best is None:
        raise Unreachable(f"({x:.0f}, {y:.0f}, {z:.0f}) {why}")
    return best[1]


def table_z(x, y):
    """ระดับโต๊ะที่ (x, y) ในพิกัดของ FK (mm) ตามที่แขนวัดเองตอน calib_touch.py (ยังไม่วัด = 0)
    ใช้บวกเข้ากับความสูงที่ต้องการ: z_สั่ง = table_z(x, y) + ความสูงเหนือโต๊ะ"""
    a, b, c = C.TABLE_PLANE
    return a + b * x + c * y


def descend(x, y, z_from, z_to, closing_deg, step_mm=15.0, near=None, half_width=None):
    """จุดผ่านสำหรับลง/ขึ้นเป็น "เส้นตรงแนวดิ่ง" (การเคลื่อนระหว่างสองท่าเป็นเส้นตรงในปริภูมิข้อต่อ ปลายแขนจะวาดโค้ง
    ถ้าไปรวดเดียว -> ซอยเป็นช่วงละ step_mm ให้โค้งเหลือไม่ถึง 1 mm)  คืน list ของ dict motor จาก z_from ถึง z_to"""
    n = max(1, int(math.ceil(abs(z_to - z_from) / step_mm)))
    out = []
    for i in range(1, n + 1):
        near = ik(x, y, z_from + (z_to - z_from) * i / n, closing_deg, near=near, half_width=half_width)   # ล็อกกิ่ง roll ตามจุดก่อนหน้า
        out.append(near)
    return out


def jaw_points(motor, open_mm):
    """จุดตัวอย่างบน jaw ทั้งสองข้าง (ช่วง 45 mm จากปลาย) ในกรอบหุ่น (mm) -> array N x 3  ใช้ตรวจชนสิ่งกีดขวาง
    เรขาคณิตจาก CAD แผ่น M18 (เฟรม gripper: x = แกนปิด jaw, ปลาย jaw อยู่ที่ z = -JAW_TIP_LEN):
      jaw ตายตัว : x จากผิวสัมผัส (JAW_FACE_X + pad) ถอยไป 17.5 mm, y +-9.5
      jaw ที่ขยับ: ผิวสัมผัสห่างจาก jaw ตายตัว open_mm แล้วหนาออกไป 24.5 mm  (คิดเป็นกล่องตรง ไม่คิดว่าปลายยกขึ้นตอนอ้า = เผื่อไว้)"""
    G = gripper_frame(motor_to_q(motor))
    face = C.JAW_FACE_X_MM + C.JAW_PAD_MM
    xs = [face - 17.5, face - 8.0, face, face + open_mm, face + open_mm + 12.0, face + open_mm + 24.5]
    pts = [(x, y, -C.JAW_TIP_LEN_MM + dz, 1.0) for x in xs for y in (-9.5, 0.0, 9.5) for dz in (0.0, 8.0, 16.0, 30.0, 45.0)]
    return (G @ np.array(pts).T).T[:, :3]
