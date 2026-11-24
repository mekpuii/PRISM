"""detect_block.py — หาบล็อก 40x30 mm จากภาพ gray แล้วคืนพิกัด mm + ทิศแกนยาว

หลักการ: threshold -> contour -> minAreaRect -> แปลงมุมทั้ง 4 เป็น mm ด้วย homography
แล้วเช็ก "ขนาดจริงเป็น mm" (ไม่ใช่สัดส่วน pixel) จึงกรองของที่ไม่ใช่บล็อกออกได้แม่นกว่า

รันตรงๆ เพื่อดูผลสด:  python detect_block.py          (PC: มีหน้าต่างภาพ)
                       python detect_block.py --nogui  (Pi headless: พิมพ์ค่าอย่างเดียว)
"""
import faulthandler as _fh
import os as _os
_FH = open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "crash_native.txt"), "w")
_fh.enable(file=_FH, all_threads=True)      # ถ้า Python ตายแบบไม่มี traceback (เช่น OpenCV/กล้อง/พอร์ตอนุกรมล่ม) จะเขียนว่าตายตรงไหนลงไฟล์นี้


def watchdog_kick(seconds=4.0):
    """เรียกทุกรอบของลูปหลัก: ถ้ารอบไหนค้างเกิน seconds วินาที จะเขียน "ค้างอยู่บรรทัดไหน" (ทุก thread) ลง crash_native.txt
    -> Not Responding ครั้งหน้า ส่งไฟล์นี้มาดูได้ทันทีว่าติดที่กล้อง / พอร์ต / การคำนวณ"""
    try:
        _fh.cancel_dump_traceback_later()
        _fh.dump_traceback_later(seconds, repeat=False, file=_FH, exit=False)
    except Exception:
        pass


def watchdog_off():
    try:
        _fh.cancel_dump_traceback_later()
    except Exception:
        pass

import math
import time
import sys
import cv2
import numpy as np
import config as C


def load_homography():
    """โหลดเมทริกซ์ pixel -> mm (ทำด้วย calib_homography.py)"""
    return np.load(C.HOMOGRAPHY_FILE)


LAST_REJECTS = []        # ก้อนที่เป็นสี่เหลี่ยมเต็มแต่ขนาด (mm) ไม่ตรงบล็อก: [(box_px, long_mm, short_mm)] — ไว้ดูว่าทำไมไม่เจอ


FAST = True           # True = ตอน TRACK ประมวลผลเฉพาะกรอบรอบบล็อก (ความละเอียดเต็ม, threshold ล็อกจากตอน SEARCH) | ตอน SEARCH ค้นทั้งภาพเสมอ | False = เต็มภาพทุกกรณี
DET_SCALE = 1.0       # ตรวจจับบนภาพย่อเท่านี้ (0.5 = เร็วขึ้น ~4 เท่า, พิกัดผลลัพธ์ผิดไม่เกิน ~1 px) ตั้ง 1.0 ถ้าต้องการเต็มความละเอียด
OPEN_BIG_PX = 65      # px: ของที่บางกว่านี้ในแนวใดแนวหนึ่งถูกตัดทิ้ง (บล็อกด้านสั้น ~150 px) ปรับลง ถ้ากล้องสูงขึ้นแล้วบล็อกเล็กลงในภาพ


UNIFORM_STD_MAX = 30.0   # ส่วนเบี่ยงเบนมาตรฐานของความสว่างภายในกรอบ ต้องไม่เกินนี้ (ผิวบล็อกเรียบ ~3-10; ผนังซี่/ราง ~20+) ถ้าบล็อกจริงโดนปัด ให้เพิ่ม
OTSU_CAP_ABOVE_DARK = 90  # threshold ไม่เกิน (พื้นมืด p10 + ค่านี้) เพื่อให้บล็อกสีส้ม (เทากลาง) ไม่หลุดเมื่อมีของขาวเยอะในภาพ
BLOCK_WHITE_GRAY = 215    # เทากลางผิวบล็อก >= นี้ เดาว่า "ขาว" ต่ำกว่า "ส้ม" (กล้องขาว-ดำ แยกสีจริงไม่ได้ ใช้แค่ป้ายช่วยดู; สีจริงให้ D415)
PREFER_W = 0.01           # มีหลายบล็อกในภาพ: ถ่วงน้ำหนักเลือกอันที่ใกล้ prefer_px (ค่าเริ่มต้น=กลางภาพ) — mm ต่อ px
EDGE_CONTRAST_MIN = 35    # ผิวบล็อกต้องสว่างกว่าวงแหวนรอบนอก (มัธยฐาน) อย่างน้อยนี้ (0-255) ผนัง/ลวดลายจะต่างน้อยกว่า
RING_DARK_FRAC_MIN = 0.55  # วงแหวนรอบบล็อกต้อง "มืด" (สายพาน) อย่างน้อยสัดส่วนนี้ — ผนังสว่างที่มีสายดำไม่กี่เส้นจะไม่ผ่าน
BORDER_PX = 3             # กรอบที่แตะขอบภาพ = บล็อกถูกตัด/เป็นผนังขอบภาพ ไม่เอา
RING_PX = 12             # ความกว้างวงแหวนรอบนอก (px)
UNIFORM_ERODE_PX = 25    # ถอยจากขอบเข้ามากี่ px ก่อนวัดความเรียบ
SAT_LEVEL = 245       # ระดับเทาที่ถือว่าอิ่มตัว (แสง LED)
SAT_FRAC_MAX = 1.01   # ปิดแล้ว (>1 = ไม่มีวันปัด): ตอนภาพสว่าง ผิวบล็อกขาวจริงจนอิ่มตัว ตัวนี้เลยปัดบล็อกจริงทิ้ง; แสง LED โดน uniformity/ขนาดกันอยู่แล้ว   # สัดส่วนพิกเซลอิ่มตัวสูงสุดที่ยอมรับในบล็อก (ถ้าบล็อกจริงโดนตัด ให้เพิ่ม/ลด CAM_EXPOSURE_VAL แทน)


def _refine_full_res(gray, best, H, bg, diff_thr, margin=8, thr=None):
    """ขัดเกลาผลที่ได้จากภาพย่อ: ครอปเฉพาะ "แปะเล็กรอบบล็อก" ที่ความละเอียดเต็ม แล้ววัด minAreaRect ใหม่
    ถูกมาก (พื้นที่เล็ก ~1 ms) แต่ได้ความแม่นเท่าเดิมที่ไม่ย่อภาพ; ถ้าผลใหม่ผิดปกติ (พื้นที่เปลี่ยน >20%) คงผลเดิม"""
    box = best["box_px"]
    x0, y0 = np.floor(box.min(axis=0)).astype(int) - margin
    x1, y1 = np.ceil(box.max(axis=0)).astype(int) + margin
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(gray.shape[1], x1), min(gray.shape[0], y1)
    if x1 - x0 < 10 or y1 - y0 < 10:
        return best
    crop = gray[y0:y1, x0:x1]
    blur = cv2.GaussianBlur(crop, (3, 3), 0)
    # โซนรอบบล็อก (กรอบเดิมขยายเล็กน้อย) — ใช้ทั้งจำกัดผลและคำนวณ threshold เฉพาะพิกเซลแถวนี้ (ไม่ให้ราง/ขาไฟในครอปดึงค่า Otsu เพี้ยน)
    zone = np.zeros(crop.shape, np.uint8)
    cv2.fillPoly(zone, [np.round((box - [x0, y0]) * 16).astype(np.int32)], 255, shift=4)
    zone = cv2.dilate(zone, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    if bg is not None:
        d = cv2.subtract(blur, bg[y0:y1, x0:x1]) if C.BLOCK_IS_BRIGHT else cv2.subtract(bg[y0:y1, x0:x1], blur)
        _, bw = cv2.threshold(d, diff_thr, 255, cv2.THRESH_BINARY)
    else:
        if thr is None:
            ring = cv2.dilate(zone, cv2.getStructuringElement(cv2.MORPH_RECT, (2 * margin + 1, 2 * margin + 1)))
            vals = blur[ring > 0].reshape(-1, 1)
            thr, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, bw = cv2.threshold(blur, thr, 255, cv2.THRESH_BINARY if C.BLOCK_IS_BRIGHT else cv2.THRESH_BINARY_INV)
    bw = cv2.bitwise_and(bw, zone)
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return best
    c = max(cnts, key=cv2.contourArea)
    rect = cv2.minAreaRect(c)
    old_area = cv2.contourArea(box.astype(np.float32))
    if old_area < 1 or abs(rect[1][0] * rect[1][1] - old_area) > 0.2 * old_area:
        return best
    nb = (cv2.boxPoints(rect) + [x0, y0]).astype(np.float32)
    mm = cv2.perspectiveTransform(nb[None], H)[0]
    e0, e1 = float(np.linalg.norm(mm[1] - mm[0])), float(np.linalg.norm(mm[2] - mm[1]))
    long_vec = (mm[1] - mm[0]) if e0 >= e1 else (mm[2] - mm[1])
    ctr = mm.mean(axis=0)
    out = dict(best)
    out.update(x=float(ctr[0]), y=float(ctr[1]), box_px=nb,
               long_deg=math.degrees(math.atan2(long_vec[1], long_vec[0])) % 180.0)
    return out


LAST_THR = [None]    # threshold ที่ Otsu เลือกในการเรียกล่าสุด (tracker เก็บไว้ใช้ซ้ำตอน TRACK)


SMOOTH_SD_MAX = 6.0       # px ที่ local std (หน้าต่าง SMOOTH_WIN_PX) เกินนี้ = มีลาย/เป็นขอบ ไม่ใช่ผิวบล็อก (บล็อกจริง ~1-3, แผ่นโลหะลาย ~20) None = ปิด
SMOOTH_WIN_PX = 9
SMOOTH_PRE_MEDIAN = 5     # median blur ก่อนวัดความเรียบ (0 = ปิด) — กันผิวเม็ดของบล็อกส้มทำให้กรอบกระพริบ
SMOOTH_CLOSE_PX = 0       # อุดรูเล็กในหน้ากากผิวเรียบ (0 = ปิด)
FILL_HOLES = False        # เติมรูข้างในก้อนก่อน open ใหญ่
RING_DIFF_GRAY = 25       # (โหมด SMOOTH) px รอบบล็อกที่เทาต่างจากผิวบล็อก >= ค่านี้ หรือมีลาย = "ต่าง"
RING_DIFF_FRAC_MIN = 0.6
RING_DARK_FRAC_SOFT = 0.25  # (โหมด SMOOTH) ยังต้องมีสายพานมืดรอบบล็อกอย่างน้อยเท่านี้ (บล็อกติด jaw/ราง ~0.3-0.5; เพดาน/ผนังสว่าง ~0)  # (โหมด SMOOTH) วงแหวนรอบบล็อกต้อง "ต่าง" อย่างน้อยสัดส่วนนี้ (ผนัง/รางที่ต่อเนื่อง ~0.1-0.4)
PX_MIN_EQ = 90            # (px_mode) ขนาดเทียบเท่าขั้นต่ำ (px) — บล็อกที่ระยะทำงาน ~150-300 px, เงาสะท้อน/ขาไฟ ~50-80
PX_RING_DARK_MIN = 0.42   # (px_mode) รอบบล็อกต้องเป็นสายพานมืดอย่างน้อยสัดส่วนนี้ (บล็อกจริง 0.46-0.99, ถังขาวบนพื้นสว่าง ~0.39)
PX_BORDER_PX = 25         # (px_mode) กรอบต้องห่างขอบภาพอย่างน้อยเท่านี้ (ถังที่โผล่ขอบภาพ ~15 px)
PX_ASPECT = (1.0, 1.6)    # (px_mode) สัดส่วนยาว/สั้นในภาพ (บล็อก 40/30 = 1.33, เผื่อมุมมองเอียง)
SPLIT_TOUCHING = False     # ก้อนสว่างที่ "ไม่ใช่สี่เหลี่ยม" (บล็อกติดกับ jaw/ราง/แผ่นโลหะ) -> แยกด้วย distance transform + watershed แล้วลองแต่ละส่วน
SPLIT_CORE_FRAC = (0.55, 0.4)   # แกนกลางของแต่ละชิ้น = จุดที่ห่างขอบก้อน >= สัดส่วนนี้ของระยะสูงสุด (ลองหลายค่า)


def _split_touching(cnts, small, min_area):
    """คืน contour ชุดใหม่: ก้อนที่เป็นสี่เหลี่ยมอยู่แล้วคงไว้ ก้อนที่เติมกรอบไม่เต็ม (<0.7) ถูกแยกเป็นชิ้นตามคอคอด/ขอบความสว่าง
    หลักการ: ส่วนหนาสุดของแต่ละวัตถุ (ยอด distance transform) = เมล็ด -> watershed บนภาพจริงหาเส้นแบ่งตามขอบความสว่าง"""
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        out.append(c)
        if a < min_area:
            continue
        r = cv2.minAreaRect(c)
        if r[1][0] * r[1][1] < 1 or a / (r[1][0] * r[1][1]) >= 0.7:
            continue                                            # เป็นสี่เหลี่ยมเต็มอยู่แล้ว ไม่ต้องแยก
        x, y, w, h = cv2.boundingRect(c)
        pad = 4
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(small.shape[1], x + w + pad), min(small.shape[0], y + h + pad)
        m = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.drawContours(m, [c - np.array([x0, y0])], -1, 255, -1)
        dist = cv2.distanceTransform(m, cv2.DIST_L2, 5)
        dmax = float(dist.max())
        if dmax < 3:
            continue
        img3 = cv2.cvtColor(small[y0:y1, x0:x1], cv2.COLOR_GRAY2BGR)
        for frac in SPLIT_CORE_FRAC:
            cores = (dist >= frac * dmax).astype(np.uint8)
            n, lab = cv2.connectedComponents(cores)
            if n <= 2:                                          # แกนเดียว -> แยกไม่ออกที่ระดับนี้ ลองระดับต่ำลง
                continue
            markers = (lab + 1).astype(np.int32)                # 1 = พื้นหลัง, 2.. = เมล็ดแต่ละชิ้น
            markers[(m > 0) & (cores == 0)] = 0                 # 0 = ยังไม่รู้ (ให้ watershed ตัดสิน)
            cv2.watershed(img3, markers)
            for k in range(2, n + 1):
                part = ((markers == k) & (m > 0)).astype(np.uint8) * 255
                pc, _ = cv2.findContours(part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for q in pc:
                    if cv2.contourArea(q) >= min_area:
                        out.append((q + np.array([x0, y0])).astype(np.int32))
            break
    return out


def find_block(gray, H, debug=False, roi=None, bg=None, diff_thr=35, tol_scale=1.0, thr=None, prefer_px=None,
               px_mode=False):
    """คืน dict(x, y, long_deg, score) หรือ None
    px_mode: True = ไม่ใช้ขนาด mm (แขนอยู่ท่าไหนก็ได้ homography ใช้ไม่ได้) -> ตัดสินด้วยขนาด/สัดส่วน px แทน
             ต้องใหญ่ >= PX_MIN_EQ, สัดส่วนยาว/สั้นใน PX_ASPECT แล้วเลือก "ก้อนใหญ่สุด" (เงาสะท้อนเล็กๆ ไม่ชนะ)
    x, y   : จุดกลางหน้าบนบล็อก (mm ในกรอบหุ่นยนต์)
    long_deg: ทิศของแกนยาว (40 mm) ในกรอบหุ่นยนต์ หน่วยองศา ช่วง [0,180)
    """
    # --- 1) ตัดเฉพาะ ROI ออกมาก่อน แล้วย่อภาพ DET_SCALE เท่า (ประหยัด CPU: ทุกขั้นหลังจากนี้ทำบนภาพเล็ก) ---
    # roi=None -> ใช้ C.DETECT_ROI | roi=(x0,y0,x1,y1) -> ใช้กรอบนั้น (เช่น ทั้งภาพตอนค้นหา หรือกรอบแคบตอนตาม)
    x0, y0, x1, y1 = C.DETECT_ROI if roi is None else roi
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(gray.shape[1], int(x1)), min(gray.shape[0], int(y1))
    if bg is not None and bg.shape != gray.shape:      # background.png จากกล้อง/ความละเอียดอื่น -> ใช้ไม่ได้ (ข้าม ไม่ให้โปรแกรมล้ม)
        bg = None
    roi_rect = (x0, y0, x1, y1)
    # ค้นทั้งภาพ (ROI ครอบเกือบทั้งภาพ) -> ประมวลผลเต็มภาพ + Otsu ทั้งภาพ | ตามบล็อก (ROI เล็ก) -> ครอปเฉพาะกรอบ
    crop_mode = FAST and (x1 - x0) * (y1 - y0) < 0.6 * gray.shape[0] * gray.shape[1]
    if not crop_mode:
        x0, y0, x1, y1 = 0, 0, gray.shape[1], gray.shape[0]
        thr = None
    sc = DET_SCALE if crop_mode else 1.0
    crop = gray[y0:y1, x0:x1]
    if crop.size == 0:
        return None
    small = cv2.resize(crop, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA) if sc != 1.0 else crop

    # --- 2) threshold ---
    blur = cv2.GaussianBlur(small, (5, 5), 0)
    if bg is not None:
        # มีภาพพื้นหลัง "ว่าง": เอาเฉพาะสิ่งที่ต่างจากพื้นหลัง -> ราง/jaw/ไฟ IR ที่อยู่นิ่งหายไปเอง
        bgs = cv2.resize(bg[y0:y1, x0:x1], (small.shape[1], small.shape[0]), interpolation=cv2.INTER_AREA)
        diff = cv2.subtract(blur, bgs) if C.BLOCK_IS_BRIGHT else cv2.subtract(bgs, blur)
        _, bw = cv2.threshold(diff, diff_thr, 255, cv2.THRESH_BINARY)
    else:
        # ไม่มี bg: Otsu (เลือกค่าเองตามภาพ)
        mode = cv2.THRESH_BINARY if C.BLOCK_IS_BRIGHT else cv2.THRESH_BINARY_INV
        if thr is None:
            t_used, bw = cv2.threshold(blur, 0, 255, mode + cv2.THRESH_OTSU)
            if C.BLOCK_IS_BRIGHT:
                # ฉากที่มีของขาวเยอะ (ผนัง/ราง/บล็อกขาว) ดัน Otsu สูงจนบล็อกสีส้ม (เทากลางๆ) หลุดไปอยู่ใต้เส้น -> จำกัดไม่ให้เกิน "สีพื้นมืด + OTSU_CAP"
                cap = float(np.percentile(blur, 10)) + OTSU_CAP_ABOVE_DARK
                if t_used > cap:
                    t_used = cap
                    _, bw = cv2.threshold(blur, t_used, 255, cv2.THRESH_BINARY)
            LAST_THR[0] = float(t_used)
        else:                                  # TRACK: ใช้ threshold ที่ได้ตอน SEARCH (Otsu ในกรอบเล็กๆ ผันผวนตามของสว่างที่หลุดเข้ากรอบ)
            _, bw = cv2.threshold(blur, thr, 255, mode)

    bw_raw = bw
    if SMOOTH_SD_MAX is not None:
        # ผิวบล็อกเรียบมาก (local std ~1-3) ส่วนแผ่นโลหะลาย/สายพานลาย/ขอบวัตถุ local std สูง -> ตัดออกก่อน
        # ผลพลอยได้: ขอบรอบบล็อก (std สูง) กลายเป็นร่องคั่น บล็อกจึงไม่ติดกับ jaw/ราง/แผ่นโลหะที่แตะกันอยู่
        # median ก่อน: ลบ "จุดลายเม็ดเล็ก" บนผิวบล็อก (บล็อกส้มมีผิวเป็นเม็ด) แต่ลายหยาบของแผ่นโลหะ/ขอบวัตถุยังอยู่
        _f = (cv2.medianBlur(small, SMOOTH_PRE_MEDIAN) if SMOOTH_PRE_MEDIAN else small).astype(np.float32)
        _ks = (SMOOTH_WIN_PX, SMOOTH_WIN_PX)
        _mu = cv2.blur(_f, _ks)
        _lsd = np.sqrt(np.maximum(cv2.blur(_f * _f, _ks) - _mu * _mu, 0.0))
        _smooth = (_lsd <= SMOOTH_SD_MAX).astype(np.uint8) * 255
        if SMOOTH_CLOSE_PX:                            # อุดรูเล็กๆ ในผิวเรียบ (ร่องขอบระหว่างวัตถุกว้างกว่านี้ จึงไม่ถูกอุด)
            _smooth = cv2.morphologyEx(_smooth, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (SMOOTH_CLOSE_PX, SMOOTH_CLOSE_PX)))
        bw = cv2.bitwise_and(bw, _smooth)
        if FILL_HOLES:                                 # รูข้างในก้อน (จุดสะท้อน/ลาย) ไม่ทำให้ก้อนแหว่งตอน open ใหญ่
            _c, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            bw = np.zeros_like(bw)
            cv2.drawContours(bw, _c, -1, 255, -1)

    if not crop_mode:                                  # ค้นทั้งภาพ: ตัดส่วนนอก ROI (เหมือนเดิม)
        _m = np.zeros_like(bw)
        _m[roi_rect[1]:roi_rect[3], roi_rect[0]:roi_rect[2]] = 255
        bw = cv2.bitwise_and(bw, _m)

    # --- 3) ล้าง noise: open ตัดจุดเล็ก, close อุดรูในตัวบล็อก ---
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k)
    # open ใหญ่: ตัดของ "บาง" (ขาไฟ IR, ราง, สาย) ที่ติดกับบล็อก — บล็อกเป็นสี่เหลี่ยมตันกว้าง >100 px จึงรอด
    # kernel วงรี: ผลไม่ขึ้นกับว่าบล็อกเอียงกี่องศา; ขนาดปรับตาม DET_SCALE
    kpx = max(3, int(round(OPEN_BIG_PX * sc)) | 1)
    kb = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kpx, kpx))
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kb)
    if SMOOTH_SD_MAX is not None:                      # คืนขอบที่ถูกกินไปตอนตัดส่วนไม่เรียบ (ภายในก้อนเดิมเท่านั้น) -> ขนาด mm ไม่หด
        _kg = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * SMOOTH_WIN_PX + 1, 2 * SMOOTH_WIN_PX + 1))
        bw = cv2.bitwise_and(cv2.dilate(bw, _kg), bw_raw if crop_mode else cv2.bitwise_and(bw_raw, _m))

    # --- 4) วนทุก contour หาอันที่ "ขนาดจริง" ใกล้ 40x30 ที่สุด ---
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if SPLIT_TOUCHING:
        cnts = _split_touching(cnts, small, int(round(C.MIN_CONTOUR_AREA_PX * sc * sc)))
    best, best_err = None, 1e9
    pf = np.array(prefer_px if prefer_px is not None else (gray.shape[1] / 2.0, gray.shape[0] / 2.0), np.float32)
    LAST_REJECTS.clear()
    for c in cnts:
        c = c.astype(np.float32) / sc + np.array([x0, y0], np.float32)       # กลับเป็นพิกัดภาพเต็ม (ย่อ/ครอปมาแล้ว)
        if cv2.contourArea(c) < C.MIN_CONTOUR_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        if rect[1][0] * rect[1][1] < 1 or cv2.contourArea(c) / (rect[1][0] * rect[1][1]) < 0.7:
            continue                                                         # ไม่ใช่สี่เหลี่ยมเต็ม (ราง/jaw/เงา)
        # ตัดของที่มี "ขาวจัด" เยอะ (แสงไฟ LED ของเซ็นเซอร์ IR / ขาไฟ) — ผิวบล็อกสว่างเรียบ ไม่ถึงขั้นอิ่มตัว
        cm = np.zeros_like(small)                                            # ตรวจบนภาพเล็ก (ถูกกว่า)
        cv2.drawContours(cm, [((c - np.array([x0, y0], np.float32)) * sc).astype(np.int32)], -1, 255, -1)
        sat = float(np.count_nonzero((small >= SAT_LEVEL) & (cm > 0))) / max(1, np.count_nonzero(cm))
        if sat > SAT_FRAC_MAX:
            LAST_REJECTS.append((cv2.boxPoints(rect).astype(np.float32), 0.0, 0.0))
            continue
        # ผิวบนบล็อก "เรียบ" (ความสว่างแทบเท่ากันทั้งหน้า) ส่วนผนังซี่/ราง/สายมีลายหรือขอบ -> ส่วนเบี่ยงเบนมาตรฐานสูง
        # ตรวจเฉพาะด้านในที่ถอยจากขอบ (กันขอบเบลอ) — ไม่ขึ้นกับท่าแขน/ homography จึงใช้ได้แม้ mm เพี้ยน
        _cm = np.zeros_like(small)
        cv2.drawContours(_cm, [((c - np.array([x0, y0], np.float32)) * sc).astype(np.int32)], -1, 255, -1)
        _ke = max(3, int(round(UNIFORM_ERODE_PX * sc)) | 1)
        _in = cv2.erode(_cm, cv2.getStructuringElement(cv2.MORPH_RECT, (_ke, _ke)))
        _in_med = None
        if np.count_nonzero(_in) > 50:
            _sd = float(small[_in > 0].std())
            if _sd > UNIFORM_STD_MAX:
                LAST_REJECTS.append((cv2.boxPoints(rect).astype(np.float32), -_sd, -1.0))      # ค่าลบ = ถูกปัดเพราะมีลาย (std)
                continue
        if np.count_nonzero(_in) > 50:
            _ko = max(3, int(round(RING_PX * sc)) | 1)
            _ring = cv2.dilate(_cm, cv2.getStructuringElement(cv2.MORPH_RECT, (2 * _ko + 1, 2 * _ko + 1))) - cv2.dilate(_cm, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            if np.count_nonzero(_ring) > 50:
                _in_med, _ring_med = float(np.median(small[_in > 0])), float(np.median(small[_ring > 0]))
                _con = (_in_med - _ring_med) if C.BLOCK_IS_BRIGHT else (_ring_med - _in_med)
                _rv = small[_ring > 0]
                if SMOOTH_SD_MAX is not None:
                    # รอบบล็อก "ต่างจากผิวบล็อก" = มืดกว่า (สายพาน) หรือ มีลาย (แผ่นโลหะ) หรือ สว่างต่างชัด (ราง/jaw)
                    # ผนัง/ราง: ผิวเรียบสีเดียวกันต่อเนื่องออกไปรอบกรอบ -> สัดส่วนนี้ต่ำ -> ไม่ใช่บล็อก
                    _diff = (np.abs(_rv.astype(np.float32) - _in_med) >= RING_DIFF_GRAY) | (_lsd[_ring > 0] > SMOOTH_SD_MAX)
                    _dfrac = float(np.count_nonzero(_diff)) / _rv.size
                    _dk = float(np.count_nonzero(_rv < _in_med - EDGE_CONTRAST_MIN)) / _rv.size
                    if _dfrac < RING_DIFF_FRAC_MIN or (C.BLOCK_IS_BRIGHT and _dk < (PX_RING_DARK_MIN if px_mode else RING_DARK_FRAC_SOFT)):
                        LAST_REJECTS.append((cv2.boxPoints(rect).astype(np.float32), float(_dfrac), -3.0))
                        continue
                else:
                    _dark = float(np.count_nonzero(_rv < _in_med - EDGE_CONTRAST_MIN)) / np.count_nonzero(_ring)
                    if C.BLOCK_IS_BRIGHT and _dark < RING_DARK_FRAC_MIN:      # รอบข้างส่วนใหญ่สว่าง = ผนัง/ราง ไม่ใช่บล็อกบนสายพานมืด
                        LAST_REJECTS.append((cv2.boxPoints(rect).astype(np.float32), float(_dark), -3.0))
                        continue
                    if _con < EDGE_CONTRAST_MIN:                              # ไม่เด่นจากรอบข้าง = ผนัง/พื้นผิวทั่วไป ไม่ใช่บล็อกบนสายพาน
                        LAST_REJECTS.append((cv2.boxPoints(rect).astype(np.float32), float(_con), -2.0))
                        continue
        box = cv2.boxPoints(rect).astype(np.float32)                         # มุม 4 จุด (pixel)
        if (box[:, 0].min() < BORDER_PX or box[:, 1].min() < BORDER_PX or
                box[:, 0].max() > gray.shape[1] - 1 - BORDER_PX or box[:, 1].max() > gray.shape[0] - 1 - BORDER_PX):
            LAST_REJECTS.append((box, 0.0, -4.0))                            # แตะขอบภาพ: ถูกตัด ใช้คำนวณตำแหน่ง/ขนาดไม่ได้
            continue
        if px_mode and (box[:, 0].min() < PX_BORDER_PX or box[:, 1].min() < PX_BORDER_PX or
                        box[:, 0].max() > gray.shape[1] - PX_BORDER_PX or box[:, 1].max() > gray.shape[0] - PX_BORDER_PX):
            LAST_REJECTS.append((box, 0.0, -4.0))                            # (px_mode) ชิดขอบภาพ = มักเป็นของที่ถูกตัด (ถัง/ผนัง) ไม่ใช่บล็อก
            continue
        if px_mode:
            _s0, _s1 = float(np.linalg.norm(box[1] - box[0])), float(np.linalg.norm(box[2] - box[1]))
            _L, _S = max(_s0, _s1), max(1e-6, min(_s0, _s1))
            if math.sqrt(_L * _S) < PX_MIN_EQ or not (PX_ASPECT[0] <= _L / _S <= PX_ASPECT[1]):
                LAST_REJECTS.append((box, _L, _S))
                continue
        mm = cv2.perspectiveTransform(box[None], H)[0]                       # -> mm
        e0 = float(np.linalg.norm(mm[1] - mm[0]))                            # ความยาวด้านที่ 1 (mm)
        e1 = float(np.linalg.norm(mm[2] - mm[1]))                            # ความยาวด้านที่ 2 (mm)
        # ระบุด้านยาว/สั้น และเวกเตอร์ของด้านยาว
        if e0 >= e1:
            long_e, short_e, long_vec = e0, e1, mm[1] - mm[0]
        else:
            long_e, short_e, long_vec = e1, e0, mm[2] - mm[1]
        err = abs(long_e - C.BLOCK_LONG_MM) + abs(short_e - C.BLOCK_SHORT_MM)
        tol = C.BLOCK_SIZE_TOL_MM * tol_scale                                # TRACK: ผ่อนขนาด (hysteresis) กันกระพริบตอนค่าอยู่ริมเกณฑ์
        if not px_mode and (abs(long_e - C.BLOCK_LONG_MM) > tol or abs(short_e - C.BLOCK_SHORT_MM) > tol):
            LAST_REJECTS.append((box, long_e, short_e))
            continue                                                         # ขนาดไม่ใช่บล็อก
        cost = err + PREFER_W * float(np.linalg.norm(box.mean(axis=0) - pf))      # ขนาดตรงที่สุด + ใกล้จุดที่ต้องการ (หลายบล็อกในภาพไม่สลับไปมา)
        if px_mode:
            cost = -cv2.contourArea(box) + 50.0 * float(np.linalg.norm(box.mean(axis=0) - pf))   # ใหญ่สุด (+ ใกล้ตำแหน่งเดิมเล็กน้อย)
        if cost < best_err:
            ctr = mm.mean(axis=0)
            ang = math.degrees(math.atan2(long_vec[1], long_vec[0])) % 180.0
            best, best_err = {"x": float(ctr[0]), "y": float(ctr[1]), "long_deg": ang,
                              "score": err, "box_px": box, "gray": _in_med,
                              "tone": None if _in_med is None else ("white" if _in_med >= BLOCK_WHITE_GRAY else "orange?")}, cost

    if best is not None and sc != 1.0:
        best = _refine_full_res(gray, best, H, bg, diff_thr, thr=thr if thr is not None else LAST_THR[0])

    if debug:                                                                # วาดกรอบลงภาพให้ดูด้วยตา
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 128, 0), 1)
        if best is not None:
            cv2.polylines(vis, [best["box_px"].astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(vis, f"x={best['x']:.1f} y={best['y']:.1f} a={best['long_deg']:.0f}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        return best, vis, bw
    return best


def find_blob_px(gray):
    """หาบล็อก "ในหน่วย pixel" โดยยังไม่มี homography (ใช้ตอน calib_touch.py)
    เลือก contour ที่เป็นสี่เหลี่ยมเต็ม (พื้นที่/กรอบ > 0.8) สัดส่วนด้าน 1.0-1.9 และใหญ่สุดใน DETECT_ROI
    คืน (cx, cy, box 4 จุด) หรือ None"""
    x0, y0, x1, y1 = C.DETECT_ROI
    roi_mask = np.zeros_like(gray)
    roi_mask[y0:y1, x0:x1] = 255
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    mode = cv2.THRESH_BINARY if C.BLOCK_IS_BRIGHT else cv2.THRESH_BINARY_INV
    _, bw = cv2.threshold(blur, 0, 255, mode + cv2.THRESH_OTSU)
    bw = cv2.bitwise_and(bw, roi_mask)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bw = cv2.morphologyEx(cv2.morphologyEx(bw, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < C.MIN_CONTOUR_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        w, h = rect[1]
        if min(w, h) < 1:
            continue
        if area / (w * h) < 0.8 or not (1.0 <= max(w, h) / min(w, h) <= 1.9):
            continue                                       # ไม่ใช่สี่เหลี่ยมเต็ม / สัดส่วนไม่ใช่ 40x30
        if area > best_area:
            best, best_area = (float(rect[0][0]), float(rect[0][1]), cv2.boxPoints(rect)), area
    return best


class BlockTracker:
    """ค้นหาทั้งภาพก่อน (SEARCH) -> เจอบล็อกแล้วหดกรอบตามบล็อก (TRACK) -> หายแล้วกลับไปค้นหาทั้งภาพ

    SEARCH: กรอบ = ทั้งภาพ  ใช้ภาพพื้นหลังว่าง (set_background) ตัดของนิ่ง (ราง/jaw/ไฟ IR) ถ้าไม่มี ใช้ Otsu ทั้งภาพ
            ต้องเจอ "ต่อเนื่อง" confirm เฟรม ในตำแหน่งใกล้กัน จึงถือว่าบล็อกมาแล้ว (กันไฟกะพริบ/ของผ่านไปมา)
    TRACK : กรอบ = กรอบรอบบล็อก + margin_px (+ ชดเชยตามการเคลื่อนที่ของเฟรมก่อน) -> เร็วขึ้น ไม่หลงไปจับอย่างอื่นนอกกรอบ
            หาไม่เจอเกิน lost_max เฟรมติด -> กลับ SEARCH
    stationary: ตำแหน่งบล็อก (mm) นิ่งภายใน still_mm ตลอด still_n เฟรม (ใช้ตัดสินว่าบล็อกมาถึงที่หยุดแล้ว)
    """

    def __init__(self, H, margin_px=60, confirm=3, lost_max=6, still_mm=0.6, still_n=10, jump_mm=25.0,
                 lost_s=0.4, confirm_s=0.08, track_tol_scale=1.6):
        import collections
        self.H, self.margin, self.confirm, self.lost_max = H, margin_px, confirm, lost_max
        self.still_mm, self.still_n, self.jump_mm = still_mm, still_n, jump_mm
        # หมดเวลา/ยืนยันเป็น "วินาที" ไม่ใช่จำนวนเฟรม (กล้อง ~115 fps: 6 เฟรม = 50 ms สั้นเกิน -> กระพริบ TRACK/SEARCH)
        self.lost_s, self.confirm_s, self.track_tol_scale = lost_s, confirm_s, track_tol_scale
        self.t_ok = self.t_first = None
        self.thr = None                  # threshold ที่ล็อกจากตอน SEARCH (ต้องมีตั้งแต่แรก ไม่งั้นตอนใช้ background เข้า TRACK แล้ว crash)
        self.bg = None
        self.state = "SEARCH"
        self.roi = None
        self.hits = 0
        self.miss = 0
        self.last = None
        self.hist = collections.deque(maxlen=still_n)

    def set_background(self, frames):
        """frames = ภาพ gray หลายเฟรมตอน "ไม่มีบล็อก" ในเลน -> ค่าเฉลี่ย + blur เก็บเป็นพื้นหลัง"""
        avg = np.mean(np.stack(frames).astype(np.float32), axis=0).astype(np.uint8)
        self.bg = cv2.GaussianBlur(avg, (5, 5), 0)
        self.reset()

    def clear_background(self):
        self.bg = None
        self.reset()

    def reset(self):
        self.state, self.roi, self.hits, self.miss, self.last = "SEARCH", None, 0, 0, None
        self.t_ok = self.t_first = None
        self.thr = None
        self.hist.clear()

    def _roi_from(self, res, shape, prev):
        xs, ys = res["box_px"][:, 0], res["box_px"][:, 1]
        dx = dy = 0.0
        if prev is not None:                              # ชดเชยการเคลื่อนที่ (เฟรมต่อเฟรม) กันบล็อกวิ่งออกนอกกรอบ
            dx, dy = 2.0 * (xs.mean() - prev[0]), 2.0 * (ys.mean() - prev[1])
        m = self.margin
        h, w = shape[:2]
        x0 = int(max(0, min(xs.min(), xs.min() + dx) - m))
        x1 = int(min(w, max(xs.max(), xs.max() + dx) + m))
        y0 = int(max(0, min(ys.min(), ys.min() + dy) - m))
        y1 = int(min(h, max(ys.max(), ys.max() + dy) + m))
        return (x0, y0, x1, y1)

    def update(self, gray, now=None):
        """ป้อนภาพ 1 เฟรม -> คืน (res หรือ None, state, roi ที่ใช้ค้นรอบนี้)"""
        now = time.perf_counter() if now is None else now
        h, w = gray.shape[:2]
        roi = (0, 0, w, h) if self.state == "SEARCH" else self.roi
        res = find_block(gray, self.H, roi=roi, bg=self.bg,
                         tol_scale=self.track_tol_scale if self.state == "TRACK" else 1.0,
                         thr=self.thr if self.state == "TRACK" else None,
                         prefer_px=None if (self.last is None or self.state != "TRACK") else self.last["box_px"].mean(axis=0))
        if self.state == "SEARCH" and res is not None and LAST_THR[0] is not None:
            self.thr = LAST_THR[0]                        # จำ threshold ของเฟรมที่เจอบล็อก (ค้นทั้งภาพ -> ค่าเสถียร)
        if res is not None and self.last is not None and \
                math.hypot(res["x"] - self.last["x"], res["y"] - self.last["y"]) > self.jump_mm:
            if self.state == "TRACK":
                res = None                                    # กระโดดผิดปกติ = ไม่ใช่ตัวเดิม
            else:
                self.hits, self.t_first = 0, None
        if res is None:
            if self.state == "TRACK":
                if now - self.t_ok > self.lost_s:             # หายนานเกิน lost_s วินาทีจึงกลับ SEARCH (หายเฟรมสองเฟรมไม่นับ)
                    self.reset()
            else:
                self.hits, self.t_first = 0, None
            return None, self.state, roi
        self.t_ok = now
        if self.state == "SEARCH":
            if self.hits == 0:
                self.t_first = now
            self.hits += 1
            if self.hits >= self.confirm and now - self.t_first >= self.confirm_s:
                self.state = "TRACK"
        prev = None if self.last is None else (self.last["box_px"][:, 0].mean(), self.last["box_px"][:, 1].mean())
        self.last = res
        self.hist.append((res["x"], res["y"]))
        if self.state == "TRACK":
            self.roi = self._roi_from(res, gray.shape, prev)
        return res, self.state, roi

    @property
    def stationary(self):
        if self.state != "TRACK" or len(self.hist) < self.still_n:
            return False
        a = np.array(self.hist)
        return bool(np.ptp(a[:, 0]) <= self.still_mm and np.ptp(a[:, 1]) <= self.still_mm)


def detect_stable(cap, H, n=12, max_std_mm=2.0):
    """อ่านหลายเฟรมแล้วใช้ค่ากลาง เพื่อกันค่าสั่น
    คืน dict หรือ None ถ้าเจอน้อยกว่า 70% ของเฟรม หรือค่ากระจายเกิน max_std_mm"""
    import camera
    hits = []
    for _ in range(n):
        g = camera.grab(cap)
        if g is None:
            continue
        r = find_block(g, H)
        if r:
            hits.append(r)
    if len(hits) < 0.7 * n:
        return None
    xs = np.array([h["x"] for h in hits])
    ys = np.array([h["y"] for h in hits])
    if xs.std() > max_std_mm or ys.std() > max_std_mm:
        return None
    # มุมแกนยาววนทุก 180° จึงต้องเฉลี่ยแบบวงกลมด้วยมุมสองเท่า (กันปัญหา 179° กับ 1°)
    a2 = np.radians([2 * h["long_deg"] for h in hits])
    ang = (math.degrees(math.atan2(np.sin(a2).mean(), np.cos(a2).mean())) / 2.0) % 180.0
    return {"x": float(np.median(xs)), "y": float(np.median(ys)), "long_deg": ang,
            "std_mm": float(max(xs.std(), ys.std()))}


def D_REJ():
    return list(LAST_REJECTS)


if __name__ == "__main__":
    # ดูผลสด: ค้นหาทั้งภาพก่อน -> เจอบล็อกแล้วหดกรอบตาม -> หายแล้วกลับค้นหาทั้งภาพ
    # คีย์:  b = ถ่ายพื้นหลัง "ว่าง" (เอาบล็อก/มือออกจากเลน) ตัดราง/jaw/ไฟ IR ที่นิ่งอยู่ | n = เลิกใช้พื้นหลัง (กลับ Otsu ทั้งภาพ) | ESC = ออก
    import camera
    gui = "--nogui" not in sys.argv
    H = load_homography()
    cap = camera.open_camera()
    trk = BlockTracker(H)
    import os
    BG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "background.png")
    if os.path.exists(BG_FILE):                       # พื้นหลังที่ถ่ายไว้ครั้งก่อน (ท่า scan เดิม) — กด b เพื่อถ่ายใหม่
        _bg = cv2.imread(BG_FILE, cv2.IMREAD_GRAYSCALE)
        if _bg is not None:
            trk.bg = _bg
            print("โหลดพื้นหลังจาก background.png (กด b ถ่ายใหม่ถ้าแสง/ท่าเปลี่ยน, n = เลิกใช้)")
    last_state = None
    _last_show = 0.0
    show_rej = False                                   # กรอบแดงซ่อนไว้ก่อน (กด d เพื่อดู)
    try:
        while True:
            watchdog_kick()
            g = camera.grab(cap)
            if g is None:                                  # ยังไม่มีเฟรมใหม่: ห้าม continue เฉยๆ (หน้าต่างจะ Not Responding) ต้องปั๊มหน้าต่างต่อ
                if gui and cv2.waitKey(10) & 0xFF == 27:
                    break
                continue
            res, state, roi = trk.update(g)
            if state != last_state:
                print(f"== {state} ==" + (" (ค้นหาทั้งภาพ)" if state == "SEARCH" else " (หดกรอบตามบล็อก)"))
                last_state = state
            _now = time.perf_counter()
            if _now - _last_show < 1.0 / 30:                  # วาด/พิมพ์ที่ 30 Hz (กล้อง ~115 fps; พิมพ์ทุกเฟรมทำเครื่องช้าและภาพกระตุก)
                continue
            _last_show = _now
            if res:
                print(f"x={res['x']:.1f} mm  y={res['y']:.1f} mm  long_axis={res['long_deg']:.1f} deg" +
                      ("  [นิ่ง]" if trk.stationary else ""))
            if gui:
                vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
                col = (255, 255, 0) if state == "SEARCH" else (0, 140, 255)
                cv2.rectangle(vis, (roi[0], roi[1]), (roi[2] - 1, roi[3] - 1), col, 2)
                for rb, rl, rs in (D_REJ() if show_rej else []):
                    cv2.polylines(vis, [rb.astype(np.int32)], True, (0, 0, 255), 2)       # แดง = เป็นสี่เหลี่ยมแต่ขนาดไม่ตรง
                    cv2.putText(vis, (f"low contrast {rl:.0f}" if rs == -2 else f"texture sd={-rl:.0f}" if rl < 0 else f"{rl:.0f}x{rs:.0f}mm"), (int(rb[:, 0].min()), int(rb[:, 1].min()) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                if res:
                    cv2.polylines(vis, [res["box_px"].astype(np.int32)], True, (0, 255, 0), 2)
                    cv2.putText(vis, f"x={res['x']:.1f} y={res['y']:.1f} a={res['long_deg']:.0f}  gray={res.get('gray') and round(res['gray'])} {res.get('tone') or ''}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(vis, f"{state}  bg={'ON' if trk.bg is not None else 'off'}  {'STATIONARY' if trk.stationary else ''}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
                cv2.imshow("detect: SEARCH whole view -> TRACK (b=bg, n=no bg, ESC)", vis)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    break
                if k == ord("b"):
                    print("ถ่ายพื้นหลัง: เอาบล็อก/มือออกจากเลน ...")
                    fr = [f for f in (camera.grab(cap) for _ in range(20)) if f is not None]
                    trk.set_background(fr)
                    cv2.imwrite(BG_FILE, trk.bg)
                    print(f"บันทึกพื้นหลังจาก {len(fr)} เฟรม -> background.png")
                if k == ord("d"):
                    show_rej = not show_rej                    # เปิด/ปิดกรอบแดงของที่ถูกปัดตก
                if k == ord("n"):
                    trk.clear_background()
                    print("เลิกใช้พื้นหลัง (Otsu ทั้งภาพ)")
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        _tb = traceback.format_exc()
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "crash_log.txt"), "w", encoding="utf-8") as _f:
            _f.write(_tb)                                  # เก็บ traceback ไว้ในไฟล์ แม้หน้าต่างปิดไปแล้ว
        print(_tb)
        print("บันทึกข้อผิดพลาดที่ crash_log.txt แล้ว — ส่งไฟล์นี้ให้ดู")
    finally:
        watchdog_off()
        cap.release()
        cv2.destroyAllWindows()
