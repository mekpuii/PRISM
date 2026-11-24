# ส่งต่อ: SO101 หยิบบล็อกด้วยกล้อง OV9281 บนข้อมือ (สำหรับรันบน Raspberry Pi)

สถานะ ณ 2026-11-11: โค้ดครบและผ่านการทดสอบแบบจำลอง (FK/IK เทียบ URDF, ลำดับหยิบกับ Mega จำลอง, ตรวจจับบนภาพสังเคราะห์)
**ยังไม่ได้ยืนยันกับแขนจริง** — ขั้นสอบเทียบใน STEPS.md ต้องทำบนเครื่องที่มีจอก่อน

## ระบบทำอะไร
แขนไป "ท่า scan" -> OV9281 (บนข้อมือ) เห็นบล็อก 40x30x20 mm -> แปลง pixel เป็น mm -> IK -> ลงหยิบแนวดิ่ง -> ยก -> วาง
สายสัญญาณเหมือน PRISM ทุกอย่าง: Pi --USB--> Mega 2560 --Serial2--> URT-2 --> STS3215 x6  (ไม่ต้องแก้เฟิร์มแวร์ ไม่ต้องคาลิเบรต LeRobot ใหม่)

## ไฟล์ที่ต้องคัดลอกไป Pi
ทั้งโฟลเดอร์ รวมไฟล์ผลสอบเทียบ (สร้างบน PC):
| ไฟล์ | มาจาก | ต้องทำใหม่เมื่อ |
|---|---|---|
| `calib.json` | calib_zero / teach_scan / set_roi / calib_touch / teach_place / repeat_test | ดูแต่ละคีย์ด้านล่าง |
| `homography.npy` | calib_touch.py | กล้อง, ฐานแขน หรือท่า scan ขยับ |
| `intrinsics.npz` (ถ้ามี) | calib_intrinsics.py | เปลี่ยนเลนส์/โฟกัส/ความละเอียด |

คีย์ใน `calib.json`: `JOINT_ZERO`,`JOINT_DIR` (ถอดประกอบ horn/มอเตอร์ หรือคาลิเบรต LeRobot ใหม่) · `SCAN_POSE` · `DETECT_ROI`,`BLOCK_IS_BRIGHT` (แสง/พื้นหลังเปลี่ยน) ·
`TABLE_PLANE` (มากับ homography) · `PLACE_POSE` · `GRIP_TICK_RANGE` (เปลี่ยน jaw/บล็อก)

## ติดตั้งบน Pi
```
pip install numpy pyserial opencv-python-headless     # ไม่มีจอ: ใช้ headless
python doctor.py                                      # ตรวจทุกอย่าง (อ่านอย่างเดียว)
```
แก้ใน `config.py`: `CAM_INDEX` (ดู `v4l2-ctl --list-devices`) — `ARM_PORT` เป็น `/dev/ttyACM0` อัตโนมัติบน Linux
สคริปต์ที่ต้องมีจอ (ทำบน PC): teach_scan, set_roi, calib_touch, calib_intrinsics, calib_homography
สคริปต์ที่รัน headless ได้: doctor, check_arm, wake_from_rest, pick_block (`--yes --save-debug`), repeat_test, detect_block `--nogui`

## รัน
```
python pick_block.py --loop --yes --place     # วน: เห็นบล็อก -> หยิบ -> วางที่ท่าวาง
```

## ต่อเข้ากับ PRISM (พอร์ต Mega มีเจ้าของได้คนเดียว!)
`mega_link.py` ในโฟลเดอร์นี้คือไฟล์เดียวกับของ PRISM (ไม่ได้แก้) มี 2 ทาง:
1. **แยกโปรแกรม** (ง่ายสุด): หยุด autoen/arm_node ก่อน แล้วรัน pick_block.py — ใช้คนละเวลา
2. **ใช้ MegaLink ร่วมกัน** (รันในโปรเซสเดียวกับ PRISM):
```python
from picker import Picker
p = Picker(link=link)        # link = MegaLink ที่ PRISM เปิดอยู่; Picker จะไม่เปิด/ปิดพอร์ต และไม่ส่ง STOP สายพาน
r = p.pick_visible()         # {"ok", "reason", "block": {x, y, long_deg}, "stall"}  — ไม่ raise สำหรับเหตุที่รู้
if r["ok"]:
    p.place()                # หรือสั่งท่า bin ของ PRISM เองต่อจากนี้ (แขนถือบล็อกอยู่เหนือจุดหยิบ)
```
ข้อควรระวังเมื่อใช้ร่วม: ห้ามมีสอง thread สั่งแขนพร้อมกัน (Picker บล็อกจนเสร็จ) และ PRISM ต้องไม่ส่ง P/H ให้ข้อ 1-6 ระหว่าง Picker ทำงาน

## โปรโตคอล Mega ที่โค้ดนี้ใช้ (ไม่มีคำสั่งใหม่)
`P,id,tick,speed` (ส่งซ้ำทุก 0.25 s — เฟิร์มแวร์แช่แข็งเป้าถ้าเงียบเกิน 500 ms) · `H,id` · `Q,id,0|1` · `C,6` -> `G,CLOSED,pos`/`G,ERROR`
ขอบเขต SAFE_MIN/MAX ใน `config.MEGA_SAFE` ต้องตรงกับเฟิร์มแวร์เสมอ (เฟิร์มแวร์บีบค่าเงียบๆ ฝั่ง Python จึงปฏิเสธเองก่อน)

## สิ่งที่รู้ว่าเป็นข้อจำกัด
- หยิบแนวดิ่งได้เฉพาะโซนห่างแกนฐาน ~100-230 mm, ซ้าย-ขวา ~±140 mm (นอกโซน: ปฏิเสธก่อนขยับ)
- บล็อกต้อง "หยุดนิ่ง" ตอนถ่ายภาพ (ไม่ได้ออกแบบให้ไล่หยิบบนสายพานที่วิ่ง)
- กล้องขาวดำ: แยกบล็อกจากพื้นด้วยความสว่างเท่านั้น (พื้นหลังต้องตัดกับบล็อก) — แยกสีไม่ได้
- ถ้าแขนอยู่ท่า rest ของ PRISM (elbow เกิน SAFE) ต้อง `wake_from_rest.py` ก่อน และโค้ดนี้ไม่พาแขนกลับท่า rest ให้ (ใช้ rest ของ PRISM)
- เรขาคณิตแขนมาจาก CAD (URDF so101_new_calib) — jaw ที่พิมพ์เอง/ติดแผ่น TPU ต้องใส่ `JAW_PAD_MM`
