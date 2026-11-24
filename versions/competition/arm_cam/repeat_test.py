"""repeat_test.py — วัดความแม่นยำ/ความทำซ้ำได้ "จริง" ของระบบ: หยิบแล้ววางคืนซ้ำ N รอบ บันทึกทุกรอบลง CSV

แต่ละรอบ: ท่า scan -> ตรวจจับ -> หยิบ -> ยก -> วางคืนที่เดิม
ดูอะไร:  * อัตราหยิบสำเร็จ
          * ตำแหน่งบล็อกที่กล้องเห็นเลื่อนไปรอบละเท่าไร (ถ้าหยิบ-วางตรงจริง บล็อกควรแทบไม่ขยับ)
          * tick ที่ gripper หยุดตอนหนีบ -> ใช้ตั้ง GRIP_TICK_RANGE (ตรวจว่าหนีบได้ "บล็อก" จริง)
ใช้:  python repeat_test.py [จำนวนรอบ=10]         ผล: repeat_log.csv
"""
import csv
import statistics
import sys
import time

import config as C
from picker import Picker

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10
if input(f"จะหยิบ-วางซ้ำ {N} รอบโดยไม่ถามอีก — พื้นที่โล่ง มือพ้นแล้ว? พิมพ์ y: ").strip().lower() != "y":
    raise SystemExit("ยกเลิก")

p = Picker()
rows = []
try:
    for i in range(1, N + 1):
        t0 = time.time()
        r = p.pick_visible()
        b = r["block"] or {}
        rd = lambda v: None if v is None else round(float(v), 2)       # ปัดทศนิยมให้อ่านง่ายใน CSV
        row = {"round": i, "ok": int(r["ok"]), "x": rd(b.get("x")), "y": rd(b.get("y")), "long_deg": rd(b.get("long_deg")),
               "stall": r["stall"], "side": (r.get("plan") or {}).get("side"), "reason": r["reason"], "sec": None}
        if r["ok"]:
            time.sleep(0.5)
            p.put_back(r["plan"])                          # วางคืนที่เดิม แล้วรอบหน้าดูว่ามันเลื่อนไหม
        row["sec"] = round(time.time() - t0, 1)
        rows.append(row)
        print(f"รอบ {i}: {'OK ' if r['ok'] else 'พลาด'} x={row['x']} y={row['y']} stall={r['stall']} {r['reason']}")
        if r["block"] is None:
            print("ไม่เห็นบล็อกแล้ว — หยุดทดสอบ")
            break
except KeyboardInterrupt:
    print("\nหยุดโดยผู้ใช้")
finally:
    p.close()

if not rows:
    raise SystemExit("ไม่มีข้อมูล")
with open("repeat_log.csv", "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

# ---------- สรุป ----------
ok = [r for r in rows if r["ok"]]
print(f"\nสำเร็จ {len(ok)}/{len(rows)} รอบ  (บันทึก repeat_log.csv)")
seen = [r for r in rows if r["x"] is not None]
if len(seen) >= 2:
    steps = [((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2) ** 0.5 for a, b in zip(seen, seen[1:])]
    drift = ((seen[-1]["x"] - seen[0]["x"]) ** 2 + (seen[-1]["y"] - seen[0]["y"]) ** 2) ** 0.5
    print(f"บล็อกเลื่อนต่อรอบ: เฉลี่ย {statistics.mean(steps):.1f} mm สูงสุด {max(steps):.1f} mm | เลื่อนสะสม {drift:.1f} mm")
    print("  (เลื่อนต่อรอบ ~ GRASP_CLEARANCE_MM เป็นเรื่องปกติ: jaw ที่ขยับดันบล็อกมาชิด jaw ตายตัวทุกครั้ง)")
stalls = [r["stall"] for r in ok if r["stall"] is not None and r["stall"] >= 0]
sides = {r["side"] for r in ok}                            # หนีบด้านสั้น 30 mm ("short") หรือด้านยาว 40 mm ("long" = อยู่ในราง)
if len(stalls) >= 5 and len(sides) > 1:
    print("รอบที่สำเร็จมีทั้งหนีบด้านสั้นและด้านยาวปนกัน — ไม่เสนอช่วง tick (ทดสอบทีละแบบ)")
elif len(stalls) >= 5:
    key = "GRIP_TICK_RANGE_LONG" if sides == {"long"} else "GRIP_TICK_RANGE"
    med = statistics.median(stalls)
    spread = max(stalls) - min(stalls)
    rng = [int(med - max(60, spread)), int(med + max(60, spread))]
    print(f"gripper หยุดที่ tick: กลาง {med:.0f} ช่วง {min(stalls)}-{max(stalls)}  -> เสนอ {key} = {rng}")
    if input(f"บันทึก {key} นี้ลง calib.json? พิมพ์ y: ").strip().lower() == "y":
        C.save_calib(**{key: rng})
        print("บันทึกแล้ว — ต่อไปถ้าหนีบแล้วหยุดนอกช่วงนี้ จะถือว่าหยิบพลาด")
