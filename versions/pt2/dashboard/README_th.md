# PRISM Dashboard — หน้าเว็บเดียวคุมทั้ง cell

เปิดที่ `http://127.0.0.1:8080` — ไม่ต้องมี internet (three.js กับโมเดล CAD อยู่ในโฟลเดอร์ `web/`)

## 1. มันคืออะไร (และไม่ใช่อะไร)

- **เป็น observer**: subscribe MQTT แล้วแสดงผล ถ้า dashboard ดับ หุ่นยังวิ่งต่อปกติ
- ปุ่มเดียวที่ "สั่งเครื่อง" ได้ = **Start / Stop / Pause / Apply settings** ในหน้า Control › Run ซึ่งแค่ส่งต่อไปที่หน้าเว็บของ `autoen_node.py` (port 8781) — กฎเดิมยังอยู่ครบ (ต้องติ๊ก "arm area is clear", autoen ตรวจ limit เองและปฏิเสธค่าที่เกิน)
- สิ่งเดียวที่ dashboard **เขียนไฟล์** = `arm_thresholds.json`, `belt_thresholds.json` (analyzer reload เองภายใน 5 s) และ `arm_model_map.json` (ค่า align โมเดล 3D)
- **ไม่ใช่ safety function** — E-stop จริงยังเป็นปุ่มแดงบนเครื่อง

## 2. หน้าที่มี

| เมนู | หน้า | เห็นอะไร |
|---|---|---|
| Control | **Run** | ปุ่ม Start/Stop/Pause, ขั้นตอนของ cycle (FEED→…→CLEAR), progress, ตาราง device สด, settings ทั้งหมด, finished sets, cycles ล่าสุด |
| Control | **Camera** | fps, จำนวน object ในภาพ, นับต่อ 10 s, window 3 นาที, ตำแหน่ง track |
| Control | **3D Twin** | โมเดล CAD ทั้ง cell ขยับตามค่าจริง (ข้อต่อ 6 ตัว, สายพาน, IR, phase, ถาด PASS/FAIL) |
| Logger | – | logger arm/camera ยังเขียนอยู่ไหม (heartbeat), เปิดดูตารางใน `.db` และ CSV ของ autoen (read-only) |
| Health | – | ทุก node: OK / STALE / OFFLINE / NOT AVAILABLE พร้อมเหตุผล, health_monitor, ทุก topic + rate, event log |
| Analysis | **Arm** | state ปัจจุบัน, timeline, กระแส 6 ข้อต่อ + เส้น warn/alarm ของ state นั้น, อุณหภูมิ, cycles, conditions, **ตัวแก้ threshold** |
| Analysis | **Conveyor** | เหมือนกัน แต่เป็น speed/current/temp ของสายพาน และ runs |

แถบ Events ด้านล่างทุกหน้า: คลิกเพื่อขยาย

## 3. ติดตั้งและรัน

```bash
pip install paho-mqtt          # dashboard ใช้แค่นี้ (stdlib อย่างอื่น)
# วางไฟล์ในโฟลเดอร์เดียวกับ autoen_node.py / arm_analyzer_node.py / belt_analyzer_node.py
python3 dashboard_node.py --data-dir <โฟลเดอร์ที่มี .db และ *_thresholds.json>
```

ตัวเลือกที่ใช้บ่อย:

```bash
--mqtt-host <broker>           # ปกติ localhost (หรือ env MQTT_HOST)
--autoen http://127.0.0.1:8781 # หน้าเว็บของ autoen_node
--log-dir riff_logs            # --log-dir ของ autoen_node (default: <data-dir>/riff_logs)
--code-dir <โฟลเดอร์>          # ถ้า analyzer .py อยู่คนละที่
--read-only                    # โหมดดูอย่างเดียว ไม่มี Start/Stop/บันทึก threshold
--port 8080
```

**เปิดจากเครื่องอื่น**: เปิดทาง SSH tunnel (หน้านี้ไม่มี login):

```bash
ssh -L 8080:127.0.0.1:8080 pi@<ip ของ Pi>     # แล้วเปิด http://127.0.0.1:8080 บนเครื่องตัวเอง
```

`--host 0.0.0.0` ทำได้ แต่ทุกคนใน network จะกด Start ได้ → ใช้กับ `--read-only` เท่านั้น

ภาระเครื่อง: Python ~34 MB RAM, ~1% CPU ที่ 10 Hz. **โมเดล 3D วาดในเบราว์เซอร์ของคนดู ไม่ใช่บน Pi** — เปิดหน้า 3D บนโน้ตบุ๊กผ่าน tunnel ดีกว่าเปิดบน Pi

## 4. ลองโดยไม่มีหุ่น (simulator)

```bash
mosquitto                                   # หรือ broker ที่ใช้อยู่
python3 sim_cell.py --fast 2                # autoen_node จริง + Mega ปลอม (sim_mega.py), page :8781
python3 fake_camera.py                      # camera/* ปลอม
python3 arm_analyzer_node.py & python3 belt_analyzer_node.py & python3 health_monitor.py &
python3 logger_node.py --topics arm --db arm.db &  python3 logger_node.py --topics camera --db camera.db &
python3 dashboard_node.py
```

แล้วเข้า Control › Run → ติ๊ก clear → Start run → ไปดู 3D Twin
(ตัวเลขกระแส/เวลาใน sim เป็นค่าแต่ง: analyzer จะขึ้น WARN/ALARM และสายพานเป็น "reverse" เพราะ sim วิ่งทิศบวก — ไม่ใช่บั๊ก)

## 5. 3D Twin — อะไรจริง อะไรประมาณ

**จริง (วัดจริง):**
- มุมข้อต่อ J1–J5 และขากรรไกร J6 ← `arm/telemetry` (ticks)
- ความเร็วสายพาน → texture เลื่อน, roller หมุน, บล็อกเคลื่อน ← `belt/telemetry`
- IR เห็นบล็อก → ดึงบล็อกในโมเดลไปที่จุดคีบ (re-sync) ← `arm/ir`
- phase / drop / cycle / counter ถาด ← `arm/state`, `autoen/state`

**ประมาณ (ไม่มีเซนเซอร์):**
- ใบพุช M06: เคลื่อนตามเวลา 2.4 s ใน phase FEED (ไม่มีตำแหน่งจริงส่งมา)
- บล็อกบนสายพาน: เดินตามความเร็วสายพาน (drift ได้ → IR re-sync)
- ชนิดบล็อก: drop `left` = FAIL tray, `down` = PASS tray (ตามชื่อ pose ของ riff-real)
- ไฟ/FOV กล้องกระพริบ 2 s แรกของ TRANSPORT

**การแปลง ticks → มุมโมเดล** (ค่าเริ่มต้นหาจาก 4 named pose):
`มุม = sign × (ticks − zero) × 2π/4096`, ขากรรไกร `g = g_close + (ticks − zero_jaw) × 2π/4096`

- J1–J4: เครื่องหมาย `+` ตรงกับ pose จริง (bin ซ้าย/ขวา J1 ได้ −1.46/+1.67 rad เทียบโมเดล −1.49/+1.67)
- **J5 เครื่องหมาย (−) และค่า zero ทุกตัวเป็นการเดา** — ตรวจด้วยมือก่อนเชื่อ

**วิธี Align (ทำครั้งเดียว ~1 นาที):**
1. ย้ายหุ่นจริงไปท่าที่โมเดลรู้จัก (เช่น ท่า standby บนบล็อก = `pick_down`)
2. 3D Twin › **Align** › เลือกท่าเดียวกัน › **Set joints from live arm**
3. หนีบบล็อกอยู่ แล้วกด **Jaw = closed on block**
4. ดูว่าหุ่นในโมเดลกับหุ่นจริงเหมือนกันทุกท่า (home, left bin, right bin) ถ้าข้อไหนกลับทิศ → เปลี่ยน sign → **Save** (เก็บใน `arm_model_map.json`)

## 6. Threshold editor

- ช่องที่ต่างจากค่า default ของ analyzer มีขอบสีส้ม
- ตรวจด้วยกฎเดียวกับ `--calibrate` (typo ใน state/joint, warn ≥ alarm → ปฏิเสธพร้อมเหตุผล)
- เขียนแบบ atomic แล้ว analyzer โหลดเองใน ≤ 5 s ไม่ต้อง restart
- ที่ความเร็วสูงขึ้น (arm_speed ถึง 8.8) กระแสเปลี่ยน → ต้องปรับ threshold ใหม่ (`arm_analyzer_node.py --calibrate` ยังใช้ได้)

## 7. ข้อจำกัดที่ต้องรู้

- **ยังไม่ได้ทดสอบกับหุ่นจริง** — ทดสอบกับ autoen_node จริง + Mega จำลอง + broker จริงเท่านั้น (ตรวจ API, security, หน้าทุกหน้า, 3D ขยับตามข้อมูล)
- ถ้า `orchestrator.py` (delta-loop) เป็นเจ้าของ serial port แทน autoen_node → หน้า Run ใช้ไม่ได้จนกว่าจะเพิ่ม bridge; หน้าอื่นทำงานจาก MQTT ตามปกติ
- `forecast/*`, `orchestrator/*` ยังไม่มี → หน้า Health แสดง NOT AVAILABLE (ไม่ใช่ error)
- health_monitor เดิมยังไม่ดู analyzer ทั้งสองตัว — dashboard ดูเอง (หน้า Health ซ้ายบน)
- 3D ต้องการ WebGL + `DecompressionStream` (Chrome / Edge / Firefox / Safari ใหม่)
- ทิศการวิ่งของสายพานในโมเดลสมมติเป็น +X เสมอ (ใช้ค่าสัมบูรณ์ของ velocity)

## 8. ไฟล์

```
dashboard_node.py     backend: MQTT -> store -> SSE, relay ปุ่ม, อ่าน DB, threshold, arm map
sim_cell.py           autoen_node จริง + Mega ปลอม        sim_mega.py   Mega ปลอม
fake_camera.py        camera/* ปลอม
web/index.html app.css app.js charts.js p_control.js p_logger.js p_health.js p_analysis.js
web/cell3d.js         3D twin (ส่วนฉาก/CAD/kinematics พอร์ตจาก artifact + engine ที่ขับด้วยค่าจริง)
web/cad_data.js       โมเดล CAD 134 ชิ้น (2.9 MB)         web/vendor/  three.js r128 + OrbitControls
```
