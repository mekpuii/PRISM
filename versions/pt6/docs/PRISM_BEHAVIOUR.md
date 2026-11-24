# How PRISM behaves now (pt5, 2026-11-12)

This page describes what the cell does today, step by step. Short version: **the arm picks one cube after another;
the optimizer decides every push of the feeder, checks that each push really delivered a cube, and handles most
problems by itself. It stops and calls you only when something needs your hands.**

---

## 1. The parts and who decides what

| Part | Program | Its job |
|---|---|---|
| Camera (Astro HD) | `vision_usb.py` | sees the cubes on the belt (colour, position, moving or not) |
| Mega + arm + belt + feeder | `auto_en_new.py` (autoen) | the **body**: moves the arm, runs the belt, pushes the feeder - only when told |
| Optimizer | `orchestrator/main.py` | the **brain**: decides when to push, how many cubes ride, which speeds, what to do when something goes wrong |
| Dashboard | `dashboard_node.py` | shows everything: http://127.0.0.1:8080 |
| Helpers | health, loggers, analyzers | record and check, never move anything |

`./start_cell.sh` starts all of them in the right order, checks that everything is connected (READY), and asks you
to press Enter. **Nothing moves before you press Enter.**

---

## 2. One pick (the arm)

The arm repeats this, without any pause between picks:

1. **Wait at Standby** (fingers open above the pick spot) until a cube is at the pick IR.
2. **Grip** - the belt is held still.
3. **Lift** to Home - the belt is still held (the cube is lifted off the belt).
4. **Carry** to the bin and **release** - now the belt may move again.
5. **Back Home**, then **down to Standby** - the belt is held while the fingers come down.

About 9.6 s of arm work per pick. That is the speed limit of the cell: a pick can not be faster than the arm.

## 3. The belt

- It runs **only for cubes that are confirmed** to be on it (see 4), and only when the arm allows (not while it grips,
  lifts, or comes down to Standby).
- It stops by itself when a cube reaches the pick IR.
- While the arm is not yet back at Standby, the first cube is **held a little before the IR**, so it arrives only
  after the arm is ready (a cube waiting under the descending fingers would be hit).

## 4. The feeder (the most important part)

### When it pushes
The optimizer pushes when:
- fewer cubes are on the way than needed (see 5), **and**
- the last cube has left the feeder exit, **and**
- the **12 V rail is healthy** (11.5 V or more) and the arm is in a phase that does not pull the rail down -
  it learned that **GRIP** does (the rail fell to ~2 V there on 11-12), so it never pushes while the arm grips,
- and at least 2 s after the last push.

### What one push looks like
The blade waits **fully back for 1 s** (the stack settles), then **jumps forward at full speed to the end** and
**presses for 800 ms**, then goes back. About 2.1 s.

### Every push is checked (your flow chart)
| servo current | camera: a new cube at the left edge | what the optimizer does |
|---|---|---|
| any | **yes** | **fed** - the belt now moves it |
| moved | no | **nudge**: belt 1 s, look again. Still nothing -> **empty**: push again |
| **did not move** | no | **empty**: push again at once. **3 in a row -> ALARM** |
| no camera | - | servo moved = fed (a late cube is noticed later) |

### A cube stuck at the left edge
If a confirmed cube does not ride away when the belt moves, the push was too short: it **pushes again, longer**
(800 -> 1050 -> 1300 -> 1500 ms) and **remembers** the length that works. It never goes below 800 ms.

### No pile-ups (pt5, 2026-11-12)
The 11-12 18:17 run piled cubes up. Cubes left on the belt were credited to new pushes, so the measured distance
feeder -> IR fell from 468 to 230 mm. Pushes the camera did not see were called "empty" and pushed again, onto a cube
that had come out. Now:
- **Room first:** no push before the belt has moved **one cube + one gap** since the blade last moved (encoder).
  While the arm grips / lifts and the belt is held, the next push waits too.
- **The camera can stop a push, never allow one alone:** a cube it sees in the exit zone blocks the push, and the belt
  is nudged to carry it away. Something that stays put while the belt moves 40 mm is a reflection and is ignored.
- **"Servo moved, camera saw nothing"** is EMPTY only after the camera has proven in this run that it sees the cubes
  come out (4+ arrivals, 80 % seen). Before that the push counts as delivered and the belt carries it; if nothing
  arrives, the ride tells.
- **Cubes nobody fed** (left on the belt, or missed by the camera) are picked as they come and do not count as rides.
  A ride is only measured if it is 0.8-1.3 x the real one (33000 counts).
- **Sorting:** a cube goes **left only when it is seen as orange**; white or not recognised -> **right**.
- Simulator (`test/test_nostack_sim.py`, the real belt, 2 cubes left on it, camera blind to every second cube):
  12 of 12 picked, every push >= 85 mm behind the last cube, ride 8.0-8.3 s, ~4 s per cube.

## 5. How many cubes ride the belt (pt3: fill the belt)

The optimizer **keeps the belt as full as is safe**. The cubes wait in a line behind the one at the pick IR, so after
each pick the next cube only travels **one gap (~1 s)**, not the whole belt (~9 s).

- **How many:** feeder exit -> IR distance / (cube 40 mm + gap), up to 6. Today: 470 mm / (40 + 60) mm = **4 cubes**.
- **When the next push may come:** once the last cube has moved **one cube + one gap** off the feeder - measured by the
  belt encoder (exact), or seen by the camera (new geometry: 0.64 mm per pixel).
- **The gap** is at least **60 mm** (room for the gripper fingers around the cube at the IR). The tuner may try 80 or
  100 mm and keeps whatever is measurably better.
- In the simulator (same fake hardware): **3.9 s per cube instead of 14.7 s** one at a time (73 % less).

## 6. Speeds

Belt 20 rpm, arm 1.5 (proven best). Now and then it tries **one** step faster for 12 picks; the first problem sends it
straight back, and that step is not tried again for a while. Settings change at once (from the next arm move),
not at the end of a pick.

**The tuner (pt3)** experiments with its own knobs, one at a time, 6 picks each, after 10 normal picks:
the **gap** (60 / 80 / 100 mm) and the **late limit** (1.25 / 1.35 / 1.6 x the ride). It keeps a value only if it
is **at least 3 % faster with no more failures**, otherwise goes back and does not try it again for 50 picks. It
never experiments during an alarm, IDLE or a probe, and it remembers its results (learned_state.json).

---

## 7. When something goes wrong - what it does by itself

| What happens | What the optimizer does | What you do |
|---|---|---|
| **Feeder empty** (3 pushes bring nothing, servo moves) | pauses at Home, belt braked; tries one push by itself now and then; **restarts by itself** after a refill | refill |
| **Feeder servo does not move** (3 pushes, no current, no cube) | **ALARM**: red banner, the **arm rolls its wrist 3 times**; tries again after 1, 2, 5, 10 min and 5 s after you touch the feeder | check the servo's **D12 plug and 5 V supply** |
| **Cube stuck at the feeder exit** | pushes again, longer | nothing (watch it learn) |
| **Cube late on the belt** (nothing came out) | gives that push up, pushes again | nothing |
| **12 V rail low** | waits; does not push | if it stays low: the supply |
| **Glare / dark** on the camera | stops using the camera (checks fall back to the servo current) | move the lamp, keep the belt area clear |
| **Someone presses Pause** | "hands off" - only watches, says why | Resume when ready |
| **Arm problem / belt fault** | pauses (RECOVER or ALERT) and tells the story | look at the dashboard |

Every stop comes with a **story** in the optimizer log and on the dashboard: *what I tried, what I saw, so what the
problem is, what I do next.*

---

### Feeder refilled after a rest (2026-11-12)
While PRISM rests because the feeder is empty (arm parked, belt braked), it makes a **light probe**: one push, nothing
else moves, and the camera looks at the feeder exit. Every 20 s for the first 10 minutes (then every minute), and within
~2 s when the feeder IR says "cubes". A new cube at the exit -> it runs again at once (that cube is carried off and
picked first). Simulator: picked 15 s after the refill (feeder IR working), 32 s (feeder IR dead); before: 1-10 min.

### Emergency grip with the hand camera (2026-11-12)
**When:** the normal grip missed (the gripper stopped outside the 178-186 deg "cube held" window) **and** the pick IR
still sees the cube.
**What:** no feed and no belt; the arm opens, goes home, then to the hand camera's scan pose. `arm_cam/rescue_eye.py`
(follow_block's own maths, no window) finds the cube and steers the arm until the cube sits still under the jaws
(0.6 s), then the **taught grip** (`arm_cam/follow_grip.json`, 51 mm down) grips it. Home with the cube, then the bin by
the main camera's list (**orange -> left, anything else -> right**). After the release: normal work again.
If the cube is not in view the base turns +-150 / +-300 ticks to look for it.
**Two tries.** Both missed -> home, the **gripper closes and opens 3 times** (the signal), the run **pauses** and the
dashboard cloud asks you to take the cube; Resume when done.
**No hand camera** (unplugged): the emergency steps aside - the old recovery (grip again next cycle), no pause.
**The grip itself is gentle and feels the servo (2026-11-12).** The gripper closes very slowly (about 10 deg/s) with its
target only a few ticks ahead, and watches its own **current and speed**. Contact = the current rose 20 mA above its
free-moving value (two readings in a row), or the servo stalled. Then it adds only 6 ticks of squeeze, looks at the
current (above 60 mA it backs off), and holds with `H,6` - nothing keeps pushing, so no "gr-gr-gr". It accepts **any
width** (a cube stops the fingers at 174 deg up to nearly open, depending on how it lies); closing right through to the
fingers meeting = nothing was there = the try fails. A normal grip held at 177.6 deg with 6-13 mA (peak 39 mA).
Not done yet: turning the wrist to the cube's angle (the angle is measured and logged in every picture, not used).
**Pictures, no video.** Saved with the date and time in `rescue_shots/` (file `YYYYMMDD_HHMMSS_run<N>_<what>.jpg`, the
same text written on the picture): the main camera when the emergency starts, the hand camera at the scan pose, **the
moment the strange cube is found**, centred and ready, gripped (with the angle and current), and when a try fails or the
cube is not found. Every step is a line in `rescue_shots/events.jsonl`. The dashboard's **Emergency grip block** (Overview)
shows how many emergency runs there were (rescued / failed / today) and the pictures of the latest ones. Nothing in
`rescue_shots/` is ever deleted by PRISM.
Settings (autoen): `--rescue 1` (on), `--rescue-tries 2`, `--rescue-follow-s 12`, `--rescue-grip-speed 120`,
`--rescue-contact-ma 20`, `--rescue-max-ma 60`, `--rescue-preload-ticks 6`, `--rescue-min-deg 174`.
Tests: `test/test_rescue_sim.py` (three cube positions, a stiff gripper, nothing between the fingers, no camera).
**The current numbers are a first guess from past runs; they need one real trial** (the sim cannot know how this servo's
current really rises at contact).
**Not yet tried on the real arm** - first a camera-only check, then one real emergency with you watching.

### Sleep: power off when no cubes come (2026-11-12)
**When:** resting for an empty feeder for **5 min** - or already after **2 min** when the forecast expects the next
refill only in 15 min or more (it learns when refills usually happen, per hour of the week).
**What:** pause, the arm to its rest pose, then the **relay cuts the cell's power** (GPIO 19 = header pin 35, GND pin 34;
`orchestrator/relay.py`). The Pi stays on, and the Mega stays on through USB, so the feeder IR and the camera still watch.
**Wake:** every **10 min**, or within ~3 s when the feeder IR sees cubes, or at once on Resume: power on, 3 s for the
servos, the arm wakes, one light probe push. A cube -> work again; nothing -> back to sleep.
The cloud says *Sleeping - no cubes for a while. Next check in ...*.
**Before the relay is used:** teach it once - `cd orchestrator && ~/orch-env/bin/python cal/relay_test.py` (cell stopped,
arm resting). Until then the relay is disabled and sleep only rests the arm with its motors off.
`start_cell.sh` switches the power ON first (PRISM may have been stopped while asleep).
Simulator: `test/test_sleep_sim.py` (sleep, timed wake + empty check, feeder-IR wake after a refill in 3 s).

### The Limits zone on the Overview (2026-11-12)
Under the PRISM cloud: one small box per limit - each arm servo J1-J6, belt motor, Raspberry Pi, servo current, the 12 V
rail, the feeder's pushes a minute, and the "no cubes for ... sleep after 5 min" timer - with what it reads now and where
PRISM rests. A box turns amber when warm and red at the rest limit. The line on top says why the cell rests right now, for
example *Resting: J3 elbow 66 C (limit 65)*. The numbers come from `orchestrator/limits.py`, the same file the optimizer
acts on, so the page cannot show a limit the machine does not use. It shows the limits even when the cell is stopped.

### Heat: thresholds and rest (2026-11-12)
| Part | Sensor | Limit | What PRISM does |
|---|---|---|---|
| Arm servos J1-J6 | temperature in each servo | warm 55 C, hot 65 C | warm: gentler settings; hot: **COOL** - pause at Home until 50 C |
| Belt motor | DS18B20 | warm 50 C, hot 57 C (firmware stops at 60) | hot: **COOL** - pause until 45 C |
| **Feeder servo (MG90S)** | **none** | **12 pushes a minute** | the feeder **rests 45 s** (the arm finishes the cubes on the belt); the cloud says "Feeder resting to cool" |
| Raspberry Pi | SoC (read by the optimizer) | warm 70 C, hot 80 C | hot: **COOL** - rest until 65 C |
| Any arm servo straining | current | over 1500 mA for 2 s | **COOL** (blocked / overloaded) until the strain is gone |

Every rest lasts **at least 30 s**, names the part (e.g. *Resting to cool: J3 elbow 66 C (limit 65)*), keeps the belt
still and the feeder off, and ends by itself. Test: `orchestrator/test/test_heat_guard.py`.

**The feeder servo is relaxed between pushes** (no signal = no holding current): it was held at its back angle all the
time, and a servo that presses even a little against the stack or a stop heats up while "resting". It gets its signal
back right before each push. The feeder test does the same. Settings (autoen): `--feeder-relax 1`,
`--feeder-max-per-min 12`, `--feeder-rest-s 45`.
For a real feeder temperature: a second DS18B20 taped to the servo on the belt's OneWire bus (firmware change).

### A report after every run (2026-11-12)
When a run ends, autoen writes **`run_reports/run_<stamp>.md`**: cubes, time and **energy per cube**, and for every
servo (J1-J6) and the belt motor the temperature (start / avg / max / rise), current (avg / peak), voltage and power,
the 12 V rail, the Raspberry Pi (temperature, CPU, memory, power warnings - from `run_logs/pi_<day>.csv`, written by the
dashboard every 10 s), what happened (emergency grips, rest probes, drifts) and a verdict with warnings.
Also `run_<stamp>_devices.csv` (a spreadsheet) and **`run_reports/runs.csv`** (one line per run, to compare runs).
By hand: `python3 orchestrator/cal/run_report.py` (the newest run) or `... run_report.py 20261112_200041`.

## 8. The camera today

- Used for: **checking every push** (a new cube at the left edge), the **gaps in mm** between cubes, learning the
  feeder exit, the live picture.
- **pt3: the new camera's geometry is ACTIVE** (`orchestrator/camera_geometry.json`: exit 65 px, IR 805 px,
  0.64 mm/px - feeder to IR 474 mm, which matches the belt's own 470 mm). So **colour sorting works with the new
  camera: white -> right bin, orange -> left bin**, and stuck cubes on the belt, backlog and slip are seen again.
- The IR position (805 px) is measured indirectly (the arm hides that spot). If cubes go to the wrong bin, run the
  1-minute check with one cube at the IR: `cd ~/Desktop/PRISM/orchestrator && ~/orch-env/bin/python cal/check_camera_geometry.py`.

- **Calibrate on the dashboard** (Control -> Calibrate): the plain camera picture; click a cube and label it
  (white / orange / light). *Fit colours from my labels* makes the colour ranges from your clicks and keeps the light
  out where its colour differs. *Cubes test* shows what would count as a cube and what is dropped as light (and why).
  The camera's own settings (exposure, white balance, 50 Hz flicker, focus) change the picture at once; *Lock the
  picture as it is now* stops it changing by itself. **Check the changes -> Save**: the old file goes to
  `backup/vision/`, vision uses the new one within ~2 s, no restart. **Undo the last save** brings the old one back.
  Colours and shape are only previewed in the browser until Save; the ROI and belt direction are not changed there.

## 9. On the dashboard (http://127.0.0.1:8080)

**Overview**
- **The PRISM cloud (centre)** always says in one sentence what is going on - *Waiting for a refill*, *Feeder not
  moving - check its plug*, *Missed the grip - trying again*, *I'm raising the belt to 22 rpm*, *All good - 3 cubes on
  the belt*. Its colour and face follow its mood (grey asleep, green good, blue info, amber warning, red alarm).
  Under it (one click): every push with its evidence, the decisions and the stories.
- **READY** and the clock; **Vision** (live picture); **Autoen** (Start / Pause / Stop and the main settings).
- **After Stop** the Autoen block gets ready for a new run by itself (about 10 s: a fresh autoen, nothing moves):
  the form is filled again and **Start** comes back - set the values, tick the box, Start.

**Machine** - the **Raspberry Pi** (CPU, memory, disk, temperature, CPU speed, load, power supply) on top, then one
verdict (*In good shape* / *Good, with notes* / *Needs attention* / *Not running*) and plain tiles: 12 V power, belt
motor, arm servos (temperature and current against their limits), arm and belt analysis, MQTT; programs, connections,
the log.

Lighter than before: live data 4 times a second (was 10), the camera picture at most 1.5 per second, repaints at most
once a second. The 3D model is gone.

## 10. Numbers in use

| Setting | Value |
|---|---|
| belt / arm / carry | 20 rpm / 1.5 / 1.5 |
| cubes on the belt (pt3) | as many as fit: 470 mm / (40 mm cube + gap), max 6 |
| gap between cubes (pt3) | >= 60 mm (tuner may use 80 / 100) |
| push | wait back 1000 ms, jump, hold **800 ms** (learned longer if needed, max 1500), return 300 ms |
| push only if rail >= | 11.5 V; arm phases with rail < 11.0 V avoided |
| min time between pushes | 2 s |
| nudge to look again | 1 s of belt |
| next push after the last cube travelled | one cube + one gap (encoder, or the camera) |
| a cube is "not coming" after | 1.35 x its ride (tuner may use 1.25 / 1.6) |
| feeder empty after | 3 pushes in a row bring nothing |
| servo ALARM after | 3 dead pushes in a row |

## 11. Commands

| | |
|---|---|
| start | `cd ~/Desktop/PRISM && ./start_cell.sh` |
| stop the run (vision + helpers keep running) | `./stop_cell.sh` |
| stop everything | `./stop_cell.sh --all` |
| the old one-at-a-time cycle | `./start_cell.sh --serial` |
| go back to a saved version | `versions/pt1/README.md`, `versions/pt2/README.md` |

## 12. Saved versions and what is switched off

- **pt1** (`versions/pt1/`): flow before the push checks. **pt2** (`versions/pt2/`): every push checked, full-stroke
  push. **pt3** (`versions/pt3/`): this version - fill the belt, tuner, camera geometry active.
- **Switched off on purpose**: the Claude advisor (`advisor.py`, off - you chose not to use it).
- **Hardware advice that still stands**: a **separate 5 V supply for the feeder servo** - the arm's grip pulls the
  shared supply down to ~2 V, which is why the optimizer must avoid pushing then.

## The FAULT system (2026-11-13)

The Raspberry Pi, the dashboard and the logging **never stop** - a fault only pauses the cell (it never shuts anything down
and never uses the relay). Code: `orchestrator/guardian.py` (detection), `optimize.py` (`_fault_step`, mode **FAULT**),
`actuate.py` (jobs `fault_stop`, `fault_retry`). Limits: `orchestrator/limits.py` (bottom).

| Fault | Detected when |
|---|---|
| camera_lost | no camera pictures for 5 s (after pictures were seen) |
| camera_covered | the belt picture is black (brightness < 18, or >= 92 % too-dark pixels) for 2 s |
| foreign (hand / object) | something bigger than 8 % of the belt area that is not in the empty-workspace picture, for 1 s, **only while the arm is parked** (Standby / waiting) - needs the picture saved on Calibrate (no picture = check off) |
| feeder_link | no data from the Mega (belt + feeder board) for 3 s |
| arm_link | no data from the arm servos for 3 s |
| feeder | the feeder servo does not move (no current on the 12 V rail after the push; replaces the old ALARM) |

**What happens:** (1) the **checkpoint**: autoen pauses at the end of the cycle (the cube in the gripper is placed first) and the
transport is aborted; the arm goes home, the belt brakes. (2) the **warning**: the arm shakes its wrist 3 times, 3 sets, one
minute apart (not for a hand in the workspace). (3) the arm goes to the **rest pose** and the cell waits. (4) every 30 s for
5 minutes, then every 2 minutes, it **tries again**: feeder = ONE push, then the belt rolls a little (8 rpm, 1.5 s) so cubes do
not stack or stick; camera = its program is started again; link/hand/covered = it looks again (a USB link cannot be reopened
from here - the banner says what to check). (5) when the fault has been gone for 10 s (hand: 3 s; feeder: a retry push draws
current) it **wakes the arm and resumes by itself**. (6) the same fault 3 times in 10 minutes: it stops resuming by itself and
waits for your Resume. Your Resume / wake command ends a fault at once.

The dashboard shows mode FAULT, the stage, the tries and the next try in the Overview voice and in the Limits zone.
**Hand check:** Calibrate -> "Empty workspace picture" (belt empty, arm at Standby, nothing in view). Not a safety stop: the camera
can miss a hand and the arm needs seconds to reach rest; the physical E-stop stays the real protection.

Not tested on the real cell: only the simulator (`orchestrator/test/test_guardian.py`, `test_fault_sim.py`, `test_feed_check.py`).
The checkpoint parks at **Home** (not Standby): every rest and wake path in autoen starts from Home.

### Changes of 2026-11-13 evening
- **A fault fixed during the warning resumes at once** (hand taken off the lens): the optimizer sends the resume, the running
  job stops at its next safe point; no waiting for the other shake sets and the rest pose. Camera-covered clears after 5 s.
- **The learning of the 200-cube run** is in `orchestrator/learned_state.json` (merge: pt5 + the main PRISM + PRISM_mek; the
  tuner champion gap 60 mm / late 1.25, push 1050 ms, and 214 cycles on 22 rpm / arm 1.75). The previous file is
  `backup/orchestrator/learned_state.json.before-mek200`. The optimizer saves its own state as it runs.
- **Feeder preset fd1 (170 / 10) is active** (the angles of the 200-cube run). fd2 has never run on the real cell.
- **Calibrate -> Working area:** drag a box (or click 3-8 corners) over the belt, Save: vision uses it in ~2 s and relearns the empty
  belt. Refused: fewer than 3 corners, outside the picture, smaller than 2 % of it. Undo = the Undo button (backup kept).
- **NaN fix:** autoen's state carried a NaN that made the dashboard show "undefined"; the dashboard now turns NaN into null.
