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
