# PRISM - information book (source material for the report)

State: **pt5**, 2026-11-12 (evening). Every number here was measured on the cell or read from its code and logs on that day,
unless it says "design value". Sections: 1 overview · 2 architecture · 3 the eight layers (5W1H each) · 4 timing ·
5 power and energy per version · 6 version history · 7 design principles · 8 how-to · 9 limits and future work ·
10 folder map · 11 glossary.

---

## 1. PRISM in one page (5W1H)

| | |
|---|---|
| **What** | An autonomous sorting cell: a feeder pushes 40 mm cubes (white / orange) onto a belt; the belt brings them to a pick point; a 6-joint robot arm picks each cube and drops it into a bin by colour. A Raspberry Pi runs the software; an Arduino Mega runs the motors and sensors. |
| **Why** | To build a small, real production cell whose control is **intelligent**: it measures itself, decides by evidence, learns its own best settings, recovers from problems by itself, explains every decision, and stays safe. |
| **Who** | The operator starts, stops and supervises; the **optimizer** (software "brain") makes the run-time decisions; **autoen** (software "body") moves the hardware; the operator is the only one who starts a hardware move. |
| **Where** | A desk cell: belt ~0.5 m feeder-to-pick (470 mm measured), arm beside the pick point, camera above the belt, two bins. All software on one Raspberry Pi 4 in `~/Desktop/PRISM`. |
| **When** | Development Oct - Nov 2026; this book describes pt5 (2026-11-12 evening). |
| **How** | Layers that talk over **MQTT** (publish/subscribe, local broker) and **HTTP** (commands): sensors -> firmware -> body -> perception -> brain -> data -> user interface. Details in section 2. |

**Result today (pt5):** 21 of 21 cubes picked with **no pile-ups and working colour sorting** (10 left / 11 right),
13.3 s and **22.7 J per cube**. The fastest and leanest run is still pt4 (11.7 s, 18.5 J per cube) - but in that run 9 of
10 cubes went to the right bin, so its sorting was not working; pt5 trades ~14 % speed for no pile-ups and correct
sorting. The first serial version took 18.7 s and 29 J per cube (section 5).

---

## 2. Architecture

```
                         Operator (dashboard http://127.0.0.1:8080, start_cell.sh)
                                              |
  L8 User interface   dashboard_node.py (+ overview.py, web/)  start_cell.sh / stop_cell.sh
                                              |  reads MQTT, sends HTTP
  L7 Data & monitoring  logger.py (arm.db, camera.db)  health.py  arms_ana.py  conv_ana.py   (passive: never move)
                                              |  MQTT
  L5 Brain (orchestrator)  main.py: observe -> predict -> optimize -> actuate   (20 Hz tick)
        |   ^ MQTT: camera/*, belt/*, arm/*, autoen/*         | HTTP to autoen :8781 (feed, verdict, config, pause, task)
  L4 Perception  vision_usb.py (camera, colour, tracks)  ----> MQTT camera/perception, video/camera/jpeg
                                                              |
  L3 Body  auto_en_new.py (autoen): the cycle, the belt/feeder controller, HTTP API :8781, MQTT arm/* belt/* autoen/*
        |  USB serial 115 200 baud, text lines
  L2 Firmware  Arduino Mega 2560: so101_mega_controller.ino (arm) + belt_module.h (belt, feeder, sensors)
        |  servo bus 1 Mbaud, PWM, I2C, GPIO
  L1 Hardware  SO-101 arm (6 x STS3215), belt motor + encoder, feeder servo, IR sensors, INA260, camera, 12 V supply
```
`L6 Communication` (MQTT + HTTP) connects all of them (section 3.6).

**Principles of the architecture**
- **Body and brain are separate programs.** autoen knows *how* to move safely; the optimizer decides *what* to do. Either can be restarted alone; autoen also runs without the optimizer (simple cycle).
- **Publish / subscribe for data, request / reply for commands.** Data flows one way on MQTT (any number of listeners, nobody waits); commands go point-to-point over HTTP with a clear answer (accepted / refused).
- **Passive helpers.** Loggers, health, analyzers and the dashboard only listen - they can crash or restart without touching the robot.
- **Safety in the lowest layer that can enforce it.** Watchdogs in the firmware stop the belt and arm if the PC goes silent; autoen holds the belt while the arm grips; the optimizer cannot bypass either.

---

## 3. The layers (5W1H each)

### 3.1 L1 - Hardware

| Part | Model / detail | Connected to |
|---|---|---|
| Computer | Raspberry Pi 4 Model B (4 GB), Raspberry Pi OS, kernel 6.18, Python 3.11 | everything (USB) |
| Controller | Arduino Mega 2560 (CH340 USB), sketch of 2026-11-11 22:39 | Pi USB |
| Robot arm | SO-101, 6 x Feetech STS3215 serial servos (J1 base ... J5 wrist roll, J6 gripper), 12 V | Mega servo bus, 1 Mbaud |
| Belt | DC gear motor JGB37-520 with Hall encoder (11 880 counts per output turn), BTS7960 H-bridge | Mega D2/D3/D8/D9, encoder D6/D7 |
| Feeder | MG90S micro servo pushing a blade under a stack of cubes | Mega D12 |
| Pick IR | FC51 IR obstacle sensor at the pick point | Mega D22 |
| Feeder IR | FC51 at the feeder (cubes present?) | Mega D36 |
| Power sensors | INA260 #1 belt 12 V rail (voltage, current), INA260 #2 arm rail | I2C D20/D21, soft I2C D50/D52 |
| Temperature | DS18B20 on the belt motor | Mega D24 (1-Wire) |
| Camera | Astro HD Camera (USB, 0408:2090), 1280x720 MJPG; earlier: RealSense D415 (broken), H264 USB camera (spare) | Pi USB |
| Supply | one 12 V supply for arm, belt and (through a 5 V regulator) the feeder servo | - |

- **Why these parts:** low-cost, hobby-grade parts that still report what they do (servo position/current/temperature, belt encoder, rail current) - the software can only be intelligent about what it can measure.
- **Known hardware limit:** the shared supply. When the gripper squeezes, the rail measured ~2 V (12:49 run): the feeder then has no power. Fix proposed: a separate 5 V supply for the feeder servo.

### 3.2 L2 - Firmware (Arduino Mega)

| | |
|---|---|
| **What** | Two parts in one sketch: the arm controller (`so101_mega_controller.ino`) and the belt module (`belt_module.h`: belt speed control, feeder, IR, INA260, temperature, telemetry). |
| **Why** | Hard real-time jobs (encoder at 10 kHz, PID speed loop, stopping at the IR within milliseconds, watchdogs) cannot run on Linux reliably. |
| **Who** | Talks only to autoen. |
| **Where** | `arduino_mega_2560/` (uploaded with the Arduino IDE; folder must be named like the .ino). |
| **When** | Always on; restarts when the USB port opens. |
| **How** | Text protocol over USB serial 115 200 baud. Arm lines: `P,<id>,<ticks>,<speed>,<acc>` (move), `H` (hold), `T,...` (telemetry). Belt lines start with `X,`: `X,CONV,<rpm>` (run until the IR, then brake + `X,E,ARRIVED`), `X,FEED` (one push -> `X,E,FEED_DONE`), `X,FEEDCFG,<home>,<push>,<hold_ms>,<return_ms>`, `X,FEEDPRE,<ready>,<wait_ms>,<ramp_ms>`, `X,SERVO,<deg>`, `X,BRAKE`, `X,STOP`. Telemetry `X,T,...` every 100 ms (rpm, current, voltage, IR, feeder state, feeder IR, faults). |

**Principles:** the Mega never decides *what* to do, only executes safely; watchdogs (arm 350 ms, belt 500-1500 ms) stop motion if the PC goes quiet; the belt stops at the IR by itself (no PC round trip); every event (ARRIVED, FEED_DONE, FAULT) is reported with the Mega's own clock.

### 3.3 L3 - Body: autoen (`autoencoder/auto_en_new.py`, 3 148 lines)

| | |
|---|---|
| **What** | Runs the work cycle on the hardware, owns the Mega's serial port, offers an HTTP API on port 8781 and a web page, publishes telemetry on MQTT, logs every run (`riff_logs/`). |
| **Why** | One program must own the hardware, know the poses and the safe order of moves, and refuse unsafe requests. |
| **Who** | Commanded by the optimizer (HTTP) and the operator (web page / start_cell). |
| **Where** | `autoencoder/`, env `autoen-env`. Poses in `config/poses.json`. |
| **When** | Started by `start_cell.sh` step 2; runs until Stop. |
| **How** | Cycle modes: **serial** (feed -> belt -> pick, one cube at a time), **pipeline B** (feed the next cube during the grip), **flow F** (default since pt1: the arm only picks; a controller thread runs the belt and feeds only on the optimizer's request). Safety rules it enforces: belt held while the arm grips, lifts and comes down to Standby; the first cube is held short of the IR until the arm is parked; a push the optimizer does not confirm does not move the belt (pt2). |

HTTP API (selection): `GET /api/state` · `POST /api/start` · `/api/pause` (toggle) · `/api/stop` · `/api/config` (settings, take effect at the next move) · `/api/task` (hold_pose, belt_run, rest, wake, shake) · `/api/feed` (flow: one push) · `/api/feed_verdict` (fed / empty / nudge) · `/api/abort_transport`.

### 3.4 L4 - Perception: vision (`vision/vision_usb.py`, 2 589 lines)

| | |
|---|---|
| **What** | Finds the cubes on the belt in the camera picture: colour, position, speed, track id; measures the light; publishes a JPEG for the dashboard. |
| **Why** | The camera is the only sensor that sees *what* is on the belt and *where* - colour for sorting, proof that a push delivered a cube, gaps between cubes. |
| **Who** | Feeds the optimizer (and the camera logger, the dashboard). |
| **Where** | `vision/`, env `prism-env`; calibration `vision_calibration_usb.json` (ROI, colours, belt direction). |
| **When** | Started by `start_cell.sh` step 1; camera 15 fps, processing **6 fps**. |
| **How** | 1280x720 -> 854x480; inside the belt ROI: HSV colour masks (white: S <= 50, V >= 180; orange: H 9-21, S >= 120, V >= 90) -> blobs -> Kalman-filtered tracks with ids -> MQTT `camera/perception`. Scene statistics (mean brightness, % burnt out, % dark) for the light check. Camera geometry for the brain (`orchestrator/camera_geometry.json`): feeder exit 65 px, pick IR 805 px, **0.64 mm per px** (cube 62 px for 40 mm, and belt speed 88 px/s = 56.2 mm/s - two methods agree). |

### 3.5 L5 - Brain: the optimizer (`orchestrator/`, ~3 900 lines + 31 test suites)

| | |
|---|---|
| **What** | Decides everything at run time: speeds, when to push the feeder, how many cubes ride, what each push really did, which bin, what to do when something goes wrong - and explains it. |
| **Why** | Fixed scripts break on real hardware (weak pushes, empty feeder, glare, power dips). A brain that measures and decides by evidence keeps the cell running and improves it. |
| **Who** | Runs alone; the operator can override (Pause / Resume / park / wake). |
| **Where** | `orchestrator/`, env `orch-env`; memory in `learned_state.json`, logs in `orchestrator/logs/orch_*.jsonl`, `run_logs/optimizer_*.log`. |
| **When** | Started by `start_cell.sh` step 3, before Start; **20 Hz** tick. |
| **How** | `observe.py` (MQTT -> world state) -> `predict.py` (cube positions, Kalman) -> `optimize.py` (decisions) -> `actuate.py` (HTTP to autoen). |

Its main abilities (pt4):
1. **Settings choice** - measures each belt / arm speed setting live (time, energy per cube, success rate) and uses the fastest safe one; tries one step faster now and then, backs off at the first problem.
2. **Fill the belt (pt3)** - cubes on the belt = 470 mm / (40 mm + gap), up to 6; the next push only when the last cube is one cube + one gap away (encoder or camera). The arm then never waits for a whole ride.
3. **Verified feeding (pt2)** - after every push: servo current (12 V dip: moved / did not move), camera (a new cube at the feeder exit), feeder IR (recorded) -> **fed** (belt runs) / **nudge** (1 s belt, look again) / **empty** (push again).
4. **Power-aware pushing** - no push below 11.5 V; learns which arm phases pull the rail down (GRIP) and avoids them.
5. **Push length learning** - a cube stuck at the exit -> pushes again, longer (800 -> 1500 ms), remembers what works, never below a length that failed.
6. **Self-recovery** - feeder empty -> pause, probe now and then, restart by itself after a refill; dead servo -> ALARM (arm shakes 3 times) and timed retries; late cube -> give up, push again; incidents -> automatic responses learned per incident type.
7. **Colour sorting** - an ordered list of the cubes on the belt (`sortq.py`); colour by majority vote over many frames; white -> right bin, orange -> left bin.
8. **Self-experimenting (`tuner.py`)** - tries one knob at a time (gap 60/80/100 mm, late limit 1.25/1.35/1.6) for 6 picks; keeps it only if >= 3 % faster without more failures.
9. **Explanations** - every stop is a story (tried / saw / so / next) on the dashboard and in the log; every push has its evidence line.

### 3.6 L6 - Communication: MQTT and HTTP

**MQTT** (Mosquitto 2.0.21 on the Pi, localhost:1883). Measured with the cell idle, 10 s:

| Topic | From | Rate (idle) | Size | Content |
|---|---|---|---|---|
| `arm/telemetry`, `arm/state`, `arm/ir`, `arm/gripper` | autoen | 10 Hz while running | ~160 B | joints, phase, IR, gripper |
| `belt/telemetry`, `belt/event`, `belt/status` | autoen | 10 Hz while running | ~200 B | rpm, current, 12 V, IR, feeder state; ARRIVED / FEED_DONE |
| `autoen/state`, `autoen/cycle` | autoen | per change / per pick | ~200 B | status, every finished pick |
| `camera/perception` | vision | up to 6 Hz | ~570 B | tracks (id, colour, u, v, speed), light stats |
| `video/camera/jpeg` | vision | ~3 Hz (2.3 measured) | ~53 kB | the annotated picture for the dashboard |
| `camera/status`, `camera/demand`, `camera/window` | vision | slow | ~150 B | camera health, counting windows |
| `orchestrator/plan`, `orchestrator/status`, `orchestrator/cmd` | optimizer / operator | per decision / 1 Hz / commands | ~3 kB | decisions + score (stories, pushes, flow) |
| `health/status` | health.py | 0.6 Hz | ~500 B | Pi and program health |
| `analyzer/arm/state`, `analyzer/belt/state` | analyzers | 1.1 Hz | ~450 B | arm / belt analysis |
| `logger/*/heartbeat`, `logger/*/status` | loggers | 0.5 Hz | ~50 B | logger alive |

**Why MQTT:** one publisher, any number of listeners (brain, loggers, dashboard) without changing the publisher; a slow listener never slows the robot; topics document themselves.
**HTTP** is used for commands, because a command needs an answer (accepted / refused, e.g. "tasks only run while paused").

### 3.7 L7 - Data and monitoring

| Program | What it does | Store |
|---|---|---|
| `logger/logger.py` (x2) | records every MQTT message of `arm/#` and `camera/#` | `logger/arm.db`, `camera.db` (SQLite) |
| `health/health.py` | Pi temperature, memory, under-voltage, programs alive | `health/status` |
| `analysis/arms_ana.py`, `conv_ana.py` | live analysis of the arm and the belt | `analyzer/*`, SQLite |
| autoen's own logs | every command and reply (`autoen_tx_*.log`), every sample (`autoen_raw_*.csv`), every pick (`autoen_cycles_*.csv`) | `autoencoder/riff_logs/` |
| optimizer logs | every decision (`orch_*.jsonl`), the human-readable log with stories | `orchestrator/logs/`, `run_logs/` |

**Why:** every number in this book came from these files - the cell can be studied after the fact, and the optimizer learns from the same data.

### 3.8 L8 - User interface

- **Dashboard** (`dashboard_node.py`, http://127.0.0.1:8080): Overview page - READY checklist (every program and every piece of hardware), alarm banner, Pi status, the optimizer's "right now" sentence, the last pushes with their evidence, the experiment in progress, the stories, the live camera, Start / Pause / Stop and the main settings. Updates 10 times a second (SSE).
- **Scripts:** `start_cell.sh` (starts everything in order, waits for READY, asks before the first move), `stop_cell.sh` (`--all` = everything), `feeder_test.py` (feeder only, manual p / l).
- **Why:** the operator must see in one look whether the cell is healthy and what it is doing and why.

---

## 4. Timing

### 4.1 Rates (frequency)

| Loop | Rate | Where |
|---|---|---|
| Belt encoder sampling | 10 000 Hz | Mega (Timer2 interrupt) |
| Arm servo bus | 1 Mbaud | Mega <-> STS3215 |
| PC <-> Mega link | 115 200 baud, >= 10 ms between lines | USB serial |
| Mega telemetry | 10 Hz (every 100 ms) | `X,T` lines |
| Heartbeat PC -> Mega | 5 Hz (0.2 s); watchdogs 350 ms arm / 500-1500 ms belt | autoen |
| Camera / processing | 15 fps / 6 fps | vision |
| Live picture | 3 Hz (2.3 measured) | vision |
| Optimizer tick | 20 Hz | main.py |
| autoen state poll | 2 Hz (0.5 s) | actuate.py |
| Optimizer status | 1 Hz | actuate.py |
| Dashboard push | 10 Hz | dashboard |

### 4.2 Latency (measured 2026-11-12)

| Path | Time |
|---|---|
| Camera frame -> message received by the brain | **133 ms median, 163 ms p90** |
| MQTT publish -> receive (local) | **3.9 ms median** |
| autoen state -> optimizer (HTTP poll) | <= 500 ms |
| A new setting -> used | from the next arm move / belt run |
| Push request -> verdict (camera sees the cube) | typically 0.4-1.5 s (one push takes ~2.1 s) |

### 4.3 One pick (time budget, median of the flow run 01:07)

grip 1.7 s + lift 1.0 + carry 2.4 + release 1.4 + home 2.3 + down to Standby 0.8 = **9.6 s of arm work** (+ waiting).
Belt ride feeder -> IR: 470 mm at 56.2 mm/s (20 rpm x 2.81 mm/s per rpm) = **8.3 s**.

---

## 5. Power and energy per version

Energy = belt (12 V rail current x voltage) + arm (sum of the six servos' current x voltage), integrated over the
**whole run** (idle, failures, waiting included) and divided by the cubes picked. Time per cube = median of the
successful picks.

| Version (run) | Cubes OK | Time per cube | Belt power | Arm power | Energy per cube | Note |
|---|---|---|---|---|---|---|
| serial, old push (11-11 22:08) | 12 | 18.7 s | 0.74 W | 0.46 W | **29.0 J** | one cube at a time |
| serial, gentle push (11-11 22:44) | 6 | 19.9 s | 0.73 W | 0.39 W | 32.0 J | slower push |
| pipeline B (11-11 23:10) | 2 | 11.8 s | 1.12 W | 0.32 W | 168 J | pushes during the lift failed (shared supply) |
| **flow pt1** (11-12 01:07) | 31 | **11.6 s** | 0.66 W | 0.51 W | **19.8 J** | the optimizer plans every push |
| pt2 (11-12 12:48) | 0 | - | 0.44 W | 0.21 W | - | rail collapsed to ~2 V during GRIP |
| pt2, full push (11-12 13:15) | 4 | 19.0 s | 0.29 W | 0.30 W | 42.8 J | feeder ran empty / weak pushes |
| **pt3 / pt4 fill** (11-12 14:23) | 10 | **11.7 s** | 0.70 W | 0.66 W | **18.5 J** | belt filled, every push verified; sorting 1 left / 9 right (not working) |
| **pt5 no pile-ups** (11-12 20:00) | 21 | 13.3 s | 0.92 W | 0.64 W | **22.7 J** | room before every push, camera may only block, sorting 10 left / 11 right |

**Reading the table**
- From serial to flow / pt4: **-37 % time per cube, -36 % energy per cube** (29.0 -> 18.5 J).
- pt4 is not faster than pt1 per cube: both are limited by the arm's 9.6 s of work. pt4's gains are reliability (verified pushes, power-aware feeding, stuck-cube handling, colour sorting) and that the arm no longer waits for a whole ride when a push fails.
- The runs with problems (pipeline B, pt2) show where energy goes when feeding fails: the belt and the arm keep drawing power while no cube comes - why the verified feeding of pt2 matters.
- Simulator (same fake hardware for both): filling the belt cut the time per cube by 73-74 % against one cube at a time.
- **pt5 against pt4:** +14 % time per cube (11.7 -> 13.3 s), +23 % energy per cube (18.5 -> 22.7 J). Where it goes: the
  arm now waits a median **3.0 s** for the next cube instead of 1.3 s (pushes wait until the belt has made room), and
  the belt draws more (0.92 W against 0.70 W: it runs to make room, and nudges cubes / objects off the feeder exit).
  What it buys: **no pile-ups**, a ride that stays true (8.3 s / 468 mm all run; on 11-12 18:17 it fell to 230 mm
  and cubes piled up), cubes left on the belt handled, and **colour sorting that works** (10 / 11 against 1 / 9).
- Against the first serial version, pt5 is still **-29 % time and -22 % energy per cube** (18.7 -> 13.3 s, 29.0 -> 22.7 J).
- How these numbers are made: autoen's raw log of each run (`autoencoder/riff_logs/autoen_raw_<run>.csv`, all six
  servos' and the belt's current x voltage, ~4 samples per second), integrated over the whole run with gaps capped at
  0.5 s, divided by the cubes picked. Re-computed for pt1 (20.0 J) and pt4 (18.6 J) with the same script, matching
  the earlier numbers within 0.2 J.
- Runs are short (2-9 minutes); repeat longer runs for the final report.

---

## 6. Version history

| Version | Date | What changed | Why |
|---|---|---|---|
| v0 serial | Oct 2026 | autoen alone: feed -> belt -> pick, fixed settings | the first working cell |
| optimizer | late Oct - 11-10 | observe / predict / optimize / actuate; settings learned live; camera trust; IDLE / probes | intelligent control |
| pipeline B / C | 10-27, 11-11 | feed the next cube during the grip; 2 cubes on the belt | faster - but pushes during the lift failed (shared supply) |
| **pt1 flow** | 11-12 01:00 | the optimizer decides every push; up to 3 cubes in flight; learned push success per arm phase; late cube -> push again | "feed many cubes, no waiting for the cycle end" |
| **pt2** | 11-12 13:30 | every push verified (servo current + camera + IR); nudge; dead servo -> ALARM + arm shake; no push on a low rail; GRIP avoided; stuck cube -> longer push; full-stroke push (wait back 1 s, jump, hold 800 ms) | the feeder delivered nothing yet the belt ran; the rail collapsed during GRIP |
| **pt3** | 11-12 14:00 | fill the belt (distance-based: encoder + camera); camera geometry active (colour sorting with the new camera); tuner connected | "many cubes in one cycle, by IR, camera, speed and distance" |
| **pt4** | 11-12 15:00 | white cubes are not "glare" (sorting fix); stuck-cube proof fix; feeder 170 -> 10 deg | first pt3 runs |
| **pt5** | 11-12 20:05 | room before every push (encoder); camera may only block a push, nudges cubes off the exit, ignores still reflections; "empty" only with a proven camera; cubes nobody fed do not spoil the ride; sorting "orange -> left, else right"; dashboard Calibrate page, Pi gauges, no 3D twin; vision shape filter, no desktop window | cubes piled up on 11-12 18:17; the camera saw light as cubes |

Saved copies: `versions/pt1`, `versions/pt2`, `versions/pt4` (pt4 contains pt3), `versions/pt5`.

---

## 7. Design principles

1. **Safety first, never traded.** Hard rules (belt held while gripping, no cube at the IR before the arm is parked, watchdogs, the alarm on a dead servo) sit below every optimization; no learning can switch them off.
2. **The operator starts every hardware move.** Software prepares; a person presses Enter / Start.
3. **Body and brain separated.** autoen = how to move safely; optimizer = what to do.
4. **Decide by evidence, not by assumption.** A push is "fed" only when a sensor shows it; a setting is "best" only after it was measured; a fix is kept only if it measured better.
5. **Measure what you want to improve.** Time, energy, success, rail voltage, every push - all logged and used.
6. **Learn, but carefully.** Small experiments, one change at a time, back off at the first problem, remember what failed.
7. **Fail safe and recover by itself.** On doubt: stop the belt, pause at home; then try again by itself (probes, retries) instead of waiting for a person - unless a person is really needed (alarm).
8. **Explain every decision.** Stories (tried / saw / so / next) and evidence lines, in plain words.
9. **Separate passive from active.** Loggers, health, analyzers and dashboard never move anything.
10. **Versions and backups.** Every change has a backup (`backup/<path>.before-<reason>`); working states are saved (`versions/ptN`); 31 automatic test suites run the real programs against a fake Mega before anything reaches the robot.

---

## 8. How to

| Task | Command / step |
|---|---|
| Start the cell | `cd ~/Desktop/PRISM && ./start_cell.sh` -> wait for READY -> Enter |
| Stop the run | `./stop_cell.sh` (vision and helpers keep running); everything: `./stop_cell.sh --all` |
| Old one-at-a-time cycle | `./start_cell.sh --serial` |
| Watch | dashboard http://127.0.0.1:8080 ; log `tail -f run_logs/optimizer_*.log` |
| Test only the feeder | stop the cell, then `cd autoencoder && ~/autoen-env/bin/python feeder_test.py --manual` (p push, l pull, f full feed, q quit) |
| Check the camera geometry | one cube at the pick IR, then `cd orchestrator && ~/orch-env/bin/python cal/check_camera_geometry.py` |
| Go back to a version | `versions/ptN/README.md` |
| Run the tests | `cd orchestrator && for t in test/test_*.py; do PYTHONPATH=. ~/db-env/bin/python $t; done` |

---

## 9. Limits and future work

- **Power:** the shared 12 V supply collapses during the grip -> a separate 5 V supply for the feeder servo.
- **Arm speed is the ceiling** (9.6 s of work per pick): shorter arm paths, faster safe speeds (the optimizer already tries them carefully).
- **Feeder sensing:** the 12 V dip shows the servo moved, not that a cube came out; a current sensor on the feeder's own 5 V line would.
- **Camera:** the pick IR position (805 px) is measured indirectly (the arm hides the spot) - confirm with the geometry check; keep the belt area free of glare.
- **Gripper clearance:** the minimum gap between cubes (60 mm) is a safe guess - measure the fingers.
- **Longer runs** for the final energy and reliability numbers.
- Optional: the Claude advisor (`advisor.py`, off) - a language model that reads the optimizer's stories and suggests experiments.

---

## 10. Folder map (`~/Desktop/PRISM`)

| Folder | Content |
|---|---|
| `arduino_mega_2560/` | the Mega sketch (arm + belt module) |
| `autoencoder/` | autoen (the body), feeder_test.py, `riff_logs/` (every run: commands, samples, picks) |
| `vision/` | vision_usb.py (camera), calibration |
| `orchestrator/` | the optimizer (main, observe, predict, optimize, actuate, scene, sortq, incidents, forecast, meter, camgeo, tuner, advisor), `test/` (31 suites), `cal/` (experiments, geometry check), `logs/`, `learned_state.json`, `camera_geometry.json` |
| `dashboard/` | dashboard_node.py, overview.py, `web/` (the pages) |
| `logger/`, `health/`, `analysis/` | the passive helpers and their databases |
| `config/` | arm poses |
| `docs/` | HOW_TO_USE_PRISM.md, PRISM_BEHAVIOUR.md, this book, reports |
| `versions/` | saved working versions pt1, pt2, pt4 |
| `backup/` | every changed file before its change |
| `run_logs/` | the programs' console logs of every start |

## 11. Glossary

| Word | Meaning |
|---|---|
| autoen | the body program that moves the hardware |
| optimizer / orchestrator | the brain program |
| flow | the mode where the optimizer plans every push and a pick never waits for a cycle end |
| push / feed | one stroke of the feeder blade |
| verdict | the optimizer's judgement of a push: fed / nudge / empty |
| ride | a cube's trip from the feeder exit to the pick IR (470 mm, 8.3 s at 20 rpm) |
| gap | free belt between two cubes (>= 60 mm) |
| rail | the 12 V supply line, measured by the INA260 |
| dip | how much the rail voltage drops during a push (~200 mV = the servo moved) |
| Standby / Home | the arm's waiting pose above the pick point / its safe pose |
| story | the optimizer's explanation of a stop: tried / saw / so / next |
| tuner | the part that experiments with the optimizer's own knobs |
| pt1 ... pt4 | saved versions of the software |
