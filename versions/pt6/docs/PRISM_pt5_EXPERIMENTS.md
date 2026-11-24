# PRISM pt5 - experiments and results (PRISM and PRISM_friend)

All numbers: runs on the real cell, 2026-11-12, measured from the cell's own logs. Where a number is only from the
simulator, or not tested yet, it says so. Written 2026-11-12 (night). Hardware was being repaired at the time of writing.

## 1. Summary

- **pt5 stopped the cubes piling up at the feeder.** On the failing pt4 run (18:17) the measured belt length from feeder to
  pick point fell from 468 mm to 230 mm and cubes stacked. In all three full pt5 runs it stayed at **460-465 mm** (the real
  length is about 468 mm).
- **Colour sorting works** in all three pt5 runs: 15 + 18, 16 + 14 and 23 + 21 cubes to the left / right bin, **0 wrong
  bins** by the optimizer's own check (2 / 3 / 7 cubes had no colour seen and went right by default).
- **Speed and energy per cube are about the same as pt4, within the spread between runs:** median **11.0-13.3 s** per cube
  and **15.6-22.1 J** per cube in pt5, against 11.7 s and 18.6 J in the pt4 run. (My first estimate that pt5 was 14 %
  slower and 23 % more costly came from one run and does not hold; see section 7.)
- **Two runs of the identical code differ by 13-23 %** (PRISM 20:00 against PRISM_friend 21:07). One run cannot prove a
  small improvement; repeat each setting at least three times.
- **Temperatures and power are comfortable:** hottest arm servo 42 C (warm limit 55 C), belt motor 25 C, the 12 V rail never
  below 11.9 V (the pushes stop below 11.5 V).
- **One real problem found:** the feeder servo failed to move at the end of the 21:30 run (alarm after 12 pushes without
  current) while the operator reported it very hot. It has no temperature sensor, so heat is likely but not proven.

## 2. The two setups

| | PRISM | PRISM_friend |
|---|---|---|
| What | The main folder `~/Desktop/PRISM`. pt5, then the later changes (section 9) | A separate copy of pt5 (`~/Desktop/PRISM_friend`) made to experiment with |
| Code | pt5 at 20:00; rest probe added by 21:30; more changes after | **Exactly pt5**: the 16 core files (autoen, optimizer, actuate, vision, dashboard, camera geometry, bins) are identical to `versions/pt5` |
| Learning | Its own (`orchestrator/learned_state.json`) | Started from pt5's, then learned on its own |
| Robot | The same single robot, Pi and Mega. The two folders cannot run at the same time | |

Because the code is identical, the PRISM_friend run is a **repeat of pt5**. It shows how much two runs of the same code differ.

## 3. The runs

All runs: belt 20 rpm, arm speed 1.5, feeder 170 deg back / 10 deg push (the old feeder angles; fd2 has not been tried
on the cell yet), camera colour calibration saved at 18:04.

| Run | Folder | Code | Length | Cubes OK / cycles | Left / right | Note |
|---|---|---|---|---|---|---|
| **A** 20:00 | PRISM | pt5 | 31.9 min | 33 / 43 | 15 / 18 | About 22 of the 32 minutes the cell rested with an empty feeder (6 times) |
| **B** 21:07 | PRISM_friend | pt5 (repeat) | 13.2 min | 30 / 30 | 16 / 14 | Ended paused by the operator |
| **C** 21:30 | PRISM | pt5 + rest probe | 10.5 min | 44 / 49 | 23 / 21 | Ended with a feeder servo alarm |
| 21:21 | PRISM | pt5 | 1.1 min | 2 / 2 | 0 / 2 | Start-up test |
| 21:24 | PRISM | pt5 | 2.0 min | 8 / 8 | 2 / 6 | Start-up test |
| **D** 00:25 (11-13) | PRISM | pt5 + rest probe + heat guard | 33.9 min | 93 / 102 | 48 / 45 | The longest run; autoen printed preset fd2 but the Mega received **fd1 values (170 / 10)** - see section 12 |

The two start-up tests are too short for conclusions and are left out of the comparisons below.

## 4. How it was measured

- Source: autoen's own logs: `autoen_raw_<run>.csv` (every servo's temperature, current and voltage, the belt's, 5 samples
  per second), `autoen_cycles_<run>.csv` (every cycle) and the optimizer's `orch_*.jsonl`. Reports: `run_reports/`.
- **Energy** = voltage x current of the belt and the six arm servos, added up over the recorded samples (gaps over 0.5 s
  are not counted). **It does not include** the Pi, the Mega, the motor drivers or the feeder servo, and **there are no
  samples while autoen is paused**, so energy spent while resting is not in it.
- **Median s per cube** = the median cycle time of cycles that ended OK, without the first cycle.
- **J per cube (all)** = all recorded energy / cubes picked. **J per cube (picked cycles)** = only energy inside cycles that
  picked a cube, so waiting and failed cycles do not count.
- Left / right bin comes from the cycle log. "Wrong bin" is the optimizer's own consistency check, **not** a person
  looking at the bins. A spot check by eye is still worth doing.

## 5. Results

### 5.1 pt5 against the earlier versions (same method, same cell)

| Version (run) | Cubes OK | Median s / cube | J / cube (all) | J / cube (picked cycles) | Left / right | What it shows |
|---|---|---|---|---|---|---|
| serial (11-11 22:08) | 12 | 18.7 | 29.0 | - | - | One cube at a time. Different recording tool, same formula (information book) |
| pt1 flow (01:07) | 31 / 33 | 11.6 | 19.9 | - | 0 / 31 | The optimizer plans every push; no colour sorting yet |
| pt4 (14:23) | 10 / 10 | 11.7 | 18.6 | 18.6 | 1 / 9 | Sorting not working (1 left, 9 right) |
| pt4 (18:17) | 8 / 8 | 10.4 | 14.1 | - | 5 / 3 | **Looks fast, but the cubes piled up** (ride 468 -> 230 mm). The numbers alone did not show it |
| **pt5 A** (20:00) | 33 / 43 | 13.3 | 23.6 | 22.1 | 15 / 18 | |
| **pt5 B** (PRISM_friend 21:07) | 30 / 30 | 11.8 | 20.2 | 17.9 | 16 / 14 | Same code as A |
| **pt5 + rest probe C** (21:30) | 44 / 49 | 11.0 | 16.2 | 15.6 | 23 / 21 | |

### 5.2 No pile-up: the measured belt length and the camera's view of the feeder exit

| Run | Belt length feeder -> pick point | Cubes that came out unseen (picked as they came) | Cubes the camera saw at the exit (last 10) |
|---|---|---|---|
| pt4 18:17 | 468 -> **230 mm** (fell during the run) | not handled | - |
| pt5 A | 465 mm | 0 | 10 of 10 |
| pt5 B | 460 mm | 2 | 8 of 10 |
| pt5 C | 462 mm | 2 | 9 of 10 |

**Why pt4 piled up:** cubes left on the belt were credited to new pushes, so the measured length shrank. Pushes the camera
did not see were called "empty" and pushed again onto a cube that had come out.
**What pt5 does:** no push until the belt has moved one cube plus one gap (60 mm or more) since the last push; the camera
may stop a push but never allow one alone; a push the camera missed counts as delivered; a ride only counts if it is 0.8-1.3
times the real one; the colour of the cube *at the pick point* decides the bin (orange left, everything else right).

### 5.3 Sorting

| Run | Cubes judged | Right bin | Wrong bin | No colour seen (went right) |
|---|---|---|---|---|
| A | 33 | 30 | 0 | 2 |
| B | 32 | 27 | 0 | 3 |
| C | 45 | 37 | 0 | 7 |

### 5.4 Energy and power (recorded time only)

| Run | Arm power | Belt power | Energy in picked cycles | Wasted in failed cycles | Between cycles |
|---|---|---|---|---|---|
| A | 0.53 W | 0.78 W | 731 J | 38 J | 10 J |
| B | 0.44 W | 0.36 W | 537 J | 0 J | 68 J |
| C | 0.67 W | 0.61 W | 684 J | 27 J | 3 J |

About **95 %** of the recorded energy goes into cycles that pick a cube. The belt uses as much as the arm or more, and how
much depends on how long it runs.

### 5.5 Temperatures and the 12 V rail

| Run | Hottest arm servo | Belt motor | Raspberry Pi | 12 V rail (lowest) |
|---|---|---|---|---|
| A | 40 C | 25 C | not logged | 12.07 V |
| B | 42 C | 25 C | not logged | 12.04 V |
| C | 41 C | 26 C | 46 C | 11.93 V |

Limits: arm servo warm 55 C / rest 65 C, belt 50 / 57 C, Pi 70 / 80 C, rail pushes wait below 11.5 V. Nothing came near a limit,
and the rail never collapsed during GRIP (the earlier pt2 problem). The feeder servo has no temperature sensor.

### 5.6 Feeding and the two copies' separate learning

The optimizer saves what it learned (`learned_state.json`). These figures are **cumulative over every run that copy has
learned from** (PRISM: all of today's runs; PRISM_friend: pt5's state at 20:05 plus run B), not one run.

| | PRISM | PRISM_friend |
|---|---|---|
| Pushes that delivered a cube during GRIP | 134 of 138 (97 %) | 108 of 113 (96 %) |
| Pushes during TRANSPORT | 24 of 60 | 20 of 47 (these include pushes into an empty feeder) |
| Push length learned | 800 ms (no longer push needed) | 800 ms |
| Gap between cubes in effect at the end | **60 mm** (4 cubes fit on the belt) | **100 mm** (3 cubes fit) |

Both copies started from the same learned state and ended with a different gap. The speed of the runs (B 11.8 s, C 11.0 s)
is too close to say which gap is better. That needs repeats.

## 6. What the runs say about restarting after a refill

- **Run A (before the rest probe):** the cell rested with an empty feeder six times, 1 to 5 minutes each, about 22 of 32
  minutes in total. It restarted only by trying a full feed after 60 s, 383 s and 705 s of rest (heavy probes: a belt run to
  the timeout, 10 cycles without a cube).
- **Run C (with the rest probe):** at 21:33:01 the feeder emptied; at 21:33:20 the feeder IR saw cubes and PRISM made one
  push with the arm parked; the camera saw a new cube at the exit and PRISM ran again at **21:33:22, 21 seconds after it
  started resting**. One real event, so this is a first sign, not a measurement of the average.
- In the simulator the rest probe restarts in 15 s (feeder IR working) or 32 s (feeder IR not working).

## 7. Corrections to earlier statements

1. The information book's pt5 row (21 cubes, 13.3 s, 22.7 J) was written while run A was still going. The finished run is
   **33 cubes, 13.3 s, 23.6 J**. "pt5 is 14 % slower and 23 % more costly than pt4" is **not supported**: runs B and C of the
   same code are as fast and as cheap as pt4, and the spread between pt5 runs is as large as the difference.
2. The feeder's push-rate limit (12 pushes a minute, then a 45 s rest) **would not have triggered in any run**: the busiest
   minute in every run was **6 pushes**. Run C averaged 5.0 pushes a minute and the feeder still stopped. The limit is too
   loose to protect the servo. What can help today is the relaxed servo between pushes (not tested on the cell yet);
   what would settle it is a temperature sensor on the feeder servo.
3. The 20:00 run's "57.9 s per cube over the whole run" in its report counts about 22 minutes of waiting. Use the median
   cycle time (13.3 s) for speed.

## 8. The feeder servo alarm in run C

- 12 of its 50 pushes showed no current on the 12 V rail (run A: 1 of 50, run B: 3 of 30).
- 21:39:39 first alarm "the feeder servo does not move" (3 pushes in a row); more alarms at 21:40:36, 21:40:56 and 21:41:10.
- Average 5.0 pushes a minute, at most 6 in a minute. The blade was held at its back angle between pushes.
- The operator reported the feeder servo very hot at that time. **Likely heat, not proven.** A dead plug or the 5 V supply
  would also fit; the alarm text points to both.

## 9. Built after pt5 and what is proven

| Change | Simulator | On the real cell |
|---|---|---|
| Light probe while resting with an empty feeder | passes | **worked once** (run C, 21 s) |
| Report after every run (`run_reports/`) | passes | **yes** (the source of this file) |
| Calibrate page and cube-shape filter | - | **yes** (0 wrong bins in all pt5 runs) |
| Machine page (Pi, programs, hardware) | - | yes |
| Feeder presets fd1 (170 / 10) and fd2 (165 / 15), feeder test asks which | passes | **not tried** |
| Feeder servo relaxed between pushes + push-rate rest | passes | **not tried** |
| Heat limits for every part (servos, belt, Pi, servo strain) | passes | not triggered |
| Overview "Limits" zone (each limit and why the cell rests) | data test passes; page drawn with test data | added; not yet seen on a live cell |
| Emergency grip with the hand camera, gentle current-feel close for any cube position | passes (3 cube positions, stiff gripper, no cube, no camera) | **not tried**; the Pi did not see the wrist camera; the current numbers are a first guess |
| Emergency pictures (date and time on each) and the Emergency block on the Overview | passes | **not tried** (needs an emergency to happen) |
| Sleep with the power relay (GPIO 19, header pin 35) | passes with a fake relay | **not tried**; the relay is not taught yet and was never switched |

Simulator results (same real belt length): filling the belt cut the time per cube from 10.9 s to 3.6 s (67 % less); with
2 cubes left on the belt and a camera blind to every second cube, 12 of 12 cubes were picked and every push landed at
least 85 mm behind the last cube.

## 10. What to run next

1. **fd1 against fd2** (feeder 170 / 10 against 165 / 15): 15 minutes each, at least 3 repeats, the same cubes. Compare
   `run_reports/runs.csv`. Start with `feeder_test.py` (it asks which preset).
2. **A feeder servo temperature sensor** (a DS18B20 taped to the servo), then set a real limit instead of the push count.
3. **Rest probe again:** let the feeder run empty, wait 5 and 10 minutes, refill, and time how long until the next cube is picked.
4. **Gap 60 mm against 100 mm** (the two copies learned different gaps): 3 runs each.
5. **Wrist camera:** plug it in, check that the Pi sees it, run the emergency grip with a hand on the stop.
6. **Relay:** run `orchestrator/cal/relay_test.py` once (cell stopped, arm resting), then try the sleep.
7. A human spot check of the bins against the real cube colours.

## 11. Where the data is

| What | Where |
|---|---|
| Per-run reports (summary, every servo, verdict) | `PRISM/run_reports/run_<stamp>.md`, `..._devices.csv`, and `runs.csv` (one line per run) |
| Raw samples, cycles, command log of a run | `<folder>/autoencoder/riff_logs/autoen_raw_`, `autoen_cycles_`, `autoen_tx_<stamp>.csv/.log` |
| Optimizer decisions and stories | `<folder>/orchestrator/logs/orch_*.jsonl` |
| Run stamps | A `20261112_200041` (PRISM) - B `20261112_210457` (PRISM_friend) - C `20261112_213017` (PRISM) - pt4 `20261112_142348`, `20261112_181716` - pt1 `20261112_010644` |
| The energy method as a script | `orchestrator/cal/run_report.py` and `orchestrator/cal/energy_from_raw.py` |
| What each limit is | `orchestrator/limits.py` |
| More | `docs/PRISM_INFORMATION_BOOK.md` (architecture, timing), `docs/PRISM_BEHAVIOUR.md` (how it behaves) |

## 12. Run D - 100 cubes, 2026-11-13 00:25 (added 2026-11-13)

Run `20261113_002452`, PRISM folder, optimizer in FLOW with the rest probe, feeder angles **170 / 10 (fd1)** as sent to the Mega
(`FEEDCFG,170,10,800,300` in `autoen_tx_20261113_002452.log`; autoen's console had printed preset fd2 - unexplained, the
simulator applies fd2 correctly), belt 20 rpm. Report: `run_reports/run_20261113_002452.md`. Written from the run's own logs.

| | Run D | Run C (best earlier) | Run A (first pt5) |
|---|---|---|---|
| Length | 33.9 min | 10.5 min | 31.9 min |
| Cubes OK / cycles | **93 / 102** (9 NO_BLOCK) | 44 / 49 | 33 / 43 |
| Left / right | 48 / 45 | 23 / 21 | 15 / 18 |
| Median s / cube | 13.0 | 11.0 | 13.3 |
| Whole run s / cube | 21.9 | - | - |
| **J / cube (all)** | **22.2** | 16.2 | 23.6 |
| Energy | 2069 J (arm 917, belt 1152) | 714 J | - |
| Average power | arm 0.52 W, belt 0.65 W | 0.67 / 0.61 W | - |
| Hottest arm servo | 42 C (J5, J6) | 41 C | 40 C |
| Belt motor max | 27 C | 26 C | - |
| 12 V rail minimum | 12.03 V | 11.93 V | - |
| Pi max / CPU avg | 43.8 C / 49 % | 46.3 C / 73 % | - |
| Measured belt length | 463-468 mm (no pile-up) | 462 mm | 465 mm |
| Wrong bins (optimizer's check) | 0 | 0 | 0 |
| Emergency grips | 0 | 0 | 0 |

What happened in run D: 16 rest probes with an empty feeder; 6 cubes nobody fed were picked as they came; 6 arm drifts
were corrected. No warning, no over-temperature, no over-current.

**What it shows**
- **Endurance:** about 100 cycles in 34 minutes, no pile-up, no wrong bin, nothing near a limit. This is the longest clean
  pt5-family run.
- **Energy per cube (22.2 J) is in the range of run A (23.6 J), higher than C (16.2 J).** Run D is far longer and includes
  the belt running through the rest probes and the 9 empty cycles; C was short. Do not read this as fd2 being worse or
  better: one run of each setting cannot show that (section 1: identical code varies 13-23 %).
- **fd2 has still not run on the real cell** (run D used the fd1 angles, see above), so fd2 against fd1 is not decided. The optimizer log shows many "feeder servo did not move (12 V rail dipped ~0 mV): giving that push
  up, feeding again" lines (both the 1/2 and 2/2 retries appear). The same status text is repeated in many log snapshots,
  so the number of separate events was **not** counted here; count them from `autoen_tx_20261113_002452.log` before
  comparing with fd1.
- **Energy figure covers 29.4 of the 33.9 minutes** (no samples while autoen was paused, method in section 4).

**Corrections from this run**
- The 23:21 run (`20261112_232142`, 2.8 min, 3 cubes, 67.5 J / cube) is a short test; its rail minimum of 1.83 V in
  `runs.csv` is a measurement dropout, not a real dip. Leave it out of comparisons.
- The pt5 row in `docs/PRISM_INFORMATION_BOOK.md` (21 cubes, 22.7 J) was written mid-run; the final run A numbers are in
  section 5.1 above.

### 12.1 Time spent feeding in run D (from `autoen_tx_20261113_002452.log` and the cycles file)

| | Value |
|---|---|
| Pushes sent / answered (FEED -> FEED_DONE) | 165 / 165 (102 cycles; the rest are extra pushes: rest probes, retries) |
| Time per push | median 2.13 s (min 2.12, max 2.54) = ready wait 1.0 s + push 0.8 s + return 0.3 s |
| All pushes added up | 355.7 s = 5.9 min = 17.5 % of the 33.9-minute run (most of it overlaps the arm's work) |
| Waiting for the belt to bring the cube (TRANSPORT) | median 3.3 s per cycle, 830 s in total = 41 % of the run (max 47 s) |
| Grip / carry | 1.6 s / 3.4 s median per cycle (150 s / 332 s in total) |
| Cycle | median 13.1 s, 1753 s in total |

The cycles file's `feed_s` column is 0 for run D: in flow mode the optimizer starts the feed, so the push is not a phase
of the cycle. The numbers above come from the command log.

### 12.2 Restart after the feeder was empty and filled again - run D (8 episodes, from the command log)

"Refill detected" = the first rest-probe push of an episode (it is made when the feeder IR sees cubes again).

| Refill detected | Probe pushes needed | Next cycle started after | Cube picked and placed after |
|---|---|---|---|
| 00:27:15 | 1 | 1.7 s | 29.3 s |
| 00:29:50 | 2 | 26.9 s | 56.0 s |
| 00:32:15 | 2 | 27.0 s | 51.2 s |
| 00:35:05 | 4 | 76.6 s | 100.8 s |
| 00:37:09 | 1 | 1.8 s | 14.0 s |
| 00:38:29 | 1 | 1.9 s | 25.6 s |
| 00:54:25 | 2 | 26.1 s | 49.6 s |
| 00:56:59 | 3 | 52.1 s | 77.1 s |
| **Median** | | **26.5 s** | **50.4 s** (14.0 - 100.8 s) |

Reading: when the first probe push puts a cube at the exit, the arm is moving again within 2 s and the cube is placed after
14-29 s. When the first push brings nothing (cubes not yet settled at the exit), the probe repeats every 20 s, and each
repeat adds about 25 s: 2 pushes = about 27 s, 3 = 52 s, 4 = 77 s before the next cycle starts.
Limits: the log does not record when the cubes were physically put in, so the times start when the feeder IR saw them,
not when the hand left the feeder. Eight episodes of one run are a first measurement, not an average. Before the rest probe
(run A) the same restart took 60 s - 12 min (section 6).

## 13. 200cubes - the 200-cube run of PRISM_mek (2026-11-13, added)

199 cubes of 200 cycles (91 left / 108 right, 1 GRIP_SUSPECT) in 39.9 minutes, **median 11.4 s per cube** (12.0 s over the whole
run), feeder fd1 (170 / 10), belt 20 rpm, hottest servo 42 C, no warnings. 203 pushes of 2.13 s; waiting for the belt only
354 s (15 %) against 830 s (41 %) in run D. **Energy not measured:** the belt current and the 12 V rail were not logged in this run
(the INA260 was silent), so the report's 9.1 J per cube is the arm alone. Full table: `docs/200cubes.md`.
