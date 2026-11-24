# PRISM optimizer: what we still need to do

Written 2026-10-25 22:30, updated 23:05 (optimizer v1 code written + tested on the simulator, `--resting` added to autoen). The data behind it is in `cal/OPTIMIZER_DATA.md`, `cal/camera_eta_summary.json` and `cal/COLLECTION_STATUS.md`.
**Goal:** run efficiently: as many cubes per minute as possible at the lowest energy per cube, and cut wasted energy when idle.

---

## 0. Where we stand

| Piece | Status |
|---|---|
| `observe.py`: MQTT → WorldState | ✅ done (also watches `autoen/cycle`) |
| `predict.py`: per-cube 1-D Kalman, belt-speed prior | ✅ done (MM_PER_PX 0.95, BELT_MM_S_PER_RPM 2.81: **do not touch**) |
| `model.py`: cycle-time fit + success table | ✅ `model.json` saved 22:46 from `exp_20261025_005038` only (**provisional, pre-repair**: refit after A1) |
| `optimize.py` | ✅ v1 written 23:00, `test/test_opt.py` passes |
| `actuate.py` | ✅ written 23:00, `--shadow` supported |
| `main.py` | ✅ real loop (backup `main.py.before-optimize`); end-to-end on the simulator: `test/test_orch_sim.py` passes |
| Belt power vs rpm | ✅ `cal/baseline_belt_summary.json` |
| Arm hold power (home / rest torque on) | ✅ 0.084 W / ~0.30 W |
| Rest ↔ home transition | ✅ ~73 s, ~20.5 J round trip (B2 stopped after 1 round: enough) |
| Rest with torque off (B1c) | ⏭ skipped: Mega firmware does not answer `Q` (needs reflash). Not needed: rest saves nothing anyway |
| Cycle time + J/cube grid | ✅ `exp_20261025_005038` but **before the arm repair + gripper re-cal**: must re-check |
| Belt energy per cube | ✅ ≈ 10–11 J, belt runs only ~50–70 % of a cycle |
| IR position on image / ETA error | 🟡 provisional from old runs: u_IR 247 px, ±0.5 s at 5 s |
| `cal/baselines_summary.json` | 🟡 written 22:50, **provisional** (pre-repair data, coarse arm W) |
| autoen `--resting` | ✅ 23:02 (backup `auto_en_new.py.before-resting`): start with the arm on the cradle, nothing moves until the wake task; `test/test_resting_sim.py` passes |
| Arm INA260 + vision frame-time logging | ✅ patched 2026-10-25 22:15 (backups `.before-armina`, `.before-frametime`), needs autoen + vision restart |

**Decisions already made from the data**
- Idle rule **O2 = hold home + brake the belt.** Rest costs more than home (0.30 vs 0.084 W), a rest round trip costs 20.5 J / 73 s, and the camera only sees a cube ~13 s ahead (wake needs ~36 s). Rest = parking/shutdown only.
- No relay power cut (the relay coil would cost about what it saves).
- Vision stays at **6 fps** (15 fps = 87 % of a core, +15 °C, no better ETA).
- Faster is also more efficient up to **20 rpm / arm 1.5**. Arm 2.0 caused transport timeouts.

---

## 1. Phase A: last data (machine needed, operator starts every move)

### A1. Step 3 run: `plans/vision_check_v1.json` ✅ DONE 23:34: `exp_20261025_232408`, 24/24 OK
20 rpm / arm 1.5 then 15 rpm / arm 1.5, 12 cycles each, vision ON, about 12 min.

**Safety order: the arm is on the cradle and autoen is NOT running** (checked 22:40). A plain autoen drives straight to home on Start, which crashes into the frame. **Never start it without `--resting` while the arm is on the cradle.**
1. Start autoen with `--resting`: `cd ~/Desktop/PRISM/autoencoder && ~/autoen-env/bin/python auto_en_new.py --resting` (plus your usual options; it is `auto_en_new.py`, not `autoen_node.py`). It also loads the arm-INA260 logging. Before Start, check it says resting: `curl -s http://127.0.0.1:8781/api/state | grep -o '"rest_state": "[a-z_]*"'`.
2. Start (nothing moves, autoen pauses):
   `curl -s -X POST http://127.0.0.1:8781/api/start -H 'Content-Type: application/json' -d '{"clear":true,"config":{"pause_before_set":1,"sets":1000,"enable_b":0,"enable_c":0,"cycles_per_set":12}}'`
3. Wake (taught wake sequence, ~36 s, ends at home):
   `curl -s -X POST http://127.0.0.1:8781/api/task -H 'Content-Type: application/json' -d '{"kind":"wake","seconds":5,"settle_s":0}'`
4. Resume: `curl -s -X POST http://127.0.0.1:8781/api/pause`. autoen goes on to the set boundary and pauses; experiment.py reuses it.
5. Restart vision normally at 6 fps (loads the frame-time logging).
6. Run:
   `python experiment.py run --plan plans/vision_check_v1.json --cubes "white 40mm" --note "vision_check_v1 grip_pos 176.8 vision on"`
7. Block 1: watch for grip misses (gripper re-cal at 18:20). If one happens, stop.

**Gives:** post-repair cycle time + success, real arm W (INA260), u_IR and ETA error at real rpm, and a yes/no on the 3 s vision lag.

### A2. Gap run (step D): optional for v1, needed for v2
Cubes in bursts with 10–60 s gaps at 20 rpm / arm 1.5, to see real idle behaviour and what holding home + braking costs between bursts.

---

## 2. Phase B: analysis (no machine)

- [x] **B1. Check the A1 run** (23:40: 24/24 OK, 0 grip misses / timeouts / faults, joints ≤ 39 °C, belt 26.5 °C): grip misses, timeouts, IR stuck, currents, temperatures.
- [x] **B2. Refit the cycle model** (23:35: drift +1.3 % / +2.9 %, grid still valid; `model.json` = grid + A1: `8.70 + 2.87/arm + 158.9/rpm`) on the A1 data plus the grid (`python3 model.py fit --runs ...`). Compare with `8.54 + 2.92/arm + 159.9/rpm`. If 20/1.5 and 15/1.5 drift > 10 %, the grid is stale for the new gripper. Save `model.json`.
- [x] **B3. Arm energy: closed 2026-10-26.** There is no arm INA260 by design; the servo current sum is the arm power method (slight underestimate: misses servo electronics). compare the arm INA260 with the servo-current sum. If they differ, re-scale arm J/cube and P_home (the current sum misses servo electronics).
- [~] **B4. Camera** (23:40: lag ≈ 0.2 s, not 3 s; u_IR ≈ 245 px; **but the cube was detected in only 4 of 24 cycles**, and cubes move toward smaller u while the camera says dir = 1. The optimizer ignores the camera until it is reliable. ETA error not measurable with 4 tracks): re-run `cal/eta_check.py` on the A1 session (u_IR from half, ETA test on the other half), with the lag from `ts_got − ts_frame` and `ts − ts_frame`.
- [~] **B5. Write `cal/baselines_summary.json`** (updated 23:45 with A1; still missing: real arm W (B3), ETA error (B4)) (step 4 of the spec): P_belt(rpm), P_arm home, P_arm rest (on; off = not measured), E/t home→rest→home, u_IR, ETA error, T_breakeven, O2 rule, and the source of every number.
- [ ] **B6. Update `cal/COLLECTION_STATUS.md`** and `cal/OPTIMIZER_DATA.md` §3/§4/§7 (B1c skipped, B2 stopped at 1 round, C provisional, INA260 now logged).

---

## 3. Phase C: build optimizer v1 (rule-based, no new packages)

v1 = pick the best **(rpm, arm_speed)** from `model.json` + the energy model, and apply O2 when idle. MPC comes later (Phase E).

Status 23:05: **C1–C3 done, C4 done except the replay test** (no new-unit session with vision yet: needs A1).

### C1. `optimize.py`
- **Inputs:** WorldState + PredictOutput (cubes seen, ETAs, belt state), `model.json`, energy constants from `baselines_summary.json`.
- **Score** for every tested config (only configs with success `p_low` ≥ a floor, e.g. 0.8):
  - `t = a + b/arm + c/rpm` (s/cube)
  - `E = E_arm(arm, t) + P_belt(rpm) × 515 mm ÷ (2.81·rpm)` (J/cube)
  - pick by a weight `w`: maximise throughput subject to J/cube ≤ limit, or minimise `E + λ·t`. Default = 20 rpm / arm 1.5.
- **Modes:**
  - `RUN`: cubes coming → chosen config.
  - `IDLE` (no cube seen and none expected for > T_idle): **hold home + X,BRAKE**.
  - `PARK` (long idle / end of shift, operator or schedule): rest. Wake from a schedule/feeder signal, never from the camera ETA.
- **Guards:** no change mid-cycle (only at a cycle boundary), hysteresis (no flip-flopping), never outside the tested grid (8–20 rpm, arm 1.0–2.0), fall back to 20/1.5 if the model is missing.
- **Output:** a Plan (target config, mode, reason), logged every time it changes.

### C2. `actuate.py`
- Turns a Plan into autoen commands through its HTTP API (`/api/config`, `/api/task`, pause/resume). It never opens the serial port itself.
- **To check first:** which `/api/config` keys autoen accepts while running vs only while paused (experiment.py only changes config while paused at a set boundary). If live changes are not allowed, use `pause_before_set` + one set per decision, like experiment.py.
- Publishes `orchestrator/plan` and `orchestrator/status` on MQTT for the dashboard/logger.
- **Dry/shadow switch:** `--shadow` = compute and log plans, send nothing.

### C3. `main.py`
Wire observe → predict → optimize → actuate at 20 Hz (the sketch already has this shape). Add `--shadow`, logging of every plan, and a clean stop.

### C4. Tests (no hardware)
- Unit tests for the scoring and the mode switching (fake WorldState).
- Replay a recorded session (`cal/replay.py`) through observe/predict/optimize in shadow mode: do the decisions make sense?
- Run against the simulator (`dashboard/sim_cell.py` / test FakeMega in `cal/test_experiment_sim.py`) end to end.

---

## 4. Phase D: put it on the machine (operator present)

How to run it (orch-env, from `orchestrator/`, autoen already started and awake, **not** under experiment.py):
- Shadow: `python3 main.py --shadow`: logs to `logs/orch_*.jsonl`, publishes `orchestrator/plan` + `orchestrator/status`, sends nothing.
- Live: `python3 main.py` (Ctrl+C leaves autoen as it is). Operator commands: `mosquitto_pub -t orchestrator/cmd -m resume` (or `park` / `wake`).
- What v1 does: picks 20/1.5 (throughput) or 15/1.5 (`--objective energy`) and sets `arrive_timeout` to match the rpm. It lowers a config's rating when it fails on the live machine. When the feeder runs dry it pauses, holds home and brakes the belt: after 1 empty cycle with the camera on, after 2 with the camera off (autoen would stop at 3). Any grip miss or fault → ALERT (paused, waits for `resume`). An operator pause → MANUAL (hands off). It never parks or wakes by itself.

1. **Shadow run:** autoen runs a normal plan, the optimizer only logs what it *would* do. Check the plans by eye.
   ✅ 2026-10-26 12:31 (`logs/orch_20261026_123101.jsonl`): 13/13 OK at 20/1.5, camera trusted after 5 cubes (10/10, orange + white, after the vision fix: orange re-measured, `--belt-dir -1`, velocity sign checked 176/176). First empty feed: pause decided 6 s into TRANSPORT, IDLE at the end of that cycle. Live it would have saved 2 empty cycles (~2 x 26 s of belt) and autoen would not have stopped with an error after 3 failures.
2. **Live, short:** optimizer in control for 1 burst (10 cubes), operator watching, Ctrl+C ready.
3. **Comparison run:** same cube feed, (a) fixed 15/1.5, (b) fixed 20/1.5, (c) optimizer. Compare cubes/min and J/cube (now with the arm INA260).
4. With gaps (needs A2): show the idle savings of hold-home + brake.
   ✅ 2026-10-26 12:59–13:12, optimizer LIVE (`cal/runs/gap_20261026_1258.jsonl`, `logs/orch_20261026_124955.jsonl`): 4 bursts (2+5+5+6 = 18 cubes, 18 OK), 48–92 s gaps, resume by operator command. Bursts: **3.12 cubes/min, 21.9 J/cube** (= fixed 20/1.5). Every burst ended after exactly ONE empty cycle (~49.5 J, of which belt ~35 J) → paused, home, braked. Idle: **arm 0.09–0.17 W, belt 0.0005 W**. autoen alone would run 3 empty cycles (~150 J) and stop with an error.
   **v1 pass check:** throughput ✅ (3.12 vs fixed 3.06–3.14), J/cube ✅ (same as fixed 20/1.5 within noise), grip misses ✅ 0, idle ≈ P_home + 0 W belt ✅.
   **Next saving found:** the one empty cycle still costs ~50 J (≈ 2 cubes' worth) because the belt runs the full arrive_timeout (25 s) after the camera already knows (at 5 s). Options: a tighter arrive_timeout when the camera is trusted (feed→IR measured 8.9–10.1 s at 20 rpm), or an autoen 'abort transport' command (autoen change, needs OK).
   ✅ Built 2026-10-26 13:25 (option B): autoen `POST /api/abort_transport` (backup `auto_en_new.py.before-abort`; brakes the belt, cycle result `NO_BLOCK`, serial cycle only); the optimizer sends it when the TRUSTED camera sees no cube 5 s into TRANSPORT (backups `optimize.py/actuate.py.before-abort`). Simulator: empty cycle 2.7 s instead of 25 s (`test/test_abort_sim.py`); all other tests unchanged.
   ✅ **Machine 2026-10-26 13:33** (`logs/orch_20261026_133258.jsonl`, `cal/runs/gap_20261026_1258.jsonl`): 7/7 OK at 3.24 cubes/min, 22.0 J/cube; the empty feed ended `NO_BLOCK` after 6.1 s. **Empty feed → idle: 12.7 s and 11.0 J (was 32.6 s and 49.5 J, −78 %).** Idle: arm 0.082 W, belt 0.0003 W. Belt INA260 back on SDA 20 / SCL 21 (12.21 V).

**v1 passes if:** throughput ≥ the best fixed config, J/cube ≤ fixed 20/1.5, zero new grip misses or IR stalls caused by a config change, and idle power between bursts ≈ P_home + 0 W belt.

---

**Fix 2026-10-26 14:50: lost Mega replies.** ~1 in a few hundred belt commands got no reply within 1 s (9 cycles 10-24..26, e.g. `no reply to X,CONV,20.00` at 14:43, Mega telemetry without gaps = the reply was lost, the Mega did not stall). autoen's `belt_cmd` now sends the command again (up to 3 tries), except `FEED` (a second FEED could push two blocks). Backup `auto_en_new.py.before-replyretry`; test `test/test_reply_retry_sim.py`.

## 5. Phase E: later (v2+, from the original 5-layer design)

- Linear MPC + OSQP for the speed plan (install `osqp` into orch-env, keep v1 as the fallback).
- Demand forecast (Prophet / Holt-Winters) from the vision demand CSVs → plan PARK/wake ahead of time.
  ✅ **v1 built 2026-10-26 14:15 instead (no new packages):** `forecast.py` learns the refill rate per hour of the week from every idle period (state in `forecast_state.json`), and `main.py --auto-resume` PROBES the feeder when a refill is likely (one FEED; an empty one is aborted by the trusted camera, ~11 J), backing off on long idles (30 s … 30 min). Probes only with a trusted camera. autoen: `NO_BLOCK` no longer counts toward max_fails (backup `auto_en_new.py.before-noblockfail`); otherwise 3 empty probes would have stopped the run. Tests: `test/test_forecast.py`, `test/test_auto_sim.py`. Needs a machine check; the forecast needs days of use before it knows your pattern.
- Health layer: Dense Autoencoder (train on the laptop, infer on the Pi); for now fixed current/temp thresholds.
- Fix the `Q` firmware (reflash) if torque-off rest is ever wanted.
- Make autoen remember `rest_state` across restarts, or refuse Start when the arm is not near home (removes the crash risk in A1 step 1).
  ✅ **Done 2026-10-26 15:20:** autoen saves `rest_state` to `autoencoder/rest_state.json` on every change and loads it at startup (`--awake` overrides; the simulator never uses the real file). `start_cell.sh` reads it and wakes a resting arm along the taught path. Backup `auto_en_new.py.before-restmemory`; test `test/test_rest_memory_sim.py`.

## 7. Self-running cell (2026-10-26)

- `./start_cell.sh` / `./stop_cell.sh` (PRISM root): vision → autoen → optimizer (`--auto-resume`) → one confirmed Start. Logs in `run_logs/`.
- Memory across restarts: `autoencoder/rest_state.json` (arm), `orchestrator/forecast_state.json` (refill pattern), `orchestrator/learned_state.json` (live success counts). Camera trust is re-earned every run on purpose.
- Energy metered live (`meter.py`, belt INA260 + servo current; matches the offline scripts within 0.1 %), logged every minute by mode (RUN / IDLE / PROBE / ...). `python3 report.py` = today's report (saved in `reports/`), `--week` = last 7 days.
- Boot: `autostart_cell.sh` + `prism-cell.desktop` (PRISM root). Install: `cp ~/Desktop/PRISM/prism-cell.desktop ~/.config/autostart/` (the operator's choice; remove that file to turn it off). The first Start after power-up is always confirmed by a person.

---

## 6. Rules that stay in force

- The operator starts every hardware move; Claude prepares commands and waits for "ran".
- No deleting files; back up as `<file>.before-<reason>`; show the diff and wait for OK before changing calibration. MM_PER_PX and BELT_MM_S_PER_RPM are not to be touched.
- New sessions only, `rpm_units: output_rpm_cpr11880` in every meta, never mixed with old sessions.
- Stop and propose a fix on any abnormality (high current, over-temperature, grip miss, cube stuck at the IR).
- Optimizer code (`optimize.py`, `actuate.py`) is written only when the operator says so.

## 8. Smart optimizer (2026-10-26 16:40)

Default objective `priority`: **1 safety** (proven settings only; servo ≥ 55 °C or belt ≥ 50 °C → gentler settings,
≥ 65 / 57 °C → mode COOL = pause at home until 5 °C below the warm limit) → **2 energy** (settings within 5 % of the
lowest J per good cube; the setting in use keeps a 10 % band) → **3 speed** (fastest of those).
J per cube is measured live per setting (meter.py) and compared relative to the others (`machine_scale`: a machine-wide
offset scales the whole grid), so only a setting that is worse than the others loses. Every 40 good cycles a safe,
low-energy setting with < 15 live samples is tried for 4 cycles. No cube 3 s after FEED → `refeed` (abort without pause,
autoen feeds again), then still nothing → abort + IDLE. Live J sums are saved in `learned_state.json`.
Backups `optimize.py / actuate.py / main.py .before-smart`; test `test/test_smart.py`. Needs a machine run.
