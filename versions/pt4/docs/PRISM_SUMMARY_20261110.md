# PRISM summary (2026-11-10)

Where PRISM stands before the break: what works, what still needs fixing, what can be improved and what can be developed next.

## 1. What PRISM is now

The main loop is **camera + IR → predict → optimizer decides → actuate sends to autoen → robot, belt and feeder**.

| Part | What it does now |
|---|---|
| Optimizer (`orchestrator/optimize.py`) | **Goal:** speed first, energy second. It scores each setting by how often it works, so one lucky cycle doesn't win. **Trying new settings:** goes beyond the old limits (up to belt 30 rpm, arm 3.0); a setting that fails is banned and the ban is saved in `learned_state.json`. **Time model:** learns how long a cycle takes (t = a + b/arm + c/rpm) while it runs. **Run control:** takes over the run (`own_run`), so Start on the web page no longer stops it after 12 cycles. |
| Incidents (`orchestrator/incidents.py`) | When a grip is missed, a cube is stuck or the feed is empty, it tries a fix by itself and learns which fixes work. If the USB fails (SerialException, I/O error) it stops and alerts you instead of retrying. |
| Feeder check | Watches the 12 V dip while feeding. A push that moves the servo gives a 33–125 mV dip; a dead push gives 6 mV or less. After a dead push it feeds again, then alerts. |
| Feeder IR | A reading of "cubes there" is only trusted after the next feed confirms it. Feeding resumes only after the IR has read empty and then cubes. |
| Camera cases (`orchestrator/scene.py`) | Judges the light (ok, dim, dark, glare), stuck cubes, cubes waiting, cubes touching and belt slip. Writes one row per cycle to `orchestrator/cases/`; `study_cases.py` shows which situations fail. |
| Stability | Resumes on its own. Handles a new run started from the web page. A leftover optimizer from an earlier run is killed when autoen starts. It no longer stays at home after you press Start. |
| Hand camera | Removed completely, ready to start again from zero. The old files are in `~/PRISM_handcam_removed_20261110/` (delete that folder if you don't need them). |
| Tests | All 21 test suites pass. |

## 2. What still needs fixing

### First, before anything else
None of the fixes have run on the real cell yet. They are only in the code and tests. Restart the cell so it loads them:
```
cd ~/Desktop/PRISM
./stop_cell.sh && ./start_cell.sh
```
Then run one full job and check that it:
- does not pause after one pick;
- does not stop early;
- resumes by itself;
- feeds after every place.

### Hardware (code cannot fix these)
1. **Feeder servo sometimes doesn't move.** I measured no voltage dip on those feeds. Check the D12 plug and the 5 V supply to the servo. This is the most likely cause of "the feeder only works at the start."
2. **Mega USB cable.** There were I/O errors (Errno 5) in the logs. Use a short, good cable, ideally with ferrite cores, plugged straight into the Pi.
3. **Gripper servo cable:** check it is firm.
4. **Feeder IR:** make sure it is mounted firmly and pointing at the right spot. It used to read "cubes" all the time.

### Waiting for your OK
- **Grip as far as it can (grip-seek):** the change is written but **not applied**. It is in
  `/tmp/claude-1000/-home-mekkhram-Desktop-PRISM/5698815f-d2e2-43a8-b368-28a0482112c4/scratchpad/gripmax/grip_seek.diff`
  That folder is temporary. Ask for it to be copied into the project if you want to keep it.

## 3. What can be improved

1. **Collect real data first.** The optimizer, incident learning and case log get smarter with more cycles. Run a few long jobs, then run:
   ```
   cd ~/Desktop/PRISM/orchestrator && python3 study_cases.py
   ```
   to see which situations fail.
2. **Measure the energy instead of estimating it.** "Energy second" currently uses an estimate. A current sensor (for example an INA219) on the 12 V rail would make it real.
3. **Find the true speed limits** for belt rpm and arm speed by watching which settings the optimizer bans.
4. **Feeder:** if the problem is not the cable, a stronger servo, or a feeder with a sensor that confirms each push.
5. **Lighting:** a fixed lamp, or a cover over the belt, so "dim / glare" happens less and the colours are more reliable.
6. **Dashboard:** one page showing the live decision, the reason, the success rate and the energy, so you can see why it changed a setting.

## 4. What can be developed (new)

1. **Hand camera, from zero.** Bring:
   - the block size (length × width × height in mm);
   - where the camera is mounted (on the gripper, and pointing which way);
   - whether a small marker can be stuck on the block (it makes distance measuring much easier).

   Steps: first just measure distance, then add detection, then make it part of the decisions.
2. **Learning from the case log:** once there are a few hundred rows, a small model (not deep learning) can predict "this situation will fail, so change the setting before the cycle starts."
3. **Grip rescue:** using the hand camera, if a cube has rotated or stood up, grab it again from another angle.
4. **Running long jobs alone:** messages to your phone on alerts, and an automatic daily report.

## Next time, in order

1. Fix the hardware: feeder servo plug and 5 V, then the USB cable.
2. `./stop_cell.sh && ./start_cell.sh`, then one full run, and send the logs (`run_logs/` and `orchestrator/logs/`).
3. Say OK (or no) to the grip-seek change.
4. Bring the block size and camera position to restart the hand camera.

## Rules that stay in force

- The operator starts every hardware move.
- No deleting files. Back up first as `<file>.before-<reason>`.
- Show the diff and wait for OK before changing calibration.
- `MM_PER_PX` and `BELT_MM_S_PER_RPM` are never touched.
- New sessions only, with `rpm_units: output_rpm_cpr11880` in every meta.
- Stop and propose a fix on any abnormality.
