# Competition day checklist (written 2026-11-13, the night before)

## Before you start (5 minutes)
1. Nothing else running: `cd ~/Desktop/PRISM && ./stop_cell.sh --all`
2. Belt EMPTY. Cubes in the feeder. Arm area clear. Hand within reach of the E-stop.
3. Feeder servo: `./feeder_manual.sh` -> choose fd1 -> `p` (push) / `u` (pull back): the blade must move and the 12 V dip line must say "the servo worked". Then `q`.
4. `./start_cell.sh` -> wait for `===== READY =====` -> press Enter (or Start on the dashboard Overview).
5. In the browser: Ctrl+Shift+R on http://127.0.0.1:8080/#/overview. The Start button shows when autoen is "ready".

## What it is set to (the 200-cube setup)
- Feeder fd1 (170 / 10), belt 20-22 rpm and arm 1.5-1.75 chosen by the optimizer from the 200-cube learning, gap 60 mm.
- Fault system ON: camera covered / lost, Mega or arm link lost, feeder servo not moving -> pauses at a safe place, warns,
  tries again, resumes by itself. A hand in the workspace is checked ONLY after you save the empty-workspace picture
  (Calibrate -> Empty workspace picture; belt empty, arm at Standby). Skip it if you are unsure: it is off without the picture.
- Heat guard on every part; sleep/relay OFF (the relay is disabled until `cal/relay_test.py` is run: leave it).

## If something goes wrong
- Dashboard shows "undefined": `Ctrl+Shift+R`; still wrong: `./stop_cell.sh --all && ./start_cell.sh`.
- Cell pauses with mode FAULT: read the reason on the Overview; fix it (uncover the lens, plug the cable); it resumes by itself
  (hand/camera: seconds; feeder: after a retry push draws current). Press Resume on the dashboard to force it.
- Feeder pushes but no cube comes out: look at the magazine (arching/jam), the blade, the servo temperature (no sensor: touch it).
- The cell is mid-run and you must stop NOW: E-stop, then `./stop_cell.sh`.
- Back to the plain automation (no optimizer): `./stop_cell.sh --all && ./automation/run.sh`.

## After each run
- `run_reports/runs.csv` has one line per run; `run_reports/run_<stamp>.md` the temperatures and energy.
- Known gap: the belt's INA260 sensor sent no data in the 200-cube run (energy not measured): check it before the run.
