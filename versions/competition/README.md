# PRISM - COMPETITION VERSION (saved 2026-11-13 ~21:30)

The exact code, settings and learning the operator chose for the competition ("this is it, this is the version I go with").
A frozen copy: nothing here is ever run or changed. To check that the live folder still matches it:
`cd ~/Desktop/PRISM/versions/competition && md5sum -c --quiet MANIFEST.md5` is for this copy; compare a live file with `cmp`.

**What it is:** pt5 (no pile-ups, orange -> left / the rest -> right, the optimizer plans every push) with:
- the learning of the 200-cube run (`orchestrator/learned_state.json`: tuner gap 60 mm / late 1.25, push 1050 ms, 214 cycles on
  22 rpm / arm 1.75 with 213 OK), feeder preset fd1 active (170 / 10, the 200-cube angles);
- the latest camera calibration and the Calibrate page's **Working area** card (drag a box / click corners, Save);
- the tower lamp (`LampSync` in `autoencoder/auto_en_new.py`: X,STATE only after an X,VER with "lamp=1");
- `stop_cell.sh` that always stops autoen AND the camera (Ctrl+C -> SIGTERM -> SIGKILL, then checks port 8781 and the camera);
- `start_cell.sh` that refuses to start while another PRISM folder is running;
- the dashboard's NaN -> null protection; the feeder tools (`feeder_manual.sh`: p push / u pull back, `feeder_test.py`).
NOT in it (kept in `versions/pt6`): the fault system, heat guard, feeder rest, emergency grip, sleep / relay, Limits zone, run report.

**Run:** `cd ~/Desktop/PRISM && ./stop_cell.sh --all && ./start_cell.sh` -> READY -> Enter. Checklist: `docs/COMPETITION_CHECKLIST.md`.
**Not in this copy (data):** logger databases, run logs, raw run data, old experiments - they are in the full backup
`~/PRISM_competition_backup_20261113.tar.gz` (the whole folder).
