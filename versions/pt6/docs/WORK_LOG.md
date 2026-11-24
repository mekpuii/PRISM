# PRISM work log

What was done, newest entry first. One entry per piece of work.

---

## 2026-11-12 ~20:50-21:05 - a copy of pt5 for a friend: `~/Desktop/PRISM_friend`

**Asked:** a copy of pt5 so a friend can experiment, as a separate folder on this Pi.

**Done**
- Copied `versions/pt5/` (saved 2026-11-12 ~20:05, code and settings only, 4.5 MB) to `~/Desktop/PRISM_friend`.
  `~/Desktop/PRISM` was not changed.
- In the copy, every `Desktop/PRISM` path was changed to `Desktop/PRISM_friend` (`start_cell.sh`, `stop_cell.sh`,
  `autostart_cell.sh`, `prism-cell.desktop`, `vision/snapshot.py`, the `orchestrator/cal/` scripts, the docs), so the
  copy runs its own code and writes its own logs.
- Made the empty folders the programs write to: `run_logs/`, `orchestrator/logs/`, `orchestrator/cal/runs/`,
  `autoencoder/riff_logs/`.
- Added a check to the copy's `start_cell.sh`: it refuses to start while cell programs from another PRISM folder are
  running (one robot, the same ports and MQTT names; without the check it would reuse the other folder's programs).
- Wrote `PRISM_friend/README.md` for the friend: how to run, only one PRISM at a time, what is shared, how to go back
  to a clean pt5.

**Checked**
- The three scripts in the copy pass a syntax check (`bash -n`). No path to the original folder is left in the copy.
- The full cell was NOT started from the copy (no robot moves), so the copy is not yet proven on the machine.

**State at 21:05**
- The operator started the copy's dashboard by hand; it runs from `PRISM_friend/dashboard` on http://127.0.0.1:8080:

      cd ~/Desktop/PRISM_friend/dashboard
      ~/db-env/bin/python dashboard_node.py --data-dir data --log-dir ~/Desktop/PRISM_friend/autoencoder/riff_logs

- Nothing else is running (no vision, autoen, optimizer, loggers).

**Keep in mind**
- Shared with the original: the Python environments (`~/autoen-env`, `~/orch-env`, `~/prism-env`, `~/db-env`, ...),
  the Mega sketch on the board, the MQTT broker, ports 8080 and 8781.
- Before starting the original again: `cd ~/Desktop/PRISM_friend && ./stop_cell.sh --all` (the original's
  `start_cell.sh` has no such check and would keep the copy's dashboard as "already running").
- The copy is pt5 as saved. The live code in `~/Desktop/PRISM` has changed since (`optimize.py`, `actuate.py`,
  `auto_en_new.py`, `overview.py`, two new tests); those changes are not in the copy.
- The copy does not start at login; the autostart still points to the original.
