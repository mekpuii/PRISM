# PRISM - pick, sort and place cell (SO-101 arm, belt, feeder, Raspberry Pi 4)

Cubes are pushed onto a belt by a feeder, ridden to an IR sensor, picked by the arm, and sorted by colour (orange -> left bin,
everything else -> right bin). A camera watches the belt; an optimizer plans every feed so cubes never pile up and the machine
rests when it is hot or the feeder is empty.

## Run it
```
cd ~/Desktop/PRISM
./start_cell.sh        # starts everything in order, asks before the arm moves
./stop_cell.sh         # stops the run, optimizer and autoen   (--all: everything)
```
Dashboard: http://127.0.0.1:8080 (Overview, Machine, Calibrate, Control). autoen's own page: http://127.0.0.1:8781.
Before a run: belt empty, cubes in the feeder, arm area clear, hand near the E-stop.

## The 100-cube run (PRISM, 2026-11-13 00:25)
93 cubes picked of 102 cycles in 33.9 min, 48 left / 45 right, 0 wrong bins; median 13.0 s and 22.2 J per cube; hottest servo
42 C; 12 V rail never below 12.03 V; no pile-up (belt length 463-468 mm).
- Report: `run_reports/run_20261113_002452.md` (every device) and `run_20261113_002452_devices.csv`
- All runs in one table: `run_reports/runs.csv`
- Comparison with the other runs and the earlier versions: `docs/PRISM_pt5_EXPERIMENTS.md` (section 12 = this run)
- Raw data: `autoencoder/riff_logs/autoen_{raw,cycles,tx}_20261113_002452.*`, optimizer decisions `orchestrator/logs/orch_20261113_002456.jsonl`

## Where things are
| Folder / file | What |
|---|---|
| `autoencoder/` | autoen: owns the Mega, runs the pick cycle (`auto_en_new.py`), feeder presets, raw run logs (`riff_logs/`) |
| `orchestrator/` | the optimizer (brain): `optimize.py`, `actuate.py`, `limits.py` (every rest limit), `cal/` (relay test, run report), `test/` (36 simulator tests) |
| `vision/` | camera: cube colour + position (`vision_usb.py`) |
| `dashboard/` | the web dashboard |
| `arm_cam/` | wrist camera for the emergency grip |
| `arduino_mega_2560/` | firmware for the Mega (belt, feeder, servo bus) |
| `run_reports/` | per-run temperature / power report and `runs.csv` |
| `competition/` | speed and energy sheets, default-vs-PRISM settings |
| `docs/` | `PRISM_pt5_EXPERIMENTS.md` (results), `PRISM_BEHAVIOUR.md` (how it behaves), `PRISM_INFORMATION_BOOK.md` (architecture), `HOW_TO_USE_PRISM.md` |
| `versions/pt5/` | frozen copy of the pt5 code |
| `backup/` | every earlier version of a changed file (`<path>.before-<reason>`); nothing is deleted |
| `archive/` | old logs (before 11-12), the SO-101 zip, and the data of the deleted folders PRISM_mek / PRISM_dupi / PRISM-Mek (see `archive/README.md`) |
| `versions/pt6/` | the version saved 2026-11-13 (pt5 feeding + the fault system and the rest) |
| `run_logs/` | output of the programs, one file per start |

## Not proven on the real cell yet (simulator only)
Feeder preset fd2 against fd1 (needs repeats), the heat guard and feeder rest, the emergency grip with the wrist camera,
sleep with the power relay (run `orchestrator/cal/relay_test.py` once first; the relay is disabled until then).
- 2026-11-13 19:45: full snapshot of the code at the moment the operator went back to pt5 (the fault system, NaN fix, working-area selector, stop_cell kills autoen + camera).
