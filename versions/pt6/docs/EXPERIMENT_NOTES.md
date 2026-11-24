# PRISM — data clean-up and the new experiment tool (2026-10-24)

What was done in this session, which files changed, and how to use the new tool.
Nothing in this work ran on the real robot. `BELT_MM_S_PER_RPM` was not changed. No file was deleted.

---

## 1. The old data (archived, not used for fitting)

| What | Where |
|---|---|
| Read-only copy of all 154 data files (same folder layout, checked by size + sha256) | `data/raw_archive/` |
| One line per file: times, cycles, results, rpm / arm_speed / carry_speed, flags, status (good / messy / junk) | `data/runs_index.csv` |

Result: 80 good, 23 messy, 51 junk.

**Important finding:** `orchestrator/cal/runs/run_100rpm_armspd2.0.jsonl` is labelled 100 rpm but the belt really ran
at **~60 rpm** (`X,CONV,60` in its TX log). So the arm_speed = 2.0 row of `armspeed_sweep.csv` changed two things at once.
This is the main reason the new tool checks the belt and reads the config back before every block.

---

## 2. What the investigation found

**Units.** The old riff logs and the cal runs use the *same* units, just different settings.

| Setting | What `auto_en_new.py` sends |
|---|---|
| `rpm` (0.5–37) | `X,CONV,<rpm>` to the Mega (target of its own speed loop). Real output-shaft rpm since the CPR fix (`X,CPR,11880`, 2026-10-24). Runs before that used `X,CPR,1320`: their rpm is 9× the real speed |
| `arm_speed` (0.2–8.8) | `P,<joint>,<target>,<ticks/s>,<acc>` with ticks/s = clamp(round(360 × arm_speed), 90, 3200). 1.0 ≈ 31.6 °/s |
| `carry_speed` | same formula, only while carrying the cube |
| gripper | fixed 1200 ticks/s |

**Signals for failed cycles.**
- Grip miss: the gripper's stop position (`G,CLOSED,<pos>`) is outside 178–186°.
- Transport timeout: no `ARRIVED` from the IR within `arrive_timeout`.
- Drop while carrying: no sensor sees it. It can only be *suspected* from the gripper (J6) position/current while carrying (`GRIP_SUSPECT`).

**Where "TimeoutError" comes from.** `auto_en_new.py` line ~1554 writes the Python error name as the result when the
error is not a `CycleError`. The two real cases were `TimeoutError: no reply to X,FEED` (the Mega did not answer the
feed command within 1 s). This line was not changed; `experiment.py` classifies around it.

---

## 3. The code

### Changed files (a backup of each original is kept next to it)

| File | Backup | Change |
|---|---|---|
| `autoencoder/auto_en_new.py` | `auto_en_new.py.before-experiment` | **pause_before_set** setting: pause before every set, so 1 set = 1 block with exactly n cycles. **POST /api/task**: while paused, run the empty belt / hold a pose / stay idle and measure belt rpm, mA, W and arm mA, W. New fields `task` and `set_boundary` in `/api/state`. With `pause_before_set = 0` (default) autoen works exactly as before. |
| `orchestrator/cal/record_run.py` | `record_run.py.before-experiment` | Recording code moved into a `Recorder` class so `experiment.py` can use it. Same command line, same file format. |

### New files

| File | What it is |
|---|---|
| `orchestrator/cal/experiment.py` | The experiment tool (modes: `run`, `calibrate`, `baseline`, plus `--dry-run`) |
| `orchestrator/cal/plans/example_grid.json` | Example plan: 4 configs × 4 repeats × 10 cycles, shuffled per round with a fixed seed |
| `orchestrator/cal/test_experiment.py` | 19 tests against a fake autoen (no robot, no broker) |
| `orchestrator/cal/test_experiment_sim.py` | Full test: real `auto_en_new.py` + simulated Mega (no serial port, no MQTT) |
| `orchestrator/cal/experiment_changes.patch` | The complete diff of everything above (for review) |

### How it talks to autoen

`experiment.py` uses autoen's local web server (`http://127.0.0.1:8781`, the same one the autoen web page uses).
Only the Pi talks to itself; **no internet is needed**. It never opens the Mega's USB port — autoen does all the moving.

| Address | Used for |
|---|---|
| `GET /api/state` | status, current settings, last cycle results, task result |
| `POST /api/config` | set rpm, arm_speed, carry_speed, cycles per set, extra keys |
| `POST /api/start`, `/api/pause`, `/api/stop` | the same as the Start / Pause / Stop buttons |
| `POST /api/task` (new) | only while paused: `belt_run` (empty belt at an rpm), `hold_pose` (up / right / left / down), `idle` |

### Result classes (one per cycle, the raw autoen result and text are always kept too)

| Class | autoen results |
|---|---|
| `OK` | OK |
| `GRIP_MISS` | GRIP_MISS, GRIP_FAILED, NO_BLOCK_AT_IR, BLOCK_STILL_AT_IR |
| `DROP_CARRY` | GRIP_SUSPECT (suspected only — compare with the bin count you type after each block) |
| `TIMEOUT_TRANSPORT` | TIMEOUT_TRANSPORT |
| `TIMEOUT_OTHER` | TIMEOUT_FEED, TIMEOUT_STANDBY, `TimeoutError` "arm did not finish …" |
| `EXCEPTION` | FAULT_n and any other error name (e.g. `TimeoutError: no reply to X,FEED`) |
| `ABORTED` | cycles that never ran (belt check failed, skipped, run stopped) |

---

## 4. How to use it

All commands run on the Pi. Shortcut used below:

```bash
EXP="$HOME/orch-env/bin/python $HOME/Desktop/PRISM/orchestrator/cal/experiment.py"
```

### Step 0 — start autoen (as usual, with the patched file)
Restart it so the running code is the new version. It waits on its page and does not move until a session starts.

### Step 1 — dry run (checks only, nothing moves, nothing is written)
```bash
$EXP run --plan ~/Desktop/PRISM/orchestrator/cal/plans/example_grid.json --dry-run
```
It prints the block order and estimated time, and checks: autoen reachable and patched, every plan key known,
`belt_ref.json` has every rpm, MQTT broker reachable.

### Step 2 — calibrate the empty belt (no cubes on the belt)
```bash
$EXP calibrate --rpms 5,8,11,15,20,25 --seconds 30
```
For each rpm it asks for Enter, runs the empty belt for 30 s, and saves the measured speed to
`orchestrator/cal/belt_ref.json`. Add every rpm your plan uses.

### Step 3 — the real run
```bash
$EXP run --plan ~/Desktop/PRISM/orchestrator/cal/plans/example_grid.json \
     --note "fresh cubes, morning" --cubes "10x white 40mm"
```
For every block:
1. sets the config and reads it back (a mismatch aborts the block),
2. asks you: **Enter** = start, **s** = skip, **q** = quit,
3. runs the empty belt for 30 s; if the speed is more than 15 % away from `belt_ref.json`, the block is aborted,
4. runs exactly n cycles and prints each result,
5. asks how many cubes landed in the bins (Enter = skip).

Ctrl-C stops autoen (belt stops, arm holds) and marks the rest as ABORTED.

### Baselines (no cubes, log power)
```bash
$EXP baseline --kind belt --rpm 11.1 --seconds 60     # empty belt
$EXP baseline --kind pose --pose right --seconds 60   # arm holds Home
$EXP baseline --kind idle --seconds 60                # nothing moves
```

### Useful options
| Option | Meaning |
|---|---|
| `--dry-run` | check and print only |
| `--belt-check-s 30` | length of the belt check |
| `--tolerance 0.15` | allowed belt speed difference (15 %) |
| `--allow-missing-ref` | run blocks whose rpm is not in `belt_ref.json` |
| `--stop-at-end` | stop autoen at the end (default: leave it paused, so the next session can reuse it) |
| `--note`, `--cubes` | saved in the meta file |

### Output files (in `orchestrator/cal/runs/`, never overwritten)
- `<run_id>.jsonl` — every MQTT message plus `exp/...` lines (config, belt check, each cycle, each block)
- `<run_id>.meta.json` — plan, block order, config snapshots, belt checks, per-block results and counts,
  sha256 of `auto_en_new.py` / `experiment.py` / the plan, your note, cube set, start and end time

### Plan file format (JSON, or YAML if PyYAML is installed)
```json
{
  "name": "example_grid",
  "seed": 20261024,
  "order": "rounds",
  "repeats": 4,
  "defaults": {"n_cycles": 10, "carry_speed": 1.0, "set": {"arrive_timeout": 40}},
  "configs": [
    {"label": "a0.7", "rpm": 11.1, "arm_speed": 0.7, "carry_speed": 0.7},
    {"label": "a1.0", "rpm": 11.1, "arm_speed": 1.0}
  ]
}
```
- `order`: `rounds` (each round runs every config once, shuffled), `shuffle` (all blocks shuffled), `as_listed`.
- `set`: extra autoen settings for the block (any key from the autoen page, e.g. `arrive_timeout`).
  `enable_b`, `enable_c`, `sets`, `cycles_per_set`, `pause_before_set` are controlled by `experiment.py` and not allowed here.
- Set `arrive_timeout` high enough for slow belts (at old "20 rpm" = 2.2 real rpm every old cycle timed out at the default 20 s).

### Running the tests
```bash
cd ~/Desktop/PRISM/orchestrator/cal
~/orch-env/bin/python test_experiment.py       # fake autoen
~/db-env/bin/python  test_experiment_sim.py    # real auto_en_new.py + simulated Mega
```

---

## 5. Things to keep in mind
- The belt check compares the encoder rpm with your own calibration. It catches a belt that slips, is mis-set or
  stopped, but it does not measure mm per rpm. Since the CPR fix, `BELT_MM_S_PER_RPM` comes from the calibration (Step 4).
- If you paste a new `auto_en_new.py` over the file, `pause_before_set` and `/api/task` are lost.
  The patched version is in `experiment_changes.patch`; the original is `auto_en_new.py.before-experiment`.
- The meta file stores the sha256 of `auto_en_new.py` on disk. Restart autoen after any change so the running code matches.
- Next big step after collecting data: write `orchestrator/optimize.py` and fit it on the new runs.
