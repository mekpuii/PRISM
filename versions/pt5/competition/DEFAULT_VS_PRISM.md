# Normal mode vs PRISM: settings and energy

Made 2026-11-10 from the project data. The same table is in `default_vs_prism_settings.csv` (opens in Excel / Sheets).

- **Normal mode** = autoen (`auto_en_new.py`) run alone with its built-in defaults, no optimizer.
- **Manual best** = autoen alone, with the best setting typed in by hand (`HOW_TO_USE_PRISM.md`).
- **PRISM** = autoen + the optimizer (`orchestrator/main.py`).

Energy = motor supplies only (belt INA260 + arm servo current sum), not the Pi / Mega / drivers.
**ESTIMATE** = not measured yet, worked out from other measurements. Run the test at the bottom to replace them.

## 1. Parameters

| Parameter | Normal mode (defaults) | Manual best | PRISM |
|---|---|---|---|
| Belt rpm (real output rpm) | **6.7** | 20 | 20 (picks from 8–20; 15 when warm) |
| Arm speed | **1.0** | 1.5 | 1.5 (picks from 1.0–2.0) |
| Carry speed | **1.0** | 1.5 | 1.5 |
| Gap between cycles | 1.0 s | 1.0 s | 1.0 s |
| Arrive timeout | 20 s | 25 s | 2.7 × belt ride (~25 s at 20 rpm) |
| Stop after N failed cycles | 3 | 3 | an empty feed is not a failure |
| Empty feeder | belt runs ~25 s, 3 times, then stops with an error | same | aborts after ~6 s, then IDLE |
| Idle | belt not braked, arm holds | same | arm home + belt braked |
| After a refill | never restarts | never restarts | restarts by itself (forecast probe / feeder IR) |
| How the setting is chosen | fixed | fixed | every cycle: safety → energy → speed |

## 2. Results

| Result | Normal mode | Manual best | PRISM |
|---|---|---|---|
| Seconds per cube | 35.3 (**ESTIMATE**) | 19.1 | 19.1 |
| Cubes per minute | 1.7 (**ESTIMATE**) | 3.14 | 3.12–3.24 (live) |
| J per good cube | ~23–24 (**ESTIMATE**) | 20.8 | 21.9–22.0 (live) |
| J per empty feeder | ~150 (3 × 49.5) | ~150 | **11.0** (−93 %) |
| Empty feed → idle | ~98 s, then stopped with an error | same | **12.7 s** |
| Idle power | ~0.13 W (**ESTIMATE**) | same | **0.082 W** |

### How to read it

- **While cubes are flowing**, every setting uses about 20–23 J per cube. PRISM's gain there is **speed**: about 1.8× more cubes per minute than the defaults, for the same or slightly less energy.
- **Most of the energy saving is in the waiting time.** An empty feeder costs 11 J instead of ~150 J. Idle draws about 0.08 W instead of ~0.13 W, and the cell restarts by itself after a refill.
- So the more often the feeder runs empty in a session, the bigger PRISM's saving.

## 3. All 12 measured settings (from `orchestrator/cal/baselines_summary.json`)

Run `exp_20261025_005038_opt_grid_v1`, 20 cycles per setting.

| rpm / arm | s per cube | J per cube | OK / 20 |
|---|---|---|---|
| 8 / 1.0 | 32.7 | 22.9 | 20 |
| 8 / 1.5 | 31.5 | 23.1 | 20 |
| 8 / 2.0 | 31.1 | 21.9 | 20 |
| 11.1 / 1.0 | 27.5 | 21.7 | 19 |
| 11.1 / 1.5 | 26.8 | 21.7 | 19 |
| 11.1 / 2.0 | 25.2 | 21.3 | 20 |
| 15 / 1.0 | 23.3 | 21.5 | 20 |
| 15 / 1.5 | 21.9 | 20.2 | 20 |
| 15 / 2.0 | 22.4 | 20.9 | 19 |
| 20 / 1.0 | 20.5 | 22.0 | 20 |
| **20 / 1.5** | **19.1** | **20.8** | **20** |
| 20 / 2.0 | 22.5 | 26.2 | 18 |

The defaults (6.7 / 1.0) are below the tested range.
- The time comes from the cycle-time model `model.json`: s = 8.70 + 2.87 / arm + 158.9 / rpm (R² 0.89).
- The energy is extrapolated from the 8 rpm rows.

## 4. Sources

| Number | Where it comes from |
|---|---|
| Default parameters | `autoencoder/auto_en_new.py` argparse defaults |
| PRISM parameters | `orchestrator/optimize.py` constants |
| Grid energy / time | `orchestrator/cal/baselines_summary.json` |
| Empty feed 49.5 J → 11.0 J, idle 0.082 W, live 21.9–22.0 J | `docs/OPTIMIZER_ROADMAP.md` (machine runs 2026-10-26) |
| Belt idle not braked 0.048 W | `orchestrator/cal/OPTIMIZER_DATA.md` |

## 5. Test to replace the estimates (for the competition)

Run the same session twice, with the same cubes and the same empty-feeder pattern. For example, 12 cubes, let it run empty, wait 5 min, refill 12.

1. **Normal mode:** autoen alone with its defaults (no `--rpm` / `--arm-speed` given), optimizer off.
2. **PRISM:** `./start_cell.sh`.

For each run, compare:
- total J
- J per good cube
- cubes per minute
- J spent while empty or idle
