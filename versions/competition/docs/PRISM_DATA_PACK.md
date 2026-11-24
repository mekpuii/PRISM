# PRISM data pack - everything about the folder and pt5, for tables and graphs (2026-11-13)

**The Excel file:** `docs/data_pack/PRISM_pt5_data_pack.xlsx` (14 sheets, first row = column names, starter charts on some sheets).
**The same data as CSV** (one file per sheet, same folder): open them in any spreadsheet or plot them with Python/R.
**Rebuild it any time** (reads the logs, calibration and settings files in the PRISM folder; nothing is changed):
`/tmp/xlenv/bin/python ~/Desktop/PRISM/docs/data_pack/build_data_pack.py`
(if `/tmp/xlenv` is gone: `python3 -m venv /tmp/xlenv && /tmp/xlenv/bin/pip install openpyxl`).

## The sheets
| Sheet | Rows | What it holds | Good for |
|---|---|---|---|
| **Runs** | 20 | One row per run: version, cubes, bins, seconds per cube, energy, temperatures, rail, pushes, feeder angles, belt rpm | the main comparison table; bar charts of s/cube and J/cube per run |
| **Versions** | 12 | serial, pipeline, pt1 ... pt5 ... pt6: date, median s/cube, J/cube, cubes, what changed | the "how it got better" graph |
| **Folders** | 19 | every folder of PRISM: what it is, file count, size | the architecture table |
| **Settings_defaults** | 101 | every setting of autoen, the optimizer and vision: name, default, meaning | the parameter table |
| **Constants_in_code** | 242 | the fixed numbers inside the code (optimizer, actuator, observer, forecast, scene, camera geometry, tuner, sorting, vision, autoen) with the comment | the "all parameters" appendix |
| **Calibration_files** | 47 | camera calibration (colours, working area, light), camera geometry, gripper calibration, feeder presets, sorting bins, arm rest state | the calibration table |
| **Learned_settings** | 7 | what the optimizer learned per (belt rpm, arm speed): cycles, ok, success rate, J and seconds per cycle | the "which setting is best" graph |
| **Learned_tuner** | 19 | the tuner champion (gap 60 mm, late 1.25), the feeder push time, the tuner's tries | tuner table |
| **Cycles_RunD_100cubes** / **Cycles_200cubes** | 102 / 200 | every cycle: start, seconds, result, bin, transport / grip / carry seconds | seconds-per-cycle line, histograms, bin counts |
| **Feeds_RunD_100cubes** / **Feeds_200cubes** | 165 / 203 | every push: time, seconds since the previous push, push duration | feeder timing plots |
| **PerMinute_RunD_100cubes** / **PerMinute_200cubes** | 33 / 41 | per minute: hottest arm servo, belt motor temperature, arm and belt power, rail minimum | temperature and power over time |

## Read this before you use the numbers
- **Energy** = belt + the six arm servos (V x I from the 5 Hz log, gaps over 0.5 s capped). The Pi, the Mega, the motor drivers and
  the feeder servo are not counted. **J per cube** = all energy of the run / cubes picked, so idle and rest time count.
- **The 200cubes run has no belt power** (the belt's current sensor sent nothing): its 9.1 J per cube is the arm only. Do not plot
  it against the others; its `belt_w` is 0. Use its seconds and counts.
- **right_down** = the right bin (the colour decides: orange -> left, everything else -> right; "down" is the right bin).
- **belt_rpm** in Runs = the first belt command of the run. The optimizer changes it during a run (run D started at 22 rpm; the
  learned table shows which settings it used and how often). Earlier notes that said "20 rpm" for run D were wrong about the start.
- **Run D's feeder angles** were 170 / 10 (fd1): the Mega received `FEEDCFG,170,10,800,300` although autoen's console printed fd2.
- **Version labels** in Runs are my reading of the notes (stamps from before the optimizer logged settings); the runs under 30
  cubes are start-up tests - leave them out of comparisons.
- **PerMinute** temperatures are the maximum in that minute; powers are the mean over the minute's samples.
- One run (`20261112_232142`) has a false 1.83 V rail minimum (a measurement dropout).

## Graph ideas
1. Bar: median seconds per cube for the runs with >= 30 cubes (Runs, `s_per_cube_median`, filter `cubes_ok` >= 30).
2. Bar: J per cube, same runs (leave out `20261113_162012`).
3. Line: `Versions` - seconds per cube from serial (18.7) to the 200-cube run (11.4).
4. Line: Cycles_200cubes `seconds` (the stability of the cycle time over 200 cubes) and a histogram of the same column.
5. Stacked bars: median transport / grip / carry seconds per run (Runs).
6. Line: PerMinute temperatures - the hottest servo and the belt motor over the 40 minutes.
7. Line: Feeds `seconds since the previous push` (one push per cycle, ~11.5 s) and the push duration (2.13 s).
8. Bar: Learned_settings success rate and seconds per cycle for each (rpm, arm speed).
