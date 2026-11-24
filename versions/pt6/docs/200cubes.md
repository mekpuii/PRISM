# 200cubes - the 200-cube run of PRISM_mek (pt5 + optimizer), 2026-11-13

Run `20261113_162012`, folder `~/Desktop/PRISM_mek` (an exact copy of pt5, with the optimizer), 16:20:17 - 17:00:13 (39.9 min).
Feeder angles 170 / 10 (fd1: `FEEDCFG,170,10,800,300`), belt 20 rpm, arm speed 1.5. Written from the run's own logs.

| | 200cubes | Run D (PRISM, 11-13 00:25) | Run C (PRISM, 11-12 21:30) |
|---|---|---|---|
| Length | 39.9 min | 33.9 min | 10.5 min |
| Cubes OK / cycles | **199 / 200** (1 GRIP_SUSPECT, cycle 79) | 93 / 102 | 44 / 49 |
| Left / right | 91 / 108 | 48 / 45 | 23 / 21 |
| Median s / cube | **11.4** | 13.0 | 11.0 |
| Whole run s / cube | 12.0 | 21.9 | - |
| Pushes (FEED -> FEED_DONE) | 203, median 2.13 s, 438 s in total (18 % of the run) | 165, 2.13 s, 356 s | - |
| Waiting for the belt (TRANSPORT) | median 1.3 s, 354 s in total (15 %) | median 3.3 s, 830 s (41 %) | - |
| Grip / carry (median) | 1.6 s / 3.9 s | 1.6 s / 3.4 s | - |
| Hottest arm servo | 42 C (J6) | 42 C | 41 C |
| Belt motor max | 28 C | 27 C | 26 C |
| Emergency grips / warnings | 0 / 0 | 0 / 0 | 0 / 0 |
| **Energy per cube** | **not measured** (see below) | 22.2 J | 16.2 J |

**What it shows**
- 199 cubes in 39.9 minutes with one doubtful grip: the longest clean run so far, and faster than run D (11.4 s against 13.0 s).
- Much less waiting for the belt than run D (354 s against 830 s): cubes were ready more often when the arm came back.
- The feed itself is the same as in every run (2.13 s a push, 203 pushes for 200 cycles: almost no retries).
- Sorting counts are not checked against the real colours here; the bins are the camera's call (a human spot check is still open).

**The energy is NOT usable.** The report says 9.1 J per cube, but that is the arm only: the raw log has **no belt current and no
12 V rail voltage** (`Belt_current` and `Belt_voltage` are empty in all 11946 samples, belt temperature and speed are logged).
The belt's INA260 sensor was not sending in this run (not working or not connected after the repair). Do not compare 9.1 J with the
other runs. It also means the rail dip of each push was not recorded. Fix the sensor, then repeat for the energy.

Data: `~/Desktop/PRISM_mek/autoencoder/riff_logs/autoen_{raw,cycles,tx}_20261113_162012.*`, report
`~/Desktop/PRISM_mek/autoencoder/run_reports/run_20261113_162012.md` (made with `orchestrator/cal/run_report.py --riff`).

## Time from one feed to the next (203 pushes, from `autoen_tx_20261113_162012.log`)

| | Seconds |
|---|---|
| **Median time between two pushes** | **11.5** (mean 11.8) |
| 10 % of the gaps are shorter than / 90 % shorter than | 10.1 / 13.5 |
| Shortest / longest | 4.7 / 23.0 |
| From the end of one push (FEED_DONE) to the start of the next | median 9.4 (2.4 - 20.9) |
| One push itself | 2.13 |
| Pushes per minute | 5.1 on average, 6 at most |

Most gaps (171 of 202) are 10-14 s, i.e. about one push per cycle (cycle median 11.4 s). Only 3 gaps were over 20 s
(16:36:35 23.0 s, 16:50:05 21.0 s, 16:52:28 22.1 s) and 8 were under 6 s (a retry or a cube nobody fed). The feeder was never
close to the 12-pushes-a-minute limit of the newer PRISM.
