# archive - what came from the other PRISM folders (2026-11-13, merged into the main PRISM; the folders were deleted)

The Desktop now holds only `PRISM`. Before deleting `PRISM_mek`, `PRISM_dupi` (the PRISM_friend copy), `PRISM_friend`, `PRISM-Mek`
(the 11-11 copy) and `PRISM-Mek.7z` (the same 11-11 copy, packed), everything that was not already in PRISM was kept:

- **Moved into the live PRISM (new file names only, nothing overwritten):** the run data of PRISM_mek and PRISM_dupi:
  `autoencoder/riff_logs/` (raw, cycles, commands), `orchestrator/logs/`, `run_logs/`, `run_reports/` and their lines in
  `run_reports/runs.csv` (including the 200-cube run `20261113_162012`).
- **Kept here, as they were (same folder layout):** `PRISM_mek/`, `PRISM_dupi/`, `PRISM-Mek/` = the data files that were new or
  different from PRISM's: the learning (`orchestrator/learned_state.json`, `forecast_state.json`), the logger databases, rest state,
  old experiment logs. Use them to compare; they were not merged (two learned states cannot be merged).
- `PRISM-Mek_code_11-11.tar.gz` = the code files of the 11-11 copy (no saved version has them). The code of PRISM_mek and
  PRISM_dupi is pt5 (`versions/pt5`), so it was not copied again.
- `run_logs/` (old program logs), `so101_pick-...zip` (the SO-101 download).
