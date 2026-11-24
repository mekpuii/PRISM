# PRISM_mek - an exact copy of pt5 (2026-11-13), with the optimizer

Copied from `~/Desktop/PRISM/versions/pt5` (pt5 = no pile-ups, orange -> left / the rest -> right, the optimizer plans every feed).
Changed only: the folder name inside the scripts (`Desktop/PRISM` -> `Desktop/PRISM_mek`) and `autoencoder/rest_state.json`
(copied from the live PRISM, because it says where the real arm is). Learning (`orchestrator/learned_state.json`,
`forecast_state.json`) starts from pt5's.

NOT in this copy (they are newer than pt5): the fault system, the heat guard, the feeder rest, the emergency grip, sleep / relay,
the Limits zone, the run report. Runs here write no `run_reports/`: use the main PRISM's script on this folder's data:
`~/orch-env/bin/python ~/Desktop/PRISM/orchestrator/cal/run_report.py --riff ~/Desktop/PRISM_mek/autoencoder/riff_logs <stamp>`
(it writes `run_reports/` next to that folder).

Run (the same robot as PRISM: never both at once):
    cd ~/Desktop/PRISM && ./stop_cell.sh --all
    cd ~/Desktop/PRISM_mek && ./start_cell.sh
Data: `autoencoder/riff_logs/` (raw 5 Hz, cycles, commands), `orchestrator/logs/` (optimizer decisions), `run_logs/`.
Results of the earlier pt5 runs: `~/Desktop/PRISM/docs/PRISM_pt5_EXPERIMENTS.md`.
