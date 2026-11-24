# PRISM pt4 - saved 2026-11-12 ~15:00 (the operator: "save the latest as pt4")

= pt3 + the fixes after the first pt3 runs:
- pt3: FILL THE BELT (as many cubes as fit: feeder->IR 470 mm / (40 mm cube + gap >= 60 mm), max 6), spacing by the
  belt encoder and the camera, the new camera's geometry ACTIVE (orchestrator/camera_geometry.json: exit 65 px,
  IR 805 px, 0.64 mm/px) -> colour sorting white -> right bin, orange -> left bin; tuner.py connected (gap 60/80/100 mm,
  late 1.25/1.35/1.6); stricter stuck-cube detector; a cube seen in the feeder mouth blocks the next push
- fix: white cubes are not "glare" any more (each adds ~6 % bright pixels with the new camera; one was enough to stop
  the colour votes -> every cube went to the right bin)
- fix: the stuck-cube proof (a still glare blob no longer blocks it)
- feeder: back 170 deg, push 10 deg, wait 1 s back, jump, hold 800 ms (learned longer if a cube sticks)
- tools: autoencoder/feeder_test.py (feeder only; --manual: p push / l pull)
There is no separate pt3 copy: pt4 contains it. pt2 (versions/pt2) is the version before filling the belt.

Only code and settings are here (no logs, databases or recordings).

## Go back to pt4
    cd ~/Desktop/PRISM
    ./stop_cell.sh --all
    rsync -a versions/pt4/ ./          # copies pt4's files over the current ones (logs and data are untouched)
    ./start_cell.sh
