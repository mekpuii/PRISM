# PRISM pt5 - saved 2026-11-12 ~20:05 (the operator: "save this version ... this is so cool")

= pt4 + no pile-ups at the feeder + colour by "orange -> left, everything else -> right" + the new dashboard.

**Feeder / optimizer (the 18:17 run piled cubes up)**
- Room first: no push before the belt has moved one cube + one gap since the blade last moved (encoder) - a belt held
  still for GRIP / lift / Standby holds the next push too.
- The camera may stop a push but never allows one alone (the encoder must agree). A cube it sees in the exit zone
  blocks the push and the belt is nudged to carry it away; something that stays put while the belt moves 40 mm is a
  reflection and is ignored.
- "Servo moved, camera saw nothing" is EMPTY only once the camera has proven in this run that it sees the cubes come
  out (4+ arrivals, 80 % seen); before that the push counts as delivered and the ride decides.
- autoen: an arrival after < 60 % of the ride is a cube nobody fed (left on the belt / missed by the camera) - picked,
  but it no longer finishes the lead push; a ride is measured only if 0.8-1.3 x 33000 counts (the ride had fallen
  468 -> 230 mm on 11-12).
- Sorting: the colour of the cube AT the IR decides; left only when it is seen as orange.
- New test: orchestrator/test/test_nostack_sim.py (31 + 1 test suites pass).

**Camera / dashboard**
- vision_usb.py: cube-shape filter (reflections are smaller / thinner / patchier); no desktop window (--show opens it);
  reloads vision_calibration_usb.json within 2 s; applies the saved camera controls; raw picture for Calibrate.
- Dashboard: Control -> Calibrate (click a cube, label it, Save; More settings: camera light, colour ranges, shape);
  Machine page with Raspberry Pi gauges; the 3D twin removed; overview None-safe.

Only code and settings are here (no logs, databases or recordings).

## Go back to pt5
    cd ~/Desktop/PRISM
    ./stop_cell.sh --all
    rsync -a versions/pt5/ ./          # copies pt5's files over the current ones (logs and data are untouched)
    ./start_cell.sh
