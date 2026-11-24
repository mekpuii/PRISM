# PRISM pt1 - saved 2026-11-12

The working version before the "verified feeding" work (pt2): FLOW mode (the optimizer plans every feed, up to
3 cubes in flight, learned push success per arm phase, late cube = given up and fed again, 3 failures = feeder
empty), the Astro HD camera in vision, the gentle-push-capable Mega sketch (gentle push OFF by default).

Only code and settings are here (no logs, databases or recordings).

## Go back to pt1
    cd ~/Desktop/PRISM
    ./stop_cell.sh --all
    rsync -a versions/pt1/ ./          # copies pt1's files over the current ones (logs and data are untouched)
    ./start_cell.sh
The Mega sketch did not change for pt1; upload arduino_mega_2560/ only if a later version changed it.
