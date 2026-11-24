# PRISM pt2 - saved 2026-11-12 ~13:30 (the version running well)

pt1 + everything of 2026-11-12:
- every push of the feeder is VERIFIED before the belt moves it (servo current + camera at the feeder exit + feeder IR,
  recorded only); nudge 1 s when unsure; dead servo -> ALARM with the arm shake
- no push on a low 12 V rail; the arm phases that pull the rail down (GRIP) are learned and avoided
- a cube stuck at the feeder exit -> pushed again, longer; push length learned (never below 800 ms)
- the push: blade waits fully back 1 s, jumps to the end at full speed, holds 800 ms
- no fixed pause between picks; at least 2 s between push requests
- Astro HD camera in vision; camera geometry code ready (camgeo.py), new-camera numbers only as a PROPOSAL
  (orchestrator/camera_geometry.proposed.json, not active)
- tuner.py (self-experimenter) and advisor.py (Claude) exist but are NOT connected / OFF

Only code and settings are here (no logs, databases or recordings).

## Go back to pt2
    cd ~/Desktop/PRISM
    ./stop_cell.sh --all
    rsync -a versions/pt2/ ./          # copies pt2's files over the current ones (logs and data are untouched)
    ./start_cell.sh
The Mega sketch is the one uploaded 2026-11-11 22:39 (gentle-push capable); pt2 needs no new upload.
