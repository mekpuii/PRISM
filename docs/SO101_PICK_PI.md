# arm_cam (was so101_pick) on the Pi (arm + wrist camera, standalone)

Ported 2026-11-12 from the Windows zip `so101_pick-20261112T120243Z-1-001.zip`. The code is in `PRISM/arm_cam/` (all commands: `arm_cam/README.md`).
It runs **by itself**: it is not connected to the central PRISM yet (no MQTT, not started by `start_cell.sh`).

## Run

```
cd ~/Desktop/PRISM/arm_cam
./pi.sh                      # what the Pi sees: Mega port, cameras. Opens nothing.
./pi.sh doctor --no-arm      # check files, calibration and camera without touching the Mega
./pi.sh doctor               # full check (reads only, but opening the port resets the Mega)
./pi.sh block_cam            # camera only: live block detection
./pi.sh follow_block --dry   # any script by name, with its own options
```

`pi.sh` uses `~/armcam-env` (Python 3.11, OpenCV 5.0.0, numpy 2.4.6, pyserial 3.5: the same versions as the Windows PC).
The original guides still apply: `STEPS.md`, `HANDOFF.md`, `MOVING.md`, `GUIDE.html`.

## Two rules

1. **Stop PRISM first** (`~/Desktop/PRISM/stop_cell.sh`). The Mega port has one owner, and opening it resets the Mega.
   An arm script now refuses to start while another program holds the port, and names that program.
2. **Plug in the OV9281 wrist camera.** PRISM's overhead camera (Astro HD Camera) is never used as the wrist camera.

## What changed for the Pi

| File | Change |
|---|---|
| `pi_env.py` (new) | Finds the Mega port and the wrist camera, lists who holds a device, reads Enter without blocking |
| `pi.sh` (new) | Launcher with the right Python |
| `config.py` | `ARM_PORT` and `CAM_INDEX` are found automatically on Linux. Override: `SO101_PORT=`, `SO101_CAM=` |
| `camera.py`, `block_cam.py` | Clear message when the wrist camera is missing or busy |
| `arm_mega.py` | Checks that the Mega port is free before opening it |
| `follow_block.py`, `calib_touch.py` | The camera window stays live while waiting for Enter (was Windows only) |

Originals: `PRISM/backup/arm_cam/<file>.before-pi-port`. The Windows `venv/` and `__pycache__/` were not unpacked.

## Not checked yet on the Pi

- The wrist camera: frame rate and exposure lock under V4L2. Config asks for 1280x720 MJPG at 120 fps, exposure 100 (10 ms).
  If the picture is too bright or slow, see `./pi.sh camera_modes` and `CAM_EXPOSURE_VAL` / `CAM_FPS` in `config.py`.
- Any arm motion.
- The calibration files came from the PC setup. `calib.json`, `homography.npy` and `follow_grip.json` are valid only if
  the arm base, camera mount and scan pose are the same as when they were taught.
