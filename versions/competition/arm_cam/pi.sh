#!/usr/bin/env bash
# pi.sh - run an arm_cam script on the Raspberry Pi with the right Python.
#
#   ./pi.sh                      show what the Pi sees (Mega port, cameras) - opens nothing
#   ./pi.sh doctor               full check: files, calibration, camera, arm (reads only, but opening the port resets the Mega)
#   ./pi.sh doctor --no-arm      the same without touching the Mega
#   ./pi.sh block_cam            camera only: live block detection (no arm)
#   ./pi.sh follow_block --dry   any script by name, with its own options
#
# Python: ~/armcam-env (Python 3.11, OpenCV 5.0.0, numpy 2.4.6, pyserial 3.5 - see requirements.txt).
#   Another one:  SO101_PY=/path/to/python ./pi.sh ...
# Devices are found automatically (pi_env.py). To name them yourself:
#   SO101_PORT=/dev/ttyUSB0 SO101_CAM=/dev/video2 ./pi.sh ...
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${SO101_PY:-$HOME/armcam-env/bin/python}"
if [ ! -x "$PY" ]; then
  echo "Python not found: $PY   (set SO101_PY=/path/to/python)"
  exit 1
fi
cd "$HERE" || exit 1
# OpenCV's own Qt has no Wayland plugin: use X11 (XWayland), so no warning is printed.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

if [ $# -eq 0 ]; then
  exec "$PY" pi_env.py
fi

script="$1"; shift
script="${script%.py}.py"
if [ ! -f "$script" ]; then
  echo "no such script: $script"
  echo "scripts here:"; ls *.py | sed 's/\.py$//' | column -c 100
  exit 2
fi
exec "$PY" "$script" "$@"
