#!/usr/bin/env bash
# automation/stop.sh - stop the plain automation (the arm holds where it is, the belt coasts), then autoen.
curl -s -m 5 -X POST http://127.0.0.1:8781/api/stop >/dev/null; sleep 1
pkill -INT -f auto_en_new.py && echo "autoen stopped"
