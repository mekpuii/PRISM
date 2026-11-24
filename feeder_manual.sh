#!/usr/bin/env bash
# feeder_manual.sh - drive the feeder blade by hand: it asks fd1 or fd2 first, then  p = push, u = pull back, q = quit.
# The cell must be stopped first (autoen holds the Mega's port):  ./stop_cell.sh
if pgrep -f auto_en_new.py >/dev/null; then echo "autoen is running - stop it first:  ./stop_cell.sh"; exit 1; fi
cd "$HOME/Desktop/PRISM/autoencoder" && exec "$HOME/autoen-env/bin/python" feeder_test.py --manual
