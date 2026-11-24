#!/usr/bin/env bash
# autostart_cell.sh - run at desktop login by ~/.config/autostart/prism-cell.desktop (in a terminal window).
# Waits for the broker and the camera, starts the cell with start_cell.sh (it asks before the arm moves), then shows
# the optimizer's live output. Close the window to leave everything running; ./stop_cell.sh stops it.
# Turn the autostart off:  rm ~/.config/autostart/prism-cell.desktop
cd "$HOME/Desktop/PRISM" || exit 1
echo "PRISM cell autostart - waiting for the broker and the camera ..."
for _ in $(seq 1 60); do
  systemctl is-active --quiet mosquitto && break
  sleep 1
done
sleep 15                                    # the camera needs a moment after boot before vision can open it
./start_cell.sh
echo
latest=$(ls -t run_logs/optimizer_*.log 2>/dev/null | head -1)
if [ -n "$latest" ]; then
  echo "=== optimizer output ($latest) - Ctrl+C stops watching, the cell keeps running ==="
  tail -n 20 -f "$latest"
fi
exec bash
