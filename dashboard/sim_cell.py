#!/usr/bin/env python3
"""
sim_cell.py - run the REAL autoen_node.py against a simulated Mega, so you can try the whole dashboard with no robot.

  python3 sim_cell.py                     real-time speed, autoen's page on http://127.0.0.1:8781, waits for Start
  python3 sim_cell.py --fast 4            servos 4x faster (quick demo)
  python3 sim_cell.py --fast 2 --sets 3 --cycles-per-set 4 --mqtt-host localhost     (everything after the sim options goes to autoen_node)

Needs: autoen_node.py in the same folder (or the current folder), pip install pyserial paho-mqtt, and an MQTT broker (mosquitto).
Nothing here talks to a serial port: autoen_node's serial.Serial is replaced by sim_mega.FakeMega.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (HERE, os.path.dirname(HERE), os.getcwd()):
    if p not in sys.path:
        sys.path.append(p)

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("--fast", type=float, default=1.0, help="speed-up of the simulated servos (1 = real-time)")
ap.add_argument("-h", "--help", action="store_true")
a, rest = ap.parse_known_args()
if a.help:
    print(__doc__)
    sys.exit(0)

try:
    import autoen_node as A
except ImportError as e:
    sys.exit(f"cannot import autoen_node.py ({e}). Put sim_cell.py next to autoen_node.py.")
from sim_mega import FakeMega

def new_fake():
    """A fresh simulated Mega (it prints its boot banner once, at creation: autoen_node waits for that)."""
    global fake
    fake = FakeMega(fast=a.fast)
    A.serial.Serial = lambda port, baud=115200, timeout=0.1: fake      # autoen_node opens its "port" and gets the fake
if "--port" not in rest:
    rest = ["--port", "SIMULATED"] + rest
print(f"[sim] simulated Mega (x{a.fast:g}); starting autoen_node with: {' '.join(rest)}", flush=True)
# autoen_node quits when its run is over. For a simulator that is annoying: start a fresh, waiting page again.
try:
    while True:
        new_fake()
        rc = A.main(rest)
        fake.close()
        print(f"[sim] autoen_node ended (rc={rc}); starting a fresh waiting page in 3 s (Ctrl+C to quit)", flush=True)
        import time
        time.sleep(3)
except KeyboardInterrupt:
    sys.exit(0)
