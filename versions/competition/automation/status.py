#!/usr/bin/env python3
"""automation/status.py - ONE line in the terminal, refreshed every second, that says whether the feeder path is alive.
    python3 ~/Desktop/PRISM/automation/status.py          (run.sh opens it for you with --watch; Ctrl+C closes it)
FEEDER OK  = the Mega's telemetry is fresh (< 1 s), no fault, and it has answered every push so far.
FEEDER ??  = no push answered for a long time while the run goes on (a stuck push or a silent servo).
FEEDER LOST= no telemetry from the Mega (USB cable, power, or autoen stopped): a BELL rings and the line turns red.
Honest limit: the feeder SERVO has no feedback wire. 'push answered' means the Mega ran the push; that the blade really
moved shows as the cube coming out (feeder IR / the next pick). So watch 'cube at exit' and the picks as well."""
import json
import sys
import time
import urllib.request

API = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8781/api/state"
RED, GRN, YEL, OFF = "\033[1;31m", "\033[1;32m", "\033[1;33m", "\033[0m"


def get():
    with urllib.request.urlopen(API, timeout=1.5) as r:
        return json.load(r)


def main():
    last_n, last_change, ok_since = None, time.monotonic(), None
    while True:
        try:
            d = get()
            f = d.get("feeder_link") or {}
            age = f.get("telemetry_age_s")
            n = f.get("feed_done_n", 0)
            if n != last_n:
                last_n, last_change = n, time.monotonic()
            running = d.get("status") == "running"
            if age is None or age > 1.5:
                col, tag, bell = RED, "FEEDER LOST", "\a"
            elif f.get("fault"):
                col, tag, bell = RED, f"FEEDER FAULT {f['fault']}", "\a"
            elif running and time.monotonic() - last_change > 90 and n:
                col, tag, bell = YEL, "FEEDER ??", ""
            else:
                col, tag, bell = GRN, "FEEDER OK", ""
            ir = {True: "cube at exit", False: "exit empty", None: "?"}[f.get("feed_ir")]
            st = {0: "idle", 1: "pushing", 2: "returning"}.get(f.get("feed"), "?")
            line = (f"{time.strftime('%H:%M:%S')}  {col}{tag}{OFF}{bell}  link {age if age is not None else '-'} s old | "
                    f"blade {st} | {ir} | pushes answered {n} | {d.get('status')} {d.get('phase')} "
                    f"cube {d.get('cycle_total')}/{d.get('cycles_per_set')} | rail {f.get('rail_v', '-')} V")
        except Exception:                                   # noqa: BLE001 - autoen not answering at all
            line = f"{time.strftime('%H:%M:%S')}  {RED}FEEDER LOST{OFF}\a  autoen does not answer on {API}"
        sys.stdout.write("\r\033[K" + line)
        sys.stdout.flush()
        time.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
