#!/usr/bin/env python3
"""
belt_power.py -- empty-belt power at several rpm WITHOUT autoen (belt only, the arm is not needed).

  cd ~/Desktop/PRISM/orchestrator/cal
  ~/autoen-env/bin/python belt_power.py --plan plans/belt_power_v1.json --dry-run     (opens nothing, moves nothing)
  ~/autoen-env/bin/python belt_power.py --plan plans/belt_power_v1.json --note "..."

autoen must NOT be running (it owns the Mega port). Opening the port resets the Mega. The servos are not needed:
only the belt module (X,... commands) is used, so a broken arm is fine. The belt settings are autoen's own defaults
(CPR 11880, PID 4.5 / 13.5 / 0, TEMPMAX, WDT 500), so the rpm means the same as in every autoen run.

Every step waits for Enter (s = skip, q = quit). rpm 0 = X,BRAKE (belt held still, power still logged).
The belt is run like autoen's belt_run task: X,CONV,<rpm> (an IR hit brakes it = the step is marked 'arrived').
Statistics skip the first settle_s; W = belt INA260 current (mA) x belt INA260 bus voltage (mV) / 1e6, as autoen.
Writes runs/beltpwr_<stamp>.meta.json (after every step) + runs/beltpwr_<stamp>.samples.csv (every belt sample).
"""
import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import sys
import time
import urllib.request
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
AUTOEN_DIR = os.path.join(HERE, "..", "..", "autoencoder")
sys.path.insert(0, AUTOEN_DIR)
from auto_en_new import Link, build_parser, find_serial_port  # noqa: E402

RPM_UNITS = "output_rpm_cpr11880"     # same as experiment.py
AUTOEN = os.path.join(AUTOEN_DIR, "auto_en_new.py")
FAULTS = {"FAULT_1": "watchdog (no X,HB from the PC)", "FAULT_2": "motor over temperature",
          "FAULT_3": "temperature sensor lost (DS18B20 on D24 gives no reading: check its wires)", "FAULT_4": "stall"}
SAMPLE_COLS = ["step", "rpm_set", "t_s", "rpm", "mA", "mV", "W", "pwm", "mode", "ir", "fault", "temp", "mega_ms"]


def iso(t=None):
    return datetime.fromtimestamp(t or time.time()).isoformat(timespec="seconds")


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def write_json_atomic(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def stat(v):
    v = [x for x in v if x is not None]
    return {"n": len(v), "mean": round(statistics.fmean(v), 4) if v else None,
            "std": round(statistics.pstdev(v), 4) if len(v) > 1 else None,
            "min": round(min(v), 4) if v else None, "max": round(max(v), 4) if v else None}


def step_stats(rows, settle_s):
    """rows after the settle time -> mean/std per signal + first-half vs second-half W (is it steady?)."""
    r = [x for x in rows if x["t_s"] >= settle_s]
    out = {k: stat([x[k] for x in r]) for k in ("rpm", "mA", "mV", "W", "pwm")}
    half = len(r) // 2
    a, b = stat([x["W"] for x in r[:half]]), stat([x["W"] for x in r[half:]])
    out["W_first_half_mean"], out["W_second_half_mean"] = a["mean"], b["mean"]
    out["n_settle_skipped"] = len(rows) - len(r)
    return out


def autoen_running():
    try:
        urllib.request.urlopen("http://127.0.0.1:8781/api/state", timeout=1)
        return True
    except Exception:                                       # noqa: BLE001
        return False


def belt_settings():
    """autoen's own defaults for the belt (parsed from its argparse, not copied by hand)."""
    a = build_parser().parse_args([])
    wdt = a.belt_wdt_ms if a.belt_wdt_ms is not None else (1500 if a.enable_b else 500)
    return [f"CPR,{a.cpr:g}", f"TEMPMAX,{a.temp_max:g}", f"PID,{a.kp:g},{a.ki:g},{a.kd:g}", f"WDT,{wdt}"]


def load_plan(path):
    with open(path, encoding="utf-8") as f:
        plan = json.load(f)
    steps = plan["steps"]
    for s in steps:
        r = float(s["rpm"])
        if not (r == 0 or 0.5 <= r <= 37.0):
            raise SystemExit(f"step {s['step']}: rpm {r:g} must be 0 or 0.5..37")
    seconds, settle = float(plan.get("seconds", 70)), float(plan.get("settle_s", 10))
    if not 0 <= settle < seconds:
        raise SystemExit("settle_s must be 0..seconds")
    return plan, steps, seconds, settle


def run_step(link, step, seconds, settle, writer):
    """One step: CONV / BRAKE, sample for `seconds`, BRAKE. -> result dict."""
    rpm = float(step["rpm"])
    while not link.events.empty():
        link.events.get_nowait()
    link.belt_cmd("BRAKE" if rpm == 0 else f"CONV,{rpm:.2f}")
    rows, last_ms, t0, why, shown = [], None, time.monotonic(), None, 0.0
    t_start = time.time()
    try:
        while (el := time.monotonic() - t0) < seconds:
            try:
                name, args = link.events.get_nowait()
                if name == "FAULT":
                    why = f"FAULT_{args[0] if args else '?'}"
                    break
                if name == "ARRIVED":
                    why = "arrived (the IR saw something: the Mega braked the belt)"
                    break
            except Exception:                               # noqa: BLE001 - queue.Empty
                pass
            with link.slock:
                b, bt = link.belt, link.belt_t
            if b is not None and b["ms"] != last_ms and time.monotonic() - bt < 1.0:
                last_ms = b["ms"]
                ma = abs(b["ma"]) if math.isfinite(b["ma"]) else None
                mv = b["mv"] if math.isfinite(b["mv"]) else None
                row = {"step": step["step"], "rpm_set": rpm, "t_s": round(el, 3), "rpm": abs(b["rpm"]), "mA": ma,
                       "mV": mv, "W": (ma * mv / 1e6) if ma is not None and mv is not None else None, "pwm": b["pwm"],
                       "mode": b["mode"], "ir": int(b["ir"]), "fault": b["fault"], "temp": b["temp"], "mega_ms": b["ms"]}
                rows.append(row)
                writer.writerow(row)
                if time.monotonic() - shown >= 1.0:
                    shown = time.monotonic()
                    print(f"\r  {el:5.0f}/{seconds:g} s  rpm {row['rpm']:6.2f}  {row['mA'] or 0:6.1f} mA  "
                          f"{(row['mV'] or 0) / 1000:5.2f} V  {row['W'] or 0:6.3f} W   ", end="", flush=True)
            time.sleep(0.02)
    finally:
        try:
            link.belt_cmd("STOP" if why and why.startswith("FAULT") else "BRAKE")
        except Exception:                                   # noqa: BLE001
            pass
    print()
    return {"step": step["step"], "round": step.get("round"), "rpm_set": rpm, "seconds": seconds, "settle_s": settle,
            "start": iso(t_start), "end": iso(), "status": "done" if why is None else "aborted", "abort_reason": why,
            "stats": step_stats(rows, settle)}


def main(argv=None):
    ap = argparse.ArgumentParser(description="empty-belt power at several rpm, belt only (no autoen, no arm)")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--port", default=os.environ.get("ARM_SERIAL_PORT"), help="default: auto-detect")
    ap.add_argument("--runs-dir", default=os.path.join(HERE, "runs"))
    ap.add_argument("--note", default="")
    ap.add_argument("--dry-run", action="store_true", help="print the plan + checks; opens no port, moves nothing")
    a = ap.parse_args(argv)
    plan, steps, seconds, settle = load_plan(a.plan)
    settings = belt_settings()

    print(f"plan {a.plan}: {len(steps)} steps, {seconds:g} s each ({settle:g} s settle not counted), "
          f"~{len(steps) * (seconds + 15) / 60:.0f} min with your Enter presses")
    for s in steps:
        print(f"  step {s['step']:>2}  round {s.get('round', '-')}  rpm {float(s['rpm']):>5g}"
              + ("  (BRAKE: belt held still)" if float(s["rpm"]) == 0 else "  X,CONV"))
    print("belt settings sent first (autoen defaults):", ", ".join(settings))
    if a.dry_run:
        busy = autoen_running()
        print(("  FAIL  " if busy else "  OK    ") + "autoen is " + ("RUNNING: stop it first (it owns the port)" if busy
                                                                   else "not running"))
        try:
            print(f"  OK    Mega port {a.port or find_serial_port()}")
        except SystemExit as e:
            print(f"  FAIL  {e}")
        print("DRY RUN: no port opened, nothing moved, nothing written.")
        return 0
    if autoen_running():
        raise SystemExit("autoen is running (it owns the Mega port): stop it first")

    os.makedirs(a.runs_dir, exist_ok=True)
    run_id = f"beltpwr_{datetime.now():%Y%m%d_%H%M%S}"
    meta_path = os.path.join(a.runs_dir, run_id + ".meta.json")
    csv_path = os.path.join(a.runs_dir, run_id + ".samples.csv")
    meta = {"run_id": run_id, "kind": "belt_power", "start": iso(), "end": None, "status": "running",
            "rpm_units": RPM_UNITS, "operator_note": a.note, "cube_set": "none (empty belt)",
            "power_formula": "W = |belt INA260 mA| x belt INA260 bus mV / 1e6 (belt motor supply), as autoen belt_run",
            "plan": {"path": os.path.abspath(a.plan), "sha256": sha256(a.plan), "seconds": seconds, "settle_s": settle},
            "script_sha256": sha256(os.path.abspath(__file__)),
            "autoen": {"path": os.path.abspath(AUTOEN), "sha256": sha256(AUTOEN), "used_for": "Link + belt defaults"},
            "belt_settings_sent": settings, "arm": "not used (autoen not running; servos not commanded)", "steps": []}
    port = a.port or find_serial_port()
    print(f"\nMega port {port} (opening it resets the Mega; wait a few s)")
    link = Link(port)
    status = "finished"
    try:
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=SAMPLE_COLS)
            writer.writeheader()
            link.belt_cmd("PING", timeout=3.0)
            link.belt_cmd("STOP")                           # clear a latched fault, belt coasts
            for cmd in settings:
                link.belt_cmd(cmd)
            link.belt_cmd("ZERO")
            meta["mega_boot_lines"] = link.boot_lines[-5:]
            write_json_atomic(meta_path, meta)
            print(f"belt ready. session {run_id}")
            for s in steps:
                rpm = float(s["rpm"])
                while True:
                    with link.slock:
                        b = dict(link.belt or {})
                    if not b.get("ir"):
                        break
                    if input("  the IR sees something: clear the belt, Enter = check again, q = quit: ").strip() == "q":
                        raise KeyboardInterrupt
                what = "belt BRAKED (still)" if rpm == 0 else f"belt runs EMPTY at {rpm:g} rpm"
                c = input(f"\nstep {s['step']}/{len(steps)}: {what} for {seconds:g} s. Enter = run, s = skip, q = quit: ")
                if c.strip().lower() == "q":
                    status = "quit"
                    break
                if c.strip().lower() == "s":
                    meta["steps"].append({"step": s["step"], "rpm_set": rpm, "status": "skipped"})
                    write_json_atomic(meta_path, meta)
                    continue
                res = run_step(link, s, seconds, settle, writer)
                f.flush()
                meta["steps"].append(res)
                write_json_atomic(meta_path, meta)
                st = res["stats"]
                print(f"  {res['status']}{' - ' + res['abort_reason'] if res['abort_reason'] else ''}: "
                      f"rpm {st['rpm']['mean']} ± {st['rpm']['std']}, W {st['W']['mean']} ± {st['W']['std']} "
                      f"(n {st['W']['n']}), V {(st['mV']['mean'] or 0) / 1000:.2f}")
                if (res["abort_reason"] or "").startswith("FAULT"):    # a fault repeats on every step: stop here
                    print(f"  STOPPED: belt {res['abort_reason']} = {FAULTS.get(res['abort_reason'], '?')}")
                    status = "fault"
                    break
    except KeyboardInterrupt:
        status = "interrupted"
    except Exception as e:                                  # noqa: BLE001
        status = "error"
        meta["error"] = f"{type(e).__name__}: {e}"
        print("STOPPED:", meta["error"])
    finally:
        link.close()                                        # sends X,STOP: never leaves the belt running
        meta["status"], meta["end"] = status, iso()
        write_json_atomic(meta_path, meta)
        print(f"\n{run_id}: {status}\nmeta:    {meta_path}\nsamples: {csv_path}")
    return 0 if status == "finished" else 1


if __name__ == "__main__":
    sys.exit(main())
