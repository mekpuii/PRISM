#!/usr/bin/env python3
"""energy_per_cube.py - joules and seconds per cube from record_run.py files, one table row per file.

Usage:  python3 energy_per_cube.py runs/run_*.jsonl
        python3 energy_per_cube.py runs/*.jsonl --csv table.csv

Window: from the END of the first finished cycle to the END of the last one, so start-up
(feeding the first block, waiting for the operator) is excluded and exactly N-1 full cycles are counted.
Arm power  = sum over joints of volt * cur * 6.5 mA   (needs "volt" in arm/telemetry; 12 V if missing)
Belt power = belt voltage * belt current
Both are integrated with a 0.5 s cap per sample gap, so a dropout is not counted as time at that power.
The arm speed factor is read from arm/state ("speed_scale"); carry_speed is not logged, put it in --note.
"""
import argparse
import csv
import json
import statistics as st

MA_PER_COUNT = 6.5            # STS3215 current register unit (autoen_node.py MA_PER_COUNT)
MAX_DT = 0.5                  # s; longest gap that still counts as "the same power"
MOVING_TICKS_S = 30.0         # a joint faster than this means the arm is moving


def load(path):
    """-> (header, rows) with only live (non-retained) dict messages that carry rt/topic/data."""
    hdr, rows = {}, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if "header" in o:
                hdr = o["header"]
            elif "topic" in o and isinstance(o.get("data"), dict) and not o.get("retained"):
                rows.append(o)
    rows.sort(key=lambda r: r["rt"])
    return hdr, rows


def arm_power(d):
    """(watts, moving) for one arm/telemetry sample."""
    j = list(d.get("joints", {}).values())
    if not j:
        return None
    volt = st.mean(x.get("volt", 12.0) for x in j)
    watts = volt * sum(x.get("cur", 0) for x in j) * MA_PER_COUNT / 1000.0
    moving = any(abs(x.get("speed", 0)) > MOVING_TICKS_S for x in j)
    return watts, moving


def integrate(samples, t0, t1):
    """Sum of value * dt over [t0, t1] for [(t, value)] (step hold, dt capped)."""
    e = 0.0
    for (ta, v), (tb, _) in zip(samples, samples[1:]):
        if ta >= t0 and ta <= t1:
            e += v * min(tb - ta, MAX_DT, t1 - ta)
    return e


def analyse(path):
    hdr, rows = load(path)
    cyc = [r for r in rows if r["topic"] == "autoen/cycle"]
    if len(cyc) < 2:
        return None
    t0, t1 = cyc[0]["rt"], cyc[-1]["rt"]
    n = len(cyc) - 1
    window = [r["data"] for r in cyc[1:]]
    arm = [(r["rt"], arm_power(r["data"])) for r in rows if r["topic"] == "arm/telemetry"]
    arm = [(t, v) for t, v in arm if v]
    belt = [(r["rt"], r["data"]["voltage"] * r["data"]["current"] / 1000.0) for r in rows
            if r["topic"] == "belt/telemetry"
            and all(isinstance(r["data"].get(k), (int, float)) for k in ("voltage", "current"))]
    e_arm = integrate([(t, v[0]) for t, v in arm], t0, t1)
    e_arm_mv = integrate([(t, v[0] if v[1] else 0.0) for t, v in arm], t0, t1)
    e_belt = integrate(belt, t0, t1)
    scales = [r["data"].get("speed_scale") for r in rows if r["topic"] == "arm/state" and r["data"].get("speed_scale")]
    secs = [c.get("secs") for c in window if isinstance(c.get("secs"), (int, float))]
    return {
        "file": path.split("/")[-1], "rpm_set": hdr.get("rpm_set"),
        "arm_speed": st.mode(scales) if scales else None, "note": hdr.get("note", ""),
        "cycles": n, "ok": sum(1 for c in window if str(c.get("result")).upper() == "OK"),
        "s_per_cube": (t1 - t0) / n, "cube_per_min": 60 * n / (t1 - t0),
        "arm_J": e_arm / n, "arm_moving_J": e_arm_mv / n, "belt_J": e_belt / n,
        "total_J": (e_arm + e_belt) / n, "avg_W": (e_arm + e_belt) / (t1 - t0),
        "cycle_s_median": st.median(secs) if secs else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--csv")
    a = ap.parse_args()
    out = [r for r in (analyse(p) for p in a.files) if r]
    print(f"{'file':<22}{'rpm':>5}{'armspd':>7}{'n':>4}{'ok':>4}{'s/cube':>8}{'cube/min':>9}"
          f"{'armJ':>7}{'mvJ':>6}{'beltJ':>7}{'totJ':>7}{'avgW':>6}")
    for r in out:
        print(f"{r['file'][:21]:<22}{r['rpm_set'] or 0:>5.0f}{r['arm_speed'] or 0:>7.2f}{r['cycles']:>4}{r['ok']:>4}"
              f"{r['s_per_cube']:>8.1f}{r['cube_per_min']:>9.2f}{r['arm_J']:>7.1f}{r['arm_moving_J']:>6.1f}"
              f"{r['belt_J']:>7.1f}{r['total_J']:>7.1f}{r['avg_W']:>6.2f}")
    print("\nCompare rows by total J/cube and cube/min. A row with ok < n timed out or failed: do not trust its J/cube.")
    if a.csv and out:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(out[0]))
            w.writeheader()
            w.writerows(out)
        print("wrote", a.csv)


if __name__ == "__main__":
    main()
