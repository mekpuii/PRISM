#!/usr/bin/env python3
"""summarize_run.py - turn a record_run.py file into the numbers optimize.py needs.

Usage:  python3 summarize_run.py runs/run3.jsonl [--mm-per-px 0.95] [--json out.json]

Answers, in order:
  1. BELT SPEED: measured mm/s per rpm vs theory (165 mm / 60 s = 2.75). This resolves the ~10x gap.
  2. BELT POWER: V x I per rpm (W) and the share of time at each rpm.
  3. ARM: time moving vs idle, mean total servo current, cycle times, throughput.
Also reads a raw camera log (like run1.jsonl: lines with "tracks"), but then only section 1
works, and only if you pass --rpm.
"""
import argparse
import json
import statistics as st
import sys

THEORY_MM_PER_RPM = 165.0 / 60.0
MA_PER_COUNT = 6.5
ARM_MOVING_TICKS_S = 30.0      # a joint faster than this counts as moving (ticks/s, ~2.6 deg/s)
MIN_HITS = 4


def load(path, rpm_default):
    """Return (header, rows). Rows are normalised to {rt, topic, data}."""
    header, rows = {"rpm_set": rpm_default}, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if "header" in o:
                header.update({k: v for k, v in o["header"].items() if v is not None})
            elif "topic" in o and "rt" in o:
                if isinstance(o["data"], dict) and not o.get("retained"):
                    rows.append(o)
                elif isinstance(o["data"], dict) and o["topic"] in ("arm/state", "autoen/state"):
                    rows.append(o)
            elif "tracks" in o:                                  # raw camera line
                rows.append({"rt": o["ts"], "topic": "camera/perception", "data": o})
    rows.sort(key=lambda r: r["rt"])
    return header, rows


def by_topic(rows, topic):
    return [r for r in rows if r["topic"] == topic]


def median(x):
    return st.median(x) if x else None


# ---------------------------------------------------------------- 1. belt speed
def belt_speed(rows, mm_per_px, rpm_fixed):
    """Per-rpm bin: median cube speed in mm/s from consecutive camera samples of the same track."""
    belt = [(r["rt"], r["data"].get("velocity")) for r in by_topic(rows, "belt/telemetry")
            if isinstance(r["data"].get("velocity"), (int, float))]
    bts = [b[0] for b in belt]

    def rpm_at(t):
        if rpm_fixed is not None and not belt:
            return rpm_fixed
        if not belt:
            return None
        import bisect
        i = bisect.bisect_right(bts, t) - 1
        return belt[i][1] if i >= 0 and t - bts[i] < 1.5 else None

    last, bins = {}, {}
    for r in by_topic(rows, "camera/perception"):
        d = r["data"]
        for tr in d.get("tracks", []):
            if int(tr.get("hits") or 0) < MIN_HITS:
                continue
            cid, u, t = tr["id"], tr["u"], r["rt"]
            if cid in last:
                u0, t0 = last[cid]
                dt = t - t0
                if 0.2 < dt < 1.5:
                    v = abs(u - u0) / dt                     # px/s along the belt axis
                    if 3.0 < v < 400.0:                      # drop stationary (cube waiting) and jumps
                        rpm = rpm_at(t)
                        if rpm is not None and rpm > 0.5:
                            bins.setdefault(round(rpm), []).append(v * mm_per_px)
            last[cid] = (u, t)
    out = {}
    for rpm, v in sorted(bins.items()):
        mm_s = median(v)
        out[rpm] = {"n": len(v), "mm_s": mm_s, "mm_s_per_rpm": mm_s / rpm,
                    "ratio_vs_theory": mm_s / rpm / THEORY_MM_PER_RPM}
    return out


# ---------------------------------------------------------------- 2. belt power
def belt_power(rows):
    """Time-weighted mean of V*I per rpm bin."""
    b = [r for r in by_topic(rows, "belt/telemetry")
         if all(isinstance(r["data"].get(k), (int, float)) for k in ("velocity", "current", "voltage"))]
    bins, total = {}, 0.0
    for i in range(len(b) - 1):
        dt = min(b[i + 1]["rt"] - b[i]["rt"], 1.0)          # cap: a gap is not "time at this rpm"
        d = b[i]["data"]
        k = round(d["velocity"])
        w = d["voltage"] * d["current"] / 1000.0            # V * mA -> W
        e = bins.setdefault(k, {"t": 0.0, "Wt": 0.0, "I": 0.0})
        e["t"] += dt
        e["Wt"] += w * dt
        e["I"] += d["current"] * dt
        total += dt
    return {k: {"time_s": e["t"], "share": e["t"] / total if total else 0,
                "mean_W": e["Wt"] / e["t"] if e["t"] else 0, "mean_mA": e["I"] / e["t"] if e["t"] else 0}
            for k, e in sorted(bins.items())}, total


# ---------------------------------------------------------------- 3. arm
def arm_stats(rows):
    tel = by_topic(rows, "arm/telemetry")
    moving_t = idle_t = 0.0
    cur_sum, pwr_sum = [], []
    for i in range(len(tel) - 1):
        dt = min(tel[i + 1]["rt"] - tel[i]["rt"], 0.5)
        j = tel[i]["data"].get("joints", {})
        if not j:
            continue
        mv = any(abs(v.get("speed", 0)) > ARM_MOVING_TICKS_S for v in j.values())
        if mv:
            moving_t += dt
        else:
            idle_t += dt
        cur_sum.append((mv, sum(v.get("cur", 0) for v in j.values()) * MA_PER_COUNT))
        if all(v.get("volt") is not None for v in j.values()):
            pwr_sum.append((mv, sum(v.get("cur", 0) * MA_PER_COUNT / 1000.0 * v["volt"] for v in j.values())))
    mv_i = [c for m, c in cur_sum if m]
    id_i = [c for m, c in cur_sum if not m]
    mv_w = [w for m, w in pwr_sum if m]
    id_w = [w for m, w in pwr_sum if not m]
    cyc = [r["data"] for r in by_topic(rows, "autoen/cycle")]
    secs = [c["secs"] for c in cyc if isinstance(c.get("secs"), (int, float))]
    ok = [c for c in cyc if str(c.get("result", "")).lower() in ("ok", "success", "done", "placed")]
    span = (rows[-1]["rt"] - rows[0]["rt"]) if rows else 0
    return {
        "moving_s": moving_t, "idle_s": idle_t,
        "moving_frac": moving_t / (moving_t + idle_t) if moving_t + idle_t else None,
        "mean_mA_moving": st.mean(mv_i) if mv_i else None,
        "mean_mA_idle": st.mean(id_i) if id_i else None,
        "mean_W_moving": st.mean(mv_w) if mv_w else None,
        "mean_W_idle": st.mean(id_w) if id_w else None,
        "cycles": len(cyc), "cycles_ok": len(ok),
        "cycle_s_median": median(secs), "cycle_s_min": min(secs) if secs else None,
        "cycle_s_max": max(secs) if secs else None,
        "cycles_per_min": len(cyc) / (span / 60) if span > 0 else None,
        "results": sorted({str(c.get("result")) for c in cyc}),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--mm-per-px", type=float, default=0.95)
    ap.add_argument("--rpm", type=float, default=None, help="only for raw camera logs without belt data")
    ap.add_argument("--json")
    a = ap.parse_args()

    hdr, rows = load(a.path, a.rpm)
    if not rows:
        sys.exit("no usable rows")
    span = rows[-1]["rt"] - rows[0]["rt"]
    topics = {}
    for r in rows:
        topics[r["topic"]] = topics.get(r["topic"], 0) + 1
    print(f"file: {a.path}   span: {span:.0f}s   rpm_set(header): {hdr.get('rpm_set')}   note: {hdr.get('note','')}")
    print("messages:", ", ".join(f"{k}={v}" for k, v in sorted(topics.items())))

    print("\n== 1. BELT SPEED (cube mm/s per belt rpm) ==")
    sp = belt_speed(rows, a.mm_per_px, a.rpm if a.rpm is not None else hdr.get("rpm_set"))
    if not sp:
        print("  no moving-cube samples matched a belt rpm (need belt/telemetry or --rpm)")
    for rpm, e in sp.items():
        print(f"  {rpm:>4} rpm  n={e['n']:4d}  {e['mm_s']:6.1f} mm/s  -> {e['mm_s_per_rpm']:.2f} mm/s/rpm"
              f"   (theory {THEORY_MM_PER_RPM:.2f}, ratio {e['ratio_vs_theory']:.2f})")
    if sp:
        r = [e["ratio_vs_theory"] for e in sp.values()]
        print("  -> ratio ~1: pulley math right. ratio far below 1 (e.g. 0.1-0.3): velocity is NOT pulley rpm at the belt (gear, encoder or units).")
        print("  -> ratio varying with rpm: belt is not linear in rpm; optimizer needs the measured table, not a constant.")

    print("\n== 2. BELT POWER (V x I) ==")
    pw, tot = belt_power(rows)
    if not pw:
        print("  no belt/telemetry")
    for k, e in pw.items():
        print(f"  {k:>4} rpm  {e['share']*100:5.1f}% of time   {e['mean_W']:6.2f} W   {e['mean_mA']:7.0f} mA")

    print("\n== 3. ARM ==")
    ar = arm_stats(rows)
    if ar["moving_frac"] is None:
        print("  no arm/telemetry")
    else:
        print(f"  moving {ar['moving_s']:.0f}s / idle {ar['idle_s']:.0f}s  (moving {ar['moving_frac']*100:.0f}%)")
        print(f"  total servo current  moving: {ar['mean_mA_moving']:.0f} mA   idle: {ar['mean_mA_idle']:.0f} mA"
              if ar["mean_mA_moving"] and ar["mean_mA_idle"] else "  current: not enough data")
        if ar["mean_W_moving"] is not None and ar["mean_W_idle"] is not None:
            print(f"  total servo power     moving: {ar['mean_W_moving']:.2f} W   idle: {ar['mean_W_idle']:.2f} W")
        else:
            print("  (no servo voltage in this recording: watts NOT known, only current)")
    print(f"  cycles: {ar['cycles']} (ok {ar['cycles_ok']}), results seen: {ar['results']}")
    if ar["cycle_s_median"]:
        print(f"  cycle time  median {ar['cycle_s_median']:.1f}s  min {ar['cycle_s_min']:.1f}  max {ar['cycle_s_max']:.1f}"
              f"   -> {ar['cycles_per_min']:.2f} cycles/min")
        print("  If the arm cycle is the bottleneck, belt rpm above (cube supply = cycle rate) wastes energy.")

    if a.json:
        with open(a.json, "w") as f:
            json.dump({"header": hdr, "belt_speed": sp, "belt_power": pw, "arm": ar}, f, indent=1)
        print("\nwrote", a.json)


if __name__ == "__main__":
    main()
