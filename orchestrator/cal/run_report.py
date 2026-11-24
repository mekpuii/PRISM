#!/usr/bin/env python3
"""run_report.py - one report per run: temperature and power of everything, written when the run ends (2026-11-12).

The operator: "make PRISM keep a log file of the temperature, the power and everything about them after each experiment".
autoen starts this by itself when a run ends; by hand:
    python3 orchestrator/cal/run_report.py                   the newest run
    python3 orchestrator/cal/run_report.py 20261112_200041   that run (the stamp of autoencoder/riff_logs/autoen_raw_*.csv)
Writes  run_reports/run_<stamp>.md         (to read: summary, every device, warnings)
        run_reports/run_<stamp>_devices.csv (one row per device, for a spreadsheet)
        run_reports/runs.csv                 (one row per run - the history to compare runs)
Reads only: autoen_raw_<stamp>.csv (servos + belt: temperature, current, voltage, ~4 per second), autoen_cycles_<stamp>.csv,
autoen_tx_<stamp>.log (emergency grips, rest probes, drifts), run_logs/pi_<day>.csv (the Pi, written by the dashboard).
Energy = the same method as the information book (V x I integrated over the whole run, gaps capped at 0.5 s).
Only the standard library: it runs in any of PRISM's Pythons.
"""
import csv
import datetime as D
import glob
import os
import statistics as st
import sys

PRISM = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RIFF = os.path.join(PRISM, "autoencoder", "riff_logs")
OUT = os.path.join(PRISM, "run_reports")
PI_LOGS = os.path.join(PRISM, "run_logs")
MAX_DT = 0.5
# the limits behind the words (the same ones the dashboard's Machine page uses)
SERVO_WARM, SERVO_HOT, SERVO_MA = 55.0, 65.0, 1500.0
BELT_WARM, BELT_HOT = 45.0, 60.0
PI_WARM, PI_HOT = 70.0, 80.0
RAIL_OK, RAIL_BAD = 11.5, 10.5
JOINTS = {"J1": "base (shoulder pan)", "J2": "shoulder lift", "J3": "elbow", "J4": "wrist flex", "J5": "wrist roll",
          "J6": "gripper"}


def num(x):
    try:
        v = float(x)
        return v if v == v else None                       # NaN -> None
    except (TypeError, ValueError):
        return None


def ts(s):
    try:
        return D.datetime.fromisoformat(s).timestamp()
    except (TypeError, ValueError):
        return None


def stats(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None
    return {"min": min(v), "avg": st.fmean(v), "max": max(v), "first": v[0], "n": len(v)}


def level(x, warm, hot):
    return "-" if x is None else "HOT" if x >= hot else "warm" if x >= warm else "ok"


def f(x, d=1):
    return "-" if x is None else f"{x:.{d}f}"


def newest_stamp():
    raws = sorted(glob.glob(os.path.join(RIFF, "autoen_raw_*.csv")), key=os.path.getmtime)
    for p in reversed(raws):
        if os.path.getsize(p) > 2000:                       # (a run that never started has a header only)
            return os.path.basename(p)[len("autoen_raw_"):-4]
    raise SystemExit("no autoen run found in " + RIFF)


def read_raw(stamp):
    rows = []
    with open(os.path.join(RIFF, f"autoen_raw_{stamp}.csv"), encoding="utf-8", errors="replace") as fh:
        for r in csv.DictReader(line for line in fh if "\x00" not in line):
            t = ts(r.get("time"))
            if t is not None:
                rows.append((t, r))
    return rows


def pi_window(t0, t1):
    rows = []
    days = {D.datetime.fromtimestamp(t).strftime("%Y%m%d") for t in (t0, t1)}
    for day in sorted(days):
        p = os.path.join(PI_LOGS, f"pi_{day}.csv")
        if not os.path.exists(p):
            continue
        with open(p, encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                t = num(r.get("t"))
                if t is not None and t0 - 5 <= t <= t1 + 5:
                    rows.append(r)
    return rows


def report(stamp):
    rows = read_raw(stamp)
    if len(rows) < 5:
        raise SystemExit(f"run {stamp}: too little data ({len(rows)} rows)")
    t0, t1 = rows[0][0], rows[-1][0]
    dur = t1 - t0
    dev = {}
    energy = {}
    rail_low_s, prev, covered = 0.0, None, 0.0
    for t, r in rows:
        dt = 0.0 if prev is None else min(MAX_DT, max(0.0, t - prev))
        prev = t
        covered += dt
        for name in list(JOINTS) + ["Belt"]:
            d = dev.setdefault(name, {"temp": [], "current": [], "voltage": [], "power": []})
            tc, i, v = num(r.get(f"{name}_temp")), num(r.get(f"{name}_current")), num(r.get(f"{name}_voltage"))
            d["temp"].append(tc)
            d["current"].append(abs(i) if i is not None else None)
            d["voltage"].append(v)
            p = None if i is None else abs(i) / 1000.0 * (v if v is not None else 12.0)
            d["power"].append(p)
            energy[name] = energy.get(name, 0.0) + (p or 0.0) * dt
        bv = num(r.get("Belt_voltage"))
        if bv is not None and bv < RAIL_OK:
            rail_low_s += dt

    cyc_path = os.path.join(RIFF, f"autoen_cycles_{stamp}.csv")
    cycles = list(csv.DictReader(open(cyc_path, encoding="utf-8", errors="replace"))) if os.path.exists(cyc_path) else []
    ok = [c for c in cycles if c.get("result") == "OK"]
    secs = [num(c.get("seconds")) for c in ok[1:]] or [num(c.get("seconds")) for c in ok]
    secs = [s for s in secs if s is not None]
    results = {}
    for c in cycles:
        results[c.get("result")] = results.get(c.get("result"), 0) + 1
    drops = {}
    for c in ok:
        drops[c.get("drop") or "?"] = drops.get(c.get("drop") or "?", 0) + 1
    tx_path = os.path.join(RIFF, f"autoen_tx_{stamp}.log")
    tx = open(tx_path, encoding="utf-8", errors="replace").read().splitlines() if os.path.exists(tx_path) else []
    events = {
        "emergency grips": sum("EMERGENCY GRIP:" in l for l in tx),
        "  held by the hand camera": sum("emergency grip: cube held" in l for l in tx),
        "  failed (paused)": sum("phase RESCUE_FAILED" in l for l in tx),
        "rest probes (empty feeder)": sum("feeder probe" in l for l in tx),
        "cubes nobody fed (picked as they came)": sum("a cube nobody fed" in l for l in tx),
        "arm drifts (corrected)": sum("## DRIFT" in l for l in tx),
    }
    pi = pi_window(t0, t1)
    pis = {k: stats([num(r.get(k)) for r in pi]) for k in ("temp_c", "cpu_pct", "mem_pct", "mhz")}
    thr = sorted({fl for r in pi for fl in (r.get("throttled") or "").split("|") if fl})

    e_arm = sum(energy[j] for j in JOINTS)
    e_all = e_arm + energy["Belt"]
    n_ok = len(ok)
    warns = []
    lines = []
    for name in list(JOINTS) + ["Belt"]:
        d = dev[name]
        T, I, V, P = stats(d["temp"]), stats(d["current"]), stats(d["voltage"]), stats(d["power"])
        hot = level(T and T["max"], *(BELT_WARM, BELT_HOT) if name == "Belt" else (SERVO_WARM, SERVO_HOT))
        if hot in ("warm", "HOT"):
            warns.append(f"{name} temperature {T['max']:.0f} C ({hot})")
        if name != "Belt" and I and I["max"] > SERVO_MA:
            warns.append(f"{name} current peak {I['max']:.0f} mA (> {SERVO_MA:.0f})")
        lines.append({"device": name, "what": JOINTS.get(name, "belt motor + 12 V rail"),
                      "temp_start_c": T and round(T["first"], 1), "temp_avg_c": T and round(T["avg"], 1),
                      "temp_max_c": T and round(T["max"], 1), "temp_rise_c": T and round(T["max"] - T["first"], 1),
                      "temp_level": hot, "current_avg_ma": I and round(I["avg"]), "current_peak_ma": I and round(I["max"]),
                      "volt_min_v": V and round(V["min"], 2), "volt_avg_v": V and round(V["avg"], 2),
                      "power_avg_w": P and round(P["avg"], 3), "power_peak_w": P and round(P["max"], 2),
                      "energy_j": round(energy[name], 1)})
    belt_v = stats(dev["Belt"]["voltage"])
    if belt_v and belt_v["min"] < RAIL_BAD:
        warns.append(f"12 V rail fell to {belt_v['min']:.2f} V (below {RAIL_BAD} V)")
    if rail_low_s > 1.0:
        warns.append(f"12 V rail below {RAIL_OK} V for {rail_low_s:.0f} s in total")
    if pis["temp_c"] and pis["temp_c"]["max"] >= PI_WARM:
        warns.append(f"Raspberry Pi {pis['temp_c']['max']:.0f} C ({level(pis['temp_c']['max'], PI_WARM, PI_HOT)})")
    if thr:
        warns.append("Pi power: " + ", ".join(thr))
    if events["  failed (paused)"]:
        warns.append(f"{events['  failed (paused)']} emergency grip(s) failed")

    os.makedirs(OUT, exist_ok=True)
    start = D.datetime.fromtimestamp(t0)
    md = [f"# PRISM run {start:%Y-%m-%d %H:%M} - temperature and power",
          "",
          f"Run `{stamp}`: {start:%H:%M:%S} - {D.datetime.fromtimestamp(t1):%H:%M:%S} ({dur / 60:.1f} min), "
          f"{len(rows)} samples. Written {D.datetime.now():%Y-%m-%d %H:%M} by `orchestrator/cal/run_report.py`.",
          "",
          "## Summary",
          "",
          "| | |",
          "|---|---|",
          f"| Cubes picked | **{n_ok}** of {len(cycles)} cycles" + (f" ({', '.join(f'{k} {v}' for k, v in results.items() if k != 'OK')})"
                                                                      if len(results) > 1 else "") + " |",
          f"| Bins | " + ", ".join(f"{'left' if k == 'left' else 'right' if k == 'down' else k} {v}" for k, v in drops.items()) + " |",
          f"| Time per cube | {f(st.median(secs) if secs else None)} s (median) · {f(dur / n_ok if n_ok else None)} s (whole run / cubes) |",
          f"| Energy | {e_all:.0f} J in total: arm {e_arm:.0f} J, belt {energy['Belt']:.0f} J |",
          f"| **Energy per cube** | **{f(e_all / n_ok if n_ok else None)} J** |",
          f"| Average power | arm {f(e_arm / covered if covered else None, 2)} W, belt {f(energy['Belt'] / covered if covered else None, 2)} W"
          f" (samples cover {covered / 60:.1f} of {dur / 60:.1f} min; gaps over {MAX_DT} s are not counted) |",
          f"| 12 V rail | min {f(belt_v and belt_v['min'], 2)} V · avg {f(belt_v and belt_v['avg'], 2)} V · below {RAIL_OK} V for {rail_low_s:.0f} s |",
          f"| Raspberry Pi | " + (f"{f(pis['temp_c']['avg'])} C avg / {f(pis['temp_c']['max'])} C max · CPU {f(pis['cpu_pct'] and pis['cpu_pct']['avg'], 0)} % avg"
                                  f" / {f(pis['cpu_pct'] and pis['cpu_pct']['max'], 0)} % max · memory {f(pis['mem_pct'] and pis['mem_pct']['max'], 0)} % max"
                                  if pis["temp_c"] else "no Pi log for this time (the dashboard writes run_logs/pi_<day>.csv)") + " |",
          "",
          "## Verdict",
          "",
          ("**Warnings:**\n\n" + "\n".join(f"- {w}" for w in warns)) if warns else
          "**In good shape:** no over-temperature, no over-current, the 12 V rail held, the Pi stayed cool.",
          "",
          "## Every device",
          "",
          "| Device | | Temp start | Temp avg | **Temp max** | Rise | Current avg | Current peak | Volt min | Power avg | Energy |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for L in lines:
        md.append(f"| {L['device']} | {L['what']} | {f(L['temp_start_c'], 0)} C | {f(L['temp_avg_c'], 0)} C | "
                  f"**{f(L['temp_max_c'], 0)} C** {'' if L['temp_level'] in ('ok', '-') else '(' + L['temp_level'] + ')'} | "
                  f"{f(L['temp_rise_c'], 0)} C | {f(L['current_avg_ma'], 0)} mA | {f(L['current_peak_ma'], 0)} mA | "
                  f"{f(L['volt_min_v'], 2)} V | {f(L['power_avg_w'], 2)} W | {f(L['energy_j'], 0)} J |")
    md += ["",
           f"Limits: servos warm {SERVO_WARM:.0f} C / hot {SERVO_HOT:.0f} C, peak current {SERVO_MA:.0f} mA; belt motor "
           f"warm {BELT_WARM:.0f} C / hot {BELT_HOT:.0f} C; Pi warm {PI_WARM:.0f} C / hot {PI_HOT:.0f} C; 12 V rail "
           f"{RAIL_OK} V (pushes wait) / {RAIL_BAD} V (collapsing).",
           "",
           "## What happened",
           ""]
    md += [f"- {k.strip()}: {v}" if not k.startswith("  ") else f"  - {k.strip()}: {v}" for k, v in events.items()]
    md += ["", "Data: `autoencoder/riff_logs/autoen_raw_%s.csv` (every sample), `autoen_cycles_%s.csv` (every cycle)."
           % (stamp, stamp), ""]
    p_md = os.path.join(OUT, f"run_{stamp}.md")
    with open(p_md, "w", encoding="utf-8") as fh:
        fh.write("\n".join(md))
    with open(os.path.join(OUT, f"run_{stamp}_devices.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(lines[0]))
        w.writeheader()
        w.writerows(lines)
    hist = os.path.join(OUT, "runs.csv")
    row = {"run": stamp, "start": f"{start:%Y-%m-%d %H:%M:%S}", "minutes": round(dur / 60, 1), "cubes_ok": n_ok,
           "cycles": len(cycles), "s_per_cube_median": round(st.median(secs), 1) if secs else "",
           "energy_j": round(e_all), "j_per_cube": round(e_all / n_ok, 1) if n_ok else "",
           "arm_w": round(e_arm / covered, 2) if covered else "", "belt_w": round(energy["Belt"] / covered, 2) if covered else "",
           "servo_temp_max_c": max((L["temp_max_c"] or 0) for L in lines if L["device"] != "Belt"),
           "belt_temp_max_c": lines[-1]["temp_max_c"], "rail_min_v": belt_v and round(belt_v["min"], 2),
           "pi_temp_max_c": pis["temp_c"] and round(pis["temp_c"]["max"], 1),
           "pi_cpu_avg_pct": pis["cpu_pct"] and round(pis["cpu_pct"]["avg"]),
           "emergency_grips": events["emergency grips"], "warnings": len(warns)}
    old = []
    if os.path.exists(hist):
        old = [r for r in csv.DictReader(open(hist, encoding="utf-8")) if r.get("run") != stamp]
    with open(hist, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(row))
        w.writeheader()
        w.writerows([{k: r.get(k, "") for k in row} for r in old] + [row])
    return p_md, row, warns


def main():
    global RIFF, OUT
    args = sys.argv[1:]
    if "--riff" in args:                                    # autoen's own --log-dir (a simulator run: report stays there)
        i = args.index("--riff")
        d = os.path.abspath(args[i + 1])
        del args[i:i + 2]
        if d != os.path.abspath(RIFF):
            RIFF, OUT = d, os.path.join(os.path.dirname(d), "run_reports")
    stamp = args[0] if args else newest_stamp()
    p, row, warns = report(stamp)
    print(f"[run_report] {p}: {row['cubes_ok']} cubes, {row['j_per_cube']} J per cube, servos max "
          f"{row['servo_temp_max_c']} C, {len(warns)} warning(s)")


if __name__ == "__main__":
    main()
