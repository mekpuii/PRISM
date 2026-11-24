# report.py - daily summary of the orchestrator (PRISM): what the cell did, what it cost, what it learned.
#
# USAGE  (orch-env, from orchestrator/)
#   python3 report.py                today
#   python3 report.py 2026-10-26     that day
#   python3 report.py --week         the last 7 days, one line each
# Reads logs/orch_*.jsonl (LIVE runs only; shadow runs are listed but not counted), forecast_state.json and
# learned_state.json. Writes reports/report_<day>.md as well as printing it.
#
# Energy comes from the "energy" records main.py writes every minute (meter.py: belt INA260 + servo current sum, the
# motors only, not the Pi / Mega / camera). Runs before 2026-10-26 15:30 have no energy records.

import argparse
import glob
import json
import os
from datetime import date, datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")
REPORT_DIR = os.path.join(HERE, "reports")
EMPTY_FEED_ALONE_J = 150.0     # autoen alone at an empty feeder: 3 empty cycles (~50 J each), then it stops (2026-10-26)


def read_day(day):
    """All records of the LIVE runs that started on `day` (YYYY-MM-DD), plus the shadow log names."""
    live, shadow = [], []
    for path in sorted(glob.glob(os.path.join(LOG_DIR, f"orch_{day.replace('-', '')}_*.jsonl"))):
        recs = []
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    recs.append(json.loads(line))
                except ValueError:
                    pass                                    # a half-written last line of a run still going
        if recs and recs[0].get("kind") == "start" and recs[0].get("shadow"):
            shadow.append(os.path.basename(path))
        else:
            live.append((os.path.basename(path), recs))
    return live, shadow


def summarize(live):
    s = {"runs": len(live), "cycles": {}, "energy": {}, "idles": 0, "probes": 0, "probe_hits": 0, "alerts": [],
         "first": None, "last": None}
    s["counted_from_plans"] = 0
    s["sort"] = {"right_bin": 0, "wrong_bin": 0, "unknown": 0}
    for _, recs in live:
        last_sort = {}
        for r in recs:                                          # each run's last plan holds its sort totals
            if r.get("kind") == "plan" and (r.get("score") or {}).get("sort"):
                last_sort = r["score"]["sort"]
        for k in s["sort"]:
            s["sort"][k] += last_sort.get(k, 0)
        last_probe = False
        if not any(r.get("kind") == "cycle" for r in recs):     # runs before 2026-10-26 15:30 log no cycles:
            oks = [(r.get("score") or {}).get("ok_live", 0) for r in recs if r.get("kind") == "plan"]
            s["counted_from_plans"] += max(oks or [0])            # use the optimizer's own OK counter instead
        for r in recs:
            iso = r.get("iso", "")
            s["first"] = min(s["first"] or iso, iso)
            s["last"] = max(s["last"] or iso, iso)
            k = r.get("kind")
            if k == "cycle":
                res = str(r.get("result"))
                s["cycles"][res] = s["cycles"].get(res, 0) + 1
                if last_probe and res == "OK":
                    s["probe_hits"] += 1
                last_probe = False
            elif k == "energy":
                for mode, e in (r.get("by_mode") or {}).items():
                    acc = s["energy"].setdefault(mode, {"belt_J": 0.0, "arm_J": 0.0, "s": 0.0})
                    for key in acc:
                        acc[key] += e.get(key, 0.0)
            elif k == "plan" and r.get("action"):
                if r["action"] == "idle":
                    s["idles"] += 1
                elif r["action"] == "resume" and str(r.get("reason", "")).startswith("probe"):
                    s["probes"] += 1
                    last_probe = True
                elif r["action"] == "pause" and r.get("mode") == "ALERT":
                    s["alerts"].append(f"{iso[11:19]} {r.get('reason')}")
    return s


def fmt_day(day, s, shadow):
    out = [f"# PRISM cell report {day}", ""]
    if not s["runs"]:
        out.append("No LIVE optimizer run this day." + (f" (shadow runs: {len(shadow)})" if shadow else ""))
        return "\n".join(out)
    c = s["cycles"]
    ok = c.get("OK", 0) + s["counted_from_plans"]
    e = s["energy"]
    tot = {k: sum(m.get(k, 0.0) for m in e.values()) for k in ("belt_J", "arm_J", "s")}
    run_s = e.get("RUN", {}).get("s", 0.0)
    idle = e.get("IDLE", {"belt_J": 0.0, "arm_J": 0.0, "s": 0.0})
    probe = e.get("PROBE", {"belt_J": 0.0, "arm_J": 0.0, "s": 0.0})
    out += [f"{s['runs']} live run(s), {s['first'][11:16]}–{s['last'][11:16]}" + (f"; {len(shadow)} shadow run(s) not counted" if shadow else ""),
            "",
            "## Work",
            f"- cubes picked: **{ok}**  (logged cycles: " + (", ".join(f"{k} {v}" for k, v in sorted(c.items())) or "none")
            + (f"; {s['counted_from_plans']} from older runs' counters" if s["counted_from_plans"] else "") + ")",
            f"- running time: {run_s / 60:.1f} min" + (f" → **{60 * ok / run_s:.2f} cubes/min**" if run_s else ""),
            f"- feeder ran empty: {s['idles']} time(s); auto probes: {s['probes']} ({s['probe_hits']} found cubes)",
            "- colour sorting (orange left, white right): " + (
                f"{s['sort']['right_bin']} right bin, **{s['sort']['wrong_bin']} wrong bin**, {s['sort']['unknown']} colour not seen"
                if sum(s["sort"].values()) else "no sorted cubes logged"),
            "",
            "## Energy (motors: belt + arm)"]
    if tot["s"]:
        out += [f"- total: **{(tot['belt_J'] + tot['arm_J']) / 1000:.2f} kJ** (belt {tot['belt_J']:.0f} J, arm {tot['arm_J']:.0f} J)"
                f" over {tot['s'] / 3600:.2f} h"]
        if ok:
            run = e.get("RUN", {"belt_J": 0.0, "arm_J": 0.0})
            out.append(f"- per cube while running: **{(run['belt_J'] + run['arm_J']) / ok:.1f} J**; "
                       f"everything included: {(tot['belt_J'] + tot['arm_J']) / ok:.1f} J")
        if idle["s"]:
            out.append(f"- idle: {idle['s'] / 60:.1f} min at **{(idle['belt_J'] + idle['arm_J']) / idle['s']:.3f} W**")
        if probe["s"]:
            out.append(f"- probing: {(probe['belt_J'] + probe['arm_J']):.0f} J over {probe['s']:.0f} s")
        if s["idles"]:
            spent = sum(m["belt_J"] + m["arm_J"] for k, m in e.items() if k in ("IDLE", "PROBE"))
            out.append(f"- estimate: autoen alone would have spent ~{s['idles'] * EMPTY_FEED_ALONE_J:.0f} J on the "
                       f"{s['idles']} empty feeder(s) and stopped each time; the optimizer spent {spent:.0f} J idle + probing "
                       "and kept going")
        for mode in sorted(set(e) - {"RUN", "IDLE", "PROBE"}):
            m = e[mode]
            if m["belt_J"] + m["arm_J"] > 1:
                out.append(f"- {mode}: {m['belt_J'] + m['arm_J']:.0f} J over {m['s'] / 60:.1f} min")
    else:
        out.append("- no energy records (runs before the meter existed)")
    out += ["", "## Problems"]
    out += [f"- ALERT {a}" for a in s["alerts"]] or ["- none"]
    return "\n".join(out)


def fmt_learned():
    out = ["", "## What it has learned"]
    try:
        with open(os.path.join(HERE, "forecast_state.json"), encoding="utf-8") as f:
            fc = json.load(f)
        gaps = fc.get("gaps", [])
        refills = [g for g in gaps if g.get("refill")]
        out.append(f"- forecast: {len(gaps)} idle periods, {len(refills)} ended by a refill"
                   + (f" (median wait {sorted(g['secs'] for g in refills)[len(refills) // 2]:.0f} s)" if refills else ""))
        busy = sorted(fc.get("bins", {}).items(), key=lambda kv: -kv[1]["refills"])[:3]
        days = "Mon Tue Wed Thu Fri Sat Sun".split()
        if busy:
            out.append("- busiest refill hours: " + ", ".join(
                f"{days[int(k.split('-')[0])]} {k.split('-')[1]}:00 ({v['refills']})" for k, v in busy))
    except (OSError, ValueError):
        out.append("- forecast: nothing learned yet")
    try:
        with open(os.path.join(HERE, "learned_state.json"), encoding="utf-8") as f:
            rows = json.load(f).get("configs", [])
        out += [f"- settings {r['rpm']:g} rpm / arm {r['arm_speed']:g}: {r['ok']}/{r['n']} OK live" for r in rows]
    except (OSError, ValueError):
        out.append("- settings: no live counts yet")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="daily summary of the PRISM orchestrator")
    ap.add_argument("day", nargs="?", default=date.today().isoformat(), help="YYYY-MM-DD (default today)")
    ap.add_argument("--week", action="store_true", help="one line per day for the last 7 days")
    a = ap.parse_args(argv)
    if a.week:
        end = datetime.strptime(a.day, "%Y-%m-%d").date()
        print(f"{'day':<11} {'cubes':>5} {'cubes/min':>9} {'J/cube':>7} {'idle W':>7} {'empty':>5} {'probes':>6} {'alerts':>6}")
        for i in range(6, -1, -1):
            d = (end - timedelta(days=i)).isoformat()
            s = summarize(read_day(d)[0])
            ok, e = s["cycles"].get("OK", 0) + s["counted_from_plans"], s["energy"]
            run, idle = e.get("RUN", {}), e.get("IDLE", {})
            cpm = f"{60 * ok / run['s']:.2f}" if run.get("s") else "-"
            jpc = f"{(run['belt_J'] + run['arm_J']) / ok:.1f}" if ok and run else "-"
            iw = f"{(idle['belt_J'] + idle['arm_J']) / idle['s']:.3f}" if idle.get("s") else "-"
            print(f"{d:<11} {ok:>5} {cpm:>9} {jpc:>7} {iw:>7} {s['idles']:>5} {s['probes']:>6} {len(s['alerts']):>6}")
        return 0
    live, shadow = read_day(a.day)
    text = fmt_day(a.day, summarize(live), shadow) + "\n" + fmt_learned() + "\n"
    os.makedirs(REPORT_DIR, exist_ok=True)
    path = os.path.join(REPORT_DIR, f"report_{a.day}.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(text)
    print(f"(saved {path})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
