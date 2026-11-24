# study_cases.py - what happens in which situation: adds up the case rows scene.py writes (one per cycle).
#
# USAGE  (orch-env, from orchestrator/)
#   python3 study_cases.py                  all days in cases/
#   python3 study_cases.py 20261028         one day
#   python3 study_cases.py --csv out.csv    also write the table
# Each table: one line per situation, with the cycles, how many ended OK, the mean cycle time and what went wrong.
# The rows themselves (cases/cases_<day>.csv) are a plain array: numpy / pandas / a spreadsheet read them directly.
import argparse
import csv
import glob
import os
from collections import Counter, defaultdict

from scene import CASES_DIR

GROUPS = {
    "light":        lambda r: r["light"] or "?",
    "cubes waiting": lambda r: "?" if r["backlog_max"] == "" else ("2+" if int(r["backlog_max"]) >= 2 else r["backlog_max"]),
    "touching cubes": lambda r: "yes" if r["train"] == "1" else "no",
    "stuck cube":   lambda r: "yes" if r["stuck_s"] not in ("", "0", "0.0") else "no",
    "setting":      lambda r: f"{float(r['rpm']):g} rpm / arm {float(r['arm_speed']):g}" if r["rpm"] else "?",
    "colour":       lambda r: r["colour"] or "not seen",
    "colour certainty": lambda r: "?" if not r["colour_share"] else
                    ("< 70 %" if float(r["colour_share"]) < 0.7 else "70-90 %" if float(r["colour_share"]) < 0.9 else ">= 90 %"),
}


def load(day=None):
    rows = []
    for path in sorted(glob.glob(os.path.join(CASES_DIR, f"cases_{day or '*'}.csv"))):
        with open(path, newline="", encoding="utf-8") as f:
            rows += list(csv.DictReader(f))
    return rows


def table(rows, key):
    by = defaultdict(list)
    for r in rows:
        by[key(r)].append(r)
    out = []
    for k, rs in sorted(by.items(), key=lambda kv: -len(kv[1])):
        ok = [r for r in rs if r["result"] == "OK"]
        secs = [float(r["secs"]) for r in ok if r["secs"]]
        bad = Counter(r["result"] for r in rs if r["result"] != "OK")
        out.append({"situation": k, "cycles": len(rs), "ok_pct": round(100 * len(ok) / len(rs)),
                    "mean_s": round(sum(secs) / len(secs), 1) if secs else None,
                    "problems": ", ".join(f"{n} {res}" for res, n in bad.most_common(3))})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="PRISM: cycle results per situation (cases/ from scene.py)")
    ap.add_argument("day", nargs="?", help="YYYYMMDD (default: every day)")
    ap.add_argument("--csv", help="also write every table to this file")
    a = ap.parse_args(argv)
    rows = load(a.day)
    if not rows:
        print(f"no case rows in {CASES_DIR} yet: they are written from the first cycle of a live run")
        return 0
    print(f"{len(rows)} cycles ({rows[0]['iso']} .. {rows[-1]['iso']})")
    flat = []
    for name, key in GROUPS.items():
        print(f"\n{name}")
        print(f"  {'situation':<22}{'cycles':>7}{'OK %':>7}{'mean s':>8}  problems")
        for t in table(rows, key):
            print(f"  {t['situation']:<22}{t['cycles']:>7}{t['ok_pct']:>7}{t['mean_s'] if t['mean_s'] else '-':>8}  "
                  f"{t['problems']}")
            flat.append({"group": name, **t})
    slips = [float(r["slip"]) for r in rows if r["slip"]]
    if slips:
        slips.sort()
        print(f"\nslip (cube speed / belt speed): median {slips[len(slips) // 2]:.2f} over {len(slips)} cycles "
              f"(1.00 = the cubes ride exactly with the belt)")
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, ["group", "situation", "cycles", "ok_pct", "mean_s", "problems"])
            w.writeheader()
            w.writerows(flat)
        print(f"\nwritten: {a.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
