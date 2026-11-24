# model.py - the learned model the optimizer runs on (PRISM orchestrator).
#
# WHAT THIS FILE DOES
#   Reads the experiment sessions in cal/runs/ (written by cal/experiment.py) and fits:
#     1. a CYCLE-TIME model, from the OK cycles:
#            secs = a + b / arm_speed + c / rpm
#        Arm moves take time ~ 1/speed; the cube's ride to the IR takes time ~ 1/belt speed.
#     2. a SUCCESS table: for every tested (rpm, arm_speed, carry_speed), how many cycles
#        worked, plus a cautious lower bound on the success rate (Wilson, one-sided 95 %).
#   The result is saved as model.json. optimize.py reads only model.json, never the runs.
#
# USAGE  (orch-env, from orchestrator/)
#   python3 model.py fit                 fit from the new-gripper sessions -> model.json
#   python3 model.py fit --runs cal/runs/exp_A.jsonl cal/runs/exp_B.jsonl
#   python3 model.py show                print model.json as a table
#
# WHICH DATA
#   By default only sessions whose operator note mentions "grip_pos" (the exact-position grip,
#   2026-10-25 onward). The older crawl-grip sessions are a different gripper and stay out.
#
# No numpy: the fit has three unknowns, so the 3x3 normal equations are solved by hand.

import argparse
import glob
import json
import math
import os
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS_DIR = os.path.join(HERE, "cal", "runs")
MODEL_FILE = os.path.join(HERE, "model.json")

DEFAULT_NOTE = "grip_pos"         # sessions with this in the operator note = new gripper
RPM_UNITS = "output_rpm_cpr11880" # only real-rpm sessions (after the CPR fix)
NOT_A_TRIAL = {"ABORTED"}         # stopped by the operator: says nothing about the settings
Z = 1.645                         # one-sided 95 % for the Wilson lower bound


# ============================================================
# READING THE RUNS
# ============================================================

def default_runs():
    """New-gripper experiment sessions in real rpm."""
    out = []
    for meta_path in sorted(glob.glob(os.path.join(RUNS_DIR, "exp_*.meta.json"))):
        with open(meta_path, encoding="utf-8") as f:
            meta = json.load(f)
        if meta.get("rpm_units") == RPM_UNITS and DEFAULT_NOTE in (meta.get("operator_note") or ""):
            out.append(meta_path[:-len(".meta.json")] + ".jsonl")
    return out


def read_cycles(paths):
    """Every exp/cycle record of the given sessions -> list of dicts."""
    cycles = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            for line in f:
                if '"exp/cycle"' not in line:          # cheap filter: the files are mostly telemetry
                    continue
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue                            # a half-written last line while a run is live
                d = rec.get("data") or {}
                if rec.get("topic") != "exp/cycle" or d.get("rpm") is None:
                    continue
                cycles.append({"run": os.path.basename(p), "rpm": float(d["rpm"]),
                               "arm_speed": float(d["arm_speed"]), "carry_speed": float(d["carry_speed"]),
                               "class": d.get("class", "?"), "secs": d.get("secs")})
    return cycles


# ============================================================
# FITTING
# ============================================================

def _solve3(A, y):
    """Solve the 3x3 system A x = y (Cramer's rule). Raises if the data cannot separate the terms."""
    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    d = det(A)
    if abs(d) < 1e-9:
        raise ValueError("cannot fit: need at least 2 different rpm AND 2 different arm speeds")
    x = []
    for col in range(3):
        m = [row[:] for row in A]
        for r in range(3):
            m[r][col] = y[r]
        x.append(det(m) / d)
    return x


def fit_time(cycles):
    """Least squares secs = a + b/arm + c/rpm on the OK cycles."""
    pts = [(1.0, 1.0 / c["arm_speed"], 1.0 / c["rpm"], float(c["secs"]))
           for c in cycles if c["class"] == "OK" and c["secs"]]
    if len(pts) < 6:
        raise ValueError(f"only {len(pts)} OK cycles: too few to fit")
    A = [[sum(p[i] * p[j] for p in pts) for j in range(3)] for i in range(3)]
    y = [sum(p[i] * p[3] for p in pts) for i in range(3)]
    a, b, c = _solve3(A, y)
    pred = [a + b * p[1] + c * p[2] for p in pts]
    mean = sum(p[3] for p in pts) / len(pts)
    ss_res = sum((p[3] - q) ** 2 for p, q in zip(pts, pred))
    ss_tot = sum((p[3] - mean) ** 2 for p in pts) or 1e-9
    return {"a": round(a, 3), "b": round(b, 3), "c": round(c, 3), "n": len(pts),
            "r2": round(1 - ss_res / ss_tot, 3), "rmse_s": round(math.sqrt(ss_res / len(pts)), 2)}


def wilson_lower(ok, n):
    """Cautious success rate: with 10/10 it says ~0.79, not 1.0. Small samples get small credit."""
    if n == 0:
        return 0.0
    p = ok / n
    den = 1 + Z * Z / n
    centre = p + Z * Z / (2 * n)
    margin = Z * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n))
    return max(0.0, (centre - margin) / den)


def success_table(cycles):
    """One row per tested config."""
    groups = {}
    for c in cycles:
        if c["class"] in NOT_A_TRIAL:
            continue
        key = (c["rpm"], c["arm_speed"], c["carry_speed"])
        g = groups.setdefault(key, {"n": 0, "ok": 0, "fails": {}, "secs": []})
        g["n"] += 1
        if c["class"] == "OK":
            g["ok"] += 1
            if c["secs"]:
                g["secs"].append(float(c["secs"]))
        else:
            g["fails"][c["class"]] = g["fails"].get(c["class"], 0) + 1
    rows = []
    for (rpm, arm, carry), g in sorted(groups.items()):
        rows.append({"rpm": rpm, "arm_speed": arm, "carry_speed": carry, "n": g["n"], "ok": g["ok"],
                     "p": round(g["ok"] / g["n"], 3), "p_low": round(wilson_lower(g["ok"], g["n"]), 3),
                     "secs_mean": round(sum(g["secs"]) / len(g["secs"]), 2) if g["secs"] else None,
                     "fails": g["fails"]})
    return rows


def fit(paths):
    cycles = read_cycles(paths)
    if not cycles:
        raise ValueError("no cycles in " + ", ".join(paths))
    rows = success_table(cycles)
    return {"fitted_at": datetime.now().isoformat(timespec="seconds"),
            "runs": [os.path.basename(p) for p in paths], "n_cycles": len(cycles),
            "time_model": fit_time(cycles), "configs": rows,
            "range": {"rpm": [min(r["rpm"] for r in rows), max(r["rpm"] for r in rows)],
                      "arm_speed": [min(r["arm_speed"] for r in rows), max(r["arm_speed"] for r in rows)]}}


# ============================================================
# USING THE MODEL
# ============================================================

def predict_secs(model, rpm, arm_speed):
    t = model["time_model"]
    return t["a"] + t["b"] / arm_speed + t["c"] / rpm


def load(path=MODEL_FILE):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def show(model):
    t = model["time_model"]
    print(f"fitted {model['fitted_at']} from {model['n_cycles']} cycles in {len(model['runs'])} sessions")
    print(f"cycle time = {t['a']} + {t['b']}/arm_speed + {t['c']}/rpm   (R2 {t['r2']}, rmse {t['rmse_s']} s, "
          f"{t['n']} OK cycles)")
    print(f"{'rpm':>5} {'arm':>4} {'carry':>5} {'n':>3} {'ok':>3} {'p':>5} {'p_low':>5} {'secs':>6} {'model':>6}  fails")
    for r in model["configs"]:
        secs = "-" if r["secs_mean"] is None else f"{r['secs_mean']:.1f}"
        print(f"{r['rpm']:>5} {r['arm_speed']:>4} {r['carry_speed']:>5} {r['n']:>3} {r['ok']:>3} {r['p']:>5} "
              f"{r['p_low']:>5} {secs:>6} {predict_secs(model, r['rpm'], r['arm_speed']):>6.1f}  {r['fails'] or ''}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="fit / show the optimizer's model")
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("fit")
    f.add_argument("--runs", nargs="*", help="session .jsonl files (default: new-gripper sessions)")
    f.add_argument("--out", default=MODEL_FILE)
    s = sub.add_parser("show")
    s.add_argument("--model", default=MODEL_FILE)
    a = ap.parse_args(argv)
    if a.cmd == "show":
        show(load(a.model))
        return 0
    paths = a.runs or default_runs()
    model = fit(paths)
    tmp = a.out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(model, fh, indent=1)
    os.replace(tmp, a.out)
    show(model)
    print(f"saved {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
