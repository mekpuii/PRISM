"""Energy per cube for every block of one experiment session (streams the file: low RAM).
Window per block = end of its 1st cycle -> end of its last cycle (pauses between blocks excluded),
same power formulas as cal/energy_per_cube.py."""
import json, sys, statistics as st
sys.path.insert(0, "/home/mekkhram/Desktop/PRISM/orchestrator/cal")
from energy_per_cube import arm_power, MAX_DT

path = sys.argv[1]
blocks = {}
with open(path) as f:                       # pass 1: cycle end times per block
    for line in f:
        if '"exp/cycle"' not in line: continue
        d = json.loads(line); x = d["data"]
        b = blocks.setdefault(x["block"], {"rpm": x["rpm"], "arm": x["arm_speed"], "t": [], "ok": 0, "secs": []})
        b["t"].append(d["rt"]); b["ok"] += x["class"] == "OK"
        if x.get("secs"): b["secs"].append(x["secs"])
win = sorted((b["t"][0], b["t"][-1], k) for k, b in blocks.items())
for b in blocks.values():
    b.update(arm_J=0.0, mv_J=0.0, idle_J=0.0, belt_J=0.0, idle_s=0.0)
last = {"arm": None, "belt": None}
def which(t):
    for t0, t1, k in win:
        if t0 <= t <= t1: return k, t1
    return None, None
with open(path) as f:                       # pass 2: integrate power inside the windows
    for line in f:
        if '"arm/telemetry"' not in line and '"belt/telemetry"' not in line: continue
        o = json.loads(line)
        if o.get("retained"): continue
        t, d = o["rt"], o["data"]
        src = "arm" if o["topic"] == "arm/telemetry" else "belt"
        prev = last[src]
        if prev:
            k, t1 = which(prev[0])
            if k is not None:
                dt = min(t - prev[0], MAX_DT, t1 - prev[0]); b = blocks[k]
                if src == "arm":
                    w, moving = prev[1]
                    b["arm_J"] += w * dt
                    if moving: b["mv_J"] += w * dt
                    else: b["idle_J"] += w * dt; b["idle_s"] += dt
                else:
                    b["belt_J"] += prev[1] * dt
        if src == "arm":
            p = arm_power(d)
            last["arm"] = (t, p) if p else None
        elif isinstance(d.get("voltage"), (int, float)) and isinstance(d.get("current"), (int, float)):
            last["belt"] = (t, d["voltage"] * d["current"] / 1000.0)
# per config (2 blocks each)
cfg = {}
for k, b in blocks.items():
    n = len(b["t"]) - 1; dur = b["t"][-1] - b["t"][0]
    c = cfg.setdefault((b["rpm"], b["arm"]), {"n": 0, "dur": 0, "arm": 0, "mv": 0, "idle": 0, "belt": 0, "idle_s": 0, "ok": 0, "N": 0})
    c["n"] += n; c["dur"] += dur; c["arm"] += b["arm_J"]; c["mv"] += b["mv_J"]; c["idle"] += b["idle_J"]
    c["belt"] += b["belt_J"]; c["idle_s"] += b["idle_s"]; c["ok"] += b["ok"]; c["N"] += len(b["t"])
print(f"{'rpm':>5}{'arm':>5}{'ok':>7}{'s/cube':>8}{'cube/min':>9}{'armJ':>7}{'moveJ':>7}{'idleJ':>7}{'idle%t':>7}{'beltJ':>7}{'totJ':>7}{'avgW':>6}")
for (rpm, arm), c in sorted(cfg.items()):
    n = c["n"]; tot = (c["arm"] + c["belt"]) / n
    print(f"{rpm:>5}{arm:>5}{c['ok']:>4}/{c['N']:<2}{c['dur']/n:>8.1f}{60*n/c['dur']:>9.2f}{c['arm']/n:>7.1f}{c['mv']/n:>7.1f}"
          f"{c['idle']/n:>7.1f}{100*c['idle_s']/c['dur']:>6.0f}%{c['belt']/n:>7.1f}{tot:>7.1f}{(c['arm']+c['belt'])/c['dur']:>6.2f}")
