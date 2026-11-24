"""IR u + ETA check on old runs. Clean cycle = the arriving cube is seen MOVING (vu<-5, u<320) <=0.6 s before ARRIVED.
u_IR from even cycles, ETA test on odd cycles. ETA = (u_IR - u) / vu  (camera Kalman vu, px/s)."""
import json, statistics as st, sys
RUNS = {"run_80rpm": 8.9, "run_100rpm": 11.1, "run_100rpm_armspd0.7": 11.1, "run_100rpm_armspd1.0": 11.1,
        "run_100rpm_armspd1.5": 11.1, "run_100rpm_armspd2.0": 6.7}   # real rpm = old / 9 (2.0 run was CONV 60)
D = "/home/mekkhram/Desktop/PRISM/orchestrator/cal/runs/"
allc, stats = [], {}
for r, rpm in RUNS.items():
    per, arr = [], []
    for line in open(D + r + ".jsonl"):
        d = json.loads(line)
        if "header" in d: continue
        if d["topic"] == "camera/perception": per.append((d["rt"], d["data"].get("tracks") or []))
        elif d["topic"] == "belt/event" and d["data"].get("name") == "ARRIVED": arr.append(d["rt"])
    clean = lost = 0
    for ta in arr:
        near = [(t, x) for t, trs in per for x in trs if ta - 0.6 <= t <= ta + 0.05 and x["vu"] < -5 and x["u"] < 320]
        if not near:
            lost += 1; continue
        t, x = near[-1]
        hist = [(tt, y) for tt, trs in per for y in trs if y["id"] == x["id"] and tt <= ta]
        allc.append(dict(run=r, rpm=rpm, ta=ta, u_arr=x["u"] + x["vu"] * (ta - t), hist=hist,
                         first_seen_s=ta - hist[0][0]))
        clean += 1
    stats[r] = dict(arrived=len(arr), clean=clean, not_seen_at_IR=lost)
cal, test = allc[0::2], allc[1::2]
U = [c["u_arr"] for c in cal]
uir = st.median(U)
vsteady = {}
for rp in set(c["rpm"] for c in cal):
    v = [y["vu"] for c in cal if c["rpm"] == rp for t, y in c["hist"] if c["ta"] - t < 6]
    vsteady[rp] = st.median(v)
rows = []
for c in test:
    for H in (5, 10):
        cand = [(abs(c["ta"] - t - H), t, y) for t, y in c["hist"] if abs(c["ta"] - t - H) <= 0.5]
        if not cand:
            rows.append(dict(run=c["run"], rpm=c["rpm"], H=H, visible=False)); continue
        _, t, y = min(cand, key=lambda z: z[0])
        act = c["ta"] - t
        e_cam = (uir - y["u"]) / y["vu"] - act if y["vu"] < -0.5 else None
        vs = vsteady.get(c["rpm"])
        e_belt = (uir - y["u"]) / vs - act if vs else None
        rows.append(dict(run=c["run"], rpm=c["rpm"], H=H, visible=True, actual=round(act, 2), u=y["u"], vu=y["vu"],
                         hits=y["hits"], err_cam=None if e_cam is None else round(e_cam, 2),
                         err_steady_v=None if e_belt is None else round(e_belt, 2)))
def summ(e):
    if not e: return None
    return dict(n=len(e), mean=round(st.mean(e), 2), sd=round(st.stdev(e), 2) if len(e) > 1 else None,
                max_abs=round(max(abs(x) for x in e), 2), within_1s=f"{sum(abs(x) <= 1 for x in e)}/{len(e)}")
res = {}
for H in (5, 10):
    for rp in sorted(set(r["rpm"] for r in rows)) + ["all"]:
        sel = [r for r in rows if r["H"] == H and (rp == "all" or r["rpm"] == rp)]
        vis = [r for r in sel if r["visible"]]
        res[f"H{H}s_rpm{rp}"] = dict(cycles=len(sel), visible=len(vis),
            camera_vu=summ([r["err_cam"] for r in vis if r["err_cam"] is not None]),
            steady_v=summ([r["err_steady_v"] for r in vis if r["err_steady_v"] is not None]))
out = dict(runs=stats, n_clean=len(allc), n_cal=len(cal), n_test=len(test),
           u_IR=dict(median=round(uir, 1), mean=round(st.mean(U), 1), sd=round(st.stdev(U), 1),
                     range=[round(min(U), 1), round(max(U), 1)], by_rpm={str(rp): round(st.median([c["u_arr"] for c in cal if c["rpm"] == rp]), 1) for rp in set(c["rpm"] for c in cal)}),
           u_IR_test_half_median=round(st.median(c["u_arr"] for c in test), 1),
           v_steady_px_s={str(k): round(v, 1) for k, v in vsteady.items()},
           first_seen_before_IR_s=[round(min(c["first_seen_s"] for c in allc), 1), round(st.median(c["first_seen_s"] for c in allc), 1), round(max(c["first_seen_s"] for c in allc), 1)],
           eta=res)
json.dump(dict(summary=out, rows=rows), open(sys.argv[1], "w"), indent=1)
print(json.dumps(out, indent=1))
for r in rows: print(r)
