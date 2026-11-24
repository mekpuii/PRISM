# energy_from_raw.py - time and energy per cube from autoen's own raw log (2026-11-12, the information book, section 5).
# Run from autoencoder/riff_logs:  python3 ../../orchestrator/cal/energy_from_raw.py 20261112_200041 20261112_142348
# Energy = belt V x I + the six servos V x I, integrated over the whole run (gaps capped at 0.5 s), / cubes picked.
import csv, sys, statistics, datetime as D
def run(stamp):
    raw=f"autoen_raw_{stamp}.csv"; cyc=f"autoen_cycles_{stamp}.csv"
    rows=list(csv.DictReader(l.replace("\x00","") for l in open(raw, errors="replace") if "\x00" not in l))
    t=lambda r: D.datetime.fromisoformat(r["time"]).timestamp()
    eb=ea=0.0; prev=None; T=0.0
    def f(x):
        try: return float(x)
        except: return None
    for r in rows:
        try: tt=t(r)
        except Exception: continue
        if prev is not None:
            dt=min(0.5,max(0.0,tt-prev[0])); T+=dt
            pb,pa=prev[1],prev[2]
            eb+=pb*dt; ea+=pa*dt
        bv,bi=f(r["Belt_voltage"]),f(r["Belt_current"])
        pb=(bv or 0)*(bi or 0)/1000.0
        pa=sum((f(r[f"J{j}_voltage"]) or 12.0)*(f(r[f"J{j}_current"]) or 0)/1000.0 for j in range(1,7))
        prev=(tt,pb,pa)
    cy=list(csv.DictReader(open(cyc)))
    ok=[c for c in cy if c["result"]=="OK"]
    secs=[float(c["seconds"]) for c in ok]
    drops={}
    for c in ok: drops[c["drop"]]=drops.get(c["drop"],0)+1
    n=len(ok)
    print(f"{stamp}: {n} OK of {len(cy)}, run {T/60:.1f} min, median {statistics.median(secs[1:] or secs):.1f} s/cube (all-OK median, first excluded), "
          f"belt {eb/T:.2f} W, arm {ea/T:.2f} W, energy/cube {(eb+ea)/max(n,1):.1f} J, run-time/cube {T/max(n,1):.1f} s, drops {drops}")
for s in sys.argv[1:]: run(s)
