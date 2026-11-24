"""Per-cycle / per-phase / per-joint time and energy from an experiment.py session (read-only)."""
import json, sys, statistics as st
MA=6.5; MAX_DT=0.5; MOVE=30.0
def run(path):
    cycles=[]; cur=None; block=None; inblock=False; phase="IDLE"; nb=0
    pa=None; pb=None
    blocks={}; tasks=[]
    def new(b,i):
        return {"block":b,"i":i,"t0":None,"ph_s":{}, "ph_arm":{}, "ph_belt":{}, "jJ":[0.0]*6, "mv_J":0.0,"still_J":0.0,"mv_s":0.0,"still_s":0.0,
                "belt_run_s":0.0,"belt_run_J":0.0,"belt_stop_J":0.0,"pos0":None,"pos1":None,"tmax":0,"btemp":[], "bv":[], "bw_run":[], "volt":[], "ir_t":None,"feed_t":None,"arr_t":None,"ph_seq":[]}
    with open(path) as f:
        for line in f:
            if '"retained": true' in line or '"topic"' not in line: continue
            tp=line[line.index('"topic": "')+10:]; tp=tp[:tp.index('"')]
            if tp in ("arm/telemetry","belt/telemetry"):
                if not inblock: 
                    if tp=="arm/telemetry": pa=None
                    else: pb=None
                    continue
                o=json.loads(line); t=o["rt"]; d=o["data"]
                if cur["t0"] is None: cur["t0"]=t
                if tp=="arm/telemetry":
                    js=[d["joints"][k] for k in sorted(d["joints"])]
                    if pa:
                        dt=min(t-pa[0],MAX_DT); w=pa[1]; ph=pa[3]
                        cur["ph_s"][ph]=cur["ph_s"].get(ph,0)+dt
                        cur["ph_arm"][ph]=cur["ph_arm"].get(ph,0)+sum(w)*dt
                        for k in range(6): cur["jJ"][k]+=w[k]*dt
                        if pa[2]: cur["mv_J"]+=sum(w)*dt; cur["mv_s"]+=dt
                        else: cur["still_J"]+=sum(w)*dt; cur["still_s"]+=dt
                    w=[x.get("volt",12.0)*x.get("cur",0)*MA/1000 for x in js]
                    cur["tmax"]=max(cur["tmax"],max(x.get("temp",0) for x in js))
                    cur["volt"].append(st.mean(x.get("volt",12.0) for x in js))
                    pa=(t,w,any(abs(x.get("speed",0))>MOVE for x in js),phase)
                else:
                    v=d.get("voltage"); c=d.get("current")
                    if not isinstance(v,(int,float)) or not isinstance(c,(int,float)): pb=None; continue
                    if pb:
                        dt=min(t-pb[0],MAX_DT); ph=pb[3]; e=pb[1]*dt
                        cur["ph_belt"][ph]=cur["ph_belt"].get(ph,0)+e
                        if pb[2]: cur["belt_run_s"]+=dt; cur["belt_run_J"]+=e; cur["bw_run"].append(pb[1]); cur["bv"].append(pb[4])
                        else: cur["belt_stop_J"]+=e
                    if cur["pos0"] is None: cur["pos0"]=d.get("position")
                    cur["pos1"]=d.get("position")
                    if isinstance(d.get("temp"),(int,float)): cur["btemp"].append(d["temp"])
                    pb=(t,v*c/1000.0,abs(d.get("velocity",0))>0.5,phase,abs(d.get("velocity",0)))
                continue
            o=json.loads(line); t=o["rt"]; d=o["data"]
            if tp=="arm/state":
                if d["phase"]!=phase:
                    phase=d["phase"]
                    if inblock: cur["ph_seq"].append(phase)
            elif tp=="exp/block":
                if d.get("status")=="started":
                    block=d["block"]; inblock=True; nb=1; cur=new(block,nb); pa=pb=None
                else:
                    blocks[d["block"]]=d; inblock=False
            elif tp=="exp/cycle" and inblock:
                cur.update(t1=t, rpm=d["rpm"], arm=d["arm_speed"], carry=d["carry_speed"], cls=d["class"], secs=d.get("secs"), notes=d.get("raw_text",""), when=d["t"])
                cycles.append(cur); nb+=1; cur=new(block,nb); cur["t0"]=t
            elif tp=="autoen/cycle" and inblock:
                cur["drop"]=d.get("drop"); cur["auto_t"]=t
            elif tp=="belt/event" and inblock:
                if d["name"]=="FEED_DONE": cur["feed_t"]=t
                elif d["name"]=="ARRIVED": cur["arr_t"]=t
                cur.setdefault("events",[]).append(d["name"])
            elif tp=="exp/task": tasks.append(d)
    for c in cycles:
        c["bv"]=round(st.mean(c["bv"]),3) if c["bv"] else None
        c["bw_run"]=round(st.mean(c["bw_run"]),4) if c["bw_run"] else None
        c["btemp"]=round(st.mean(c["btemp"]),2) if c["btemp"] else None
        c["volt"]=round(st.mean(c["volt"]),3) if c["volt"] else None
    return {"cycles":cycles,"blocks":blocks,"tasks":tasks}
if __name__=="__main__":
    json.dump(run(sys.argv[1]),open(sys.argv[2],"w"))
