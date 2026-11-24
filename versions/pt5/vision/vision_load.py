"""Measure the load of a running vision node (read-only): fps from camera/perception, CPU %, RAM, Pi temperature.
Usage: python vision_load.py --label "6fps" --seconds 60      (vision must already be running)"""
import argparse, json, os, statistics as st, subprocess, time
import paho.mqtt.client as mqtt

ap = argparse.ArgumentParser()
ap.add_argument("--label", default="")
ap.add_argument("--seconds", type=float, default=60)
ap.add_argument("--match", default="vision", help="text in the vision process command line")
ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "vision_load.jsonl"))
a = ap.parse_args()

me = os.getpid()
pids = [int(p) for p in subprocess.run(["pgrep", "-f", a.match], capture_output=True, text=True).stdout.split()]
pids = [p for p in pids if p != me and "vision_load" not in open(f"/proc/{p}/cmdline").read()]
if not pids:
    raise SystemExit("no vision process found")
pid = pids[0]
print("vision pid", pid, open(f"/proc/{pid}/cmdline").read().replace("\0", " "))

fps = []
def on_msg(c, u, m):
    try: fps.append(json.loads(m.payload)["fps"])
    except Exception: pass
cl = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
cl.on_message = on_msg
cl.connect("localhost", 1883); cl.subscribe("camera/perception"); cl.loop_start()

hz = os.sysconf("SC_CLK_TCK")
def proc_ticks():
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])
def temp():
    return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
cpu, rss, tmp = [], [], []
t0 = time.time(); last_t, last_k = t0, proc_ticks()
while time.time() - t0 < a.seconds:
    time.sleep(2)
    now, k = time.time(), proc_ticks()
    cpu.append((k - last_k) / hz / (now - last_t) * 100); last_t, last_k = now, k
    rss.append(int(open(f"/proc/{pid}/statm").read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2**20)
    tmp.append(temp())
cl.loop_stop()
row = dict(label=a.label, at=time.strftime("%Y-%m-%dT%H:%M:%S"), seconds=a.seconds, pid=pid,
           fps_mean=round(st.mean(fps), 2) if fps else None, fps_min=min(fps) if fps else None, n_msgs=len(fps),
           cpu_pct_mean=round(st.mean(cpu), 1), cpu_pct_max=round(max(cpu), 1),
           rss_mb=round(st.mean(rss), 1), temp_c_start=tmp[0], temp_c_end=tmp[-1], temp_c_max=max(tmp),
           cpu_count=os.cpu_count())
print(json.dumps(row))
open(a.out, "a").write(json.dumps(row) + "\n")
