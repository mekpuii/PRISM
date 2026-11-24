# test_fault_sim.py - run: ~/db-env/bin/python test/test_fault_sim.py   (needs pyserial + paho: db-env)
# The FAULT system (2026-11-13, the operator: "if the camera is covered ... pause at the checkpoint, never shut down;
# warn with the arm; go to the rest pose; try again; carry on by itself"). The real auto_en_new.py in FLOW + the real
# optimizer on the fake belt. Times are shortened (shake gap 3 s, retry every 5 s, clear after 3 s).
#   1  picking; then the camera is covered (a black belt picture) -> FAULT, autoen paused, the cube in the gripper finished
#   2  the arm is at home, shakes 3 sets x 3 times (a gap between the sets), then rests
#   3  while resting it tries again (fault_retry jobs in the log); the Pi-side programs are never touched
#   4  the picture is back -> after the clear time it wakes the arm and RESUMES by itself; refilled cubes are picked
import json
import os
import sys
import tempfile
import threading
import time
from dataclasses import replace

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ["PRISM_CAMGEO"] = os.path.join(os.path.dirname(HERE), "camera_geometry.json")
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402
import test_verify_sim as V                                 # noqa: E402
import test_restprobe_sim as RP                             # noqa: E402
import camgeo                                               # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
import guardian as G                                        # noqa: E402
import actuate as A                                         # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7
O.FAULT_RETRY_FAST_S = 5.0
A.FAULT_SHAKE_GAP_S = 3.0
G.CLEAR["camera_covered"] = 3.0


def main():
    T.TMP = tempfile.mkdtemp(prefix="prism_fault_sim_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST
    fake = RP.RestMega(fast=8.0)
    fake.cubes_left = 6
    api = T.start_autoen(fake)
    phase = [""]
    fake.parked = lambda: phase[0] in ("TRANSPORT", "IDLE", "")
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 1, "enable_b": 0, "enable_c": 0, "rpm": 20.0,
                                                      "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0,
                                                      "enable_verify": 0}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    opt.feed_moved = 10
    log_path = os.path.join(T.TMP, "orch.jsonl")
    act = Actuate(api=api, mqtt_enabled=False, log_path=log_path, fixers={})
    stop, modes, cover = threading.Event(), [], [False]
    dark = {"v_mean": 3.0, "s_mean": 0.0, "glare_pct": 0.0, "dark_pct": 99.0}
    fine = {"v_mean": 120.0, "s_mean": 40.0, "glare_pct": 0.0, "dark_pct": 1.0}

    def loop():
        seq, last = 0, 0.0
        while not stop.is_set():
            t = time.monotonic()
            if t - last >= 0.17:
                seq, last = seq + 1, t
            v = act.view()
            phase[0] = v.phase or ""
            w = replace(fake.world(t, seq), light=dark if cover[0] else fine)
            act.step(opt.step(w, PredictOutput(t=t, cubes=[], belt_mm_s=0, belt_ok=False, gaps=[]), v, act.commands()))
            if not modes or modes[-1][1] != opt.mode:
                modes.append((time.monotonic(), opt.mode))
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()

    def wait(pred, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            st = api.state()
            if pred(st):
                return st
            if st["status"] in ("error", "stopped", "finished"):
                raise AssertionError(f"autoen ended while waiting for {what}: {st['status']} {st['message']}")
            time.sleep(0.2)
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | {api.state().get('status')} "
                             f"rest {api.state().get('rest_state')} | modes {[m for _, m in modes]}")

    def jobs():
        out = []
        for line in open(log_path):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("kind") == "job":
                out.append(r["data"] if "data" in r else r)
        return out
    try:
        wait(lambda s: fake.picked >= 3, 300, "3 cubes picked")
        picked0 = fake.picked
        cover[0] = True
        wait(lambda s: opt.mode == "FAULT", 30, "FAULT after the camera was covered")
        print(f"1  camera covered -> {opt.mode}: {opt.reason[:90]}...   OK")
        wait(lambda s: s["status"] == "paused", 120, "autoen paused at the checkpoint")
        st = wait(lambda s: s.get("rest_state") != "awake" and not act.busy(), 120, "shakes done and the arm resting")
        tx = open(sorted(__import__("glob").glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read()
        shakes = tx.count(" shake ")
        assert shakes == A.FAULT_SHAKE_SETS, f"{shakes} shake tasks, expected {A.FAULT_SHAKE_SETS} sets"
        print(f"2  paused at the checkpoint, {shakes} shake sets (gap 3 s), then the arm is {st['rest_state']}   OK")
        wait(lambda s: any(j.get("job", "").startswith("fault_retry") for j in jobs()), 60, "a retry while resting")
        n_retry = sum(1 for j in jobs() if str(j.get("job", "")).startswith("fault_retry"))
        assert opt.mode == "FAULT" and opt._fault["stage"] == "resting", (opt.mode, opt._fault)
        print(f"3  resting: {n_retry} retr{'y' if n_retry == 1 else 'ies'} so far (fault_retry jobs), still FAULT   OK")
        cover[0] = False
        with fake.lock:
            fake.cubes_left = 3
        wait(lambda s: opt.mode == "RUN" and s["status"] == "running" and s.get("rest_state") == "awake", 120,
             "waking and resuming by itself")
        wait(lambda s: fake.picked >= picked0 + 3, 240, "cubes picked after the fault")
        print(f"4  picture back -> woke the arm and resumed by itself; {fake.picked - picked0} more cubes picked   OK")
        # 5. the fault is fixed WHILE the warning shakes are still going (a hand taken off the lens): resume at once, no
        #    waiting for the other shake sets and the rest pose
        A.FAULT_SHAKE_GAP_S = 20.0
        picked1, shakes0 = fake.picked, open(sorted(__import__("glob").glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read().count(" shake ")
        cover[0] = True
        wait(lambda s: opt.mode == "FAULT" and s["status"] == "paused" and s.get("rest_state") == "awake", 120, "the 2nd fault at the checkpoint")
        t_end = time.monotonic() + 60
        while time.monotonic() < t_end and open(sorted(__import__("glob").glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read().count(" shake ") < shakes0 + 1:
            time.sleep(0.3)
        t_fix = time.monotonic()
        cover[0] = False
        wait(lambda s: opt.mode == "RUN" and s["status"] == "running", 60, "resuming during the warning")
        took = time.monotonic() - t_fix
        shakes1 = open(sorted(__import__("glob").glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read().count(" shake ")
        assert shakes1 - shakes0 < A.FAULT_SHAKE_SETS, f"all {shakes1 - shakes0} shake sets were shaken before resuming"
        assert api.state().get("rest_state") == "awake"
        with fake.lock:
            fake.cubes_left = 2
        wait(lambda s: fake.picked >= picked1 + 2, 240, "cubes picked after the early resume")
        print(f"5  fixed during the warning ({shakes1 - shakes0} of {A.FAULT_SHAKE_SETS} shake sets done): resumed {took:.1f} s later, no rest pose, {fake.picked - picked1} more cubes picked   OK")
        print(f"   modes: {' > '.join(m for _, m in modes)}")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall fault sim tests passed")


if __name__ == "__main__":
    main()
