# test_auto_sim.py - run: ~/db-env/bin/python test/test_auto_sim.py   (needs pyserial + paho: db-env)
# End-to-end WITHOUT the robot, camera ON, NO operator commands: the real auto_en_new.py runs against FakeMega;
# optimize.py (with forecast.py) + actuate.py drive it LIVE. The feeder runs dry -> IDLE; the optimizer PROBES on its
# own; the first probe finds nothing (aborted, NO_BLOCK) -> IDLE again; then the test refills the feeder and the next
# probe must pick the cubes up without anyone sending "resume". Files go to a fresh /tmp/prism_auto_sim_* folder.
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_abort_sim as S                                  # noqa: E402  (FakeMega feeder + fake camera)
import test_orch_sim as T                                   # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_auto_sim_")
F.PRIOR_RATE_H, F.MIN_DELAY_S = 2000.0, 3.0                 # probe after ~3 s so the test stays short


def main():
    fake = T.FeederMega(fast=8.0)
    fake.cubes_left = 6
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0}})
    fc = F.Forecast(path=os.path.join(T.TMP, "forecast_state.json"))
    opt = O.Optimize(forecast=fc, auto_resume=True)
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    plans, stop = [], threading.Event()

    def loop():
        while not stop.is_set():
            st, pr = S.camera(fake, time.monotonic())
            p = opt.step(st, pr, act.view(), act.commands())
            act.step(p)
            if not plans or plans[-1].seq != p.seq:
                plans.append(p)
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
        raise AssertionError(f"timed out waiting for {what}: {api.state()['status']} mode {opt.mode}")

    def results(st):
        return [c["result"] for c in st["cycles"]]

    try:
        # 1. 6 cubes, then the feeder is empty -> NO_BLOCK -> IDLE
        wait(lambda s: results(s).count("OK") == 6 and "NO_BLOCK" in results(s) and opt.mode == "IDLE", 180, "first IDLE")
        print("idle:     6 OK, feeder empty -> NO_BLOCK -> IDLE   OK")
        # 2. an automatic probe finds nothing -> a second NO_BLOCK -> IDLE again (nobody sent a command)
        wait(lambda s: results(s).count("NO_BLOCK") >= 2 and opt.mode == "IDLE" and s["status"] == "paused"
             and not act.busy(), 120, "an empty probe")
        assert not act.commands() and not any("operator" in p.reason for p in plans)
        print("probe:    automatic probe of the empty feeder -> aborted -> IDLE again   OK")
        # 3. refill while idle -> the next probe picks the cubes up on its own
        good = lambda: opt.configs[(20.0, 1.5)]["ok"]                 # noqa: E731 - autoen lists only its last 12 cycles
        n_ok = good()
        fake.cubes_left = 3
        st = wait(lambda s: good() == n_ok + 3 and opt.mode == "IDLE" and s["status"] == "paused"
                  and not act.busy(), 180, "the refill picked up")
        assert "TIMEOUT_TRANSPORT" not in results(st) and st["status"] == "paused", results(st)
        assert any(g["refill"] for g in fc.gaps), fc.gaps
        print(f"refill:   3 cubes refilled while idle -> picked up by a probe, no command; forecast learned "
              f"{len(fc.gaps)} idle periods ({sum(g['refill'] for g in fc.gaps)} with a refill)   OK")
        assert opt.target == (20.0, 1.5) and opt.configs[(20.0, 1.5)]["n"] == 9, "empty probes must not blame 20/1.5"
        print("blame:    settings still 20/1.5, 9 good cycles counted, empty probes not   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print(f"\nlog: {act.log_path}")
    print("all auto-resume sim tests passed")


if __name__ == "__main__":
    main()
