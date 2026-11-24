# test_late_sim.py - run: ~/db-env/bin/python test/test_late_sim.py   (needs pyserial + paho: db-env)
# "No cube came out" WITHOUT the camera (2026-11-11). The 12 V dip cannot tell a loaded push from an empty one (~200 mV
# either way), but TIME can: at one belt speed every cube reaches the IR within the measured ride. The real
# auto_en_new.py on FeederMega, optimize.py + actuate.py live, camera OFF, in the serial cycle AND in the pipeline:
#   - after a few cubes the ride time is known
#   - feeder empty: the late cube is noticed long before arrive_timeout -> belt braked, fed again at once
#   - still nothing -> paused at home (IDLE), with a story
#   - refilled: its OWN probe feed finds the cubes (blind probes are allowed now) - nobody presses anything
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0   # probe soon so the test stays short
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7                 # the real >= 20 s timeout: lateness must beat it


class SlowBelt(T.FeederMega):
    """The fake belt brings a cube to the IR 3 s after CONV (sim_mega: 1.2 s), so the arm waits for it in the
    pipeline too, like the real belt (8.7 s at 20 rpm)."""

    def _cmd(self, line):
        super()._cmd(line)
        p = line.split(",")
        if p[0] == "X" and len(p) > 2 and p[1] == "CONV" and self.arrive_at is not None:
            self.arrive_at = time.monotonic() + 3.0


def run(enable_b):
    T.TMP = tempfile.mkdtemp(prefix=f"prism_late_sim_b{enable_b}_")
    fake = SlowBelt(fast=8.0)
    fake.cubes_left = 6
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": enable_b, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()
    name = "pipeline" if enable_b else "serial"

    def loop():
        while not stop.is_set():
            t = time.monotonic()
            act.step(opt.step(T.blind_state(t), PredictOutput(t=t, cubes=[], belt_mm_s=0, belt_ok=False, gaps=[]),
                              act.view(), act.commands()))
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
                raise AssertionError(f"{name}: autoen ended while waiting for {what}: {st['status']} {st['message']}")
            time.sleep(0.2)
        raise AssertionError(f"{name}: timed out waiting for {what}: {api.state()['status']} mode {opt.mode} ({opt.reason})")

    res = lambda s: [c["result"] for c in s["cycles"]]       # noqa: E731
    try:
        st = wait(lambda s: opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(), 300, "IDLE (feeder empty)")
        r = res(st)
        assert opt._late_s(act.view()) is not None, f"{name}: the ride time was never learned: {opt._rides}"
        empty = [c for c in st["cycles"] if c["result"] != "OK"]
        assert empty and all(c["result"] == "NO_BLOCK" for c in empty), f"{name}: {r}"
        assert all(c["secs"] < 20 for c in empty), f"{name}: an empty feed ran to the 25 s timeout: {empty}"
        assert fake.rpm_set == 0.0, "belt must be braked"
        print(f"{name:8s} empty:  {r.count('OK')} OK, then 'no cube came' after ~{opt._late_s(act.view()):.0f} s "
              f"(timeout 25 s), fed again, paused   OK   [" + ", ".join(f"{c['result']} {c['secs']}s" for c in empty) + "]")
        print(f"{name:8s} story:  {opt.story['problem']}: {opt.story['so']}")
        n_ok = r.count("OK")
        fake.cubes_left = 2                                  # refilled - nobody presses anything
        wait(lambda s: res(s).count("OK") >= n_ok + 1, 240, "its own probe finds the refill")
        print(f"{name:8s} refill: its own probe feed found the cubes, no Resume pressed   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass


def main():
    run(0)
    run(1)
    print("\nall late-cube sim tests passed")


if __name__ == "__main__":
    main()
