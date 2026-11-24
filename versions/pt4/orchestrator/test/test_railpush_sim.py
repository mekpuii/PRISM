# test_railpush_sim.py - run: ~/db-env/bin/python test/test_railpush_sim.py   (needs pyserial + paho: db-env)
# The 2026-11-12 12:49 run, without the robot: the 12 V rail collapsed to ~2 V while the arm gripped (feeder and arm on
# one supply), pushes then were dead or too short, and a cube stuck half out at the feeder exit (the operator: "the push
# is too short - push again, with a longer push"). The real auto_en_new.py in FLOW on RailMega, optimize.py live:
#   - RailMega's rail reads 2.1 V during GRIP and a push then brings nothing
#   - its feeder needs a NEED_MS push; a shorter one leaves the cube stuck at the exit (seen, but it does not ride)
# Checks: the optimizer learns that GRIP pulls the rail down and stops pushing there; it sees the stuck cube, pushes
# again with a longer push until the cube rides, keeps that length, and the picking goes on.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import test_verify_sim as V                                 # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

NEED_MS = 1000                                              # (the push starts at 800 ms since 11-12: too short here)
PUSH_TOL = 50                                               # (it may be probing 50 ms below the length that works)


class RailMega(V.VerifyMega):
    def __init__(self, *a, **k):
        self.push_ms, self.stuck_id, self.rail_low = 300, None, (lambda: False)
        self.grip_pushes, self.stuck_seen = 0, 0
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[0] == "X" and len(p) > 5 and p[1] == "FEEDCFG":
            self.push_ms = int(p[4])
        if p[0] == "X" and len(p) > 1 and p[1] == "FEED":
            low = self.rail_low()
            self.grip_pushes += low
            if low:                                         # no power: the blade barely moves, nothing comes out
                with self.lock:
                    left, self.cubes_left = self.cubes_left, 0
                super()._cmd(line)
                with self.lock:
                    self.cubes_left = left
                return
            if self.stuck_id is not None:                   # a cube is stuck in the mouth: only a long push frees it
                if self.push_ms >= NEED_MS:
                    self.stuck_id = None
                with self.lock:
                    left, self.cubes_left = self.cubes_left, 0
                super()._cmd(line)
                with self.lock:
                    self.cubes_left = left
                return
            n0 = len(self.cubes)
            super()._cmd(line)
            if len(self.cubes) > n0 and self.push_ms < NEED_MS:
                self.stuck_id = self.ids[-1]                # out, but stuck half in the feeder mouth
                self.stuck_seen += 1
            return
        super()._cmd(line)

    def _step_belt(self):
        with self.lock:
            if self.stuck_id is not None and self.stuck_id in self.ids:
                i = self.ids.index(self.stuck_id)
                keep = self.cubes[i]
        super()._step_belt()
        with self.lock:
            if self.stuck_id is not None and self.stuck_id in self.ids:
                self.cubes[self.ids.index(self.stuck_id)] = max(keep, self.RIDE)   # it does not ride

    def world(self, t, seq):
        w = super().world(t, seq)
        if self.rail_low():
            w.belt.voltage_v = 2.1
        return w


def main():
    T.TMP = tempfile.mkdtemp(prefix="prism_railpush_sim_")
    fake = RailMega(fast=8.0)
    fake.cubes_left = 10
    api = T.start_autoen(fake)
    phase = [""]
    fake.parked = lambda: phase[0] in ("TRANSPORT", "IDLE", "")
    fake.rail_low = lambda: phase[0] == "GRIP"
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 1, "enable_b": 0, "enable_c": 0, "rpm": 20.0,
                                                      "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0,
                                                      "enable_verify": 0, "feeder_push_ms": 300}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    opt.feed_moved = 10
    opt._flow_phase = {"GRIP": [26, 4]}                     # what pt1 learned on 11-12 01:07: GRIP looked like the best moment
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

    def loop():
        seq, last = 0, 0.0
        while not stop.is_set():
            t = time.monotonic()
            if t - last >= 0.17:
                seq, last = seq + 1, t
            v = act.view()
            phase[0] = v.phase or ""
            act.step(opt.step(fake.world(t, seq), PredictOutput(t=t, cubes=[], belt_mm_s=0, belt_ok=False, gaps=[]),
                              v, act.commands()))
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
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | flow {opt.flow_info} "
                             f"| picked {fake.picked} stuck {fake.stuck_id} push {fake.push_ms} grip pushes {fake.grip_pushes}")

    try:
        wait(lambda s: fake.picked >= 7, 400, "7 cubes picked")
        assert "GRIP" in opt.flow_info.get("sagging", []), opt.flow_info
        g0 = fake.grip_pushes
        fake.cubes_left += 12
        wait(lambda s: fake.picked >= 18 and fake.stuck_id is None, 400, "9 more cubes, nothing stuck")
        assert fake.grip_pushes == g0, f"pushed during GRIP after learning the rail sags there ({fake.grip_pushes - g0})"
        assert fake.stuck_seen >= 1, fake.stuck_seen
        assert opt._push_ms >= opt._push_fail + 50, (opt._push_ms, opt._push_fail)   # never below a length that failed
        assert NEED_MS <= opt._push_ms + PUSH_TOL, (opt._push_fail, opt._push_ms)   # it ends long enough
        print(f"rail:    learned that the rail sags in {opt.flow_info['sagging']} -> no pushes there any more   OK")
        print(f"stuck:   {fake.stuck_seen} cube(s) stuck at the exit -> pushed again, longer; push now {opt._push_ms} ms, "
              f"longest that failed {opt._push_fail} ms (the fake feeder needs {NEED_MS}); {fake.picked} picked   OK")
        for x in [x for x in opt._evidence if "stuck" in x][-2:]:
            print(f"         {x}")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall rail/push sim tests passed")


if __name__ == "__main__":
    main()
