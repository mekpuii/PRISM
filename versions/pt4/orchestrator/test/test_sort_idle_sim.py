# test_sort_idle_sim.py - run: ~/db-env/bin/python test/test_sort_idle_sim.py
# Regression for the 2026-11-10 colour-training run (cube 2: white went LEFT): the feeder IR read "empty" while the
# last cube was still riding, the optimizer went IDLE ("finishing the cube on the belt first"), and the bin change for
# that cube was never sent (only sent in RUN, and queued behind the ~26 s idle job). The cube took the previous bin.
# Here: orange, then white; the feeder IR reads "empty" as soon as the white cube is pushed. Both must land right.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_sort_sim as S                                   # noqa: E402
T, O = S.T, S.O
from actuate import Actuate, ApiError                       # noqa: E402
from observe import BeltObs                                 # noqa: E402

S.ORDER[:] = ["orange", "orange", "white"]
T.TMP = tempfile.mkdtemp(prefix="prism_sort_idle_sim_")


RIDE_S, BLIND_S = 4.0, 2.7      # real belt: the cube reached the camera ~3 s after the feeder IR read "empty" (< NO_CUBE_S 3 s)


class SlowFeeder(S.ColourFeeder):
    """Rides last RIDE_S (the base fake: 1.2 s), so a feeder-IR IDLE can land before the camera sees the cube."""

    def _cmd(self, line):
        super()._cmd(line)
        if line.split(",")[:2] == ["X", "CONV"] and self.arrive_at is not None:
            self.ride_t0, self.arrive_at = time.monotonic(), time.monotonic() + RIDE_S


def camera(fake, t):
    st, pr = S.camera(fake, t)
    if fake.arrive_at is not None and t - getattr(fake, "ride_t0", 0.0) < BLIND_S:
        st.cubes, pr.cubes = [], []                                  # not on the camera yet
        S.CAM["frame"] = 0                                           # so it first shows up at the feeder end
    st.belt, st.belt_fresh = BeltObs(feed_ir=bool(fake.queue)), True   # "empty" once the last cube was pushed
    return st, pr


def main():
    fake = SlowFeeder(fast=8.0)
    fake.cubes_left = len(S.ORDER)
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0,
                                                      "drop_mode": "alternate"}})
    opt = O.Optimize()
    opt._fir_right = O.FEED_IR_PROVEN       # only a PROVEN feeder IR idles at once (unproven: the next feed decides)
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()
    went_idle = []

    def loop():
        while not stop.is_set():
            st, pr = camera(fake, time.monotonic())
            plan = opt.step(st, pr, act.view(), act.commands())
            if plan.mode == "IDLE" and fake.arrive_at is not None:
                went_idle.append(1)
            act.step(plan)
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()
    try:
        end = time.monotonic() + 180
        while time.monotonic() < end:
            st = api.state()
            if len([c for c in st["cycles"] if c["result"] == "OK"]) >= len(S.ORDER):
                break
            time.sleep(0.3)
        oks = [c for c in api.state()["cycles"] if c["result"] == "OK"][:len(S.ORDER)]
        bins = [c["drop"] for c in oks]
        want = [O.SORT_BINS[c] for c in S.ORDER]
        assert went_idle, "the feeder IR never sent the optimizer IDLE while a cube was riding: test did not test it"
        assert bins == want, f"\n got  {bins}\n want {want}"
        print(f"idle mid-ride: {S.ORDER}, feeder IR 'empty' while the last cube rides -> IDLE; "
              f"bins {bins}   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall sort-idle sim tests passed")


if __name__ == "__main__":
    main()
