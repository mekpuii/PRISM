# test_sort_sim.py - run: ~/db-env/bin/python test/test_sort_sim.py   (needs pyserial + paho: db-env)
# Colour sorting end to end WITHOUT the robot: the real auto_en_new.py runs against FakeMega; optimize.py + actuate.py
# drive it LIVE. The fake feeder pushes cubes in a known colour order and a fake camera reports the colour of the cube
# riding the belt, like the real one did on 2026-10-26 17:44: every 4th frame shows the WRONG colour and the track
# breaks up and restarts under a new id halfway. Every cube must still land in its own bin: orange -> "left",
# white -> "down" (the right bin), read back from autoen's own cycle log.
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (sets up sys.path, FeederMega, start_autoen)
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from observe import ArmStatus, BeltObs, CubeObs, WorldState  # noqa: E402
from predict import CubeEstimate, PredictOutput             # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_sort_sim_")
ORDER = ["orange", "white", "white", "orange", "white", "orange", "orange", "white"]


class ColourFeeder(T.FeederMega):
    """FeederMega that remembers the colour of the cube now on the belt."""

    def __init__(self, *a, **k):
        self.queue, self.on_belt = list(ORDER), None
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[:2] == ["X", "FEED"] and self.cubes_left > 0:
            self.on_belt = self.queue.pop(0)
        super()._cmd(line)


CAM = {"seq": 0, "tick": 0, "ride": None, "frame": 0}
OTHER = {"orange": "white", "white": "orange"}


def camera(fake, t):
    """6 fps snapshots (every 3rd 20 Hz tick). Every 4th frame has the wrong colour; the track id changes halfway."""
    CAM["tick"] += 1
    if CAM["tick"] % 3 == 0:
        CAM["seq"] += 1
    riding = fake.arrive_at is not None and fake.on_belt
    raw, est = [], []
    if riding:
        if CAM["ride"] != fake.arrive_at:                   # a new ride
            CAM["ride"], CAM["frame"] = fake.arrive_at, 0
        if CAM["tick"] % 3 == 0:
            CAM["frame"] += 1
        f = CAM["frame"]
        colour = OTHER[fake.on_belt] if f % 4 == 0 else fake.on_belt
        tid = 100 + 2 * ORDER.index(fake.on_belt) + (1 if f > 4 else 0)   # breaks up after 4 frames
        raw = [CubeObs(id=tid + 10 * len(fake.queue), color=colour, along_px=-max(245.0, 590.0 - 50.0 * (f - 1)), across_px=240.0,
                       along_vel_px_s=59.0, dist_m=None, w_px=43, h_px=43, hits=6 + f)]
        est = [CubeEstimate(id=1, color=fake.on_belt, t=t, pos_mm=-300.0, vel_mm_s=56.0, sigma_mm=2.0, coasting=False,
                            age_s=0.0)]
    st = WorldState(t=t, connected=True, joints={}, joints_fresh=False, arm=ArmStatus(), arm_fresh=False, gripper=None,
                    belt=BeltObs(), belt_fresh=False, cubes=raw, camera_fresh=True, cam_seq=CAM["seq"], events=[])
    return st, PredictOutput(t=t, cubes=est, belt_mm_s=56.0, belt_ok=True, gaps=[])


def main():
    fake = ColourFeeder(fast=8.0)
    fake.cubes_left = len(ORDER)
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0,
                                                      "drop_mode": "alternate"}})
    opt = O.Optimize()
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            st, pr = camera(fake, time.monotonic())
            act.step(opt.step(st, pr, act.view(), act.commands()))
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()
    try:
        end = time.monotonic() + 240
        while time.monotonic() < end:
            st = api.state()
            oks = [c for c in st["cycles"] if c["result"] == "OK"]
            if len(oks) >= len(ORDER):
                break
            assert st["status"] not in ("error", "stopped", "finished"), f"{st['status']} {st['message']}"
            time.sleep(0.3)
        oks = [c for c in api.state()["cycles"] if c["result"] == "OK"][:len(ORDER)]
        bins = [c["drop"] for c in oks]
        want = [O.SORT_BINS[c] for c in ORDER]
        assert bins == want, f"\n got  {bins}\n want {want}"
        end = time.monotonic() + 5                          # the optimizer sees the last cycle a moment later
        while sum(opt.sort_stats.values()) < len(ORDER) and time.monotonic() < end:
            time.sleep(0.1)
        assert opt.sort_stats["right_bin"] == len(ORDER) and opt.sort_stats["wrong_bin"] == 0, opt.sort_stats
        print(f"sort:   {len(ORDER)} cubes {ORDER}\n        bins  {bins}  -> all in the right bin "
              f"(optimizer checked: {opt.sort_stats})   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall sort sim tests passed")


if __name__ == "__main__":
    main()
