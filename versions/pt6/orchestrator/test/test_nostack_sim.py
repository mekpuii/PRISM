# test_nostack_sim.py - run: ~/db-env/bin/python test/test_nostack_sim.py   (needs pyserial + paho: db-env)
# pt5 (2026-11-12, the operator: "the feeder is too quick, the cubes stack"). The 11-12 18:17 run, rebuilt on the REAL
# belt (470 mm, 33000 counts, the new camera's picture):
#   - 2 cubes were already on the belt before the run (nobody fed them; the camera sees them), a glare blob at the exit
#   - the camera does not see every second cube (white cubes under a too-tight colour range / glare)
# Then, before pt5: arrivals were credited to the wrong pushes (ride 468 -> 230 mm), "servo moved + camera saw nothing"
# was called EMPTY and pushed again - onto a cube that HAD come out. Checks now:
#   no stack     every push lands >= one cube + gap (60 mm, -10 tolerance) behind the last cube
#   true ride    the ride stays 0.8..1.3 x 8.3 s (the cubes left on the belt are "extras", not rides)
#   all picked   every cube fed and every cube left on the belt is picked; no false "feeder empty"
import os
import statistics
import sys
import threading
import time
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ["PRISM_CAMGEO"] = os.path.join(os.path.dirname(HERE), "camera_geometry.json")   # the NEW camera
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import test_verify_sim as V                                 # noqa: E402
import test_fill_sim as FL                                  # noqa: E402
import camgeo                                               # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402
from observe import CubeObs                                  # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7


class BlindMega(FL.FillMega):
    """The camera misses every second FED cube; it sees the 2 cubes left on the belt (as on 11-12) and a still glare
    blob of cube size 16 mm from the feeder exit (a reflection: it never moves with the belt)."""

    def world(self, t, seq):
        w = super().world(t, seq)
        with self.lock:
            hidden = {i for i in self.ids if i < 900 and i % 2 == 1}
        w.cubes[:] = [c for c in w.cubes if c.id not in hidden and c.id != 7]   # (7: the big D415 glare blob)
        w.cubes.append(CubeObs(id=8, color="white", along_px=camgeo.ENTRY_ALONG + 25.0, across_px=250.0,
                               along_vel_px_s=0.0, dist_m=None, w_px=40.0, h_px=35.0, hits=50))   # cube-sized, never moves
        return w


def main():
    T.TMP = tempfile.mkdtemp(prefix="prism_nostack_sim_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST
    fake = BlindMega(fast=8.0)
    fake.cubes_left = 10
    with fake.lock:                                         # left on the belt before the run: 300 and 150 mm from the IR
        fake.cubes, fake.ids = [21000, 31000], [900, 901]
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
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop, rides = threading.Event(), []

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
            if (opt.flow_info or {}).get("ride_s"):
                rides.append(opt.flow_info["ride_s"])
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()
    try:
        end = time.monotonic() + 500
        while time.monotonic() < end and fake.picked < 12:
            st = api.state()
            if st["status"] in ("error", "stopped", "finished"):
                raise AssertionError(f"autoen ended: {st['status']} {st['message']}")
            if opt.mode in ("IDLE", "ALERT") and fake.cubes_left > 0:
                raise AssertionError(f"false stop with {fake.cubes_left} cubes in the feeder: {opt.mode} {opt.reason}")
            time.sleep(0.3)
        info = opt.flow_info
        assert fake.picked >= 12, f"only {fake.picked} of 12 picked ({opt.reason}) {info}"
        print(f"all picked:  12 of 12 (10 fed + 2 left on the belt), the camera saw only every second cube   OK")
        gaps = fake.push_gaps_mm
        assert gaps and min(gaps) >= O.FILL_GAP_MIN_MM - 10, f"a push landed on a cube: gaps {[round(g) for g in gaps]}"
        print(f"no stack:    every push >= {min(gaps):.0f} mm behind the last cube (min {O.FILL_GAP_MIN_MM:.0f}), "
              f"{len(gaps)} pushes   OK")
        lo, hi = min(rides), max(rides)
        assert 0.8 * 8.3 <= lo and hi <= 1.3 * 8.3, f"ride went {lo}..{hi} s (true 8.3)"
        print(f"true ride:   {lo:.1f}..{hi:.1f} s (true 8.3 s; 11-12 it fell to 4.1)   OK")
        print(f"camera:      exit {info.get('cam_exit')}, extras {info.get('extras')}, "
              f"{opt.feed_stats.get('verified_fed', 0)} fed / {opt.feed_stats.get('verified_empty', 0)} empty verdicts")
        assert 8 in opt._blk_ignore, f"the still blob at the exit was never recognised as a reflection {opt._blk}"
        assert not opt._blk_ignore & {900, 901}, f"a real cube was taken for a reflection: {opt._blk_ignore}"
        print("reflection:  the still blob at the exit was ignored once the belt moved; the real cubes never were   OK")
        unseen = [x for x in opt._evidence if "not proven" in x]
        print(f"             {len(unseen)} pushes the camera missed were counted as delivered (not pushed again)")
        per = [c["secs"] for c in api.state()["cycles"] if c["result"] == "OK"]
        print(f"pace:        {statistics.median(per[2:]):.1f} s per cube (median)")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall no-stack sim tests passed")


if __name__ == "__main__":
    main()
