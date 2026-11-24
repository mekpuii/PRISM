# test_fill_sim.py - run: ~/db-env/bin/python test/test_fill_sim.py   (needs pyserial + paho: db-env)
# pt3 (2026-11-12): FILL THE BELT - many cubes ride at once, spaced by distance (encoder + camera), so after each pick
# the next cube is only one gap away. The real auto_en_new.py in FLOW on a fake with the REAL geometry: 470 mm feeder ->
# IR (~8.3 s at 20 rpm, 33000 counts), the new camera's picture (exit 65 px, IR 805 px, 0.64 mm/px).
# Runs the same fake twice: one cube at a time (FILL_MAX = 1) and pt3. Checks: several cubes ride at once, the pick is
# much faster, every gap at push time is >= ~60 mm, no cube reaches the IR before the arm is parked, no false "stuck".
import os
import statistics
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ["PRISM_CAMGEO"] = os.path.join(os.path.dirname(HERE), "camera_geometry.json")   # the NEW camera
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import test_verify_sim as V                                 # noqa: E402
import camgeo                                               # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

assert camgeo.ACTIVE, "camera_geometry.json missing"
F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7
COUNTS_PER_MM = 33000 / 470.0


class FillMega(V.VerifyMega):
    RIDE = 33000

    def __init__(self, *a, **k):
        self.push_gaps_mm = []
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[0] == "X" and len(p) > 1 and p[1] == "FEED":
            with self.lock:
                prev = self.cubes[-1] if self.cubes else None
            super()._cmd(line)
            with self.lock:
                if prev is not None and len(self.cubes) >= 2 and self.cubes[-1] != prev:
                    self.push_gaps_mm.append((self.RIDE - prev) / COUNTS_PER_MM - O.CUBE_MM_BELT)
            return
        super()._cmd(line)


def run(fill_max, cubes):
    O.FILL_MAX = fill_max
    O.FLOW_MAX = fill_max
    T.TMP = tempfile.mkdtemp(prefix=f"prism_fill_sim_{fill_max}_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST          # the fake camera = the real picture
    fake = FillMega(fast=8.0)
    fake.cubes_left = cubes
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
    stop, peak = threading.Event(), [0]

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
            with fake.lock:
                peak[0] = max(peak[0], len(fake.cubes))
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()
    try:
        end = time.monotonic() + 500
        while time.monotonic() < end and fake.picked < cubes:
            st = api.state()
            if st["status"] in ("error", "stopped", "finished"):
                raise AssertionError(f"autoen ended: {st['status']} {st['message']}")
            time.sleep(0.3)
        assert fake.picked >= cubes, f"only {fake.picked} of {cubes} picked ({opt.reason}) {opt.flow_info}"
        st = api.state()
        oks = [c["secs"] for c in st["cycles"] if c["result"] == "OK"]
        return {"per": statistics.median(oks[2:]), "peak": peak[0], "gaps": fake.push_gaps_mm,
                "unparked": fake.arrivals_unparked, "stuck": [x for x in opt._evidence if "stuck" in x],
                "info": opt.flow_info, "tuner_picks": opt.tuner.picks}
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass


def main():
    one = run(1, 8)
    print(f"one at a time: {one['per']:.1f} s per cube, at most {one['peak']} cube on the belt")
    fill = run(O.FILL_MAX if O.FILL_MAX > 1 else 6, 12)
    print(f"pt3 fill:      {fill['per']:.1f} s per cube, up to {fill['peak']} cubes on the belt "
          f"(fits {fill['info'].get('fits')}, gap {fill['info'].get('gap_mm')} mm)")
    assert fill["peak"] >= 3, fill
    assert fill["per"] < 0.7 * one["per"], (fill["per"], one["per"])
    print(f"faster:        {one['per']:.1f} -> {fill['per']:.1f} s per cube "
          f"({100 * (1 - fill['per'] / one['per']):.0f} % less)   OK")
    gaps = fill["gaps"]
    assert gaps and min(gaps) >= O.FILL_GAP_MIN_MM - 10, gaps
    print(f"spacing:       every new cube pushed >= {min(gaps):.0f} mm behind the last one (min {O.FILL_GAP_MIN_MM:.0f})   OK")
    assert fill["unparked"] == 0 and one["unparked"] == 0, (fill["unparked"], one["unparked"])
    print("safe:          no cube reached the IR before the arm was parked   OK")
    assert not fill["stuck"], fill["stuck"]
    print(f"no false stuck calls; the tuner recorded {fill['tuner_picks']} clean picks   OK")
    print("\nall fill sim tests passed")


if __name__ == "__main__":
    main()
