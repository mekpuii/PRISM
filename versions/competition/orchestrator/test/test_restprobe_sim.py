# test_restprobe_sim.py - run: ~/db-env/bin/python test/test_restprobe_sim.py   (needs pyserial + paho: db-env)
# pt5 (2026-11-12, the operator: "after a long rest with no cubes I fill the feeder and PRISM takes a long time to start
# again"). The real auto_en_new.py in FLOW on the real belt geometry (new camera). The feeder runs dry -> IDLE (paused,
# arm parked, belt braked). Then, while resting, light probes: one push, the camera looks at the feeder exit.
#   ir     the feeder IR works: a refill is seen within ~2 s -> probe -> cube seen -> running again
#   noir   the feeder IR reads nothing (as on 11-12): the 20 s probes find the refill
# Checks: restart within the expected time; resting probes never move the belt or the arm; no false restart while empty.
import os
import sys
import tempfile
import threading
import time

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

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7


class RestMega(FL.FillMega):
    ir_works = True

    def world(self, t, seq):
        w = super().world(t, seq)
        w.cubes[:] = [c for c in w.cubes if c.id != 7]      # (7: the D415 test's big glare blob)
        if not self.ir_works:
            w.belt.feed_ir = None
        return w


def run(ir_works):
    T.TMP = tempfile.mkdtemp(prefix=f"prism_rest_sim_{ir_works}_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST
    fake = RestMega(fast=8.0)
    fake.ir_works = ir_works
    fake.cubes_left = 3
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
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | picked {fake.picked} "
                             f"left {fake.cubes_left} | flow {opt.flow_info}")
    try:
        wait(lambda s: fake.picked >= 3, 300, "3 cubes picked")
        wait(lambda s: opt.mode == "IDLE" and s["status"] == "paused", 200, "IDLE (feeder dry)")
        picked0 = fake.picked
        time.sleep(30.0)                                    # resting, empty: light probes find nothing
        rest = [x for x in opt._evidence if "rest probe" in x]
        assert opt.mode == "IDLE" and not rest, f"restarted while still empty: {opt.mode} {rest}"
        import glob                                         # the only belt move while resting: autoen's 1 s pause brake
        tx = open(sorted(glob.glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read().splitlines()
        rest_lines = [l for l in tx if l[:12] >= time.strftime("%H:%M:%S", time.localtime(time.time() - 30.0))]
        runs = [l for l in rest_lines if "TX X,CONV," in l and float(l.split("CONV,")[1]) >= 1.0]
        probes = [l for l in rest_lines if "feeder probe" in l]
        assert not runs, f"the belt ran while resting: {runs}"
        assert probes, "no light probe while resting"
        with fake.lock:
            fake.cubes_left = 4                             # the operator refills
        t_refill = time.monotonic()
        wait(lambda s: fake.picked >= picked0 + 1, 120, "the first cube after the refill")
        took = time.monotonic() - t_refill
        ev = [x for x in opt._evidence if "rest probe" in x]
        assert ev, "restarted, but not by a light probe"
        wait(lambda s: fake.picked >= picked0 + 4, 200, "all 4 refilled cubes")
        return took, ev[-1], len(probes)
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass


def main():
    took, ev, n = run(True)
    print(f"rest:  {n} light probe(s) in 30 s of empty rest, belt never ran, no false restart   OK")
    print(f"ir:    feeder IR works -> first cube picked {took:.0f} s after the refill ({ev[:70]})")
    assert took < 25.0, took
    print("       all 4 refilled cubes picked; the belt did not move while resting   OK")
    took2, ev2, _ = run(False)
    print(f"noir:  feeder IR dead -> first cube picked {took2:.0f} s after the refill (20 s probes)")
    assert took2 < 45.0, took2
    print("       all 4 refilled cubes picked   OK")
    print("\nall rest-probe sim tests passed")


if __name__ == "__main__":
    main()
