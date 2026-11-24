# test_sleep_sim.py - run: ~/db-env/bin/python test/test_sleep_sim.py   (needs pyserial + paho: db-env)
# SLEEP (2026-11-12, the operator: "no cubes for 5-10 minutes - also by the forecast - cut the power with the relay and
# switch it on again after some time to check"). The real auto_en_new.py in FLOW + the optimizer, on the real belt,
# with a FAKE relay (it only records). Times shortened: sleep after 8 s of empty-feeder rest, wake every 15 s.
#   1  the feeder runs dry -> rest -> SLEEP: arm resting (motors off), relay OFF, no belt
#   2  15 s later it wakes: relay ON, arm awake, one light probe -> nothing -> asleep again
#   3  refill -> the feeder IR wakes it within ~3 s -> probe -> a cube -> working; the refilled cubes are picked
import glob
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.environ["PRISM_CAMGEO"] = os.path.join(os.path.dirname(HERE), "camera_geometry.json")
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import test_verify_sim as V                                 # noqa: E402
import test_restprobe_sim as RP                             # noqa: E402
import camgeo                                               # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7
O.SLEEP_AFTER_S, O.SLEEP_EARLY_S, O.WAKE_EVERY_S = 8.0, 9999.0, 15.0


class FakeRelay:
    enabled = True

    def __init__(self):
        self.log = []

    def on(self, force=False):
        self.log.append((time.monotonic(), "on"))
        return "cell power on (fake)"

    def off(self, force=False):
        self.log.append((time.monotonic(), "off"))
        return "cell power off (fake)"


def main():
    T.TMP = tempfile.mkdtemp(prefix="prism_sleep_sim_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST
    fake = RP.RestMega(fast=8.0)
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
    relay = FakeRelay()
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"), relay=relay)
    stop, modes = threading.Event(), []

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
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | relay {relay.log} | "
                             f"rest {api.state().get('rest_state')} | picked {fake.picked}")
    try:
        wait(lambda s: fake.picked >= 3, 300, "3 cubes picked")
        st = wait(lambda s: opt.mode == "SLEEP" and not act.busy() and relay.log and relay.log[-1][1] == "off", 120,
                  "SLEEP with the power off")
        assert st.get("rest_state") != "awake", st.get("rest_state")
        t_sleep = time.monotonic()
        print(f"1  feeder dry -> SLEEP: arm {st.get('rest_state')}, relay OFF  ({opt.reason[:80]})   OK")
        wait(lambda s: len([x for x in relay.log if x[1] == "on"]) >= 1, 60, "the timed wake-up")
        took = [x for x in relay.log if x[1] == "on"][0][0] - t_sleep
        wait(lambda s: opt.mode == "SLEEP" and len([x for x in relay.log if x[1] == "off"]) >= 2 and not act.busy(),
             90, "asleep again after an empty check")
        probes = [x for x in opt._evidence if "rest probe" in x]
        print(f"2  woke after {took:.0f} s (relay ON, arm awake), one probe, nothing -> asleep again   OK")
        tx = open(sorted(glob.glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read()
        assert "feeder probe" in tx, "no probe push after waking"
        picked0 = fake.picked
        with fake.lock:
            fake.cubes_left = 3                             # the operator refills: the feeder IR says "cubes"
        t_refill = time.monotonic()
        wait(lambda s: len([x for x in relay.log if x[1] == "on"]) >= 2, 30, "the feeder IR waking it")
        woke_ir = [x for x in relay.log if x[1] == "on"][1][0] - t_refill
        wait(lambda s: fake.picked >= picked0 + 3, 240, "the refilled cubes picked")
        assert woke_ir < 8.0, woke_ir
        print(f"3  refill -> the feeder IR woke it after {woke_ir:.1f} s -> probe -> working; {fake.picked - picked0} "
              f"refilled cubes picked   OK")
        print(f"   modes: {' > '.join(m for _, m in modes)}")
        del probes
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall sleep sim tests passed")


if __name__ == "__main__":
    main()
