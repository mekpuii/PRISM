# test_abort_sim.py - run: ~/db-env/bin/python test/test_abort_sim.py   (needs pyserial + paho: db-env)
# End-to-end WITHOUT the robot, camera ON: the real auto_en_new.py runs against FakeMega (no serial port, --no-mqtt);
# optimize.py + actuate.py drive it LIVE. A fake camera "sees" a moving cube while a fed cube rides the fake belt.
# Checks: the camera becomes trusted, and when the feeder runs dry the optimizer aborts the transport
# (POST /api/abort_transport) so the cycle ends NO_BLOCK within seconds instead of running the belt to arrive_timeout,
# then pauses at home with the belt braked. Files go to a fresh /tmp/prism_abort_sim_* folder.
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from observe import ArmStatus, BeltObs, WorldState          # noqa: E402
from predict import CubeEstimate, PredictOutput             # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_abort_sim_")
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7                 # the real 25 s timeout at 20 rpm: the abort must beat it
O.NO_CUBE_S = 2.0                                           # the fake cube reaches the IR 1.2 s after CONV


def camera(fake, t):
    """A fresh camera that sees one moving cube while a fed cube is on its way to the IR."""
    st = WorldState(t=t, connected=True, joints={}, joints_fresh=False, arm=ArmStatus(), arm_fresh=False, gripper=None,
                    belt=BeltObs(), belt_fresh=False, cubes=[], camera_fresh=True, cam_seq=0, events=[])
    cubes = [CubeEstimate(id=1, color="orange", t=t, pos_mm=-300.0, vel_mm_s=56.0, sigma_mm=2.0, coasting=False,
                          age_s=0.0)] if fake.arrive_at is not None else []
    return st, PredictOutput(t=t, cubes=cubes, belt_mm_s=56.0, belt_ok=True, gaps=[])


def main():
    fake = T.FeederMega(fast=8.0)
    fake.cubes_left = 7
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0}})
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

    try:
        wait(lambda s: opt.camera_trusted(), 120, "camera trusted")
        print(f"trust:   camera trusted after {sum(opt._cam_record)} seen cubes   OK")
        st = wait(lambda s: s["status"] == "paused" and opt.mode == "IDLE" and not act.busy()
                  and (s.get("task") or {}).get("kind") == "belt_run" and s["task"]["status"] == "done", 180, "IDLE")
        cyc = st["cycles"]
        res = [c["result"] for c in cyc]
        assert res.count("OK") == 7 and res[-1] == "NO_BLOCK" and "TIMEOUT_TRANSPORT" not in res, res
        assert cyc[-1]["secs"] < 10, f"the empty cycle must end long before the 25 s timeout: {cyc[-1]}"
        assert fake.rpm_set == 0.0, "belt must be braked"
        print(f"abort:   7 OK, then the empty feed ended NO_BLOCK after {cyc[-1]['secs']} s (timeout 25 s) "
              f"-> paused, home, braked   OK")
        assert opt.configs[(20.0, 1.5)]["n"] == 7 and opt.target == (20.0, 1.5), "the empty feed must not blame 20/1.5"
        print("blame:   settings still 20/1.5, empty feed not counted   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print(f"\nlog: {act.log_path}")
    print("all abort sim tests passed")


if __name__ == "__main__":
    main()
