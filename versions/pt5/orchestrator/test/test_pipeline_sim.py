# test_pipeline_sim.py - run: ~/db-env/bin/python test/test_pipeline_sim.py   (needs pyserial + paho: db-env)
# The real auto_en_new.py with the PIPELINE on (enable_b = 1: the next cube is fed while the arm grips, the belt runs
# while it carries) against FeederMega (no robot, --no-mqtt), optimize.py + actuate.py driving it live, camera off.
# Checks (2026-11-11):
#   - the optimizer's settings reach autoen in the MIDDLE of the first cycle, not after it
#   - pipelined cycles run OK
#   - the feeder runs empty: autoen does NOT end the run (before: "recovery failed - stopping"), the optimizer pauses
#     it (IDLE, home, braked) and tells the story of why
#   - refilled + resumed: the pipeline is rebuilt first (prepare: open, home, standby, feed) and runs OK again
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
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
from predict import PredictOutput                           # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_pipeline_sim_")


def main():
    fake = T.FeederMega(fast=8.0)
    fake.cubes_left = 5
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 1, "enable_c": 0, "rpm": 6.7, "arm_speed": 1.0,
                                                      "carry_speed": 1.0, "enable_verify": 0}})
    opt = O.Optimize()
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

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
                raise AssertionError(f"autoen ended while waiting for {what}: {st['status']} {st['message']}")
            time.sleep(0.2)
        raise AssertionError(f"timed out waiting for {what}: {api.state()['status']} mode {opt.mode} {opt.reason}")

    def results(st):
        return [c["result"] for c in st["cycles"]]

    try:
        st = wait(lambda s: s["config"]["rpm"] == 20.0 and s["config"]["arm_speed"] == 1.5, 60, "settings 20/1.5")
        assert len(st["cycles"]) == 0, f"the settings waited for a cycle end: {results(st)}"
        print("at once:   20 rpm / arm 1.5 reached autoen during the first cycle, not after it   OK")

        st = wait(lambda s: results(s).count("OK") >= 4, 180, "4 pipelined OK cycles")
        print(f"pipeline:  {results(st).count('OK')} OK pipelined cycles   OK")

        st = wait(lambda s: s["status"] == "paused" and opt.mode == "IDLE" and not act.busy(), 240, "IDLE (feeder empty)")
        res = results(st)
        assert "OK" in res and res[-1] in ("TIMEOUT_TRANSPORT", "NO_BLOCK"), res
        assert st["status"] == "paused", "the empty feeder must not end the run"
        assert fake.rpm_set == 0.0, "belt must be braked"
        assert opt.story and ("IR" in opt.story["problem"] or "feeder empty" in opt.story["problem"]) \
            and opt.story["tried"], opt.story
        print(f"empty:     {res.count('OK')} OK, then the feeder ran dry -> paused (not ended), home, braked   OK")
        print(f"story:     {opt.story['problem']}: {opt.story['so']} -> {opt.story['next']}")

        n_ok = res.count("OK")
        fake.cubes_left = 3                                  # the operator refills ...
        api.post("/api/pause")                               # ... and resumes (toggle)
        st = wait(lambda s: results(s).count("OK") >= n_ok + 2, 180, "OK again after the refill")
        notes = " ".join(c.get("notes") or "" for c in st["cycles"])
        assert "prepare:" in notes, "the pipeline must be rebuilt after the pause"
        print(f"refill:    resumed -> pipeline rebuilt (prepare), {results(st).count('OK') - n_ok} more OK   OK")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print(f"\nlog: {act.log_path}")
    print("all pipeline sim tests passed")


if __name__ == "__main__":
    main()
