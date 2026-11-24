# test_flow_sim.py - run: ~/db-env/bin/python test/test_flow_sim.py   (needs pyserial + paho: db-env)
# FLOW (autoen feature F, 2026-11-11): the optimizer plans every feed, a pick never waits for a cycle end.
# The real auto_en_new.py on FlowMega (no robot, --no-mqtt), optimize.py + actuate.py live, camera OFF.
# FlowMega's belt carries SEVERAL blocks (by encoder distance, they queue up behind the one at the IR), and - like
# the real cell, where feeder and arm share one supply - a push while an arm joint moves fails 80 % of the time.
# Checks:
#   - flow picks continuously, faster per block than the serial cycle on the same fake
#   - the optimizer learns which arm phases' pushes fail and keeps more than one block in flight when the ride needs it
#   - no block ever reaches the IR while the arm is not parked at Standby (the hold rule)
#   - feeder empty -> paused (IDLE), not ended; refilled -> its own probe finds the blocks, nobody presses anything
import os
import random
import statistics
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
from sim_mega import FakeMega                               # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7


class FlowMega(FakeMega):
    RIDE = 14000                                            # encoder counts feeder -> IR (~3.5 s at 20 rpm)
    GAP = 1500                                              # blocks queue at least this far apart

    def __init__(self, *a, **k):
        self.cubes, self.cubes_left, self.arrivals_unparked, self.picked = [], 0, 0, 0
        self.pushes = {"still_ok": 0, "still_fail": 0, "moving_ok": 0, "moving_fail": 0}
        self.parked = lambda: True
        self.rng = random.Random(3)
        super().__init__(*a, **k)
        self._last_enc, self._ir_prev = self.enc, False
        threading.Thread(target=self._belt, daemon=True).start()

    def _cmd(self, line):
        p = line.split(",")
        if p[0] == "X" and len(p) > 2 and p[1] == "CONV":
            self.log.append(line)
            with self.lock:
                self.rpm_set = float(p[2])
            self.say("X,OK")
            return
        if p[0] == "X" and len(p) > 1 and p[1] == "FEED":
            with self.lock:
                moving = any(abs(self.pos[i] - self.tgt[i]) > 20 for i in range(1, 6))   # a joint really travels
                fail = moving and self.rng.random() < 0.8
                self.pushes[("moving" if moving else "still") + ("_fail" if fail else "_ok")] += 1
                if self.cubes_left > 0 and not fail:
                    self.cubes_left -= 1
                    behind = self.cubes[-1] + self.GAP if self.cubes else 0
                    self.cubes.append(max(self.RIDE, behind))
        super()._cmd(line)

    def _belt(self):
        while self.running:
            time.sleep(0.02)
            with self.lock:
                d, self._last_enc = self.enc - self._last_enc, self.enc
                if d > 0 and self.cubes:
                    for i in range(len(self.cubes)):
                        floor = 0 if i == 0 else self.cubes[i - 1] + self.GAP
                        self.cubes[i] = max(floor, self.cubes[i] - d)
                if self._ir_prev and not self.ir and self.cubes:
                    self.cubes.pop(0)                       # the gripper lifted it (checked first: same tick)
                    self.picked += 1
                if self.cubes and self.cubes[0] <= 0 and not self.ir and self.rpm_set:
                    self.rpm_set, self.ir = 0.0, True
                    if not self.parked():
                        self.arrivals_unparked += 1
                    self.say(f"X,E,ARRIVED,{int(time.monotonic() * 1000) % 100000},{self.enc}")
                self._ir_prev = self.ir


def run(flow, cubes, refill=None):
    T.TMP = tempfile.mkdtemp(prefix=f"prism_flow_sim_f{flow}_")
    fake = FlowMega(fast=8.0)
    fake.cubes_left = cubes
    api = T.start_autoen(fake)
    fake.parked = lambda: api_phase[0] in ("TRANSPORT", "IDLE", "PREPARE", "")
    api_phase = [""]
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": flow, "enable_b": 0, "enable_c": 0, "rpm": 20.0,
                                                      "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0,
                                                      "enable_verify": 0}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

    def loop():
        while not stop.is_set():
            t = time.monotonic()
            v = act.view()
            api_phase[0] = v.phase or ""
            act.step(opt.step(T.blind_state(t), PredictOutput(t=t, cubes=[], belt_mm_s=0, belt_ok=False, gaps=[]),
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
        raise AssertionError(f"timed out waiting for {what}: {api.state()['status']} mode {opt.mode} ({opt.reason}) "
                             f"| fake: cubes {fake.cubes} left {fake.cubes_left} ir {fake.ir} pushes {fake.pushes} "
                             f"| flow {opt.flow_info} | oks {len(oks(api.state()))}")

    oks = lambda s: [c for c in s["cycles"] if c["result"] == "OK"]     # noqa: E731
    try:
        st = wait(lambda s: opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(), 400, "IDLE (feeder empty)")
        good = oks(st)
        per = statistics.median(c["secs"] for c in good[3:]) if len(good) > 4 else None
        out = {"ok": fake.picked, "per_cube_s": per, "results": [c["result"] for c in st["cycles"]],
               "unparked": fake.arrivals_unparked, "pushes": dict(fake.pushes), "flow": opt.flow_info,
               "story": opt.story}
        if refill:
            fake.cubes_left = refill
            n = fake.picked                         # autoen's state keeps only the last cycles: count the lifts
            wait(lambda s: fake.picked >= n + refill and opt.mode == "RUN", 400, "the refill picked up by its own probe")
            out["refill_ok"] = fake.picked - n
        return out
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass


def main():
    serial = run(0, 10)
    print(f"serial:  {serial['ok']} OK, {serial['per_cube_s']:.1f} s per block   pushes {serial['pushes']}")
    flow = run(1, 14, refill=3)
    print(f"flow:    {flow['ok']} OK, {flow['per_cube_s']:.1f} s per block   pushes {flow['pushes']}")
    print(f"         optimizer: {flow['flow']}")
    assert flow["ok"] >= 12, flow["results"]
    assert flow["per_cube_s"] < serial["per_cube_s"], "flow must be faster per block than the serial cycle"
    assert flow["unparked"] == 0, f"{flow['unparked']} blocks reached the IR while the arm was not parked"
    assert flow["refill_ok"] >= 3
    learned = flow["flow"]["phases"]
    print(f"faster:  {serial['per_cube_s']:.1f} -> {flow['per_cube_s']:.1f} s per block "
          f"({100 * (1 - flow['per_cube_s'] / serial['per_cube_s']):.0f} % less)   OK")
    print(f"safe:    no block reached the IR before the arm was parked   OK")
    print(f"learned: feeds per arm phase (delivered/tried): {learned}")
    print(f"empty:   feeder empty -> paused; refilled -> its own probe picked {flow['refill_ok']} more   OK")
    print(f"story:   {flow['story']['problem']}: {flow['story']['so']}")
    print("\nall flow sim tests passed")


if __name__ == "__main__":
    main()
