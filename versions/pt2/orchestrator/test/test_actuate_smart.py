# test_actuate_smart.py - the smarter actuator against a fake autoen API (no robot, no autoen, no broker):
# autoen restarts, stale jobs, a pause toggle that crosses someone else's, retries when autoen does not answer.
# Run (orch-env, from orchestrator/):  python3 test/test_actuate_smart.py     Expected last line: "all actuator tests passed"
import os
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import actuate as A                                                # noqa: E402
from optimize import Plan                                          # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_act_smart_")
A.RETRY_WAITS = (0.05, 0.05, 0.05)


class FakeApi:
    base = "fake"

    def __init__(self):
        self.lock = threading.Lock()
        self.st = {"status": "running", "paused": False, "rest_state": "awake", "phase": "TRANSPORT",
                   "cycle_total": 0, "config": {"rpm": 20.0}, "cycles": [], "task": None}
        self.calls = []
        self.down = 0                  # this many next calls get no answer at all
        self.crossed = 0               # this many next pause presses are cancelled by "someone else"
        self.refuse_config = False

    def _gate(self, what):
        self.calls.append(what)
        if self.down > 0:
            self.down -= 1
            raise A.ApiError("autoen API not reachable at fake (refused)", transient=True)

    def state(self):
        with self.lock:
            self._gate("state")
            return {**self.st, "config": dict(self.st["config"]), "cycles": list(self.st["cycles"])}

    def post(self, path, body=None):
        with self.lock:
            self._gate(path)
            if path == "/api/pause":
                if self.crossed > 0:
                    self.crossed -= 1          # someone pressed Pause at the same moment: the two cancel out
                else:
                    self.st["paused"] = not self.st["paused"]
                return {}
            if path == "/api/config":
                if self.refuse_config:
                    raise A.ApiError("/api/config: bad value")
                self.st["config"].update(body)
                return {"applied": body}
            return {}

    def cycle(self, n, result="OK"):
        with self.lock:
            self.st["cycle_total"] = n
            self.st["cycles"] = (self.st["cycles"] + [{"n": n, "result": result, "secs": 12.0, "notes": ""}])[-12:]


def plan(mode="RUN", action=None, seq=1):
    return Plan(t=time.monotonic(), mode=mode, rpm=20.0, arm_speed=1.5, carry_speed=1.5, arrive_timeout=30.0,
                reason="test", action=action, seq=seq)


def until(pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return
        time.sleep(0.05)
    raise AssertionError(f"timed out: {what}")


def new_act(api, name):
    return A.Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(TMP, name + ".jsonl"))


def handed(act, secs=1.2):
    out, end = [], time.monotonic() + secs
    while time.monotonic() < end:
        out += [c["n"] for c in act.view().new_cycles]
        time.sleep(0.1)
    return out


def main():
    # 1. autoen restarts (or Start is pressed again): cycle numbers go back to 1 -> still handed to the optimizer.
    api = FakeApi()
    for n in range(1, 8):
        api.cycle(n)
    act = new_act(api, "restart")
    try:
        until(lambda: act._last_n == 7, 3, "first poll")
        api.cycle(8)
        assert handed(act) == [8]
        with api.lock:
            api.st["cycles"], api.st["cycle_total"] = [], 0
        api.cycle(1)
        api.cycle(2)
        got = handed(act)
        assert got == [1, 2] and act.restarts == 1, (got, act.restarts)
        print(f"restart:   autoen restarted at cycle 8 -> new cycles {got} still reach the optimizer   OK")
    finally:
        act.stop()

    # 2. A refill while the idle job still waits for the pause to land: the resume is not stuck behind it.
    api = FakeApi()
    act = new_act(api, "stale")
    try:
        act.step(plan("IDLE", "idle", 1))
        until(lambda: api.st["paused"] and act._busy == "idle", 3, "idle waiting for the cycle end")
        t0 = time.monotonic()
        act.step(plan("RUN", "resume", 2))
        until(lambda: not api.st["paused"] and not act.busy(), 5, "resume")
        secs = time.monotonic() - t0
        assert secs < 3 and act.last_job == "resume ok", (secs, act.last_job)
        assert "/api/task" not in api.calls, "the stale idle job still drove home"
        print(f"stale:     resume came while idle waited for the pause -> running again in {secs:.1f} s "
              f"(before: up to {30 + 90} s)   OK")

        # 3. Settings queued in RUN but the mode is ALERT by the time they would go: not sent.
        act._last_plan = plan("ALERT", None, 3)
        act._jobs.put(("config", {"rpm": 15.0}))
        until(lambda: not act.busy(), 2, "config job")
        assert api.st["config"]["rpm"] == 20.0, "sent settings in ALERT"
        print("mode:      settings queued before an ALERT are not sent in ALERT   OK")
    finally:
        act.stop()

    # 4. Someone presses Pause in the same moment as our pause: the toggle cancels out -> noticed, pressed once more.
    api = FakeApi()
    api.crossed = 1
    act = new_act(api, "toggle")
    try:
        act.step(plan("IDLE", "pause", 1))
        until(lambda: not act.busy(), 8, "pause job")
        presses = api.calls.count("/api/pause")
        assert api.st["paused"] and presses == 2 and act.last_job == "pause ok", (api.st["paused"], presses)
        api.crossed = 0
        act.step(plan("RUN", "resume", 2))
        until(lambda: not act.busy(), 5, "resume job")
        assert not api.st["paused"] and api.calls.count("/api/pause") == 3
        print("toggle:    a crossed pause press was noticed and put right (2 presses), resume checked   OK")
    finally:
        act.stop()

    # 5. autoen does not answer twice: the settings still go through. autoen says "no": not repeated.
    api = FakeApi()
    act = new_act(api, "retry")
    try:
        until(lambda: act.view().ok, 3, "first poll")
        act._last_plan = plan("RUN")
        api.down = 2
        act._jobs.put(("config", {"rpm": 15.0}))
        until(lambda: not act.busy() and act.last_job.startswith("config"), 5, "config job")
        assert api.st["config"]["rpm"] == 15.0 and act.last_job == "config ok", act.last_job
        api.refuse_config = True
        n = api.calls.count("/api/config")
        act._jobs.put(("config", {"rpm": 8.0}))
        until(lambda: act.last_job == "config failed", 3, "refused config")
        assert api.calls.count("/api/config") == n + 1, "a refusal was retried"
        h = act.health()
        assert h["jobs"]["config"]["ok"] == 1 and h["jobs"]["config"]["failed"] == 1, h
        print(f"retry:     no answer twice -> settings still applied; a refusal is not repeated; health {h['jobs']}   OK")
    finally:
        act.stop()

    print("\nall actuator tests passed")


if __name__ == "__main__":
    main()
