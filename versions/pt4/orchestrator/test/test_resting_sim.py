# test_resting_sim.py - run: ~/db-env/bin/python test/test_resting_sim.py   (needs pyserial + paho: db-env)
# autoen --resting WITHOUT the robot (dashboard/sim_mega.FakeMega, --no-mqtt): the arm starts on the cradle.
# Checks: Start moves NO joint and ends paused; Resume is refused while resting; the wake task runs the taught
# wake sequence from config/poses.json and ends at Home; Resume then continues to the set boundary
# (pause_before_set = 1), which is the state experiment.py reuses.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import json
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ORCH = os.path.dirname(HERE)
PRISM = os.path.dirname(ORCH)
sys.path[:0] = [ORCH, os.path.join(ORCH, "cal"), os.path.join(PRISM, "autoencoder"), os.path.join(PRISM, "dashboard")]

import auto_en_new as A                                     # noqa: E402
from actuate import AutoenApi, ApiError                     # noqa: E402
from test_experiment_sim import EmptyBeltMega, free_port    # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_resting_sim_")


def main():
    rest = json.load(open(os.path.join(PRISM, "config", "poses.json")))["rest"]
    fake = EmptyBeltMega(fast=8.0)
    fake.pos.update({int(j): int(v) for j, v in rest.items()})   # the arm sits on the cradle
    fake.tgt = dict(fake.pos)
    A.serial.Serial = lambda *a, **k: fake
    A.webbrowser.open = lambda *a, **k: None
    port = free_port()
    argv = ["--port", "SIMULATED", "--grip-cal", "", "--no-mqtt", "--web-port", str(port),
            "--log-dir", os.path.join(TMP, "riff"), "--resting"]
    threading.Thread(target=lambda: A.main(argv), daemon=True).start()
    api = AutoenApi(f"http://127.0.0.1:{port}")
    end = time.monotonic() + 30
    while True:
        try:
            if api.state()["status"] == "ready":
                break
        except ApiError:
            pass
        assert time.monotonic() < end, "simulated autoen did not come up"
        time.sleep(0.2)
    assert api.state()["rest_state"] == "resting"

    def wait(pred, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            st = api.state()
            if pred(st):
                return st
            assert st["status"] not in ("error", "stopped", "finished"), f"{what}: {st['status']} {st['message']}"
            time.sleep(0.2)
        raise AssertionError(f"timed out: {what} ({api.state()['status']})")

    try:
        # 1. Start (experiment.py's start config): nothing moves, autoen pauses
        before = dict(fake.pos)
        api.post("/api/start", {"clear": True, "config": {"pause_before_set": 1, "sets": 1000, "enable_f": 0, "enable_b": 0,
                                                          "enable_c": 0, "cycles_per_set": 12}})
        wait(lambda s: s["status"] == "paused", 30, "paused after Start")
        time.sleep(2)
        moved = {j: (before[j], fake.pos[j]) for j in before if abs(fake.pos[j] - before[j]) > 3}
        assert not moved and fake.tgt == before, f"no joint may move before wake: {moved}"
        print("start:   arm on the cradle, Start -> paused, no joint moved   OK")

        # 2. Resume is refused while resting
        try:
            api.post("/api/pause")
            raise AssertionError("resume must be refused while resting")
        except ApiError as e:
            assert "resting" in str(e), e
        print("guard:   Resume refused while resting   OK")

        # 3. wake task -> taught wake sequence -> Home
        tid = api.post("/api/task", {"kind": "wake", "seconds": 2, "settle_s": 0})["task"]["id"]
        st = wait(lambda s: (s.get("task") or {}).get("id") == tid and s["task"]["status"] in ("done", "error"), 180, "wake")
        assert st["task"]["status"] == "done" and st["rest_state"] == "awake", st["task"]
        home = A.DEFAULT_POSES["right"]
        off = {j: (fake.pos[j], int(home[str(j)])) for j in range(1, 6) if abs(fake.pos[j] - int(home[str(j)])) > 30}
        assert not off, f"arm not at Home after wake: {off}"
        print(f"wake:    wake sequence done in {st['task'].get('move_s')} s, arm at Home   OK")

        # 4. Resume -> on to the set boundary (what experiment.py reuses)
        api.post("/api/pause")
        st = wait(lambda s: s["status"] == "paused" and s.get("set_boundary"), 60, "set boundary")
        print(f"resume:  paused at the set boundary (set {st['set']}), ready for experiment.py   OK")
    finally:
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall resting sim tests passed")


if __name__ == "__main__":
    main()
