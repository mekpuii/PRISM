# test_rest_memory_sim.py - run: ~/db-env/bin/python test/test_rest_memory_sim.py   (needs pyserial + paho: db-env)
# autoen WITHOUT the robot (FakeMega, --no-mqtt), with --rest-file in a temp folder (never the real rest_state.json).
# Checks: the rest task saves "resting"; a NEW autoen on the same file starts resting by itself (no --resting):
# Start moves no joint and pauses; wake brings the arm Home and saves "awake". Also: --port SIMULATED without
# --rest-file never writes the real autoencoder/rest_state.json.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import json
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (sets up sys.path)
import auto_en_new as A                                     # noqa: E402
from actuate import AutoenApi, ApiError                     # noqa: E402
from test_experiment_sim import EmptyBeltMega, free_port    # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_restmem_sim_")
REST = os.path.join(TMP, "rest_state.json")


def start(fake, extra=()):
    A.serial.Serial = lambda *a, **k: fake
    A.webbrowser.open = lambda *a, **k: None
    port = free_port()
    argv = ["--port", "SIMULATED", "--grip-cal", "", "--no-mqtt", "--web-port", str(port),
            "--log-dir", os.path.join(TMP, "riff"), *extra]
    threading.Thread(target=lambda: A.main(argv), daemon=True).start()
    api = AutoenApi(f"http://127.0.0.1:{port}")
    end = time.monotonic() + 30
    while time.monotonic() < end:
        try:
            if api.state()["status"] == "ready":
                return api
        except ApiError:
            pass
        time.sleep(0.2)
    raise AssertionError("simulated autoen did not come up")


def wait(api, pred, timeout, what):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = api.state()
        if pred(st):
            return st
        assert st["status"] not in ("error", "finished"), f"{what}: {st['status']} {st['message']}"
        time.sleep(0.2)
    raise AssertionError(f"timed out: {what}")


def task(api, kind):
    tid = api.post("/api/task", {"kind": kind, "seconds": 2, "settle_s": 0})["task"]["id"]
    st = wait(api, lambda s: (s.get("task") or {}).get("id") == tid and s["task"]["status"] in ("done", "error"), 120, kind)
    assert st["task"]["status"] == "done", st["task"]
    return st


def saved():
    with open(REST, encoding="utf-8") as f:
        return json.load(f)["rest_state"]


def main():
    real = A.REST_FILE
    real_before = os.path.getmtime(real) if os.path.exists(real) else None

    # 1. first autoen: awake -> rest task -> rest_state.json says resting
    api = start(EmptyBeltMega(fast=8.0), ["--rest-file", REST])
    assert saved() == "awake"
    api.post("/api/start", {"clear": True, "config": {"pause_before_set": 1, "sets": 1000, "enable_f": 0, "enable_b": 0}})
    wait(api, lambda s: s["status"] == "paused" and s.get("set_boundary"), 60, "set boundary")
    task(api, "rest")
    assert saved() == "resting", saved()
    api.post("/api/stop")
    print("rest:     rest task -> rest_state.json = resting   OK")

    # 2. a NEW autoen (restart) on the same file, no --resting: it knows the arm rests; Start moves nothing
    rest_pose = json.load(open(os.path.join(T.PRISM, "config", "poses.json")))["rest"]
    fake = EmptyBeltMega(fast=8.0)
    fake.pos.update({int(j): int(v) for j, v in rest_pose.items()})
    fake.tgt = dict(fake.pos)
    api = start(fake, ["--rest-file", REST])
    assert api.state()["rest_state"] == "resting", api.state()["rest_state"]
    before = dict(fake.pos)
    api.post("/api/start", {"clear": True, "config": {"pause_before_set": 0, "sets": 1, "enable_f": 0, "enable_b": 0}})
    wait(api, lambda s: s["status"] == "paused", 30, "paused after Start")
    time.sleep(2)
    moved = {j: (before[j], fake.pos[j]) for j in before if abs(fake.pos[j] - before[j]) > 3}
    assert not moved, f"no joint may move: {moved}"
    print("restart:  new autoen remembers 'resting', Start moves no joint   OK")

    # 3. wake -> Home, rest_state.json = awake
    task(api, "wake")
    assert saved() == "awake" and api.state()["rest_state"] == "awake"
    api.post("/api/stop")
    print("wake:     wake -> Home, rest_state.json = awake   OK")

    # 4. --awake overrides a file that says resting
    with open(REST, "w", encoding="utf-8") as f:
        json.dump({"rest_state": "resting"}, f)
    api = start(EmptyBeltMega(fast=8.0), ["--rest-file", REST, "--awake"])
    assert api.state()["rest_state"] == "awake"
    api.post("/api/stop")
    print("override: --awake wins over a file that says resting   OK")

    # 5. the simulator without --rest-file never wrote the real memory
    real_after = os.path.getmtime(real) if os.path.exists(real) else None
    assert real_after == real_before, "the real rest_state.json was touched"
    print(f"safe:     real {real} untouched   OK")
    print("\nall rest-memory sim tests passed")


if __name__ == "__main__":
    main()
