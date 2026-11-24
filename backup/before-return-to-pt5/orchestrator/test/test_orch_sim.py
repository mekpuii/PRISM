# test_orch_sim.py - run: ~/db-env/bin/python test/test_orch_sim.py   (needs pyserial + paho: db-env)
# End-to-end WITHOUT the robot: the real auto_en_new.py runs against dashboard/sim_mega.FakeMega (no serial port,
# --no-mqtt), and optimize.py + actuate.py drive it LIVE through its web API on a free localhost port.
# The fake feeder holds a few cubes; when it runs dry the optimizer must pause autoen BEFORE autoen's
# max_fails (3) stops the run, drive home and brake; after a refill + "resume" it must run again.
# Camera is off here (blind rule: 2 transport timeouts). Files go to a fresh /tmp/prism_orch_sim_* folder.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
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
import optimize as O                                        # noqa: E402
from actuate import Actuate, AutoenApi, ApiError            # noqa: E402
from observe import ArmStatus, BeltObs, WorldState          # noqa: E402
from predict import PredictOutput                           # noqa: E402
from test_experiment_sim import EmptyBeltMega, free_port    # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_orch_sim_")
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 5.0, 0.5                  # 5 s transport timeouts keep the test short


class FeederMega(EmptyBeltMega):
    """A feeder with a fixed number of cubes: FEED with an empty feeder puts nothing on the belt."""

    def __init__(self, *a, **k):
        self.cubes_left = 0
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        empty = p[0] == "X" and len(p) > 1 and p[1] == "FEED" and self.cubes_left <= 0
        super()._cmd(line)
        if p[0] == "X" and len(p) > 1 and p[1] == "FEED":
            if empty:
                self.cube_on_belt = False
            else:
                self.cubes_left -= 1


def start_autoen(fake, extra=()):
    A.serial.Serial = lambda *a, **k: fake
    # pt5: autoen's feeder -> IR prior is the REAL belt (33000 counts); a fake belt tells its own length (FlowMega 14000)
    A.Study.FLOW_PRIOR_COUNTS = getattr(fake, "RIDE", 33000)
    A.webbrowser.open = lambda *a, **k: None
    port = free_port()
    argv = ["--port", "SIMULATED", "--grip-cal", "", "--no-mqtt", "--web-port", str(port),
            "--log-dir", os.path.join(TMP, "riff"), "--gap", "1.0", *extra]
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


def blind_state(t):
    return WorldState(t=t, connected=True, joints={}, joints_fresh=False, arm=ArmStatus(), arm_fresh=False,
                      gripper=None, belt=BeltObs(), belt_fresh=False, cubes=[], camera_fresh=False, cam_seq=0, events=[])


def main():
    fake = FeederMega(fast=8.0)
    fake.cubes_left = 4
    api = start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 6.7, "arm_speed": 1.0,
                                                      "enable_verify": 0}})   # the fake gripper catches nothing

    opt = O.Optimize()
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(TMP, "orch.jsonl"))
    plans, stop = [], threading.Event()

    def loop():
        while not stop.is_set():
            t = time.monotonic()
            p = opt.step(blind_state(t), PredictOutput(t=t, cubes=[], belt_mm_s=0, belt_ok=False, gaps=[]),
                         act.view(), act.commands())
            act.step(p)
            if not plans or plans[-1].seq != p.seq:
                plans.append(p)
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
            time.sleep(0.3)
        raise AssertionError(f"timed out waiting for {what}: {api.state()['status']} mode {opt.mode}")

    try:
        # 1. settings reach autoen at a boundary
        st = wait(lambda s: s["config"]["rpm"] == 20.0 and s["config"]["arm_speed"] == 1.5, 90, "settings 20/1.5")
        print(f"settings:  autoen now {st['config']['rpm']:g} rpm / arm {st['config']['arm_speed']:g}, "
              f"arrive_timeout {st['config']['arrive_timeout']:g} s   OK")

        # 2. feeder runs dry -> IDLE: paused (not stopped), arm home, belt braked
        wait(lambda s: s["status"] == "paused" and opt.mode == "IDLE" and not act.busy()
             and (s.get("task") or {}).get("kind") == "belt_run" and s["task"]["status"] == "done", 180, "IDLE + brake")
        cyc = api.state()["cycles"]
        res = [c["result"] for c in cyc]
        # 2026-11-11: a learned ride time ends an empty feed early (NO_BLOCK) instead of the 2 full timeouts
        assert res.count("OK") >= 4 and all(r in ("TIMEOUT_TRANSPORT", "NO_BLOCK") for r in res[-2:]), res
        assert fake.rpm_set == 0.0, "belt must be stopped/braked"
        print(f"idle:      {res.count('OK')} OK, then 2 empty feeds -> paused at home, belt braked, "
              f"autoen still alive   OK")

        # 3. stays idle (no auto-resume), then refill + operator resume -> runs again
        n_before = api.state()["cycle_total"]
        time.sleep(5)
        assert api.state()["cycle_total"] == n_before and api.state()["status"] == "paused", "must stay paused"
        fake.cubes_left = 3
        act._cmds.put("resume")
        wait(lambda s: sum(c["result"] == "OK" for c in s["cycles"] if c["n"] > n_before) >= 3, 120, "3 OK after resume")
        wait(lambda s: s["status"] == "paused" and opt.mode == "IDLE" and not act.busy(), 120, "IDLE again")
        print("resume:    refill + 'resume' -> 3 more OK -> IDLE again   OK")

        # 4. the empty feeds never blamed the settings, and nothing was abnormal
        modes = [p.mode for p in plans]
        assert "ALERT" not in modes and "MANUAL" not in modes, [(p.mode, p.reason) for p in plans]
        cfg = api.state()["config"]
        assert (opt.target, cfg["rpm"], cfg["arm_speed"]) == ((20.0, 1.5), 20.0, 1.5), (opt.target, cfg)
        print("settings:  still 20/1.5 after two empty feeders   OK")
        print("plans:     " + " -> ".join(dict.fromkeys(f"{p.mode}" for p in plans)))
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print(f"\nlog: {act.log_path}")
    print("all orchestrator sim tests passed")


if __name__ == "__main__":
    main()
