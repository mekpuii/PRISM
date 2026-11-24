# test_stability_sim.py - run: ~/db-env/bin/python test/test_stability_sim.py   (needs pyserial + paho: db-env)
# The 2026-11-10 runs, end to end WITHOUT the robot (real auto_en_new.py on FakeMega, optimize.py + actuate.py live):
#   1. Start pressed on the autoen WEB PAGE: its defaults are 2 sets x 6 cycles at 6.7 rpm / 20 s. The optimizer must
#      take the run over: settings in before the first cycle, budget raised - the run must NOT "finish" after 12 cycles.
#   2. Feeder empty -> IDLE; refilled -> a probe picks the cubes up by itself (nobody sends anything).
#   3. The operator presses Resume on the page while IDLE (after a refill): it runs, it is not stopped again at once.
#   4. autoen is restarted and Start pressed again while the SAME optimizer keeps running: the new run's cycles
#      (numbers start at 1 again) must be counted, not taken for the old ones.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_abort_sim as S                                  # noqa: E402  (fake camera)
import test_orch_sim as T                                   # noqa: E402  (FakeMega feeder, start_autoen)
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from incidents import Responder                             # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_stability_sim_")
F.PRIOR_RATE_H, F.MIN_DELAY_S = 2000.0, 3.0                 # probe after ~3 s so the test stays short
PAGE_START = {"clear": True, "config": {"enable_verify": 0, "enable_f": 0, "enable_b": 0, "enable_c": 0, "pause_before_set": 0}}
# everything else = autoen's own defaults, as the web page sends them: sets 2, cycles_per_set 6, rpm 6.7, 20 s


class Recorder(T.FeederMega):
    """Remembers the belt speed of every CONV the Mega is told to run."""

    def __init__(self, *a, **k):
        self.conv = []
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[:2] == ["X", "CONV"] and len(p) > 2:
            self.conv.append(float(p[2]))
        super()._cmd(line)


def main():
    fake = Recorder(fast=8.0)
    fake.cubes_left = 14                                    # more than the page's 12-cycle budget
    api = T.start_autoen(fake)
    d = api.state()["config"]
    assert (d["sets"], d["cycles_per_set"], d["rpm"]) == (2, 6, 6.7), d
    fc = F.Forecast(path=os.path.join(T.TMP, "forecast_state.json"))
    opt = O.Optimize(forecast=fc, auto_resume=True, own_run=True, responder=Responder(path=None))
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    plans, stop, mark, ctx = [], threading.Event(), {}, {"fake": fake}

    def loop():
        while not stop.is_set():
            st, pr = S.camera(ctx["fake"], time.monotonic())
            v = act.view()
            if v.ok and "cfg" not in mark and v.config.get("rpm") == 20.0:
                mark["cfg"] = (v.cycle_total, v.status, v.phase)
            p = opt.step(st, pr, v, act.commands())
            act.step(p)
            if not plans or plans[-1].seq != p.seq:
                plans.append(p)
            time.sleep(0.05)
    th = threading.Thread(target=loop, daemon=True)
    th.start()

    def wait(pred, timeout, what, a=None):
        a = a or api
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            st = a.state()
            if pred(st):
                return st
            if st["status"] in ("error", "stopped", "finished"):
                raise AssertionError(f"autoen ended while waiting for {what}: {st['status']} {st['message']} "
                                     f"(cycle {st['cycle_total']})")
            time.sleep(0.2)
        raise AssertionError(f"timed out waiting for {what}: {a.state()['status']} mode {opt.mode} ({opt.reason})")

    good = lambda: sum(c["ok"] for c in opt.configs.values())          # noqa: E731
    try:
        time.sleep(1.5)                                     # the optimizer is up and sees "ready" (as start_cell.sh has it)
        api.post("/api/start", PAGE_START)
        # 1. the page's 12 cycles do not end the run; settings were in before the first belt run
        st = wait(lambda s: good() == 14 and opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(),
                  300, "14 cubes, then IDLE")
        cfg = st["config"]
        assert st["cycle_total"] > 12 and (cfg["sets"], cfg["rpm"], cfg["arrive_timeout"]) == (1000, 20.0, 25.0), cfg
        assert fake.conv[0] == 20.0, f"the first belt run was at {fake.conv[0]} rpm, not the optimizer's 20"
        print(f"page start: 2 x 6 cycles at 6.7 rpm -> the FIRST belt run already at {fake.conv[0]:g} rpm, budget "
              f"{cfg['sets']} sets: {st['cycle_total']} cycles and still alive, 14 OK   OK")

        # 2. refill -> a probe picks them up, nobody sends anything
        n = good()
        fake.cubes_left = 2
        wait(lambda s: good() == n + 2 and opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(),
             240, "the refill picked up by a probe")
        assert not any("operator" in p.reason for p in plans), [p.reason for p in plans if "operator" in p.reason]
        print("auto:      feeder empty -> IDLE -> refilled -> picked up by its own probe (no command, no button)   OK")

        # 3. the operator refills and presses Resume on the page: it runs (not stopped again at once)
        n = good()
        O.PROBE_FIRST_S, keep = 1e9, O.PROBE_FIRST_S        # no probe now: this must be the operator's resume alone
        F.MIN_DELAY_S = 1e6
        wait(lambda s: opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(), 60, "idle before the button")
        time.sleep(1.0)
        fake.cubes_left = 2
        api.post("/api/pause")
        wait(lambda s: good() == n + 2, 120, "2 cubes after the operator's Resume")
        assert any("resumed by the operator" in p.reason or "a cube arrived" in p.reason for p in plans)
        assert not any(p.mode == "MANUAL" for p in plans), [(p.mode, p.reason) for p in plans if p.mode == "MANUAL"]
        O.PROBE_FIRST_S, F.MIN_DELAY_S = keep, 3.0
        print("button:    Resume pressed on the page while IDLE -> 2 cubes picked; never 'paused by someone else'   OK")

        # 4. autoen restarted, Start again, the same optimizer: the new run's cycles 1.. are counted
        api.post("/api/stop")
        time.sleep(2.0)
        fake2 = Recorder(fast=8.0)
        fake2.cubes_left = 3
        api2 = T.start_autoen(fake2)
        ctx["fake"] = fake2
        act.api = api2                                      # the same address in real life (127.0.0.1:8781)
        n = good()
        time.sleep(1.5)
        api2.post("/api/start", PAGE_START)
        st = wait(lambda s: good() == n + 3, 180, "3 cubes of the second run", api2)
        assert st["config"]["sets"] == 1000 and act.restarts >= 1, (st["config"]["sets"], act.restarts)
        print(f"restart:   Start pressed again with the optimizer still running -> the new run's cycles 1..3 counted "
              f"({good()} good cubes in all), budget raised again   OK")
        ok = [p for p in plans if p.mode == "ALERT"]
        assert not ok, [(p.mode, p.reason) for p in ok]
    finally:
        stop.set()
        th.join(2)
        act.stop()
        for a in (api, locals().get("api2")):
            try:
                a and a.post("/api/stop")
            except ApiError:
                pass
    print(f"\nlog: {act.log_path}")
    print("all stability sim tests passed")


if __name__ == "__main__":
    main()
