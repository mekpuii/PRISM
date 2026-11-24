# test_experiment_sim.py - run: ~/db-env/bin/python test_experiment_sim.py     (needs pyserial: db-env or autoen-env)
# End-to-end WITHOUT the robot: the real auto_en_new.py runs against dashboard/sim_mega.FakeMega (its serial.Serial is
# replaced, so no port is opened), with --no-mqtt and the browser disabled; experiment.py then drives it through its
# web API on a free localhost port: calibrate -> baseline -> a 2-block plan. Checks pause_before_set gives exactly
# n_cycles per set, /api/task runs the belt without FEED, and every cycle is classified.
# Files go to a fresh /tmp/prism_exp_sim_* folder that is left in place (nothing is deleted).
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import csv
import glob
import json
import os
import socket
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PRISM = os.path.dirname(os.path.dirname(HERE))
sys.path[:0] = [HERE, os.path.join(PRISM, "autoencoder"), os.path.join(PRISM, "dashboard")]

import auto_en_new as A                                     # noqa: E402
import experiment as E                                      # noqa: E402
from sim_mega import FakeMega                               # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_exp_sim_")


class EmptyBeltMega(FakeMega):
    """sim_mega always 'sees' a block 1.2 s after CONV. Here a block only arrives if FEED put one on the belt,
    so a belt_run task (CONV with no FEED) keeps running like the real empty belt."""

    def __init__(self, *a, **k):
        self.cube_on_belt = False
        super().__init__(*a, **k)

    def _cmd(self, line):
        super()._cmd(line)
        p = line.split(",")
        if p[0] == "X" and len(p) > 1:
            if p[1] == "FEED":
                self.cube_on_belt = True
            elif p[1] == "CONV":
                if self.cube_on_belt:
                    self.cube_on_belt = False
                else:
                    self.arrive_at = None


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def start_autoen():
    fake = EmptyBeltMega(fast=8.0)
    A.serial.Serial = lambda *a, **k: fake                  # autoen "opens" the fake instead of a port
    A.webbrowser.open = lambda *a, **k: None
    port = free_port()
    out = {}
    argv = ["--port", "SIMULATED", "--grip-cal", "", "--no-mqtt", "--web-port", str(port), "--log-dir", os.path.join(TMP, "riff"),
            "--sets", "1", "--gap", "0"]
    th = threading.Thread(target=lambda: out.setdefault("rc", A.main(argv)), daemon=True)
    th.start()
    api = E.Api(f"http://127.0.0.1:{port}")
    end = time.monotonic() + 30
    while time.monotonic() < end:
        try:
            if api.state()["status"] == "ready":
                return fake, api, th, out
        except E.ApiError:
            pass
        time.sleep(0.2)
    raise AssertionError("simulated autoen did not come up")


def common(api):
    return ["--api", api.base, "--runs-dir", os.path.join(TMP, "runs"), "--belt-ref", os.path.join(TMP, "belt_ref.json"),
            "--no-record", "--settle-s", "1"]


def newest_meta(kind):
    return json.load(open(max(glob.glob(os.path.join(TMP, "runs", f"{kind}_*.meta.json")), key=os.path.getmtime)))


def main():
    fake, api, th, out = start_autoen()
    ask = lambda prompt: ""                                 # noqa: E731 - the "operator" presses Enter, bins: skip
    try:
        # ---- 1. calibrate (starts autoen: PREPARE, then the pause before set 1)
        args = E.build_parser().parse_args(["calibrate", "--rpms", "6.7,11.1", "--seconds", "4"] + common(api))
        assert E.cmd_calibrate(args, api, ask=ask, recorder_factory=E.NullRecorder) == 0
        ref = E.BeltRef(os.path.join(TMP, "belt_ref.json"))
        for r in (6.7, 11.1):
            e = ref.get(r)
            assert e and abs(e["rpm_meas_mean"] - r) / r < 0.05 and e["n"] >= 10, e
        st = api.state()
        assert st["status"] == "paused" and st["set_boundary"] and st["cycle_total"] == 0, "calibration must run no cycle"
        assert "X,FEED" not in fake.log, "calibration must never FEED"
        print("calibrate: OK", {k: v["rpm_meas_mean"] for k, v in ref.data["entries"].items()})

        # ---- 2. baseline idle, reusing the paused autoen
        args = E.build_parser().parse_args(["baseline", "--kind", "idle", "--seconds", "2"] + common(api))
        assert E.cmd_baseline(args, api, ask=ask, recorder_factory=E.NullRecorder) == 0
        assert newest_meta("base")["result"]["status"] == "done"
        print("baseline: OK")

        # ---- 3. a plan: 2 configs x 1 repeat x 2 cycles (sim gripper closes at 202 deg: widen the block window)
        plan = {"name": "sim", "seed": 1, "repeats": 1, "defaults": {"n_cycles": 2, "carry_speed": 2.0,
                "set": {"grip_contact_max_deg": 210, "dwell_standby": 0, "dwell_grip": 0, "dwell_drop": 0, "dwell_home": 0}},
                "configs": [{"rpm": 6.7, "arm_speed": 2.0}, {"rpm": 11.1, "arm_speed": 3.0}]}
        p = os.path.join(TMP, "plan.json")
        json.dump(plan, open(p, "w"))
        args = E.build_parser().parse_args(["run", "--plan", p, "--belt-check-s", "4"] + common(api))
        rc = E.cmd_run(args, api, ask=ask, recorder_factory=E.NullRecorder, poll=0.2)
        meta = newest_meta("exp")
        assert rc == 0, meta.get("error")
        assert [b["status"] for b in meta["blocks"]] == ["complete", "complete"], [b["reason"] for b in meta["blocks"]]
        for b in meta["blocks"]:
            assert b["belt_check"]["ok"] and len(b["cycles"]) == 2
            assert all(c["class"] in E.RESULT_CLASSES for c in b["cycles"])
            assert all(c["class"] == "OK" for c in b["cycles"]), [(c["class"], c["raw_text"]) for c in b["cycles"]]
        # autoen's own cycles file: exactly 2 cycles in set 1 and 2 in set 2, with the block's settings
        cyc = list(csv.DictReader(open(glob.glob(os.path.join(TMP, "riff", "autoen_cycles_*.csv"))[0])))
        assert [r["set"] for r in cyc] == ["1", "1", "2", "2"], [r["set"] for r in cyc]
        convs = [c for c in fake.log if c.startswith("X,CONV")]
        assert "X,CONV,6.70" in convs and "X,CONV,11.10" in convs
        moves = {int(c.split(",")[3]) for c in fake.log if c.startswith("P,1,")}
        assert {720, 1080} <= moves, f"arm_speed 2.0 / 3.0 -> 720 / 1080 ticks/s, saw {sorted(moves)}"
        snap = meta["config_snapshots"]
        assert all(not s["mismatch"] for s in snap)
        for b, s in zip(meta["blocks"], snap):              # block order is shuffled: compare each with its own block
            assert s["readback"]["arm_speed"] == b["arm_speed"] and s["readback"]["rpm"] == b["rpm"]
        st = api.state()
        assert st["status"] == "paused" and st["set_boundary"], "autoen waits before the next set"
        print("run: OK", [(b["block"], b["counts"]["OK"]) for b in meta["blocks"]])
    finally:
        try:
            api.post("/api/stop")
        except E.ApiError:
            pass
        th.join(15)
    assert out.get("rc") is not None, "autoen did not exit after Stop"
    tx = open(glob.glob(os.path.join(TMP, "riff", "autoen_tx_*.log"))[0]).read()
    assert tx.count("## task") >= 2 * 4, "every task is logged in the tx log (start + result)"
    sets = [r["set"] for r in csv.DictReader(open(glob.glob(os.path.join(TMP, "riff", "autoen_sets_*.csv"))[0]))]
    assert sets.count("2") == sets.count("1"), "Stop at the set boundary must not add a stray partial set row"
    print(f"\nall experiment sim tests passed  (files in {TMP})")


def default_is_unchanged():
    """pause_before_set left at 0: autoen runs its sets straight through, as before the patch."""
    fake = EmptyBeltMega(fast=8.0)
    A.serial.Serial = lambda *a, **k: fake
    log = os.path.join(TMP, "riff_default")
    rc = A.main(["--port", "SIMULATED", "--grip-cal", "", "--no-mqtt", "--web-port", "0", "--auto-start", "--yes", "--sets", "2",
                 "--cycles-per-set", "2", "--gap", "0", "--grip-contact-max-deg", "210", "--log-dir", log])
    cyc = list(csv.DictReader(open(glob.glob(os.path.join(log, "autoen_cycles_*.csv"))[0])))
    assert rc == 0 and [r["set"] for r in cyc] == ["1", "1", "2", "2"] and all(r["result"] == "OK" for r in cyc), cyc
    assert not any(l.startswith("## task") for l in open(glob.glob(os.path.join(log, "autoen_tx_*.log"))[0]))
    print("default (pause_before_set=0) unchanged: OK")


if __name__ == "__main__":
    main()
    default_is_unchanged()
