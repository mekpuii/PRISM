# test_rescue_sim.py - run: ~/db-env/bin/python test/test_rescue_sim.py   (needs pyserial + paho: db-env; ~/armcam-env)
# EMERGENCY GRIP (2026-11-12, the operator): a missed grip (the gripper says empty) with the cube still at the IR ->
# no feed, no belt; the hand camera (arm_cam/rescue_eye.py --fake: a cube right under the jaws) finds it, the arm
# grips it with the taught grip, goes home, to the bin of its colour, then back to normal. 2 tries; then home, the
# gripper opens/closes 3 times and the run pauses. The real auto_en_new.py in FLOW + the optimizer, on the real belt.
#   A  the first grip misses -> emergency grip catches it on try 1 -> all cubes picked, no feed / belt meanwhile
#   B  every grip misses -> 2 emergency tries -> home, 3 signals, paused, RESCUE_FAILED
import glob
import json
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
import test_fill_sim as FL                                  # noqa: E402
import camgeo                                               # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from predict import PredictOutput                           # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7
OK_TICKS, MISS_TICKS = 2070, 2300                           # 181.9 deg (inside 178-186) / 202 deg (nothing caught)
DEG = lambda ticks: ticks * 360.0 / 4096.0                  # noqa: E731


class RescueMega(FL.FillMega):
    """The emergency's cube: after the first normal grip misses, a cube lies at the pick point. Its width decides where the
    fingers stop (cube_pos, gripper ticks; None = nothing there). The gripper's current = 13 mA free + 3 mA per tick the
    gripper is COMMANDED past the cube (squeeze) - so a hard squeeze shows in the current, like a real servo's would."""
    misses = 1                                              # the first N grips miss (and leave the cube at the IR)
    cube_pos = 2020                                         # 177.6 deg (a held cube sat at 177.3-178.6 deg)
    gain = 5.0                                              # mA per tick of squeeze

    def __init__(self, *a, **k):
        self.n_close, self._suppress, self._clear_at = 0, False, None
        self.squeeze, self.cube_here, self.held = 0, False, False
        self.squeeze_at_hold, self.max_cur_ma, self.gentle_closes = None, 0.0, 0
        super().__init__(*a, **k)

    @property
    def clear_at(self):
        return self._clear_at

    @clear_at.setter
    def clear_at(self, v):
        if v is not None and self._suppress:                # a missed grip: the cube stays at the IR
            self._suppress = False
            return
        self._clear_at = v

    def _cmd(self, line):
        if line.startswith("C,"):
            self.n_close += 1
            self._suppress = self.n_close <= self.misses
            self.cube_here = self.cube_here or self._suppress   # the cube stays at the IR after the miss
        super()._cmd(line)
        p = line.split(",")
        if p[0] == "P" and p[1] == "6" and self.cube_here and self.cube_pos is not None:
            target = int(p[2])
            if target < self.cube_pos:                      # closing on the cube: the fingers stop at it
                self.gentle_closes += 1
                self.squeeze = self.cube_pos - target
                self.tgt[6] = self.cube_pos
                if not self.held:
                    self.held = True
                    self.clear_at = time.monotonic() + 1.2  # (the arm lifts it off the IR soon)
            elif self.held and target > self.cube_pos + 60: # opened again (at the bin): the cube is gone
                self.squeeze, self.held, self.cube_here = 0, False, False
        elif p[0] == "H" and p[1] == "6" and self.held:
            if self.squeeze_at_hold is None:                # the FIRST hold = the end of the gentle close (later ones are
                self.squeeze_at_hold = self.squeeze         # the torque-on before releasing the cube at the bin)
            self.squeeze = 0
            self.tgt[6] = self.pos[6]

    def say(self, l):
        if l.startswith("G,CLOSED,"):
            l = f"G,CLOSED,{MISS_TICKS if self._suppress else OK_TICKS}"
        elif l.startswith("T,6,"):                          # the gripper's own current: small when free, rising with squeeze
            f = l.split(",")
            v = int(f[3])
            blocked = self.held and self.cube_pos is not None and int(f[2]) <= self.cube_pos + 2
            ma = 13.0 + (self.gain * self.squeeze if blocked else 0.0)
            self.max_cur_ma = max(self.max_cur_ma, ma) if self.held else self.max_cur_ma
            l = f"T,6,{f[2]},{0 if blocked else v},{max(1, int(round(ma / 6.5)))}"
        super().say(l)

    def world(self, t, seq):
        w = super().world(t, seq)
        w.cubes[:] = [c for c in w.cubes if c.id != 7]
        return w


def run(misses, cubes, fake_eye=True, cube_pos=2020, gain=5.0):
    T.TMP = tempfile.mkdtemp(prefix=f"prism_rescue_sim_{misses}_")
    V.EXIT0, V.SPAN = camgeo.ENTRY_ALONG, camgeo.ENTRY_DIST
    fake = RescueMega(fast=8.0)
    fake.misses = misses
    fake.cube_pos = cube_pos
    fake.gain = gain
    fake.cubes_left = cubes
    fake.shots = os.path.join(T.TMP, "rescue_shots")
    api = T.start_autoen(fake, extra=["--rescue-fake", "1" if fake_eye else "0", "--rescue-dir", fake.shots,
                                      "--rescue-grip-speed", "15"])     # 15 x the fake world's 8 = 120 ticks/s, like the real one
    phase = [""]
    fake.parked = lambda: phase[0] in ("TRANSPORT", "IDLE", "")
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 1, "enable_b": 0, "enable_c": 0, "rpm": 20.0,
                                                      "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0,
                                                      "enable_verify": 1}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    opt.feed_moved = 10
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

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
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | picked {fake.picked} "
                             f"| phase {api.state().get('phase')} | closes {fake.n_close}")
    try:
        if not fake_eye:                                    # no camera: the normal recovery grips it next cycle
            wait(lambda s: fake.picked >= 1 and any(c["result"] == "GRIP_MISS" for c in s["cycles"]), 300,
                 "the missed grip, then the cube picked by the normal retry")
        elif misses == 1:
            wait(lambda s: fake.picked >= cubes, 400, f"{cubes} cubes picked")
        else:
            wait(lambda s: any(c["result"] == "RESCUE_FAILED" for c in s["cycles"]) and s["status"] == "paused",
                 300, "RESCUE_FAILED + paused")
        tx = open(sorted(glob.glob(os.path.join(T.TMP, "riff", "autoen_tx_*.log")))[-1]).read().splitlines()
        return api.state(), tx, fake
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass


def window(tx, start, end):
    i = next(k for k, l in enumerate(tx) if start in l)
    j = next((k for k, l in enumerate(tx) if k > i and end in l), len(tx))
    return tx[i:j]


def events(fake):
    with open(os.path.join(fake.shots, "events.jsonl"), encoding="utf-8") as fh:
        return [json.loads(l) for l in fh if l.strip()]


def check_pictures(fake, ev):
    files = [e["file"] for e in ev if e.get("file") and e["event"] in ("scan", "detected", "ready", "gripped")]
    assert files, "no pictures were taken"
    for fn in files:
        path = os.path.join(fake.shots, fn)
        assert os.path.getsize(path) > 2000, path
        stamp = fn.split("_run")[0]                         # YYYYMMDD_HHMMSS: the date and time are in the name
        assert len(stamp) == 15 and stamp[8] == "_", fn
    return files


def main():
    for pos in (2020, 2218, 2367):                          # 177.6, 195.0, 208.0 deg: different ways the cube can lie
        st, tx, fake = run(1, 3, cube_pos=pos)
        ev = events(fake)
        names = [e["event"] for e in ev]
        for need in ("start", "scan", "detected", "ready", "gripped", "held", "placed"):
            assert need in names, (pos, need, names)
        g = next(e for e in ev if e["event"] == "gripped")
        assert abs(g["contact_deg"] - DEG(pos)) < 1.5, (pos, g)
        assert fake.squeeze_at_hold is not None and 0 < fake.squeeze_at_hold <= 8, \
            f"squeezed {fake.squeeze_at_hold} ticks at hold (the preload is 6)"
        assert fake.max_cur_ma <= 60.0, f"the gripper current reached {fake.max_cur_ma:.0f} mA"
        files = check_pictures(fake, ev)
        w = window(tx, "EMERGENCY GRIP", "cube placed - back to normal")
        bad = [l for l in w if "TX X,FEED" in l or ("TX X,CONV," in l and float(l.split("CONV,")[1]) >= 1.0)]
        assert not bad, f"feed / belt during the emergency: {bad}"
        print(f"A  cube stopping the fingers at {DEG(pos):.1f} deg -> gripped at {g['contact_deg']} deg (felt by "
              f"{g['how']}), squeeze at hold {fake.squeeze_at_hold} ticks, current peak {fake.max_cur_ma:.0f} mA "
              f"(limit 60), {len(files)} pictures, no feed or belt meanwhile   OK")
        assert fake.picked >= 3, fake.picked
    print(f"   emergency #{ev[0]['id']} log: {' > '.join(names)}")

    st, tx, fake = run(1, 3, cube_pos=2020, gain=12.0)       # a stiff servo: the preload already shows as a high current
    assert fake.squeeze_at_hold is not None and fake.squeeze_at_hold < 6, fake.squeeze_at_hold   # it backed off
    print(f"D  a stiff gripper (12 mA per tick): it backed off the preload to {fake.squeeze_at_hold} ticks and "
          f"held it   OK")

    st, tx, fake = run(99, 2, cube_pos=None)                # nothing between the fingers: they close right through
    res = [c for c in st["cycles"] if c["result"] == "RESCUE_FAILED"]
    sig = [l for l in tx if "signal: close" in l]
    tries = [l for l in tx if "## emergency grip try" in l]
    assert res and st["status"] == "paused", (st["status"], [c["result"] for c in st["cycles"]])
    assert len(tries) == 2, tries
    ev = events(fake)
    names = [e["event"] for e in ev]
    assert names.count("try_failed") == 2 and names[-1] == "failed", names
    assert any("felt nothing" in str(e.get("why")) for e in ev if e["event"] == "try_failed"), ev
    k = next(i for i, l in enumerate(tx) if "phase RESCUE_FAILED" in l)
    g = [int(l.split("P,6,")[1].split(",")[0]) for l in tx[k:] if "TX P,6," in l][:6]
    assert len(g) == 6 and g[0] < g[1] and g[2] < g[3] and g[4] < g[5], f"the 3x close/open signal: {g}"
    pics = [e["file"] for e in ev if e.get("file")]
    print(f"B  nothing between the fingers -> {len(tries)} tries ('felt nothing'), home, gripper close/open x3, paused, "
          f"RESCUE_FAILED; {len(pics)} pictures taken   OK")

    os.environ["SO101_CAM"] = "/dev/no-such-wrist-camera"   # the eye starts, but its camera can not open
    st, tx, fake = run(1, 3, fake_eye=False)
    assert any("emergency grip not possible" in l for l in tx), "no fallback note"
    assert not any(c["result"] == "RESCUE_FAILED" for c in st["cycles"]), "paused although the camera was missing"
    print("C  no hand camera -> the normal recovery: GRIP_MISS, the next cycle gripped it again; no RESCUE_FAILED   OK")
    print("\nall rescue sim tests passed")


if __name__ == "__main__":
    main()
