# test_verify_sim.py - run: ~/db-env/bin/python test/test_verify_sim.py   (needs pyserial + paho: db-env)
# pt2 (2026-11-12): EVERY push is verified before the belt moves it (the operator's flow chart). The real
# auto_en_new.py in FLOW on VerifyMega, optimize.py + actuate.py live, with a fake camera and fake belt telemetry:
#   - the servo draws current on a real push (12 V dip ~200 mV), none when "dead"
#   - the camera sees a block appear at the LEFT EDGE (feeder exit) and ride to the right; a phantom blob sits still
#     near the exit (like the real white glare blob) and must never count as a fed block
# Checks:
#   normal   every delivered push is confirmed by the camera ("fed"), the exit position is learned
#   empty    the servo moves but nothing comes out: nudge (1 s belt), look again, "empty", push again; 3 in a row ->
#            IDLE - and the belt only ran the nudges (pt1 ran it for the whole ride every time)
#   dead     the servo does not move and nothing is seen: "empty" at once, pushed again; the 3rd -> ALARM
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (also sets up sys.path)
import test_flow_sim as FS                                  # noqa: E402
import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import Actuate, ApiError                       # noqa: E402
from observe import ArmStatus, BeltObs, CubeObs, WorldState # noqa: E402
from predict import PredictOutput                           # noqa: E402

F.PRIOR_RATE_H, F.MIN_DELAY_S, F.MIN_DELAY_BLIND_S = 2000.0, 3.0, 3.0
O.ARRIVE_MIN_S, O.ARRIVE_FACTOR = 20.0, 2.7
EXIT0, SPAN, PHANTOM = 60.0, 700.0, 96.0                    # px: where blocks appear, how far they ride, the phantom


class VerifyMega(FS.FlowMega):
    def __init__(self, *a, **k):
        self.servo_dead, self.feed_until, self.ids, self.next_id, self.belt_on_s = False, 0.0, [], 100, 0.0
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[0] == "X" and len(p) > 1 and p[1] == "FEED":
            with self.lock:
                self.feed_until = time.monotonic() + 0.3
                n0 = len(self.cubes)
            if self.servo_dead:                             # the blade does not move: nothing can come out
                with self.lock:
                    left, self.cubes_left = self.cubes_left, 0
                super()._cmd(line)
                with self.lock:
                    self.cubes_left = left
            else:
                super()._cmd(line)
            with self.lock:
                if len(self.cubes) > n0:
                    self.ids.append(self.next_id)
                    self.next_id += 1
            return
        super()._cmd(line)

    def _belt(self):
        t = time.monotonic()
        while self.running:
            time.sleep(0.02)
            now = time.monotonic()
            with self.lock:
                if self.rpm_set:
                    self.belt_on_s += now - t
                n = len(self.cubes)
            t = now
            self._step_belt()

    def _step_belt(self):
        with self.lock:
            d, self._last_enc = self.enc - self._last_enc, self.enc
            if d > 0 and self.cubes:
                for i in range(len(self.cubes)):
                    floor = 0 if i == 0 else self.cubes[i - 1] + self.GAP
                    self.cubes[i] = max(floor, self.cubes[i] - d)
            if self._ir_prev and not self.ir and self.cubes:
                self.cubes.pop(0)
                self.ids.pop(0)
                self.picked += 1
            if self.cubes and self.cubes[0] <= 0 and not self.ir and self.rpm_set:
                self.rpm_set, self.ir = 0.0, True
                if not self.parked():
                    self.arrivals_unparked += 1
                self.say(f"X,E,ARRIVED,{int(time.monotonic() * 1000) % 100000},{self.enc}")
            self._ir_prev = self.ir

    def world(self, t, seq):
        """What MQTT would bring: belt telemetry (feed state, 12 V) and the camera's tracks."""
        with self.lock:
            pushing = t < self.feed_until
            volts = 12.17 - (0.2 if pushing and not self.servo_dead else 0.0)
            cubes = [CubeObs(id=i, color="orange", along_px=EXIT0 + (self.RIDE - r) / self.RIDE * SPAN, across_px=250.0,
                             along_vel_px_s=0.0, dist_m=None, w_px=30.0, h_px=30.0, hits=5)
                     for i, r in zip(self.ids, self.cubes)]
        cubes.append(CubeObs(id=7, color="white", along_px=PHANTOM, across_px=240.0, along_vel_px_s=0.0, dist_m=None,
                             w_px=200.0, h_px=80.0, hits=50))
        belt = BeltObs(voltage_v=volts, feed=1 if pushing else 0, feed_ir=self.cubes_left > 0, live_t=t)
        return WorldState(t=t, connected=True, joints={}, joints_fresh=False, arm=ArmStatus(), arm_fresh=False,
                          gripper=None, belt=belt, belt_fresh=True, cubes=cubes, camera_fresh=True, cam_seq=seq, events=[])


def main():
    T.TMP = tempfile.mkdtemp(prefix="prism_verify_sim_")
    fake = VerifyMega(fast=8.0)
    fake.cubes_left = 8
    api = T.start_autoen(fake)
    phase = [""]
    fake.parked = lambda: phase[0] in ("TRANSPORT", "IDLE", "")
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 1, "enable_b": 0, "enable_c": 0, "rpm": 20.0,
                                                      "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0,
                                                      "enable_verify": 0}})
    opt = O.Optimize(forecast=F.Forecast(path=os.path.join(T.TMP, "forecast.json")), auto_resume=True,
                     learned_path=os.path.join(T.TMP, "learned.json"))
    opt.feed_moved = 10                                     # the dip check has seen real pushes on this machine
    act = Actuate(api=api, mqtt_enabled=False, log_path=os.path.join(T.TMP, "orch.jsonl"))
    stop = threading.Event()

    def loop():
        seq, last = 0, 0.0
        while not stop.is_set():
            t = time.monotonic()
            if t - last >= 0.17:                            # the camera: ~6 frames per second
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
        raise AssertionError(f"timed out waiting for {what}: mode {opt.mode} ({opt.reason}) | stats {opt.feed_stats} "
                             f"| flow {opt.flow_info} | picked {fake.picked} left {fake.cubes_left}")

    try:
        # 1. normal: 8 cubes, every delivered push confirmed by the camera, then the feeder runs empty
        wait(lambda s: fake.picked >= 8, 300, "8 cubes picked")
        fed = opt.feed_stats.get("verified_fed", 0)
        assert fed >= 8, opt.feed_stats
        assert opt._exit_u < 200, f"exit learned at {opt._exit_u} px (blocks appear at {EXIT0})"
        assert all(e != 7 for e in opt._feed_track.values()), "the phantom blob was taken for a fed block"
        print(f"normal:  8 picked, {fed} pushes confirmed by the camera, exit learned at {opt._exit_u:.0f} px "
              f"(blocks appear at {EXIT0:.0f}), phantom at {PHANTOM:.0f} never counted   OK")

        # 2. empty: the servo moves, nothing comes out -> nudge, look again, empty, 3 in a row -> IDLE
        belt0 = fake.belt_on_s
        st = wait(lambda s: opt.mode == "IDLE" and s["status"] == "paused" and not act.busy(), 200, "IDLE (empty)")
        belt = fake.belt_on_s - belt0
        empties = opt.feed_stats.get("verified_empty", 0)
        assert empties >= 3, opt.feed_stats
        assert belt < 8.0, f"the belt ran {belt:.1f} s for pushes that brought nothing"
        print(f"empty:   {empties} pushes found empty (servo moved, camera saw nothing even after a 1 s nudge) -> "
              f"paused; the belt ran only {belt:.1f} s for them   OK")
        print(f"story:   {opt.story['problem']}: {opt.story['so']}")

        # 3. dead servo: refilled, but the blade does not move -> empty at once, pushed again, 3rd -> ALARM
        fake.cubes_left, fake.servo_dead = 5, True
        opt._run_clear = 0                                  # 11-12: NOT proven in the run - no current + no block must
                                                            # still be the servo ALARM, never "feeder empty"
        api.post("/api/pause")                              # the operator resumes after the refill
        wait(lambda s: opt.mode == "ALERT", 200, "ALARM (dead servo)")
        assert opt._feed_stop and opt._feed_stop["kind"] == "dead", opt._feed_stop
        print(f"dead:    no current and no block -> pushed again at once, then: {opt.reason[:80]}...   OK")
        tail = [x for x in opt._evidence if "=>" in x][-2:]
        for x in tail:
            print(f"         {x}")
    finally:
        stop.set()
        th.join(2)
        act.stop()
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall verify sim tests passed")


if __name__ == "__main__":
    main()
