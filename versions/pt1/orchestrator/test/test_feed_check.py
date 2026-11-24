# test_feed_check.py - did the feeder servo really push? (optimize._watch_feed, from the 2026-11-10 logs: the Mega
# answers FEED_DONE by time only; a real push dips the 12 V rail 33-125 mV, a dead one <= 6 mV.)
# No robot, no broker, no autoen.
# Run (orch-env, from orchestrator/):  python3 test/test_feed_check.py     Expected last line: "all feed check tests passed"
import os
import sys
import tempfile
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimize as O                                               # noqa: E402
from observe import ArmStatus                                      # noqa: E402
from test_opt import cyc, pred, view                               # noqa: E402
from test_smart import settings, tele                              # noqa: E402

V0 = 12.171


class Cell:
    def __init__(self, opt):
        self.opt, self.t, self.n = opt, 0.0, 1
        self.fir = None                                             # feeder IR: None unknown / True cubes / False empty

    def tick(self, phase, volts, cycles=(), paused=False):
        self.t += 0.1                                               # belt telemetry: 10 Hz
        s = tele(self.t)
        s.belt.voltage_v, s.belt.live_t, s.belt.feed_ir = volts, self.t, self.fir
        s.arm = ArmStatus(phase=phase, live_t=self.t)
        v = replace(view(self.t, phase=phase, cycles=cycles, config=settings(self.opt), paused=paused,
                         status="paused" if paused else "running"), cycle_total=self.n)
        return self.opt.step(s, pred(self.t), v)

    def feed(self, dip_mv):
        """IDLE 1 s, FEED 0.6 s with this dip on the rail, then the first TRANSPORT tick. Returns that plan."""
        self.n += 1
        for _ in range(10):
            self.tick("IDLE", V0)
        for _ in range(6):
            self.tick("FEED", V0 - dip_mv / 1000.0)
        return self.tick("TRANSPORT", V0)

    def end(self, result):
        for _ in range(3):
            self.tick("TRANSPORT", V0)
        return self.tick("IDLE", V0, cycles=[cyc(self.n, result)], paused=self.opt.mode in ("ALERT", "IDLE"))

    def wait_paused(self, secs):
        """Paused at home for `secs`; returns the first plan with an action (or the last one)."""
        p = None
        for _ in range(int(secs * 10)):
            p = self.tick("IDLE", V0, paused=True)
            if p.action:
                return p
        return p


def armed():
    opt = O.Optimize()
    opt.feed_moved = O.FEED_ARM_PUSHES
    opt._run_clear = O.FEED_ARM_RUN                                 # and proven in this run
    c = Cell(opt)
    c.tick("IDLE", V0)
    assert opt.mode == "RUN"
    return opt, c


def main():
    # 1. A real push (80 mV dip): nothing to do. A dead push (0 mV): feed again at once, twice; the third -> ALERT.
    opt, c = armed()
    p = c.feed(80)
    assert p.action is None and opt.feed_stats == {"pushes": 1, "dead": 0, "last_dip_mV": 80}, (p.action, opt.feed_stats)
    c.end("OK")
    p = c.feed(0)
    assert p.action == "refeed" and "did not move" in opt.reason and "(1/2)" in opt.reason, (p.action, opt.reason)
    c.end(O.NO_BLOCK)
    assert opt.mode == "RUN" and opt._refeeds_left == O.REFEEDS, "a dead push used up the camera's refeed"
    p = c.feed(2)
    assert p.action == "refeed" and "(2/2)" in opt.reason, (p.action, opt.reason)
    c.end(O.NO_BLOCK)
    p = c.feed(1)
    assert p.action == "alarm" and opt.mode == "ALERT" and "ALARM" in opt.reason and "D12" in opt.reason, (p.action, opt.reason)
    c.end(O.NO_BLOCK)
    assert opt.mode == "ALERT" and "feeder servo" in opt.reason, (opt.mode, opt.reason)
    print(f"dead push:  no dip on the 12 V rail -> fed again at once (2 x), then: {opt.reason[:62]}...   OK")

    # 1b. 2026-11-11: no operator Resume needed. It retries by itself after 60 s; still dead -> stops at once (no
    #     refeeds, no second shake) and waits longer; the servo works again -> it carries on.
    p = c.wait_paused(59)
    assert p.action is None and opt.mode == "ALERT", "retried too early"
    p = c.wait_paused(2)
    assert p.action == "resume" and opt.mode == "RUN" and "retry 1" in opt.reason, (p.action, opt.reason)
    p = c.feed(0)
    assert p.action == "stop_feed" and opt.mode == "ALERT" and "retry 1" in opt.reason and "120 s" in opt.reason, \
        (p.action, opt.reason)
    c.end(O.NO_BLOCK)
    assert c.wait_paused(119).action is None
    assert c.wait_paused(2).action == "resume"
    p = c.feed(80)
    assert p.action is None and opt._feed_stop is None and "pushes again" in opt.reason, (p.action, opt.reason)
    c.end("OK")
    assert opt.mode == "RUN"
    print("self-retry: dead -> alarm once, retry by itself after 60 s / 120 s, a real push -> carries on   OK")

    # 1c. Someone at the feeder (its IR changes): retry 5 s later, not after the full wait.
    opt, c = armed()
    c.fir = True
    for dip in (0, 0, 0):
        p = c.feed(dip)
        c.end(O.NO_BLOCK)
    assert opt.mode == "ALERT" and opt._feed_stop["kind"] == "dead", (opt.mode, opt.reason)
    c.wait_paused(3)
    c.fir = False                                                   # the stack lifted out
    p = c.wait_paused(10)
    assert p.action == "resume" and "feeder IR changed" in opt.reason, (p.action, opt.reason)
    print("touched:    the feeder IR changes while it waits -> retries 5 s later   OK")

    # 2. A dead push, then the servo works again: the count starts over (it is intermittent).
    opt, c = armed()
    assert c.feed(0).action == "refeed"
    c.end(O.NO_BLOCK)
    assert c.feed(95).action is None and opt._dead_row == 0
    c.end("OK")
    assert c.feed(0).action == "refeed" and "(1/2)" in opt.reason
    print("recovers:   dead, then a real push -> the cube is picked, the count starts over   OK")

    # 3. Not armed (the dip was never seen on this machine, e.g. a simulator): a missing dip means nothing.
    opt = O.Optimize()
    c = Cell(opt)
    c.tick("IDLE", V0)
    assert c.feed(0).action is None, "acted although not armed"
    for _ in range(O.FEED_ARM_PUSHES):
        c.end("OK")
        assert c.feed(70).action is None
    c.end("OK")
    assert c.feed(0).action == "refeed", "5 real pushes seen: it must be armed now"
    print(f"arming:     unarmed -> ignored; after {O.FEED_ARM_PUSHES} real pushes a missing dip counts   OK")

    # 4. The first cycle of a run (arm still driving home) and an unclear dip (20 mV): no verdict.
    opt, c = armed()
    c.n = 0
    assert c.feed(0).action is None, "judged the first cycle of a run"
    c.end("OK")
    assert c.feed(20).action is None and opt.feed_stats["dead"] == 0
    print("careful:    first cycle of a run and an unclear 20 mV dip -> no verdict   OK")

    # 5. Kept across restarts.
    path = os.path.join(tempfile.mkdtemp(prefix="prism_feedcheck_"), "learned.json")
    opt = O.Optimize(learned_path=path)
    c = Cell(opt)
    c.tick("IDLE", V0)
    for _ in range(6):
        c.feed(60)
        c.end("OK")
    assert O.Optimize(learned_path=path).feed_moved >= O.FEED_ARM_PUSHES
    print("memory:     the pushes seen are kept in learned_state.json: armed from the first run after   OK")

    # 6. 2026-11-11: armed by the old wiring (56 pushes ever), but in THIS run every push dips only 1-5 mV and still
    #    delivers its cube. It must never stop the belt, refeed or pause: it notes it, and the cube proves it wrong.
    opt = O.Optimize()
    opt.feed_moved = 56
    c = Cell(opt)
    c.tick("IDLE", V0)
    c.feed(60)                                                      # cycle 1 (not judged)
    c.end("OK")
    for dip in (1, 5, 1, 2, 4, 1):
        p = c.feed(dip)
        assert p.action is None and opt.mode == "RUN", (dip, p.action, opt.mode, opt.reason)
        c.end("OK")
    assert opt.feed_stats["wrong"] >= 1 and "pushed after all" in opt.reason, (opt.feed_stats, opt.reason)
    print(f"11-11 case: no dip, cubes arrive -> never refeeds or pauses; '{opt.reason[:70]}...'   OK")

    # 7. ... and once clear pushes are seen again in the run, a missing dip counts again (the servo can still die).
    for _ in range(O.FEED_ARM_RUN):
        assert c.feed(70).action is None
        c.end("OK")
    assert c.feed(0).action == "refeed", "re-proven in this run: it must act again"
    print(f"re-proven:  {O.FEED_ARM_RUN} clear pushes in the run -> a dead push is acted on again   OK")

    # 8. The user's rule: feeder IR says EMPTY and the push drew no current -> nothing was fed: pause quietly (no
    #    refeeds, no alarm); cubes loaded (the IR sees them 5 s) -> carries on by itself, no Resume.
    opt, c = armed()
    c.fir = False
    c.feed(80)
    c.end("OK")
    p = c.feed(1)
    assert p.action == "stop_feed" and opt.mode == "IDLE" and "feeder empty" in opt.reason, (p.action, opt.mode, opt.reason)
    c.end(O.NO_BLOCK)
    assert c.wait_paused(30).action is None and opt.mode == "IDLE", "resumed without cubes"
    c.fir = True                                                    # refilled
    p = c.wait_paused(10)
    assert p.action == "resume" and "cubes loaded" in opt.reason, (p.action, opt.reason)
    assert c.feed(70).action is None
    c.end("OK")
    assert opt.mode == "RUN" and opt._feed_stop is None, (opt.mode, opt.reason)
    print("empty:      IR empty + no current -> pause quietly; cubes loaded -> carries on by itself   OK")

    print("\nall feed check tests passed")


if __name__ == "__main__":
    main()
