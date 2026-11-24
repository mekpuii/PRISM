# test_feeder_ir.py - the feeder IR on D36: idle the moment the feeder is empty, resume the moment it is refilled,
# ignore flicker, stop trusting it after one wrong call, and behave as before with the old firmware (no reading).
# No robot, no broker, no autoen.
# Run (orch-env, from orchestrator/):  python3 test/test_feeder_ir.py     Expected last line: "all feeder IR tests passed"
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimize as O                                               # noqa: E402
from test_opt import cyc, pred, view                               # noqa: E402
from test_smart import settings, tele                              # noqa: E402


def st(t, fir):
    s = tele(t)
    s.belt.feed_ir = fir
    return s


def ticks(opt, t, secs, fir, **v):
    """Step every 0.25 s for `secs` with the feeder IR reading `fir`; returns (t, plans)."""
    plans = []
    for _ in range(int(secs / 0.25)):
        t += 0.25
        plans.append(opt.step(st(t, fir), pred(t), view(t, config=settings(opt), **v)))
    return t, plans


def main():
    # 1. RUN, the last cube is pushed: a PROVEN feeder IR reads "empty" for 2 s -> IDLE at once (no empty feed).
    #    The cube already on the belt finishes OK while going idle: still IDLE, and it is not taken for a refill.
    opt = O.Optimize(probe_s=120)
    opt._fir_right = O.FEED_IR_PROVEN
    t, _ = ticks(opt, 0.0, 3, True)
    assert opt.mode == "RUN", opt.mode
    t, plans = ticks(opt, t, 1.5, False)
    assert opt.mode == "RUN", "acted on a reading younger than FEED_IR_STABLE_S"
    t, plans = ticks(opt, t, 1.0, False)
    assert opt.mode == "IDLE" and any(p.action == "idle" for p in plans), (opt.mode, opt.reason)
    t += 1
    opt.step(st(t, False), pred(t), view(t, paused=True, cycles=[cyc(1)], config=settings(opt)))
    assert opt.mode == "IDLE" and "feeder IR" in opt.reason, (opt.mode, opt.reason)
    print(f"empty:     feeder IR 'empty' 2 s -> {opt.mode} ({opt.reason}); the last cube finished, still IDLE   OK")

    # 2. IDLE, the operator refills: 'cubes' for 2 s -> resume at once (no waiting for a probe). The cube arrives:
    #    one right call.
    t, plans = ticks(opt, t, 1.0, True, status="paused", paused=True, phase="IDLE")
    assert opt.mode == "IDLE", "resumed on a reading younger than FEED_IR_STABLE_S"
    t, plans = ticks(opt, t, 1.5, True, status="paused", paused=True, phase="IDLE")
    assert opt.mode == "RUN" and any(p.action == "resume" for p in plans), (opt.mode, opt.reason)
    t, _ = ticks(opt, t, 2, True)
    opt.step(st(t, True), pred(t), view(t, cycles=[cyc(2)], config=settings(opt)))
    assert opt.mode == "RUN" and opt._fir_right == O.FEED_IR_PROVEN + 1 and opt.feeder is True, (opt.mode, opt._fir_right)
    print(f"refill:    feeder IR 'cubes' 2 s while idle -> resume, the cube came: right calls {opt._fir_right}   OK")

    # 3. Flicker (a blade pass, a hand): readings that do not hold 2 s change nothing.
    for i in range(20):
        t, _ = ticks(opt, t, 0.5, i % 2 == 0)
    assert opt.mode == "RUN", opt.mode
    print("flicker:   0.5 s on / 0.5 s off for 10 s -> ignored, still RUN   OK")

    # 4. A wrong call: it reads 'cubes' but the feed is empty -> it is ignored for the rest of the run (warning).
    opt = O.Optimize(probe_s=120)
    opt._fir_right = O.FEED_IR_PROVEN                               # proven, so it idles on its word first
    t, _ = ticks(opt, 0.0, 3, False)                                 # idle on its word
    t += 1
    opt.step(st(t, False), pred(t), view(t, paused=True, cycles=[cyc(1)], config=settings(opt)))
    t, _ = ticks(opt, t, 3, True, status="paused", paused=True, phase="IDLE")
    assert opt.mode == "RUN"
    t += 1
    opt.step(st(t, True), pred(t), view(t, cycles=[cyc(2, O.NO_BLOCK)], config=settings(opt)))
    assert opt._fir_wrong and opt.warnings, (opt._fir_wrong, opt.warnings)
    t, _ = ticks(opt, t, 5, True, status="paused", paused=True, phase="IDLE")
    assert opt.mode == "IDLE" and opt.feeder is None, "resumed on a feeder IR that was already caught wrong"
    print(f"wrong:     'cubes' but the feed was empty -> ignored this run ({opt.warnings[-1]})   OK")

    # 5. Old firmware: no feeder IR reading at all -> exactly the old behaviour (no idle without a cycle result).
    opt = O.Optimize(probe_s=120)
    t, _ = ticks(opt, 0.0, 10, None)
    assert opt.mode == "RUN" and opt.feeder is None, opt.mode
    print("old fw:    no reading -> RUN as before   OK")

    # 6. Proven (3 right calls): while it reads 'empty', no probes; unproven, probes still run as a backup.
    opt = O.Optimize(probe_s=120)
    opt._cam_record.extend([True] * O.CAM_WINDOW)                   # trusted camera (probes need it)
    t, _ = ticks(opt, 0.0, 3, False)
    assert opt._probe_delay(t) == 120, "unproven sensor must not switch the probes off"
    opt._fir_right = O.FEED_IR_PROVEN
    assert opt._probe_delay(t) is None, "a proven sensor reading 'empty' should replace the probes"
    print(f"proven:    after {O.FEED_IR_PROVEN} right calls its 'empty' replaces the probes; before that they run   OK")

    # 7. NOT proven yet (2026-11-10: it read "empty" and stopped the cell after one cube): "empty" is only a claim.
    #    The cube on the belt finishes, the NEXT feed decides. Empty feed -> a right call; a cube -> wrong, ignored.
    opt = O.Optimize(probe_s=120)
    t, _ = ticks(opt, 0.0, 3, False)
    assert opt.mode == "RUN" and "next feed checks it" in opt.reason, (opt.mode, opt.reason)
    t += 1
    opt.step(st(t, False), pred(t), view(t, cycles=[cyc(1)], config=settings(opt)))     # the cube already on the belt
    assert opt.mode == "RUN" and opt._fir_right == 0 and opt._fir_claim == "judge", (opt.mode, opt._fir_claim)
    t, _ = ticks(opt, t, 1, False)
    opt.step(st(t, False), pred(t), view(t, cycles=[cyc(2, "TIMEOUT_TRANSPORT")], config=settings(opt)))
    t, _ = ticks(opt, t, 1, False)
    opt.step(st(t, False), pred(t), view(t, cycles=[cyc(3, "TIMEOUT_TRANSPORT")], config=settings(opt)))
    assert opt.mode == "IDLE" and opt._fir_right == 0, (opt.mode, opt._fir_right)   # blind: 2 timeouts -> IDLE
    print(f"claim:     unproven 'empty' -> kept running, the next feed decided -> {opt.mode}   OK")
    opt = O.Optimize(probe_s=120)
    t, _ = ticks(opt, 0.0, 3, False)
    t += 1
    opt.step(st(t, False), pred(t), view(t, cycles=[cyc(1)], config=settings(opt)))
    t, _ = ticks(opt, t, 1, False)
    opt.step(st(t, False), pred(t), view(t, cycles=[cyc(2)], config=settings(opt)))     # the next feed brought one
    t, _ = ticks(opt, t, 5, False)
    assert opt.mode == "RUN" and opt._fir_wrong and opt.feeder is None, (opt.mode, opt._fir_wrong)
    print(f"claim:     unproven 'empty' but the next feed brought a cube -> ignored this run ({opt.warnings[-1]})   OK")

    print("\nall feeder IR tests passed")


if __name__ == "__main__":
    main()
