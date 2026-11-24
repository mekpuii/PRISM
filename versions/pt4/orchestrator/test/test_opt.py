# test_opt.py - tests for optimize.py. No robot, no broker, no autoen needed.
# Run (orch-env, from orchestrator/):  python3 test/test_opt.py     Expected last line: "all optimize tests passed"
#
# HOW THE TESTS WORK
#   A fake AutoenView (what actuate.py's poller hands over) and a WorldState / PredictOutput built by hand
#   are stepped through Optimize at times we choose, so cycles, pauses and the camera can be scripted.
import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from actuate import AutoenView                                     # noqa: E402
from observe import ArmStatus, BeltObs, Event, WorldState          # noqa: E402
from predict import CubeEstimate, PredictOutput                    # noqa: E402
import optimize as O                                               # noqa: E402

O.REFEEDS = 0      # these tests check the single-step empty-feeder logic; test_smart.py covers feeding again

RUNNING = {"rpm": 20.0, "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25.0, "pause_before_set": 0}


def state(t, camera=False, events=()):
    return WorldState(t=t, connected=True, joints={}, joints_fresh=False, arm=ArmStatus(), arm_fresh=False,
                      gripper=None, belt=BeltObs(), belt_fresh=False, cubes=[], camera_fresh=camera, cam_seq=0,
                      events=list(events))


def pred(t, moving=False, vel=-56.0):
    cubes = [CubeEstimate(id=1, color="white", t=t, pos_mm=100.0, vel_mm_s=vel, sigma_mm=2.0, coasting=False,
                          age_s=0.0)] if moving else []
    return PredictOutput(t=t, cubes=cubes, belt_mm_s=56.0, belt_ok=True, gaps=[])


def view(t, status="running", paused=False, phase="GRIP", cycles=(), config=None, rest="awake"):
    return AutoenView(ok=True, t=t, status=status, paused=paused, rest_state=rest, phase=phase,
                      config=dict(config or RUNNING), new_cycles=list(cycles))


def cyc(n, result="OK"):
    return {"n": n, "result": result, "secs": 19.0, "notes": ""}


def seen_cycles(opt, t, n0, count):
    """Good cycles where the camera saw the moving cube during TRANSPORT (builds camera trust)."""
    for i in range(count):
        t += 1
        opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
        t += 3
        opt.step(state(t, camera=True), pred(t, moving=True), view(t, phase="TRANSPORT"))
        t += 15
        opt.step(state(t, camera=True), pred(t), view(t, phase="GRIP", cycles=[cyc(n0 + i)]))
    return t


def run_ok_cycles(opt, t, n0, count, config=None):
    for i in range(count):
        t += 19.0
        p = opt.step(state(t), pred(t), view(t, cycles=[cyc(n0 + i)], config=config))
    return t, p


def main():
    # 1. Choice from the real model.json + baselines: throughput -> 20/1.5, energy -> 15/1.5, 20/2.0 below the floor.
    opt = O.Optimize()
    assert opt.target == (20.0, 1.5), opt.target
    assert (20.0, 2.0) not in [O._key(c["rpm"], c["arm"]) for c in opt.eligible()], "20/2.0 should be below p_low 0.8"
    e = O.Optimize(objective="energy")
    assert e.target == (15.0, 1.5), e.target
    print(f"choice:       throughput -> {opt.target}, energy -> {e.target}, 20/2.0 excluded   OK")

    # 2. No model -> the safe fixed point, with a warning.
    m = O.Optimize(model_path="/nonexistent/model.json")
    assert m.target == (20.0, 1.5) and m.warnings, (m.target, m.warnings)
    print("no model:     falls back to 20/1.5 with a warning   OK")

    # 3. OFF when autoen is missing, idle, or experiment.py drives it.
    opt = O.Optimize()
    assert opt.step(state(0), pred(0), None).mode == "OFF"
    assert opt.step(state(0), pred(0), view(0, status="ready")).mode == "OFF"
    p = opt.step(state(0), pred(0), view(0, config={**RUNNING, "pause_before_set": 1}))
    assert p.mode == "OFF" and "experiment" in p.reason, p
    print("off:          no autoen / ready / experiment.py in control -> OFF   OK")

    # 4. RUN from autoen's default 6.7 rpm: settings target 20/1.5, sent AT ONCE (2026-11-11: autoen takes a setting at
    #    its next move / belt run, nothing waits for the cycle end). A cycle that ran on two settings counts for neither.
    opt = O.Optimize()
    slow = {**RUNNING, "rpm": 6.7, "arm_speed": 1.0, "carry_speed": 1.0}
    p = opt.step(state(1), pred(1), view(1, config=slow))
    assert p.mode == "RUN" and (p.rpm, p.arm_speed) == (20.0, 1.5) and p.at_boundary, p
    p = opt.step(state(5), pred(5), view(5, cycles=[cyc(1)], config=slow))
    assert p.at_boundary and p.arrive_timeout == 25.0, p
    fast = {**RUNNING, "rpm": 20.0, "arm_speed": 1.5, "carry_speed": 1.5}
    p = opt.step(state(9), pred(9), view(9, config=fast))       # taken in the middle of cycle 2: autoen marks it
    n0 = opt.configs[(20.0, 1.5)]["n"]
    p = opt.step(state(20), pred(20), view(20, cycles=[{**cyc(2), "mixed": True}], config=fast))
    assert opt.configs[(20.0, 1.5)]["n"] == n0, "the mixed cycle must not count for 20/1.5"
    p = opt.step(state(40), pred(40), view(40, cycles=[cyc(3)], config=fast))
    assert opt.configs[(20.0, 1.5)]["n"] == n0 + 1, "the next clean cycle counts"
    print("run:          target 20/1.5, sent at once (mid-cycle); the mixed cycle counts for neither setting   OK")

    # 5. Online learning: transport timeouts with a cube on the camera demote 20/1.5 -> next best config.
    opt = O.Optimize(objective="throughput")
    t = 0.0
    opt.step(state(t), pred(t), view(t))
    t, _ = run_ok_cycles(opt, t, 1, 3)
    n = 4
    fails = 0
    while opt.target == (20.0, 1.5) and fails < 12:       # cube seen during TRANSPORT, then a real timeout
        t += 1
        opt.step(state(t, camera=True), pred(t, moving=True), view(t, phase="TRANSPORT"))
        t += 24
        p = opt.step(state(t, camera=True), pred(t, moving=True), view(t, cycles=[cyc(n, "TIMEOUT_TRANSPORT")]))
        n += 1
        fails += 1
        t, p = run_ok_cycles(opt, t, n, 1)
        n += 1
    assert opt.target != (20.0, 1.5) and p.mode == "RUN", (opt.target, p)
    assert 3 < fails < 12, f"32/32 offline evidence should survive 3 failures but not 12 ({fails})"
    print(f"learning:     {fails} real transport timeouts at 20/1.5 -> switched to {opt.target}   OK")

    # 6. Empty feeder, camera TRUSTED (saw the last 5 cubes, moving toward smaller u = negative velocity):
    #    no moving cube 5 s into TRANSPORT -> pause, then IDLE after that one cycle.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    t = seen_cycles(opt, 0.0, 1, 5)
    assert opt.camera_trusted(), list(opt._cam_record)
    t += 1
    p = opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    assert p.action is None
    t += 5.5
    p = opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    assert p.action == "abort" and p.mode == "RUN", p
    t += 1
    p = opt.step(state(t, camera=True), pred(t), view(t, paused=True, phase="TRANSPORT"))
    assert p.mode == "RUN" and p.action is None, f"our own pause must not look like MANUAL: {p}"
    t += 18
    p = opt.step(state(t, camera=True), pred(t), view(t, paused=True, status="paused", phase="IDLE",
                                                       cycles=[cyc(6, "TIMEOUT_TRANSPORT")]))
    assert p.mode == "IDLE" and p.action == "idle", p
    assert opt.configs[(20.0, 1.5)]["n"] == 5, "an empty feed must not count against the settings"
    print("empty (cam):  trusted camera, no cube 5 s into TRANSPORT -> abort -> IDLE after one empty cycle   OK")

    # 6a. autoen took the abort: the cycle ends NO_BLOCK -> IDLE, never counted against the settings.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    t = seen_cycles(opt, 0.0, 1, 5) + 1
    opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    t += 5.5
    assert opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT")).action == "abort"
    t += 0.5
    p = opt.step(state(t, camera=True), pred(t), view(t, paused=True, status="paused", cycles=[cyc(6, "NO_BLOCK")]))
    assert p.mode == "IDLE" and p.action == "idle", p
    assert opt.configs[(20.0, 1.5)]["n"] == 5, opt.configs[(20.0, 1.5)]
    print("abort:        NO_BLOCK result -> IDLE at once, settings not blamed   OK")

    # 6b. Camera NOT trusted (like vision_check_v1: it missed 20 of 24 cubes): no early pause.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    t = 0.0
    for i in range(5):                                    # fresh camera, but it never sees the cube
        t += 1
        opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
        t += 18
        opt.step(state(t, camera=True), pred(t), view(t, cycles=[cyc(i + 1)]))
    assert not opt.camera_trusted()
    t += 1
    opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    t += 6
    p = opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    assert p.action is None and p.mode == "RUN", f"an untrusted camera must not pause: {p}"
    print("cam untrusted: camera that misses cubes never triggers the early pause   OK")

    # 7. ... and the trusted camera missed a cube that did arrive: resume right away.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    t = seen_cycles(opt, 0.0, 1, 5) + 1
    opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    t += 6
    assert opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT")).action == "abort"
    t += 5
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused", cycles=[cyc(6)]))
    assert p.action == "resume" and p.mode == "RUN", p
    t += 1
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"))
    assert p.mode == "RUN", f"must not go MANUAL while our resume lands: {p}"
    print("camera miss:  cube arrived after all -> resume, stays RUN   OK")

    # 8. Empty feeder, camera OFF: IDLE after 2 transport timeouts in a row (autoen would stop at 3).
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    p = opt.step(state(25), pred(25), view(25, cycles=[cyc(1, "TIMEOUT_TRANSPORT")]))
    assert p.mode == "RUN" and p.action is None, p
    p = opt.step(state(50), pred(50), view(50, cycles=[cyc(2, "TIMEOUT_TRANSPORT")]))
    assert p.mode == "IDLE" and p.action == "idle", p
    assert opt.configs[(20.0, 1.5)]["n"] == 0 and opt.target == (20.0, 1.5), "empty feeds must not demote the settings"
    blind = O.Optimize()
    blind.step(state(0), pred(0), view(0))
    blind.step(state(25), pred(25), view(25, cycles=[cyc(1, "TIMEOUT_TRANSPORT")]))
    blind.step(state(45), pred(45), view(45, cycles=[cyc(2)]))
    c = blind.configs[(20.0, 1.5)]
    assert (c["n"], c["ok"]) == (2, 1), f"timeout followed by a normal cycle is a real failure: {c}"
    print("empty (blind): 2 timeouts in a row -> IDLE, settings not blamed; 1 timeout + OK counted   OK")

    # 9. In IDLE: stays paused until the operator resumes; "park" and "wake" are operator-only.
    t = 60.0
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"))
    assert p.mode == "IDLE" and p.action is None
    t += 600
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"))
    assert p.mode == "IDLE" and p.action is None, "no auto-resume without --probe-s"
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"), ["park"])
    assert p.action == "park", p
    t += 40
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused", rest="resting"))
    assert p.mode == "PARK", p
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused", rest="resting"), ["resume"])
    assert p.mode == "PARK" and p.action is None, "resume must not move a resting arm"
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused", rest="resting"), ["wake"])
    assert p.action == "wake", p
    t += 40
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"))
    assert p.mode == "IDLE" and p.action is None, p
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused"), ["resume"])
    assert p.mode == "RUN" and p.action == "resume", p
    print("idle/park:    waits for the operator; park/wake/resume only on command   OK")

    # 10. --probe-s: blind (camera not trusted) it never probes: an empty probe would count toward autoen's
    #     max_fails. With a trusted camera IDLE tries one feed after the probe time.
    opt = O.Optimize(probe_s=120)
    opt.step(state(0), pred(0), view(0))
    opt.step(state(25), pred(25), view(25, cycles=[cyc(1, "TIMEOUT_TRANSPORT")]))
    opt.step(state(50), pred(50), view(50, cycles=[cyc(2, "TIMEOUT_TRANSPORT")]))
    p = opt.step(state(1000), pred(1000), view(1000, paused=True, status="paused"))
    assert p.mode == "IDLE" and p.action is None, f"blind: no probe: {p}"
    opt = O.Optimize(probe_s=120)
    opt.step(state(0), pred(0), view(0))
    t = seen_cycles(opt, 0.0, 1, 5) + 1
    opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    t += 6
    opt.step(state(t, camera=True), pred(t), view(t, phase="TRANSPORT"))
    t += 1
    p = opt.step(state(t), pred(t), view(t, paused=True, status="paused", cycles=[cyc(6, "NO_BLOCK")]))
    assert p.mode == "IDLE", p
    assert opt.step(state(t + 60), pred(t + 60), view(t + 60, paused=True, status="paused")).action is None
    p = opt.step(state(t + 121), pred(t + 121), view(t + 121, paused=True, status="paused"))
    assert p.action == "resume" and p.mode == "RUN", p
    print("probe:        blind -> never probes; trusted camera -> IDLE resumes after --probe-s   OK")

    # 11. Grip miss -> ALERT + pause; the operator pausing autoen -> MANUAL, hands off.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    p = opt.step(state(20), pred(20), view(20, cycles=[cyc(1, "GRIP_FAILED")]))
    assert p.mode == "ALERT" and p.action == "pause", p
    p = opt.step(state(60), pred(60), view(60, paused=True, status="paused"))
    assert p.mode == "ALERT" and p.action is None
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    p = opt.step(state(5), pred(5), view(5, paused=True))
    assert p.mode == "MANUAL" and p.action is None, p
    p = opt.step(state(9), pred(9), view(9))
    assert p.mode == "RUN" and p.action is None, p
    print("alert/manual: grip miss -> ALERT; operator pause -> MANUAL, never overridden   OK")

    # 12. Duplicate cycle reports (MQTT + HTTP) are counted once.
    opt = O.Optimize()
    opt.step(state(0), pred(0), view(0))
    ev = Event(source="cycle", name="OK", args=[19.0, 1, ""], t=19.0)
    opt.step(state(19, events=[ev]), pred(19), view(19, cycles=[cyc(1)]))
    assert opt.configs[(20.0, 1.5)]["n"] == 1, opt.configs[(20.0, 1.5)]
    print("dedupe:       one cycle from MQTT + HTTP counted once   OK")

    # 13. arrive_timeout per rpm stays inside autoen's limits.
    for rpm in (8.0, 11.1, 15.0, 20.0):
        at = O.arrive_timeout(rpm)
        assert 20 <= at <= 120, (rpm, at)
    print(f"timeouts:     8 rpm {O.arrive_timeout(8):g} s, 15 rpm {O.arrive_timeout(15):g} s, "
          f"20 rpm {O.arrive_timeout(20):g} s   OK")

    # 14. Memory: counts learned live are saved and loaded by the next run; camera trust too, for CAM_KEEP_S
    #     (2026-11-10: a run that starts blind cannot see an empty feed and never probes by itself).
    import tempfile
    path = os.path.join(tempfile.mkdtemp(prefix="prism_learned_"), "learned_state.json")
    opt = O.Optimize(learned_path=path)
    opt.step(state(0), pred(0), view(0))
    t = seen_cycles(opt, 0.0, 1, 5)
    opt.step(state(t + 20), pred(t + 20), view(t + 20, cycles=[cyc(6, "GRIP_FAILED")]))
    again = O.Optimize(learned_path=path)
    c = again.configs[(20.0, 1.5)]
    assert (c["n"], c["ok"]) == (6, 5) and again.camera_trusted(), (c, again.camera_trusted())
    assert O.Optimize().configs[(20.0, 1.5)]["n"] == 0, "no learned_path = nothing loaded"
    import json
    d = json.load(open(path))
    d["camera"]["saved"] -= O.CAM_KEEP_S + 60                      # yesterday's trust: earned again
    json.dump(d, open(path, "w"))
    assert not O.Optimize(learned_path=path).camera_trusted(), "camera trust older than CAM_KEEP_S was kept"
    print("memory:       6 live cycles (5 OK) saved and reloaded; camera trust kept for 12 h, then re-earned   OK")

    print("\nall optimize tests passed")


if __name__ == "__main__":
    main()
