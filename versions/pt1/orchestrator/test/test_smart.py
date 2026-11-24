# test_smart.py - the optimizer's priority choice (1 safety, 2 energy, 3 speed), live energy learning, exploration,
# the safety governor and feeding again. No robot, no broker, no autoen.
# Run (orch-env, from orchestrator/):  python3 test/test_smart.py     Expected last line: "all smart tests passed"
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimize as O                                               # noqa: E402
from observe import ArmStatus, BeltObs, JointObs, WorldState       # noqa: E402
from test_opt import cyc, pred, view                               # noqa: E402

O.REFEEDS = 1      # test_opt switches feeding-again off for its own tests; here it is under test

V = 12.0


def tele(t, watts=1.0, servo_c=35.0, belt_c=25.0, camera=False):
    """A WorldState with fresh belt + arm telemetry: `watts` split half belt, half arm."""
    per_joint_ma = (watts / 2) / V * 1000 / 6
    joints = {i: JointObs(pos=2000, speed=0, cur_ma=per_joint_ma, temp_c=servo_c, volt_v=V) for i in range(1, 7)}
    belt = BeltObs(temp_c=belt_c, current_ma=(watts / 2) / V * 1000, voltage_v=V, rpm=0.0, fault=0, live_t=t)
    return WorldState(t=t, connected=True, joints=joints, joints_fresh=True, arm=ArmStatus(), arm_fresh=True,
                      gripper=None, belt=belt, belt_fresh=True, cubes=[], camera_fresh=camera, cam_seq=0, events=[])


def run_cycles(opt, t, n0, count, secs=19.0, watts=1.15, cfg=None, **kw):
    """`count` OK cycles of `secs` each at a steady `watts` (1.15 W x 19 s ~ 21.9 J, the measured 20/1.5)."""
    p = None
    for i in range(count):
        for _ in range(int(secs / 0.25)):
            t += 0.25
            opt.step(tele(t, watts, **kw), pred(t), view(t, config=cfg or settings(opt)))
        p = opt.step(tele(t, watts, **kw), pred(t), view(t, cycles=[dict(cyc(n0 + i), secs=secs)], config=cfg or settings(opt)))
    return t, p


def settings(opt):
    """autoen follows the optimizer's target (as actuate.py makes it)."""
    return {"rpm": opt.target[0], "arm_speed": opt.target[1], "carry_speed": opt.target[1], "arrive_timeout": 25.0,
            "pause_before_set": 0}


def main():
    # 1. Priority: 15/1.5 has the lowest J, but 20/1.5 is within 5 % and faster -> 20/1.5.
    opt = O.Optimize()
    j = {k: round(opt.per_good(c)[1], 2) for k, c in opt.configs.items() if k in ((15.0, 1.5), (20.0, 1.5))}
    assert opt.target == (20.0, 1.5), opt.target
    print(f"priority:   J/good cube {j} -> 20/1.5 (inside the 5 % energy band, and fastest)   OK")

    # 2a. Live energy, the whole machine costs 40 % more today (cold, worn, the meter counts the gaps):
    #     the grid is scaled by the same factor, so nothing changes -> still 20/1.5.
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t, p = run_cycles(opt, 0.0, 1, 25, watts=1.4 * 20.8 / 19.0)
    assert opt.target == (20.0, 1.5) and 1.3 < opt.machine_scale() < 1.5, (opt.target, opt.machine_scale())
    print(f"live J:     everything +{100 * (opt.machine_scale() - 1):.0f} % today -> grid scaled, still 20/1.5   OK")

    # 2b. Only 20/1.5 got expensive (30 J; the others still cost what the grid says): exploration measures another
    #     setting, the comparison shows 20/1.5 is the odd one out -> it moves to a cheaper setting and stays there.
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t, n = 0.0, 1
    for _ in range(O.EXPLORE_EVERY + 30):
        jc = 30.0 if opt.target == (20.0, 1.5) else opt.configs[opt.target]["J"]
        t, p = run_cycles(opt, t, n, 1, watts=jc / 19.0)
        n += 1
    c20 = opt.configs[(20.0, 1.5)]
    assert opt.target != (20.0, 1.5), (opt.target, opt.j_est(c20), opt.machine_scale())
    print(f"live J:     20/1.5 alone costs 30 J -> found by comparison, now {opt.target[0]:g}/{opt.target[1]:g} "
          f"(20/1.5 at {opt.j_est(c20):.1f} J)   OK")

    # 3. Exploration: after EXPLORE_EVERY good cycles it tries another low-energy setting for a few cycles, then
    #    returns to the best one. Trials only use safe settings inside the energy band.
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t, _ = run_cycles(opt, 0.0, 1, O.EXPLORE_EVERY + 1)
    tried = opt.target
    assert tried != (20.0, 1.5) and opt._explore is not None and "exploring" in opt.reason, (tried, opt.reason)
    t, _ = run_cycles(opt, t, 100, O.EXPLORE_CYCLES + O.MIN_HOLD_CYCLES + 1)
    assert opt.configs[tried]["j_n"] >= O.EXPLORE_CYCLES - 1, opt.configs[tried]
    assert opt.target == (20.0, 1.5), opt.target
    print(f"explore:    after {O.EXPLORE_EVERY} cycles tried {tried[0]:g}/{tried[1]:g} for {O.EXPLORE_CYCLES} cycles, "
          "measured it, back to 20/1.5   OK")

    # 4. Safety governor: warm -> gentler settings; hot -> COOL (pause at home); cool -> runs again.
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t, _ = run_cycles(opt, 0.0, 1, 4)
    t, p = run_cycles(opt, t, 10, 1, servo_c=57.0)
    assert opt.heat == "warm" and opt.target[0] <= 15.0, (opt.heat, opt.target)
    print(f"warm:       servo 57 C -> gentler {opt.target[0]:g}/{opt.target[1]:g}   OK")
    t += 1
    p = opt.step(tele(t, servo_c=66.0), pred(t), view(t, config=settings(opt)))
    assert p.mode == "COOL" and p.action == "idle", p
    t += 60
    p = opt.step(tele(t, servo_c=58.0), pred(t), view(t, paused=True, status="paused", config=settings(opt)))
    assert p.mode == "COOL" and p.action is None, "still warm: stays paused"
    t += 300
    p = opt.step(tele(t, servo_c=45.0), pred(t), view(t, paused=True, status="paused", config=settings(opt)))
    assert p.mode == "RUN" and p.action == "resume" and opt.heat == "ok", p
    t += 1
    p = opt.step(tele(t, servo_c=45.0), pred(t), view(t, paused=True, status="paused", config=settings(opt)))
    assert p.mode == "RUN", f"must not go MANUAL while its own resume lands: {p}"
    print("hot:        servo 66 C -> COOL (paused at home); 58 C stays paused; 45 C -> resumes by itself   OK")
    t += 2
    p = opt.step(tele(t, belt_c=58.0), pred(t), view(t, config=settings(opt)))
    assert p.mode == "COOL" and p.action == "idle", p
    print("belt hot:   belt 58 C -> COOL as well   OK")

    # 5. Feeding again: trusted camera, no cube 3 s after FEED -> 'refeed' (no pause); that NO_BLOCK is not "empty";
    #    still nothing on the next feed -> 'abort' -> IDLE. A cube after a refeed resets it.
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t = 0.0
    for i in range(5):                                             # earn camera trust
        t += 1
        opt.step(tele(t, camera=True), pred(t), view(t, phase="TRANSPORT", config=settings(opt)))
        t += 2
        opt.step(tele(t, camera=True), pred(t, moving=True), view(t, phase="TRANSPORT", config=settings(opt)))
        t += 15
        opt.step(tele(t, camera=True), pred(t), view(t, cycles=[cyc(i + 1)], config=settings(opt)))
    assert opt.camera_trusted()
    t += 1
    opt.step(tele(t, camera=True), pred(t), view(t, phase="TRANSPORT", config=settings(opt)))
    t += 3.2
    p = opt.step(tele(t, camera=True), pred(t), view(t, phase="TRANSPORT", config=settings(opt)))
    assert p.action == "refeed" and p.mode == "RUN", p
    t += 0.5
    p = opt.step(tele(t, camera=True), pred(t), view(t, phase="FEED", cycles=[cyc(6, "NO_BLOCK")], config=settings(opt)))
    assert p.mode == "RUN" and p.action is None and "feeding again" in p.reason, p
    t += 1
    opt.step(tele(t, camera=True), pred(t), view(t, phase="TRANSPORT", config=settings(opt)))
    t += 3.2
    p = opt.step(tele(t, camera=True), pred(t), view(t, phase="TRANSPORT", config=settings(opt)))
    assert p.action == "abort", p
    t += 0.5
    p = opt.step(tele(t, camera=True), pred(t), view(t, paused=True, status="paused", cycles=[cyc(7, "NO_BLOCK")],
                                                     config=settings(opt)))
    assert p.mode == "IDLE" and p.action == "idle", p
    assert opt.configs[(20.0, 1.5)]["n"] == 5, "empty feeds never count against the settings"
    print("refeed:     no cube 3 s after FEED -> feed again; still nothing -> stop, IDLE   OK")

    # 6. Sorting with several cubes on the belt at once, real geometry: cubes come out of the feeder at u ~590 one by
    #    one, ride toward the IR at u ~245 (~10 px per camera frame), the belt stops when the front one reaches the IR,
    #    the arm takes it (it vanishes from the camera). The 2nd cube's track breaks up mid-belt (new id), and every
    #    4th frame shows the wrong colour. The list must give O W O -> left, right, left, all confirmed.
    from observe import CubeObs
    opt = O.Optimize()
    opt.step(tele(0), pred(0), view(0, config=settings(opt)))
    t, seq = 0.0, 0
    order = ["orange", "white", "orange"]
    belt = []                                   # [track id, u, colour]  front (nearest the IR) first
    feed_u, ir_u, step_px, gap_px = 590.0, 245.0, 10.0, 120.0
    next_id = [1]

    def snap(phase, moving, cycles=()):
        nonlocal t, seq
        t += 0.17
        seq += 1
        raw = []
        for tid, u, colour in belt:
            col = ("white" if colour == "orange" else "orange") if seq % 4 == 0 else colour
            raw.append(CubeObs(id=tid, color=col, along_px=-u, across_px=240.0, along_vel_px_s=59.0 if moving else 0.0,
                               dist_m=None, w_px=43, h_px=43, hits=6))
        st = tele(t)
        st.cubes, st.camera_fresh, st.cam_seq = raw, True, seq
        return opt.step(st, pred(t, moving=moving), view(t, phase=phase, cycles=list(cycles), config=settings(opt)))

    def feed(colour):
        belt.append([next_id[0], feed_u, colour])
        next_id[0] += 1

    feed(order[0])                              # three cubes queued on the belt before the first pick
    for colour in order[1:]:
        for _ in range(int(gap_px / step_px)):
            for c in belt:
                c[1] -= step_px
            snap("TRANSPORT", True)
        feed(colour)
    bins, broke = [], False
    for n in range(1, 4):
        while belt[0][1] > ir_u:                # ride until the front cube is at the IR
            for c in belt:
                c[1] -= step_px
            if not broke and len(belt) > 1 and belt[1][1] < 420:
                belt[1][0], broke = 99, True    # the camera loses the 2nd cube and finds it again as track 99
            snap("TRANSPORT", True)
        for _ in range(4):
            p = snap("GRIP", False)             # belt stopped at the IR, the arm closes
        bins.append(p.drop)
        belt.pop(0)                             # the arm lifts the front cube away
        for _ in range(4):
            snap("CARRY_HOME", False)
        snap("IDLE", False, cycles=[dict(cyc(n), drop=bins[-1])])
    assert bins == ["left", "down", "left"], (bins, opt.queue.stats)
    assert opt.sort_stats["right_bin"] == 3 and opt.sort_stats["confirmed"] == 3, opt.sort_stats
    assert opt.queue.stats["cubes"] == 3 and opt.queue.stats["relinked"] >= 1, opt.queue.stats
    print(f"sort list:  O W O queued together, a broken track and flickering colours -> bins {bins}; "
          f"list saw {opt.queue.stats['cubes']} cubes, re-linked {opt.queue.stats['relinked']} track; "
          f"{opt.sort_stats['confirmed']}/3 picks confirmed   OK")

    # 6b. A phantom: something appears at the feeder end but never reaches the IR (a shadow). The re-sync at the IR
    #     drops it, so the real cube behind it still gets its own bin.
    q = opt.queue
    q.update([CubeObs(id=500, color="white", along_px=-585.0, across_px=240.0, along_vel_px_s=59.0, dist_m=None,
                      w_px=43, h_px=43, hits=6)])                   # the phantom, then it is never seen again
    belt.clear()
    for _ in range(12):                                             # 2 s with nothing on the camera
        snap("IDLE", False)
    feed("orange")
    while belt[0][1] > ir_u:
        belt[0][1] -= step_px
        snap("TRANSPORT", True)
    for _ in range(4):
        p = snap("GRIP", False)
    assert p.drop == "left" and q.stats["phantoms_dropped"] >= 1, (p.drop, q.stats)
    print(f"phantom:    a shadow listed before the real cube is dropped at the IR -> orange still left   OK")

    print("\nall smart tests passed")


if __name__ == "__main__":
    main()
