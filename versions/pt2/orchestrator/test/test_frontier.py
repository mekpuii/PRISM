# test_frontier.py - efficiency first, live cycle times, and stepping beyond the tested grid (optimize.py frontier):
# a step past the best proven setting after a clean run, proven after 12 of 12, back + banned at the first problem,
# never past the outer wall, never while warm, carry speed capped, everything kept across a restart.
# No robot, no broker, no autoen.
# Run (orch-env, from orchestrator/):  python3 test/test_frontier.py     Expected last line: "all frontier tests passed"
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimize as O                                               # noqa: E402
from test_opt import cyc, pred, view                               # noqa: E402
from test_smart import settings, tele                              # noqa: E402


class Cell:
    """autoen as actuate.py makes it: runs the optimizer's setting; cycle time from a 'true' machine model."""

    def __init__(self, opt, true_t=lambda rpm, arm: 6.0 + 1.5 / arm + 120.0 / rpm, fail_at=None):
        self.opt, self.true_t, self.fail_at, self.t, self.n = opt, true_t, fail_at or set(), 0.0, 0

    def cycles(self, count, servo_c=35.0):
        p = None
        for _ in range(count):
            rpm, arm = self.opt.target
            secs = self.true_t(rpm, arm)
            for _ in range(int(secs / 0.5)):
                self.t += 0.5
                self.opt.step(tele(self.t, 1.15, servo_c=servo_c), pred(self.t),
                              view(self.t, config=settings(self.opt)))
            self.n += 1
            res = "GRIP_MISS" if (rpm, arm) in self.fail_at else "OK"
            p = self.opt.step(tele(self.t, 1.15, servo_c=servo_c), pred(self.t),
                              view(self.t, cycles=[dict(cyc(self.n, res), secs=round(secs, 1))],
                                   config=settings(self.opt)))
            if self.opt.mode == "ALERT":              # a person would resume (no responder in this test)
                self.opt._set_mode("RUN", "test: operator resume", self.t)
        return p


def main():
    tmp = tempfile.mkdtemp(prefix="prism_frontier_")

    # 1. Efficiency: the fastest safe setting; it learns the REAL cycle time (not the grid model's).
    opt = O.Optimize(objective="efficiency")
    cell = Cell(opt)
    cell.cycles(10)
    c = opt.configs[opt.target]
    assert opt.target == (20.0, 1.5) and c["t_n"] == 9, (opt.target, c["t_n"])   # the first cycle started before RUN
    assert abs(opt.t_est(c) - cell.true_t(20.0, 1.5)) < 0.15 * cell.true_t(20.0, 1.5), opt.t_est(c)
    print(f"efficiency: 20/1.5 chosen; time learned live {opt.t_est(c):.1f} s (grid model said {c['t_s']:.1f} s)   OK")

    # 2. Frontier: after 30 clean good cycles, ONE step past 20/1.5 (predicted faster), carry stays <= CARRY_MAX.
    opt = O.Optimize(objective="efficiency", frontier=True)
    cell = Cell(opt)
    p = cell.cycles(O.FRONTIER_EVERY + 1)
    assert opt._trial is not None and "frontier: trying" in opt.reason, opt.reason
    k = opt._trial["key"]
    assert k in ((22.0, 1.5), (20.0, 1.75), (22.0, 1.75)) and opt.target == k, k
    print(f"frontier:  after {O.FRONTIER_EVERY} clean cycles -> trying {k[0]:g}/{k[1]:g} ({opt.reason[:60]}...)   OK")

    # 3. 12 of 12 OK -> proven, eligible, used; then it keeps stepping out, never past the outer wall.
    p = cell.cycles(O.TRIAL_CYCLES)
    assert opt._trial is None and "proven" in opt.reason and opt.p_low(opt.configs[k]) >= O.P_LOW_FLOOR, opt.reason
    p = cell.cycles(400)
    rpm, arm = opt.target
    beyond = p.score["beyond_grid"]
    assert rpm > 20.0 or arm > 1.5, opt.target
    assert all(O.OUTER_RPM[0] <= x["rpm"] <= O.OUTER_RPM[1] and O.OUTER_ARM[0] <= x["arm"] <= O.OUTER_ARM[1]
               for x in opt.configs.values())
    assert p.carry_speed <= O.CARRY_MAX, p.carry_speed
    t0, t1 = cell.true_t(20.0, 1.5), cell.true_t(rpm, arm)
    print(f"growing:   proven beyond the grid {beyond}; now {rpm:g}/{arm:g}: {t1:.1f} s per cycle instead of "
          f"{t0:.1f} s ({(1 - t1 / t0):.0%} faster), carry {p.carry_speed:g}   OK")

    # 4. The first problem ends a trial: back to the proven setting, banned (not tried again soon).
    opt = O.Optimize(objective="efficiency", frontier=True)
    cell = Cell(opt, fail_at={(22.0, 1.5), (20.0, 1.75), (22.0, 1.75)})
    for _ in range(O.FRONTIER_EVERY + 5):
        cell.cycles(1)
        if opt._trial is not None:
            break
    first = opt._trial["key"]
    cell.cycles(2)                                    # the next cycle runs on the trial setting and misses
    assert opt._trial is None and opt.target == (20.0, 1.5) and opt._banned(first), (opt.target, opt.reason)
    cell.cycles(60)
    assert opt.configs[first]["n"] == 1, "a banned point was tried again"
    print(f"failure:   {first[0]:g}/{first[1]:g} missed once -> straight back to 20/1.5, banned   OK")

    # 5. Never while warm; a warm spell ends a running trial without a ban.
    opt = O.Optimize(objective="efficiency", frontier=True)
    cell = Cell(opt)
    cell.cycles(O.FRONTIER_EVERY + 5, servo_c=O.SERVO_WARM_C + 1)
    assert opt._trial is None and not any(c.get("frontier") for c in opt.configs.values()), "explored while warm"
    print("warm:      servo warm -> no step beyond the grid   OK")

    # 6. Kept across a restart: the proven frontier points, their live times, and the bans.
    path = os.path.join(tmp, "learned.json")
    opt = O.Optimize(objective="efficiency", frontier=True, learned_path=path)
    cell = Cell(opt, fail_at={(20.0, 1.75)})
    cell.cycles(200)
    again = O.Optimize(objective="efficiency", frontier=True, learned_path=path)
    pts = {k for k, c in again.configs.items() if c.get("frontier")}
    assert pts and pts == {k for k, c in opt.configs.items() if c.get("frontier") and c["n"]}, (pts,)
    assert all(again.configs[k]["t_n"] == opt.configs[k]["t_n"] for k in pts)
    assert ((20.0, 1.75) not in opt._bans) or again._banned((20.0, 1.75)), "the ban was forgotten"
    print(f"restart:   {len(pts)} frontier points, their live times and the bans survive a restart   OK")

    # 7. --no-frontier / default class: stays in the grid.
    opt = O.Optimize(objective="efficiency")
    cell = Cell(opt)
    cell.cycles(80)
    assert not any(c.get("frontier") for c in opt.configs.values())
    print("off:       frontier off -> never leaves the tested grid   OK")

    print("\nall frontier tests passed")


if __name__ == "__main__":
    main()
