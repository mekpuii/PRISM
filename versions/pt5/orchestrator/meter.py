# meter.py - energy meter for the orchestrator (PRISM): belt + arm joules, split by the optimizer's mode.
#
# Same power formulas as cal/energy_per_cube.py, but live from the WorldState main.py already has, so every run is
# metered without a separate recorder:
#   belt W = belt INA260 current x volts        arm W = sum over the 6 servos of current x volts (no arm INA260 exists)
# Power is integrated only between two fresh samples at most MAX_DT apart: a gap is a gap, never guessed.
# main.py writes take() to the orchestrator log every minute ("kind": "energy"); report.py adds them up per day.

MAX_DT = 0.5            # s: longest step that still counts (= cal/energy_per_cube.MAX_DT)


def belt_w(state):
    b = state.belt
    if not state.belt_fresh or b.current_ma is None or b.voltage_v is None:
        return None
    return abs(b.current_ma) * b.voltage_v / 1000.0


def arm_w(state):
    """A servo that skipped its volts gets the mean of the others (as cal/energy_per_cube.py): ~7 % of the samples,
    mostly while the arm moves, so skipping them would under-count the arm by ~9 %."""
    if not state.joints_fresh:
        return None
    js = list(state.joints.values())
    volts = [j.volt_v for j in js if j.volt_v is not None]
    v_mean = sum(volts) / len(volts) if volts else 12.0
    return sum((j.cur_ma or 0.0) * (j.volt_v if j.volt_v is not None else v_mean) for j in js) / 1000.0


class EnergyMeter:
    def __init__(self):
        self.acc = {}           # mode -> {"belt_J", "arm_J", "s", "metered_s"}
        self._last = None       # (t, mode, belt W, arm W)

    def step(self, state, mode):
        """Call every tick with the mode that was in force since the last call."""
        t, bw, aw = state.t, belt_w(state), arm_w(state)
        if self._last is not None:
            t0, m0, bw0, aw0 = self._last
            dt = t - t0
            a = self.acc.setdefault(m0, {"belt_J": 0.0, "arm_J": 0.0, "s": 0.0, "belt_s": 0.0, "arm_s": 0.0})
            a["s"] += dt
            if 0 < dt <= MAX_DT:                    # belt and arm each count whenever their own data is fresh
                if bw0 is not None:
                    a["belt_J"] += bw0 * dt
                    a["belt_s"] += dt
                if aw0 is not None:
                    a["arm_J"] += aw0 * dt
                    a["arm_s"] += dt
        self._last = (t, mode, bw, aw)

    def take(self):
        """The totals since the last take(), rounded, and start again."""
        out = {m: {k: round(v, 2) for k, v in a.items()} for m, a in self.acc.items()}
        self.acc = {}
        return out
