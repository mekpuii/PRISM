# tuner.py - the optimizer experiments on its own knobs (PRISM orchestrator, 2026-11-12).
#
# The operator: "make the optimizer smart, ready to make new things". optimize.py learns numbers inside fixed rules;
# this lets it TRY other values of its own flow knobs and keep what measurably works better - always inside safe
# ranges, one experiment at a time, never during trouble.
#
# HOW
#   cost of one pick = its seconds (OK) or seconds + FAIL_PENALTY_S (anything else): lower is better.
#   champion  = the value in use for each knob.
#   every time WARM clean picks were recorded on the champions, ONE knob gets ONE challenger value for TRIAL_N picks
#   (the value tried least, not banned). Afterwards:
#     challenger mean cost < baseline mean (the WARM picks before it) x (1 - MARGIN) and no more failures -> it becomes
#     the champion; else back to the champion, and that value is banned for BAN_PICKS picks.
#   The result is told in words (tell()) and kept (state()/load()).
# WHAT IT NEVER DOES
#   leave a knob's safe list; run two experiments at once; count a pick the caller marks unclean (alarm, idle, probe,
#   incident, mixed settings) - the caller simply does not record those.

import statistics

KNOBS = {   # name: (values, start, what it means)
    "in_flight": ((-1, 0, 1), 0, "cubes in flight, relative to the ride-time math"),
    "spacing": ((0.20, 0.33, 0.50), 0.33, "share of the ride the last cube travels before the next push"),
    "late": ((1.25, 1.35, 1.6), 1.35, "x its ride before a cube counts as not coming"),
}
WARM = 10           # clean picks on the champions before (and between) experiments
TRIAL_N = 6         # picks per experiment
MARGIN = 0.03       # a challenger must be this much cheaper per pick
FAIL_PENALTY_S = 20.0
BAN_PICKS = 50      # a losing value is not tried again for this many picks


class Tuner:
    def __init__(self, knobs=None):
        self.knobs = dict(knobs or KNOBS)
        self.champ = {k: v[1] for k, v in self.knobs.items()}
        self.tries = {k: {} for k in self.knobs}            # knob -> value -> times tried
        self.bans = {}                                       # (knob, value) -> pick number until which it is banned
        self.history = []                                    # finished experiments (for the report / dashboard)
        self.picks = 0                                       # clean picks recorded
        self._base = []                                      # costs on the champions since the last experiment
        self._trial = None                                   # {"knob", "value", "costs", "fails", "base", "base_fails"}
        self.last_words = None

    # ----- what the optimizer uses -----
    def current(self, knob):
        t = self._trial
        return t["value"] if t is not None and t["knob"] == knob else self.champ[knob]

    @property
    def trial(self):
        return None if self._trial is None else {"knob": self._trial["knob"], "value": self._trial["value"],
                                                 "left": TRIAL_N - len(self._trial["costs"])}

    # ----- input: one clean pick -----
    def record(self, secs, ok):
        """One clean pick (the caller skips alarms / idle / probes / incidents / mixed settings). Returns words when an
        experiment starts or ends, else None."""
        cost = float(secs) + (0.0 if ok else FAIL_PENALTY_S)
        self.picks += 1
        t = self._trial
        if t is None:
            self._base = (self._base + [(cost, not ok)])[-WARM:]
            if len(self._base) >= WARM:
                return self._start()
            return None
        t["costs"].append(cost)
        t["fails"] += not ok
        if len(t["costs"]) >= TRIAL_N:
            return self._finish()
        return None

    def _start(self):
        cands = []
        for k, (values, _, _) in self.knobs.items():
            for v in values:
                if v == self.champ[k] or self.bans.get((k, v), -1) > self.picks:
                    continue
                cands.append((self.tries[k].get(v, 0), list(self.knobs).index(k), values.index(v), k, v))
        if not cands:
            return None
        _, _, _, k, v = min(cands)
        base = [c for c, _ in self._base]
        self._trial = {"knob": k, "value": v, "costs": [], "fails": 0, "base": statistics.mean(base),
                       "base_fails": sum(f for _, f in self._base)}
        self.tries[k][v] = self.tries[k].get(v, 0) + 1
        self.last_words = (f"experiment: {k} {self.champ[k]} -> {v} ({self.knobs[k][2]}) for {TRIAL_N} picks; "
                           f"to beat: {self._trial['base']:.1f} s per pick")
        return self.last_words

    def _finish(self):
        t, self._trial = self._trial, None
        mean = statistics.mean(t["costs"])
        fail_rate_base = t["base_fails"] / WARM
        better = mean < t["base"] * (1.0 - MARGIN) and t["fails"] <= round(fail_rate_base * TRIAL_N)
        k, v, old = t["knob"], t["value"], self.champ[t["knob"]]
        if better:
            self.champ[k] = v
            words = (f"kept: {k} {old} -> {v}: {mean:.1f} s per pick vs {t['base']:.1f} s "
                     f"({100 * (1 - mean / t['base']):.0f} % better, {t['fails']} failed)")
        else:
            self.bans[(k, v)] = self.picks + BAN_PICKS
            words = (f"back to {k} {old}: {v} gave {mean:.1f} s per pick vs {t['base']:.1f} s"
                     + (f", {t['fails']} failed" if t["fails"] else "") + f" - not tried again for {BAN_PICKS} picks")
        self.history.append({"knob": k, "value": v, "old": old, "mean": round(mean, 2), "base": round(t["base"], 2),
                             "fails": t["fails"], "kept": better, "pick": self.picks})
        self.history = self.history[-30:]
        self._base = []
        self.last_words = words
        return words

    def abort(self, why=""):
        """Trouble in the middle of an experiment: drop it (counts as nothing)."""
        if self._trial is not None:
            self.last_words = f"experiment {self._trial['knob']} {self._trial['value']} dropped" + (f": {why}" if why else "")
            self._trial = None
            self._base = []
            return self.last_words
        return None

    # ----- memory -----
    def state(self):
        return {"champ": self.champ, "tries": {k: {str(v): n for v, n in d.items()} for k, d in self.tries.items()},
                "history": self.history, "picks": self.picks}

    def load(self, d):
        if not d:
            return
        for k, v in (d.get("champ") or {}).items():
            if k in self.knobs and v in self.knobs[k][0]:
                self.champ[k] = v
        for k, tv in (d.get("tries") or {}).items():
            if k in self.knobs:
                self.tries[k] = {float(v) if "." in v else int(v): n for v, n in tv.items()}
        self.history = list(d.get("history") or [])[-30:]
        self.picks = int(d.get("picks") or 0)
