# forecast.py - refill forecast for the orchestrator's auto-resume (PRISM). No new packages.
#
# WHAT THIS MODULE DOES
#   The feeder has no sensor: nobody tells the cell that cubes were refilled. While IDLE the optimizer can PROBE
#   (resume -> one FEED -> the trusted camera aborts an empty transport after ~6 s, ~11 J). This module decides
#   WHEN to probe: often when refills usually happen, rarely when they do not (nights, breaks).
#
# THE MODEL  (a Poisson rate per hour of the week, learned online)
#   Each idle period is an observation: it lasted `exposure` seconds and either ended with a refill (a probe or
#   an operator resume found cubes) or was cut off (autoen stopped, park ...). For each hour-of-week bin:
#       rate = (refills + K * global_rate) / (idle_hours + K)        refills per idle hour, shrunk toward the
#   global rate when a bin has little data (K = PRIOR_HOURS). With no history at all the rate is PRIOR_RATE_H.
#   Delay until the next probe = the time by which a refill has happened with probability P_TARGET:
#       delay = -ln(1 - P_TARGET) / rate,  then at least BACKOFF x the idle time so far, clamped to [min, MAX_DELAY_S].
#   min = MIN_DELAY_S with a trusted camera (a probe costs ~11 J), MIN_DELAY_BLIND_S without (a probe costs ~50 J).
#
# STATE  forecast_state.json next to this file (atomic writes), so the history survives restarts.

import json
import math
import os
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(HERE, "forecast_state.json")

PRIOR_RATE_H = 30.0         # refills per idle hour before any history (~1 per 2 min: the 2026-10-26 sessions)
PRIOR_HOURS = 0.5           # idle hours of "global" evidence each hour-of-week bin starts with
P_TARGET = 0.5              # probe when a refill has happened with this probability
BACKOFF = 0.5               # ... but wait at least this fraction of the idle time so far (long idle -> rare probes)
MIN_DELAY_S = 30.0          # trusted camera: a failed probe costs ~11 J / ~13 s
MIN_DELAY_BLIND_S = 300.0   # camera not trusted: a failed probe runs the belt to arrive_timeout, ~50 J
MAX_DELAY_S = 1800.0        # never wait longer than 30 min between probes
MAX_GAPS = 200              # idle periods kept for inspection


def hour_bin(t_wall):
    """'<weekday 0=Mon>-<hour>' for a wall-clock time."""
    d = datetime.fromtimestamp(t_wall)
    return f"{d.weekday()}-{d.hour:02d}"


class Forecast:
    def __init__(self, path=STATE_FILE, clock=time.time):
        self.path, self.clock = path, clock
        self.bins = {}              # hour bin -> {"refills": n, "hours": idle hours}
        self.gaps = []              # recent idle periods: {"start", "secs", "refill"}
        self._idle_start = None     # wall time the current idle period began
        self._load()

    # ----- persistence -----

    def _load(self):
        try:
            with open(self.path, encoding="utf-8") as f:
                d = json.load(f)
            self.bins, self.gaps = d.get("bins", {}), d.get("gaps", [])
        except (OSError, ValueError):
            pass

    def _save(self):
        if not self.path:
            return
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump({"updated": datetime.now().isoformat(timespec="seconds"), "bins": self.bins,
                       "gaps": self.gaps[-MAX_GAPS:]}, f, indent=1)
        os.replace(tmp, self.path)

    # ----- observations -----

    def idle_started(self):
        if self._idle_start is None:
            self._idle_start = self.clock()

    def idle_ended(self, refill):
        """The idle period is over: refill=True if cubes were found (probe or operator), False if cut off."""
        if self._idle_start is None:
            return
        t0, t1 = self._idle_start, self.clock()
        self._idle_start = None
        self._add_exposure(t0, t1)
        if refill:
            b = self.bins.setdefault(hour_bin(t1), {"refills": 0, "hours": 0.0})
            b["refills"] += 1
        self.gaps.append({"start": datetime.fromtimestamp(t0).isoformat(timespec="seconds"),
                          "secs": round(t1 - t0, 1), "refill": bool(refill)})
        self._save()

    def _add_exposure(self, t0, t1):
        """Split the idle time over the hour-of-week bins it covered."""
        t = t0
        while t < t1:
            nxt = min(t1, (math.floor(t / 3600.0) + 1) * 3600.0)
            b = self.bins.setdefault(hour_bin(t), {"refills": 0, "hours": 0.0})
            b["hours"] = round(b["hours"] + (nxt - t) / 3600.0, 5)
            t = nxt

    # ----- the forecast -----

    def global_rate(self):
        refills = sum(b["refills"] for b in self.bins.values())
        hours = sum(b["hours"] for b in self.bins.values())
        return (refills + PRIOR_HOURS * PRIOR_RATE_H) / (hours + PRIOR_HOURS)

    def rate(self, t_wall=None):
        """Expected refills per idle hour in the hour-of-week of t_wall."""
        b = self.bins.get(hour_bin(self.clock() if t_wall is None else t_wall), {"refills": 0, "hours": 0.0})
        return (b["refills"] + PRIOR_HOURS * self.global_rate()) / (b["hours"] + PRIOR_HOURS)

    def probe_delay(self, idle_s, camera_trusted=True):
        """Seconds from the START of this idle period (or the last probe) until the next probe."""
        lam = max(self.rate(), 1e-6) / 3600.0                     # per second
        delay = -math.log(1.0 - P_TARGET) / lam
        delay = max(delay, BACKOFF * idle_s)
        lo = MIN_DELAY_S if camera_trusted else MIN_DELAY_BLIND_S
        return min(MAX_DELAY_S, max(lo, delay))

    def summary(self):
        return {"rate_h": round(self.rate(), 2), "global_rate_h": round(self.global_rate(), 2),
                "idle_periods": len(self.gaps), "bins": len(self.bins)}
