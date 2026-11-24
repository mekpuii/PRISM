#!/usr/bin/env python3
"""
BELT ANALYZER NODE v0  -  "what is the conveyor doing right now, and is it healthy?"
(PRISM, Raspberry Pi 4)                         Sister of analyzer_node.py (the ARM analyzer).

It NEVER commands an actuator. It measures, observes, records and reports.
Only the Orchestrator may act on what it says.

INPUT  (subscribed straight from the broker, not through the Logger)
  belt/telemetry   JSON  {"ts": epoch_s, "temp": degC, "current": mA, "velocity": rpm, "position": rev}
                   ** NO NODE PUBLISHES THIS YET **  (arm_node.py only handles the arm).  The field names and
                   units are exactly the Belt_* columns of your riff recording.  `temp` may be missing or
                   slower than the rest (DS18B20); the last value is kept.  `position` is optional.
  arm/ir, health/status   stored for context only

OUTPUT
  analyzer/belt/state       retained   state + live numbers + condition levels (now on change, else 1 Hz)
  analyzer/belt/transition  QoS1       one message per state change
  analyzer/belt/run         QoS1       one message per finished belt run (start -> stopped)
  analyzer/belt/condition   QoS1       a current / temp level changed (OK <-> WARN <-> ALARM)
  analyzer/belt/status      retained   online flag + Last Will
  belt_analyzer.db          SQLite     tables: transitions, runs, conditions

STATES  (from belt speed and how fast that speed is changing - no statistics needed)
  STOPPED   |speed| below V_STOP_RPM
  STARTING  moving and speeding up
  RUNNING   moving at steady speed           detail says "forward" or "REVERSE"
  STOPPING  moving and slowing down (coast to a halt)
  STALLED   current >= STALL_CURRENT_MA while the belt does not move, for STALL_HOLD_S
  NO_DATA   no fresh belt/telemetry. "I cannot tell", not "the belt is broken".
A normal run is STOPPED > STARTING > RUNNING > STOPPING > STOPPED. Any other jump is flagged "unexpected".

RUN RECORD (one per run): seconds, distance_rev, peak/steady current, steady speed, charge (mA*s),
  start-up and stop-down times, temp at start/end, direction, result
    OK        full sequence, forward, no condition above OK
    ANOMALY   full sequence, but something odd (reasons listed)
    PARTIAL   analyzer started mid-run, or the sequence was incomplete
    ABORTED   telemetry vanished mid-run

CONDITIONS  (your numbers, in belt_thresholds.json - the node never invents a limit)
  per STATE (STOPPED, STARTING, RUNNING, STOPPING, STALLED): warn_current / alarm_current  (mA, |current|)
  TEMPERATURE: warn / alarm (degC, all states).  HOLD_S: seconds a value must stay over a limit.
  It only REPORTS. Hardware STOP limits stay in the Mega / Orchestrator.

RUN
  python3 belt_analyzer_node.py --calibrate --ref riff_raw.csv   open the thresholds next to the recorded data,
                                                         change them, test them, save them (like --calibrate of the camera)
  python3 belt_analyzer_node.py --write-thresholds       create belt_thresholds.json to edit by hand
  python3 belt_analyzer_node.py                          live (MQTT_HOST env var = broker)
  python3 belt_analyzer_node.py --replay riff_raw.csv    offline test on a recording
  python3 belt_analyzer_node.py --play riff_raw.csv      publish a recording to belt/telemetry (for testing)
  python3 belt_analyzer_node.py --show                   last transitions / runs / conditions

Install: pip install paho-mqtt        (--replay, --show and --write-thresholds work without it)
Not a safety function.
"""

import argparse
import csv
import json
import os
import queue
import signal
import sqlite3
import statistics
import sys
import threading
import time
from collections import Counter, defaultdict, deque
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
except ImportError:                      # replay / show do not need MQTT
    mqtt = None

# ============================================================================
# CONFIG  -  every number you may want to tune lives here
# ============================================================================
BROKER_HOST = os.environ.get("MQTT_HOST", "localhost")
BROKER_PORT = 1883
DB_FILE = "belt_analyzer.db"
THRESHOLD_FILE = "belt_thresholds.json"

# Topics
T_TELEMETRY = "belt/telemetry"
T_IR, T_HEALTH = "arm/ir", "health/status"                     # context only
T_STATE, T_TRANSITION, T_RUN, T_CONDITION, T_STATUS = (
    "analyzer/belt/state", "analyzer/belt/transition", "analyzer/belt/run",
    "analyzer/belt/condition", "analyzer/belt/status")

# Which sign of `velocity` means "forward". Your riff recording runs at about -325 rpm and its position
# counts DOWN, so -1 matches the data. NOTE: belt-cell-bringup says "count sign must be + for forward":
# the two disagree. Decide which one is right and set this (or fix the encoder wiring). A run in the
# other direction is flagged "REVERSE" and makes the run an ANOMALY.
FORWARD_SIGN = -1

# State detection (rpm, rpm per second, mA, seconds). Measured on the riff run:
# steady speed 325 rpm, ramp-up ~1 s, coast-down ~0.8 s, steady current ~265 mA, stopped < 60 mA.
# Those rpm were CPR-1320 units (9x the real output rpm; 325 = 36 real). Since the CPR fix (11880) velocity is the
# real output rpm, so every rpm threshold below is the old value / 9.
V_STOP_RPM = 0.55             # below this the belt counts as not moving
ACCEL_RPM_S = 16.7            # |speed| changing faster than this = STARTING / STOPPING (steady ripple is ~1)
STEADY_RPM_S = 4.4            # once STARTING / STOPPING, stay in it until the slope falls inside +-this (hysteresis)
V_RUN_MIN_RPM = 3.3           # below this the belt is still "creeping" up or down. A belt that runs STEADILY slower than
                               # this would not be recognised as RUNNING: set it below your slowest real belt speed.
WINDOW_S = 0.6                 # time span used to measure the speed slope
STALL_CURRENT_MA = 150.0       # stopped belt drawing at least this much ...
STALL_HOLD_S = 0.8             # ... for this long = STALLED  (start-up stiction lasts ~0.2 s at < 50 mA)
DEBOUNCE_S = 0.2               # a new state must hold this long before it is committed (backdated)
STALE_S = 1.5                  # no telemetry for this long -> NO_DATA
STATE_REFRESH_S = 1.0          # analyzer/belt/state is re-published at least this often
SUMMARY_EVERY_S = 10.0         # console heartbeat line, so the terminal is never silent
TEMP_STALE_S = 30.0            # a temp reading older than this is not used for conditions
HYSTERESIS = 0.10              # a level clears only after the value is 10 % below its threshold

EXPECTED_NEXT = {"STOPPED": {"STARTING"}, "STARTING": {"RUNNING"},
                 "RUNNING": {"STOPPING"}, "STOPPING": {"STOPPED"}}
COUNTED_STATES = ("STOPPED", "STARTING", "RUNNING", "STOPPING", "STALLED")

# ----------------------------------------------------------------------------
# DEFAULT THRESHOLDS = a STARTING POINT only. Source: your 30-minute riff run (100 runs, NO load on the
# belt). warn ~ 1.25 x the p99 of that state, alarm ~ 1.6 x its max. Edit belt_thresholds.json, not this.
# Current is compared as |current| because the sign under load is not verified (INA260 read -3 mA idle).
# ----------------------------------------------------------------------------
# The FILE schema (what you edit) is flat: one block per state, plus TEMPERATURE and an optional HOLD_S.
DEFAULT_THRESHOLDS = {
    "STOPPED":  {"warn_current": 25,  "alarm_current": 100},
    "STARTING": {"warn_current": 700, "alarm_current": 1000},
    "RUNNING":  {"warn_current": 340, "alarm_current": 450},
    "STOPPING": {"warn_current": 50,  "alarm_current": 150},
    "STALLED":  {"warn_current": 150, "alarm_current": 400},
    "TEMPERATURE": {"warn": 40, "alarm": 55},
    "HOLD_S": {"current": 1.0, "temp": 5.0},        # how long a value must stay over a limit before it counts
}


def _convert(file_dict):
    """file schema -> the internal tables the Conditions class reads."""
    th = {"current_mA": {}, "temp_C": {}, "hold_s": {}}
    for state in COUNTED_STATES:
        row = file_dict.get(state)
        if isinstance(row, dict):
            th["current_mA"][state] = {"warn": row.get("warn_current"), "alarm": row.get("alarm_current")}
    th["temp_C"] = {"warn": (file_dict.get("TEMPERATURE") or {}).get("warn"),
                    "alarm": (file_dict.get("TEMPERATURE") or {}).get("alarm")}
    th["hold_s"] = dict(file_dict.get("HOLD_S") or {})
    return th


def load_file_schema(path):
    """Read belt_thresholds.json in FILE schema, merged over the defaults. Returns (dict, file_was_used)."""
    merged = json.loads(json.dumps(DEFAULT_THRESHOLDS))                # deep copy
    try:
        with open(path) as f:
            user = json.load(f)
        if not isinstance(user, dict):
            raise ValueError("top level must be an object")
        for key, row in user.items():
            if isinstance(row, dict):
                merged.setdefault(key, {}).update({k: float(x) for k, x in row.items() if x is not None})
        return merged, True
    except (OSError, ValueError, TypeError):
        return json.loads(json.dumps(DEFAULT_THRESHOLDS)), False


def load_thresholds(path):
    """A key you leave out keeps its default; a missing or broken file (or a non-number value) means the
    defaults. Returns (internal tables, file_was_used)."""
    merged, used = load_file_schema(path)
    return _convert(merged), used


# ============================================================================
# 1. STATE ENGINE  -  pure logic, no MQTT, no clock of its own (live / replay / tests share it)
# ============================================================================
class BeltEngine:
    def __init__(self):
        self.win = deque()                   # (t, |speed|)  for the slope
        self.hist = deque()                  # (t, position) the last few seconds, to find the position at a backdated time
        self.state, self.detail, self.since = "NO_DATA", "waiting for belt/telemetry", None
        self.cand = None                     # (state, detail, t_first) debounce candidate
        self.skipped = None                  # a candidate that was replaced before it was committed (see _commit)
        self.last_raw = "NO_DATA"            # previous raw (not debounced) classification
        self.last_t = None
        self.stall_t0 = None
        self.temp, self.temp_t = None, None
        self.feat = {"speed": 0.0, "current": 0.0, "velocity": 0.0, "slope": 0.0, "position": None}
        self.run = None
        self.runs_ok = self.runs_total = 0
        self.last_run = None

    # ---- public API -----------------------------------------------------------
    def on_sample(self, t, velocity, current, position=None, temp=None):
        """Feed one telemetry sample. Returns a list of events (transition / run dicts)."""
        events = self.on_tick(t)
        if self.last_t is not None and t <= self.last_t:
            return events                                      # out-of-order / duplicate: ignore
        dt = 0.0 if self.last_t is None else min(t - self.last_t, 0.5)     # never integrate across a gap
        self.last_t = t
        if temp is not None and temp > -100:                   # DS18B20 reports -127 when it fails
            self.temp, self.temp_t = temp, t
        speed = abs(velocity)
        self.win.append((t, speed))
        while self.win and t - self.win[0][0] > WINDOW_S:
            self.win.popleft()
        if position is not None:
            self.hist.append((t, position))
            while self.hist and t - self.hist[0][0] > 4.0:
                self.hist.popleft()
        slope = 0.0
        if len(self.win) >= 2 and self.win[-1][0] > self.win[0][0]:
            slope = (self.win[-1][1] - self.win[0][1]) / (self.win[-1][0] - self.win[0][0])
        self.feat = {"speed": speed, "current": abs(current), "velocity": velocity,
                     "slope": slope, "position": position}
        state, detail = self._classify(t, velocity, current, speed, slope)
        self.last_raw = state
        if self.run is not None:                               # integrate the run while it lasts
            r = self.run
            r["charge"] += abs(current) * dt
            r["peak_current"] = max(r["peak_current"], abs(current))
            r["peak_speed"] = max(r["peak_speed"], speed)
            if self.state == "RUNNING" and state == "RUNNING" and self.since is not None and t - self.since >= 0.6:
                # steady-state averages: committed AND raw state RUNNING (the debounce lags by 0.2 s), and skip the
                # first 0.6 s, where the speed still creeps up
                r["steady_i"].append(abs(current))
                r["steady_v"].append(speed)
            r["last_pos"] = position if position is not None else r["last_pos"]
            if state == "STALLED":
                r["flags"].add("stalled")
            if state == "RUNNING" and not detail.startswith("forward"):
                r["flags"].add("reverse")
        return events + self._propose(state, detail, t)

    def on_tick(self, t):
        """Call regularly with the current time: no fresh data -> NO_DATA at once."""
        if self.last_t is not None and t - self.last_t > STALE_S and self.state != "NO_DATA":
            self.win.clear()
            self.cand = None
            return self._commit("NO_DATA", "no telemetry for %.1f s" % (t - self.last_t), self.last_t)
        return []

    # ---- classification ------------------------------------------------------
    def _classify(self, t, velocity, current, speed, slope):
        if speed < V_STOP_RPM:
            if abs(current) >= STALL_CURRENT_MA:
                if self.stall_t0 is None:
                    self.stall_t0 = t
                if t - self.stall_t0 >= STALL_HOLD_S:
                    return "STALLED", f"{abs(current):.0f} mA with the belt at {speed:.0f} rpm"
            else:
                self.stall_t0 = None
            return "STOPPED", "not moving"
        self.stall_t0 = None
        way = "forward" if (velocity > 0) == (FORWARD_SIGN > 0) else "REVERSE"
        rest = self.last_raw in ("STOPPED", "STALLED")         # the previous sample was at rest: any motion is a start
        if slope <= -ACCEL_RPM_S:
            return "STOPPING", f"{way}, {speed:.0f} rpm and falling"
        if slope >= ACCEL_RPM_S or rest:
            return "STARTING", f"{way}, {speed:.0f} rpm and rising"
        # hysteresis: once speeding up / slowing down, stay there until the slope is clearly flat
        if self.last_raw == "STARTING" and slope >= STEADY_RPM_S:
            return "STARTING", f"{way}, {speed:.0f} rpm and rising"
        if self.last_raw == "STOPPING" and slope <= -STEADY_RPM_S:
            return "STOPPING", f"{way}, {speed:.0f} rpm and falling"
        if speed < V_RUN_MIN_RPM:
            # creeping with a flat slope: continue the direction we were already going
            if self.last_raw in ("RUNNING", "STOPPING"):
                return "STOPPING", f"{way}, {speed:.0f} rpm and falling"
            return "STARTING", f"{way}, {speed:.0f} rpm and rising"
        return "RUNNING", f"{way}, {speed:.0f} rpm"

    # ---- debounce + commit ----------------------------------------------------
    def _propose(self, state, detail, t):
        if state == self.state:
            self.detail = detail
            self.cand = None
            return []
        if self.state == "NO_DATA":                            # first sample / data is back: no debounce
            return self._commit(state, detail, t)
        if self.cand is None or self.cand[0] != state:
            # A candidate that dies before it is committed is remembered if it was the NEXT step of the
            # normal sequence: a fast stop (RUNNING -> 40 rpm -> 3 rpm) can pass STOPPING in under 0.2 s.
            if self.cand is not None and self.cand[0] in EXPECTED_NEXT.get(self.state, ()):
                self.skipped = self.cand
            self.cand = (state, detail, t)
            return []
        if t - self.cand[2] >= DEBOUNCE_S:                     # held long enough: commit, backdated
            first = self.cand[2]
            self.cand = None
            return self._commit(state, detail, first)
        return []

    def _pos_at(self, t):
        """Position sample nearest to time t (the debounce backdates state changes)."""
        if not self.hist:
            return None
        return min(self.hist, key=lambda h: abs(h[0] - t))[1]

    def _commit(self, to, detail, since):
        skipped, self.skipped = self.skipped, None
        if (skipped is not None and to not in EXPECTED_NEXT.get(self.state, ())
                and skipped[0] in EXPECTED_NEXT.get(self.state, ()) and to in EXPECTED_NEXT.get(skipped[0], ())
                and skipped[2] < since):
            # the physical order is fixed, so the step that was too short to debounce really happened:
            # book it (backdated) instead of reporting a false "unexpected" jump
            first_events = self._commit(skipped[0], skipped[1], skipped[2])
            return first_events + self._commit(to, detail, since)
        prev, prev_since = self.state, self.since
        dwell = None if prev_since is None else round(since - prev_since, 2)
        unexpected = (prev in EXPECTED_NEXT and to in EXPECTED_NEXT and to not in EXPECTED_NEXT[prev])
        events = [{"type": "transition", "t": since, "from": prev, "to": to, "detail": detail,
                   "dwell_s": dwell, "speed_rpm": round(self.feat["speed"], 1),
                   "current_mA": round(self.feat["current"], 1), "unexpected": unexpected}]
        if self.run is not None and dwell is not None:
            self.run["phases"][prev] += dwell
        # A run starts when the belt leaves STOPPED, and ends when it is back in STOPPED.
        if self.run is None and to in ("STARTING", "RUNNING", "STOPPING") and prev in ("STOPPED", "NO_DATA", "STALLED"):
            self.run = {"start": since, "phases": defaultdict(float), "seen": [], "unexpected": [], "flags": set(),
                        "charge": 0.0, "peak_current": 0.0, "peak_speed": 0.0, "steady_i": [], "steady_v": [],
                        "pos0": self._pos_at(since), "last_pos": None, "temp0": self.temp,
                        "mid_run": prev == "NO_DATA" or to != "STARTING",
                        "dir": "forward" if detail.startswith("forward") else "REVERSE"}
        if self.run is not None:
            if not self.run["seen"] or self.run["seen"][-1] != to:
                self.run["seen"].append(to)
            if unexpected:
                self.run["unexpected"].append(f"{prev}->{to}")
            if to in ("STOPPED", "NO_DATA"):
                events.append(self._finish_run(since, aborted=(to == "NO_DATA")))
        self.state, self.detail, self.since = to, detail, since
        return events

    def _finish_run(self, end, aborted):
        r, self.run = self.run, None
        seen = r["seen"]
        complete = (not r["mid_run"]) and all(s in seen for s in ("STARTING", "RUNNING", "STOPPING"))
        reasons = list(r["unexpected"]) + sorted(r["flags"])
        if aborted:
            result = "ABORTED"
        elif not complete:
            result = "PARTIAL"
        else:
            result = "ANOMALY" if reasons else "OK"
        seconds = round(end - r["start"], 2)
        dist = None
        if r["pos0"] is not None and r["last_pos"] is not None:
            dist = round(abs(r["last_pos"] - r["pos0"]), 3)
        out = {"type": "run", "t": end, "start": r["start"], "seconds": seconds, "result": result,
               "direction": r["dir"], "distance_rev": dist,
               "steady_speed_rpm": round(statistics.mean(r["steady_v"]), 1) if r["steady_v"] else None,
               "steady_current_mA": round(statistics.mean(r["steady_i"]), 1) if r["steady_i"] else None,
               "peak_current_mA": round(r["peak_current"], 1), "peak_speed_rpm": round(r["peak_speed"], 1),
               "charge_mAs": round(r["charge"], 1),
               "startup_s": round(r["phases"].get("STARTING", 0.0), 2),
               "stopdown_s": round(r["phases"].get("STOPPING", 0.0), 2),
               "temp_start_C": r["temp0"], "temp_end_C": self.temp,
               "reasons": reasons, "phases": {k: round(v, 2) for k, v in r["phases"].items()}}
        self.runs_total += 1
        if result == "OK":
            self.runs_ok += 1
        self.last_run = out
        return out


# ============================================================================
# 2. CONDITIONS  -  your thresholds, evaluated per state; pure logic like the engine
# ============================================================================
class Conditions:
    """Two metrics: 'current' (limits depend on the state) and 'temp' (one limit for all states).
    A level (OK / WARN / ALARM) is raised only after the value stays at or above its threshold for
    hold_s, and cleared only after it stays HYSTERESIS below it for hold_s: no flicker around a limit."""

    def __init__(self, thresholds):
        self.set_thresholds(thresholds)
        self.level = {"current": "OK", "temp": "OK"}
        self.raw = {"current": ("OK", None), "temp": ("OK", None)}     # (raw level, since t)
        self.worst = {"current": "OK", "temp": "OK"}                   # worst level since last read_and_reset

    def set_thresholds(self, th):
        self.th = th

    @staticmethod
    def _rank(level):
        return {"OK": 0, "WARN": 1, "ALARM": 2}[level]

    def _limits(self, metric, state):
        if metric == "temp":
            return self.th["temp_C"].get("warn"), self.th["temp_C"].get("alarm")
        row = self.th["current_mA"].get(state)
        return (None, None) if row is None else (row.get("warn"), row.get("alarm"))

    def update(self, t, state, current, temp):
        """Returns a list of condition events (usually empty)."""
        events = []
        for metric, value in (("current", None if current is None else abs(current)), ("temp", temp)):
            if value is None or state == "NO_DATA":
                continue                                       # nothing fresh to judge
            warn, alarm = self._limits(metric, state)
            hyst = (1.0 - HYSTERESIS)
            # level the number would have WITHOUT hysteresis, then lower it only if we are in the dead band
            if alarm is not None and value >= alarm:
                want = "ALARM"
            elif warn is not None and value >= warn:
                want = "WARN"
            else:
                want = "OK"
            cur = self.level[metric]
            if self._rank(want) < self._rank(cur):             # falling: inside the dead band the old level stays
                limit = {"ALARM": alarm, "WARN": warn}[cur]
                if limit is not None and value >= limit * hyst:
                    want = cur
            raw, since = self.raw[metric]
            if raw != want:
                self.raw[metric] = (want, t)                   # the wanted level changed: restart its timer
                continue
            hold = self.th["hold_s"]["current" if metric == "current" else "temp"]
            if want != cur and since is not None and t - since >= hold:
                limit = {"ALARM": alarm, "WARN": warn}.get(want if want != "OK" else cur)
                events.append({"type": "condition", "t": since, "metric": metric, "state": state,
                               "level": want, "prev": cur, "value": round(value, 1),
                               "threshold": limit, "held_s": round(t - since, 2),
                               "text": f"{metric} {value:.0f} {'mA' if metric == 'current' else 'C'} "
                                       f"in {state}: {cur} -> {want}"})
                self.level[metric] = want
            if self._rank(self.level[metric]) > self._rank(self.worst[metric]):
                self.worst[metric] = self.level[metric]
        return events


# ============================================================================
# 3. SQLITE LOGGER
# ============================================================================
class Log:
    def __init__(self, path):
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")           # readers (dashboards) never block us
        self.db.execute("PRAGMA synchronous=NORMAL")         # a few rows per run: no full fsync needed
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS transitions(
                id INTEGER PRIMARY KEY, ts REAL, iso TEXT, from_state TEXT, to_state TEXT, detail TEXT,
                dwell_s REAL, speed_rpm REAL, current_mA REAL, unexpected INTEGER);
            CREATE TABLE IF NOT EXISTS runs(
                id INTEGER PRIMARY KEY, start_ts REAL, start_iso TEXT, end_iso TEXT, seconds REAL, result TEXT,
                direction TEXT, distance_rev REAL, steady_speed_rpm REAL, steady_current_mA REAL,
                peak_current_mA REAL, peak_speed_rpm REAL, charge_mAs REAL, startup_s REAL, stopdown_s REAL,
                temp_start_C REAL, temp_end_C REAL, reasons TEXT, phases TEXT);
            CREATE TABLE IF NOT EXISTS conditions(
                id INTEGER PRIMARY KEY, ts REAL, iso TEXT, metric TEXT, state TEXT, level TEXT, prev TEXT,
                value REAL, threshold REAL, held_s REAL);
            CREATE INDEX IF NOT EXISTS ix_tr_ts ON transitions(ts);
            CREATE INDEX IF NOT EXISTS ix_run_ts ON runs(start_ts);
            CREATE INDEX IF NOT EXISTS ix_co_ts ON conditions(ts);""")
        self.db.commit()

    def write(self, ev):
        """ev carries WALL-clock times (the caller converts)."""
        iso = lambda ts: datetime.fromtimestamp(ts).isoformat(timespec="milliseconds")
        if ev["type"] == "transition":
            self.db.execute("INSERT INTO transitions(ts,iso,from_state,to_state,detail,dwell_s,speed_rpm,current_mA,"
                            "unexpected) VALUES(?,?,?,?,?,?,?,?,?)",
                            (ev["t"], iso(ev["t"]), ev["from"], ev["to"], ev["detail"], ev["dwell_s"],
                             ev["speed_rpm"], ev["current_mA"], int(ev["unexpected"])))
        elif ev["type"] == "run":
            self.db.execute("INSERT INTO runs(start_ts,start_iso,end_iso,seconds,result,direction,distance_rev,"
                            "steady_speed_rpm,steady_current_mA,peak_current_mA,peak_speed_rpm,charge_mAs,startup_s,"
                            "stopdown_s,temp_start_C,temp_end_C,reasons,phases) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                            (ev["start"], iso(ev["start"]), iso(ev["t"]), ev["seconds"], ev["result"], ev["direction"],
                             ev["distance_rev"], ev["steady_speed_rpm"], ev["steady_current_mA"],
                             ev["peak_current_mA"], ev["peak_speed_rpm"], ev["charge_mAs"], ev["startup_s"],
                             ev["stopdown_s"], ev["temp_start_C"], ev["temp_end_C"],
                             ",".join(ev["reasons"]), json.dumps(ev["phases"])))
        else:
            self.db.execute("INSERT INTO conditions(ts,iso,metric,state,level,prev,value,threshold,held_s) "
                            "VALUES(?,?,?,?,?,?,?,?,?)",
                            (ev["t"], iso(ev["t"]), ev["metric"], ev["state"], ev["level"], ev["prev"],
                             ev["value"], ev["threshold"], ev["held_s"]))
        self.db.commit()                                     # a few rows per run: commit each

    def close(self):
        self.db.close()


# ============================================================================
# 4. MQTT BUS  -  subscriber + store + publisher.  Callbacks only STORE or QUEUE data.
# ============================================================================
def parse_telemetry(d):
    """belt/telemetry payload -> (velocity, current, position|None, temp|None) or None if unusable."""
    try:
        vel, cur = float(d["velocity"]), float(d["current"])
    except (KeyError, TypeError, ValueError):
        return None
    def opt(key):
        try:
            return None if d.get(key) is None else float(d[key])
        except (TypeError, ValueError):
            return None
    return vel, cur, opt("position"), opt("temp")


class Bus:
    def __init__(self, host, port, inbox):
        if mqtt is None:
            raise SystemExit("paho-mqtt is not installed:  pip install paho-mqtt")
        self.inbox, self.lock, self.store, self.connected = inbox, threading.Lock(), {}, False
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="belt_analyzer_node")
        self.c.will_set(T_STATUS, json.dumps({"online": False}), qos=1, retain=True)
        self.c.on_connect, self.c.on_disconnect = self._on_connect, self._on_disconnect
        self.c.on_message, self.c.on_connect_fail = self._on_message, self._on_connect_fail
        self.c.connect_async(host, port, keepalive=10)       # never blocks, retries in the background
        self.c.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[belt-analyzer] connected to broker: {reason_code}", flush=True)
        self.connected = True
        client.subscribe([(T_TELEMETRY, 0), (T_IR, 0), (T_HEALTH, 0)])   # re-subscribe on every connect
        client.publish(T_STATUS, json.dumps({"online": True, "ts": time.time()}), qos=1, retain=True)

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self.connected = False
        print("[belt-analyzer] lost the broker", flush=True)

    def _on_connect_fail(self, client, userdata):
        print(f"[belt-analyzer] cannot reach the broker at {BROKER_HOST}:{BROKER_PORT} - retrying", flush=True)

    def _on_message(self, client, userdata, msg):
        now = time.monotonic()
        try:
            data = json.loads(msg.payload)
        except ValueError:
            return                                           # garbage payload: drop
        if msg.topic == T_TELEMETRY:
            parsed = parse_telemetry(data) if isinstance(data, dict) else None
            if parsed:
                self.inbox.put((now, parsed))
        with self.lock:
            self.store[msg.topic] = {"t": now, "d": data, "retained": bool(msg.retain)}

    def latest(self, topic):
        with self.lock:
            return self.store.get(topic)

    def publish(self, topic, payload, qos=0, retain=False):
        self.c.publish(topic, json.dumps(payload), qos=qos, retain=retain)

    def close(self):
        try:   # wait_for_publish raises if the broker was never reachable; shutdown must still finish
            self.c.publish(T_STATUS, json.dumps({"online": False, "ts": time.time()}),
                           qos=1, retain=True).wait_for_publish(1.0)
        except Exception:
            pass
        self.c.loop_stop()
        self.c.disconnect()


# ============================================================================
# 5. LIVE MODE
# ============================================================================
def build_state_payload(eng, cond, bus, now, wall_offset):
    tel, ir, health = bus.latest(T_TELEMETRY), bus.latest(T_IR), bus.latest(T_HEALTH)
    nodes = (health["d"].get("nodes") or {}) if health and isinstance(health["d"], dict) else {}
    f = eng.feat
    blind = eng.state == "NO_DATA"
    temp_ok = eng.temp is not None and eng.temp_t is not None and now - eng.temp_t <= TEMP_STALE_S
    return {
        "ts": now + wall_offset, "state": eng.state, "detail": eng.detail,
        "since_s": None if eng.since is None else round(now - eng.since, 2),
        # NO_DATA = "cannot tell": never show the last numbers as if they were live
        "speed_rpm": None if blind else round(f["speed"], 1), "current_mA": None if blind else round(f["current"], 1),
        "temp_C": eng.temp if temp_ok and not blind else None, "position_rev": None if blind else f["position"],
        "levels": dict(cond.level), "run_active": eng.run is not None,
        "runs_ok": eng.runs_ok, "runs_total": eng.runs_total, "last_run": eng.last_run,
        "ir_detected": None if ir is None or not isinstance(ir["d"], dict) else bool(ir["d"].get("detected")),
        "health": {k: v.get("status") for k, v in nodes.items()} or None,
        "telemetry_age_s": None if tel is None else round(now - tel["t"], 2),
    }


def run_live(db_path, th_path):
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))   # systemctl stop
    eng, log, inbox = BeltEngine(), Log(db_path), queue.SimpleQueue()
    th, found = load_thresholds(th_path)
    cond = Conditions(th)
    th_mtime = os.path.getmtime(th_path) if found else None
    bus = Bus(BROKER_HOST, BROKER_PORT, inbox)
    wall_offset = time.time() - time.monotonic()
    print(f"[belt-analyzer] up. broker {BROKER_HOST}:{BROKER_PORT}, db {db_path}, thresholds "
          f"{th_path if found else 'built-in defaults (run --write-thresholds to edit them)'}. Ctrl+C to stop.",
          flush=True)
    last_pub = last_sum = last_th = time.monotonic()
    try:
        while True:
            events = []
            try:
                item = inbox.get(timeout=0.05)               # wait at most 50 ms so staleness is still checked
                while True:
                    t, (vel, cur, pos, temp) = item
                    events += eng.on_sample(t, vel, cur, pos, temp)
                    temp_ok = eng.temp is not None and eng.temp_t is not None and t - eng.temp_t <= TEMP_STALE_S
                    events += cond.update(t, eng.state, cur, eng.temp if temp_ok else None)
                    item = inbox.get_nowait()
            except queue.Empty:
                pass
            now = time.monotonic()
            events += eng.on_tick(now)
            if now - last_th >= 5.0:                         # pick up edits to the threshold file without a restart
                last_th = now
                try:
                    m = os.path.getmtime(th_path)
                    if m != th_mtime:
                        th_mtime = m
                        cond.set_thresholds(load_thresholds(th_path)[0])
                        print(f"[belt-analyzer] thresholds reloaded from {th_path}", flush=True)
                except OSError:
                    pass
            for ev in events:
                ev["t"] += wall_offset
                if "start" in ev:
                    ev["start"] += wall_offset
                log.write(ev)
                hhmmss = datetime.fromtimestamp(ev["t"]).strftime("%H:%M:%S")
                if ev["type"] == "transition":
                    bus.publish(T_TRANSITION, ev, qos=1)
                    print(f"[{hhmmss}] {ev['from']:<9}-> {ev['to']:<9} ({ev['detail']})"
                          f"{'  UNEXPECTED' if ev['unexpected'] else ''}", flush=True)
                elif ev["type"] == "run":
                    bus.publish(T_RUN, ev, qos=1)
                    print(f"[{hhmmss}] RUN {ev['result']} {ev['seconds']} s, {ev['distance_rev']} rev, "
                          f"steady {ev['steady_current_mA']} mA @ {ev['steady_speed_rpm']} rpm"
                          f"{'  ' + ', '.join(ev['reasons']) if ev['reasons'] else ''}", flush=True)
                else:
                    bus.publish(T_CONDITION, ev, qos=1)
                    print(f"[{hhmmss}] CONDITION {ev['text']} (held {ev['held_s']} s)", flush=True)
            if events or now - last_pub >= STATE_REFRESH_S:
                last_pub = now
                bus.publish(T_STATE, build_state_payload(eng, cond, bus, now, wall_offset),
                            qos=1 if events else 0, retain=True)
            if now - last_sum >= SUMMARY_EVERY_S:
                last_sum = now
                tel = bus.latest(T_TELEMETRY)
                print(f"[{time.strftime('%H:%M:%S')}] state={eng.state}  broker={'up' if bus.connected else 'DOWN'}  "
                      f"belt/telemetry={'none yet' if tel is None else '%.1f s ago' % (now - tel['t'])}  "
                      f"runs_ok={eng.runs_ok}/{eng.runs_total}  levels={cond.level}", flush=True)
    except KeyboardInterrupt:
        print("\n[belt-analyzer] stopping")
    finally:
        bus.close()
        log.close()


# ============================================================================
# 6. REPLAY  -  run a recorded CSV through the same engine and report
# ============================================================================
def read_riff(path):
    """riff_raw CSV -> list of (t seconds, velocity rpm, current mA, position rev, temp C, phase)."""
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                t = datetime.fromisoformat(r["time"]).timestamp()
                vel, cur = float(r["Belt_velocity"]), float(r["Belt_current"])
            except (ValueError, KeyError):
                continue
            pos = r.get("Belt_position")
            temp = r.get("Belt_temp")
            rows.append((t, vel, cur, float(pos) if pos not in (None, "") else None,
                         float(temp) if temp not in (None, "") else None, r.get("phase", "")))
    return rows


def pct(v, p):
    v = sorted(v)
    k = (len(v) - 1) * p / 100
    f = int(k)
    c = min(f + 1, len(v) - 1)
    return v[f] + (v[c] - v[f]) * (k - f)


def run_replay(path, th_path, db_path=None, verbose=False):
    rows = read_riff(path)
    eng, th = BeltEngine(), load_thresholds(th_path)[0]
    cond, log = Conditions(th), (Log(db_path) if db_path else None)
    events, per_state = [], defaultdict(lambda: {"cur": [], "n": 0})
    for t, vel, cur, pos, temp, phase in rows:
        events += eng.on_sample(t, vel, cur, pos, temp)
        events += cond.update(t, eng.state, cur, eng.temp)
        per_state[eng.state]["cur"].append(abs(cur))
    if log:
        for e in events:
            for k in ("t", "start"):
                if k in e:
                    e[k] = e[k]                              # replay keeps the recording's own clock
            log.write(e)
        log.close()
    if verbose:
        for e in events:
            if e["type"] == "transition":
                print(f"{e['t'] - rows[0][0]:8.2f}s  {e['from']:<9}-> {e['to']:<9} {e['detail']}")
    print(f"\nREPLAY {os.path.basename(path)}: {len(rows)} samples, {rows[-1][0] - rows[0][0]:.0f} s")
    runs = [e for e in events if e["type"] == "run"]
    # independent count: stretches of the file where |velocity| >= 20 rpm (no engine involved)
    ep, on = 0, False
    for _, vel, *_r in rows:
        if abs(vel) >= 20 and not on:
            on = True
            ep += 1
        elif abs(vel) < 20:
            on = False
    print(f"runs found by the engine: {len(runs)}   independent count (|v|>=20 rpm stretches): {ep}")
    print("results:", dict(Counter(r["result"] for r in runs)))
    for r in runs:
        if r["result"] != "OK":
            print(f"  run @ {r['start'] - rows[0][0]:7.1f} s  {r['result']:<8} {r['seconds']:5.2f} s  {', '.join(r['reasons'])}")
    ok = [r for r in runs if r["result"] == "OK"]
    if ok:
        print(f"\n{'OK-run feature':<22}{'mean':>9}{'sd':>8}{'min':>9}{'max':>9}")
        for key, label in (("seconds", "seconds"), ("distance_rev", "distance (rev)"),
                           ("steady_speed_rpm", "steady speed (rpm)"), ("steady_current_mA", "steady current (mA)"),
                           ("peak_current_mA", "peak current (mA)"), ("charge_mAs", "charge (mA*s)"),
                           ("startup_s", "start-up (s)"), ("stopdown_s", "stop-down (s)")):
            v = [r[key] for r in ok if r[key] is not None]
            if v:
                print(f"{label:<22}{statistics.mean(v):>9.2f}{statistics.pstdev(v):>8.2f}{min(v):>9.2f}{max(v):>9.2f}")
    print("\nstate           samples   |current| mA:  p50     p95     p99     max")
    for s in COUNTED_STATES + ("NO_DATA",):
        v = per_state[s]["cur"]
        if v:
            print(f"{s:<14}{len(v):>8}                {pct(v, 50):>6.0f}  {pct(v, 95):>6.0f}  {pct(v, 99):>6.0f}  {max(v):>6.0f}")
    conds = [e for e in events if e["type"] == "condition"]
    print(f"\ncondition events with the current thresholds: {len(conds)}")
    for e in conds[:12]:
        print(f"  {e['t'] - rows[0][0]:7.1f} s  {e['text']}  (held {e['held_s']} s)")
    unexp = [e for e in events if e["type"] == "transition" and e["unexpected"]]
    print(f"unexpected transitions: {len(unexp)}")
    return events


# ============================================================================
# 7. --play : publish a recording to belt/telemetry (so the node can be tested before a real publisher exists)
# ============================================================================
def run_play(path, speed):
    rows = read_riff(path)
    if mqtt is None:
        raise SystemExit("paho-mqtt is not installed:  pip install paho-mqtt")
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="belt_player")
    c.connect(BROKER_HOST, BROKER_PORT, 10)
    c.loop_start()
    print(f"[play] {len(rows)} samples to {T_TELEMETRY} at {speed}x. Ctrl+C to stop.", flush=True)
    t0, w0 = rows[0][0], time.monotonic()
    try:
        for t, vel, cur, pos, temp, _ in rows:
            delay = (t - t0) / speed - (time.monotonic() - w0)
            if delay > 0:
                time.sleep(delay)
            c.publish(T_TELEMETRY, json.dumps({"ts": time.time(), "temp": temp, "current": cur,
                                               "velocity": vel, "position": pos}))
    except KeyboardInterrupt:
        pass
    c.loop_stop()
    c.disconnect()
    print("[play] done")


# ============================================================================
# 7b. --calibrate : look at the thresholds next to the recorded data, change them, test them, save them
# ============================================================================
def reference_stats(path):
    """Run a recording through the engine: per state the p99 / max of |current|, and the max temp."""
    rows = read_riff(path)
    eng, per = BeltEngine(), defaultdict(list)
    tmax = None
    for t, vel, cur, pos, temp, _ in rows:
        eng.on_sample(t, vel, cur, pos, temp)
        per[eng.state].append(abs(cur))
        if temp is not None:
            tmax = temp if tmax is None else max(tmax, temp)
    return {s: (pct(v, 99), max(v)) for s, v in per.items() if v}, tmax, rows


def _fmt(x):
    return "-" if x is None else (f"{x:g}")


def _table(th, ref):
    print(f"\n  #  {'STATE':<12}{'warn_current':>13}{'alarm_current':>15}   | recorded: {'p99':>6}{'max':>7}   (mA)")
    for i, st in enumerate(COUNTED_STATES, 1):
        row = th.get(st, {})
        r = ref.get(st) if ref else None
        print(f"  {i}  {st:<12}{_fmt(row.get('warn_current')):>13}{_fmt(row.get('alarm_current')):>15}   | "
              f"{'':>10}{('%.0f' % r[0]) if r else '-':>6}{('%.0f' % r[1]) if r else '-':>7}")
    t = th.get("TEMPERATURE", {})
    print(f"  t  {'TEMPERATURE':<12}{_fmt(t.get('warn')):>13}{_fmt(t.get('alarm')):>15}   (degC)")
    h = th.get("HOLD_S", {})
    print(f"  h  {'HOLD_S':<12}{'current ' + _fmt(h.get('current')):>13}{'temp ' + _fmt(h.get('temp')):>15}   (seconds over a limit before it counts)")


def _test(th, rows):
    """What would these thresholds have reported on the recording?"""
    if not rows:
        print("  no recording to test on (start with  --calibrate --ref riff_raw.csv)")
        return
    eng, cond = BeltEngine(), Conditions(_convert(th))
    evs = []
    for t, vel, cur, pos, temp, _ in rows:
        eng.on_sample(t, vel, cur, pos, temp)
        evs += cond.update(t, eng.state, cur, eng.temp)
    n = Counter((e["metric"], e["state"], e["level"]) for e in evs if e["level"] != "OK")
    print(f"  on the recording ({rows[-1][0] - rows[0][0]:.0f} s): {sum(n.values())} WARN/ALARM events")
    for (m, s, lv), c in sorted(n.items()):
        print(f"    {m:<8}{s:<10}{lv:<6} x{c}")
    if not n:
        print("    none: nothing would have fired")


def run_calibrate(th_path, ref_path):
    th, used = load_file_schema(th_path)
    ref, rows = {}, []
    if ref_path:
        ref, _tmax, rows = reference_stats(ref_path)
    print(f"[calibrate] {th_path}  ({'loaded' if used else 'not found: starting from the defaults'})")
    print("            recorded data: " + (os.path.basename(ref_path) if ref_path else "none (add --ref riff_raw.csv to see p99/max next to each limit)"))
    help_text = (
        "\ncommands:\n"
        "  <#> <warn> <alarm>     set both limits of a state          e.g.  3 360 450\n"
        "  <#> w <warn>           change only warn                    e.g.  3 w 360\n"
        "  <#> a <alarm>          change only alarm                   e.g.  3 a 500\n"
        "  t <warn> <alarm>       temperature limits                  e.g.  t 40 55\n"
        "  h <current_s> <temp_s> hold times in seconds               e.g.  h 1 5\n"
        "  test                   what would these limits have reported on the recording?\n"
        "  show                   print the table again       reset   back to the defaults\n"
        "  save                   write the file              q       quit (asks if unsaved)\n")
    print(help_text)
    saved = json.dumps(th, sort_keys=True)
    _table(th, ref)
    while True:
        try:
            line = input("\ncalibrate> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            line = "q"
        p = line.split()
        if not p:
            continue
        try:
            if p[0] in ("q", "quit", "exit"):
                if json.dumps(th, sort_keys=True) != saved and input("unsaved changes - quit anyway? (y/N) ").strip().lower() != "y":
                    continue
                return
            elif p[0] in ("show", "s"):
                _table(th, ref)
            elif p[0] in ("help", "?"):
                print(help_text)
            elif p[0] == "reset":
                th = json.loads(json.dumps(DEFAULT_THRESHOLDS))
                _table(th, ref)
            elif p[0] == "test":
                _test(th, rows)
            elif p[0] == "save":
                with open(th_path, "w") as f:
                    json.dump(th, f, indent=2)
                saved = json.dumps(th, sort_keys=True)
                print(f"  saved {th_path}  (a running node reloads it within 5 s)")
            elif p[0] == "h":
                th["HOLD_S"] = {"current": float(p[1]), "temp": float(p[2])}
                if min(th["HOLD_S"].values()) < 0:
                    raise ValueError("hold times cannot be negative")
                _table(th, ref)
            elif p[0] == "t":
                w, a = float(p[1]), float(p[2])
                if not 0 < w < a:
                    raise ValueError("need 0 < warn < alarm")
                th["TEMPERATURE"] = {"warn": w, "alarm": a}
                _table(th, ref)
            elif p[0].isdigit() and 1 <= int(p[0]) <= len(COUNTED_STATES):
                st = COUNTED_STATES[int(p[0]) - 1]
                row = dict(th.get(st, {}))
                if p[1] == "w":
                    row["warn_current"] = float(p[2])
                elif p[1] == "a":
                    row["alarm_current"] = float(p[2])
                else:
                    row["warn_current"], row["alarm_current"] = float(p[1]), float(p[2])
                if not 0 < row["warn_current"] < row["alarm_current"]:
                    raise ValueError("need 0 < warn < alarm")
                th[st] = {k: (int(v) if float(v).is_integer() else v) for k, v in row.items()}
                _table(th, ref)
            else:
                print("  not a command: type  ?  for help")
        except (IndexError, ValueError) as e:
            print(f"  not changed: {e if str(e) else 'check the command, type  ?  for help'}")


# ============================================================================
# 8. --show / --write-thresholds / MAIN
# ============================================================================
def show(db_path, n=12):
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)         # read-only: safe while the node runs
    print(f"== last {n} transitions ==")
    for iso, a, b, detail, dwell in db.execute(
            "SELECT iso,from_state,to_state,detail,dwell_s FROM transitions ORDER BY id DESC LIMIT ?", (n,)):
        print(f"{iso[11:23]}  {a:<9}-> {b:<9} after {dwell if dwell is not None else '-'} s   {detail}")
    print(f"\n== last {n} runs ==")
    for iso, sec, res, dist, ci, why in db.execute(
            "SELECT start_iso,seconds,result,distance_rev,steady_current_mA,reasons FROM runs ORDER BY id DESC LIMIT ?", (n,)):
        print(f"{iso[11:19]}  {sec:6.2f} s  {res:<8} {dist} rev  {ci} mA  {why}")
    print(f"\n== last {n} conditions ==")
    for iso, m, s, lv, pv, val, thr in db.execute(
            "SELECT iso,metric,state,level,prev,value,threshold FROM conditions ORDER BY id DESC LIMIT ?", (n,)):
        print(f"{iso[11:23]}  {m:<8} {s:<9} {pv} -> {lv}   value {val} (limit {thr})")


def main():
    p = argparse.ArgumentParser(description="PRISM belt analyzer v0: conveyor state, runs and conditions")
    p.add_argument("--db", default=DB_FILE, help="SQLite file")
    p.add_argument("--thresholds", default=THRESHOLD_FILE, help="threshold JSON file")
    p.add_argument("--write-thresholds", action="store_true", help="write the default threshold file to edit, then exit")
    p.add_argument("--calibrate", action="store_true", help="interactive: view / change / test / save the thresholds")
    p.add_argument("--ref", metavar="RIFF_RAW_CSV", help="with --calibrate: a recording to show p99/max and to test on")
    p.add_argument("--replay", metavar="RIFF_RAW_CSV", help="offline: run a recording through the engine")
    p.add_argument("--write-db", action="store_true", help="with --replay: also write the db")
    p.add_argument("--verbose", action="store_true", help="with --replay: print every transition")
    p.add_argument("--play", metavar="RIFF_RAW_CSV", help="publish a recording to belt/telemetry")
    p.add_argument("--speed", type=float, default=1.0, help="with --play: playback speed factor")
    p.add_argument("--show", action="store_true", help="print recent transitions / runs / conditions and exit")
    a = p.parse_args()
    if a.write_thresholds:
        if os.path.exists(a.thresholds):
            raise SystemExit(f"{a.thresholds} already exists - not overwriting it")
        with open(a.thresholds, "w") as f:
            json.dump(DEFAULT_THRESHOLDS, f, indent=2)
        print(f"wrote {a.thresholds}. Edit it; a running node reloads it within 5 s.")
    elif a.calibrate:
        run_calibrate(a.thresholds, a.ref)
    elif a.show:
        show(a.db)
    elif a.replay:
        run_replay(a.replay, a.thresholds, a.db if a.write_db else None, a.verbose)
    elif a.play:
        run_play(a.play, a.speed)
    else:
        run_live(a.db, a.thresholds)


if __name__ == "__main__":
    sys.exit(main())
