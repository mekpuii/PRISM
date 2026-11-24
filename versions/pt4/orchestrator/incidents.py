# incidents.py - respond to incidents by itself instead of stopping on every one (PRISM orchestrator).
#
# WHAT THIS MODULE DOES
#   A cycle ends abnormally (the IR saw the cube but the grip missed, a belt fault, a stuck cube ...). Before this,
#   optimize.py paused in ALERT on every one and waited for a person. Now it answers with a RESPONSE from a short,
#   safe menu, checks whether the next cycle works, and LEARNS which response works for which incident:
#     retry    pause, wait a moment, run again unchanged (autoen already opened the gripper and went home)
#     gentle   pause, switch to the slowest safe setting for GENTLE_CYCLES cycles, run again (a fast approach can knock
#              or turn the cube; a slower belt keeps the next cube from pushing into the one being gripped)
#   One incident = one EPISODE: each response is tried at most once, best-known first; the first OK cycle ends it
#   (that response gets the credit). Nothing worked -> ALERT, exactly as before: a person decides.
#
# WHAT IT NEVER DOES ON ITS OWN  (safety first)
#   - arm move timeouts (the arm did not reach a pose: something may be in the way), belt over-temperature, a failed
#     recovery inside autoen: ALERT at once, no try
#   - more than MAX_AUTO responses in WINDOW_S: ALERT (something is wrong that retrying will not fix)
#   - it never invents moves: every response is a pause, a TESTED setting from the grid, and a resume
#   A NEW incident it has never seen (an unknown result code) gets one plain retry, and is remembered.
#
# LEARNING
#   incidents_state.json: for every (incident, response) how often it was tried and how often the next cycle was OK.
#   The order of the menu is the learned success rate (untried = 0.5), so the cell finds by itself which answer
#   works for which incident on this machine. report / dashboard: Responder.summary().

import json
import os
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(HERE, "incidents_state.json")

RESPONSES = ("retry", "gentle")
GENTLE_CYCLES = 5               # cycles on the slowest safe setting after a "gentle" response
MAX_AUTO = 6                    # automatic responses allowed in WINDOW_S ...
WINDOW_S = 1800.0               # ... (30 min); more = ALERT
MAX_FAILS_AUTOEN = 8            # autoen ends the run after max_fails failed cycles in a row (default 3). An episode
                                # needs 1 failure + one try per response, and feeds that bring no cube count as
                                # failures in autoen too (2026-11-10: 3 empty feeds + 1 grip failure ended the run).
                                # The optimizer pauses on every abnormal cycle itself; this is only the backstop.

# incident class -> responses it may get (in the default order; learning reorders them)
CLASSES = {
    "grip":        ("retry", "gentle"),   # GRIP_MISS, GRIP_FAILED, BLOCK_STILL_AT_IR, NO_BLOCK_AT_IR
    "grip_suspect": ("retry", "gentle"),  # carried, but probably not held
    "belt_fault":  ("retry",),            # FAULT_1 (watchdog) and other belt faults except FAULT_2
    "feed":        ("retry",),            # TIMEOUT_FEED: the feeder did not report done
    "stuck_cube":  ("retry", "gentle"),   # the camera saw a cube still on the running belt (scene.py)
}
GRIP = ("GRIP_MISS", "GRIP_FAILED", "BLOCK_STILL_AT_IR", "NO_BLOCK_AT_IR")
NEVER = ("FAULT_2",)                      # belt over-temperature: autoen stops itself


def classify(code, notes=""):
    """(class, auto_ok). auto_ok False = ALERT at once."""
    code = str(code)
    if "RECOVERY FAILED" in (notes or ""):
        return "recovery_failed", False
    if code in ("SerialException", "OSError") or "Input/output error" in (notes or ""):
        return "the USB link to the Mega is lost: check the cable / power, then restart the cell", False
    if code in GRIP:
        return "grip", True
    if code == "GRIP_SUSPECT":
        return "grip_suspect", True
    if code == "STUCK_CUBE":
        return "stuck_cube", True
    if code in NEVER:
        return code.lower(), False
    if code.startswith("FAULT_"):
        return "belt_fault", True
    if code == "TIMEOUT_FEED":
        return "feed", True
    if code.startswith("TIMEOUT_"):
        return "arm_timeout", False       # TIMEOUT_STANDBY, move timeouts: the arm may be blocked
    return "new:" + code, True            # never seen: one plain retry


class Responder:
    def __init__(self, path=STATE_FILE):
        self.path = path                  # None = learn for this run only (tests, shadow)
        self.stats = {}                   # "class|response" -> [tried, ok]
        self.episode = None               # {"cls", "tried": [...], "last": response, "t0"}
        self.recent = deque()             # times of automatic responses (rate limit)
        self.log = deque(maxlen=20)       # last decisions, for the plan / dashboard
        self._load()

    # ----- persistence -----

    def _load(self):
        if self.path and os.path.exists(self.path):
            try:
                with open(self.path, encoding="utf-8") as f:
                    self.stats = {k: list(v) for k, v in json.load(f).get("stats", {}).items()}
            except (OSError, ValueError):
                self.stats = {}

    def _save(self):
        if not self.path:
            return
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({"stats": self.stats, "saved": time.strftime("%Y-%m-%dT%H:%M:%S")}, f, indent=1)
            os.replace(tmp, self.path)
        except OSError:
            pass

    # ----- learning -----

    def rate(self, cls, resp):
        n, ok = self.stats.get(f"{cls}|{resp}", (0, 0))
        return (ok + 1) / (n + 2)         # untried = 0.5; 3 of 3 = 0.8; 0 of 3 = 0.2

    def _credit(self, resp, ok):
        e = self.episode
        s = self.stats.setdefault(f"{e['cls']}|{resp}", [0, 0])
        s[0] += 1
        s[1] += int(ok)
        self._save()

    def menu(self, cls):
        base = CLASSES.get(cls, ("retry",))
        return sorted(base, key=lambda r: (-self.rate(cls, r), base.index(r)))

    # ----- decisions -----

    def decide(self, code, notes="", now=None):
        """An abnormal cycle (or a stuck cube). Returns (response or None, why). None = ALERT."""
        now = time.monotonic() if now is None else now
        cls, auto_ok = classify(code, notes)
        if self.episode is not None and self.episode.get("last"):
            self._credit(self.episode["last"], False)     # the response before this did not help
        if not auto_ok:
            return self._end(None, f"{code}: never answered automatically ({cls}) - a person checks first")
        while self.recent and now - self.recent[0] > WINDOW_S:
            self.recent.popleft()
        if len(self.recent) >= MAX_AUTO:
            return self._end(None, f"{code}: {len(self.recent)} automatic responses in {WINDOW_S / 60:.0f} min "
                                   "- something retrying will not fix")
        if self.episode is None or self.episode["cls"] != cls:
            self.episode = {"cls": cls, "tried": [], "last": None, "t0": now}
        left = [r for r in self.menu(cls) if r not in self.episode["tried"]]
        if not left:
            tried = ", ".join(self.episode["tried"])
            return self._end(None, f"{code}: tried {tried}, still failing")
        resp = left[0]
        self.episode["tried"].append(resp)
        self.episode["last"] = resp
        self.recent.append(now)
        new = " (new incident: remembered)" if cls.startswith("new:") and not any(
            k.startswith(cls + "|") for k in self.stats) else ""
        why = (f"{code}: auto response '{resp}' (try {len(self.episode['tried'])}/{len(CLASSES.get(cls, ('retry',)))}, "
               f"learned success {self.rate(cls, resp):.0%}){new}")
        self.log.append({"t": round(now, 1), "code": code, "class": cls, "response": resp})
        return resp, why

    def ok_cycle(self):
        """A cycle ended OK: the response in force worked."""
        if self.episode is not None:
            if self.episode.get("last"):
                self._credit(self.episode["last"], True)
            self.episode = None

    def _end(self, resp, why):
        self.log.append({"t": round(time.monotonic(), 1), "code": why.split(":")[0], "response": "ALERT"})
        self.episode = None
        return resp, why

    def summary(self):
        return {"learned": {k: f"{ok}/{n}" for k, (n, ok) in sorted(self.stats.items())},
                "episode": None if self.episode is None else {"class": self.episode["cls"],
                                                              "tried": list(self.episode["tried"])},
                "auto_last_30min": len(self.recent)}
