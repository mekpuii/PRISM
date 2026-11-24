# guardian.py - the FAULT detector (2026-11-13, the operator: "if the camera is covered, the feeder lost its connection ...
# stop everything, not as shutdown but paused after the best checkpoint; the Pi never stops").
# Pure logic: no I/O, no clock of its own. optimize.py feeds it the world every tick and acts on what it reports.
#   camera_lost    no camera frames for CAMERA_LOST_S (after frames were seen in this run of the program)
#   camera_covered the belt is dark / black for COVER_S (lens covered or blocked)
#   foreign        something big that is not in the empty-workspace reference photo (a hand), only while the arm is parked
#   feeder_link    the Mega (belt + feeder board) sends nothing for LINK_LOST_S
#   arm_link       the arm servos send nothing for LINK_LOST_S
#   feeder         the feeder servo does not move (optimize.py's push check, passed in as `feeder_dead`)
# A condition must hold for its `hold` time to become a fault (no flicker) and be gone for its `clear` time to end it.
from limits import (CAMERA_LOST_S, CLEAR_S, COVER_DARK_PCT, COVER_S, COVER_V_MEAN, LINK_LOST_S, OBJECT_CLEAR_S,
                    OBJECT_PCT, OBJECT_S)

PARKED_PHASES = ("IDLE", "TRANSPORT", "STANDBY", "")      # the arm sits where the reference photo was taken

TEXT = {
    "camera_lost": "the camera sends no pictures (unplugged, crashed or frozen)",
    "camera_covered": "the camera is covered or its lens is blocked (the belt picture is black)",
    "foreign": "something big is in the workspace (a hand or an object)",
    "feeder_link": "the belt / feeder board (Mega) sends nothing (USB cable or power?)",
    "arm_link": "the arm servos send nothing (bus cable or power?)",
    "feeder": "the feeder servo does not move (no current on the 12 V rail)",
}
HOLD = {"camera_lost": CAMERA_LOST_S, "camera_covered": COVER_S, "foreign": OBJECT_S, "feeder_link": LINK_LOST_S,
        "arm_link": LINK_LOST_S, "feeder": 0.0}
CLEAR = {"camera_lost": CLEAR_S, "camera_covered": 5.0, "foreign": OBJECT_CLEAR_S, "feeder_link": CLEAR_S,
         "arm_link": CLEAR_S, "feeder": 0.0}                # (the feeder clears when a retry push draws current)
# what to try again at a checkpoint: "open those or send the signal again"
RETRY = {"camera_lost": "restart the camera program", "camera_covered": "look at the picture again",
         "foreign": "look at the picture again", "feeder_link": "check the Mega link",
         "arm_link": "check the arm link", "feeder": "one push, then roll the belt a little"}


class Guardian:
    def __init__(self):
        self.seen = {"camera": False, "belt": False, "arm": False}   # was this ever fresh? (blind tests / start-up are no fault)
        self._since = {}            # code -> when its condition started to hold
        self._gone = {}             # code -> when its condition stopped holding (active faults only)
        self.active = {}            # code -> text

    def conditions(self, state, phase, feeder_dead):
        """Which conditions hold right now? {code: True}"""
        self.seen["camera"] |= bool(state.camera_fresh)
        self.seen["belt"] |= bool(state.belt_fresh)
        self.seen["arm"] |= bool(state.joints_fresh)
        c = {}
        if self.seen["camera"] and not state.camera_fresh:
            c["camera_lost"] = True
        lt = state.light or {}
        if state.camera_fresh and lt:
            v, d = lt.get("v_mean"), lt.get("dark_pct")
            if (v is not None and v < COVER_V_MEAN) or (d is not None and d >= COVER_DARK_PCT):
                c["camera_covered"] = True
            if lt.get("ref") and (lt.get("obj_pct") or 0.0) >= OBJECT_PCT and phase in PARKED_PHASES:
                c["foreign"] = True
        if self.seen["belt"] and not state.belt_fresh:
            c["feeder_link"] = True
        if self.seen["arm"] and not state.joints_fresh:
            c["arm_link"] = True
        if feeder_dead:
            c["feeder"] = True
        return c

    def update(self, state, phase, now, feeder_dead=False, enabled=True):
        """One tick. Returns the dict of ACTIVE faults (code -> text). Disabled (asleep, waking, relay off): everything resets."""
        if not enabled:
            self._since.clear(), self._gone.clear(), self.active.clear()
            return self.active
        cond = self.conditions(state, phase, feeder_dead)
        for code in list(self._since):
            if code not in cond:
                del self._since[code]
        for code in cond:
            self._since.setdefault(code, now)
            if code not in self.active and now - self._since[code] >= HOLD[code]:
                self.active[code] = TEXT[code]
                self._gone.pop(code, None)
            elif code in self.active:
                self._gone.pop(code, None)
        for code in list(self.active):
            if code not in cond:
                self._gone.setdefault(code, now)
                if now - self._gone[code] >= CLEAR[code]:
                    del self.active[code]
                    del self._gone[code]
        return self.active

    def resolve(self, code):
        """The cause is known to be fixed (e.g. a retry push drew current): end that fault now."""
        self.active.pop(code, None)
        self._since.pop(code, None)
        self._gone.pop(code, None)
