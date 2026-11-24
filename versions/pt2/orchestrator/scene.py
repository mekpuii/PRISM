# scene.py - what the camera says about the SITUATION, not only "there is a cube" (PRISM orchestrator).
#
# WHAT THIS MODULE DOES
#   Every tick it reads the camera (observe.WorldState) and the belt (predict.PredictOutput) and keeps four live
#   judgements for optimize.py:
#     light    ok / dim / dark / glare / unknown   from the light on the belt that vision measures (scene_stats in
#              vision_live_tracks.py). dark or glare = the colours cannot be trusted: the camera is not used for
#              decisions until the light is ok again. dim = a warning only.
#     stuck    a cube that does not move while the belt runs (TRANSPORT) and is not at the IR yet: jammed on the rail,
#              at the feeder exit or in front of the IR. optimize.py brakes the belt and raises ALERT at once instead of
#              running the belt the whole arrive_timeout (the operator's rule: stop on a cube stuck at the IR).
#     backlog  cubes waiting on the belt (seen by the camera, before the IR). Cubes keep coming faster than the arm
#              takes them -> optimize.py picks the fastest safe setting until the belt is clear again ("rush").
#     train    two cubes touching or nearly (free space < TRAIN_MM): the arm may grab both, the colour may mix.
#   and one more measurement: slip = how fast the cubes really move / how fast the belt says it runs.
#
# THE CASE ARRAY (for study, and later for learning)
#   Each finished cycle becomes ONE ROW in orchestrator/cases/cases_<day>.csv: the settings, the result, and what the
#   camera saw during that cycle (light, backlog, closest gap, stuck, slip, colour certainty ...). Every row is one
#   case; the file is a plain table (numpy.loadtxt / pandas.read_csv / a spreadsheet read it as an array).
#   study_cases.py adds the rows up per situation: which situations fail, and which settings work in which situation.
#
# THRESHOLDS  (measured on the real belt, vision/snap_raw.png 2026-10-25, ROI only)
#   normal light: V 75, 0.1 % too dark, 0 % burnt out.  Lamp dimmed to 60 %: V 45, 67 % too dark.
#   Glare (x1.8 + 40): V 163, 12 % burnt out.
#
# It never acts by itself: optimize.py decides, actuate.py sends.

import csv
import os
import statistics
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
CASES_DIR = os.path.join(HERE, "cases")

# light (vision's scene_stats, inside the belt ROI)
BASE_V = 75.0               # normal brightness of the belt (measured); adapts slowly while the light is ok
BASE_ALPHA = 0.01           # per camera snapshot (2 Hz): the baseline follows a slow change in ~1 min
DIM_RATIO = 0.65            # V below 65 % of normal = dim (warning only)
DARK_RATIO = 0.45           # V below 45 % of normal = dark
DARK_PCT = 30.0             # or this much of the belt too dark to see a colour
GLARE_PCT = 8.0             # this much of the belt burnt out = glare (a white cube is ~2 %)
GLARE_RATIO = 1.8           # or V this far above normal
LIGHT_STABLE_S = 3.0        # a light verdict must hold this long before it counts (a hand, a shadow passing)

# belt geometry: camgeo.py (positions as "pixels upstream of the IR", any camera direction). Without
# camera_geometry.json these are the D415 numbers of before: IR u 245, feeder end u 600, 0.95 mm/px.
import camgeo                                                        # noqa: E402
IR_U = 245.0                # (D415 picture, kept for reading old notes; the rules use camgeo.dist)
AT_IR_PX = 45.0 * camgeo.SCALE   # a cube this close to the IR is "at the IR" (the belt stops for it): never "stuck"
ENTRY_U = 600.0             # (D415 picture)
MIN_HITS = 3                # camera frames before a track counts
MM_PER_PX = camgeo.MM_PER_PX     # = predict.MM_PER_PX (do not touch: changed only by camera_geometry.json)
CUBE_MM = 40.0              # = predict.CUBE_EDGE_MM

STUCK_S = 4.0               # still this long while the belt runs = stuck (a cube rides 20 rpm = 56 mm/s)
STUCK_PX = 15.0 * camgeo.SCALE   # "still" = moved less than this
BELT_MOVING_MM_S = 10.0     # the belt counts as running above this
TRAIN_MM = 20.0             # free space between two cubes below half a cube = a train
BACKLOG_RUSH = 2            # this many cubes waiting on the belt during a whole cycle = cubes come faster than the arm
SLIP_MIN_V_MM_S = 10.0      # cube speeds below this say nothing about slip

COLUMNS = ["iso", "n", "result", "secs", "rpm", "arm_speed", "mode", "drop", "colour", "colour_share",
           "light", "v_mean", "s_mean", "glare_pct", "dark_pct", "light_bad_s",
           "backlog_max", "train", "min_gap_mm", "stuck_s", "slip", "cube_seen", "camera_fresh_pct"]


class Scene:
    def __init__(self, cases_dir=CASES_DIR, record=True):
        self.cases_dir, self.record = cases_dir, record
        self.base_v = BASE_V
        self.light = "unknown"           # the verdict in force (held LIGHT_STABLE_S)
        self._raw, self._raw_t = "unknown", None
        self.lux = {}                    # latest scene_stats from vision
        self.stuck = None                # {"id", "u", "secs"} or None
        self.backlog = 0
        self.train = False
        self.min_gap_mm = None
        self._still = {}                 # track id -> (u when it stopped moving, time)
        self._seq = None
        self.rows = 0
        self._new_cycle()

    # ----- live -----

    def step(self, state, pred, phase, now):
        """One tick. phase = autoen's phase ("TRANSPORT" while the belt carries the cube to the IR)."""
        self._light(state, now)
        belt_runs = bool(pred is not None and pred.belt_ok and abs(pred.belt_mm_s) >= BELT_MOVING_MM_S)
        self._c["ticks"] += 1
        self._c["fresh"] += state.camera_fresh
        if self.light in ("dark", "glare"):
            self._c["light_bad_s"] += 0.05
        if not state.camera_fresh or state.cam_seq == self._seq:
            if not state.camera_fresh:
                self.stuck, self._still, self.backlog, self.train, self.min_gap_mm = None, {}, 0, False, None
            return
        self._seq = state.cam_seq
        on_belt = sorted((camgeo.dist(c.along_px), c) for c in state.cubes      # (px upstream of the IR, cube)
                         if c.hits >= MIN_HITS and -AT_IR_PX <= camgeo.dist(c.along_px) <= camgeo.ENTRY_DIST)
        us = [u for u, _ in on_belt]
        self.backlog = sum(u > AT_IR_PX for u in us)                 # waiting, not the one at the IR
        gaps = [(b - a) * MM_PER_PX - CUBE_MM for a, b in zip(us, us[1:])]
        self.min_gap_mm = round(min(gaps), 1) if gaps else None
        self.train = bool(gaps) and min(gaps) < TRAIN_MM
        self._stuck(on_belt, phase == "TRANSPORT" and belt_runs, now)
        if phase == "TRANSPORT" and belt_runs:
            for _, c in on_belt:
                v = abs(c.along_vel_px_s) * MM_PER_PX
                if v >= SLIP_MIN_V_MM_S:
                    self._c["slip"].append(v / abs(pred.belt_mm_s))
        c = self._c
        c["backlog_max"] = max(c["backlog_max"], self.backlog)
        c["train"] = c["train"] or self.train
        if self.min_gap_mm is not None:
            c["min_gap_mm"] = self.min_gap_mm if c["min_gap_mm"] is None else min(c["min_gap_mm"], self.min_gap_mm)
        if self.stuck:
            c["stuck_s"] = max(c["stuck_s"], self.stuck["secs"])

    def _light(self, state, now):
        lux = state.light if state.camera_fresh else None
        if lux and lux.get("v_mean") is not None:
            self.lux = lux
            v = float(lux["v_mean"])
            if (lux.get("glare_pct") or 0) >= GLARE_PCT or v >= GLARE_RATIO * self.base_v:
                raw = "glare"
            elif (lux.get("dark_pct") or 0) >= DARK_PCT or v < DARK_RATIO * self.base_v:
                raw = "dark"
            elif v < DIM_RATIO * self.base_v:
                raw = "dim"
            else:
                raw = "ok"
        else:
            raw = "unknown"                 # camera off, or a vision without scene_stats
        if raw != self._raw:
            self._raw, self._raw_t = raw, now
        if raw == "unknown" or now - self._raw_t >= LIGHT_STABLE_S:
            self.light = raw
        if self.light == "ok" and state.cam_seq != self._seq and lux:
            self.base_v += BASE_ALPHA * (float(lux["v_mean"]) - self.base_v)

    def _stuck(self, on_belt, belt_runs, now):
        if not belt_runs:
            self._still, self.stuck = {}, None
            return
        worst = None
        seen = set()
        for u, c in on_belt:
            seen.add(c.id)
            u0, t0 = self._still.get(c.id, (u, now))
            if abs(u - u0) > STUCK_PX:
                u0, t0 = u, now                       # it moved: start again
            self._still[c.id] = (u0, t0)
            if u > AT_IR_PX and now - t0 >= STUCK_S and (worst is None or now - t0 > worst["secs"]):
                worst = {"id": c.id, "u": round(u), "secs": round(now - t0, 1)}
        self._still = {k: v for k, v in self._still.items() if k in seen}
        self.stuck = worst

    @property
    def light_bad(self):
        return self.light in ("dark", "glare")

    def summary(self):
        """For the plan score / dashboard."""
        return {"light": self.light, **{k: self.lux.get(k) for k in ("v_mean", "glare_pct", "dark_pct") if self.lux},
                "base_v": round(self.base_v, 1), "backlog": self.backlog, "train": self.train,
                "min_gap_mm": self.min_gap_mm, "stuck": self.stuck, "cases": self.rows}

    # ----- one row per cycle -----

    def _new_cycle(self):
        self._c = {"ticks": 0, "fresh": 0, "light_bad_s": 0.0, "backlog_max": 0, "train": False, "min_gap_mm": None,
                   "stuck_s": 0.0, "slip": []}

    def end_cycle(self, cyc, cfg, mode, colour=None, colour_share=None, cube_seen=None):
        """A cycle finished: write its row (the case) and start the next. Returns the row."""
        c = self._c
        row = {"iso": datetime.now().isoformat(timespec="seconds"), "n": cyc.get("n"), "result": cyc.get("result"),
               "secs": cyc.get("secs"), "rpm": cfg.get("rpm"), "arm_speed": cfg.get("arm_speed"), "mode": mode,
               "drop": cyc.get("drop"), "colour": colour,
               "colour_share": None if colour_share is None else round(colour_share, 2),
               "light": self.light, **{k: self.lux.get(k) for k in ("v_mean", "s_mean", "glare_pct", "dark_pct")},
               "light_bad_s": round(c["light_bad_s"], 1), "backlog_max": c["backlog_max"], "train": int(c["train"]),
               "min_gap_mm": c["min_gap_mm"], "stuck_s": c["stuck_s"],
               "slip": round(statistics.median(c["slip"]), 2) if c["slip"] else None,
               "cube_seen": None if cube_seen is None else int(cube_seen),
               "camera_fresh_pct": round(100.0 * c["fresh"] / c["ticks"]) if c["ticks"] else None}
        self._new_cycle()
        if self.record:
            self._write(row)
        return row

    def _write(self, row):
        try:
            os.makedirs(self.cases_dir, exist_ok=True)
            path = os.path.join(self.cases_dir, f"cases_{time.strftime('%Y%m%d')}.csv")
            new = not os.path.exists(path)
            with open(path, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, COLUMNS)
                if new:
                    w.writeheader()
                w.writerow({k: "" if row.get(k) is None else row[k] for k in COLUMNS})
            self.rows += 1
        except OSError as e:                        # a full disk must never stop the cell
            print(f"[scene] could not write the case row: {e}", flush=True)
