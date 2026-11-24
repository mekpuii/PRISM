"""
DELTA - Object Perception (selectable colours) + Spacing + Demand Data Collection
---------------------------------------------------------------------------------
Hardware:
    - Intel RealSense D415
    - Raspberry Pi 4

VISION v3 (this version) - what is new compared with v2
    - YOU CHOOSE THE COLOURS:   python 3mins.py --colors white,red,blue
      Presets: white black red orange yellow green cyan blue purple (see --list-colors).  Default = white (v2 behaviour).
      Each colour's HSV range can be re-measured on the real cubes with  --calibrate  (right-click a cube to sample it).
    - SPACING BETWEEN PIECES:  for every pair of neighbouring cubes along the belt the node publishes the gap in
        pixels, in cm (3-D, using the depth camera) and in "pieces" (gap / cube size), plus how fast the gap opens/closes.
      The positions come from the per-cube Kalman filter, so one noisy frame does not move the gap.
      The belt direction is a calibration item:  --belt-axis u|v  --belt-dir 1|-1  (or A / D in --calibrate).
    - Counting is per colour:  camera/demand and the CSVs keep `count` = TOTAL (so L4 and the orchestrator are unchanged)
      and add `by_color`.

v2 features kept:
    - Conveyor ROI (calibrated polygon), one cube = one track = one count, tracking without depth,
      no double count when a track is lost and reappears, last partial 10 s sample is published.
    - 3-minute collection windows, 10-second demand samples, CSV history, automatic summaries.

CSV files:
    delta_demand_history.csv   time-series every 10 s     (+ by_color column: JSON)
    delta_3min_summary.csv     one row per 3-minute window (+ by_color column: JSON)
    !! the first run of v3 moves your old CSVs aside (new header) instead of mixing two layouts.

MQTT (camera/perception now also carries colour, size, velocity and the gaps; old fields are unchanged):
    {"ts","fps","active","axis","dir","colors":[...],
     "tracks":[{"id","u","v","dist_m","color","w","h","vu","vv"}],
     "gaps":[{"ahead":id,"behind":id,"gap_px","gap_cm","edge_cm","pieces","rate_px_s","rate_cm_s","depth":"measured|belt_assumed|none"}]}
    gap_cm  = 3-D distance between the two cube CENTRES;   edge_cm = free space between the cubes' edges along the belt
    pieces  = edge_cm / cube size (1.0 = one cube-width of free space).  cm values are None if there is no usable depth.

Requirements:
    - Python 3.11, numpy, opencv-python, paho-mqtt, pyrealsense2 (only needed to run the camera, not to import this file)

Run from the prism-env environment:
    prism        (activates ~/prism-env)
    cd DCAM && python 3mins.py --calibrate --colors white,red,blue      # once per setup
    cd DCAM && python 3mins.py --colors white,red,blue
"""

import cv2
import numpy as np
try:
    import pyrealsense2 as rs
except ImportError:                      # lets the file be imported / tested on a machine without the camera SDK
    rs = None
import math
import time
import traceback
import csv
import os
import json
import argparse
from collections import Counter
from datetime import datetime

import paho.mqtt.client as mqtt          # pip install paho-mqtt


# ============================================================
# CONFIGURATION
# ============================================================
 
WIDTH = 640
HEIGHT = 480
 
# The standalone D415 test succeeded at 6 FPS for both streams.
FPS = 6
 
# Frame timeout in milliseconds.
FRAME_TIMEOUT_MS = 15000
 
# Number of initial frames to discard after starting the camera.
WARMUP_FRAMES = 3
 
MIN_AREA = 500
MAX_AREA = 200000
 
MAX_MISSED_FRAMES = 12
MAX_MATCH_DISTANCE = 90
 
DEPTH_SAMPLE_RADIUS = 5
MIN_VALID_DEPTH = 0.15
MAX_VALID_DEPTH = 5.0
 
WINDOW_NAME = "DELTA - Object Detection & Tracking"
SHOW_MASKS = False


# ============================================================
# VISION v2: ROI / COUNTING CONFIGURATION
# ============================================================

# Written by `python 3mins.py --calibrate`, read at start-up.
CALIB_FILE = "vision_calibration.json"

# "confirm": count a track once it has been matched CONFIRM_FRAMES times inside the ROI.
#            Works wherever cubes appear (e.g. dropped onto the belt inside the view).
# "line":    count a track when it crosses the count line (needs the line in the calibration file).
COUNT_MODE = "confirm"

# A blob must survive this many frames before it may be counted. 6 frames = 1 s at 6 FPS.
# This is what rejects one-frame flicker blobs (glare, hands, shadows).
CONFIRM_FRAMES = 6

# A counted track that vanishes and a NEW track that appears within this time AND distance
# is treated as the same cube (inherits counted=True) instead of being counted again.
DEDUP_SECONDS = 3.0
DEDUP_DISTANCE = 60
 
 
# ============================================================
# DEMAND DATA COLLECTION CONFIGURATION
# ============================================================
 
# One complete demand-collection window.
DATA_COLLECTION_SECONDS = 180
 
# How often to save a time-series sample.
DATA_SAMPLE_INTERVAL = 10
 
# CSV files.
HISTORY_CSV = "delta_demand_history.csv"
SUMMARY_CSV = "delta_3min_summary.csv"
 
 
# ============================================================
# MQTT CONFIGURATION
# ============================================================
 
# Broker address. localhost = broker on this Pi; otherwise set the
# MQTT_HOST environment variable, e.g.  MQTT_HOST=192.168.1.20 python 3mins.py
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = 1883
 
# Live perception snapshot (active tracks) is published this often.
# The camera only runs at 6 FPS, so 2 Hz is plenty and keeps traffic small.
PERCEPTION_PUBLISH_INTERVAL = 0.5
 

# ============================================================
# COLOUR CONFIGURATION  (v3)
# ============================================================

# OpenCV HSV: H 0..179, S 0..255, V 0..255.
# A colour is a LIST of (lower, upper) ranges.  Red needs two because hue wraps around 0/179.
# These are STARTING POINTS from theory: re-measure them on your cubes under your lamp with --calibrate.
#   white  = ANY hue, LOW saturation, HIGH brightness     (tune S max / V min)
#   black  = LOW brightness: shadows look the same, so keep it OFF unless the belt is not dark
COLOR_PRESETS = {
    "white":  [((0, 0, 180),     (179, 50, 255))],
    "black":  [((0, 0, 0),       (179, 255, 50))],
    "red":    [((0, 110, 70),    (8, 255, 255)), ((172, 110, 70), (179, 255, 255))],
    "orange": [((9, 120, 90),    (21, 255, 255))],
    "yellow": [((22, 100, 100),  (34, 255, 255))],
    "green":  [((40, 70, 60),    (85, 255, 255))],
    "cyan":   [((86, 80, 60),    (99, 255, 255))],
    "blue":   [((100, 100, 50),  (130, 255, 255))],
    "purple": [((131, 60, 50),   (160, 255, 255))],
}

# Overlay colours (BGR) used on the video.  White gets cyan because a white box would vanish on a white cube.
COLOR_BGR = {
    "white": (255, 255, 0), "black": (60, 60, 60), "red": (0, 0, 255), "orange": (0, 140, 255),
    "yellow": (0, 255, 255), "green": (0, 200, 0), "cyan": (255, 200, 0), "blue": (255, 80, 0),
    "purple": (200, 0, 160),
}
BOX_COLOR = (255, 255, 0)            # fallback

MORPH_KERNEL = np.ones((5, 5), np.uint8)

# ---- belt geometry (which camera axis runs along the belt) - set with --belt-axis/--belt-dir or in --calibrate
DEFAULT_BELT_AXIS = "u"              # "u" = image x, "v" = image y
DEFAULT_BELT_DIR = 1                 # +1: the coordinate GROWS in the direction of travel, -1: it shrinks
GAP_MIN_HITS = CONFIRM_FRAMES        # only confirmed cubes take part in the spacing
GAP_MAX_MISSED = 2                   # ... and only if seen in the last 2 frames (no ghosts from predictions)


def _validate_range(name, lo, hi):
    lo, hi = [int(x) for x in lo], [int(x) for x in hi]
    if len(lo) != 3 or len(hi) != 3:
        raise ValueError(f"{name}: a range needs [H,S,V] twice")
    for i, top in enumerate((179, 255, 255)):
        if not (0 <= lo[i] <= hi[i] <= top):
            raise ValueError(f"{name}: lower must be <= upper and inside 0..{top} (got {lo} .. {hi})")
    return np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8)


class ColorSet:
    """The colours to detect, in PRIORITY order (a pixel that fits two colours goes to the first one)."""

    def __init__(self, names, overrides=None):
        names = list(dict.fromkeys(n.strip().lower() for n in names if n.strip()))
        if not names:
            raise ValueError("no colours selected")
        for n in names:
            if n not in COLOR_PRESETS:
                raise ValueError(f"unknown colour '{n}'. Choose from: {', '.join(COLOR_PRESETS)}")
        self.names = names
        self.ranges = {n: [_validate_range(n, lo, hi) for lo, hi in COLOR_PRESETS[n]] for n in names}
        for n, rngs in (overrides or {}).items():
            if n in self.ranges:
                self.ranges[n] = [_validate_range(n, lo, hi) for lo, hi in rngs]

    def set_single(self, name, h0, h1, s0, s1, v0, v1):
        """One H/S/V box.  h0 > h1 means the hue wraps (red): it becomes two ranges."""
        if h0 <= h1:
            self.ranges[name] = [_validate_range(name, (h0, s0, v0), (h1, s1, v1))]
        else:
            self.ranges[name] = [_validate_range(name, (0, s0, v0), (h1, s1, v1)),
                                 _validate_range(name, (h0, s0, v0), (179, s1, v1))]

    def as_sliders(self, name):
        """-> (h0, h1, s0, s1, v0, v1) for the calibration sliders (two ranges = wrapped hue)."""
        r = self.ranges[name]
        lo, hi = r[0]
        if len(r) == 1:
            return int(lo[0]), int(hi[0]), int(lo[1]), int(hi[1]), int(lo[2]), int(hi[2])
        lo2, _ = r[1]
        return int(lo2[0]), int(hi[0]), int(lo[1]), int(hi[1]), int(lo[2]), int(hi[2])

    def to_json(self):
        return {n: [[lo.tolist(), hi.tolist()] for lo, hi in self.ranges[n]] for n in self.names}


def sample_to_range(h, s, v):
    """Turn one clicked pixel (median HSV of a small patch) into a sensible single H/S/V box."""
    if s < 50:                                        # greyish / white: any hue, low saturation
        return 0, 179, 0, int(min(255, s + 40)), int(max(0, v - 50)), 255
    h0, h1 = h - 10, h + 10
    if h0 < 0:
        h0 += 180
    if h1 > 179:
        h1 -= 180
    return int(h0), int(h1), int(max(0, s - 70)), 255, int(max(0, v - 80)), 255


# ============================================================
# CSV INITIALIZATION
# ============================================================

HISTORY_HEADER = [
    "timestamp", "window_id", "elapsed_seconds", "interval_seconds",
    "count", "cumulative", "by_color",
]
SUMMARY_HEADER = [
    "window_id", "window_start", "window_end", "duration_seconds",
    "total_objects", "by_color",
]


def _ensure_csv(path, header):
    """
    Create the CSV if missing. If an old file has a DIFFERENT header (an older layout), move it aside
    instead of mixing two formats.
    """

    if os.path.exists(path):
        with open(path, newline="") as file:
            first_row = next(csv.reader(file), [])

        if first_row == header:
            return

        backup = path.replace(
            ".csv",
            datetime.now().strftime("_old_%Y%m%d_%H%M%S.csv")
        )
        os.rename(path, backup)
        print(f"Old CSV format detected. Moved {path} -> {backup}")

    with open(path, "w", newline="") as file:
        csv.writer(file).writerow(header)

    print(f"Created: {path}")


def initialize_csv_files():
    """Create the forecasting CSV files."""

    _ensure_csv(HISTORY_CSV, HISTORY_HEADER)
    _ensure_csv(SUMMARY_CSV, SUMMARY_HEADER)


def _by_color_text(by_color):
    """{'red': 2, 'blue': 1} -> '{"red":2,"blue":1}'  (zero colours are kept so the column set is stable)."""
    return json.dumps(dict(by_color), separators=(",", ":"), sort_keys=True)


def save_history_sample(
    window_id,
    elapsed_seconds,
    interval_seconds,
    count,
    cumulative,
    by_color=None,
):
    """
    Save one time-series sample.

    count:       NEW objects (all colours) counted during this interval.
    cumulative:  total objects since the beginning of the current 3-minute window.
    by_color:    the same `count` split by colour (JSON text).
    """

    timestamp = datetime.now().isoformat(timespec="seconds")

    with open(HISTORY_CSV, "a", newline="") as file:
        csv.writer(file).writerow([
            timestamp,
            window_id,
            round(elapsed_seconds, 1),
            interval_seconds,
            count,
            cumulative,
            _by_color_text(by_color or {}),
        ])


def save_window_summary(
    window_id,
    window_start,
    window_end,
    total,
    by_color=None,
):
    """
    Save the completed 3-minute window.
    """

    duration_seconds = window_end - window_start

    with open(SUMMARY_CSV, "a", newline="") as file:
        csv.writer(file).writerow([
            window_id,
            datetime.fromtimestamp(
                time.time() - duration_seconds
            ).isoformat(timespec="seconds"),
            datetime.now().isoformat(timespec="seconds"),
            round(duration_seconds, 1),
            total,
            _by_color_text(by_color or {}),
        ])


# ============================================================
# MQTT BRIDGE
# ============================================================

def live_tracks(tracks):
    """
    Return only tracks that were matched to a real detection
    in the latest processed frame.

    missed_frames == 0 means the camera actually observed the track
    in the latest frame. Tracks with missed_frames > 0 are only being
    predicted by the camera tracker and are not published as live cubes.
    """
    return {
        tid: track
        for tid, track in tracks.items()
        if track.missed_frames == 0
    }


class MqttBridge:
    """
    Publishes camera data to the broker. The vision loop never waits on it:
    connect_async + loop_start run all network work in a background thread,
    and publish() only drops the message into a queue.

    Topics:
        camera/status      retained  online/offline (+ Last Will if we crash), enabled colours
        camera/perception  QoS 0     live active tracks + spacing between them, ~2 Hz
        camera/demand      QoS 1     10-second demand sample (cannot be re-derived), total + by_color
        camera/window      QoS 1     completed 3-minute window summary
    """

    def __init__(self, host, port, colors=None, axis="u", direction=1):
        self.colors = list(colors or [])
        self.axis, self.dir = axis, direction
        self.c = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="camera_node"
        )

        # Last Will: published BY THE BROKER if we vanish without a clean
        # disconnect (power cut, crash), so other nodes learn we are gone.
        self.c.will_set(
            "camera/status",
            json.dumps({"online": False}),
            qos=1,
            retain=True
        )

        self.c.on_connect = self._on_connect
        self.c.connect_async(host, port, keepalive=10)
        self.c.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        # Runs on every (re)connect, so status is restored after an outage.
        client.publish(
            "camera/status",
            json.dumps({
                "online": True,
                "ts": time.time(),
                "fps_target": FPS,
                "window_seconds": DATA_COLLECTION_SECONDS,
                "sample_interval": DATA_SAMPLE_INTERVAL,
                "colors": self.colors,
                "axis": self.axis,
                "dir": self.dir,
            }),
            qos=1,
            retain=True
        )

    def publish_perception(self, fps, tracks, gaps=None):
        payload = {
            "ts": time.time(),
            "fps": round(fps, 1),
            "active": len(tracks),
            "axis": self.axis,
            "dir": self.dir,
            "colors": self.colors,
            "tracks": [
                {
                    "id": t.id,
                    "u": t.position()[0],
                    "v": t.position()[1],
                    "dist_m": round(t.distance(), 3),
                    "color": t.color,
                    "w": int(round(t.w_px)),
                    "h": int(round(t.h_px)),
                    "vu": round(t.vu, 1),
                    "vv": round(t.vv, 1),
                    "hits": t.hits,      # เพิ่มหลัง "vv": round(t.vv, 1),
                }
                for t in tracks.values()
            ],
            "gaps": gaps or [],
        }
        self.c.publish("camera/perception", json.dumps(payload), qos=0)

    def publish_demand(
        self, window_id, elapsed, interval, count, cumulative, by_color=None
    ):
        payload = {
            "ts": time.time(),
            "window_id": window_id,
            "elapsed_s": round(elapsed, 1),
            "interval_s": interval,
            "count": count,
            "cumulative": cumulative,
            "by_color": dict(by_color or {}),
        }
        # QoS 1: queued and re-sent if the broker is briefly unreachable.
        self.c.publish("camera/demand", json.dumps(payload), qos=1)

    def publish_window(self, window_id, duration, total, by_color=None):
        payload = {
            "ts": time.time(),
            "window_id": window_id,
            "duration_s": round(duration, 1),
            "total": total,
            "by_color": dict(by_color or {}),
        }
        self.c.publish("camera/window", json.dumps(payload), qos=1)

    def close(self):
        # Clean shutdown: say "offline" ourselves (the Will only fires on
        # an unclean death), and give queued QoS 1 messages a moment to go out.
        try:
            info = self.c.publish(
                "camera/status",
                json.dumps({"online": False, "ts": time.time()}),
                qos=1,
                retain=True
            )
            info.wait_for_publish(1.0)
        except Exception:
            pass
        self.c.loop_stop()
        self.c.disconnect()


# ============================================================
# CALIBRATION (ROI polygon, count line, colours, belt direction)
# ============================================================

def load_calibration():
    """
    Return {'roi': Nx2 int32, 'line': [[x,y],[x,y]] or None, 'colors': {name: ranges} (overrides),
            'enabled': [names] or None, 'belt': (axis, dir) or None}, or None if unusable.
    A v2 file (only "hsv") still works: it becomes the white override.
    """

    try:
        with open(CALIB_FILE) as file:
            data = json.load(file)
        roi = np.array(data["roi"], dtype=np.int32)
        if roi.ndim != 2 or roi.shape[0] < 3 or roi.shape[1] != 2:
            raise ValueError("roi needs at least 3 [x, y] points")
        size = data.get("size")
        if size and list(size) != [WIDTH, HEIGHT]:
            raise ValueError(f"calibrated at {size}, camera is {[WIDTH, HEIGHT]}")
        line = data.get("line")
        if line is not None and len(line) != 2:
            line = None

        colors = {}
        hsv = data.get("hsv")
        if hsv:                                                    # v2 file: the two white numbers
            colors["white"] = [[[0, 0, int(hsv["v_min"])], [179, int(hsv["s_max"]), 255]]]
        for name, rngs in (data.get("colors") or {}).items():
            colors[name] = rngs
        for name, rngs in colors.items():                          # fail HERE, with the file name, not later
            for lo, hi in rngs:
                _validate_range(name, lo, hi)

        belt = None
        b = data.get("belt")
        if b:
            if b.get("axis") not in ("u", "v") or b.get("dir") not in (1, -1):
                raise ValueError('belt needs axis "u"/"v" and dir 1/-1')
            belt = (b["axis"], int(b["dir"]))

        enabled = data.get("enabled")
        if enabled is not None and not isinstance(enabled, list):
            raise ValueError('"enabled" must be a list of colour names')
        return {"roi": roi, "line": line, "colors": colors, "enabled": enabled, "belt": belt}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"[calibration] cannot use {CALIB_FILE}: {exc}")
        return None


def build_roi_mask(roi):
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    cv2.fillPoly(mask, [roi], 255)
    return mask


def side_of_line(line, x, y):
    """+1 / -1 for the two sides of the count line, 0 exactly on it."""

    (x1, y1), (x2, y2) = line
    value = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
    return (value > 0) - (value < 0)


def should_count(track, line):
    """
    The ONE place that decides whether a track becomes a count.
    A track is counted at most once in its life (track.counted).
    """

    if track.counted or track.hits < CONFIRM_FRAMES:
        return False

    if COUNT_MODE == "line" and line is not None:
        side = side_of_line(line, *track.position())

        crossed = (
            track.side != 0
            and side != 0
            and side != track.side
        )

        if side != 0:
            track.side = side

        return crossed

    return True


def calibrate(color_names, axis, direction):
    """
    Live calibration window.

        left-click   add a point to the CURRENT shape (ROI or line mode)
        right-click  SAMPLE the colour under the mouse -> sets the current colour's H/S/V box automatically
        R / L        draw the conveyor ROI polygon (>= 3 points)  /  the COUNT LINE (2 points, optional)
        N / M        next / previous colour (the sliders switch to that colour)
        A            belt axis  u <-> v        D   belt direction +1 <-> -1      (the arrow shows the travel direction)
        U / C        undo last point / clear the current shape
        S            save to vision_calibration.json        Q / ESC   quit
    Sliders: H/S/V min and max of the CURRENT colour (H min > H max = hue wraps, used for red).
    GREEN = pixels of the current colour that WILL be used (inside ROI). RED = same colour but IGNORED (outside ROI).
    The other selected colours are painted in their own colour so you can see that they do not overlap.
    """

    saved = load_calibration() or {}
    cs = ColorSet(color_names, saved.get("colors"))
    pipeline, _profile = start_camera()
    shapes = {"roi": [], "line": []}
    if saved.get("roi") is not None:
        shapes["roi"] = [list(map(int, p)) for p in saved["roi"]]
        shapes["line"] = [list(map(int, p)) for p in (saved.get("line") or [])]
    mode = ["roi"]
    belt = {"axis": axis, "dir": direction}
    cur = [0]
    win = "DELTA calibration"
    hsv_now = [None]
    last_sliders = [None]

    def push_sliders():
        vals = cs.as_sliders(cs.names[cur[0]])
        for tb, v in zip(("H min", "H max", "S min", "S max", "V min", "V max"), vals):
            cv2.setTrackbarPos(tb, win, v)
        last_sliders[0] = vals

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_RBUTTONDOWN and hsv_now[0] is not None:
            patch = hsv_now[0][max(0, y - 4):y + 5, max(0, x - 4):x + 5].reshape(-1, 3)
            if patch.size:
                h, s, v = [int(np.median(patch[:, i])) for i in range(3)]
                cs.set_single(cs.names[cur[0]], *sample_to_range(h, s, v))
                push_sliders()
                print(f"sampled {cs.names[cur[0]]}: HSV=({h},{s},{v}) -> {cs.as_sliders(cs.names[cur[0]])}")
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        pts = shapes[mode[0]]
        if mode[0] == "line" and len(pts) >= 2:
            return
        pts.append([x, y])

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    for tb, top in (("H min", 179), ("H max", 179), ("S min", 255), ("S max", 255), ("V min", 255), ("V max", 255)):
        cv2.createTrackbar(tb, win, 0, top, lambda v: None)
    push_sliders()

    try:
        while True:
            frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            color = frames.get_color_frame()

            if not color:
                continue

            image = np.asanyarray(color.get_data()).copy()
            hsv_now[0] = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Sliders -> current colour, but only when the user MOVED them (not right after switching colour).
            vals = tuple(cv2.getTrackbarPos(tb, win) for tb in ("H min", "H max", "S min", "S max", "V min", "V max"))
            if vals != last_sliders[0]:
                try:
                    cs.set_single(cs.names[cur[0]], *vals)
                    last_sliders[0] = vals
                except ValueError:
                    pass                                  # min above max while dragging: ignore until it is valid

            masks = create_color_masks(image, cs, None)
            roi_pts = np.array(shapes["roi"], dtype=np.int32)
            roi_mask = build_roi_mask(roi_pts) if len(roi_pts) >= 3 else None
            name = cs.names[cur[0]]

            for other, m in masks.items():
                if other != name:
                    image[m > 0] = COLOR_BGR.get(other, BOX_COLOR)
            m = masks[name]
            inside = cv2.bitwise_and(m, roi_mask) if roi_mask is not None else m
            outside = cv2.bitwise_and(m, cv2.bitwise_not(inside))
            image[outside > 0] = (0, 0, 255)
            image[inside > 0] = (0, 255, 0)

            if len(roi_pts) >= 1:
                cv2.polylines(image, [roi_pts], len(roi_pts) >= 3, (255, 255, 0), 2)

            if len(shapes["line"]) == 2:
                cv2.line(image, tuple(shapes["line"][0]), tuple(shapes["line"][1]), (0, 165, 255), 2)

            for x, y in shapes["roi"] + shapes["line"]:
                cv2.circle(image, (x, y), 4, (255, 0, 255), -1)

            # Belt travel arrow, so axis/dir are checked by eye.
            d = 60 * belt["dir"]
            if belt["axis"] == "u":
                cx, cy = WIDTH // 2, HEIGHT - 40
                tip = (cx + d, cy)
            else:
                cx, cy = WIDTH - 40, HEIGHT // 2
                tip = (cx, cy + d)                        # image y grows downwards
            cv2.arrowedLine(image, (cx, cy), tip, (255, 255, 255), 3, tipLength=0.3)

            cv2.putText(
                image,
                f"colour={name} ({cur[0] + 1}/{len(cs.names)})  mode={mode[0].upper()}  belt axis={belt['axis']} dir={belt['dir']:+d}",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
            )
            cv2.putText(
                image,
                "N/M colour | right-click sample | R roi | L line | A axis | D dir | U undo | C clear | S save | Q quit",
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA
            )

            cv2.imshow(win, image)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q"), ord("Q")):
                break
            elif key in (ord("r"), ord("R")):
                mode[0] = "roi"
            elif key in (ord("l"), ord("L")):
                mode[0] = "line"
            elif key in (ord("n"), ord("N")):
                cur[0] = (cur[0] + 1) % len(cs.names)
                push_sliders()
            elif key in (ord("m"), ord("M")):
                cur[0] = (cur[0] - 1) % len(cs.names)
                push_sliders()
            elif key in (ord("a"), ord("A")):
                belt["axis"] = "v" if belt["axis"] == "u" else "u"
            elif key in (ord("d"), ord("D")):
                belt["dir"] = -belt["dir"]
            elif key in (ord("u"), ord("U")) and shapes[mode[0]]:
                shapes[mode[0]].pop()
            elif key in (ord("c"), ord("C")):
                shapes[mode[0]].clear()
            elif key in (ord("s"), ord("S")):
                if len(shapes["roi"]) < 3:
                    print("Need at least 3 ROI points before saving.")
                    continue
                data = {
                    "size": [WIDTH, HEIGHT],
                    "roi": shapes["roi"],
                    "line": shapes["line"] if len(shapes["line"]) == 2 else None,
                    "colors": cs.to_json(),
                    "enabled": cs.names,
                    "belt": {"axis": belt["axis"], "dir": belt["dir"]},
                }
                if "white" in cs.names:                      # keep the v2 keys so an old 3mins.py still reads this file
                    lo, hi = cs.ranges["white"][0]
                    data["hsv"] = {"s_max": int(hi[1]), "v_min": int(lo[2])}
                with open(CALIB_FILE, "w") as file:
                    json.dump(data, file, indent=2)
                print(f"Saved {CALIB_FILE}: colours={cs.names}, {len(shapes['roi'])} ROI points, "
                      f"line={'yes' if data['line'] else 'no'}, belt axis={belt['axis']} dir={belt['dir']:+d}")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


# ============================================================
# KALMAN TRACK
# ============================================================

class ObjectTrack:
    """
    State vector:
        [u, v, distance, velocity_u, velocity_v, velocity_distance]

    Measurement vector:
        [u, v, distance]

    v3: also remembers the cube's apparent size (w_px, h_px, smoothed) for the spacing, and exposes the
    filtered velocity (vu, vv) in pixels/second.
    """

    def __init__(self, track_id, color, u, v, distance, w=0, h=0):
        self.id = track_id
        self.color = color
        self.kalman = cv2.KalmanFilter(6, 3)

        self.kalman.transitionMatrix = np.eye(6, dtype=np.float32)

        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float32)

        self.kalman.processNoiseCov = (
            np.eye(6, dtype=np.float32) * 0.03
        )

        self.kalman.measurementNoiseCov = np.diag(
            [4.0, 4.0, 0.03]
        ).astype(np.float32)

        self.kalman.errorCovPost = np.eye(
            6, dtype=np.float32
        )

        self.kalman.statePost = np.array([
            [float(u)],
            [float(v)],
            [float(distance)],
            [0.0],
            [0.0],
            [0.0],
        ], dtype=np.float32)

        self.predicted_u = float(u)
        self.predicted_v = float(v)
        self.predicted_distance = float(distance)
        self.vu = 0.0
        self.vv = 0.0
        self.w_px = float(w)
        self.h_px = float(h)

        self.missed_frames = 0
        self.age = 1
        self.hits = 1

        # Counting state: a track is counted at most ONCE in its life.
        self.counted = False
        self.side = 0        # side of the count line last seen on (line mode)

    def predict(self, dt):
        # Clamp dt to avoid unstable predictions after a stall.
        dt = max(0.001, min(float(dt), 1.0))

        self.kalman.transitionMatrix = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0, dt],
            [0, 0, 0, 1,  0, 0],
            [0, 0, 0, 0,  1, 0],
            [0, 0, 0, 0,  0, 1],
        ], dtype=np.float32)

        prediction = self.kalman.predict()

        self.predicted_u = float(prediction[0, 0])
        self.predicted_v = float(prediction[1, 0])
        self.predicted_distance = float(prediction[2, 0])
        self.vu = float(prediction[3, 0])
        self.vv = float(prediction[4, 0])

        return self.position()

    def update(self, u, v, distance, w=None, h=None):
        # distance may be None: white/glossy cubes often have NO depth reading. The track
        # then keeps its predicted distance instead of being dropped and re-created.
        if distance is None:
            distance = self.predicted_distance

        measurement = np.array([
            [float(u)],
            [float(v)],
            [float(distance)],
        ], dtype=np.float32)

        estimate = self.kalman.correct(measurement)

        self.predicted_u = float(estimate[0, 0])
        self.predicted_v = float(estimate[1, 0])
        self.predicted_distance = float(estimate[2, 0])
        self.vu = float(estimate[3, 0])
        self.vv = float(estimate[4, 0])

        # Cube size: smoothed so one oddly-shaped contour does not change the "pieces" number.
        if w is not None and h is not None and w > 0 and h > 0:
            if self.w_px <= 0:
                self.w_px, self.h_px = float(w), float(h)
            else:
                self.w_px = 0.7 * self.w_px + 0.3 * float(w)
                self.h_px = 0.7 * self.h_px + 0.3 * float(h)

        self.missed_frames = 0
        self.age += 1
        self.hits += 1

    def mark_missed(self):
        self.missed_frames += 1
        self.age += 1

    def position(self):
        return (
            int(round(self.predicted_u)),
            int(round(self.predicted_v)),
        )

    def distance(self):
        return max(0.0, self.predicted_distance)


# ============================================================
# SPACING BETWEEN PIECES  (from the Kalman-filtered tracks)
# ============================================================

class Intrinsics:
    """Pinhole camera model (what the D415 colour stream is, to a very good approximation)."""

    def __init__(self, fx, fy, ppx, ppy):
        self.fx, self.fy, self.ppx, self.ppy = float(fx), float(fy), float(ppx), float(ppy)

    @classmethod
    def from_realsense(cls, profile):
        i = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        return cls(i.fx, i.fy, i.ppx, i.ppy)


def _usable_depth(track, belt_depth):
    """(z metres, source) - the cube's own filtered depth if it has one, else the belt depth, else None."""

    z = track.distance()
    if z >= MIN_VALID_DEPTH:
        return z, "measured"
    if belt_depth is not None:
        return belt_depth, "belt_assumed"
    return None, "none"


def compute_gaps(tracks, axis="u", direction=1, intr=None, belt_depth=None,
                 min_hits=None, max_missed=None):
    """
    Spacing between neighbouring cubes along the belt.

    Order: the cube furthest ahead in the direction of travel comes first.  For each neighbouring pair:
        gap_px     distance between the filtered centres along the belt axis (pixels)
        gap_cm     3-D distance between the two centres (needs depth + intrinsics)
        edge_cm    free space between the two cubes' EDGES along the belt  = centre distance - mean cube size
        pieces     edge_cm / mean cube size   (1.0 = room for one more cube between them)
        rate_*     how fast the gap OPENS (+) or CLOSES (-), from the Kalman velocities
        depth      "measured" (both cubes have a depth), "belt_assumed" (a cube without depth borrowed the belt's),
                   "none" (no depth at all: only the pixel numbers are valid)
    Cubes that are not confirmed yet, or not seen for a couple of frames, are left out (no ghost gaps).
    """

    min_hits = GAP_MIN_HITS if min_hits is None else min_hits
    max_missed = GAP_MAX_MISSED if max_missed is None else max_missed

    def coord(t):
        return t.predicted_u if axis == "u" else t.predicted_v

    def vel(t):
        return t.vu if axis == "u" else t.vv

    live = [t for t in tracks.values() if t.hits >= min_hits and t.missed_frames <= max_missed]
    live.sort(key=lambda t: direction * coord(t), reverse=True)          # leader first

    gaps = []
    for ahead, behind in zip(live, live[1:]):
        gap_px = abs(coord(ahead) - coord(behind))
        # + = opening: the leader moves away faster than the follower, measured along the direction of travel.
        rate_px = direction * (vel(ahead) - vel(behind))
        entry = {"ahead": ahead.id, "behind": behind.id, "gap_px": round(gap_px, 1),
                 "gap_cm": None, "edge_cm": None, "pieces": None,
                 "rate_px_s": round(rate_px, 2), "rate_cm_s": None, "depth": "none"}

        if intr is not None:
            za, src_a = _usable_depth(ahead, belt_depth)
            zb, src_b = _usable_depth(behind, belt_depth)
            if za is not None and zb is not None:
                pa = ((ahead.predicted_u - intr.ppx) * za / intr.fx, (ahead.predicted_v - intr.ppy) * za / intr.fy, za)
                pb = ((behind.predicted_u - intr.ppx) * zb / intr.fx, (behind.predicted_v - intr.ppy) * zb / intr.fy, zb)
                centre_m = math.dist(pa, pb)
                along_m = abs(pa[0] - pb[0]) if axis == "u" else abs(pa[1] - pb[1])
                if axis == "u":
                    size_a, size_b = ahead.w_px * za / intr.fx, behind.w_px * zb / intr.fx
                else:
                    size_a, size_b = ahead.h_px * za / intr.fy, behind.h_px * zb / intr.fy
                piece_m = 0.5 * (size_a + size_b)
                edge_m = max(0.0, along_m - piece_m)
                z_mean = 0.5 * (za + zb)
                f = intr.fx if axis == "u" else intr.fy
                entry.update({
                    "gap_cm": round(centre_m * 100, 1),
                    "edge_cm": round(edge_m * 100, 1),
                    "pieces": round(edge_m / piece_m, 2) if piece_m > 0 else None,
                    "rate_cm_s": round(rate_px * z_mean / f * 100, 2),
                    "depth": "measured" if (src_a == "measured" and src_b == "measured") else "belt_assumed",
                })
        gaps.append(entry)
    return gaps


# ============================================================
# DEPTH
# ============================================================
 
def get_robust_depth(
    depth_frame,
    cx,
    cy,
    radius=DEPTH_SAMPLE_RADIUS
):
    """Median of valid depths in a small square around the centroid."""
 
    depth_image = np.asanyarray(depth_frame.get_data())
 
    x_min = max(0, int(cx) - radius)
    x_max = min(
        depth_image.shape[1],
        int(cx) + radius + 1
    )
 
    y_min = max(0, int(cy) - radius)
    y_max = min(
        depth_image.shape[0],
        int(cy) + radius + 1
    )
 
    patch = depth_image[y_min:y_max, x_min:x_max]
 
    if patch.size == 0:
        return None
 
    depth_scale = depth_frame.get_units()
    metres = patch.astype(np.float32) * depth_scale
 
    valid = metres[
        (metres >= MIN_VALID_DEPTH) &
        (metres <= MAX_VALID_DEPTH)
    ]
 
    if valid.size == 0:
        return None
 
    return float(np.median(valid))


# ============================================================
# COLOR DETECTION  (v3: any number of colours, one HSV conversion)
# ============================================================

def create_color_masks(frame, colors, roi_mask=None):
    """
    {colour name: mask}.  The frame is converted to HSV ONCE.  A pixel that fits two colours goes to the one that
    comes first in `colors.names` (so masks never overlap and a cube can only be one colour).
    """

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    claimed = np.zeros(hsv.shape[:2], dtype=np.uint8)
    masks = {}

    for name in colors.names:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lo, hi in colors.ranges[name]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

        # Conveyor ROI: colour anywhere else (table, boxes, people) is thrown away HERE,
        # before any contour/track exists, so it can never be counted.
        if roi_mask is not None:
            mask = cv2.bitwise_and(mask, roi_mask)

        mask = cv2.bitwise_and(mask, cv2.bitwise_not(claimed))
        claimed = cv2.bitwise_or(claimed, mask)

        # OPEN removes small speckles; CLOSE fills small holes inside objects.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

        masks[name] = mask

    return masks


def detect_objects(mask, color_name):
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
 
    detections = []
 
    for contour in contours:
 
        area = cv2.contourArea(contour)
 
        if area < MIN_AREA or area > MAX_AREA:
            continue
 
        x, y, w, h = cv2.boundingRect(contour)
 
        if w <= 0 or h <= 0:
            continue
 
        moments = cv2.moments(contour)
 
        if moments["m00"] == 0:
            continue
 
        cx = int(
            moments["m10"] / moments["m00"]
        )
 
        cy = int(
            moments["m01"] / moments["m00"]
        )
 
        detections.append({
            "color": color_name,
            "bbox": (x, y, w, h),
            "cx": cx,
            "cy": cy,
            "area": area,
        })
 
    return detections


# ============================================================
# TRACK MATCHING
# ============================================================
 
def distance_2d(x1, y1, x2, y2):
    return math.hypot(
        x1 - x2,
        y1 - y2
    )
 
 
def match_detections_to_tracks(
    detections,
    tracks
):
    """Greedy nearest-neighbor matching, restricted by color."""
 
    candidate_pairs = []
 
    for det_index, det in enumerate(detections):
 
        for track_id, track in tracks.items():
 
            if track.color != det["color"]:
                continue
 
            px, py = track.position()
 
            distance = distance_2d(
                det["cx"],
                det["cy"],
                px,
                py
            )
 
            if distance <= MAX_MATCH_DISTANCE:
                candidate_pairs.append(
                    (distance, det_index, track_id)
                )
 
    candidate_pairs.sort(
        key=lambda item: item[0]
    )
 
    matches = []
    used_detections = set()
    used_tracks = set()
 
    for _, det_index, track_id in candidate_pairs:
 
        if det_index in used_detections:
            continue
 
        if track_id in used_tracks:
            continue
 
        matches.append(
            (det_index, track_id)
        )
 
        used_detections.add(det_index)
        used_tracks.add(track_id)
 
    unmatched_detections = [
        i
        for i in range(len(detections))
        if i not in used_detections
    ]
 
    unmatched_tracks = [
        track_id
        for track_id in tracks
        if track_id not in used_tracks
    ]
 
    return (
        matches,
        unmatched_detections,
        unmatched_tracks
    )


# ============================================================
# DRAWING
# ============================================================

def draw_track(
    frame,
    track,
    detected_this_frame
):
    x, y = track.position()
    distance = track.distance()

    box_color = COLOR_BGR.get(track.color.lower(), BOX_COLOR)

    cv2.circle(
        frame,
        (x, y),
        5,
        (0, 0, 255),
        -1
    )

    cv2.line(
        frame,
        (x - 10, y),
        (x + 10, y),
        box_color,
        2
    )

    cv2.line(
        frame,
        (x, y - 10),
        (x, y + 10),
        box_color,
        2
    )

    status = (
        ""
        if detected_this_frame
        else " PRED"
    )

    label = (
        f"ID {track.id} | "
        f"{track.color} | "
        f"{distance:.2f} m{status}{'  COUNTED' if track.counted else ''}"
    )

    cv2.putText(
        frame,
        label,
        (max(5, x - 70), max(20, y - 15)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        box_color,
        2,
        cv2.LINE_AA
    )


def draw_detection_box(frame, detection):

    x, y, w, h = detection["bbox"]

    color = COLOR_BGR.get(detection["color"].lower(), BOX_COLOR)

    cv2.rectangle(
        frame,
        (x, y),
        (x + w, y + h),
        color,
        2
    )

    cv2.circle(
        frame,
        (
            detection["cx"],
            detection["cy"]
        ),
        4,
        (255, 0, 0),
        -1
    )


def draw_gaps(frame, tracks, gaps):
    """A line between each pair of neighbouring cubes with 'cm / pieces' written on it."""

    for g in gaps:
        a, b = tracks.get(g["ahead"]), tracks.get(g["behind"])

        if a is None or b is None:
            continue

        pa, pb = a.position(), b.position()
        mid = ((pa[0] + pb[0]) // 2, (pa[1] + pb[1]) // 2)

        cv2.line(frame, pa, pb, (255, 255, 255), 1, cv2.LINE_AA)

        if g["edge_cm"] is not None:
            text = f"{g['edge_cm']:.1f} cm  ({g['pieces']:.1f} pcs)"
        else:
            text = f"{g['gap_px']:.0f} px"

        cv2.putText(frame, text, (mid[0] - 40, mid[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


# ============================================================
# DASHBOARD
# ============================================================

def draw_dashboard(
    frame,
    tracks,
    total_unique_objects,
    fps,
    window_id,
    window_elapsed,
    window_count,
    by_color=None,
    colors=None,
):
    """
    Draw both current active tracks and demand-collection data.
    """

    elapsed = min(
        window_elapsed,
        DATA_COLLECTION_SECONDS
    )

    remaining = max(
        0,
        DATA_COLLECTION_SECONDS - elapsed
    )

    names = list(colors or [])
    by_color = by_color or {}

    lines = [
        "DELTA PERCEPTION",
        f"Active: {len(tracks)}   (tracks created: {total_unique_objects})",
        "",
        f"3-MIN WINDOW: #{window_id}",
        f"Counted (this window): {window_count}",
    ]
    lines += [f"   {n}: {by_color.get(n, 0)}" for n in names]
    lines += [
        f"Time: {elapsed:05.1f} / {DATA_COLLECTION_SECONDS}s",
        f"Remaining: {remaining:05.1f}s",
        f"FPS: {fps:.1f}",
    ]

    overlay = frame.copy()

    cv2.rectangle(
        overlay,
        (10, 10),
        (370, 20 + 18 * len(lines)),
        (20, 20, 20),
        -1
    )

    cv2.addWeighted(
        overlay,
        0.70,
        frame,
        0.30,
        0,
        frame
    )

    for i, line in enumerate(lines):

        cv2.putText(
            frame,
            line,
            (20, 35 + i * 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )


# ============================================================
# FRAME PROCESSOR  (everything between "a colour image arrived" and "tracks + counts are updated")
# Kept separate from the camera loop so it can be tested with synthetic images, no camera needed.
# ============================================================

class FrameProcessor:

    def __init__(self, colors, roi_mask=None, count_line=None, intr=None,
                 axis=DEFAULT_BELT_AXIS, direction=DEFAULT_BELT_DIR):
        self.colors = colors
        self.roi_mask = roi_mask
        self.count_line = count_line
        self.intr = intr
        self.axis = axis
        self.direction = direction

        self.tracks = {}
        self.next_track_id = 1
        self.total_unique_objects = 0

        # Counted tracks that just vanished: (u, v, time, colour). Used to avoid counting the same cube twice.
        self.graveyard = []

        self.window_count = 0
        self.interval_count = 0
        self.window_by_color = Counter()
        self.interval_by_color = Counter()

        # Typical depth of the belt surface (EMA of what the cubes measured). A cube whose depth the camera
        # could not read (white / glossy!) borrows it, so the spacing in cm still works.
        self.belt_depth = None

    def reset_interval(self):
        self.interval_count = 0
        self.interval_by_color = Counter()

    def reset_window(self):
        self.window_count = 0
        self.window_by_color = Counter()
        self.reset_interval()

    def process(self, color_image, depth_metres, now, dt):
        """
        color_image:  BGR frame.   depth_metres: float32 HxW array in metres (or None = no depth at all).
        Returns (detections, detected_track_ids, masks).
        """

        tracks = self.tracks

        # ---- predict existing tracks
        for track in tracks.values():
            track.predict(dt)

        # ---- colour detection
        masks = create_color_masks(color_image, self.colors, self.roi_mask)
        detections = []

        for name in self.colors.names:
            detections += detect_objects(masks[name], name.upper())

        # ---- depth (display + spacing only; a detection without depth is KEPT)
        for detection in detections:

            distance = None

            if depth_metres is not None:
                cx, cy = detection["cx"], detection["cy"]
                radius = DEPTH_SAMPLE_RADIUS

                patch = depth_metres[
                    max(0, cy - radius):min(depth_metres.shape[0], cy + radius + 1),
                    max(0, cx - radius):min(depth_metres.shape[1], cx + radius + 1),
                ]

                valid = patch[
                    (patch >= MIN_VALID_DEPTH)
                    &
                    (patch <= MAX_VALID_DEPTH)
                ]

                if valid.size:
                    distance = float(np.median(valid))

            detection["distance"] = distance

        # ---- match
        (
            matches,
            unmatched_detections,
            unmatched_tracks
        ) = match_detections_to_tracks(
            detections,
            tracks
        )

        detected_track_ids = set()

        # ---- update existing tracks
        for det_index, track_id in matches:

            detection = detections[det_index]
            _x, _y, bw, bh = detection["bbox"]

            tracks[track_id].update(
                detection["cx"],
                detection["cy"],
                detection["distance"],
                bw,
                bh,
            )

            detected_track_ids.add(track_id)

        # ---- create new tracks
        for det_index in unmatched_detections:

            detection = detections[det_index]
            _x, _y, bw, bh = detection["bbox"]

            track = ObjectTrack(
                self.next_track_id,
                detection["color"],
                detection["cx"],
                detection["cy"],
                detection["distance"]
                if detection["distance"] is not None
                else 0.0,
                bw,
                bh,
            )

            # Is this "new" track really a counted cube of the SAME colour that was lost for a moment?
            # Then it inherits counted=True and will NOT be counted again.
            for grave in self.graveyard:
                if (
                    grave[3] == detection["color"]
                    and now - grave[2] <= DEDUP_SECONDS
                    and distance_2d(
                        detection["cx"], detection["cy"],
                        grave[0], grave[1]
                    ) <= DEDUP_DISTANCE
                ):
                    track.counted = True
                    self.graveyard.remove(grave)
                    break

            tracks[self.next_track_id] = track
            detected_track_ids.add(self.next_track_id)
            self.total_unique_objects += 1
            self.next_track_id += 1

        # ---- counting: one cube = one track = one count; should_count() is the only place that decides
        for track_id in detected_track_ids:

            track = tracks[track_id]

            if should_count(track, self.count_line):

                track.counted = True

                self.window_count += 1
                self.interval_count += 1
                self.window_by_color[track.color.lower()] += 1
                self.interval_by_color[track.color.lower()] += 1

        # ---- missed / expired tracks
        for track_id in unmatched_tracks:

            if track_id in tracks:
                tracks[track_id].mark_missed()

        expired_ids = [
            track_id
            for track_id, track in tracks.items()
            if track.missed_frames
            > MAX_MISSED_FRAMES
        ]

        for track_id in expired_ids:

            if tracks[track_id].counted:
                self.graveyard.append((
                    tracks[track_id].predicted_u,
                    tracks[track_id].predicted_v,
                    now,
                    tracks[track_id].color,
                ))

            del tracks[track_id]

        self.graveyard[:] = [
            g for g in self.graveyard
            if now - g[2] <= DEDUP_SECONDS
        ]

        # ---- belt depth: slow average of what confirmed cubes actually measured this frame
        measured = [
            d["distance"] for d in detections
            if d["distance"] is not None
        ]

        if measured:
            m = float(np.median(measured))
            self.belt_depth = m if self.belt_depth is None else 0.9 * self.belt_depth + 0.1 * m

        return detections, detected_track_ids, masks

    def gaps(self):
        """Spacing between neighbouring cubes (see compute_gaps)."""

        return compute_gaps(
            self.tracks, self.axis, self.direction, self.intr, self.belt_depth
        )


# ============================================================
# CAMERA INITIALIZATION
# ============================================================
 
def start_camera():
    """
    Start matching low-rate streams and verify that frames arrive.
    """
 
    if rs is None:
        raise RuntimeError("pyrealsense2 is not installed: the camera cannot start (colour logic can still be tested).")

    pipeline = rs.pipeline()
    config = rs.config()
 
    config.enable_stream(
        rs.stream.color,
        WIDTH,
        HEIGHT,
        rs.format.bgr8,
        FPS
    )
 
    config.enable_stream(
        rs.stream.depth,
        WIDTH,
        HEIGHT,
        rs.format.z16,
        FPS
    )
 
    print(
        f"\nStarting RealSense D415 at "
        f"{WIDTH}x{HEIGHT}, {FPS} FPS..."
    )
 
    profile = pipeline.start(config)
 
    try:
 
        for i in range(WARMUP_FRAMES):
 
            frames = pipeline.wait_for_frames(
                FRAME_TIMEOUT_MS
            )
 
            if (
                not frames.get_color_frame()
                or not frames.get_depth_frame()
            ):
                raise RuntimeError(
                    "Camera started, but a color "
                    "or depth frame is missing."
                )
 
            print(
                f"Camera warm-up: "
                f"{i + 1}/{WARMUP_FRAMES}",
                flush=True
            )
 
    except Exception:
 
        pipeline.stop()
        raise
 
    return pipeline, profile


# ============================================================
# MAIN
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(description="DELTA vision node (selectable colours, spacing, demand)")
    parser.add_argument("--colors", default=None,
                        help="comma-separated colours to detect, in priority order, e.g. white,red,blue "
                             "(default: the 'enabled' list of vision_calibration.json, else white)")
    parser.add_argument("--list-colors", action="store_true", help="print the available colours and exit")
    parser.add_argument("--belt-axis", choices=("u", "v"), default=None,
                        help="image axis along the belt (u = x, v = y). Default: from calibration, else u")
    parser.add_argument("--belt-dir", type=int, choices=(1, -1), default=None,
                        help="+1 if that coordinate GROWS in the direction of travel, -1 if it shrinks")
    parser.add_argument("--calibrate", action="store_true",
                        help="open the calibration window (ROI, count line, colours, belt direction) and exit")
    parser.add_argument("--no-roi", action="store_true",
                        help="run WITHOUT a conveyor ROI (old behaviour: the colours are counted anywhere in the frame)")
    return parser.parse_args()


def main():

    args = parse_args()

    if args.list_colors:
        print("Available colours (HSV starting points; re-measure with --calibrate):")
        for name, rngs in COLOR_PRESETS.items():
            print(f"  {name:7s} " + "  +  ".join(f"H{lo[0]}-{hi[0]} S{lo[1]}-{hi[1]} V{lo[2]}-{hi[2]}" for lo, hi in rngs))
        return

    calibration = load_calibration()
    saved = calibration or {}

    names = (
        [c for c in args.colors.split(",")] if args.colors
        else (saved.get("enabled") or ["white"])
    )
    belt_axis, belt_dir = saved.get("belt") or (DEFAULT_BELT_AXIS, DEFAULT_BELT_DIR)
    belt_axis = args.belt_axis or belt_axis
    belt_dir = args.belt_dir or belt_dir

    try:
        colors = ColorSet(names, saved.get("colors"))
    except ValueError as exc:
        print(f"Colour setup error: {exc}")
        return

    if args.calibrate:
        calibrate(colors.names, belt_axis, belt_dir)
        return

    if calibration is None and not args.no_roi:
        print(f"No usable {CALIB_FILE}. Run:  python 3mins.py --calibrate --colors {','.join(colors.names)}")
        print("(or start with --no-roi to count the colours anywhere in the frame)")
        return

    print("=" * 65)
    print(
        "DELTA - Raspberry Pi 4 Object Perception "
        "+ Spacing + Demand Collection"
    )
    print("=" * 65)

    print(
        "Mode: HSV color detection + "
        "RealSense depth + Kalman"
    )

    print(f"Colours (priority order): {', '.join(colors.names)}")
    print(f"Belt axis: {belt_axis}   direction: {belt_dir:+d}")

    print(
        f"Camera streams: "
        f"{WIDTH}x{HEIGHT} at {FPS} FPS"
    )

    print(
        f"Demand window: "
        f"{DATA_COLLECTION_SECONDS} seconds"
    )

    print(
        f"Data sample interval: "
        f"{DATA_SAMPLE_INTERVAL} seconds"
    )

    print(
        f"History file: {HISTORY_CSV}"
    )

    print(
        f"Summary file: {SUMMARY_CSV}"
    )

    print("Press ESC or Q to exit.")

    print("=" * 65)

    # --------------------------------------------------------
    # Initialize data files
    # --------------------------------------------------------

    initialize_csv_files()

    # --------------------------------------------------------
    # Start camera
    # --------------------------------------------------------

    pipeline = None

    try:

        pipeline, profile = start_camera()

    except Exception as exc:

        print(
            "\nERROR: RealSense did not deliver "
            "the requested streams."
        )

        print(f"Details: {exc}")

        print(
            "\nCheck that RealSense Viewer is closed, "
            "then try again."
        )

        print(
            "The standalone test succeeded at 6 FPS, "
            "so this error may indicate a "
            "stream-startup or device-access issue."
        )

        traceback.print_exc()

        return

    # --------------------------------------------------------
    # Tracking state
    # --------------------------------------------------------

    # Connect to the broker only after the camera started successfully.
    # Non-blocking: if the broker is down, vision + CSV logging still run.
    bridge = MqttBridge(MQTT_HOST, MQTT_PORT, colors.names, belt_axis, belt_dir)
    last_perception_pub = 0.0

    align = rs.align(rs.stream.color)

    roi_mask = build_roi_mask(calibration["roi"]) if calibration else None
    count_line = calibration["line"] if calibration else None

    try:
        intr = Intrinsics.from_realsense(profile)
    except Exception as exc:                       # no intrinsics -> spacing in pixels only, everything else still works
        print(f"[spacing] camera intrinsics unavailable ({exc}); gaps will be in pixels only")
        intr = None

    proc = FrameProcessor(colors, roi_mask, count_line, intr, belt_axis, belt_dir)

    # --------------------------------------------------------
    # Demand collection state
    # --------------------------------------------------------

    window_id = 1

    window_start_time = time.monotonic()

    last_sample_time = window_start_time

    # --------------------------------------------------------
    # FPS
    # --------------------------------------------------------

    previous_time = time.monotonic()
    fps_display = 0.0

    try:

        while True:

            # =================================================
            # GET FRAME
            # =================================================

            try:

                frames = pipeline.wait_for_frames(
                    FRAME_TIMEOUT_MS
                )

            except RuntimeError as exc:

                print(
                    f"\nFrame timeout/error: {exc}"
                )

                print(
                    "No new frames arrived within "
                    "the configured timeout."
                )

                break

            aligned_frames = align.process(
                frames
            )

            color_frame = (
                aligned_frames.get_color_frame()
            )

            depth_frame = (
                aligned_frames.get_depth_frame()
            )

            if (
                not color_frame
                or not depth_frame
            ):

                print(
                    "Warning: received a frameset "
                    "missing color or depth; skipping."
                )

                continue

            color_image = np.asanyarray(
                color_frame.get_data()
            )

            # =================================================
            # TIME / FPS
            # =================================================

            current_time = time.monotonic()

            dt = current_time - previous_time

            previous_time = current_time

            if dt > 0:

                instant_fps = 1.0 / dt

                fps_display = (
                    0.9 * fps_display
                    + 0.1 * instant_fps
                    if fps_display > 0
                    else instant_fps
                )

            # =================================================
            # DETECT, TRACK, COUNT  (FrameProcessor)
            # =================================================

            depth_metres = (
                np.asanyarray(depth_frame.get_data()).astype(np.float32)
                * depth_frame.get_units()
            )

            detections, detected_track_ids, masks = proc.process(
                color_image, depth_metres, current_time, dt
            )

            gaps = proc.gaps()

            # =================================================
            # PUBLISH LIVE PERCEPTION (2 Hz)
            # =================================================

            if (
                current_time - last_perception_pub
                >= PERCEPTION_PUBLISH_INTERVAL
            ):
                bridge.publish_perception(
                    fps_display,
                    live_tracks(proc.tracks),
                    gaps
                )
                last_perception_pub = current_time

            # =================================================
            # DEMAND DATA SAMPLING
            # =================================================

            elapsed_window = (
                current_time
                - window_start_time
            )

            elapsed_sample = (
                current_time
                - last_sample_time
            )

            # ------------------------------------------------
            # Save every 10 seconds
            # ------------------------------------------------

            if (
                elapsed_sample
                >= DATA_SAMPLE_INTERVAL
            ):

                save_history_sample(
                    window_id=window_id,
                    elapsed_seconds=elapsed_window,
                    interval_seconds=DATA_SAMPLE_INTERVAL,
                    count=proc.interval_count,
                    cumulative=proc.window_count,
                    by_color=proc.interval_by_color,
                )

                print(
                    f"[DATA] Window {window_id} | "
                    f"{elapsed_window:6.1f}s | "
                    f"+{proc.interval_count} {dict(proc.interval_by_color)} "
                    f"(total {proc.window_count})"
                )

                # Send the same sample to the broker (CSV stays as backup).
                bridge.publish_demand(
                    window_id,
                    elapsed_window,
                    DATA_SAMPLE_INTERVAL,
                    proc.interval_count,
                    proc.window_count,
                    proc.interval_by_color,
                )

                # Reset interval counter.
                proc.reset_interval()

                last_sample_time = current_time

            # =================================================
            # FINISH 3-MINUTE WINDOW
            # =================================================

            if (
                elapsed_window
                >= DATA_COLLECTION_SECONDS
            ):

                # Objects counted after the last 10 s sample would otherwise be in the
                # window total but in no sample. Publish that tail as its own sample.
                tail_seconds = current_time - last_sample_time   # fresh: 0 if a sample just fired

                if tail_seconds >= 0.5:

                    save_history_sample(
                        window_id=window_id,
                        elapsed_seconds=elapsed_window,
                        interval_seconds=round(tail_seconds),
                        count=proc.interval_count,
                        cumulative=proc.window_count,
                        by_color=proc.interval_by_color,
                    )

                    bridge.publish_demand(
                        window_id,
                        elapsed_window,
                        round(tail_seconds),
                        proc.interval_count,
                        proc.window_count,
                        proc.interval_by_color,
                    )

                window_end_time = current_time

                save_window_summary(
                    window_id=window_id,
                    window_start=window_start_time,
                    window_end=window_end_time,
                    total=proc.window_count,
                    by_color=proc.window_by_color,
                )

                print("\n" + "-" * 65)

                print(
                    f"3-MINUTE WINDOW #{window_id} COMPLETE"
                )

                print(
                    f"Objects : {proc.window_count}   by colour: {dict(proc.window_by_color)}"
                )

                print(
                    f"Saved to: {SUMMARY_CSV}"
                )

                bridge.publish_window(
                    window_id,
                    elapsed_window,
                    proc.window_count,
                    proc.window_by_color,
                )

                print("-" * 65 + "\n")

                # ------------------------------------------------
                # Start a NEW 3-minute window.
                # ------------------------------------------------

                window_id += 1

                window_start_time = current_time

                last_sample_time = current_time

                proc.reset_window()

            # =================================================
            # DRAW
            # =================================================

            annotated = color_image.copy()

            for detection in detections:

                draw_detection_box(
                    annotated,
                    detection
                )

            for track_id, track in proc.tracks.items():

                draw_track(
                    annotated,
                    track,
                    track_id in detected_track_ids
                )

            draw_gaps(annotated, proc.tracks, gaps)

            if calibration is not None:
                cv2.polylines(annotated, [calibration["roi"]], True, (0, 200, 0), 2)

                if count_line is not None and COUNT_MODE == "line":
                    cv2.line(annotated, tuple(count_line[0]), tuple(count_line[1]), (0, 165, 255), 2)

            # Current window elapsed time.
            display_window_elapsed = (
                current_time
                - window_start_time
            )

            draw_dashboard(
                annotated,
                proc.tracks,
                proc.total_unique_objects,
                fps_display,
                window_id,
                display_window_elapsed,
                proc.window_count,
                proc.window_by_color,
                colors.names,
            )

            # =================================================
            # DISPLAY
            # =================================================

            if SHOW_MASKS:

                for name, mask in masks.items():
                    cv2.imshow(f"Mask {name}", mask)

            cv2.imshow(
                WINDOW_NAME,
                annotated
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (
                27,
                ord("q"),
                ord("Q")
            ):
                break

    except KeyboardInterrupt:

        print("\nStopping...")

    except Exception:

        print(
            "\nRuntime error during processing:"
        )

        traceback.print_exc()

    finally:

        print(
            "\nStopping RealSense..."
        )

        if pipeline is not None:

            try:
                pipeline.stop()

            except Exception as exc:

                print(
                    f"Camera shutdown note: {exc}"
                )

        bridge.close()

        cv2.destroyAllWindows()

        print("Done.")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
