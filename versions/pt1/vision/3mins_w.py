"""
DELTA - Lightweight Object Perception + Demand Data Collection
---------------------------------------------------------------
Hardware:
    - Intel RealSense D415
    - Raspberry Pi 4
 
Features:
    - RGB + aligned depth capture
    - White-object HSV detection (low saturation, high brightness)
    - OpenCV contour detection
    - Robust median depth estimation
    - Lightweight Kalman filtering
    - Greedy nearest-neighbor object tracking
    - Unique IDs for tracked objects
    - Live object counts and distance dashboard
 
VISION v2 (this version):
    - Conveyor ROI: white is searched ONLY inside a calibrated polygon (python 3mins.py --calibrate)
    - One cube = one track ID = one count (track.counted); counting happens ONCE per track,
      either when the track is CONFIRMED (COUNT_MODE="confirm") or when it crosses a count line
    - Cubes with a missing depth reading are still tracked (white/glossy surfaces drop depth often)
    - A track that is lost and reappears right where it vanished does NOT count again
    - The last partial 10 s sample of each window is now published, so sum(count) == window total

NEW:
    - 3-minute collection windows
    - White-object arrival counting
    - 10-second demand samples
    - CSV history for future forecasting
    - Automatic 3-minute summaries
 
CSV files:
    delta_demand_history.csv
        Time-series data sampled every 10 seconds.
 
    delta_3min_summary.csv
        Summary of every completed 3-minute collection window.
 
Requirements:
    - Python 3.11
    - numpy
    - opencv-python
    - pyrealsense2
 
Run from the prism-env environment:
    prism        (activates ~/prism-env)
    cd DCAM && python 3mins.py
"""
 
import cv2
import numpy as np
import pyrealsense2 as rs
import math
import time
import traceback
import csv
import os
import json
import argparse
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
# HSV COLOR CONFIGURATION
# ============================================================
 
# White = ANY hue, LOW saturation, HIGH brightness.
# These two numbers are what you tune for your lighting:
#   - object not detected  -> lower WHITE_LOWER V (e.g. 180 -> 150) or raise S max
#   - walls/table detected -> raise WHITE_LOWER V or lower S max
# Set SHOW_MASKS = True to see the mask while tuning.
WHITE_LOWER = np.array([0, 0, 180], dtype=np.uint8)
WHITE_UPPER = np.array([179, 50, 255], dtype=np.uint8)
 
# Overlay colour (BGR). Cyan, because white boxes would vanish on white objects.
BOX_COLOR = (255, 255, 0)
 
MORPH_KERNEL = np.ones((5, 5), np.uint8)
 
 
# ============================================================
# CSV INITIALIZATION
# ============================================================
 
HISTORY_HEADER = [
    "timestamp", "window_id", "elapsed_seconds", "interval_seconds",
    "count", "cumulative",
]
SUMMARY_HEADER = [
    "window_id", "window_start", "window_end", "duration_seconds",
    "total_objects",
]
 
 
def _ensure_csv(path, header):
    """
    Create the CSV if missing. If an old file has a DIFFERENT header (the old
    orange/green layout), move it aside instead of mixing two formats.
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
    """Create the forecasting CSV files (white-only layout)."""
 
    _ensure_csv(HISTORY_CSV, HISTORY_HEADER)
    _ensure_csv(SUMMARY_CSV, SUMMARY_HEADER)
 
 
def save_history_sample(
    window_id,
    elapsed_seconds,
    interval_seconds,
    count,
    cumulative,
):
    """
    Save one time-series sample.
 
    count:
        NEW white objects detected during this interval.
 
    cumulative:
        Total white objects detected since the beginning of the
        current 3-minute window.
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
        ])
 
 
def save_window_summary(
    window_id,
    window_start,
    window_end,
    total,
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
        ])
 
 
# ============================================================
# MQTT BRIDGE
# ============================================================
 
class MqttBridge:
    """
    Publishes camera data to the broker. The vision loop never waits on it:
    connect_async + loop_start run all network work in a background thread,
    and publish() only drops the message into a queue.
 
    Topics:
        camera/status      retained  online/offline (+ Last Will if we crash)
        camera/perception  QoS 0     live active tracks, ~2 Hz
        camera/demand      QoS 1     10-second demand sample (cannot be re-derived)
        camera/window      QoS 1     completed 3-minute window summary
    """
 
    def __init__(self, host, port):
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
            }),
            qos=1,
            retain=True
        )
 
    def publish_perception(self, fps, tracks):
        payload = {
            "ts": time.time(),
            "fps": round(fps, 1),
            "active": len(tracks),
            "tracks": [
                {
                    "id": t.id,
                    "u": t.position()[0],
                    "v": t.position()[1],
                    "dist_m": round(t.distance(), 3),
                }
                for t in tracks.values()
            ],
        }
        self.c.publish("camera/perception", json.dumps(payload), qos=0)
 
    def publish_demand(
        self, window_id, elapsed, interval, count, cumulative
    ):
        payload = {
            "ts": time.time(),
            "window_id": window_id,
            "elapsed_s": round(elapsed, 1),
            "interval_s": interval,
            "count": count,
            "cumulative": cumulative,
        }
        # QoS 1: queued and re-sent if the broker is briefly unreachable.
        self.c.publish("camera/demand", json.dumps(payload), qos=1)
 
    def publish_window(self, window_id, duration, total):
        payload = {
            "ts": time.time(),
            "window_id": window_id,
            "duration_s": round(duration, 1),
            "total": total,
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
# CALIBRATION (ROI polygon, count line, white threshold)
# ============================================================

def load_calibration():
    """Return {'roi': Nx2 int32, 'line': [[x,y],[x,y]] or None, 'hsv': {...} or None}, or None."""

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
        return {"roi": roi, "line": line, "hsv": data.get("hsv")}
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"[calibration] cannot use {CALIB_FILE}: {exc}")
        return None


def apply_hsv_calibration(hsv):
    """Overwrite the two tunable white-threshold numbers (in place, module-wide)."""

    if hsv:
        WHITE_UPPER[1] = int(hsv["s_max"])
        WHITE_LOWER[2] = int(hsv["v_min"])


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


def calibrate():
    """
    Live calibration window.

        left-click   add a point to the CURRENT shape
        R            draw the conveyor ROI polygon (>= 3 points)
        L            draw the COUNT LINE (2 points; optional, only needed for COUNT_MODE="line")
        U            undo last point of the current shape
        C            clear the current shape
        S            save to vision_calibration.json
        Q / ESC      quit
    Sliders: "S max" and "V min" tune the white threshold.
    GREEN = white pixels that WILL be used (inside ROI). RED = white pixels IGNORED (outside ROI).
    """

    pipeline, _profile = start_camera()
    shapes = {"roi": [], "line": []}
    mode = ["roi"]
    win = "DELTA calibration"

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        pts = shapes[mode[0]]
        if mode[0] == "line" and len(pts) >= 2:
            return
        pts.append([x, y])

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    cv2.createTrackbar("S max", win, int(WHITE_UPPER[1]), 255, lambda v: None)
    cv2.createTrackbar("V min", win, int(WHITE_LOWER[2]), 255, lambda v: None)

    try:
        while True:
            frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            color = frames.get_color_frame()

            if not color:
                continue

            image = np.asanyarray(color.get_data()).copy()

            # Live threshold from the sliders.
            WHITE_UPPER[1] = cv2.getTrackbarPos("S max", win)
            WHITE_LOWER[2] = cv2.getTrackbarPos("V min", win)

            everywhere = create_white_mask(image, None)
            roi_pts = np.array(shapes["roi"], dtype=np.int32)
            inside = everywhere

            if len(roi_pts) >= 3:
                inside = cv2.bitwise_and(everywhere, build_roi_mask(roi_pts))

            outside = cv2.bitwise_and(everywhere, cv2.bitwise_not(inside))

            image[outside > 0] = (0, 0, 255)      # ignored white  -> red
            image[inside > 0] = (0, 255, 0)       # used white     -> green

            if len(roi_pts) >= 1:
                cv2.polylines(image, [roi_pts], len(roi_pts) >= 3, (255, 255, 0), 2)

            if len(shapes["line"]) == 2:
                cv2.line(image, tuple(shapes["line"][0]), tuple(shapes["line"][1]), (0, 165, 255), 2)

            for x, y in shapes["roi"] + shapes["line"]:
                cv2.circle(image, (x, y), 4, (255, 0, 255), -1)

            cv2.putText(
                image,
                f"mode={mode[0].upper()}  R roi | L line | U undo | C clear | S save | Q quit",
                (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA
            )

            cv2.imshow(win, image)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q"), ord("Q")):
                break
            elif key in (ord("r"), ord("R")):
                mode[0] = "roi"
            elif key in (ord("l"), ord("L")):
                mode[0] = "line"
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
                    "hsv": {
                        "s_max": int(WHITE_UPPER[1]),
                        "v_min": int(WHITE_LOWER[2]),
                    },
                }
                with open(CALIB_FILE, "w") as file:
                    json.dump(data, file, indent=2)
                print(f"Saved {CALIB_FILE}: {len(shapes['roi'])} ROI points, "
                      f"line={'yes' if data['line'] else 'no'}, "
                      f"S max={data['hsv']['s_max']}, V min={data['hsv']['v_min']}")

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
    """
 
    def __init__(self, track_id, color, u, v, distance):
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
 
        return self.position()
 
    def update(self, u, v, distance):
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
# COLOR DETECTION
# ============================================================
 
def create_white_mask(frame, roi_mask=None):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)

    # Conveyor ROI: white anywhere else (table, boxes, people) is thrown away HERE,
    # before any contour/track exists, so it can never be counted.
    if roi_mask is not None:
        mask = cv2.bitwise_and(mask, roi_mask)

    # OPEN removes small speckles; CLOSE fills small holes inside objects.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    return mask


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
 
    box_color = BOX_COLOR
 
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
 
    color = BOX_COLOR
 
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
 
    overlay = frame.copy()
 
    cv2.rectangle(
        overlay,
        (10, 10),
        (370, 205),
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
 
    lines = [
        "DELTA PERCEPTION (WHITE)",
        f"Active White:  {len(tracks)}",
        f"Tracks created: {total_unique_objects}",
        "",
        f"3-MIN WINDOW: #{window_id}",
        f"Counted (this window): {window_count}",
        f"Time: {elapsed:05.1f} / {DATA_COLLECTION_SECONDS}s",
        f"Remaining: {remaining:05.1f}s",
        f"FPS: {fps:.1f}",
    ]
 
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
# CAMERA INITIALIZATION
# ============================================================
 
def start_camera():
    """
    Start matching low-rate streams and verify that frames arrive.
    """
 
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
    parser = argparse.ArgumentParser(description="DELTA vision node (white cubes on the conveyor)")
    parser.add_argument("--calibrate", action="store_true",
                        help="open the calibration window (ROI, count line, white threshold) and exit")
    parser.add_argument("--no-roi", action="store_true",
                        help="run WITHOUT a conveyor ROI (old behaviour: white anywhere is counted)")
    return parser.parse_args()


def main():

    args = parse_args()

    if args.calibrate:
        calibrate()
        return

    calibration = load_calibration()

    if calibration is None and not args.no_roi:
        print(f"No usable {CALIB_FILE}. Run:  python 3mins.py --calibrate")
        print("(or start with --no-roi to count white anywhere in the frame)")
        return

    if calibration is not None:
        apply_hsv_calibration(calibration["hsv"])
 
    print("=" * 65)
    print(
        "DELTA - Raspberry Pi 4 Object Perception "
        "+ Demand Collection"
    )
    print("=" * 65)
 
    print(
        "Mode: HSV color detection + "
        "RealSense depth + Kalman"
    )
 
    print("Color: WHITE")
 
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
 
        pipeline, _profile = start_camera()
 
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
    bridge = MqttBridge(MQTT_HOST, MQTT_PORT)
    last_perception_pub = 0.0
 
    align = rs.align(rs.stream.color)

    roi_mask = build_roi_mask(calibration["roi"]) if calibration else None
    count_line = calibration["line"] if calibration else None

    # Counted tracks that just vanished: (u, v, time). Used to avoid counting the same cube twice.
    graveyard = []
 
    tracks = {}
 
    next_track_id = 1
 
    total_unique_objects = 0
 
    # --------------------------------------------------------
    # Demand collection state
    # --------------------------------------------------------
 
    window_id = 1
 
    window_start_time = time.monotonic()
 
    last_sample_time = window_start_time
 
    window_count = 0
 
    # Count since the last 10-second sample.
    interval_count = 0
 
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
            # PREDICT EXISTING TRACKS
            # =================================================
 
            for track in tracks.values():
                track.predict(dt)
 
            # =================================================
            # COLOR DETECTION
            # =================================================
 
            white_mask = create_white_mask(color_image, roi_mask)
 
            detections = detect_objects(
                white_mask,
                "WHITE"
            )
 
            # =================================================
            # DEPTH
            # =================================================
 
            depth_image = (
                np.asanyarray(
                    depth_frame.get_data()
                ).astype(np.float32)
            )

            depth_metres = (
                depth_image * depth_frame.get_units()
            )

            for detection in detections:

                cx = detection["cx"]
                cy = detection["cy"]

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

                # None = no depth here. The detection is KEPT: depth is only for display,
                # and dropping the detection made tracks flicker and be re-created.
                detection["distance"] = (
                    float(np.median(valid))
                    if valid.size
                    else None
                )

            valid_detections = detections

            # =================================================
            # MATCH TRACKS
            # =================================================
 
            (
                matches,
                unmatched_detections,
                unmatched_tracks
            ) = match_detections_to_tracks(
                valid_detections,
                tracks
            )
 
            detected_track_ids = set()
 
            # =================================================
            # UPDATE EXISTING TRACKS
            # =================================================
 
            for det_index, track_id in matches:
 
                detection = (
                    valid_detections[det_index]
                )
 
                tracks[track_id].update(
                    detection["cx"],
                    detection["cy"],
                    detection["distance"]
                )
 
                detected_track_ids.add(
                    track_id
                )
 
            # =================================================
            # CREATE NEW TRACKS
            # =================================================

            for det_index in unmatched_detections:

                detection = (
                    valid_detections[det_index]
                )

                track = ObjectTrack(
                    next_track_id,
                    detection["color"],
                    detection["cx"],
                    detection["cy"],
                    detection["distance"]
                    if detection["distance"] is not None
                    else 0.0
                )

                # Is this "new" track really a counted cube that was lost for a moment?
                # Then it inherits counted=True and will NOT be counted again.
                for grave in graveyard:
                    if (
                        current_time - grave[2] <= DEDUP_SECONDS
                        and distance_2d(
                            detection["cx"], detection["cy"],
                            grave[0], grave[1]
                        ) <= DEDUP_DISTANCE
                    ):
                        track.counted = True
                        graveyard.remove(grave)
                        break

                tracks[next_track_id] = track

                detected_track_ids.add(
                    next_track_id
                )

                total_unique_objects += 1

                next_track_id += 1

            # =================================================
            # COUNTING
            # One cube = one track = one count. should_count()
            # is the only place that decides.
            # =================================================

            for track_id in detected_track_ids:

                track = tracks[track_id]

                if should_count(track, count_line):

                    track.counted = True

                    window_count += 1
                    interval_count += 1

            # =================================================
            # MARK MISSED TRACKS
            # =================================================
 
            for track_id in unmatched_tracks:
 
                if track_id in tracks:
                    tracks[track_id].mark_missed()
 
            # =================================================
            # DELETE EXPIRED TRACKS
            # =================================================

            expired_ids = [
                track_id
                for track_id, track in tracks.items()
                if track.missed_frames
                > MAX_MISSED_FRAMES
            ]

            for track_id in expired_ids:

                if tracks[track_id].counted:
                    graveyard.append((
                        tracks[track_id].predicted_u,
                        tracks[track_id].predicted_v,
                        current_time,
                    ))

                del tracks[track_id]

            graveyard[:] = [
                g for g in graveyard
                if current_time - g[2] <= DEDUP_SECONDS
            ]

            # =================================================
            # PUBLISH LIVE PERCEPTION (2 Hz)
            # =================================================
 
            if (
                current_time - last_perception_pub
                >= PERCEPTION_PUBLISH_INTERVAL
            ):
                bridge.publish_perception(fps_display, tracks)
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
                    count=interval_count,
                    cumulative=window_count,
                )
 
                print(
                    f"[DATA] Window {window_id} | "
                    f"{elapsed_window:6.1f}s | "
                    f"White +{interval_count} "
                    f"(total {window_count})"
                )
 
                # Send the same sample to the broker (CSV stays as backup).
                bridge.publish_demand(
                    window_id,
                    elapsed_window,
                    DATA_SAMPLE_INTERVAL,
                    interval_count,
                    window_count,
                )
 
                # Reset interval counter.
                interval_count = 0
 
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
                        count=interval_count,
                        cumulative=window_count,
                    )

                    bridge.publish_demand(
                        window_id,
                        elapsed_window,
                        round(tail_seconds),
                        interval_count,
                        window_count,
                    )

                window_end_time = current_time
 
                save_window_summary(
                    window_id=window_id,
                    window_start=window_start_time,
                    window_end=window_end_time,
                    total=window_count,
                )
 
                print("\n" + "-" * 65)
 
                print(
                    f"3-MINUTE WINDOW #{window_id} COMPLETE"
                )
 
                print(
                    f"White objects : "
                    f"{window_count}"
                )
 
                print(
                    f"Saved to: {SUMMARY_CSV}"
                )
 
                bridge.publish_window(
                    window_id,
                    elapsed_window,
                    window_count,
                )
 
                print("-" * 65 + "\n")
 
                # ------------------------------------------------
                # Start a NEW 3-minute window.
                # ------------------------------------------------
 
                window_id += 1
 
                window_start_time = current_time
 
                last_sample_time = current_time
 
                window_count = 0
 
                interval_count = 0
 
            # =================================================
            # DRAW
            # =================================================
 
            annotated = color_image.copy()
 
            for detection in valid_detections:
 
                draw_detection_box(
                    annotated,
                    detection
                )
 
            for track_id, track in tracks.items():
 
                draw_track(
                    annotated,
                    track,
                    track_id in detected_track_ids
                )
 
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
                tracks,
                total_unique_objects,
                fps_display,
                window_id,
                display_window_elapsed,
                window_count,
            )
 
            # =================================================
            # DISPLAY
            # =================================================
 
            if SHOW_MASKS:
 
                cv2.imshow(
                    "White Mask",
                    white_mask
                )
 
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