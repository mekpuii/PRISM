"""
DELTA - Lightweight Object Perception + Demand Data Collection
---------------------------------------------------------------
Hardware:
    - Intel RealSense D415
    - Raspberry Pi 4

Features:
    - RGB + aligned depth capture
    - Orange and green HSV color detection
    - OpenCV contour detection
    - Robust median depth estimation
    - Lightweight Kalman filtering
    - Greedy nearest-neighbor object tracking
    - Unique IDs for tracked objects
    - Live object counts and distance dashboard

NEW:
    - 3-minute collection windows
    - Green/orange arrival counting
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
from datetime import datetime


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
# HSV COLOR CONFIGURATION
# ============================================================

ORANGE_LOWER = np.array([5, 100, 80], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 255, 255], dtype=np.uint8)

GREEN_LOWER = np.array([35, 60, 50], dtype=np.uint8)
GREEN_UPPER = np.array([90, 255, 255], dtype=np.uint8)

MORPH_KERNEL = np.ones((5, 5), np.uint8)


# ============================================================
# CSV INITIALIZATION
# ============================================================

def initialize_csv_files():
    """
    Create the forecasting CSV files if they do not already exist.
    """

    if not os.path.exists(HISTORY_CSV):
        with open(HISTORY_CSV, "w", newline="") as file:
            writer = csv.writer(file)

            writer.writerow([
                "timestamp",
                "window_id",
                "elapsed_seconds",
                "interval_seconds",
                "green_count",
                "orange_count",
                "total_count",
                "green_cumulative",
                "orange_cumulative",
                "total_cumulative",
            ])

        print(f"Created: {HISTORY_CSV}")

    if not os.path.exists(SUMMARY_CSV):
        with open(SUMMARY_CSV, "w", newline="") as file:
            writer = csv.writer(file)

            writer.writerow([
                "window_id",
                "window_start",
                "window_end",
                "duration_seconds",
                "green_total",
                "orange_total",
                "total_objects",
            ])

        print(f"Created: {SUMMARY_CSV}")


def save_history_sample(
    window_id,
    elapsed_seconds,
    interval_seconds,
    green_count,
    orange_count,
    green_cumulative,
    orange_cumulative,
):
    """
    Save one time-series sample.

    green_count / orange_count:
        Number of NEW objects detected during this interval.

    green_cumulative / orange_cumulative:
        Total number of detected objects since the beginning
        of the current 3-minute window.
    """

    timestamp = datetime.now().isoformat(timespec="seconds")

    total_count = green_count + orange_count
    total_cumulative = green_cumulative + orange_cumulative

    with open(HISTORY_CSV, "a", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            timestamp,
            window_id,
            round(elapsed_seconds, 1),
            interval_seconds,
            green_count,
            orange_count,
            total_count,
            green_cumulative,
            orange_cumulative,
            total_cumulative,
        ])


def save_window_summary(
    window_id,
    window_start,
    window_end,
    green_total,
    orange_total,
):
    """
    Save the completed 3-minute window.
    """

    duration_seconds = window_end - window_start
    total_objects = green_total + orange_total

    with open(SUMMARY_CSV, "a", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            window_id,
            datetime.fromtimestamp(
                time.time() - duration_seconds
            ).isoformat(timespec="seconds"),
            datetime.now().isoformat(timespec="seconds"),
            round(duration_seconds, 1),
            green_total,
            orange_total,
            total_objects,
        ])


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

def create_color_masks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    orange_mask = cv2.inRange(
        hsv,
        ORANGE_LOWER,
        ORANGE_UPPER
    )

    green_mask = cv2.inRange(
        hsv,
        GREEN_LOWER,
        GREEN_UPPER
    )

    orange_mask = cv2.morphologyEx(
        orange_mask,
        cv2.MORPH_OPEN,
        MORPH_KERNEL
    )

    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_OPEN,
        MORPH_KERNEL
    )

    orange_mask = cv2.morphologyEx(
        orange_mask,
        cv2.MORPH_CLOSE,
        MORPH_KERNEL
    )

    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_CLOSE,
        MORPH_KERNEL
    )

    return orange_mask, green_mask


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

    box_color = (
        (0, 140, 255)
        if track.color == "ORANGE"
        else (0, 255, 0)
    )

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
        f"{distance:.2f} m{status}"
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

    color = (
        (0, 140, 255)
        if detection["color"] == "ORANGE"
        else (0, 255, 0)
    )

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
    window_green,
    window_orange,
):
    """
    Draw both current active tracks and demand-collection data.
    """

    orange_active = sum(
        t.color == "ORANGE"
        for t in tracks.values()
    )

    green_active = sum(
        t.color == "GREEN"
        for t in tracks.values()
    )

    window_total = (
        window_green +
        window_orange
    )

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
        (370, 265),
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
        "DELTA PERCEPTION",
        f"Active Orange: {orange_active}",
        f"Active Green:  {green_active}",
        f"Active Total:  {orange_active + green_active}",
        f"Unique Tracks: {total_unique_objects}",
        "",
        f"3-MIN WINDOW: #{window_id}",
        f"Collected Orange: {window_orange}",
        f"Collected Green:  {window_green}",
        f"Collected Total:  {window_total}",
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

def main():

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

    print("Colors: ORANGE / GREEN")

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

    align = rs.align(rs.stream.color)

    tracks = {}

    next_track_id = 1

    total_unique_objects = 0

    # --------------------------------------------------------
    # Demand collection state
    # --------------------------------------------------------

    window_id = 1

    window_start_time = time.monotonic()

    last_sample_time = window_start_time

    window_green_count = 0
    window_orange_count = 0

    # Counts since the last 10-second sample.
    interval_green_count = 0
    interval_orange_count = 0

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

            orange_mask, green_mask = (
                create_color_masks(color_image)
            )

            detections = (
                detect_objects(
                    orange_mask,
                    "ORANGE"
                )
                +
                detect_objects(
                    green_mask,
                    "GREEN"
                )
            )

            # =================================================
            # DEPTH
            # =================================================

            depth_image = (
                np.asanyarray(
                    depth_frame.get_data()
                ).astype(np.float32)
            )

            depth_scale = (
                depth_frame.get_units()
            )

            depth_metres = (
                depth_image * depth_scale
            )

            valid_detections = []

            for detection in detections:

                cx = detection["cx"]
                cy = detection["cy"]

                radius = DEPTH_SAMPLE_RADIUS

                x0 = max(
                    0,
                    cx - radius
                )

                x1 = min(
                    depth_metres.shape[1],
                    cx + radius + 1
                )

                y0 = max(
                    0,
                    cy - radius
                )

                y1 = min(
                    depth_metres.shape[0],
                    cy + radius + 1
                )

                patch = depth_metres[
                    y0:y1,
                    x0:x1
                ]

                valid = patch[
                    (patch >= MIN_VALID_DEPTH)
                    &
                    (patch <= MAX_VALID_DEPTH)
                ]

                depth = (
                    float(np.median(valid))
                    if valid.size
                    else None
                )

                detection["distance"] = depth

                if depth is not None:
                    valid_detections.append(
                        detection
                    )

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
                )

                tracks[next_track_id] = track

                detected_track_ids.add(
                    next_track_id
                )

                # ------------------------------------------------
                # IMPORTANT:
                # A new track represents a newly detected object.
                #
                # We count it ONCE here instead of counting it
                # every frame.
                # ------------------------------------------------

                total_unique_objects += 1

                if detection["color"] == "GREEN":

                    window_green_count += 1
                    interval_green_count += 1

                elif detection["color"] == "ORANGE":

                    window_orange_count += 1
                    interval_orange_count += 1

                next_track_id += 1

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
                del tracks[track_id]

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
                    green_count=interval_green_count,
                    orange_count=interval_orange_count,
                    green_cumulative=window_green_count,
                    orange_cumulative=window_orange_count,
                )

                print(
                    f"[DATA] Window {window_id} | "
                    f"{elapsed_window:6.1f}s | "
                    f"Green +{interval_green_count} "
                    f"(total {window_green_count}) | "
                    f"Orange +{interval_orange_count} "
                    f"(total {window_orange_count})"
                )

                # Reset interval counters.
                interval_green_count = 0
                interval_orange_count = 0

                last_sample_time = current_time

            # =================================================
            # FINISH 3-MINUTE WINDOW
            # =================================================

            if (
                elapsed_window
                >= DATA_COLLECTION_SECONDS
            ):

                window_end_time = current_time

                save_window_summary(
                    window_id=window_id,
                    window_start=window_start_time,
                    window_end=window_end_time,
                    green_total=window_green_count,
                    orange_total=window_orange_count,
                )

                print("\n" + "-" * 65)

                print(
                    f"3-MINUTE WINDOW #{window_id} COMPLETE"
                )

                print(
                    f"Green objects : "
                    f"{window_green_count}"
                )

                print(
                    f"Orange objects: "
                    f"{window_orange_count}"
                )

                print(
                    f"Total objects : "
                    f"{window_green_count + window_orange_count}"
                )

                print(
                    f"Saved to: {SUMMARY_CSV}"
                )

                print("-" * 65 + "\n")

                # ------------------------------------------------
                # Start a NEW 3-minute window.
                # ------------------------------------------------

                window_id += 1

                window_start_time = current_time

                last_sample_time = current_time

                window_green_count = 0
                window_orange_count = 0

                interval_green_count = 0
                interval_orange_count = 0

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
                window_green_count,
                window_orange_count,
            )

            # =================================================
            # DISPLAY
            # =================================================

            if SHOW_MASKS:

                combined_mask = (
                    cv2.bitwise_or(
                        orange_mask,
                        green_mask
                    )
                )

                cv2.imshow(
                    "Color Masks",
                    combined_mask
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

        cv2.destroyAllWindows()

        print("Done.")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()