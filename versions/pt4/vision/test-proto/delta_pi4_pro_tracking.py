"""
DELTA - Lightweight Object Perception for Raspberry Pi 4
---------------------------------------------------------
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

Requirements:
    - Python 3.11
    - numpy
    - opencv-python
    - pyrealsense2

Run from the realsense311 environment. If needed:
    export PYTHONPATH="$HOME/librealsense/build/Release"
    python delta_pi4_color_tracking.py
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import math
import time
import traceback


# ============================================================
# CONFIGURATION
# ============================================================

WIDTH = 640
HEIGHT = 480

# The standalone D415 test succeeded at 6 FPS for both streams.
FPS = 6

# Frame timeout in milliseconds. A timeout is reported clearly.
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

ORANGE_LOWER = np.array([5, 100, 80], dtype=np.uint8)
ORANGE_UPPER = np.array([25, 255, 255], dtype=np.uint8)

GREEN_LOWER = np.array([35, 60, 50], dtype=np.uint8)
GREEN_UPPER = np.array([90, 255, 255], dtype=np.uint8)

MORPH_KERNEL = np.ones((5, 5), np.uint8)


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

        self.kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.diag(
            [4.0, 4.0, 0.03]
        ).astype(np.float32)
        self.kalman.errorCovPost = np.eye(6, dtype=np.float32)

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

def get_robust_depth(depth_frame, cx, cy, radius=DEPTH_SAMPLE_RADIUS):
    """Median of valid depths in a small square around the centroid."""
    # Read the depth image once for this call and convert to metres.
    depth_image = np.asanyarray(depth_frame.get_data())
    x_min = max(0, int(cx) - radius)
    x_max = min(depth_image.shape[1], int(cx) + radius + 1)
    y_min = max(0, int(cy) - radius)
    y_max = min(depth_image.shape[0], int(cy) + radius + 1)

    patch = depth_image[y_min:y_max, x_min:x_max]
    if patch.size == 0:
        return None

    # RealSense Z16 values are in device depth units. Use the frame's
    # scale so this remains correct for the connected device.
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

    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    orange_mask = cv2.morphologyEx(
        orange_mask, cv2.MORPH_OPEN, MORPH_KERNEL
    )
    green_mask = cv2.morphologyEx(
        green_mask, cv2.MORPH_OPEN, MORPH_KERNEL
    )
    orange_mask = cv2.morphologyEx(
        orange_mask, cv2.MORPH_CLOSE, MORPH_KERNEL
    )
    green_mask = cv2.morphologyEx(
        green_mask, cv2.MORPH_CLOSE, MORPH_KERNEL
    )

    return orange_mask, green_mask


def detect_objects(mask, color_name):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
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

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])

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
    return math.hypot(x1 - x2, y1 - y2)


def match_detections_to_tracks(detections, tracks):
    """Greedy nearest-neighbor matching, restricted by color."""
    candidate_pairs = []

    for det_index, det in enumerate(detections):
        for track_id, track in tracks.items():
            if track.color != det["color"]:
                continue

            px, py = track.position()
            distance = distance_2d(det["cx"], det["cy"], px, py)

            if distance <= MAX_MATCH_DISTANCE:
                candidate_pairs.append((distance, det_index, track_id))

    candidate_pairs.sort(key=lambda item: item[0])

    matches = []
    used_detections = set()
    used_tracks = set()

    for _, det_index, track_id in candidate_pairs:
        if det_index in used_detections or track_id in used_tracks:
            continue
        matches.append((det_index, track_id))
        used_detections.add(det_index)
        used_tracks.add(track_id)

    unmatched_detections = [
        i for i in range(len(detections)) if i not in used_detections
    ]
    unmatched_tracks = [
        track_id for track_id in tracks if track_id not in used_tracks
    ]

    return matches, unmatched_detections, unmatched_tracks


# ============================================================
# DRAWING
# ============================================================

def draw_track(frame, track, detected_this_frame):
    x, y = track.position()
    distance = track.distance()

    box_color = (0, 140, 255) if track.color == "ORANGE" else (0, 255, 0)

    cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
    cv2.line(frame, (x - 10, y), (x + 10, y), box_color, 2)
    cv2.line(frame, (x, y - 10), (x, y + 10), box_color, 2)

    status = "" if detected_this_frame else " PRED"
    label = f"ID {track.id} | {track.color} | {distance:.2f} m{status}"

    cv2.putText(
        frame, label, (max(5, x - 70), max(20, y - 15)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2, cv2.LINE_AA
    )


def draw_detection_box(frame, detection):
    x, y, w, h = detection["bbox"]
    color = (0, 140, 255) if detection["color"] == "ORANGE" else (0, 255, 0)

    cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
    cv2.circle(frame, (detection["cx"], detection["cy"]), 4, (255, 0, 0), -1)


def draw_dashboard(frame, tracks, total_unique_objects, fps):
    orange_count = sum(t.color == "ORANGE" for t in tracks.values())
    green_count = sum(t.color == "GREEN" for t in tracks.values())

    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (300, 145), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)

    lines = [
        "DELTA PERCEPTION",
        f"Orange: {orange_count}",
        f"Green:  {green_count}",
        f"Active: {orange_count + green_count}",
        f"Unique: {total_unique_objects}",
        f"FPS:    {fps:.1f}",
    ]

    for i, line in enumerate(lines):
        cv2.putText(
            frame, line, (20, 35 + i * 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255),
            1, cv2.LINE_AA
        )


# ============================================================
# CAMERA INITIALIZATION
# ============================================================

def start_camera():
    """Start matching low-rate streams and verify that frames arrive."""
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS
    )
    config.enable_stream(
        rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS
    )

    print(f"\nStarting RealSense D415 at {WIDTH}x{HEIGHT}, {FPS} FPS...")
    profile = pipeline.start(config)

    try:
        for i in range(WARMUP_FRAMES):
            frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            if not frames.get_color_frame() or not frames.get_depth_frame():
                raise RuntimeError("Camera started, but a color or depth frame is missing.")
            print(f"Camera warm-up: {i + 1}/{WARMUP_FRAMES}", flush=True)
    except Exception:
        pipeline.stop()
        raise

    return pipeline, profile


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("DELTA - Raspberry Pi 4 Object Perception")
    print("=" * 60)
    print("Mode: HSV color detection + RealSense depth + Kalman")
    print("Colors: ORANGE / GREEN")
    print(f"Camera streams: {WIDTH}x{HEIGHT} at {FPS} FPS")
    print("Press ESC or Q to exit.")
    print("=" * 60)

    pipeline = None
    try:
        pipeline, _profile = start_camera()
    except Exception as exc:
        print("\nERROR: RealSense did not deliver the requested streams.")
        print(f"Details: {exc}")
        print("\nCheck that RealSense Viewer is closed, then try again.")
        print("The standalone test succeeded at 6 FPS, so this error may")
        print("indicate a stream-startup or device-access issue.")
        traceback.print_exc()
        return

    align = rs.align(rs.stream.color)
    tracks = {}
    next_track_id = 1
    total_unique_objects = 0
    previous_time = time.monotonic()
    fps_display = 0.0

    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(FRAME_TIMEOUT_MS)
            except RuntimeError as exc:
                print(f"\nFrame timeout/error: {exc}")
                print("No new frames arrived within the configured timeout.")
                break

            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                print("Warning: received a frameset missing color or depth; skipping.")
                continue

            color_image = np.asanyarray(color_frame.get_data())

            current_time = time.monotonic()
            dt = current_time - previous_time
            previous_time = current_time

            if dt > 0:
                instant_fps = 1.0 / dt
                fps_display = (
                    0.9 * fps_display + 0.1 * instant_fps
                    if fps_display > 0 else instant_fps
                )

            for track in tracks.values():
                track.predict(dt)

            orange_mask, green_mask = create_color_masks(color_image)
            detections = (
                detect_objects(orange_mask, "ORANGE")
                + detect_objects(green_mask, "GREEN")
            )

            # Read the aligned depth image once, then sample each detection.
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_scale = depth_frame.get_units()
            depth_metres = depth_image * depth_scale

            valid_detections = []
            for detection in detections:
                cx, cy = detection["cx"], detection["cy"]
                radius = DEPTH_SAMPLE_RADIUS
                x0 = max(0, cx - radius)
                x1 = min(depth_metres.shape[1], cx + radius + 1)
                y0 = max(0, cy - radius)
                y1 = min(depth_metres.shape[0], cy + radius + 1)

                patch = depth_metres[y0:y1, x0:x1]
                valid = patch[
                    (patch >= MIN_VALID_DEPTH) &
                    (patch <= MAX_VALID_DEPTH)
                ]

                depth = float(np.median(valid)) if valid.size else None
                detection["distance"] = depth

                if depth is not None:
                    valid_detections.append(detection)

            matches, unmatched_detections, unmatched_tracks = (
                match_detections_to_tracks(valid_detections, tracks)
            )
            detected_track_ids = set()

            for det_index, track_id in matches:
                detection = valid_detections[det_index]
                tracks[track_id].update(
                    detection["cx"], detection["cy"], detection["distance"]
                )
                detected_track_ids.add(track_id)

            for det_index in unmatched_detections:
                detection = valid_detections[det_index]
                track = ObjectTrack(
                    next_track_id, detection["color"], detection["cx"],
                    detection["cy"], detection["distance"]
                )
                tracks[next_track_id] = track
                detected_track_ids.add(next_track_id)
                total_unique_objects += 1
                next_track_id += 1

            for track_id in unmatched_tracks:
                if track_id in tracks:
                    tracks[track_id].mark_missed()

            expired_ids = [
                track_id for track_id, track in tracks.items()
                if track.missed_frames > MAX_MISSED_FRAMES
            ]
            for track_id in expired_ids:
                del tracks[track_id]

            annotated = color_image.copy()
            for detection in valid_detections:
                draw_detection_box(annotated, detection)

            for track_id, track in tracks.items():
                draw_track(annotated, track, track_id in detected_track_ids)

            draw_dashboard(annotated, tracks, total_unique_objects, fps_display)

            if SHOW_MASKS:
                combined_mask = cv2.bitwise_or(orange_mask, green_mask)
                cv2.imshow("Color Masks", combined_mask)

            cv2.imshow(WINDOW_NAME, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception:
        print("\nRuntime error during processing:")
        traceback.print_exc()
    finally:
        print("\nStopping RealSense...")
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception as exc:
                print(f"Camera shutdown note: {exc}")

        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
