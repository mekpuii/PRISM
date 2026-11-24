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