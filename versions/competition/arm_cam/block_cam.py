"""block_cam.py — detect the 40x30 mm block with the wrist camera ONLY (no arm, no COM port, no calibration needed)

Needs only:  pip install opencv-python numpy
Run:         python block_cam.py              (camera index 1 = OV9281 on this laptop)
             python block_cam.py --cam 0      (another camera)
             python block_cam.py --image frame.png   (test on a saved picture)
Keys:        Esc = quit   s = save snapshot   d = show/hide the mask (what the detector sees)

Shows the block box, centre (u,v px), angle, size in px, and - if cam_focal.json (from calib_focal.py)
sits next to this file - the distance from the lens to the top of the block (mm) and jaw-tip height.

How it finds the block (grey camera, so no colour):
  1) bright pixels       : Otsu threshold, capped so mid-grey (orange) blocks are not lost when white things are in view
  2) smooth pixels only  : the block face is very smooth; textured metal plate / belt / object edges are removed
                           (this also cuts the block free when it touches the jaw, the rail or the shiny plate)
  3) remove thin things  : big round "opening" removes rails, wires, sensor legs
  4) keep rectangles     : filled >= 70 % of its rotated box, length/width 1.0-1.6 (block 40/30 = 1.33)
  5) must sit on the belt: surroundings mostly darker / different from the block, not cut by the picture edge
  6) choose the largest one (near the last position), keep it ~0.3 s if one frame misses (no flicker)
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- camera ----------------
CAM_INDEX = 1
CAM_W, CAM_H, CAM_FPS = 1280, 720, 120
CAM_EXPOSURE = -7                 # Windows: 2^value seconds (-7 = 1/128 s). Image too bright -> -8
# ---------------- detector (same values that were tested on your screenshots) ----------------
BLOCK_LONG_MM, BLOCK_SHORT_MM, BLOCK_H_MM = 40.0, 30.0, 20.0
MIN_AREA_PX = 400
OTSU_CAP_ABOVE_DARK = 90          # threshold <= (dark p10 + this)
SMOOTH_MEDIAN = 5                 # median blur before measuring smoothness (removes the orange block's grain)
SMOOTH_WIN = 9                    # local std window
SMOOTH_SD_MAX = 6.0               # local std above this = texture / edge, not block face
OPEN_BIG_PX = 65                  # removes things thinner than this (rails, wires, sensor legs)
FILL_MIN = 0.70                   # contour area / rotated box area
ASPECT = (1.0, 1.6)               # long/short in the picture
MIN_EQ_PX = 90                    # sqrt(long*short) in px - smaller = reflection / LED
BORDER_PX = 25                    # box must be this far from the picture edge
UNIFORM_ERODE_PX = 25
UNIFORM_STD_MAX = 30.0            # brightness std inside the block face
RING_PX = 12                      # ring around the box used for "is it on the belt?"
RING_DIFF_GRAY = 25
RING_DIFF_FRAC_MIN = 0.60         # ring pixels that differ from the block (darker / textured / clearly other grey)
RING_DARK_MIN = 0.42              # ring pixels darker than the block by >= 35 (dark belt)
DARK_DELTA = 35
WHITE_GRAY = 215                  # face grey >= this -> "white", else "orange?"
HOLD_S = 0.3                      # keep last detection this long when a frame misses


def detect(gray, prefer=None):
    """-> dict(u, v, long_px, short_px, eq_px, angle, gray, tone, box) or None"""
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    t, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cap = float(np.percentile(blur, 10)) + OTSU_CAP_ABOVE_DARK
    if t > cap:
        _, bw = cv2.threshold(blur, cap, 255, cv2.THRESH_BINARY)
    bw_raw = bw

    f = cv2.medianBlur(gray, SMOOTH_MEDIAN).astype(np.float32)
    mu = cv2.blur(f, (SMOOTH_WIN, SMOOTH_WIN))
    lsd = np.sqrt(np.maximum(cv2.blur(f * f, (SMOOTH_WIN, SMOOTH_WIN)) - mu * mu, 0.0))
    bw = cv2.bitwise_and(bw, (lsd <= SMOOTH_SD_MAX).astype(np.uint8) * 255)

    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, k5)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, k5)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_BIG_PX, OPEN_BIG_PX)))
    kg = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * SMOOTH_WIN + 1, 2 * SMOOTH_WIN + 1))
    bw = cv2.bitwise_and(cv2.dilate(bw, kg), bw_raw)            # give back the edge eaten by the smooth mask

    h, w = gray.shape
    pf = np.array(prefer if prefer is not None else (w / 2.0, h / 2.0), np.float32)
    best, best_cost = None, 1e18
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        a = cv2.contourArea(c)
        if a < MIN_AREA_PX:
            continue
        rect = cv2.minAreaRect(c)
        rw, rh = rect[1]
        if rw * rh < 1 or a / (rw * rh) < FILL_MIN:
            continue
        box = cv2.boxPoints(rect).astype(np.float32)
        if box[:, 0].min() < BORDER_PX or box[:, 1].min() < BORDER_PX or \
                box[:, 0].max() > w - BORDER_PX or box[:, 1].max() > h - BORDER_PX:
            continue
        s0, s1 = float(np.linalg.norm(box[1] - box[0])), float(np.linalg.norm(box[2] - box[1]))
        L, S = max(s0, s1), max(1e-6, min(s0, s1))
        eq = math.sqrt(L * S)
        if eq < MIN_EQ_PX or not (ASPECT[0] <= L / S <= ASPECT[1]):
            continue
        m = np.zeros_like(gray)
        cv2.drawContours(m, [c], -1, 255, -1)
        inner = cv2.erode(m, cv2.getStructuringElement(cv2.MORPH_RECT, (UNIFORM_ERODE_PX, UNIFORM_ERODE_PX)))
        if np.count_nonzero(inner) <= 50:
            continue
        face = gray[inner > 0]
        if float(face.std()) > UNIFORM_STD_MAX:
            continue
        med = float(np.median(face))
        ko = 2 * RING_PX + 1
        ring = cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_RECT, (ko, ko))) - \
            cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        rv = gray[ring > 0].astype(np.float32)
        if rv.size <= 50:
            continue
        diff = (np.abs(rv - med) >= RING_DIFF_GRAY) | (lsd[ring > 0] > SMOOTH_SD_MAX)
        if float(diff.mean()) < RING_DIFF_FRAC_MIN or float((rv < med - DARK_DELTA).mean()) < RING_DARK_MIN:
            continue
        ctr = box.mean(axis=0)
        cost = -cv2.contourArea(box) + 50.0 * float(np.linalg.norm(ctr - pf))   # largest, slightly prefer last position
        if cost < best_cost:
            lv = (box[1] - box[0]) if s0 >= s1 else (box[2] - box[1])
            best_cost = cost
            best = {"u": float(ctr[0]), "v": float(ctr[1]), "long_px": L, "short_px": S, "eq_px": eq,
                    "angle": math.degrees(math.atan2(float(lv[1]), float(lv[0]))) % 180.0,
                    "gray": med, "tone": "white" if med >= WHITE_GRAY else "orange?", "box": box}
    return best, bw


# ---------------- optional height from cam_focal.json ----------------
def load_focal():
    try:
        with open(os.path.join(HERE, "cam_focal.json"), encoding="utf-8") as fh:
            d = json.load(fh)
        return float(d["f_px"]), float(d.get("offset_mm", 0.0)), d.get("cam_to_tip_mm")
    except (OSError, ValueError, KeyError):
        return None


def height_text(det, focal):
    if focal is None:
        return []
    f, off, tip = focal
    d = f * math.sqrt(BLOCK_LONG_MM * BLOCK_SHORT_MM) / det["eq_px"] - off
    out = [f"cam -> block top: {d:.0f} mm"]
    if tip is not None:
        dep = float(tip) - d
        out.append(f"jaw tip -> block: {-dep:.0f} mm" + ("  (IN block)" if dep > 0 else ""))
        out.append(f"jaw tip -> belt : {BLOCK_H_MM - dep:.0f} mm")
    return out


# ---------------- camera reader (background thread = window never freezes) ----------------
class Camera:
    def __init__(self, index):
        self.index = index
        self.frame, self.seq, self.t = None, 0, time.perf_counter()
        self.lock = threading.Condition()
        self.cap = self._open()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        backend = cv2.CAP_MSMF if sys.platform.startswith("win") else cv2.CAP_V4L2
        cap = cv2.VideoCapture(self.index, backend)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if sys.platform.startswith("win"):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, CAM_EXPOSURE)
        if not cap.isOpened():
            raise SystemExit(f"cannot open camera {self.index} (try --cam 0 / --cam 2, close other programs using it)")
        return cap

    def _loop(self):
        while self.running:
            ok, fr = self.cap.read()
            if ok and fr is not None:
                with self.lock:
                    self.frame, self.seq, self.t = fr, self.seq + 1, time.perf_counter()
                    self.lock.notify_all()
                continue
            time.sleep(0.01)
            if time.perf_counter() - self.t > 3.0:              # stuck -> reopen
                print("[camera] no frames for 3 s -> reopening", flush=True)
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap, self.t = self._open(), time.perf_counter()

    def read(self, after, timeout=0.25):
        end = time.perf_counter() + timeout
        with self.lock:
            while self.frame is None or self.seq <= after:
                left = end - time.perf_counter()
                if left <= 0:
                    return after, None
                self.lock.wait(left)
            return self.seq, self.frame

    def release(self):
        self.running = False
        time.sleep(0.05)
        self.cap.release()


def draw(vis, det, focal, fps, held):
    if det is not None:
        col = (0, 200, 255) if held else (0, 255, 0)
        cv2.polylines(vis, [det["box"].astype(np.int32)], True, col, 3)
        cv2.drawMarker(vis, (int(det["u"]), int(det["v"])), col, cv2.MARKER_CROSS, 28, 2)
        lines = [f"BLOCK  u={det['u']:.0f} v={det['v']:.0f} px",
                 f"size {det['long_px']:.0f} x {det['short_px']:.0f} px   angle {det['angle']:.0f} deg",
                 f"gray {det['gray']:.0f} ({det['tone']})"] + height_text(det, focal)
    else:
        lines = ["no block"]
    y = 34
    for t in lines + [f"{fps:.0f} fps"]:
        cv2.putText(vis, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5)
        cv2.putText(vis, t, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        y += 32


def main():
    ap = argparse.ArgumentParser(description="camera-only block detector")
    ap.add_argument("--cam", default=None, help="camera index or /dev/videoN (default: 1 on Windows, found by name on the Pi)")
    ap.add_argument("--image", help="detect on a picture file instead of the camera")
    args = ap.parse_args()
    focal = load_focal()
    win = "block_cam (Esc quit, s save, d mask)"
    show_mask = False

    if args.image:
        g = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if g is None:
            raise SystemExit(f"cannot read {args.image}")
        det, bw = detect(g)
        print("FOUND" if det else "no block", {k: (round(v, 1) if isinstance(v, float) else v)
                                               for k, v in (det or {}).items() if k != "box"})
        vis = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        draw(vis, det, focal, 0, False)
        cv2.imshow(win, vis)
        cv2.waitKey(0)
        return

    if args.cam is None:                              # Pi: find the wrist camera by name (never PRISM's overhead camera)
        import pi_env
        args.cam = CAM_INDEX if sys.platform.startswith("win") else pi_env.find_wrist_camera()
        if args.cam is None:
            raise SystemExit(pi_env.camera_help())
    elif str(args.cam).isdigit():
        args.cam = int(args.cam)
    cam = Camera(args.cam)
    seq, last, last_t, last_print = 0, None, 0.0, 0.0
    t_fps, n_fps, fps = time.perf_counter(), 0, 0.0
    try:
        while True:
            seq, fr = cam.read(seq)
            if fr is None:                                   # no new frame yet: keep the window alive
                if cv2.waitKey(10) & 0xFF == 27:
                    break
                continue
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr
            now = time.perf_counter()
            det, bw = detect(g, prefer=(last["u"], last["v"]) if last else None)
            held = False
            if det is not None:
                last, last_t = det, now
            elif last is not None and now - last_t < HOLD_S:
                det, held = last, True                       # one missed frame: keep showing (no flicker)
            else:
                last = None
            n_fps += 1
            if now - t_fps >= 1.0:
                fps, n_fps, t_fps = n_fps / (now - t_fps), 0, now
            if now - last_print >= 0.5:
                last_print = now
                print(("block u=%.0f v=%.0f size %.0fx%.0f px angle %.0f %s" % (det["u"], det["v"], det["long_px"],
                       det["short_px"], det["angle"], " | ".join(height_text(det, focal))) if det else "no block").ljust(110),
                      end="\r", flush=True)
            vis = cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR) if show_mask else cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
            draw(vis, det, focal, fps, held)
            cv2.imshow(win, vis)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
            if k == ord("d"):
                show_mask = not show_mask
            if k == ord("s"):
                fn = os.path.join(HERE, time.strftime("block_%Y%m%d_%H%M%S.png"))
                cv2.imwrite(fn, vis)
                print(f"\nsaved {fn}")
    finally:
        print()
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
