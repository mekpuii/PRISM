#!/usr/bin/env python3
"""rescue_eye.py - the wrist camera's eye for PRISM's EMERGENCY GRIP (2026-11-12).

The operator: "if the arm's IR still sees the cube after GRIP and the grip failed, open the follow_block function as
the emergency: no camera window, find the cube, grip it, go home, then orange -> left, else -> right".

PRISM's autoen owns the Mega (one port, one owner), so this program never moves the arm. It is the EYE and the BRAIN of
follow_block.py, and autoen is its hands: autoen sends the arm's joints ~12 times a second, this program answers with
follow_block's own corrections, and says READY with the taught grip when the cube sits still under the jaws.
The maths is follow_block.py, unchanged (imported): BlockTracker -> block_error_mm -> plan_follow (Jacobian, height and
pitch kept) -> integrate_goals; centred = within CENTER_PX of the jaw target (follow_target.json) for HOLD_S without
moving more than JITTER_PX; the grip = follow_grip.json (taught at the pick spot: joints down by delta, z checked).
No window, no keyboard. The camera opens at "start" and is released by "close".

Protocol: one JSON object per line on stdin; one JSON reply per line on stdout (all other output goes to stderr).
  {"cmd": "hello"}                       -> {"ok": true, "scan": {joint: tick}, "range": {...}, "limits": {...}}
  {"cmd": "start"}                       -> camera on, tracker reset (searches the whole picture)
  {"cmd": "step", "joints": {name: tick}}-> {"ok": true, "found": bool, "cmds": {name: [goal, speed]}, "err_px": [u, v],
                                             "centred": bool, "ready": bool, "grip": {"down": {...}, "up": {...}}}
                                            {"ok": false, "abort": "..."}  (the arm's height drifted - stop)
  {"cmd": "close"}  camera off            {"cmd": "quit"}
Test without a camera:  --fake   (a cube that appears where the jaws are after 0.5 s and holds still)
"""
import datetime
import json
import os
import sys
import time

OUT = sys.stdout
sys.stdout = sys.stderr                                     # library prints must not mix into the protocol
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import config as C                                          # noqa: E402
import kinematics as K                                      # noqa: E402

HOLD_S = 0.6                  # centred and still this long -> grip (follow_block's --hold default is 1.0)
SHOTS_DIR = os.path.join(os.path.dirname(HERE), "rescue_shots")
SHOTS = SHOTS_DIR   # --shots-dir overrides (the tests use a temp folder)
JPEG_Q = 80
GRIP_FILE = os.path.join(HERE, "follow_grip.json")
TARGET_FILE = os.path.join(HERE, "follow_target.json")


def reply(**kv):
    OUT.write(json.dumps(kv, default=float) + "\n")
    OUT.flush()


class Eye:
    def __init__(self, fake=False):
        self.fake, self.cap, self.H, self.trk = fake, None, None, None
        self.last = None                                    # the newest frame + what was found in it (for the screenshots)
        self.jaw_deg = None
        self.scan = {j: int(C.SCAN_POSE[j]) for j in K.JOINTS5}
        import follow_block as FB
        self.FB = FB
        self.td = json.load(open(GRIP_FILE, encoding="utf-8")) if os.path.exists(GRIP_FILE) else None
        try:
            t = json.load(open(TARGET_FILE, encoding="utf-8"))
            self.tu, self.tv = float(t["u"]), float(t["v"])
            if t.get("claw") and t.get("gripper"):          # the jaw axis in the picture (clicked once in follow_block)
                import math
                self.jaw_deg = math.degrees(math.atan2(t["gripper"][1] - t["claw"][1], t["gripper"][0] - t["claw"][0]))
        except (OSError, ValueError, KeyError):
            self.tu = self.tv = None                        # the picture's centre (set at the first frame)

    def hello(self):
        return {"ok": True, "scan": self.scan, "range": dict(self.FB.RANGE_TICKS),
                "limits": {j: list(C.JOINT_LIMITS[j]) for j in K.JOINTS5}, "grip_taught": self.td is not None,
                "fake": self.fake}

    def start(self):
        import detect_block as D
        if self.H is None:
            self.H = D.load_homography()
        if not self.fake and self.cap is None:
            import camera
            self.cap = camera.open_camera()
        self.trk = D.BlockTracker(self.H)
        self.trk.bg = None                                  # (the empty-lane background is only valid at the scan pose)
        self.ref_zp, self.cmd_goal, self.since, self.hist, self.t0 = None, {}, None, [], time.monotonic()
        return {"ok": True, "grip_taught": self.td is not None}

    def close(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:                               # noqa: BLE001
                pass
        self.cap = None

    def _see(self):
        """-> (centre px (u, v) or None). Also keeps the frame for the screenshots."""
        if self.fake:
            import numpy as np
            g = np.full((720, 1280), 40, np.uint8)          # a grey picture with a bright square where the cube is
            seen = time.monotonic() - self.t0 >= 0.5
            u, v = (self.tu or 640.0) + 3.0, (self.tv or 360.0) - 2.0
            if seen:
                g[int(v) - 40:int(v) + 40, int(u) - 55:int(u) + 55] = 230
            self.last = {"gray": g, "box": None, "c": (u, v) if seen else None, "t": time.time()}
            return (u, v) if seen else None
        import camera
        g = camera.grab(self.cap)
        if g is None:
            return None
        if self.tu is None:
            self.tu, self.tv = g.shape[1] / 2.0, g.shape[0] / 2.0
        res, state, _roi = self.trk.update(g)
        found = res is not None and state == "TRACK"
        self.last = {"gray": g, "box": res["box_px"] if found else None, "c": None, "t": time.time()}
        if not found:
            return None
        c = res["box_px"].mean(axis=0)
        self.last["c"] = (float(c[0]), float(c[1]))
        return float(c[0]), float(c[1])

    def _cube_deg(self):
        """The cube's turn in the picture (0 = a side parallel to the picture's x axis, -45..45) and, if the jaws were
        clicked once in follow_block, how far that is from the jaw axis. Logged with the screenshots: it tells how the
        cube lay when the normal grip missed (it is NOT used to move the wrist yet)."""
        import math
        box = (self.last or {}).get("box")
        if box is None:
            return None, None
        d = math.degrees(math.atan2(box[1][1] - box[0][1], box[1][0] - box[0][0]))
        cube = (d + 45.0) % 90.0 - 45.0
        off = None if self.jaw_deg is None else (cube - self.jaw_deg + 45.0) % 90.0 - 45.0
        return round(cube, 1), None if off is None else round(off, 1)

    def shot(self, tag, run=None, note="", shots_dir=SHOTS_DIR):
        """Save the newest hand-camera frame as a JPEG with the date, time, emergency number, event and what was found
        written on it. File: <dir>/<YYYYMMDD_HHMMSS>_run<N>_<tag>.jpg. Returns {"ok", "file", "cube_deg", "jaw_off_deg"}."""
        import cv2
        if self.trk is not None and (self.fake or self.cap is not None):
            self._see()                                     # a fresh frame (what the camera sees right now)
        if self.last is None:
            return {"ok": False, "why": "no picture yet"}
        g = self.last["gray"]
        img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        now = datetime.datetime.now()
        if self.last.get("box") is not None:
            cv2.polylines(img, [self.last["box"].astype("int32")], True, (0, 255, 0), 2)
        if self.last.get("c") is not None:
            cv2.drawMarker(img, (int(self.last["c"][0]), int(self.last["c"][1])), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        if self.tu is not None:
            cv2.drawMarker(img, (int(self.tu), int(self.tv)), (0, 255, 255), cv2.MARKER_DIAMOND, 22, 2)
        cube, off = self._cube_deg()
        lines = [f"{now:%Y-%m-%d %H:%M:%S}  EMERGENCY #{run if run is not None else '?'}  {tag}"]
        if note:
            lines.append(str(note)[:110])
        if cube is not None:
            lines.append(f"cube turned {cube:+.0f} deg in the picture" + (f", {off:+.0f} deg from the jaw axis" if off is not None else ""))
        for i, t in enumerate(lines):
            cv2.putText(img, t, (12, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(img, t, (12, 30 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        os.makedirs(shots_dir, exist_ok=True)
        name = f"{now:%Y%m%d_%H%M%S}_run{run if run is not None else 'x'}_{tag}.jpg"
        cv2.imwrite(os.path.join(shots_dir, name), img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        return {"ok": True, "file": name, "cube_deg": cube, "jaw_off_deg": off}

    def step(self, joints):
        FB = self.FB
        if self.trk is None or (not self.fake and self.cap is None):
            return {"ok": False, "abort": "the hand camera is not open (start failed)", "no_camera": True}
        c = self._see()
        if c is None:
            self.cmd_goal.clear()
            self.since, self.hist = None, []
            return {"ok": True, "found": False, "cmds": {}, "centred": False, "ready": False}
        tu, tv = (self.tu or 640.0), (self.tv or 360.0)
        cmds = {}
        if not self.fake:
            e = FB.block_error_mm(self.H, c, (tu, tv))
            if self.ref_zp is None:
                ts = FB.tip_state(joints)
                self.ref_zp = (float(ts[2]), float(ts[3]))
            _dq, cmds = FB.plan_follow(joints, self.scan, e, ref_zp=self.ref_zp)
            dz = float(FB.tip_state(joints)[2] - self.ref_zp[0])
            if abs(dz) > FB.Z_ABORT_MM:
                return {"ok": False, "abort": f"the hand's height drifted {dz:+.0f} mm while following"}
            near = ((c[0] - tu) ** 2 + (c[1] - tv) ** 2) ** 0.5 <= FB.HOLD_PX
            cmds = {} if near else FB.integrate_goals(cmds, joints, self.scan, self.cmd_goal)
        centred = abs(c[0] - tu) <= FB.CENTER_PX and abs(c[1] - tv) <= FB.CENTER_PX
        out = {"ok": True, "found": True, "cmds": {j: [int(g), int(s)] for j, (g, s) in cmds.items()},
               "err_px": [round(c[0] - tu), round(c[1] - tv)], "centred": centred, "ready": False}
        now = time.monotonic()
        if not centred:
            self.since, self.hist = None, []
            return out
        if self.since is None:
            self.since, self.hist = now, []
        self.hist.append(c)
        us, vs = zip(*self.hist)
        if max(us) - min(us) > FB.JITTER_PX or max(vs) - min(vs) > FB.JITTER_PX:
            self.since, self.hist = now, [c]                # still moving: start again
            return out
        if now - self.since < HOLD_S:
            return out
        if self.td is None:
            return {"ok": False, "abort": "no taught grip (follow_grip.json): teach it once with follow_block.py, key t"}
        down = {n: int(joints[n] + self.td["delta_joints"][n]) for n in FB.GRIP_J}
        for n, v in down.items():
            lo, hi = C.JOINT_LIMITS[n]
            if not lo <= v <= hi:
                return {"ok": False, "abort": f"the grip pose is outside the joint limits: {n}={v}"}
        z_dn = float(FB.tip_state({**joints, **down})[2])
        if z_dn < self.td["grip_z_mm"] - FB.GRIP_Z_TOL_MM:
            return {"ok": False, "abort": f"the grip pose would go {self.td['grip_z_mm'] - z_dn:.0f} mm lower than "
                                          "taught - it could hit the belt"}
        out.update(ready=True, grip={"down": down, "up": {n: int(joints[n]) for n in FB.GRIP_J},
                                     "drop_mm": round(float(self.td["drop_mm"]), 1)})
        return out


def main():
    global SHOTS
    SHOTS = SHOTS_DIR
    if "--shots-dir" in sys.argv:
        SHOTS = os.path.abspath(sys.argv[sys.argv.index("--shots-dir") + 1])
    eye = Eye(fake="--fake" in sys.argv)
    while True:
        line = sys.stdin.readline()                     # line by line (a for-loop over stdin reads ahead in blocks)
        if not line:
            break
        try:
            m = json.loads(line)
            cmd = m.get("cmd")
            if cmd == "hello":
                reply(**eye.hello())
            elif cmd == "start":
                reply(**eye.start())
            elif cmd == "shot":
                reply(**eye.shot(str(m.get("tag") or "shot"), m.get("run"), m.get("note") or "", SHOTS))
            elif cmd == "step":
                reply(**eye.step({j: int(v) for j, v in (m.get("joints") or eye.scan).items()}))
            elif cmd == "close":
                eye.close()
                reply(ok=True)
            elif cmd == "quit":
                eye.close()
                reply(ok=True)
                break
            else:
                reply(ok=False, why=f"unknown command {cmd!r}")
        except Exception as e:                              # noqa: BLE001 - one bad step must not end the eye
            import traceback
            traceback.print_exc()
            reply(ok=False, abort=f"{type(e).__name__}: {e}")
    eye.close()


if __name__ == "__main__":
    main()
