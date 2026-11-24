# check_camera_geometry.py - PASSIVE: measure the camera geometry from what the camera and the pick IR see now.
# Moves NOTHING (it only listens to MQTT). Needs vision and autoen running (autoen publishes the pick IR).
#
#   cd ~/Desktop/PRISM/orchestrator && ~/orch-env/bin/python cal/check_camera_geometry.py
#
# Place ONE cube under the gripper at the pick IR (the IR must read "blocked") and, if you like, a few more cubes along
# the belt. It listens 15 s and prints:
#   - every cube the camera sees (track, colour, position along the belt, width in px)
#   - the position of the cube at the pick IR (the most downstream one while the IR reads blocked) -> ir_along
#   - mm per pixel from the cube widths (40 mm cubes)
#   - a camera_geometry.json suggestion next to the current proposal (camera_geometry.proposed.json)
# Nothing is written: copy the numbers in by hand after checking them.
import json
import os
import statistics
import sys
import time

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))
ORCH = os.path.dirname(HERE)
CUBE_MM = 40.0
LISTEN_S = 15.0

frames, ir_reads = [], []


def on_message(_c, _u, msg):
    try:
        d = json.loads(msg.payload)
    except ValueError:
        return
    if msg.topic == "camera/perception":
        frames.append((time.time(), d))
    elif msg.topic == "belt/telemetry" and d.get("ir") is not None:
        ir_reads.append((time.time(), bool(d["ir"])))


def main():
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="check_camera_geometry")
    c.on_message = on_message
    c.connect("localhost", 1883, 10)
    c.subscribe([("camera/perception", 0), ("belt/telemetry", 0)])
    c.loop_start()
    print(f"listening {LISTEN_S:g} s (camera + pick IR) - nothing moves ...")
    time.sleep(LISTEN_S)
    c.loop_stop()
    if not frames:
        sys.exit("no camera data: is vision running?")
    sign = -1 if int(frames[-1][1].get("dir", 1)) < 0 else 1
    tracks = {}
    for _, d in frames:
        for t in d.get("tracks") or []:
            if (t.get("hits") or 0) >= 3:
                tracks.setdefault(t["id"], []).append(t)
    print(f"\n{len(frames)} camera frames, belt direction {sign:+d} (along = {'u' if sign > 0 else '-u'})")
    rows = []
    for tid, ts in sorted(tracks.items()):
        along = statistics.median(t["u"] for t in ts) * sign
        w = statistics.median(t.get("w") or 0 for t in ts)
        rows.append((along, tid, ts[-1].get("color"), w))
    for along, tid, col, w in sorted(rows):
        print(f"  track {tid:4d}  {str(col):7s} along {along:7.1f} px   width {w:5.1f} px  -> {CUBE_MM / w:.3f} mm/px" if w else "")
    blocked = sum(1 for _, b in ir_reads if b)
    print(f"\npick IR: {blocked} of {len(ir_reads)} readings blocked" if ir_reads else "\npick IR: no readings (autoen not running?)")
    sug = {}
    if rows and ir_reads and blocked > len(ir_reads) / 2:
        sug["ir_along"] = round(max(r[0] for r in rows), 1)
        print(f"  the cube at the IR (most downstream): along {sug['ir_along']} px")
    ws = [r[3] for r in rows if r[3]]
    if ws:
        sug["mm_per_px"] = round(CUBE_MM / statistics.median(ws), 3)
        print(f"  mm per px from {len(ws)} cube widths: {sug['mm_per_px']}")
    try:
        with open(os.path.join(ORCH, "camera_geometry.proposed.json")) as f:
            prop = json.load(f)
        print(f"\nproposal now: ir_along {prop.get('ir_along')}, entry_along {prop.get('entry_along')}, "
              f"mm_per_px {prop.get('mm_per_px')}")
    except (OSError, ValueError):
        pass
    print(f"measured now: {json.dumps(sug)}")
    print("\nNothing was written. If they agree: edit camera_geometry.proposed.json, then rename it to camera_geometry.json"
          " and restart the optimizer.")


if __name__ == "__main__":
    main()
