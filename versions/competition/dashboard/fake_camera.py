#!/usr/bin/env python3
"""fake_camera.py - publishes camera/status, camera/perception (5 Hz), camera/demand (every 10 s) and camera/window
(every 3 min) with the same payloads as 3mins.py, so the Camera page and the camera node health can be tried without the D415."""
import json, math, os, random, time
import paho.mqtt.client as mqtt
host = os.environ.get("MQTT_HOST", "localhost")
c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake_camera")
c.will_set("camera/status", json.dumps({"online": False, "ts": time.time()}), qos=1, retain=True)
c.connect(host, 1883, 10); c.loop_start()
c.publish("camera/status", json.dumps({"online": True, "ts": time.time(), "fps_target": 30, "window_seconds": 180, "sample_interval": 10}), qos=1, retain=True)
win, t0, cum, last_d, tracks, nid = 1, time.time(), 0, time.time(), {}, 0
try:
    while True:
        now = time.time()
        if random.random() < 0.06 and len(tracks) < 5:                   # a new object enters the view
            nid += 1; tracks[nid] = [random.randint(60, 580), 0]
        for k in list(tracks):
            tracks[k][1] += random.randint(8, 22)
            if tracks[k][1] > 470: del tracks[k]; cum += 1
        c.publish("camera/perception", json.dumps({"ts": now, "fps": round(29 + random.random(), 1), "active": len(tracks),
            "tracks": [{"id": k, "u": v[0], "v": v[1], "dist_m": round(0.5 + v[1] / 900, 3)} for k, v in tracks.items()]}))
        if now - last_d >= 10:
            last_d = now; c.publish("camera/demand", json.dumps({"ts": now, "window_id": win, "elapsed_s": round(now - t0, 1), "interval_s": 10, "count": random.randint(2, 9), "cumulative": cum}), qos=1)
        if now - t0 >= 180:
            c.publish("camera/window", json.dumps({"ts": now, "window_id": win, "duration_s": round(now - t0, 1), "total": cum}), qos=1)
            win, t0, cum = win + 1, now, 0
        time.sleep(0.2)
except KeyboardInterrupt:
    c.publish("camera/status", json.dumps({"online": False, "ts": time.time()}), qos=1, retain=True); time.sleep(.3)
