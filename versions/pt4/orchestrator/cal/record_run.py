#!/usr/bin/env python3
"""record_run.py - log every topic the optimizer needs into one JSONL file.

Usage:  python3 record_run.py --out runs/run3.jsonl --rpm 30 --note "belt 30 rpm, 6 cubes"
Stop with Ctrl-C.  Run it on the Pi while the normal system (camera + autoen) is running.

One line per MQTT message:
  {"rt": <wall clock at receipt>, "mono": <monotonic at receipt>, "topic": ..., "retained": bool, "data": <parsed JSON or raw str>}
First line is a header: {"header": {...}} with the rpm you set and a note.
We stamp OUR receive time ("rt"), never trust payload "ts" across machines: the camera and
Mega clocks are different clocks. Pi-local time keeps all topics on one time axis.

Load on the Pi: ~40 msgs/s, tiny. Each line is flushed so a power cut loses < 1 s.

experiment.py uses the Recorder class below for its session files and adds its own lines with
write_event() (topic "exp/...", same line format), so plan / config / results share the time axis.
"""
import argparse
import json
import os
import signal
import threading
import time

import paho.mqtt.client as mqtt

TOPICS = [
    ("camera/perception", 0), ("belt/telemetry", 0), ("belt/event", 1),
    ("arm/telemetry", 0), ("arm/state", 0), ("arm/gripper", 1), ("arm/ir", 1),
    ("autoen/state", 0), ("autoen/cycle", 1), ("analyzer/arm/#", 0), ("health/status", 0),
]


class Recorder:
    """One JSONL file: header line, then every MQTT message on TOPICS, plus write_event() lines.
    mode="a" keeps the old record_run behaviour; experiment.py passes mode="x" (never overwrite a session)."""

    def __init__(self, path, header, host="localhost", port=1883, mode="a"):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path, self.host, self.port = path, host, port
        self.f = open(path, mode, buffering=1, encoding="utf-8")        # line-buffered
        self.lock = threading.Lock()                                   # MQTT thread + write_event() callers
        self.counts, self.connected, self.client = {}, False, None
        self.f.write(json.dumps({"header": header}) + "\n")

    def _line(self, topic, data, retained=False):
        line = json.dumps({"rt": time.time(), "mono": time.monotonic(), "topic": topic,
                           "retained": retained, "data": data}) + "\n"
        with self.lock:
            if not self.f.closed:
                self.f.write(line)
            self.counts[topic] = self.counts.get(topic, 0) + 1

    def write_event(self, topic, data):
        self._line(topic, data)

    def on_connect(self, c, ud, flags, rc, *rest):
        # (re)subscribe on every connect so a broker restart does not silently stop the log
        for t, q in TOPICS:
            c.subscribe(t, q)
        self.connected = True
        print("connected, recording ->", self.path)

    def on_message(self, c, ud, m):
        try:
            data = json.loads(m.payload)
        except Exception:
            data = m.payload.decode("utf-8", "replace")        # keep it, summarize will skip it
        self._line(m.topic, data, bool(m.retain))

    def start(self):
        try:                                                        # paho 2.x needs the version arg
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:                                      # paho 1.x
            c = mqtt.Client()
        c.on_connect, c.on_message = self.on_connect, self.on_message
        c.connect(self.host, self.port, 30)
        c.loop_start()
        self.client = c

    def stop(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
        with self.lock:
            self.f.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rpm", type=float, default=None, help="belt rpm you set on the autoen page")
    ap.add_argument("--note", default="")
    a = ap.parse_args()

    rec = Recorder(a.out, {"rt": time.time(), "rpm_set": a.rpm, "note": a.note}, a.host, a.port)
    rec.start()

    stop = []
    signal.signal(signal.SIGINT, lambda *x: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *x: stop.append(1))
    t0 = time.time()
    while not stop:
        time.sleep(5)
        print(f"{time.time()-t0:6.0f}s  " + "  ".join(f"{k}:{v}" for k, v in sorted(rec.counts.items())))
    rec.stop()
    print("saved", a.out)


if __name__ == "__main__":
    main()
