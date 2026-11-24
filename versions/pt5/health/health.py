#!/usr/bin/env python3
"""
HEALTH MONITOR - tells you, within seconds, whether each PRISM node is alive AND still
producing the data you expect. Passive: it only subscribes and publishes health/status.
It NEVER commands an actuator.

    python3 health_monitor.py              a line on every state change + a one-line summary every 10 s
    python3 health_monitor.py --table      live table instead (redrawn every second)

Statuses:  UNKNOWN  just started, nothing seen yet (short grace period)
           OK       every expected signal arrives on time
           STALE    something is late            (node may still be alive)
           OFFLINE  something is missing for a long time, or the broker announced the node died
           RECOVERED  was STALE/OFFLINE, everything is back (shown ~3 s, then OK)

Two kinds of evidence per node:
  1. DATA signals   - the topics the node publishes anyway (arm/state, camera/perception, ...).
                      Age is measured by the time THIS program received them, never by the
                      "ts" inside the payload (the Pi clock can be wrong).
  2. STATUS topic   - <node>/status with a Last Will. If the node crashes or loses power the
                      broker publishes online:false and we mark OFFLINE immediately.

Health is not data quality: camera count = 0 is still OK. Only a MISSING message is a problem.

Install:  pip install paho-mqtt        Broker: MQTT_HOST env var (default localhost)
"""

import argparse
import json
import os
import sys
import threading
import time

import paho.mqtt.client as mqtt

BROKER_HOST = os.environ.get("MQTT_HOST", "localhost")
BROKER_PORT = 1883

# ----------------------------------------------------------------------------
# WHAT TO WATCH.  signal topic -> (seconds until STALE, seconds until OFFLINE)
# The numbers come from how often each node really publishes:
#   arm/state        10 Hz always            -> the arm node's control loop is running
#   arm/telemetry    10 Hz, ONLY while all 6 servos answer -> the Mega/servo link works
#   camera/perception 2 Hz                   -> the camera loop is running
#   camera/demand    every 10 s              -> the demand pipeline works (also sent when count = 0)
#   logger/<x>/heartbeat every 2 s           -> the logger's WRITER loop is running
# Change them here only. Tune after you have watched the real rates.
# ----------------------------------------------------------------------------
NODES = {
    "arm": {
        "status": "arm/status",
        "signals": {"arm/state": (1.0, 5.0), "arm/telemetry": (1.0, 5.0)},
    },
    "camera": {
        "status": "camera/status",
        "signals": {"camera/perception": (3.0, 10.0), "camera/demand": (25.0, 45.0)},
    },
    "logger_arm": {
        "status": "logger/arm/status",
        "signals": {"logger/arm/heartbeat": (6.0, 15.0)},
    },
    "logger_camera": {
        "status": "logger/camera/status",
        "signals": {"logger/camera/heartbeat": (6.0, 15.0)},
    },
}

HEALTH_TOPIC = "health/status"
EVAL_EVERY_S = 0.5          # how often statuses are recomputed
PUBLISH_EVERY_S = 2.0       # health/status refresh even if nothing changed
RECOVERED_HOLD_S = 3.0      # how long RECOVERED is shown before OK

UNKNOWN, OK, RECOVERED, STALE, OFFLINE = "UNKNOWN", "OK", "RECOVERED", "STALE", "OFFLINE"
SEVERITY = {OK: 0, RECOVERED: 0, UNKNOWN: 1, STALE: 2, OFFLINE: 3}


class NodeState:
    """Everything we know about one node. Written by the MQTT thread, read by the main loop."""

    def __init__(self):
        self.last_rx = {}        # signal topic -> time.monotonic() of last LIVE message
        self.counts = {}         # signal topic -> messages received
        self.offline_flag = None  # set when <node>/status says online:false
        self.shown = UNKNOWN     # status we currently display
        self.since = time.monotonic()
        self.rec_until = 0.0
        self.reason = "waiting for first message"


class Monitor:
    def __init__(self, scale, grace):
        self.scale = scale           # multiplies every timeout (tests / quick demos)
        self.grace = grace           # seconds after start before "never seen" becomes OFFLINE
        self.lock = threading.Lock()
        self.nodes = {name: NodeState() for name in NODES}
        self.started = time.monotonic()
        self.connected = False

    # ---------------------------------------------------------------- MQTT side
    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[health] connected to broker: {reason_code}", flush=True)
        topics = []
        for cfg in NODES.values():
            topics.append((cfg["status"], 0))
            topics.extend((t, 0) for t in cfg["signals"])
        client.subscribe(topics)
        with self.lock:
            # After a broker outage we heard nothing, so every node would look dead.
            # That is OUR blindness, not their failure: forget old timestamps, start over.
            self.started = time.monotonic()
            self.connected = True
            for st in self.nodes.values():
                st.last_rx.clear()
                st.shown, st.since = UNKNOWN, time.monotonic()
                st.reason = "waiting for first message"

    def on_connect_fail(self, client, userdata):
        # connect_async retries forever in a background thread and never raises, so without
        # this callback a dead / wrong broker address produced NO output at all.
        print(f"[health] cannot reach the broker at {BROKER_HOST}:{BROKER_PORT} "
              f"(is mosquitto running? is MQTT_HOST right?) - retrying", flush=True)

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        with self.lock:
            self.connected = False
        print("[health] lost the broker: health is UNKNOWN until it is back", flush=True)

    def on_message(self, client, userdata, msg):
        now = time.monotonic()
        with self.lock:
            for name, cfg in NODES.items():
                st = self.nodes[name]
                if msg.topic in cfg["signals"]:
                    # A retained message is a stored copy replayed when we subscribe. It says
                    # nothing about the node being alive NOW, so it must not count.
                    if msg.retain:
                        continue
                    st.last_rx[msg.topic] = now
                    st.counts[msg.topic] = st.counts.get(msg.topic, 0) + 1
                elif msg.topic == cfg["status"]:
                    try:
                        online = bool(json.loads(msg.payload).get("online"))
                    except (ValueError, AttributeError):
                        continue
                    st.offline_flag = None if online else time.time()

    # ------------------------------------------------------------ evaluation
    def raw_status(self, name, now):
        """What the evidence says RIGHT NOW: (status, reason). Caller holds the lock."""
        cfg, st = NODES[name], self.nodes[name]
        if st.offline_flag is not None:
            return OFFLINE, f"{cfg['status']} says offline (last will or clean exit)"
        worst, why = OK, ""
        for topic, (stale_s, offline_s) in cfg["signals"].items():
            last = st.last_rx.get(topic)
            if last is None:
                if now - self.started < self.grace:
                    level, text = UNKNOWN, f"{topic}: no message yet"
                else:
                    level, text = OFFLINE, f"{topic}: never seen"
            else:
                age = now - last
                if age > offline_s * self.scale:
                    level, text = OFFLINE, f"{topic} silent {age:.1f} s"
                elif age > stale_s * self.scale:
                    level, text = STALE, f"{topic} silent {age:.1f} s"
                else:
                    level, text = OK, ""
            if SEVERITY[level] > SEVERITY[worst]:
                worst, why = level, text
        return worst, why

    def step(self):
        """Recompute every node. Returns a list of (name, old, new, reason) changes."""
        now = time.monotonic()
        changes = []
        with self.lock:
            if not self.connected:
                return changes
            for name in NODES:
                st = self.nodes[name]
                raw, why = self.raw_status(name, now)
                old = st.shown
                if raw == OK:
                    if old in (STALE, OFFLINE):
                        new = RECOVERED                     # came back: say so for a moment
                        st.rec_until = now + RECOVERED_HOLD_S
                    elif old == RECOVERED and now < st.rec_until:
                        new = RECOVERED
                    else:
                        new = OK
                else:
                    new = raw
                st.reason = why
                if new != old:
                    st.shown, st.since = new, now
                    changes.append((name, old, new, why))
        return changes

    def snapshot(self):
        """JSON-able view of all nodes (for health/status and --table)."""
        now = time.monotonic()
        out = {}
        with self.lock:
            for name, st in self.nodes.items():
                age = min((now - t for t in st.last_rx.values()), default=None)
                out[name] = {
                    "status": st.shown,
                    "reason": st.reason,
                    "age_s": None if age is None else round(age, 1),
                    "in_status_s": round(now - st.since, 1),
                    "messages": sum(st.counts.values()),
                }
        return out


def draw_table(snap, snap_connected=True):
    os.system("cls" if os.name == "nt" else "clear")
    broker = "connected" if snap_connected else "NOT CONNECTED (nothing below is real)"
    print(time.strftime("%H:%M:%S") + f"   PRISM node health   broker {BROKER_HOST}:{BROKER_PORT}: {broker}\n")
    print(f"{'NODE':<15}{'STATUS':<11}{'LAST MSG':<11}{'MESSAGES':<10}REASON")
    print("-" * 70)
    for name, s in snap.items():
        age = "-" if s["age_s"] is None else f"{s['age_s']:.1f} s"
        print(f"{name.upper():<15}{s['status']:<11}{age:<11}{s['messages']:<10}{s['reason']}")


def main():
    p = argparse.ArgumentParser(description="PRISM health monitor (passive)")
    p.add_argument("--table", action="store_true", help="live table instead of change lines")
    p.add_argument("--grace", type=float, default=15.0,
                   help="seconds after start before a never-seen signal counts as OFFLINE")
    p.add_argument("--scale", type=float, default=1.0, help="multiply all timeouts (tests/demos)")
    args = p.parse_args()

    mon = Monitor(args.scale, args.grace)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="health_monitor")
    # Last Will: if THIS program dies, readers see monitor_online=false instead of stale "OK".
    client.will_set(HEALTH_TOPIC, json.dumps({"monitor_online": False}), qos=1, retain=True)
    client.on_connect, client.on_message, client.on_disconnect = mon.on_connect, mon.on_message, mon.on_disconnect
    client.on_connect_fail = mon.on_connect_fail
    client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=10)
    client.loop_start()
    print(f"[health] watching {', '.join(NODES)} via {BROKER_HOST}:{BROKER_PORT}. Ctrl+C to stop.", flush=True)

    last_pub = 0.0
    last_table = 0.0
    last_summary = time.monotonic()      # first summary 10 s after start
    SUMMARY_EVERY_S = 10.0
    try:
        while True:
            time.sleep(EVAL_EVERY_S)
            changes = mon.step()
            for name, old, new, why in changes:
                line = f"[{time.strftime('%H:%M:%S')}] {name.upper():<14}{old} -> {new}"
                print(line + (f"   ({why})" if why else ""), flush=True)

            now = time.monotonic()
            if changes or now - last_pub >= PUBLISH_EVERY_S:
                last_pub = now
                payload = {"monitor_online": True, "ts": time.time(), "nodes": mon.snapshot()}
                client.publish(HEALTH_TOPIC, json.dumps(payload), qos=0, retain=True)

            if not args.table and now - last_summary >= SUMMARY_EVERY_S:
                last_summary = now
                if mon.connected:
                    snap = mon.snapshot()
                    print(f"[{time.strftime('%H:%M:%S')}] " +
                          "  ".join(f"{n}={s['status']}" for n, s in snap.items()), flush=True)
                else:
                    print(f"[{time.strftime('%H:%M:%S')}] NOT connected to the broker - "
                          f"no health information", flush=True)

            if args.table and now - last_table >= 1.0:
                last_table = now
                draw_table(mon.snapshot(), mon.connected)
    except KeyboardInterrupt:
        pass
    finally:
        try:   # raises if the broker was never reachable; shutdown must still finish
            client.publish(HEALTH_TOPIC, json.dumps({"monitor_online": False}),
                           qos=1, retain=True).wait_for_publish(1.0)
        except Exception:
            pass
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    sys.exit(main())