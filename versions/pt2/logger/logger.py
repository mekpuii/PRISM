"""
LOGGER NODE - passive subscriber. Records messages, never decides, never publishes
(except its own connection).
 
ONE program, SEPARATE databases: you choose which topics a running instance
records and which database file it writes to. Run one instance per database.
 
    python3 logger_node.py --topics arm    --db arm.db
    python3 logger_node.py --topics camera --db camera.db
 
    (no arguments = everything into logger.db, the old single-database behaviour)
 
Camera display: a camera instance prints the white-object table live, one row per
10 s sample, and a total when the 3-minute window completes. To look at stored data:
 
    python3 logger_node.py --db camera.db --show              (latest window)
    python3 logger_node.py --db camera.db --show --window 3   (a specific window)
 
Dashboards (read-only, no MQTT, never touches the recording instances):

    python3 logger_node.py --db camera.db --dashboard            (one snapshot)
    python3 logger_node.py --db arm.db    --dashboard --watch 2  (refresh every 2 s)

    The dashboard is chosen by what is INSIDE the database: camera tables -> camera
    dashboard, telemetry table -> arm dashboard.

Health:  each instance also publishes logger/<name>/status (retained + Last Will) and
         logger/<name>/heartbeat every 2 s, so health_monitor.py can tell when logging stops.

Flow:  broker --(MQTT)--> on_message (network thread) --> queue --> batch writer --> SQLite
 
Install:  pip install paho-mqtt        (SQLite is built into Python)
Broker:   MQTT_HOST env var = broker address (default localhost)
Inspect:  sqlite3 arm.db    "select * from telemetry order by id desc limit 10;"
          sqlite3 camera.db "select * from camera_demand order by id desc limit 5;"
"""
 
import argparse
import json
import os
import queue
import signal
import sqlite3
import sys
import time
 
import paho.mqtt.client as mqtt
 
BROKER_HOST = os.environ.get("MQTT_HOST", "localhost")
BROKER_PORT = 1883
HEARTBEAT_EVERY_S = 2.0    # logger/<name>/heartbeat: proves the WRITER loop is alive (read by health_monitor.py)
FLUSH_INTERVAL_S = 1.0     # batch writes: one transaction per second, not one per row
 
# topic group -> subscription QoS.
# camera/# is QoS 1 so the camera's demand/window messages (published at QoS 1)
# are delivered with acknowledgement; effective QoS = min(publisher, subscriber).
TOPIC_GROUPS = {
    "arm":          ("arm/#", 0),
    "camera":       ("camera/#", 1),
    "analyzer":     ("analyzer/#", 0),
    "orchestrator": ("orchestrator/#", 0),
}
 
# Tables each topic group needs. 'events' (raw JSON) is created for every database.
ARM_SQL = """
    CREATE TABLE IF NOT EXISTS telemetry (
        id INTEGER PRIMARY KEY,
        ts_src REAL, ts_rx REAL,           -- source time and receive time
        joint INTEGER, pos INTEGER, speed INTEGER, cur INTEGER,  -- RAW values
        temp INTEGER                                             -- deg C, NULL if not sent
    );
    CREATE INDEX IF NOT EXISTS idx_tel_ts ON telemetry(ts_src);
    CREATE INDEX IF NOT EXISTS idx_tel_joint ON telemetry(joint, id);  -- 'latest row per joint'
    CREATE INDEX IF NOT EXISTS idx_tel_rx ON telemetry(ts_rx);         -- 'last N seconds'
"""
CAMERA_SQL = """
    CREATE TABLE IF NOT EXISTS camera_demand (   -- one row per 10 s sample
        id INTEGER PRIMARY KEY,
        ts_src REAL, ts_rx REAL,
        window_id INTEGER, elapsed_s REAL, interval_s INTEGER,
        count INTEGER,                           -- NEW white objects in this interval
        cumulative INTEGER                       -- running total in the 3-min window
    );
    CREATE TABLE IF NOT EXISTS camera_windows (  -- one row per completed 3-min window
        id INTEGER PRIMARY KEY,
        ts_src REAL, ts_rx REAL,
        window_id INTEGER, duration_s REAL,
        total INTEGER
    );
    CREATE INDEX IF NOT EXISTS idx_cam_demand_ts ON camera_demand(ts_src);
"""
EVENTS_SQL = """
    CREATE TABLE IF NOT EXISTS events (    -- everything that has no table of its own
        id INTEGER PRIMARY KEY,
        ts_src REAL, ts_rx REAL, topic TEXT, payload TEXT
    );
"""
SCHEMA_BY_GROUP = {"arm": ARM_SQL, "camera": CAMERA_SQL}
 
inbox = queue.Queue()      # network thread puts messages here, main thread drains it
 
 
# ---------------------------------------------------------------------------
# MQTT callbacks - run in paho's background network thread. Keep them FAST:
# no disk I/O here, just hand the message to the queue.
# ---------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[logger] connected to broker: {reason_code}")
    # Subscribe inside on_connect so subscriptions are restored after a reconnect.
    client.subscribe(userdata["subscriptions"])
    # Tell the health monitor we are up (retained). The Last Will below flips it to offline if we die.
    client.publish(userdata["status_topic"], json.dumps({"online": True, "ts": time.time()}), qos=1, retain=True)
 
def on_message(client, userdata, msg):
    # Safety net: keep only topics this instance was asked to record. A broker should
    # never send others, but retained messages from some brokers can slip through,
    # and a database must not collect another node's data by accident.
    if not msg.topic.startswith(userdata["prefixes"]):
        return
    # Record receive time now; the payload's own "ts" is the measurement time.
    inbox.put((time.time(), msg.topic, msg.payload))
 
 
# ---------------------------------------------------------------------------
# STORAGE
# ---------------------------------------------------------------------------
def migrate_old_camera_tables(db):
    """
    The camera used to count orange/green; it now counts white only.
    If a camera table with the OLD columns exists, rename it (data kept)
    so the new layout can be created cleanly.
    """
    for table in ("camera_demand", "camera_windows"):
        cols = [row[1] for row in db.execute(f"PRAGMA table_info({table})")]
        if cols and "green" in cols:
            db.execute(f"ALTER TABLE {table} RENAME TO {table}_orange_green")
            db.execute("DROP INDEX IF EXISTS idx_cam_demand_ts")
            print(f"[logger] old-format table {table} kept as {table}_orange_green")
    db.commit()
 
 
def migrate_arm_temp(db):
    """arm.db made before the dashboard has no temp column: add it (old rows get NULL)."""
    cols = [row[1] for row in db.execute("PRAGMA table_info(telemetry)")]
    if cols and "temp" not in cols:
        db.execute("ALTER TABLE telemetry ADD COLUMN temp INTEGER")
        db.commit()
        print("[logger] added 'temp' column to existing telemetry table")


def open_db(path, groups):
    """Open the database and create ONLY the tables the chosen topics need."""
    db = sqlite3.connect(path)
    db.execute("PRAGMA journal_mode=WAL")      # readers (analyzer) don't block the writer
    db.execute("PRAGMA synchronous=NORMAL")    # fewer fsyncs: faster, safe enough here
    migrate_old_camera_tables(db)              # no-op unless an old camera table exists
    db.executescript(EVENTS_SQL)
    for group in groups:
        db.executescript(SCHEMA_BY_GROUP.get(group, ""))
    if "arm" in groups:
        migrate_arm_temp(db)
    return db
 
 
# ---------------------------------------------------------------------------
# CAMERA TABLE DISPLAY - the same layout is used live and by --show
# ---------------------------------------------------------------------------
TABLE_FMT = "{:<15}{:<21}{}"
display_state = {"window": None}     # window currently being printed live
 
 
def fmt_clock(seconds):
    s = int(round(seconds or 0))
    return f"{s // 60:02d}:{s % 60:02d}"
 
def table_header(window_id):
    return "\n".join([
        f"White-object counting - window #{window_id}",
        TABLE_FMT.format("Elapsed time", "New white objects", "Cumulative count"),
        "-" * 52,
    ])
 
def table_row(elapsed_s, count, cumulative):
    return TABLE_FMT.format(fmt_clock(elapsed_s), str(count), str(cumulative))
 
def table_footer(window_id, duration_s, total):
    minutes = round((duration_s or 0) / 60.0, 1)
    return "\n".join([
        "-" * 52,
        f"After {minutes:g} minutes (window #{window_id}):  Total = {total}",
        "",
    ])
 
 
def print_live_camera_rows(demand_rows, window_rows):
    """Print new camera rows in the table layout as they arrive."""
    for (_ts, _rx, wid, elapsed, _interval, count, cumulative) in demand_rows:
        if wid != display_state["window"]:
            print("\n" + table_header(wid))
            display_state["window"] = wid
        print(table_row(elapsed, count, cumulative))
    for (_ts, _rx, wid, duration, total) in window_rows:
        print(table_footer(wid, duration, total))
        display_state["window"] = None
 
 
def show_window(db_path, window_id=None):
    """--show: print one stored window (default: the latest) and exit."""
    if not os.path.exists(db_path):
        print(f"{db_path} does not exist.")
        return
    db = sqlite3.connect(db_path)
    try:
        print_current_window(db, window_id, title="WINDOW")
    except sqlite3.OperationalError:
        print(f"{db_path} has no camera tables. Was it created with --topics camera?")
    finally:
        db.close()


# ---------------------------------------------------------------------------
# DASHBOARDS - read-only views of a stored database (plain text, no MQTT).
# The recording instances never import or call any of this.
# ---------------------------------------------------------------------------
ARM_TICKS_PER_REV = 4096       # STS3215 encoder: 4096 ticks = 360 deg
ARM_MA_PER_COUNT = 6.5         # ASSUMED current unit - verify against a bench supply
ARM_SPEED_TICKS_PER_S = True   # ASSUMED: raw speed is ticks/s (-> deg/s). Verify.
STALE_S = 5.0                  # no new row for this long = say so, loudly


def has_table(db, name):
    return db.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name=?", (name,)).fetchone() is not None


def age_text(seconds):
    return f"{seconds:.1f} s" if seconds < 60 else f"{seconds / 60:.1f} min"


CAMERA_WINDOW_S = 180          # must match DATA_COLLECTION_SECONDS in the camera node
RULE = "-" * 52


def clock(ts):
    """Epoch seconds -> HH:MM:SS in the Pi's local time."""
    return time.strftime("%H:%M:%S", time.localtime(ts))


def latest_run_rows(db, window_id):
    """Samples of the NEWEST run of this window_id, oldest first.

    The camera restarts its numbering at #1 every time it starts, so one window_id can
    belong to several runs (and mixing them gives a table that restarts halfway down).
    Walking back from the newest row while elapsed_s keeps DEcreasing isolates one run.
    """
    rows = db.execute(
        "SELECT id, ts_rx, elapsed_s, count, cumulative FROM camera_demand "
        "WHERE window_id=? ORDER BY id DESC LIMIT 60", (window_id,)).fetchall()
    run = []
    for r in rows:
        if run and not ((r[2] or 0) < (run[-1][2] or 0)):
            break                                   # elapsed went up: older run, stop
        run.append(r)
    return run[::-1]


def closing_row(db, window_id, run):
    """The camera_windows row that CLOSED this run, or None while it is still running.
    It arrives after the run's last sample, so ts_rx >= that sample; an older run's row doesn't."""
    if not run:
        return None
    return db.execute(
        "SELECT id, ts_src, ts_rx, duration_s, total FROM camera_windows "
        "WHERE window_id=? AND ts_rx >= ? ORDER BY id DESC LIMIT 1",
        (window_id, run[-1][1])).fetchone()


def print_current_window(db, window_id=None, title="CURRENT WINDOW", run=None, done=None):
    """Elapsed / New objects / Cumulative for one window, plus a status footer."""
    if window_id is None:
        r = db.execute("SELECT window_id FROM camera_demand ORDER BY id DESC LIMIT 1").fetchone()
        window_id = r[0] if r else None
    if window_id is None:
        print("No camera samples stored yet.")
        return
    if run is None:
        run = latest_run_rows(db, window_id)
        done = closing_row(db, window_id, run)
    print(f"{title} #{window_id}")
    print(RULE)
    if not run:
        print(f"No samples stored for window #{window_id}.")
        return
    fmt = "{:<14}{:<18}{}"
    print(fmt.format("Elapsed", "New objects", "Cumulative"))
    for _id, _rx, elapsed, count, cumulative in run:
        print(fmt.format(fmt_clock(elapsed), str(count), str(cumulative)))
    if done:
        total, last_cum = done[4], run[-1][4]
        if total is not None and last_cum is not None and total != last_cum:
            # Objects counted after the last 10 s sample are in the total but in no sample.
            print(fmt.format(fmt_clock(done[3]) + "*", str(total - last_cum), str(total)))
            print("* counted after the last 10 s sample")
        print(f"\n✓ Window #{window_id} COMPLETE — {total} white objects")
    else:
        print(f"\nWindow #{window_id} in progress — {fmt_clock(run[-1][2])} of {fmt_clock(CAMERA_WINDOW_S)}")


def camera_dashboard(db, db_path, n_windows):
    """Completed windows (Start / End / Total) + the latest window's 10 s table."""
    print("CAMERA — WHITE OBJECT DETECTION")
    print("=" * 52)

    # Work out the latest window first, so a window that is shown below as
    # "CURRENT ... COMPLETE" is not also listed in the history above it.
    last = db.execute("SELECT window_id FROM camera_demand ORDER BY id DESC LIMIT 1").fetchone()
    cur_id = last[0] if last else None
    run = latest_run_rows(db, cur_id) if cur_id is not None else []
    done = closing_row(db, cur_id, run) if run else None

    print("\n3-MINUTE PRODUCTION")
    print(RULE)
    rows = db.execute(
        "SELECT id, window_id, ts_src, ts_rx, duration_s, total FROM camera_windows "
        "ORDER BY id DESC LIMIT ?", (n_windows + 1,)).fetchall()
    if done:
        rows = [r for r in rows if r[0] != done[0]]
    rows = rows[:n_windows][::-1]
    if rows:
        print(f"{'Window':<13}{'Start':<12}{'End':<12}{'Total':>5}")
        prev = None
        for _id, wid, ts_src, ts_rx, dur, total in rows:
            if prev is not None and wid <= prev:
                print("   -- camera restarted (window numbers start again at #1) --")
            prev = wid
            end = ts_src or ts_rx
            print(f"{'#' + str(wid):<13}{clock(end - (dur or 0)):<12}{clock(end):<12}{total:>5}")
        avg = sum((r[5] or 0) for r in rows) / len(rows)
        print(f"\nAverage: {avg:.1f} white objects / 3 minutes  (last {len(rows)} window(s))")
    else:
        print("No completed 3-minute window yet.")

    print()
    print_current_window(db, cur_id, run=run, done=done)

    newest = db.execute("SELECT MAX(ts_rx) FROM camera_demand").fetchone()[0]
    if newest is not None and time.time() - newest > STALE_S * 3:
        print(f"\n!! NO CAMERA DATA for {age_text(time.time() - newest)} - camera node stopped?")


def arm_dashboard(db):
    """Latest value per joint, plus data age, peak current, and message rate."""
    print("ROBOT ARM MONITOR")
    print("=" * 52)
    latest = db.execute(
        "SELECT joint,pos,speed,cur,temp,ts_rx FROM telemetry "
        "WHERE id IN (SELECT MAX(id) FROM telemetry GROUP BY joint) ORDER BY joint").fetchall()
    if not latest:
        print("No arm telemetry stored yet.")
        return
    now = time.time()
    peaks = dict(db.execute(
        "SELECT joint, MAX(cur) FROM telemetry WHERE ts_rx > ? GROUP BY joint", (now - 60,)).fetchall())
    rate = db.execute("SELECT COUNT(*) FROM telemetry WHERE ts_rx > ?", (now - 10,)).fetchone()[0] / 10.0

    cols = []
    for joint, pos, speed, cur, temp, ts_rx in latest:
        sp = (speed or 0) * 360.0 / ARM_TICKS_PER_REV
        peak = peaks.get(joint)
        cols.append({
            "head": f"J{joint}",
            "Position":    f"{(pos or 0) * 360.0 / ARM_TICKS_PER_REV:.1f}\u00b0",
            "Velocity":    f"{sp:.0f}\u00b0/s",
            "Temperature": "--" if temp is None else f"{temp}\u00b0C",
            "Current":     "--" if cur is None else f"{cur * ARM_MA_PER_COUNT / 1000:.2f} A",
            "Peak 60 s":   "--" if peak is None else f"{peak * ARM_MA_PER_COUNT / 1000:.2f} A",
            "Data age":    f"{now - ts_rx:.1f} s",
        })
    print("".ljust(13) + "".join(c["head"].ljust(10) for c in cols))
    for label in ("Position", "Velocity", "Temperature", "Current", "Peak 60 s", "Data age"):
        print(label.ljust(13) + "".join(c[label].ljust(10) for c in cols))
    print(f"\nRate: {rate:.0f} rows/s  ({len(cols)} joint(s) seen)")
    newest = max(r[5] for r in latest)
    if now - newest > STALE_S:
        print(f"!! NO ARM DATA for {age_text(now - newest)} - arm node or Mega stopped?")
    print("Units assumed: 6.5 mA/count, speed in ticks/s - not verified on the real arm.")


def dashboard(db_path, n_windows):
    """Pick the dashboard from the tables inside the database and print it once."""
    if not os.path.exists(db_path):           # sqlite3.connect would silently CREATE an empty file
        print(f"{db_path} does not exist. Start a logger instance first.")
        return
    db = sqlite3.connect(db_path)
    try:
        print(time.strftime("%Y-%m-%d %H:%M:%S") + f"   [{db_path}]\n")
        shown = False
        if has_table(db, "camera_windows") or has_table(db, "camera_demand"):
            camera_dashboard(db, db_path, n_windows)
            shown = True
        if has_table(db, "telemetry"):
            if shown:
                print()
            arm_dashboard(db)
            shown = True
        if not shown:
            print("This database has no camera or telemetry tables, so no dashboard to draw.")
    except sqlite3.OperationalError as e:
        print(f"database busy/unreadable: {e}")
    finally:
        db.close()


def run_dashboard(db_path, watch_s, n_windows):
    """One snapshot, or redraw every watch_s seconds until Ctrl+C."""
    try:
        sys.stdout.reconfigure(errors="replace")   # old Windows consoles: never crash on a symbol
    except Exception:
        pass
    while True:
        if watch_s:
            os.system("cls" if os.name == "nt" else "clear")
        dashboard(db_path, n_windows)
        if not watch_s:
            return
        try:
            time.sleep(watch_s)
        except KeyboardInterrupt:
            return


def flush(db):
    """Drain the queue and write everything in ONE transaction."""
    tel_rows, ev_rows, demand_rows, window_rows = [], [], [], []
    while True:
        try:
            ts_rx, topic, raw = inbox.get_nowait()
        except queue.Empty:
            break
        try:
            data = json.loads(raw)
        except ValueError:
            continue                            # not JSON: drop it, don't crash
        ts_src = data.get("ts")
        if topic == "arm/telemetry":
            # One MQTT message carries all six joints -> one DB row per joint.
            for joint, v in data.get("joints", {}).items():
                tel_rows.append((ts_src, ts_rx, int(joint), v.get("pos"), v.get("speed"), v.get("cur"), v.get("temp")))
        elif topic == "camera/demand":
            # Structured table: this is the data the demand forecast trains on.
            demand_rows.append((ts_src, ts_rx, data.get("window_id"), data.get("elapsed_s"),
                                data.get("interval_s"), data.get("count"), data.get("cumulative")))
        elif topic == "camera/window":
            window_rows.append((ts_src, ts_rx, data.get("window_id"), data.get("duration_s"),
                                data.get("total")))
        else:
            # Everything else (arm/state, camera/perception, status, ...) is kept
            # raw as JSON text. Nothing is lost; we can add tables later.
            ev_rows.append((ts_src, ts_rx, topic, raw.decode("utf-8", "ignore")))
    if tel_rows or ev_rows or demand_rows or window_rows:
        with db:                                # 'with db' = a single commit
            # Each INSERT only runs if it has rows, so a database that has no
            # telemetry table (camera.db) never tries to write to one.
            if tel_rows:
                db.executemany("INSERT INTO telemetry(ts_src,ts_rx,joint,pos,speed,cur,temp) VALUES (?,?,?,?,?,?,?)", tel_rows)
            if ev_rows:
                db.executemany("INSERT INTO events(ts_src,ts_rx,topic,payload) VALUES (?,?,?,?)", ev_rows)
            if demand_rows:
                db.executemany("INSERT INTO camera_demand(ts_src,ts_rx,window_id,elapsed_s,interval_s,count,cumulative) VALUES (?,?,?,?,?,?,?)", demand_rows)
            if window_rows:
                db.executemany("INSERT INTO camera_windows(ts_src,ts_rx,window_id,duration_s,total) VALUES (?,?,?,?,?)", window_rows)
    print_live_camera_rows(demand_rows, window_rows)
    return len(tel_rows), len(ev_rows), len(demand_rows), len(window_rows)
 
 
def parse_args(argv=None):
    p = argparse.ArgumentParser(description="MQTT logger: chosen topics -> chosen SQLite file")
    p.add_argument("--topics", nargs="+", choices=sorted(TOPIC_GROUPS),
                   default=sorted(TOPIC_GROUPS),
                   help="topic groups to record (default: all)")
    p.add_argument("--db", default="logger.db", help="SQLite file to write (default: logger.db)")
    p.add_argument("--show", action="store_true",
                   help="print a stored camera window as a table and exit (no MQTT)")
    p.add_argument("--window", type=int, default=None,
                   help="with --show: window number (default: latest)")
    p.add_argument("--dashboard", action="store_true",
                   help="draw the dashboard for this database and exit (no MQTT)")
    p.add_argument("--watch", type=float, default=None, metavar="SEC",
                   help="with --dashboard (implied): redraw every SEC seconds")
    p.add_argument("--windows", type=int, default=8,
                   help="camera dashboard: how many 3-minute windows in the bar chart")
    return p.parse_args(argv)
 
 
def main():
    args = parse_args()
    if args.show:
        show_window(args.db, args.window)
        return
    if args.dashboard or args.watch:
        run_dashboard(args.db, args.watch, args.windows)
        return
    groups = list(args.topics)
    subscriptions = [TOPIC_GROUPS[g] for g in groups]
    db = open_db(args.db, groups)
 
    # client_id MUST be unique per running instance: the broker allows one connection
    # per client_id, so two loggers sharing an id would kick each other off forever.
    client_id = "logger_" + "_".join(groups)
    name = "_".join(groups)                      # "arm" or "camera" -> logger/arm/..., logger/camera/...
    status_topic = f"logger/{name}/status"
    heartbeat_topic = f"logger/{name}/heartbeat"
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                         userdata={"subscriptions": subscriptions,
                                   "prefixes": tuple(f"{g}/" for g in groups),
                                   "status_topic": status_topic})
    client.on_connect = on_connect
    client.on_message = on_message
    client.will_set(status_topic, json.dumps({"online": False}), qos=1, retain=True)
    # Treat SIGTERM (systemd stop, `kill`) like Ctrl+C so the last batch is still flushed.
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=10)   # non-blocking, auto-reconnects
    client.loop_start()                                            # network thread
    print(f"[logger] {client_id}: topics {', '.join(groups)} -> {args.db}; "
          f"broker {BROKER_HOST}:{BROKER_PORT}. Ctrl+C to stop.")
    written, last_hb = 0, 0.0
    try:
        while True:
            time.sleep(FLUSH_INTERVAL_S)
            t, e, d, w = flush(db)
            written += t + e + d + w
            if time.monotonic() - last_hb >= HEARTBEAT_EVERY_S:
                last_hb = time.monotonic()
                # Sent AFTER flush(): if the database write hangs, this stops and health goes STALE.
                client.publish(heartbeat_topic, json.dumps(
                    {"ts": time.time(), "written": written, "queued": inbox.qsize()}), qos=0)
            if t or e or d or w:
                print(f"[logger] wrote {t} telemetry, {e} events, {d} camera demand, {w} camera windows")
    except KeyboardInterrupt:
        pass
    finally:
        flush(db)                               # don't lose the last batch
        try:   # tell the health monitor this was a clean stop (fails harmlessly with no broker)
            client.publish(status_topic, json.dumps({"online": False, "ts": time.time()}),
                           qos=1, retain=True).wait_for_publish(1.0)
        except Exception:
            pass
        client.loop_stop()
        client.disconnect()
        db.close()
 
 
if __name__ == "__main__":
    main()