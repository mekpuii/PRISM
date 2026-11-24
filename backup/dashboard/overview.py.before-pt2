"""
OVERVIEW  -  the data behind the Overview page of dashboard_node.py (PRISM, Raspberry Pi 4).

  PiStat      the Pi itself: CPU, temperature, memory, disk, uptime, power (vcgencmd get_throttled), and which PRISM
              programs are running (found in /proc, with their CPU and memory). Sampled every PI_EVERY_S by a thread.
  hardware()  is each piece of hardware connected and answering? Judged from live MQTT data and /dev.
  readiness() READY = every REQUIRED item is OK. The page and start_cell.sh show the same list.
  Trace       the optimizer's decisions as sentences ("I changed the belt from 20 to 24 rpm because ..."), from its
              orchestrator/plan messages, plus the history in orchestrator/logs/orch_*.jsonl.
  cases_tail()  the last rows of the camera's case array (orchestrator/cases/cases_<day>.csv).

Everything here only READS. The sentences come from the optimizer's own reason text: nothing is invented, and the
raw reason is kept next to every sentence ("why").
"""

import csv
import glob
import json
import os
import re
import shutil
import subprocess
import threading
import time
from collections import deque

PRISM = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORCH_DIR = os.path.join(PRISM, "orchestrator")
ORCH_LOGS = os.path.join(ORCH_DIR, "logs")
CASES_DIR = os.path.join(ORCH_DIR, "cases")

PI_EVERY_S = 2.0              # Pi + program sample period
THROTTLE_EVERY_S = 10.0       # vcgencmd is a subprocess: not every sample
TRACE_KEEP = 400              # sentences kept in memory
HISTORY_LINES = 4000          # last lines of the newest optimizer log read at start
FRESH_S = 3.0                 # telemetry younger than this = the device answers
RAIL_V = (11.0, 13.2)         # 12 V rail: outside this = warn
FEED_DEAD_MV = 15             # = optimize.FEED_DEAD_MV: a push with a smaller 12 V dip did not move the servo
FEED_IR_PROVEN = 3            # = optimize.FEED_IR_PROVEN

# name, cmdline regex, node names in dashboard_node.NODES (MQTT side), needs cwd == orchestrator, required for READY
PROGRAMS = [
    ("MQTT broker",   r"(^|/)mosquitto( |$)",              [],                                         False, True),
    ("vision",        r"vision_live_tracks\.py|vision_usb\.py",            ["camera"],                                 False, True),
    ("autoen",        r"auto_en_new\.py",                   ["autoen (arm + feeder + belt driver)"],    False, True),
    ("optimizer",     r"(^|/| )main\.py( |$)",              ["orchestrator"],                           True,  True),
    ("health",        r"health\.py",                        ["health monitor"],                         False, False),
    ("logger arm",    r"logger\.py.*--topics arm( |$)",     ["logger (arm)"],                           False, False),
    ("logger camera", r"logger\.py.*--topics camera( |$)",  ["logger (camera)"],                        False, False),
    ("arm analyzer",  r"arms_ana\.py",                      ["arm analyzer"],                           False, False),
    ("belt analyzer", r"conv_ana\.py",                      ["belt analyzer"],                          False, False),
    ("dashboard",     r"dashboard_node\.py",                [],                                         False, False),
]
_PROG_RE = [(n, re.compile(rx), nodes, cwd, req) for n, rx, nodes, cwd, req in PROGRAMS]

THROTTLE_BITS = {0: "under-voltage now", 1: "CPU speed capped now", 2: "throttled now", 3: "temperature limit now",
                 16: "under-voltage since boot", 17: "speed capped since boot", 18: "throttled since boot",
                 19: "temperature limit since boot"}


def _read(path, default=None):
    try:
        with open(path, "rb") as f:
            return f.read().decode("utf-8", "replace")
    except OSError:
        return default


# ============================================================================
# 1. THE PI + ITS PROGRAMS
# ============================================================================
class PiStat:
    def __init__(self):
        self.lock = threading.Lock()
        self.snap = {}
        self._cpu_prev = None
        self._proc_prev = {}                       # pid -> (ticks, monotonic)
        self._thr_t, self._thr = 0.0, None
        self.model = (_read("/proc/device-tree/model", "") or "").strip("\x00 \n") or None
        try:
            self.tck = os.sysconf("SC_CLK_TCK")
            self.page = os.sysconf("SC_PAGE_SIZE")
        except (AttributeError, ValueError, OSError):  # not Linux: the Pi part stays empty
            self.tck = self.page = None
        self.ncpu = os.cpu_count() or 1
        threading.Thread(target=self._run, daemon=True).start()

    def get(self):
        with self.lock:
            return self.snap

    def _run(self):
        while True:
            try:
                s = self._sample()
                with self.lock:
                    self.snap = s
            except Exception as e:                  # noqa: BLE001 - the sampler must never die
                print(f"[overview] Pi sample failed: {e!r}", flush=True)
            time.sleep(PI_EVERY_S)

    def _sample(self):
        out = {"model": self.model, "t": time.time()}
        st = _read("/proc/stat")
        if st:
            v = [int(x) for x in st.split("\n", 1)[0].split()[1:]]
            idle, total = v[3] + (v[4] if len(v) > 4 else 0), sum(v[:8])
            if self._cpu_prev and total > self._cpu_prev[1]:
                out["cpu_pct"] = round(100.0 * (1 - (idle - self._cpu_prev[0]) / (total - self._cpu_prev[1])), 1)
            self._cpu_prev = (idle, total)
        t = _read("/sys/class/thermal/thermal_zone0/temp")
        if t:
            out["temp_c"] = round(int(t) / 1000.0, 1)
        fr = _read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        if fr:
            out["mhz"] = int(fr) // 1000
        mi = _read("/proc/meminfo")
        if mi:
            kv = {ln.split(":")[0]: int(ln.split()[1]) for ln in mi.splitlines() if ln.split()[1:2]}
            if "MemTotal" in kv and "MemAvailable" in kv:
                out["mem_total_mb"] = kv["MemTotal"] // 1024
                out["mem_used_mb"] = (kv["MemTotal"] - kv["MemAvailable"]) // 1024
        try:
            du = shutil.disk_usage(PRISM)
            out["disk_free_gb"] = round(du.free / 1e9, 1)
            out["disk_used_pct"] = round(100.0 * du.used / du.total, 1)
        except OSError:
            pass
        up = _read("/proc/uptime")
        if up:
            out["uptime_s"] = float(up.split()[0])
        try:
            out["load"] = [round(x, 2) for x in os.getloadavg()]
        except (AttributeError, OSError):
            pass
        now = time.monotonic()
        if now - self._thr_t >= THROTTLE_EVERY_S:
            self._thr_t = now
            try:
                r = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2)
                m = re.search(r"0x([0-9a-fA-F]+)", r.stdout)
                self._thr = int(m.group(1), 16) if m else None
            except (OSError, subprocess.SubprocessError):
                self._thr = None
        if self._thr is not None:
            out["throttled"] = {"raw": hex(self._thr), "flags": [txt for b, txt in THROTTLE_BITS.items() if self._thr >> b & 1]}
        out["programs"] = self._programs(now)
        return out

    def _programs(self, now):
        found = {n: [] for n, *_ in PROGRAMS}
        seen = {}
        for pd in glob.glob("/proc/[0-9]*"):
            pid = int(pd[6:])
            raw = _read(pd + "/cmdline")
            if not raw:
                continue
            argv = [a for a in raw.split("\x00") if a]
            cmd = " ".join(argv)
            for name, rx, _nodes, need_cwd, _req in _PROG_RE:
                if not rx.search(cmd):
                    continue
                if name != "MQTT broker" and not os.path.basename(argv[0]).startswith("python"):
                    continue                        # an editor or a grep that mentions the file name is not the program
                if need_cwd:
                    try:
                        if os.readlink(pd + "/cwd") != ORCH_DIR:
                            continue
                    except OSError:
                        continue
                info = {"pid": pid}
                stat = _read(pd + "/stat")
                if stat and self.tck:
                    f = stat.rsplit(")", 1)[1].split()
                    ticks = int(f[11]) + int(f[12])
                    p = self._proc_prev.get(pid)
                    if p and now > p[1]:
                        info["cpu_pct"] = round(100.0 * (ticks - p[0]) / self.tck / (now - p[1]) / self.ncpu, 1)
                    seen[pid] = (ticks, now)
                    info["mem_mb"] = round(int(f[21]) * self.page / 1e6, 1)
                found[name].append(info)
                break
        self._proc_prev = seen
        return found


# ============================================================================
# 2. HARDWARE + READY
# ============================================================================
def _item(name, state, text, required=True, hint=""):
    """state: ok | warn | bad | off (off = not judged: nothing to look at yet)."""
    return {"name": name, "state": state, "text": text, "required": required, "hint": hint}


def hardware(store, pi):
    g = store.get
    belt, belt_age = g("belt/telemetry"), store.age("belt/telemetry")
    belt_ok = belt is not None and belt_age is not None and belt_age < FRESH_S
    arm_age = store.age("arm/telemetry")
    orch = g("orchestrator/status") or {}
    orch_age = store.age("orchestrator/status")
    score = (orch.get("score") or {}) if orch_age is not None and orch_age < 10 else {}
    progs = (pi or {}).get("programs") or {}
    autoen_up = bool(progs.get("autoen")) or (store.age("autoen/state") or 99) < 5
    out = []

    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if belt_ok:
        out.append(_item("Mega (USB)", "ok", f"talking on {', '.join(ports) or 'USB'}"))
    elif ports:
        out.append(_item("Mega (USB)", "warn" if autoen_up else "off",
                         f"plugged in ({', '.join(ports)}) but no data" + ("" if autoen_up else ": autoen is not running"),
                         hint="autoen owns the Mega: start the cell"))
    else:
        out.append(_item("Mega (USB)", "bad", "not found (no /dev/ttyUSB* or /dev/ttyACM*)",
                         hint="check the USB cable and the Mega's power"))

    if arm_age is not None and arm_age < FRESH_S:
        arm = store.arm or {}
        temps = [j.get("temp") for j in arm.get("joints", []) if j.get("temp") is not None]
        cur = sum(j.get("cur_mA") or 0 for j in arm.get("joints", []))
        out.append(_item("Arm servos (6)", "ok", f"all 6 answer · {cur:.0f} mA" + (f" · max {max(temps)} °C" if temps else "")))
    elif belt_ok:
        out.append(_item("Arm servos (6)", "bad", "not all 6 servos answer",
                         hint="check the servo bus cable and the servo power"))
    else:
        out.append(_item("Arm servos (6)", "off", "no data yet (needs the Mega)"))

    if belt_ok:
        flt = belt.get("fault")
        out.append(_item("Belt motor + encoder", "bad" if flt else "ok",
                         f"FAULT {flt}" if flt else
                         f"{belt.get('velocity', 0):.1f} rpm · {belt.get('current', 0):.0f} mA"
                         + (f" · {belt['temp']:.0f} °C" if belt.get("temp") is not None else "")))
        v = belt.get("voltage")
        if v is not None:
            okv = RAIL_V[0] <= v <= RAIL_V[1]
            out.append(_item("12 V rail", "ok" if okv else "warn", f"{v:.2f} V" + (
                f" · arm supply {belt['arm_voltage']:.2f} V" if belt.get("arm_voltage") else ""),
                hint="" if okv else "check the 12 V supply"))
    else:
        out.append(_item("Belt motor + encoder", "off", "no data yet (needs the Mega)"))
        out.append(_item("12 V rail", "off", "no data yet (needs the Mega)"))

    fd = score.get("feeder") or {}
    dip = fd.get("last_dip_mV")
    stop = score.get("feed_stop") or {}
    if stop.get("kind") == "dead":
        out.append(_item("Feeder servo", "bad", f"ALARM: does not move (12 V dip {stop.get('dip_mV')} mV) · the optimizer "
                                                f"retries by itself" + (f" in {stop['next_retry_s']} s" if stop.get("next_retry_s") is not None else ""),
                         required=False, hint="check the D12 plug and the servo's 5 V supply"))
    elif stop.get("kind") == "empty":
        out.append(_item("Feeder servo", "warn", "feeder empty: load cubes, the optimizer starts again by itself", required=False))
    elif not score:
        out.append(_item("Feeder servo", "off", "judged by the optimizer: not running", required=False))
    elif dip is None:
        out.append(_item("Feeder servo", "off", "no push seen yet", required=False))
    elif dip < FEED_DEAD_MV and not fd.get("armed"):
        out.append(_item("Feeder servo", "off", f"12 V dip {dip} mV: the dip check is not proven in this run "
                                                f"({fd.get('clear_this_run', 0)} clear pushes), the cubes decide", required=False))
    elif dip < FEED_DEAD_MV:
        out.append(_item("Feeder servo", "bad", f"last push did not move (12 V dip {dip} mV)", required=False,
                         hint="check the D12 plug and the servo's 5 V supply"))
    else:
        out.append(_item("Feeder servo", "ok", f"last push moved (dip {dip} mV) · {fd.get('pushes', 0)} pushes, "
                                               f"{fd.get('dead', 0)} dead", required=False))

    if belt_ok and belt.get("feed_ir") is not None:
        fir = belt.get("feed_ir")
        right = score.get("feeder_ir_right")
        proven = right is not None and right >= FEED_IR_PROVEN
        out.append(_item("Feeder IR", "ok", ("sees cubes" if fir else "empty") +
                         ("" if right is None else f" · {'trusted' if proven else 'not trusted yet'} ({right}/{FEED_IR_PROVEN} right)"),
                         required=False))
    else:
        out.append(_item("Feeder IR", "off", "no data yet", required=False))

    ir = g("arm/ir")
    out.append(_item("Pick IR", "ok" if belt_ok else "off",
                     ("block at the pick point" if ir and ir.get("detected") else "clear") if belt_ok else "no data yet",
                     required=False))

    gr = g("arm/gripper")
    if gr is None:
        out.append(_item("Gripper", "off", "no grip seen yet", required=False))
    else:
        st = gr.get("state")
        out.append(_item("Gripper", "bad" if st == "ERROR" else "ok",
                         f"last: {st}" + (f" ({gr.get('reason')})" if gr.get("reason") else ""), required=False,
                         hint="check the gripper servo cable" if st == "ERROR" else ""))

    cam, cam_ret, cam_age = store.get_live("camera/perception")
    if cam is not None and not cam_ret and cam_age is not None and cam_age < FRESH_S:
        out.append(_item("Camera", "ok", f"{cam.get('fps', 0)} fps · {cam.get('active', 0)} tracks"))
    else:
        out.append(_item("Camera", "bad" if progs.get("vision") else "off",
                         "vision runs but sends no frames" if progs.get("vision") else "vision is not running",
                         hint="check the camera's USB cable" if progs.get("vision") else ""))

    thr = (pi or {}).get("throttled")
    if thr is not None:
        now_bad = [f for f in thr["flags"] if f.endswith("now")]
        past = [f for f in thr["flags"] if f.endswith("boot")]
        out.append(_item("Pi power", "bad" if now_bad else "warn" if past else "ok",
                         ", ".join(now_bad or past) or "good (no under-voltage)", required=False,
                         hint="use the official 5.1 V / 3 A supply: under-voltage also breaks USB" if now_bad or past else ""))
    return out


def programs(store, pi, node_status):
    progs = (pi or {}).get("programs") or {}
    out = []
    for name, _rx, nodes, _cwd, req in PROGRAMS:
        procs = progs.get(name) or []
        ns = [dict(node_status[n], node=n) for n in nodes if n in node_status]
        data = "OK" if not ns else (
            "OFFLINE" if any(x["status"] == "OFFLINE" for x in ns) else
            "STALE" if any(x["status"] in ("STALE", "NO DATA") for x in ns) else
            "NOT AVAILABLE" if all(x["status"] == "NOT AVAILABLE" for x in ns) else "OK")
        if name == "MQTT broker":
            data = "OK" if store.mqtt_connected else "OFFLINE"
        if not procs and not (name == "MQTT broker" and store.mqtt_connected):
            state = "bad" if req else "off"
            text = "not running"
        elif name == "MQTT broker" and data != "OK":
            state, text = "bad", "running, but the dashboard cannot connect to it"
        elif data == "OK":
            state, text = "ok", "running"
        else:
            state = "warn"
            text = "running, but " + "; ".join(f"{x['node']}: {x['reason'] or x['status']}" for x in ns if x["status"] != "OK")
        if name == "optimizer" and procs:
            st = store.get("orchestrator/status") or {}
            if st.get("shadow"):
                text += " · SHADOW (only watches)"
        cpu = sum(p.get("cpu_pct") or 0 for p in procs)
        mem = sum(p.get("mem_mb") or 0 for p in procs)
        out.append({"name": name, "state": state, "text": text, "required": req, "pids": [p["pid"] for p in procs],
                    "cpu_pct": round(cpu, 1) if procs else None, "mem_mb": round(mem, 1) if procs else None,
                    "twice": len(procs) > 1})
    return out


def readiness(hw, progs):
    """READY = every required program runs and every required piece of hardware answers. Optional ones only warn."""
    missing = [f"{x['name']}: {x['text']}" for x in progs + hw if x["required"] and x["state"] != "ok"]
    notes = [f"{x['name']}: {x['text']}" for x in progs + hw if not x["required"] and x["state"] in ("bad", "warn")]
    notes += [f"{x['name']}: running twice" for x in progs if x.get("twice")]
    return {"ready": not missing, "missing": missing, "notes": notes}


# ============================================================================
# 3. THE OPTIMIZER'S VOICE
# ============================================================================
MODE_SAY = {
    "RUN":     ("I'm working.", "info"),
    "IDLE":    ("I'm waiting: there is nothing to do right now.", "info"),
    "ALERT":   ("I need your help.", "alarm"),
    "RECOVER": ("Something went wrong. I'm trying to fix it myself.", "warn"),
    "MANUAL":  ("Someone else is controlling autoen, so I keep my hands off.", "warn"),
    "OFF":     ("autoen is not running a job, so I'm only watching.", "info"),
    "PARK":    ("I'm parking the arm on its cradle.", "info"),
    "COOL":    ("The motors are too hot. I'm pausing at home to let them cool down.", "warn"),
}
ACTION_SAY = {
    "pause": "I paused autoen.", "resume": "I pressed resume.", "refeed": "I'm feeding again.",
    "abort": "I stopped the belt.", "idle": "I parked the arm at home and braked the belt to save energy.",
    "park": "I put the arm on its cradle.", "wake": "I'm waking the arm up.",
    "stop_feed": "I paused, stopped the belt and parked the arm at home.",
    "alarm": "I paused, parked the arm and shook it as an alarm.",
}
WHY_SAY = [            # (words in the optimizer's reason, what they mean) - first match wins
    ("frontier", "I'm testing a faster setting I haven't proven yet. At the first problem I go straight back."),
    ("exploring", "I don't have enough measurements of this one yet, so I'm trying it for a few cycles to learn."),
    ("warm", "The motors are warm, so I'm being gentler."),
    ("incident response", "After a problem I use the slowest safe setting for a while."),
    ("success floor", "The old setting started failing too often, so I moved to a safer one."),
    ("rush", "Cubes are piling up on the belt, so I'm using the fastest safe setting."),
    ("within 5 %", "This is the fastest setting that is still safe and close to the lowest energy."),
]
_SETTINGS_RE = re.compile(r"settings [\d.]+/[\d.]+ -> [\d.]+/[\d.]+")
_TRIAL_END = [         # how optimize._trial_cycle words the end of a frontier trial -> (sentence, level)
    (re.compile(r"frontier ([\d.]+/[\d.]+) proven \(([^)]*)\)"),
     lambda m: (f"My test of {m[1]} worked ({m[2]}). I may use it from now on.", "good")),
    (re.compile(r"frontier ([\d.]+/[\d.]+) failed \(([^)]*)\): back to ([\d.]+/[\d.]+)"),
     lambda m: (f"My test of {m[1]} failed ({m[2]}). I went back to {m[3]} and won't try it again for a while.", "warn")),
    (re.compile(r"frontier ([\d.]+/[\d.]+) stopped: warm"),
     lambda m: (f"I stopped testing {m[1]}: the motors got warm.", "warn")),
]


def _num(x):
    return None if x is None else round(float(x), 2)


class Trace:
    def __init__(self, on_entry=None):
        self.lock = threading.Lock()
        self.items = deque(maxlen=TRACE_KEEP)
        self._id = 0
        self._last = None                       # {"key", "seq", "mode", "rpm", "arm", "carry"}
        self._cycles = []                       # OK / not OK of the current block of 10 cycles
        self.on_entry = on_entry                # e.g. Store.add_event: the sentences also go to the event log
        self._story = None                      # the last story told (its id + time)

    def since(self, i, limit=200):
        with self.lock:
            return [e for e in self.items if e["id"] > i][-limit:]

    def last_id(self):
        with self.lock:
            return self._id

    def _add(self, t, say, level, kind, why="", setting=None, history=False):
        with self.lock:
            self._id += 1
            e = {"id": self._id, "t": t, "say": say, "level": level, "kind": kind, "why": why, "setting": setting,
                 "history": history}
            self.items.append(e)
        if self.on_entry and not history:
            self.on_entry("optimizer", {"alarm": "alarm", "warn": "warn"}.get(level, "info"), say, t=t)

    # ---- inputs ----
    def started(self, t, shadow, history=False):
        self._last = None
        self._add(t, "I've started." + (" Shadow mode: I only think out loud and send nothing to autoen." if shadow else
                                         " I watch the camera and the robot and choose the settings."),
                  "info", "start", history=history)

    def story(self, st, t, history=False):
        """The optimizer's own explanation of a stop: what it tried, what it saw, so what, and what next."""
        key = (st.get("id"), st.get("t"))
        if key == self._story:
            return
        self._story = key
        tried = "; ".join(st.get("tried") or [])
        solved = str(st.get("problem", "")).startswith("solved")
        say = (f"{str(st.get('problem', '')).capitalize()}. " + (f"What I tried: {tried}. " if tried else "")
               + f"What I saw: {st.get('saw', '')}. So: {st.get('so', '')}. Next: {st.get('next', '')}.")
        self._add(t, say, "good" if solved else "warn", "story", history=history)

    def plan(self, d, t, retained=False, history=False):
        st = (d.get("score") or {}).get("story")
        if st:
            self.story(st, t, history=history)
        mode, reason, action = d.get("mode"), d.get("reason") or "", d.get("action")
        rpm, arm, carry = _num(d.get("rpm")), _num(d.get("arm_speed")), _num(d.get("carry_speed"))
        key = (mode, rpm, arm, carry, action, reason)
        seq = d.get("seq")
        prev = self._last
        if prev is not None:
            if key == prev["key"] or (action is None and key[:4] == prev["key"][:4] and reason == prev["key"][5]):
                prev["key"], prev["seq"] = key, seq        # same decision, or only the one-shot action was cleared
                return
            if retained and seq is not None and prev["seq"] is not None and seq <= prev["seq"]:
                return                          # the broker's stored copy of a plan we already told
        self._last = {"key": key, "seq": seq, "mode": mode, "rpm": rpm, "arm": arm, "carry": carry}
        setting = {"rpm": rpm, "arm": arm, "carry": carry, "mode": mode}
        parts, level, kind = [], "info", "note"
        if prev is None or mode != prev["mode"]:
            s, level = MODE_SAY.get(mode, (f"I'm in {mode} mode.", "info"))
            if mode == "RUN" and prev is not None and prev["mode"] in ("IDLE", "ALERT", "MANUAL", "RECOVER", "COOL"):
                s = "I'm back to work."
            parts.append(s)
            kind = "mode"
        if prev is not None and (rpm, arm, carry) != (prev["rpm"], prev["arm"], prev["carry"]):
            ch = []
            if rpm != prev["rpm"]:
                ch.append(f"the belt from {prev['rpm']:g} to {rpm:g} rpm")
            if arm != prev["arm"]:
                ch.append(f"the arm from {prev['arm']:g} to {arm:g}")
            if carry != prev["carry"]:
                ch.append(f"the carry speed from {prev['carry']:g} to {carry:g}")
            parts.append("I changed " + " and ".join(ch) + ".")
            low = reason.lower()
            why = next((w for k, w in WHY_SAY if k in low), None)
            if why:
                parts.append(why)
            kind = "change"
            if level == "info":
                level = "good"
        if action:
            parts.append(ACTION_SAY.get(action, f"I sent '{action}' to autoen."))
            if level in ("info", "good") and action in ("refeed", "abort", "pause"):
                level = "warn"
            kind = kind if kind != "note" else "action"
        reason_shown = reason
        for rx, say in _TRIAL_END:
            m = rx.search(reason)
            if m:
                s, level = say(m)
                parts = [p for p in parts if not p.startswith("I changed")] + [s]
                kind = "trial"
                break
        if kind == "note":
            parts.append(_SETTINGS_RE.sub("", reason).strip(" ;") or reason)
            reason_shown = ""
        self._add(t, " ".join(parts), level, kind, why=reason_shown, setting=setting, history=history)

    def cycle(self, d, t, history=False):
        """autoen/cycle (or a 'cycle' line of the log): failures are told at once, the rest in blocks of 10."""
        n, res = d.get("n"), d.get("result")
        if n is None:
            return
        ok = res == "OK"
        if not ok:
            self._add(t, f"I saw cycle {n} fail: {res}" + (f" ({d.get('notes')})" if d.get("notes") else "") + ".",
                      "warn", "seen", history=history)
        self._cycles.append((ok, d.get("secs")))
        if int(n) % 10 == 0 and self._cycles:
            good = [s for o, s in self._cycles if o and s is not None]
            avg = sum(good) / len(good) if good else None
            self._add(t, f"Cycles up to {n}: {sum(o for o, _ in self._cycles)}/{len(self._cycles)} OK"
                      + (f", {avg:.1f} s each on average." if avg is not None else "."),
                      "good" if all(o for o, _ in self._cycles) else "info", "progress", history=history)
            self._cycles = []

    def load_history(self, log_dir=ORCH_LOGS):
        """The story so far: the newest optimizer log (a page opened in the middle of a run still has it)."""
        files = sorted(glob.glob(os.path.join(log_dir, "orch_*.jsonl")), key=os.path.getmtime)
        if not files:
            return 0
        try:
            with open(files[-1], encoding="utf-8", errors="replace") as f:
                lines = deque(f, maxlen=HISTORY_LINES)
        except OSError:
            return 0
        n = 0
        for ln in lines:
            try:
                d = json.loads(ln)
            except ValueError:
                continue
            k, t = d.get("kind"), d.get("ts") or time.time()
            if k == "start":
                self.started(t, d.get("shadow"), history=True)
            elif k == "plan":
                self.plan(d, t, history=True)
            elif k == "cycle":
                self.cycle(d, t, history=True)
            else:
                continue
            n += 1
        return n


# ============================================================================
# 4. THE CAMERA'S CASE ARRAY
# ============================================================================
CASE_COLS = ["iso", "n", "result", "secs", "rpm", "arm_speed", "colour", "colour_share", "light", "backlog_max",
             "train", "stuck_s", "slip"]


def cases_tail(limit=10, cases_dir=CASES_DIR):
    files = sorted(glob.glob(os.path.join(cases_dir, "cases_*.csv")))
    if not files:
        return {"file": None, "rows": [], "columns": CASE_COLS}
    try:
        with open(files[-1], newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        rows = []
    return {"file": os.path.basename(files[-1]), "total": len(rows), "columns": CASE_COLS,
            "rows": [{k: r.get(k, "") for k in CASE_COLS} for r in rows[-limit:]]}
