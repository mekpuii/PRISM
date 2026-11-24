#!/usr/bin/env python3
"""build_data_pack.py - everything about the PRISM folder and pt5 in one workbook + CSV files, for tables and graphs.
Run:  /tmp/xlenv/bin/python ~/Desktop/PRISM/docs/data_pack/build_data_pack.py     (needs openpyxl: python3 -m venv /tmp/xlenv && /tmp/xlenv/bin/pip install openpyxl)
Reads only: the logs, calibration and settings files of the PRISM folder. Writes: docs/data_pack/*.csv and PRISM_pt5_data_pack.xlsx."""
import ast, csv, datetime as D, glob, json, os, re, statistics as st, sys, time
from collections import Counter

PRISM = os.path.expanduser("~/Desktop/PRISM"); OUT = os.path.join(PRISM, "docs", "data_pack")
RIFFS = [os.path.join(PRISM, "autoencoder", "riff_logs"), os.path.join(PRISM, "automation", "riff_logs")]
sheets = {}          # name -> (header list, rows list)


def put(name, header, rows):
    sheets[name] = (header, rows)


def find(stamp, kind, ext):
    for d in RIFFS:
        p = os.path.join(d, f"autoen_{kind}_{stamp}.{ext}")
        if os.path.exists(p):
            return p
    return None


def med(v):
    return round(st.median(v), 2) if v else None


# ------------------------------------------------------------------ Runs (one row per run)
def run_rows():
    notes = {
        "20261112_010644": ("pt1", "PRISM", "pt1 flow: the optimizer plans every push; no colour sorting yet"),
        "20261112_142348": ("pt3/pt4", "PRISM", "belt filled, every push verified; sorting not working (1 left / 9 right)"),
        "20261112_181716": ("pt4", "PRISM", "looked fast but the cubes PILED UP (belt length 468 -> 230 mm)"),
        "20261112_200041": ("pt5", "PRISM", "run A: first pt5; ~22 of 32 min resting with an empty feeder"),
        "20261112_210457": ("pt5", "PRISM_friend", "run B: repeat of pt5 (same code); ended paused"),
        "20261112_212059": ("pt5", "PRISM", "start-up test (too short)"),
        "20261112_212414": ("pt5", "PRISM", "start-up test (too short)"),
        "20261112_213017": ("pt5 + rest probe", "PRISM", "run C: ended with a feeder servo alarm (servo very hot)"),
        "20261112_232142": ("pt5 + rest probe", "PRISM", "short test; the 1.83 V rail minimum is a measurement dropout"),
        "20261113_002452": ("pt5 + rest probe + heat guard", "PRISM", "run D: the 100-cube run (feeder angles 170/10 as sent to the Mega)"),
        "20261113_162012": ("pt5 + optimizer", "PRISM_mek", "200cubes: the 200-cube run; belt current/rail NOT logged -> energy is the arm only"),
        "20261113_184928": ("pt6", "PRISM", "short test with the fault system"),
    }
    rows, seen = [], set()
    for rep in (os.path.join(PRISM, "run_reports", "runs.csv"), os.path.join(PRISM, "automation", "run_reports", "runs.csv")):
        if not os.path.exists(rep):
            continue
        auto = "automation" in rep
        for r in csv.DictReader(open(rep)):
            stamp = r["run"]
            if stamp in seen:
                continue
            seen.add(stamp)
            ver, folder, note = notes.get(stamp, ("plain automation (no optimizer)" if auto else "pt5-family", "PRISM/automation" if auto else "PRISM", ""))
            ex = {}
            cp = find(stamp, "cycles", "csv")
            if cp:
                cy = list(csv.DictReader(open(cp)))
                ok = [c for c in cy if c["result"] == "OK"]
                f = lambda k, rows_=cy: [float(c[k]) for c in rows_ if c.get(k) not in (None, "")]
                ex = {"left": sum(c["drop"] == "left" for c in ok), "right_down": sum(c["drop"] == "down" for c in ok),
                      "no_block": sum(c["result"] == "NO_BLOCK" for c in cy), "other_fail": sum(c["result"] not in ("OK", "NO_BLOCK") for c in cy),
                      "median_cycle_s": med(f("seconds", ok)), "median_transport_s": med(f("transport_s", ok)),
                      "median_grip_s": med(f("grip_s", ok)), "median_carry_s": med(f("carry_s", ok)),
                      "total_transport_s": round(sum(f("transport_s")), 1)}
            tp = find(stamp, "tx", "log")
            if tp:
                tx = open(tp, errors="replace").read().splitlines()
                t = lambda l: D.datetime.strptime(l[:12], "%H:%M:%S.%f")
                fe = [t(l) for l in tx if l.endswith("TX X,FEED")]
                gaps = [(b - a).total_seconds() for a, b in zip(fe, fe[1:]) if 0 < (b - a).total_seconds() < 120]
                pend, durs = None, []
                for l in tx:
                    if l.endswith("TX X,FEED"):
                        pend = t(l)
                    elif "RX X,E,FEED_DONE" in l and pend:
                        durs.append((t(l) - pend).total_seconds()); pend = None
                cfg = next((l for l in tx if "TX X,FEEDCFG" in l), "")
                conv = next((l for l in tx if "TX X,CONV," in l), "")
                ex.update({"pushes": len(fe), "median_gap_between_pushes_s": med(gaps), "median_push_s": med(durs),
                           "feeder_cfg_home_push_hold_return": cfg.split("FEEDCFG,")[-1] if cfg else "",
                           "belt_rpm": conv.split("CONV,")[-1] if conv else ""})
            rows.append({"run": stamp, "start": r["start"], "version": ver, "folder": folder, "note": note, **r, **ex})
    return rows


RUN_COLS = ["run", "start", "version", "folder", "minutes", "cycles", "cubes_ok", "left", "right_down", "no_block", "other_fail",
            "s_per_cube_median", "median_cycle_s", "median_transport_s", "median_grip_s", "median_carry_s", "total_transport_s",
            "energy_j", "j_per_cube", "arm_w", "belt_w", "servo_temp_max_c", "belt_temp_max_c", "rail_min_v", "pi_temp_max_c",
            "pi_cpu_avg_pct", "emergency_grips", "warnings", "pushes", "median_gap_between_pushes_s", "median_push_s",
            "feeder_cfg_home_push_hold_return", "belt_rpm", "note"]


# ------------------------------------------------------------------ per-cycle and per-minute series
def cycles_sheet(stamp):
    cp = find(stamp, "cycles", "csv")
    out = []
    for c in csv.DictReader(open(cp)):
        out.append([int(c["cycle_total"]), c["start"], float(c["seconds"] or 0), c["result"], c["drop"],
                    c.get("transport_s"), c.get("grip_s"), c.get("carry_s"), c.get("travel_counts"), c.get("notes", "")[:60]])
    return ["cycle", "start", "seconds", "result", "bin (left/down=right)", "transport_s", "grip_s", "carry_s", "travel_counts", "notes"], out


def feed_sheet(stamp):
    tp = find(stamp, "tx", "log")
    tx = open(tp, errors="replace").read().splitlines()
    t = lambda l: D.datetime.strptime(l[:12], "%H:%M:%S.%f")
    fe = [t(l) for l in tx if l.endswith("TX X,FEED")]
    done = [t(l) for l in tx if "RX X,E,FEED_DONE" in l]
    rows = []
    for i, a in enumerate(fe):
        d = next((x for x in done if x >= a), None)
        gap = (a - fe[i - 1]).total_seconds() if i else None
        rows.append([i + 1, a.strftime("%H:%M:%S.%f")[:-3], None if gap is None else round(gap, 2),
                     None if d is None else round((d - a).total_seconds(), 2)])
    return ["push #", "time", "seconds since the previous push", "push duration s (FEED -> FEED_DONE)"], rows


def minute_sheet(stamp):
    rp = find(stamp, "raw", "csv")
    by = {}
    for r in csv.DictReader(open(rp)):
        m = r["time"][:16]
        b = by.setdefault(m, {"temps": [], "belt_t": [], "arm_w": [], "belt_w": [], "volt": []})
        try:
            b["temps"].append(max(float(r[f"J{i}_temp"]) for i in range(1, 7) if r.get(f"J{i}_temp")))
            w = sum(float(r[f"J{i}_current"]) * float(r[f"J{i}_voltage"]) / 1000.0 for i in range(1, 7) if r.get(f"J{i}_current") and r.get(f"J{i}_voltage"))
            b["arm_w"].append(w)
        except (ValueError, KeyError):
            pass
        try:
            b["belt_t"].append(float(r["Belt_temp"]))
        except (ValueError, KeyError):
            pass
        try:
            b["belt_w"].append(float(r["Belt_current"]) * float(r["Belt_voltage"]) / 1000.0)
            b["volt"].append(float(r["Belt_voltage"]))
        except (ValueError, KeyError):
            pass
    rows = []
    for i, (m, b) in enumerate(sorted(by.items())):
        rows.append([i, m[11:16], max(b["temps"]) if b["temps"] else None, max(b["belt_t"]) if b["belt_t"] else None,
                     round(st.mean(b["arm_w"]), 3) if b["arm_w"] else None, round(st.mean(b["belt_w"]), 3) if b["belt_w"] else None,
                     min(b["volt"]) if b["volt"] else None])
    return ["minute", "clock", "hottest arm servo C", "belt motor C", "arm power W (mean)", "belt power W (mean)", "12 V rail min V"], rows


# ------------------------------------------------------------------ parameters
def argparse_params(path, label):
    src = open(path, errors="replace").read()
    tree = ast.parse(src)
    rows = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "add_argument" and n.args and isinstance(n.args[0], ast.Constant):
            name = n.args[0].value
            kw = {k.arg: k.value for k in n.keywords}
            def lit(x):
                try:
                    return ast.literal_eval(x)
                except Exception:
                    return ast.unparse(x) if x is not None else None
            help_ = lit(kw["help"]) if "help" in kw else ""
            rows.append([label, name, lit(kw["default"]) if "default" in kw else None,
                         ast.unparse(kw["type"]) if "type" in kw else "", str(help_).replace("\n", " ")[:260]])
    return rows


def const_params(path, label):
    src = open(path, errors="replace").read(); lines = src.splitlines(); tree = ast.parse(src); rows = []
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and re.fullmatch(r"[A-Z][A-Z0-9_]+", n.targets[0].id):
            try:
                v = ast.literal_eval(n.value)
            except Exception:
                continue
            c = lines[n.lineno - 1].split("#", 1)[1].strip() if "#" in lines[n.lineno - 1] else ""
            rows.append([label, n.targets[0].id, json.dumps(v) if not isinstance(v, (int, float, str)) else v, c[:230]])
    return rows


def flat_json(path, label, prefix=""):
    d = json.load(open(path)); rows = []
    def walk(x, p):
        if isinstance(x, dict):
            for k, v in x.items():
                if k.startswith("_doc"):
                    continue
                walk(v, f"{p}.{k}" if p else k)
        else:
            rows.append([label, p, json.dumps(x) if isinstance(x, (list, tuple)) else x])
    walk(d, prefix)
    return rows


def main():
    os.makedirs(OUT, exist_ok=True)
    runs = run_rows()
    put("Runs", RUN_COLS, [[r.get(c) for c in RUN_COLS] for r in runs])

    # --- version comparison (numbers from docs/PRISM_pt5_EXPERIMENTS.md and the information book)
    put("Versions", ["version", "date", "median s per cube", "J per cube (belt+arm unless noted)", "cubes", "what it is / what changed"], [
        ["serial (old one-at-a-time cycle)", "2026-11-11 22:08", 18.7, 29.0, 12, "feed -> belt -> pick -> place, one cube at a time"],
        ["serial, gentle push", "2026-11-11 22:44", 19.9, 32.0, 6, "slower push"],
        ["pipeline B", "2026-11-11 23:10", 11.8, 168, 2, "pushes during the lift failed (shared 12 V supply)"],
        ["pt1 flow", "2026-11-12 01:07", 11.6, 19.9, 31, "the optimizer plans every push; no colour sorting"],
        ["pt3/pt4 fill", "2026-11-12 14:23", 11.7, 18.6, 10, "belt filled, every push verified; sorting 1 left / 9 right"],
        ["pt4 (piled up)", "2026-11-12 18:17", 10.4, 14.1, 8, "looked fast but cubes piled up at the feeder (ride 468 -> 230 mm)"],
        ["pt5 run A", "2026-11-12 20:00", 13.3, 23.6, 33, "room-first spacing, camera may only block a push; orange->left, rest->right"],
        ["pt5 run B (PRISM_friend)", "2026-11-12 21:07", 11.8, 20.2, 30, "repeat of pt5"],
        ["pt5 + rest probe run C", "2026-11-12 21:30", 11.0, 16.2, 44, "restart after a refill in seconds; ended with a feeder servo alarm"],
        ["pt5 run D (100 cubes)", "2026-11-13 00:25", 13.0, 22.2, 93, "93 of 102 cycles, 0 wrong bins"],
        ["pt5 200cubes (PRISM_mek)", "2026-11-13 16:20", 11.4, "not measured (arm only: 9.1)", 199, "199 of 200 cycles in 39.9 min"],
        ["pt6", "2026-11-13 17:40", None, None, None, "pt5 + fault system, heat guard, emergency grip, sleep/relay, Limits zone, run report (saved in versions/pt6; NOT active in the main folder since 19:40)"],
    ])

    # --- folders
    desc = {"analysis": "arm / belt analyzers", "arduino_mega_2560": "Mega firmware (belt, feeder, servo bus)", "arm_cam": "wrist camera tools (emergency grip)",
            "autoencoder": "autoen: owns the Mega, runs the pick cycle; riff_logs = raw run data", "automation": "plain automation without the optimizer",
            "backup": "every earlier version of a changed file", "competition": "speed/energy sheets", "config": "health thresholds",
            "dashboard": "web dashboard", "docs": "documents", "health": "health monitor", "logger": "MQTT -> sqlite loggers", "orchestrator": "the optimizer (brain)",
            "run_logs": "program output per start", "run_reports": "per-run temperature/power reports", "versions": "frozen copies pt1, pt2, pt4, pt5, pt6",
            "vision": "camera: colour + position of cubes", "archive": "data of the deleted PRISM_mek / PRISM_dupi / PRISM-Mek copies", "data": "old databases"}
    rows = []
    for d in sorted(os.listdir(PRISM)):
        p = os.path.join(PRISM, d)
        if os.path.isdir(p) and not d.startswith("."):
            n = sum(len(f) for _, _, f in os.walk(p)); sz = sum(os.path.getsize(os.path.join(r, x)) for r, _, f in os.walk(p) for x in f if os.path.exists(os.path.join(r, x)))
            rows.append([d, desc.get(d, ""), n, round(sz / 1e6, 1)])
    put("Folders", ["folder", "what it is", "files", "size MB"], rows)

    # --- parameters
    prm = []
    prm += argparse_params(os.path.join(PRISM, "autoencoder", "auto_en_new.py"), "autoen settings (command line / web page)")
    prm += argparse_params(os.path.join(PRISM, "orchestrator", "main.py"), "optimizer program options")
    prm += argparse_params(os.path.join(PRISM, "vision", "vision_usb.py"), "vision program options")
    put("Settings_defaults", ["group", "name", "default", "type", "meaning"], prm)
    const = []
    for f, lab in (("autoencoder/auto_en_new.py", "autoen constants"), ("orchestrator/optimize.py", "optimizer constants"),
                   ("orchestrator/actuate.py", "actuator constants"), ("orchestrator/observe.py", "observer constants"),
                   ("orchestrator/forecast.py", "forecast constants"), ("orchestrator/scene.py", "scene (light) constants"),
                   ("orchestrator/camgeo.py", "camera geometry constants"), ("orchestrator/tuner.py", "tuner constants"),
                   ("orchestrator/sortq.py", "sorting constants"), ("vision/vision_usb.py", "vision constants")):
        if os.path.exists(os.path.join(PRISM, f)):
            const += const_params(os.path.join(PRISM, f), lab)
    put("Constants_in_code", ["group", "name", "value", "comment in the code"], const)
    cal = []
    for f, lab in (("vision/vision_calibration_usb.json", "camera calibration (colours, working area, light)"), ("orchestrator/camera_geometry.json", "camera geometry (belt in pixels, ride)"),
                   ("autoencoder/gripper_cal.json", "gripper calibration"), ("autoencoder/feeder_presets.json", "feeder presets"),
                   ("orchestrator/sort_bins.json", "sorting bins"), ("autoencoder/rest_state.json", "arm rest state")):
        if os.path.exists(os.path.join(PRISM, f)):
            try:
                cal += flat_json(os.path.join(PRISM, f), lab)
            except Exception as e:
                cal.append([lab, "(unreadable)", str(e)])
    put("Calibration_files", ["file", "key", "value"], cal)

    # --- what the optimizer has learned
    ls = json.load(open(os.path.join(PRISM, "orchestrator", "learned_state.json")))
    lr = []
    for c in ls["configs"]:
        n, ok = c["n"], c["ok"]
        lr.append([c["rpm"], c["arm_speed"], n, ok, round(ok / n, 3) if n else None, round(c["j_sum"] / c["j_n"], 2) if c["j_n"] else None,
                   c["j_n"], round(c["t_sum"] / c["t_n"], 2) if c["t_n"] else None, c["t_n"]])
    put("Learned_settings", ["belt rpm", "arm speed", "cycles", "ok", "success rate", "J per cycle (mean)", "J samples", "seconds per cycle (mean)", "t samples"], lr)
    tn = ls.get("tuner", {})
    put("Learned_tuner", ["item", "value"], [["champion", json.dumps(tn.get("champ"))], ["feeder moved pushes", ls["feeder"].get("moved")],
                                             ["feeder push_ms learned", ls["feeder"].get("push_ms")], ["feeder push_fail_ms", ls["feeder"].get("push_fail_ms")]]
        + [[f"tuner try {i + 1}", json.dumps(h)] for i, h in enumerate(tn.get("history", []))])

    # --- series for graphs
    for stamp, tag in (("20261113_002452", "RunD_100cubes"), ("20261113_162012", "200cubes")):
        if find(stamp, "cycles", "csv"):
            put(f"Cycles_{tag}", *cycles_sheet(stamp))
        if find(stamp, "tx", "log"):
            put(f"Feeds_{tag}", *feed_sheet(stamp))
        if find(stamp, "raw", "csv"):
            put(f"PerMinute_{tag}", *minute_sheet(stamp))

    # --- CSV files
    for name, (h, rows) in sheets.items():
        with open(os.path.join(OUT, f"{name}.csv"), "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(h); w.writerows(rows)

    # --- the workbook (with starter charts)
    from openpyxl import Workbook
    from openpyxl.chart import BarChart, LineChart, Reference
    from openpyxl.styles import Font, PatternFill, Alignment
    from openpyxl.utils import get_column_letter as L
    wb = Workbook(); wb.remove(wb.active)
    for name, (h, rows) in sheets.items():
        ws = wb.create_sheet(name[:31]); ws.append(h)
        for r in rows:
            ws.append([("" if v is None else v) if not isinstance(v, (list, dict)) else json.dumps(v) for v in r])
        for c in ws[1]:
            c.font = Font(bold=True, color="FFFFFF"); c.fill = PatternFill("solid", fgColor="1F3864"); c.alignment = Alignment(wrap_text=True, vertical="center")
        ws.freeze_panes = "A2"
        for i, col in enumerate(ws.columns, 1):
            ws.column_dimensions[L(i)].width = min(60, max(10, max(len(str(c.value or "")) for c in list(col)[:60]) + 2))
    def chart_bar(sheet, title, ycol, xcol, anchor, ytitle):
        ws = wb[sheet]; n = ws.max_row
        ch = BarChart(); ch.title = title; ch.y_axis.title = ytitle; ch.height, ch.width = 9, 22
        ch.add_data(Reference(ws, min_col=ycol, min_row=1, max_row=n), titles_from_data=True); ch.set_categories(Reference(ws, min_col=xcol, min_row=2, max_row=n))
        ws.add_chart(ch, anchor)
    def chart_line(sheet, title, cols, anchor, ytitle):
        ws = wb[sheet]; n = ws.max_row
        ch = LineChart(); ch.title = title; ch.y_axis.title = ytitle; ch.x_axis.title = "minute"; ch.height, ch.width = 9, 22
        for c in cols:
            ch.add_data(Reference(ws, min_col=c, min_row=1, max_row=n), titles_from_data=True)
        ch.set_categories(Reference(ws, min_col=1, min_row=2, max_row=n)); ws.add_chart(ch, anchor)
    chart_bar("Versions", "Median seconds per cube by version", 3, 1, "H2", "s per cube")
    for tag in ("RunD_100cubes", "200cubes"):
        if f"PerMinute_{tag}" in wb.sheetnames:
            chart_line(f"PerMinute_{tag}"[:31], f"Temperatures, {tag}", [3, 4], "I2", "deg C")
        if f"Cycles_{tag}"[:31] in wb.sheetnames:
            ws = wb[f"Cycles_{tag}"[:31]]; ch = LineChart(); ch.title = f"Seconds per cycle, {tag}"; ch.height, ch.width = 9, 22
            ch.add_data(Reference(ws, min_col=3, min_row=1, max_row=ws.max_row), titles_from_data=True); ws.add_chart(ch, "L2")
        if f"Feeds_{tag}"[:31] in wb.sheetnames:
            ws = wb[f"Feeds_{tag}"[:31]]; ch = LineChart(); ch.title = f"Seconds between pushes, {tag}"; ch.height, ch.width = 9, 22
            ch.add_data(Reference(ws, min_col=3, min_row=1, max_row=ws.max_row), titles_from_data=True); ws.add_chart(ch, "G2")
    wb.save(os.path.join(OUT, "PRISM_pt5_data_pack.xlsx"))
    print("sheets:", {k: len(v[1]) for k, v in sheets.items()})


main()
