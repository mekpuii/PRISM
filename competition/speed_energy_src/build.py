import json, sys
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter as L
S = sys.argv[1]; OUT = sys.argv[2]
grid = json.load(open(S + "/grid.json")); a1 = json.load(open(S + "/a1.json"))
bb = json.load(open("/home/mekkhram/Desktop/PRISM/orchestrator/cal/baseline_belt_summary.json"))["rpm"]

F = "Arial"
f_norm = Font(name=F, size=10); f_bold = Font(name=F, size=10, bold=True); f_in = Font(name=F, size=10, color="0000FF")
f_link = Font(name=F, size=10, color="008000"); f_title = Font(name=F, size=14, bold=True); f_h = Font(name=F, size=10, bold=True, color="FFFFFF")
fill_h = PatternFill("solid", fgColor="1F3864"); fill_in = PatternFill("solid", fgColor="FFFF00")
fill_g = PatternFill("solid", fgColor="E2EFDA"); fill_r = PatternFill("solid", fgColor="FCE4D6"); fill_y = PatternFill("solid", fgColor="FFF2CC"); fill_s = PatternFill("solid", fgColor="D9E1F2")
wrap = Alignment(wrap_text=True, vertical="top"); thin = Side(style="thin", color="BFBFBF"); box = Border(left=thin, right=thin, top=thin, bottom=thin)
wb = Workbook()

def header(ws, row, names, height=45):
    for i, n in enumerate(names, 1):
        c = ws.cell(row=row, column=i, value=n); c.font = f_h; c.fill = fill_h; c.alignment = Alignment(wrap_text=True, vertical="center", horizontal="center"); c.border = box
    ws.row_dimensions[row].height = height
def put(ws, r, c, v, font=f_norm, fmt=None, fill=None, al=None):
    x = ws.cell(row=r, column=c, value=v); x.font = font; x.border = box
    if fmt: x.number_format = fmt
    if fill: x.fill = fill
    if al: x.alignment = al
    return x
def widths(ws, ws_w):
    for i, w in enumerate(ws_w, 1): ws.column_dimensions[L(i)].width = w

# ============================================================ Baselines (all measured constants, one place)
wsB = wb.active; wsB.title = "Baselines"
header(wsB, 1, ["Quantity", "Key", "Value", "Unit", "Kind", "Source / how it was measured"], 30)
B = {}
bl = [
 ("BELT", None),
 ("Belt speed per rpm", "mmps", 2.81, "mm/s per rpm", "Measured", "1800 mm in 57.57 s at 11.11 rpm; orchestrator/predict.py BELT_MM_S_PER_RPM"),
 ("Empty belt power at 8 rpm", "pb8", bb["8"]["W_mean"], "W", "Measured", "beltpwr_20261025_164625, 2 x 70 s, belt INA260 V x I (sd %.3f W)" % bb["8"]["W_std"]),
 ("Empty belt power at 11.1 rpm", "pb11", bb["11.1"]["W_mean"], "W", "Measured", "same run (sd %.3f W)" % bb["11.1"]["W_std"]),
 ("Empty belt power at 15 rpm", "pb15", bb["15"]["W_mean"], "W", "Measured", "same run (sd %.3f W)" % bb["15"]["W_std"]),
 ("Empty belt power at 20 rpm", "pb20", bb["20"]["W_mean"], "W", "Measured", "same run (sd %.3f W)" % bb["20"]["W_std"]),
 ("Belt power per rpm (straight-line fit through zero)", "pbslope", "=(8*C4+11.1*C5+15*C6+20*C7)/(8^2+11.1^2+15^2+20^2)", "W per rpm", "Calculated", "least-squares line through the origin over the 4 rows above"),
 ("Belt stopped, driver on, NOT braked", "pb_idle", 0.048, "W", "Measured", "OPTIMIZER_DATA.md section 2 (3.9 mA at 12.18 V), autoen idle"),
 ("Belt stopped and braked", "pb_brake", bb["0"]["W_mean"], "W", "Measured", "beltpwr_20261025_164625, rpm 0 = X,BRAKE"),
 ("ARM", None),
 ("Arm holding HOME pose", "pa_home", 0.084, "W", "Measured", "base_20261025_190036, 70 s hold (sd 0.053 W)"),
 ("Arm holding REST pose, torque on", "pa_rest", 0.30, "W", "Measured", "4 runs base_20261025_18xxxx/190640, range 0.28-0.35 W"),
 ("Move home -> rest: time", "t_h2r", 37, "s", "Measured", "4 runs, 36.8-38.7 s"),
 ("Move home -> rest: energy", "e_h2r", 10.4, "J", "Measured", "same runs"),
 ("Move rest -> home: time", "t_r2h", 36, "s", "Measured", "3 runs, 35.8-36.4 s"),
 ("Move rest -> home: energy", "e_r2h", 10.1, "J", "Measured", "same runs"),
 ("EMPTY FEEDER AND IDLE", None),
 ("One empty cycle WITHOUT the optimizer (belt runs the full 25 s timeout)", "e_empty", 49.5, "J", "Measured", "gap_20261026_1258, live 2026-10-26 12:59 (belt ~35 J of it)"),
 ("Time of that empty cycle", "t_empty", 32.6, "s", "Measured", "same run"),
 ("Empty cycles before autoen stops with an error", "n_empty", 3, "cycles", "Setting", "auto_en_new.py --max-fails default"),
 ("Empty feed WITH the optimizer (camera aborts the transport)", "e_empty_p", 11.0, "J", "Measured", "live 2026-10-26 13:33, logs/orch_20261026_133258.jsonl"),
 ("Empty feed -> idle time WITH the optimizer", "t_empty_p", 12.7, "s", "Measured", "same run"),
 ("Idle with the optimizer: arm", "pa_idle_p", 0.082, "W", "Measured", "same run (arm home)"),
 ("Idle with the optimizer: belt (braked)", "pb_idle_p", 0.0003, "W", "Measured", "same run"),
 ("LIVE RUNS WITH THE OPTIMIZER", None),
 ("PRISM live: energy per cube", "j_prism", 22.0, "J per cube", "Measured", "2026-10-26 13:33, 7/7 OK (the 12:59 run gave 21.9 J over 18 cubes)"),
 ("PRISM live: speed", "cpm_prism", 3.24, "cubes/min", "Measured", "same run (the 12:59 run gave 3.12)"),
 ("NORMAL MODE (autoen defaults, no optimizer)", None),
 ("Default belt rpm", "rpm_def", 6.7, "rpm", "Setting", "auto_en_new.py --rpm default"),
 ("Default arm speed", "arm_def", 1.0, "x", "Setting", "auto_en_new.py --arm-speed default"),
 ("CYCLE-TIME MODEL  s = a + b / arm_speed + c / rpm", None),
 ("a", "ma", 8.695, "s", "Fitted", "orchestrator/model.json, 264 cycles, R2 0.885, RMSE 1.62 s"),
 ("b", "mb", 2.867, "s", "Fitted", "same"),
 ("c", "mc", 158.914, "s x rpm", "Fitted", "same"),
 ("VISION COMPUTE LOAD (RealSense, old camera)", None),
 ("CPU at 6 fps", "cpu6", 0.38, "of one core", "Measured", "vision/vision_load.py 2026-10-25"),
 ("CPU at 15 fps", "cpu15", 0.87, "of one core", "Measured", "same; Pi ran about 15 C hotter"),
 ("METHOD CONSTANTS", None),
 ("Servo current register unit", "ma_count", 6.5, "mA per count", "Datasheet", "STS3215; autoen_node.py MA_PER_COUNT"),
 ("Longest sample gap counted", "maxdt", 0.5, "s", "Setting", "a telemetry dropout longer than this is not counted as energy"),
 ("Telemetry rate", "hz", 10, "Hz", "Setting", "arm/telemetry and belt/telemetry"),
]
r = 2
for row in bl:
    if row[1] is None:
        c = put(wsB, r, 1, row[0], f_bold, fill=fill_s)
        for k in range(2, 7): put(wsB, r, k, None, fill=fill_s)
    else:
        put(wsB, r, 1, row[0], al=wrap); put(wsB, r, 2, row[1])
        isf = isinstance(row[2], str) and row[2].startswith("=")
        put(wsB, r, 3, row[2], f_norm if isf else f_in, "0.0000" if (isf or (isinstance(row[2], float) and row[2] < 1)) else "General")
        put(wsB, r, 4, row[3]); put(wsB, r, 5, row[4]); put(wsB, r, 6, row[5], al=wrap)
        B[row[1]] = "Baselines!$C$%d" % r
    r += 1
assert wsB["B4"].value == "pb8" and wsB["B7"].value == "pb20"
widths(wsB, [52, 11, 11, 14, 11, 80]); wsB.freeze_panes = "A2"

# ============================================================ Cycles (raw measured, one row per cube)
wsC = wb.create_sheet("Cycles")
PH = ["IDLE", "FEED", "TRANSPORT", "GRIP", "CARRY_HOME", "CARRY_BIN", "RELEASE", "HOME"]
PHD = {"IDLE": "Gap before feed (arm at home)", "FEED": "Feeder pushes a cube on", "TRANSPORT": "Cube rides the belt to the IR", "GRIP": "Arm grips the cube",
       "CARRY_HOME": "Lift back to home", "CARRY_BIN": "Carry to the bin (left or down)", "RELEASE": "Open the gripper", "HOME": "Return to home"}
cols = ["Block", "Cycle in block", "Time", "Belt rpm", "Arm speed", "Carry speed", "Result", "Bin", "Counted (0 = first cycle of a block)",
        "Cycle time (s)", "autoen cycle time (s)"] + ["%s time (s)" % p for p in PH] + ["Arm J in %s" % p for p in PH] + \
       ["Belt J while running", "Belt J while stopped", "Arm J", "Belt J", "Total J", "Average power (W)"] + ["J%d J" % k for k in range(1, 7)] + \
       ["Arm moving (s)", "Arm still (s)", "Arm J while moving", "Arm J while still", "Belt running (s)", "Belt W while running", "Belt rpm while running (mean, with ramps)",
        "Belt turns", "Belt travel (mm)", "Hottest joint (C)", "Belt motor temp (C)", "Servo bus (V)", "Feed -> arrived (s)", "Notes from autoen"] + ["Belt J in %s" % p for p in PH]
header(wsC, 1, cols, 60)
CI = {n: L(i) for i, n in enumerate(cols, 1)}
def ph(c, d, p):
    if p == "CARRY_BIN": return d.get("CARRY_LEFT", 0) + d.get("CARRY_DOWN", 0)
    return d.get(p, 0)
def cyc_rows(ws, cycles, full=True):
    for n, c in enumerate(cycles):
        r = n + 2
        vals = [c["block"], c["i"], c["when"], c["rpm"], c["arm"], c["carry"], c["cls"], c.get("drop") or "", "=IF(B%d=1,0,1)" % r,
                round(c["t1"] - c["t0"], 2), c["secs"]] + [round(ph(c, c["ph_s"], p), 2) for p in PH] + [round(ph(c, c["ph_arm"], p), 3) for p in PH] + \
               [round(c["belt_run_J"], 3), round(c["belt_stop_J"], 4),
                "=SUM(%s%d:%s%d)" % (CI["Arm J in IDLE"], r, CI["Arm J in HOME"], r), "=%s%d+%s%d" % (CI["Belt J while running"], r, CI["Belt J while stopped"], r),
                "=%s%d+%s%d" % (CI["Arm J"], r, CI["Belt J"], r), "=%s%d/%s%d" % (CI["Total J"], r, CI["Cycle time (s)"], r)] + [round(x, 3) for x in c["jJ"]] + \
               [round(c["mv_s"], 2), round(c["still_s"], 2), round(c["mv_J"], 3), round(c["still_J"], 3), round(c["belt_run_s"], 2), c["bw_run"], c["bv"],
                round((c["pos1"] or 0) - (c["pos0"] or 0), 4), "=%s%d*60*%s" % (CI["Belt turns"], r, B["mmps"]), c["tmax"], c["btemp"], c["volt"],
                round(c["arr_t"] - c["feed_t"], 2) if c.get("arr_t") and c.get("feed_t") else None, c["notes"]] + [round(ph(c, c["ph_belt"], p), 3) for p in PH]
        for k, v in enumerate(vals, 1):
            isf = isinstance(v, str) and v.startswith("=")
            x = put(ws, r, k, v, f_norm if isf or k in (3, 7, 8) or cols[k - 1].startswith("Notes") else f_in)
            if isf and k != 9: x.number_format = "0.00"
        if c["cls"] != "OK":
            for k in range(1, 12): ws.cell(row=r, column=k).fill = fill_r
cyc_rows(wsC, grid["cycles"])
NC = len(grid["cycles"]); R1 = NC + 1
wsC.freeze_panes = "H2"; wsC.auto_filter.ref = "A1:%s%d" % (L(len(cols)), R1)
widths(wsC, [7, 7, 19, 7, 7, 7, 20, 7, 10] + [10] * (len(cols) - 9)); wsC.column_dimensions[CI["Notes from autoen"]].width = 60
def rng(name, ws="Cycles", last=None): return "%s!$%s$2:$%s$%d" % (ws, CI[name], CI[name], last or R1)
def avg(name, r, extra=""): return "AVERAGEIFS(%s,%s,$A%d,%s,$B%d,%s,1%s)" % (rng(name), rng("Belt rpm"), r, rng("Arm speed"), r, rng("Counted (0 = first cycle of a block)"), extra)
def cnt(r, extra=""): return "COUNTIFS(%s,$A%d,%s,$B%d%s)" % (rng("Belt rpm"), r, rng("Arm speed"), r, extra)
USE = ",%s,1" % rng("Counted (0 = first cycle of a block)"); OKC = ',%s,"OK"' % rng("Result")
def sd(name, r, meancell, ncell):
    return "=SQRT(SUMPRODUCT((%s=$A%d)*(%s=$B%d)*(%s=1)*(%s-%s)^2)/(%s-1))" % (rng("Belt rpm"), r, rng("Arm speed"), r, rng("Counted (0 = first cycle of a block)"), rng(name), meancell, ncell)

# ============================================================ Settings
wsS = wb.create_sheet("Settings")
confs = sorted({(c["rpm"], c["arm"]) for c in grid["cycles"]})
scol = ["Belt rpm", "Arm speed", "Belt speed (mm/s)", "Cycles counted", "OK cycles (of all)", "All cycles", "Success rate",
        "Seconds per cube", "sd (s)", "Cubes per minute", "Cubes per hour", "Arm J per cube", "Belt J per cube", "TOTAL J per cube", "sd (J)",
        "Average power while producing (W)", "Arm J while moving", "Arm J while still (waiting)", "Arm still: share of cycle time", "Belt running: share of cycle time",
        "Belt W while running", "Belt W vs empty-belt baseline", "Belt travel per cube (mm)", "Wh per 1000 cubes", "J per GOOD cube (failures charged to good cubes)",
        "Seconds per cube, OK cycles only", "J per cube, OK cycles only", "Model: seconds per cube", "Model error (s)", "Time vs fastest setting", "Energy vs lowest setting",
        "95 % margin on J per cube (+/-)", "Arm power while still (W)", "Arm power while moving (W)", "Hottest joint, mean (C)"]
header(wsS, 1, scol, 75); SI = {n: L(i) for i, n in enumerate(scol, 1)}
SR = {}
for n, (rpm, arm) in enumerate(confs):
    r = n + 2; SR[(rpm, arm)] = r
    base = {8.0: B["pb8"], 11.1: B["pb11"], 15.0: B["pb15"], 20.0: B["pb20"]}[rpm]
    lastS = len(confs) + 1
    v = [rpm, arm, "=A%d*%s" % (r, B["mmps"]), "=" + cnt(r, USE), "=" + cnt(r, OKC), "=" + cnt(r), "=E%d/F%d" % (r, r),
         "=" + avg("Cycle time (s)", r), sd("Cycle time (s)", r, "$H%d" % r, "$D%d" % r), "=60/H%d" % r, "=3600/H%d" % r,
         "=" + avg("Arm J", r), "=" + avg("Belt J", r), "=L%d+M%d" % (r, r), sd("Total J", r, "$N%d" % r, "$D%d" % r), "=N%d/H%d" % (r, r),
         "=" + avg("Arm J while moving", r), "=" + avg("Arm J while still", r),
         "=%s/(%s+%s)" % (avg("Arm still (s)", r), avg("Arm still (s)", r), avg("Arm moving (s)", r)), "=%s/H%d" % (avg("Belt running (s)", r), r),
         "=%s/%s" % (avg("Belt J while running", r), avg("Belt running (s)", r)), "=U%d/%s-1" % (r, base), "=" + avg("Belt travel (mm)", r), "=N%d*1000/3600" % r,
         "=N%d*D%d/COUNTIFS(%s,$A%d,%s,$B%d%s%s)" % (r, r, rng("Belt rpm"), r, rng("Arm speed"), r, USE, OKC),
         "=" + avg("Cycle time (s)", r, OKC), "=" + avg("Total J", r, OKC), "=%s+%s/B%d+%s/A%d" % (B["ma"], B["mb"], r, B["mc"], r), "=AB%d-Z%d" % (r, r),
         "=H%d/MIN($H$2:$H$%d)-1" % (r, lastS), "=N%d/MIN($N$2:$N$%d)-1" % (r, lastS), "=1.96*O%d/SQRT(D%d)" % (r, r),
         "=%s/%s" % (avg("Arm J while still", r), avg("Arm still (s)", r)), "=%s/%s" % (avg("Arm J while moving", r), avg("Arm moving (s)", r)), "=" + avg("Hottest joint (C)", r)]
    fm = ["0.0", "0.0", "0.0", "0", "0", "0", "0%", "0.00", "0.00", "0.00", "0", "0.00", "0.00", "0.00", "0.00", "0.000", "0.00", "0.00", "0%", "0%", "0.000", "+0.0%;-0.0%", "0", "0.00", "0.00",
          "0.00", "0.00", "0.00", "+0.00;-0.00", "+0.0%;-0.0%;0.0%", "+0.0%;-0.0%;0.0%", "0.00", "0.000", "0.000", "0.0"]
    for k, x in enumerate(v, 1): put(wsS, r, k, x, f_in if k <= 2 else f_norm, fm[k - 1])
    for k in (8, 10, 14): wsS.cell(row=r, column=k).fill = fill_y
nS = len(confs) + 3
notes = ["How to read: one row per setting, 20 cycles each (2 blocks of 10). The first cycle of each block is not counted (start-up), so 18 cycles per setting.",
         "Yellow columns are the three headline numbers. Every value is a formula over the Cycles sheet; change nothing here.",
         "Energy is motor supplies only: belt INA260 (V x I) + the sum of the six servo currents x bus voltage. The Pi, the Mega and the motor drivers are NOT included.",
         "'J per GOOD cube' divides all energy (including failed cycles) by the cubes that actually arrived. 'OK cycles only' leaves the failed cycles out.",
         "95 % margin: two settings whose J per cube differ by less than their margins added together are not proven different by this data.",
         "Belt W vs baseline: negative = the loaded belt drew less than the empty-belt baseline (the baseline was a different run; treat +/- 8 % as noise)."]
for i, t in enumerate(notes): wsS.cell(row=nS + i, column=1, value=t).font = f_norm
wsS.freeze_panes = "C2"; widths(wsS, [8, 8] + [12] * (len(scol) - 2))
def SC(conf, name): return "Settings!$%s$%d" % (SI[name], SR[conf])

# ============================================================ Phases
wsP = wb.create_sheet("Phases")
header(wsP, 1, ["Belt rpm", "Arm speed", "Phase", "What happens", "Time (s)", "Arm J", "Belt J", "Total J", "Mean power (W)", "Share of cycle time", "Share of cycle energy", "Useful work?"], 40)
useful = {"IDLE": "No: fixed 1 s gap", "FEED": "Yes", "TRANSPORT": "Belt yes; arm only waits", "GRIP": "Yes", "CARRY_HOME": "Yes", "CARRY_BIN": "Yes", "RELEASE": "Yes", "HOME": "Yes (return)"}
r = 2; PR = {}
for conf in confs:
    first = r
    for p in PH:
        PR[(conf, p)] = r
        put(wsP, r, 1, conf[0], f_in, "0.0"); put(wsP, r, 2, conf[1], f_in, "0.0"); put(wsP, r, 3, p, f_bold); put(wsP, r, 4, PHD[p]); 
        put(wsP, r, 5, "=" + avg("%s time (s)" % p, r), fmt="0.00"); put(wsP, r, 6, "=" + avg("Arm J in %s" % p, r), fmt="0.00"); put(wsP, r, 7, "=" + avg("Belt J in %s" % p, r), fmt="0.00")
        put(wsP, r, 8, "=F%d+G%d" % (r, r), fmt="0.00"); put(wsP, r, 9, "=IF(E%d>0,H%d/E%d,0)" % (r, r, r), fmt="0.000")
        put(wsP, r, 10, "=E%d/SUM($E$%d:$E$%d)" % (r, first, first + 7), fmt="0%"); put(wsP, r, 11, "=H%d/SUM($H$%d:$H$%d)" % (r, first, first + 7), fmt="0%"); put(wsP, r, 12, useful[p])
        if p == "TRANSPORT":
            for k in range(1, 13): wsP.cell(row=r, column=k).fill = fill_y
        r += 1
    put(wsP, r, 1, conf[0], f_bold, "0.0"); put(wsP, r, 2, conf[1], f_bold, "0.0"); put(wsP, r, 3, "WHOLE CYCLE", f_bold); put(wsP, r, 4, "sum of the 8 phases", f_bold)
    for k, fm in ((5, "0.00"), (6, "0.00"), (7, "0.00"), (8, "0.00")): put(wsP, r, k, "=SUM(%s%d:%s%d)" % (L(k), first, L(k), first + 7), f_bold, fm)
    put(wsP, r, 9, "=H%d/E%d" % (r, r), f_bold, "0.000"); put(wsP, r, 10, "=SUM(J%d:J%d)" % (first, first + 7), f_bold, "0%"); put(wsP, r, 11, "=SUM(K%d:K%d)" % (first, first + 7), f_bold, "0%"); put(wsP, r, 12, None)
    for k in range(1, 13): wsP.cell(row=r, column=k).fill = fill_s
    PR[(conf, "ALL")] = r; r += 1
wsP.cell(row=r + 1, column=1, value="TRANSPORT (yellow) is the largest phase in both time and energy: the belt carries the cube while the arm holds still and draws current. Failed cycles are included in these means, as in Settings.").font = f_norm
wsP.freeze_panes = "E2"; widths(wsP, [8, 8, 14, 30, 9, 9, 9, 9, 11, 11, 11, 26])

# ============================================================ Joints
wsJ = wb.create_sheet("Joints")
jn = ["J1 base", "J2 shoulder", "J3 elbow", "J4 wrist flex", "J5 wrist roll", "J6 gripper"]
header(wsJ, 1, ["Belt rpm", "Arm speed"] + ["%s (J per cube)" % j for j in jn] + ["Arm total (J per cube)"] + ["%s share" % j for j in jn] + ["Biggest consumer"], 45)
for conf in confs:
    r = SR[conf]; put(wsJ, r, 1, conf[0], f_in, "0.0"); put(wsJ, r, 2, conf[1], f_in, "0.0")
    for k in range(6): put(wsJ, r, 3 + k, "=" + avg("J%d J" % (k + 1), r), fmt="0.00")
    put(wsJ, r, 9, "=SUM(C%d:H%d)" % (r, r), f_bold, "0.00")
    for k in range(6): put(wsJ, r, 10 + k, "=%s%d/$I%d" % (L(3 + k), r, r), fmt="0%")
    put(wsJ, r, 16, "=INDEX($C$1:$H$1,MATCH(MAX(C%d:H%d),C%d:H%d,0))" % (r, r, r, r))
rr = len(confs) + 2
put(wsJ, rr, 1, "All settings (mean)", f_bold); put(wsJ, rr, 2, None)
for k in range(3, 10): put(wsJ, rr, k, "=AVERAGE(%s2:%s%d)" % (L(k), L(k), rr - 1), f_bold, "0.00")
for k in range(6): put(wsJ, rr, 10 + k, "=%s%d/$I%d" % (L(3 + k), rr, rr), f_bold, "0%")
wsJ.cell(row=rr + 2, column=1, value="Joint names follow the usual SO-101 order (1 base ... 6 gripper); check against your arm before quoting. Joint J = bus V x current register x 6.5 mA, summed over the cycle.").font = f_norm
widths(wsJ, [18, 8] + [12] * 14); wsJ.freeze_panes = "C2"

# ============================================================ Failures
wsF = wb.create_sheet("Failures")
header(wsF, 1, ["Block", "Cycle", "Belt rpm", "Arm speed", "Result", "Time lost (s)", "Arm J", "Belt J", "Total J", "Mean OK cycle at this setting (s)", "Mean OK cycle at this setting (J)",
                "Failure costs this many good cubes of energy", "Failure costs this many good cubes of time", "What autoen said"], 60)
bad = [n + 2 for n, c in enumerate(grid["cycles"]) if c["cls"] != "OK"]
for i, cr in enumerate(bad):
    r = i + 2
    for k, nm in enumerate(["Block", "Cycle in block", "Belt rpm", "Arm speed", "Result", "Cycle time (s)", "Arm J", "Belt J", "Total J"], 1):
        put(wsF, r, k, "=Cycles!%s%d" % (CI[nm], cr), f_link, "0.0" if k in (3, 4) else ("0.00" if k >= 6 else "General"))
    okavg = lambda nm: '=AVERAGEIFS(%s,%s,$C%d,%s,$D%d,%s,"OK"%s)' % (rng(nm), rng("Belt rpm"), r, rng("Arm speed"), r, rng("Result"), USE)
    put(wsF, r, 10, okavg("Cycle time (s)"), fmt="0.00"); put(wsF, r, 11, okavg("Total J"), fmt="0.00")
    put(wsF, r, 12, "=I%d/K%d" % (r, r), fmt="0.00"); put(wsF, r, 13, "=F%d/J%d" % (r, r), fmt="0.00"); put(wsF, r, 14, "=Cycles!%s%d" % (CI["Notes from autoen"], cr), f_link)
fr = len(bad) + 2
put(wsF, fr, 1, "Mean", f_bold)
for k in range(2, 15): put(wsF, fr, k, "=AVERAGE(%s2:%s%d)" % (L(k), L(k), fr - 1) if k in (6, 7, 8, 9, 12, 13) else None, f_bold, "0.00")
put(wsF, fr + 1, 1, "Failure rate, all 240 cycles", f_bold); put(wsF, fr + 1, 6, '=COUNTIFS(%s,"<>OK")/COUNTA(%s)' % (rng("Result"), rng("Result")), f_bold, "0.0%")
put(wsF, fr + 2, 1, "Failures at arm speed 2.0", f_bold); put(wsF, fr + 2, 6, '=COUNTIFS(%s,"<>OK",%s,2)' % (rng("Result"), rng("Arm speed")), f_bold, "0")
put(wsF, fr + 3, 1, "Failures at arm speed 1.0 and 1.5", f_bold); put(wsF, fr + 3, 6, '=COUNTIFS(%s,"<>OK",%s,"<2")' % (rng("Result"), rng("Arm speed")), f_bold, "0")
for i, t in enumerate(["All five failures are the same kind: the cube never reached the IR sensor within the 40 s timeout, so the belt ran for the whole timeout.",
                       "The arm did not cause them directly (the arm was waiting). With 20 cycles per setting, 3 of 5 at arm speed 2.0 is a hint, not proof, that arm 2.0 is the cause.",
                       "The cost scales with belt rpm because belt power does: a timeout at 20 rpm burns about twice the joules of one at 11.1 rpm."]):
    wsF.cell(row=fr + 5 + i, column=1, value=t).font = f_norm
widths(wsF, [30, 7, 8, 8, 20, 10, 9, 9, 9, 13, 13, 14, 14, 50])

# ============================================================ Check run (A1, after the gripper re-calibration)
wsA = wb.create_sheet("Check run")
header(wsA, 1, ["Block", "Cycle in block", "Time", "Belt rpm", "Arm speed", "Result", "Counted", "Cycle time (s)", "Arm J", "Belt J", "Total J"], 40)
for n, c in enumerate(a1["cycles"]):
    r = n + 2
    for k, v in enumerate([c["block"], c["i"], c["when"], c["rpm"], c["arm"], c["cls"], "=IF(B%d=1,0,1)" % r, round(c["t1"] - c["t0"], 2), round(sum(c["jJ"]), 3), round(c["belt_run_J"] + c["belt_stop_J"], 3), "=I%d+J%d" % (r, r)], 1):
        put(wsA, r, k, v, f_norm if (isinstance(v, str)) else f_in, "0.00" if k == 11 else None)
NA = len(a1["cycles"]) + 1; ar = NA + 3
header(wsA, ar, ["Belt rpm", "Arm speed", "", "Check run: s per cube", "Check run: J per cube", "Grid: s per cube", "Grid: J per cube", "Difference in time", "Difference in energy"], 40)
for i, conf in enumerate([(15.0, 1.5), (20.0, 1.5)]):
    r = ar + 1 + i; put(wsA, r, 1, conf[0], f_in); put(wsA, r, 2, conf[1], f_in); put(wsA, r, 3, None)
    a = lambda col: "=AVERAGEIFS($%s$2:$%s$%d,$D$2:$D$%d,$A%d,$E$2:$E$%d,$B%d,$G$2:$G$%d,1)" % (col, col, NA, NA, r, NA, r, NA)
    put(wsA, r, 4, a("H"), fmt="0.00"); put(wsA, r, 5, a("K"), fmt="0.00"); put(wsA, r, 6, "=" + SC(conf, "Seconds per cube"), f_link, "0.00"); put(wsA, r, 7, "=" + SC(conf, "TOTAL J per cube"), f_link, "0.00")
    put(wsA, r, 8, "=D%d/F%d-1" % (r, r), fmt="+0.0%;-0.0%"); put(wsA, r, 9, "=E%d/G%d-1" % (r, r), fmt="+0.0%;-0.0%")
wsA.cell(row=ar + 4, column=1, value="Session exp_20261025_232408_vision_check_v1: 24 cycles run 22 hours after the grid, after the arm repair and gripper re-calibration. It shows how repeatable the grid numbers are.").font = f_norm
widths(wsA, [8, 8, 19, 12, 12, 12, 12, 12, 12, 10, 10])

# ============================================================ Pain points
wsX = wb.create_sheet("Pain points")
wsX["A1"] = "Pain points: what is wasted, what the fix costs, and whether the fix is worth it"; wsX["A1"].font = f_title
wsX["A2"] = "Every number is a formula from the measured sheets. Green = the fix pays back. Red = the fix costs more than it saves. Yellow = depends, or not measured."; wsX["A2"].font = f_norm
header(wsX, 4, ["#", "Pain point", "How big is the waste", "Waste", "Unit", "The fix", "What the fix costs", "Cost", "Unit", "Net result", "Net", "Unit", "Pays back after", "Verdict", "Evidence and limits"], 40)
slow, best, be15, fast2, a10 = (8.0, 1.0), (20.0, 1.5), (15.0, 1.5), (20.0, 2.0), (20.0, 1.0)
J = "TOTAL J per cube"; T = "Seconds per cube"
pp = [
 ("Arm is powered while it waits for the cube", "Arm energy spent holding still, per cube, at 8 rpm / arm 1.0", "=" + SC(slow, "Arm J while still (waiting)"), "J per cube",
  "Run the belt faster (20 rpm, arm 1.5) so the wait is shorter", "Extra belt energy per cube at 20 rpm", "=%s-%s" % (SC(best, "Belt J per cube"), SC(slow, "Belt J per cube")), "J per cube",
  "Total energy per cube: 20/1.5 minus 8/1.0 (negative = saved)", "=%s-%s" % (SC(best, J), SC(slow, J)), "J per cube", "Immediately", "WORTH IT (mostly for speed)",
  "Settings sheet. Also %s faster. Belt J per cube stays near 10-11 J at any rpm because the belt travel per cube is fixed; only the arm's waiting changes."),
 ("Running slowly 'to save energy'", "Extra energy per cube at 8 rpm / 1.0 compared with 20 / 1.5", "=%s-%s" % (SC(slow, J), SC(best, J)), "J per cube",
  "None needed: do not slow down", "Extra time per cube when slow", "=%s-%s" % (SC(slow, T), SC(best, T)), "s per cube",
  "Slow = more energy AND more time", "=%s/%s-1" % (SC(slow, J), SC(best, J)), "more energy", "Never", "NOT WORTH IT",
  "Settings sheet. Slower is not greener on this cell: the arm is powered the whole time it waits. The energy difference is small (close to the 95 % margins); the time difference is large and certain."),
 ("Pushing the arm to speed 2.0", "Time per cube at 20/2.0 minus 20/1.5, OK cycles only (positive = arm 2.0 was SLOWER)", "=%s-%s" % (SC(fast2, "Seconds per cube, OK cycles only"), SC(best, "Seconds per cube, OK cycles only")), "s per cube",
  "Arm speed 2.0 at 20 rpm", "Failures seen at 20/2.0 (out of 20)", "=%s-%s" % (SC(fast2, "All cycles"), SC(fast2, "OK cycles (of all)")), "failed cycles",
  "Energy per good cube: 20/2.0 minus 20/1.5", "=%s-%s" % (SC(fast2, "J per GOOD cube (failures charged to good cubes)"), SC(best, "J per GOOD cube (failures charged to good cubes)")), "J per cube", "Never (at this failure rate)", "NOT WORTH IT",
  "Settings + Failures sheets. At 8-15 rpm arm 2.0 gained 0.4-0.7 s per cube; at 20 rpm it gained nothing and 2 of 20 cycles failed. Only 20 cycles: the link between arm 2.0 and the failures is a hint, not proof."),
 ("Belt transport is the largest energy item of a good cycle", "Belt energy per cube at 20 / 1.5 (about the same at every rpm)", "=" + SC(best, "Belt J per cube"), "J per cube",
  "Shorten the belt distance from the feeder to the pick point", "Mechanical rebuild", None, "not measured",
  "Belt energy saved per 100 mm less travel (calculated)", "=%s/%s*100" % (SC(best, "Belt J per cube"), SC(best, "Belt travel per cube (mm)")), "J per cube", "Not measured", "NOT MEASURED",
  "Settings + Phases sheets. Calculated from belt J / belt travel; assumes belt energy is proportional to distance. It would also shorten the cycle. Never tested on the cell."),
 ("One failed transport (cube never reaches the IR)", "Energy burned by one failure (mean of the 5 seen)", "=Failures!$I$%d" % fr, "J per failure",
  "Shorter arrive timeout matched to belt speed (PRISM: 2.7 x belt ride)", "Risk: a slow cube is called a failure too early", None, "not measured",
  "One failure = this many good cubes of energy", "=Failures!$L$%d" % fr, "good cubes", "Each failure", "WORTH IT",
  "Failures sheet. The belt runs the whole 40 s timeout. Timeout 2.7 x ride is in optimize.py; its saving per failure has not been measured on the cell."),
 ("Feeder runs empty, no optimizer", "Energy of the empty cycles before autoen stops (3 cycles)", "=%s*%s" % (B["e_empty"], B["n_empty"]), "J per event",
  "PRISM: trusted camera sees no cube 5 s into transport and aborts", "Energy of the aborted feed with PRISM", "=" + B["e_empty_p"], "J per event",
  "Saved per empty-feeder event", "=%s*%s-%s" % (B["e_empty"], B["n_empty"], B["e_empty_p"]), "J per event", "The first event", "WORTH IT",
  "Baselines sheet (live 2026-10-26). One run each; without PRISM the cell also stops with an error and needs a person."),
 ("Belt driver left on while idle (not braked)", "Power of the belt driver doing nothing", "=" + B["pb_idle"], "W",
  "Brake the belt when idle", "Energy to brake", 0, "J",
  "Saved per idle hour", "=(%s-%s)*3600" % (B["pb_idle"], B["pb_brake"]), "J per hour", "Immediately", "WORTH IT",
  "Baselines sheet. Small (one cube's energy every ~7 idle minutes) but free."),
 ("Parking the arm in the REST pose when idle (torque on)", "Arm power holding home", "=" + B["pa_home"], "W",
  "Move home -> rest, then rest -> home when work returns", "Round trip energy", "=%s+%s" % (B["e_h2r"], B["e_r2h"]), "J",
  "Power SAVED while resting (negative = rest costs MORE)", "=%s-%s" % (B["pa_home"], B["pa_rest"]), "W", "Never", "NOT WORTH IT",
  "Baselines sheet. Rest draws about 3.5x home (gravity load on J3/J4/J5), plus the round trip and 73 s of not being available."),
 ("Same, if rest could be 0 W (torque off)", "Arm power holding home", "=" + B["pa_home"], "W",
  "Torque off in rest", "Round trip energy", "=%s+%s" % (B["e_h2r"], B["e_r2h"]), "J",
  "Idle time needed before it pays back", "=(%s+%s)/%s" % (B["e_h2r"], B["e_r2h"], B["pa_home"]), "s", "=TEXT((%s+%s)/%s/60,\"0.0\")&\" min of idle\"" % (B["e_h2r"], B["e_r2h"], B["pa_home"]), "ONLY FOR LONG IDLE",
  "Calculated best case: torque-off rest power was never measured (firmware did not answer). Real break-even is longer."),
 ("Probing an empty feeder to see if it was refilled", "Energy of one empty probe with PRISM", "=" + B["e_empty_p"], "J per probe",
  "Probe less often (back-off 30 s ... 30 min)", "Idle power while waiting instead", "=%s+%s" % (B["pa_idle_p"], B["pb_idle_p"]), "W",
  "One probe = this many seconds of idling", "=%s/(%s+%s)" % (B["e_empty_p"], B["pa_idle_p"], B["pb_idle_p"]), "s", "Probe gap longer than this", "DEPENDS",
  "Baselines sheet. Probing more often than this spends more on probes than on waiting; that is why the probe interval backs off."),
 ("Vision at a higher frame rate", "CPU at 6 fps (share of one core)", "=" + B["cpu6"], "of a core",
  "15 fps for smoother tracking", "CPU at 15 fps", "=" + B["cpu15"], "of a core",
  "Extra CPU load", "=%s/%s-1" % (B["cpu15"], B["cpu6"]), "more CPU", "Not measured in W", "NOT MEASURED",
  "CPU load only (old RealSense camera), about +15 C on the Pi. Its effect in watts was not measured."),
 ("Controller power (Pi, Mega, drivers) is not in any number here", "Average MOTOR power while producing at 20/1.5", "=" + SC(best, "Average power while producing (W)"), "W",
  "Put a power meter on the 5 V / 12 V inputs", "One sensor", None, "not measured",
  "A Raspberry Pi 4 typically draws a few watts, likely more than the motors", None, "not measured", "-", "NOT MEASURED",
  "Biggest gap in the data. If asked, say plainly: the figures are motor energy; total wall energy has not been measured."),
]
vfill = {"WORTH IT": fill_g, "WORTH IT (mostly for speed)": fill_g, "NOT WORTH IT": fill_r}
for i, p in enumerate(pp):
    r = 5 + i; vals = [i + 1] + list(p)
    if i == 0: vals[-1] = "=\"Settings sheet. Also \"&TEXT(1-%s/%s,\"0%%\")&\" less time per cube: that is the solid gain. The energy saving is small, about the size of the two 95 %% margins added together, so quote it as 'no worse, slightly better'.\"" % (SC(best, T), SC(slow, T))
    for k, v in enumerate(vals, 1):
        fmt = None
        if k in (4, 8, 11) and v is not None: fmt = "0.0%" if (vals[k] in ("more energy", "more CPU")) else ("0.000" if vals[k] == "W" else "0.00")
        x = put(wsX, r, k, v, f_bold if k in (2, 14) else f_norm, fmt, al=wrap)
    wsX.cell(row=r, column=14).fill = vfill.get(p[-2], fill_y); wsX.row_dimensions[r].height = 78
widths(wsX, [4, 30, 34, 9, 12, 34, 30, 9, 12, 34, 9, 12, 20, 16, 70]); wsX.freeze_panes = "C5"
PPR = {i + 1: 5 + i for i in range(len(pp))}

# ============================================================ Normal vs PRISM
wsN = wb.create_sheet("Normal vs PRISM")
header(wsN, 1, ["Result", "Normal mode (defaults 6.7 rpm / arm 1.0)", "Manual best (20 rpm / arm 1.5 by hand)", "PRISM (optimizer)", "PRISM vs Normal", "How each number was obtained"], 45)
rowsN = [
 ("Seconds per cube", "=%s+%s/%s+%s/%s" % (B["ma"], B["mb"], B["arm_def"], B["mc"], B["rpm_def"]), "=" + SC(best, T), "=60/%s" % B["cpm_prism"], "0.0",
  "Normal: ESTIMATE from the cycle-time model (6.7 rpm is below the tested range). Manual: measured grid. PRISM: measured live."),
 ("Cubes per minute", "=60/B2", "=60/C2", "=" + B["cpm_prism"], "0.00", "60 / seconds per cube"),
 ("J per cube", "=%s+%s*(B2-%s)" % (SC(slow, J), SC(slow, "Arm power while still (W)"), SC(slow, T)), "=" + SC(best, J), "=" + B["j_prism"], "0.0",
  "Normal: ESTIMATE = measured 8 rpm / 1.0 plus the arm's waiting power for the extra seconds. Manual: measured grid. PRISM: measured live (7 cubes)."),
 ("J per empty-feeder event", "=%s*%s" % (B["e_empty"], B["n_empty"]), "=%s*%s" % (B["e_empty"], B["n_empty"]), "=" + B["e_empty_p"], "0.0", "Measured live 2026-10-26; Normal = 3 empty cycles before autoen stops."),
 ("Idle power (W)", "=%s+%s" % (B["pb_idle"], B["pa_home"]), "=%s+%s" % (B["pb_idle"], B["pa_home"]), "=%s+%s" % (B["pa_idle_p"], B["pb_idle_p"]), "0.000",
  "Normal: sum of two separate measurements (belt not braked + arm home), not one run. PRISM: measured live."),
 ("Restarts by itself after a refill", "No", "No", "Yes", None, "Behaviour, not a measurement."),
]
for i, (nm, a, b, c, fm, how) in enumerate(rowsN):
    r = 2 + i; put(wsN, r, 1, nm, f_bold); put(wsN, r, 2, a, fmt=fm); put(wsN, r, 3, b, fmt=fm); put(wsN, r, 4, c, fmt=fm)
    put(wsN, r, 5, "=D%d/B%d-1" % (r, r) if fm else None, fmt="+0%;-0%"); put(wsN, r, 6, how, al=wrap); wsN.row_dimensions[r].height = 32
    if i in (0, 2): wsN.cell(row=r, column=2).fill = fill_y
wsN.cell(row=9, column=1, value="Yellow cells are ESTIMATES, not measurements. A side-by-side run (same cubes, same empty pattern, Normal then PRISM) would replace them.").font = f_norm
wsN.cell(row=10, column=1, value="While cubes flow, PRISM's gain is speed, not joules per cube (it equals the best manual setting). Its energy saving is in empty-feeder events and idle time.").font = f_norm
widths(wsN, [30, 22, 22, 18, 14, 90])

# ============================================================ Calculator
wsK = wb.create_sheet("Calculator")
wsK["A1"] = "Scale-up calculator: change the YELLOW cells, everything else recalculates"; wsK["A1"].font = f_title
inp = [("Cubes to sort per day", 1000, "cubes", "example value: change it"), ("Working hours per day", 8, "h", "example value: change it"),
       ("Empty-feeder events per day", 10, "events", "example value: change it"), ("Working days per year", 250, "days", "example value: change it"),
       ("Electricity price", 4.5, "per kWh", "EXAMPLE ONLY, in your currency: put your real tariff here"),
       ("Controller power (Pi + Mega + drivers)", 5, "W", "NOT MEASURED: placeholder, typical for a Raspberry Pi 4 system. Measure it or say so.")]
header(wsK, 3, ["Input", "Value", "Unit", "Note"], 25)
for i, (a, b, c, d) in enumerate(inp):
    r = 4 + i; put(wsK, r, 1, a, f_bold); put(wsK, r, 2, b, f_in, fill=fill_in); put(wsK, r, 3, c); put(wsK, r, 4, d)
header(wsK, 12, ["Result", "Normal mode (estimate)", "Manual best", "PRISM", "PRISM vs Normal", "Formula in words"], 30)
N = "'Normal vs PRISM'!"
kr = [
 ("Seconds per cube", "=%s{c}2" % N, "0.0", "from Normal vs PRISM"),
 ("J per cube", "=%s{c}4" % N, "0.0", "from Normal vs PRISM"),
 ("J per empty-feeder event", "=%s{c}5" % N, "0.0", "from Normal vs PRISM"),
 ("Idle power (W)", "=%s{c}6" % N, "0.000", "from Normal vs PRISM"),
 ("Production time needed (h)", "=$B$4*{c}13/3600", "0.00", "cubes x seconds per cube"),
 ("Fits in the working day?", "=IF({c}17<=$B$5,\"Yes\",\"NO: needs \"&TEXT({c}17,\"0.0\")&\" h\")", None, "production time vs working hours"),
 ("Idle time (h)", "=MAX(0,$B$5-{c}17)", "0.00", "working hours minus production time"),
 ("Energy for the cubes (J)", "=$B$4*{c}14", "#,##0", "cubes x J per cube"),
 ("Energy for empty-feeder events (J)", "=$B$6*{c}15", "#,##0", "events x J per event"),
 ("Energy while idle (J)", "={c}19*3600*{c}16", "#,##0", "idle seconds x idle power"),
 ("MOTOR energy per day (J)", "={c}20+{c}21+{c}22", "#,##0", "sum of the three rows above"),
 ("MOTOR energy per day (Wh)", "={c}23/3600", "0.00", "J / 3600"),
 ("Share of motor energy that is NOT cubes (waste)", "=({c}21+{c}22)/{c}23", "0.0%", "(empty events + idle) / total"),
 ("Motor energy per cube, all-in (J)", "={c}23/$B$4", "0.0", "daily motor energy / cubes"),
 ("MOTOR energy per year (kWh)", "={c}24*$B$7/1000", "0.00", "Wh per day x days / 1000"),
 ("Motor electricity cost per year", "={c}27*$B$8", "0.00", "kWh x price"),
 ("Controller energy per day (Wh), not measured", "=$B$9*$B$5", "0.0", "controller W x working hours (same for all three)"),
 ("Motors' share of total energy", "={c}24/({c}24+{c}29)", "0.0%", "motor Wh / (motor Wh + controller Wh)"),
 ("Total energy per year incl. controller (kWh)", "=({c}24+{c}29)*$B$7/1000", "0.00", "(motor + controller) x days / 1000"),
]
for i, (nm, f, fm, how) in enumerate(kr):
    r = 13 + i; put(wsK, r, 1, nm, f_bold)
    for c in "BCD": put(wsK, r, " BCD".index(c) + 1, f.format(c=c), f_link if i < 4 else f_norm, fm)
    put(wsK, r, 5, None if fm is None or i == 16 else "=IF(B%d=0,0,D%d/B%d-1)" % (r, r, r), fmt="+0%;-0%;0%"); put(wsK, r, 6, how)
assert wsK["A17"].value.startswith("Production time") and wsK["A23"].value.startswith("MOTOR energy per day (J)") and wsK["A29"].value.startswith("Controller")
for i, t in enumerate(["Read this before quoting the money rows: motor energy on this cell is tiny (about 1 W while producing), so the yearly cost is very small in any currency.",
                       "The honest headline is therefore relative: joules per cube, share of energy wasted, and cubes per hour. Those scale to a real factory; the absolute Wh do not.",
                       "Normal mode cannot restart by itself after an empty feeder; this calculator assumes someone restarts it at once, which flatters Normal mode."]):
    wsK.cell(row=33 + i, column=1, value=t).font = f_norm
widths(wsK, [46, 22, 16, 14, 16, 60])

# ============================================================ Q&A
wsQ = wb.create_sheet("Q&A")
wsQ["A1"] = "Questions a committee may ask, with the number and where it comes from"; wsQ["A1"].font = f_title
header(wsQ, 3, ["#", "Question", "Short answer", "Number", "Unit", "How it is calculated", "Sheet"], 30)
ALLR = PR[(best, "ALL")]; TRR = PR[(best, "TRANSPORT")]
qa = [
 ("How much energy does it take to sort one cube?", "About 21 J at the best setting, motors only.", "=" + SC(best, J), "J per cube", "Mean of 18 cycles at 20 rpm / arm 1.5: arm J + belt J, each = sum of V x I x dt at 10 Hz.", "Settings"),
 ("How sure are you of that number?", "The 95 % margin on the mean is below 1 J; a repeat run 22 h later was about 5 % higher.", "=" + SC(best, "95 % margin on J per cube (+/-)"), "J (+/-)", "1.96 x sd / sqrt(n). Repeat-run difference is on the Check run sheet.", "Settings, Check run"),
 ("How fast is it?", "About 3.1 cubes per minute at the best setting.", "=" + SC(best, "Cubes per minute"), "cubes/min", "60 / mean cycle time.", "Settings"),
 ("Where does the energy of one cube go?", "A little over half to the belt, the rest to the arm (the elbow joint is the arm's biggest consumer).", "=%s/%s" % (SC(best, "Belt J per cube"), SC(best, J)), "belt share", "Belt J / total J at 20 / 1.5.", "Settings, Phases"),
 ("What is the biggest pain point?", "Energy spent on nothing. One empty-feeder event without PRISM burns as much as several good cubes; one failed transport burns about two.", "=%s*%s/%s" % (B["e_empty"], B["n_empty"], SC(best, J)), "good cubes per empty event", "3 empty cycles x 49.5 J / J per good cube at 20 / 1.5.", "Pain points #6, #5"),
 ("Which part of a good cycle uses the most energy?", "The transport phase: the belt carries the cube while the arm waits.", "=Phases!$K$%d" % TRR, "of cycle energy", "TRANSPORT J (belt + arm) / cycle J at 20 / 1.5. It is also the longest phase.", "Phases"),
 ("How much arm energy is spent just waiting?", "About a quarter of the arm's energy per cube is spent holding still.", "=%s/%s" % (SC(best, "Arm J while still (waiting)"), SC(best, "Arm J per cube")), "of arm energy", "Arm J while no joint is moving / arm J.", "Settings"),
 ("Does running slower save energy?", "No. At 8 rpm a cube costs slightly more energy and takes much longer.", "=%s/%s-1" % (SC(slow, J), SC(best, J)), "more energy when slow", "J per cube at 8/1.0 vs 20/1.5.", "Pain points #2"),
 ("Does a faster belt cost more energy per cube?", "Only a little. Belt power rises with rpm but the belt runs for a shorter time, because the travel per cube is fixed; the arm's waiting energy falls by more.", "=%s/%s-1" % (SC(best, "Belt J per cube"), SC(slow, "Belt J per cube")), "belt J change 8 -> 20 rpm", "E = P(rpm) x travel / speed; P and speed both scale with rpm, so they nearly cancel.", "Settings"),
 ("Why not run the arm at full speed?", "At 20 rpm, arm 2.0 was not faster and 2 of 20 cycles failed, so each good cube cost more.", "=%s-%s" % (SC(fast2, "J per GOOD cube (failures charged to good cubes)"), SC(best, "J per GOOD cube (failures charged to good cubes)")), "J per good cube extra", "All energy incl. failed cycles / good cubes. Small sample: 20 cycles.", "Pain points #3"),
 ("What does one failure cost?", "About as much energy as two good cubes, and 42 s.", "=Failures!$L$%d" % fr, "good cubes of energy", "Mean J of the 5 failed cycles / mean J of an OK cycle at the same setting.", "Failures"),
 ("Why not park the arm to save energy when idle?", "Parking costs more than holding home: the rest pose draws more power and the move itself costs energy.", "=%s/%s" % (B["pa_rest"], B["pa_home"]), "x home power", "P_rest / P_home; plus 20.5 J and 73 s for the round trip.", "Pain points #8"),
 ("When would parking pay back?", "Only if rest drew 0 W, and then only after about 4 minutes of idle.", "=(%s+%s)/%s/60" % (B["e_h2r"], B["e_r2h"], B["pa_home"]), "min", "Round-trip J / home W. Best case; torque-off was not measured.", "Pain points #9"),
 ("What does PRISM save when the feeder runs empty?", "About 93 % of the energy of an empty-feeder event.", "=1-%s/(%s*%s)" % (B["e_empty_p"], B["e_empty"], B["n_empty"]), "saved", "1 - 11.0 J / (3 x 49.5 J).", "Pain points #6"),
 ("Does PRISM use less energy per cube than a good fixed setting?", "No. While cubes flow it matches the best manual setting; it is slightly higher in the one live run.", "=%s-%s" % (B["j_prism"], SC(best, J)), "J per cube difference", "Live 22.0 J (7 cubes) vs grid 20.8 J (18 cycles). The Check run at the same fixed setting gave 21.7 J, so this is run-to-run variation.", "Normal vs PRISM"),
 ("How much faster is PRISM than the defaults?", "About 1.9x the cubes per minute. The default figure is an estimate.", "='Normal vs PRISM'!$D$3/'Normal vs PRISM'!$B$3", "x", "3.24 measured / estimate from the cycle-time model at 6.7 rpm.", "Normal vs PRISM"),
 ("How good is the cycle-time model?", "Its error on the measured settings is within about 2 s.", "=MAX(MAX(Settings!$AC$2:$AC$%d),-MIN(Settings!$AC$2:$AC$%d))" % (len(confs) + 1, len(confs) + 1), "s worst error", "Model s minus measured s, worst of 12 settings. R2 0.885 on 264 cycles.", "Settings"),
 ("What is NOT in these numbers?", "The Pi, the Mega and the motor drivers. Only motor supply energy is measured.", None, "-", "No sensor on the controller supply. Arm power is from servo current registers (coarse, slight underestimate).", "Read me"),
 ("Is the whole system's energy dominated by the motors?", "Probably not: the motors average about 1 W while producing.", "=" + SC(best, "Average power while producing (W)"), "W", "Total J per cube / seconds per cube. Controller power is not measured.", "Calculator"),
]
for i, q in enumerate(qa):
    r = 4 + i; put(wsQ, r, 1, i + 1)
    for k, v in enumerate(q, 2):
        fm = None
        if k == 4 and v: fm = "0%" if q[3] in ("belt share", "of cycle energy", "of arm energy", "more energy when slow", "belt J change 8 -> 20 rpm", "saved") else "0.00"
        put(wsQ, r, k, v, f_bold if k == 2 else f_norm, fm, al=wrap)
    wsQ.row_dimensions[r].height = 44
widths(wsQ, [4, 44, 60, 9, 22, 62, 22]); wsQ.freeze_panes = "A4"

# ============================================================ Read me (first sheet)
wsR = wb.create_sheet("Read me", 0)
wsR["A1"] = "PRISM: speed and energy data, and the pain points"; wsR["A1"].font = f_title
lines = [
 ("What this file is", None),
 ("", "Every production cycle of the grid experiment (240 cubes, 12 settings), broken down by phase and by joint, with the calculations a committee may ask for."),
 ("", "Source recording: orchestrator/cal/runs/exp_20261025_005038_opt_grid_v1.jsonl (2026-10-25, 00:50-07:20). Nothing was re-run for this file."),
 ("Sheets", None),
 ("Pain points", "START HERE. Each waste, the fix, what the fix costs, and the verdict."),
 ("Q&A", "Likely committee questions with the number and how it is calculated."),
 ("Settings", "One row per setting (belt rpm x arm speed): speed, energy, split, waste, margins."),
 ("Phases", "Each setting split into the 8 phases of a cycle: seconds, arm J, belt J, watts."),
 ("Joints", "Arm energy per joint for each setting."),
 ("Failures", "The 5 failed cycles and what each one cost."),
 ("Normal vs PRISM", "Defaults vs best manual setting vs the optimizer. Estimates are marked yellow."),
 ("Calculator", "Type cubes per day, hours, empty events: gives energy per day and year, and the waste share."),
 ("Check run", "24 cycles recorded 22 h later, to show the numbers repeat."),
 ("Cycles", "The raw table: one row per cube, %d columns. Everything else is calculated from it." % len(cols)),
 ("Baselines", "Every measured constant used (belt power, arm hold power, empty-feeder runs) with its source."),
 ("How energy is measured", None),
 ("Belt", "INA260 sensor on the belt motor supply: power = voltage x current, sampled at 10 Hz."),
 ("Arm", "Sum over the 6 servos of (bus voltage x current register x 6.5 mA). There is no power sensor on the arm supply."),
 ("Energy", "Power x time between samples, added up. A gap longer than 0.5 s is not counted."),
 ("Cycle", "From the end of the previous cycle to the end of this one. The first cycle of each block is recorded but not counted in the means (start-up)."),
 ("Moving / still", "The arm is 'moving' when any joint is faster than 30 ticks/s; otherwise 'still'."),
 ("Limits to state openly", None),
 ("1", "Motor energy only. The Raspberry Pi, the Mega and the motor drivers are not measured and are probably larger than the motors."),
 ("2", "Arm power from the servo current register is coarse (6.5 mA steps) and misses servo electronics: arm joules are a slight underestimate."),
 ("3", "20 cycles per setting (18 counted). Differences of about 1 J between settings are inside the margin; see the 95 % margin column."),
 ("4", "The grid was recorded before the arm repair and gripper re-calibration; the Check run after the repair agrees within a few percent."),
 ("5", "Normal-mode speed and energy are estimates (the defaults are below the tested range). PRISM live numbers come from single short runs."),
 ("6", "Nothing above 20 rpm and no torque-off rest was measured."),
 ("Colours", None),
 ("Blue text", "A measured value or a setting typed in (not a formula)."),
 ("Black text", "A formula. Green text = a link to another sheet."),
 ("Yellow fill", "An input you may change (Calculator), a headline column, or an estimate."),
]
r = 3
for a, b in lines:
    if b is None:
        c = wsR.cell(row=r, column=1, value=a); c.font = f_bold; c.fill = fill_s; wsR.cell(row=r, column=2).fill = fill_s
    else:
        wsR.cell(row=r, column=1, value=a).font = f_bold; c = wsR.cell(row=r, column=2, value=b); c.font = f_norm; c.alignment = wrap
    r += 1
wsR.column_dimensions["A"].width = 24; wsR.column_dimensions["B"].width = 130

order = ["Read me", "Pain points", "Q&A", "Settings", "Phases", "Joints", "Failures", "Normal vs PRISM", "Calculator", "Check run", "Cycles", "Baselines"]
wb._sheets = [wb[n] for n in order]
wb.save(OUT)
print("saved", OUT)
