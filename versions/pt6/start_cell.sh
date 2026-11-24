#!/usr/bin/env bash
# start_cell.sh - start the PRISM cell in the right order:
#   health + loggers + analyzers + dashboard -> vision -> autoen -> optimizer -> Start.
#
#   ./start_cell.sh              optimizer with --auto-resume (probes the feeder by itself when idle)
#   ./start_cell.sh --no-auto    optimizer waits for:  mosquitto_pub -t orchestrator/cmd -m resume
#   ./start_cell.sh --shadow     optimizer only logs what it would do (sends nothing to autoen)
#   ./start_cell.sh --no-extras  skip step 0 (health, loggers, analyzers, dashboard)
#   ./start_cell.sh --serial     the old cycle: feed, wait for the cube, pick, place, THEN feed the next one
#   ./start_cell.sh --pipeline   autoen's fixed pipeline (feature B): feeds while the arm lifts (weak push on 11-11)
#   ./start_cell.sh --queue      pipeline + belt queue (feature C; not tried with the optimizer)
# Default (2026-11-12): FLOW (autoen feature F): the OPTIMIZER plans every feed - how many cubes ride the belt and at
# which moment the feeder pushes (learned: while the arm lifts, the shared supply makes the push weak). The belt
# brings cubes while the arm works; a pick never waits for a cycle to end.
#   ./stop_cell.sh               stop the run, optimizer and autoen  (--all: everything, step 0 and vision too)
#
# Programs that are already running are kept (not started twice). Their output goes to PRISM/run_logs/.
# Nothing moves until you press Enter at the last question (Start = gripper opens, arm drives Home).
set -u
PRISM="$HOME/Desktop/PRISM"
LOGS="$PRISM/run_logs"
API="http://127.0.0.1:8781"
STAMP=$(date +%Y%m%d_%H%M%S)
OPT_ARGS="--auto-resume"
EXTRAS=1
ENABLE_B=0
ENABLE_C=0
ENABLE_F=1
for a in "$@"; do
  case "$a" in
    --no-auto)   OPT_ARGS="" ;;
    --shadow)    OPT_ARGS="--shadow" ;;
    --no-extras) EXTRAS="" ;;
    --serial)    ENABLE_F=0; ENABLE_B=0; ENABLE_C=0 ;;
    --pipeline)  ENABLE_F=0; ENABLE_B=1; ENABLE_C=0 ;;
    --queue)     ENABLE_F=0; ENABLE_B=1; ENABLE_C=1 ;;
    *) echo "unknown option: $a  (use --no-auto, --shadow, --no-extras, --serial, --pipeline or --queue)"; exit 2 ;;
  esac
done
mkdir -p "$LOGS"

# A cell of ANOTHER folder (PRISM_mek, PRISM_dupi ...) still running would be reused ("already running") and answer with its old
# code and state - the dashboard then shows "autoen undefined" and a paused run (2026-11-13). Refuse, and say how to stop it.
for pat in "auto_en_new.py" "dashboard_node.py" "main.py"; do
  for pid in $(pgrep -f "$pat"); do
    cwd=$(readlink "/proc/$pid/cwd" 2>/dev/null)
    case "$cwd" in
      "$PRISM"|"$PRISM"/*|"") ;;
      "$HOME"/Desktop/PRISM*|"$HOME"/Desktop/PRISM_*)
        echo "another PRISM folder is still running ($pat in $cwd)."
        echo "stop it first:   cd ${cwd%/*} && ./stop_cell.sh --all      then run ./start_cell.sh here again"; exit 1 ;;
    esac
  done
done

running() { pgrep -f "$1" >/dev/null; }
start_bg() {    # start_bg <name> <pgrep pattern> <folder> <env> <log name> <command ...>
  local name="$1" pat="$2" dir="$3" env="$4" log="$5"
  shift 5
  if running "$pat"; then
    printf '%-12s already running\n' "$name:"
  else
    (cd "$PRISM/$dir" && nohup "$HOME/$env/bin/python" "$@" > "$LOGS/${log}_$STAMP.log" 2>&1 &)
    printf '%-12s started   (log run_logs/%s_%s.log)\n' "$name:" "$log" "$STAMP"
  fi
}

# ---- 0. the closed-loop helpers. All PASSIVE: they listen to MQTT and record / report, they never move anything.
#         Started first so they see the whole run from its first message.
if [ -n "$EXTRAS" ]; then
  if ! systemctl is-active --quiet mosquitto; then
    echo "the MQTT broker (mosquitto) is not running:  sudo systemctl start mosquitto"
    exit 1
  fi
  # the patterns also catch a copy you started by hand: two with the same MQTT client id would kick each other off
  start_bg "health"     "health\.py"                      health    health-env health     "$PRISM/health/health.py"
  start_bg "log arm"    "logger\.py.*--topics arm( |$)"    logger    logger-env logger_arm "$PRISM/logger/logger.py" \
           --topics arm --db arm.db
  start_bg "log camera" "logger\.py.*--topics camera( |$)" logger    logger-env logger_cam "$PRISM/logger/logger.py" \
           --topics camera --db camera.db
  start_bg "arm ana"    "arms_ana\.py"                    analysis  ana-env    arm_ana    "$PRISM/analysis/arms_ana.py"
  start_bg "belt ana"   "conv_ana\.py"                    analysis  ana-env    belt_ana   "$PRISM/analysis/conv_ana.py"
  start_bg "dashboard"  "dashboard_node\.py"              dashboard db-env     dashboard  "$PRISM/dashboard/dashboard_node.py" \
           --data-dir data --log-dir "$PRISM/autoencoder/riff_logs"
  echo "             dashboard page: http://127.0.0.1:8080   (autoen's own page is :8781)"
  if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v xdg-open >/dev/null; then
    # open the Overview in the browser, so READY can be watched while the rest starts (the page waits for the server)
    (sleep 3; xdg-open "http://127.0.0.1:8080/#/overview" >/dev/null 2>&1 &)
  fi
fi
opt_pids() {     # every main.py running in PRISM/orchestrator, however it was started
  local pid
  for pid in $(pgrep -f "main.py"); do
    [ "$(readlink "/proc/$pid/cwd" 2>/dev/null)" = "$HOME/Desktop/PRISM/orchestrator" ] && echo "$pid"
  done
}
autoen_status() {
  curl -s -m 2 "$API/api/state" | python3 -c 'import json,sys; print(json.load(sys.stdin)["status"])' 2>/dev/null
}
autoen_field() {
  curl -s -m 2 "$API/api/state" | python3 -c "import json,sys; print(json.load(sys.stdin).get('$1'))" 2>/dev/null
}
autoen_task_done() {   # 0 when the last task is done; 1 while it runs; 2 when it failed
  curl -s -m 2 "$API/api/state" | python3 -c 'import json,sys
t = json.load(sys.stdin).get("task") or {}
sys.exit(0 if t.get("status") == "done" else 2 if t.get("status") == "error" else 1)' 2>/dev/null
}

# ---- 1. vision (camera): white + orange cubes. vision_usb.py takes the Astro HD Camera (calibrated 2026-11-12,
#         vision_calibration_usb.json); without it the H264 USB camera, then any USB camera. The RealSense version
#         (vision_live_tracks.py) is kept for when the D415 is repaired. Either one already running counts.
if running "vision_live_tracks\.py|vision_usb\.py"; then
  echo "vision:    already running"
else
  (cd "$PRISM/vision" && nohup "$HOME/prism-env/bin/python" -u vision_usb.py --colors white,orange \
     > "$LOGS/vision_$STAMP.log" 2>&1 &)
  echo "vision:    started   (Astro HD camera, log run_logs/vision_$STAMP.log)"
fi

# ---- power relay (2026-11-12): PRISM may have been stopped while asleep (power off) - power ON before autoen.
#      Does nothing until the relay was taught once: orchestrator/cal/relay_test.py
RELAY_OUT=$("$HOME/orch-env/bin/python" "$PRISM/orchestrator/relay.py" on 2>&1)
echo "relay:     $RELAY_OUT"
case "$RELAY_OUT" in *"power on"*) sleep 3 ;; esac       # the servos and the Mega's 12 V side boot

# ---- 2. autoen (owns the Mega); wait until its web API answers
AUTOEN_NEW=""
if running "auto_en_new.py"; then
  echo "autoen:    already running"
else
  AUTOEN_NEW=1
  (cd "$PRISM/autoencoder" && nohup "$HOME/autoen-env/bin/python" auto_en_new.py > "$LOGS/autoen_$STAMP.log" 2>&1 &)
  echo -n "autoen:    starting "
fi
st=""
for _ in $(seq 1 60); do
  st=$(autoen_status)
  [ -n "$st" ] && break
  echo -n "."
  sleep 1
done
echo
if [ -z "$st" ]; then
  echo "autoen did not answer on $API - look at run_logs/autoen_$STAMP.log (the dashboard http://127.0.0.1:8080 shows what is missing)"
  exit 1
fi
echo "autoen:    $st"

# ---- 3. optimizer: must be running BEFORE Start, or autoen runs on its own like the old automation
if [ -n "$AUTOEN_NEW" ] && [ -n "$(opt_pids)" ]; then
  # a new autoen = a new run: an optimizer left over from the last one would keep its old code and old state
  # (2026-11-10). Stopping the optimizer moves nothing (it only stops deciding).
  echo "optimizer: one is left from the last run - replacing it with a fresh one"
  for pid in $(opt_pids); do kill "$pid" 2>/dev/null; done
  for _ in $(seq 1 10); do [ -z "$(opt_pids)" ] && break; sleep 0.5; done
fi
if [ -n "$(opt_pids)" ]; then
  echo "optimizer: already running ($(tr '\0' ' ' < "/proc/$(opt_pids | head -1)/cmdline"))"
else
  (cd "$PRISM/orchestrator" && nohup "$HOME/orch-env/bin/python" main.py $OPT_ARGS > "$LOGS/optimizer_$STAMP.log" 2>&1 &)
  echo -n "optimizer: starting ($OPT_ARGS) "
fi
ok=""
for _ in $(seq 1 20); do
  last=$(ls -t "$PRISM"/orchestrator/logs/orch_*.jsonl 2>/dev/null | head -1)
  if [ -n "$last" ] && grep '"kind": "plan"' "$last" | tail -1 | grep -q '"reason": "autoen' \
     && ! grep '"kind": "plan"' "$last" | tail -1 | grep -q 'not reachable'; then
    ok=1
    break
  fi
  echo -n "."
  sleep 1
done
echo
if [ -z "$ok" ]; then
  echo "the optimizer does not see autoen yet - look at run_logs/optimizer_$STAMP.log. Not pressing Start. (the dashboard http://127.0.0.1:8080 shows what is missing)"
  exit 1
fi
echo "optimizer: connected to autoen"

# ---- 3b. everything connected? The dashboard checks every program and every piece of hardware (the same list as
#          its Overview page). Telemetry needs a few seconds after autoen starts: wait up to 20 s for READY.
DASH="http://127.0.0.1:8080"
if curl -s -m 2 "$DASH/api/ready" >/dev/null; then
  echo -n "checking:  every program and every piece of hardware "
  for _ in $(seq 1 20); do
    report=$(curl -s -m 3 "$DASH/api/ready")
    [ "$(echo "$report" | tail -1)" = "READY" ] && break
    echo -n "."
    sleep 1
  done
  echo
  echo "$report" | sed '$d'
  READY=$(echo "$report" | tail -1)
  echo "           ===== $READY =====   (live on $DASH)"
else
  READY="unknown"
  echo "checking:  the dashboard is not running (--no-extras): no connection check"
fi

# ---- 4. Start (the only step that moves the robot). autoen remembers whether the arm rests on the cradle
#         (autoencoder/rest_state.json): a resting arm is woken along the taught path, never driven straight Home.
if [ "$st" != "ready" ]; then
  echo
  echo "autoen is '$st', not 'ready': it is already started. Nothing more to do."
  exit 0
fi
rest=$(autoen_field rest_state)
echo
if [ "$rest" = "awake" ]; then
  read -r -p "autoen remembers the arm is NOT on the cradle (awake). Correct? [Y/n] " ans
  if [[ "$ans" =~ ^[Nn] ]]; then
    echo "Then: ./stop_cell.sh, and start autoen by hand with --resting (it will wake the arm safely). Not started."
    exit 1
  fi
else
  echo "autoen remembers the arm is RESTING on the cradle ($rest): Start moves nothing, then the arm is woken"
  echo "along the taught path (~36 s) and the run begins."
fi
echo "Belt EMPTY, cubes in the feeder, arm area clear, E-stop within reach."
if [ "$ENABLE_F" = 1 ]; then echo "Cycle: FLOW - the optimizer plans every feed   (--serial: one at a time)"
elif [ "$ENABLE_C" = 1 ]; then echo "Cycle: pipeline + belt queue (2 cubes on the belt)"
elif [ "$ENABLE_B" = 1 ]; then echo "Cycle: pipeline (the next cube is fed while the arm works)   (--serial: one at a time)"
else echo "Cycle: serial (one cube at a time)"; fi
if [ "$READY" = "NOT READY" ]; then
  echo "NOT READY: something above is FAIL. Fix it first (the dashboard shows when it turns READY)."
  read -r -p "Enter = Start anyway, q = quit: " ans
else
  read -r -p "Enter = Start, q = quit: " ans
fi
if [[ "$ans" =~ ^[Qq] ]]; then
  echo "Not started. Everything else keeps running (./stop_cell.sh to stop it)."
  exit 0
fi
curl -s -m 5 -X POST "$API/api/start" -H 'Content-Type: application/json' -d '{"clear": true, "config": {
  "rpm": 20, "arm_speed": 1.5, "carry_speed": 1.5, "arrive_timeout": 25,
  "sets": 100, "cycles_per_set": 100, "pause_before_set": 0, "enable_f": '"$ENABLE_F"', "enable_b": '"$ENABLE_B"', "enable_c": '"$ENABLE_C"'}}'
echo
if [ "$rest" != "awake" ]; then
  for _ in $(seq 1 30); do [ "$(autoen_status)" = "paused" ] && break; sleep 1; done
  echo -n "waking the arm "
  curl -s -m 5 -X POST "$API/api/task" -H 'Content-Type: application/json' -d '{"kind": "wake", "seconds": 5, "settle_s": 0}' >/dev/null
  rc=1
  for _ in $(seq 1 120); do autoen_task_done; rc=$?; [ "$rc" != 1 ] && break; echo -n "."; sleep 1; done
  echo
  if [ "$rc" != 0 ] || [ "$(autoen_field rest_state)" != "awake" ]; then
    echo "wake did not finish (look at the autoen page). The run stays paused."
    exit 1
  fi
  curl -s -m 5 -X POST "$API/api/pause" >/dev/null       # resume: Home, then the cycles
  echo "awake: the run continues"
fi
echo "Started. Watch the optimizer:  tail -f $LOGS/optimizer_$STAMP.log"
echo "Stop everything:               ./stop_cell.sh"
