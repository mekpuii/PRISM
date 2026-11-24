#!/usr/bin/env python3
"""
rest_teach.py -- teach the rest move (home -> cradle) as waypoints with the keyboard.

  cd ~/Desktop/PRISM/autoencoder && python3.11 rest_teach.py        (autoen must NOT be running: it owns the port)

Start with the arm at HOME (press h to go there first). Jog the arm step by step toward the cradle; at every
point the arm must pass through, press Enter. Each waypoint stores only the joints that moved since the one before,
so autoen repeats it exactly: the same joints, in the same order. The last waypoint is the rest pose.

DEFAULT = arm_node.py's keys on the Pi's own keyboard: hold A S D F G H (J1..J6) + Z / X to move a joint (as long as
you hold it), 5-8 or = / - speed (starts at 50%), Enter = remember the pose, Backspace undo, W / R wake / rest test,
L list, K save (Y = yes), 3 home, Space or Q stop, Esc quit. --terminal-keys = the keys below (e.g. over ssh).

Or teach BY HAND: hold the arm, press f and every joint goes limp (torque off). Move it with your hands and
press Enter at each pose: Enter, move, Enter, move ... The arm stays free; f locks it. (needs the Q firmware)

  f           free the arm for hand moving / lock it again (HOLD THE ARM: it drops when torque goes off)
  1..6        pick the joint (1 base, 2 shoulder, 3 elbow, 4 wrist flex, 5 wrist roll, 6 gripper)
  = / -       selected joint +10 / -10 ticks (~0.9 deg)        + / _   +50 / -50 ticks
  Enter       remember the current pose as the next waypoint
  u           undo: move back to the previous waypoint and forget the last one
  l           list the waypoints
  w           wake test: play the waypoints in REVERSE back to the start (the arm ends where you started)
  r           rest test: play the waypoints forward from the start
  h           go home (only allowed while no waypoint is recorded: from a folded arm it could hit the frame)
  space       stop: every joint holds where it is
  s           save to PRISM/config/poses.json (asks y/n; the old file is backed up first)
  q           quit, nothing saved

Every move is slow (TEACH_SPEED) and blocked-guarded: a joint that stays more than BLOCK_TICKS short of its
target for BLOCK_S is pulled back to where it stopped, so it never keeps pushing against the frame.
Saved: rest = the last waypoint (J1-6); rest_sequence = the waypoints; wake_sequence = the exact reverse.
Other keys in poses.json (rest_speed, torque_off_joints, ...) are kept. Restart autoen to use it.
"""
import argparse
import json
import os
import shutil
import sys
import time
import urllib.request
from datetime import datetime

from auto_en_new import DEFAULT_POSES, POSE_CONFIG, Link, find_serial_port, load_pose_config
from grip_calibrate import Keys

try:                                                        # arm_node.py's keyboard: real key-down / key-up (evdev)
    import arm_node
except ImportError:                                         # no evdev: only the terminal keys below
    arm_node = None

# arm_node-style keys (physical Pi keyboard): hold A S D F G H + Z / X to jog, like arm_node.py.
# The other keys are turned into this script's terminal keys, so both modes share one loop.
EV_KEYS = {"KEY_ENTER": "\n", "KEY_KPENTER": "\n", "KEY_SPACE": " ", "KEY_Q": " ", "KEY_ESC": "q",
           "KEY_BACKSPACE": "u", "KEY_W": "w", "KEY_R": "r", "KEY_L": "l", "KEY_K": "s", "KEY_Y": "y", "KEY_3": "h",
           "KEY_5": "spd:0.25", "KEY_6": "spd:0.50", "KEY_7": "spd:0.75", "KEY_8": "spd:1.00",
           "KEY_EQUAL": "spd:+", "KEY_MINUS": "spd:-"}
JOG_SPEED = 0.5                 # start at 50% of arm_node's jog rate (teaching close to the frame)


class EvKeys:
    """arm_node.Keyboard behind the Keys interface: get() -> mapped key-down edges; held() for the jog."""
    def __init__(self, term):
        self.kb, self.term, self.queue = arm_node.Keyboard(), term, []

    def held(self):
        held, edges = self.kb.poll()
        self.queue += [EV_KEYS[e] for e in edges if e in EV_KEYS]
        return held

    def get(self, timeout=0.2):
        end = time.monotonic() + timeout
        while True:
            while self.term.get(0) is not None:             # the terminal sees the same keys: swallow them
                pass
            self.held()
            if self.queue:
                return self.queue.pop(0)
            if time.monotonic() >= end:
                return None
            time.sleep(0.01)

SAFE_MIN = {1: 784, 2: 868, 3: 996, 4: 813, 5: 298, 6: 1768}      # = the firmware's SAFE_MIN / SAFE_MAX
SAFE_MAX = {1: 3404, 2: 3151, 3: 3056, 4: 3201, 5: 3859, 6: 3116}
TEACH_SPEED = 120               # ticks/s for jogs and test moves
MOVED_TICKS = 15                # a joint that moved more than this since the last waypoint belongs to the new step
ARRIVE_TICKS = 15
BLOCK_TICKS = 25                # stuck this far short of the target ...
BLOCK_S = 0.8                   # ... for this long = blocked
MOVE_TIMEOUT_S = 25.0
HOME = {int(k): v for k, v in DEFAULT_POSES["right"].items() if k != "6"}
JOINT = {1: "base", 2: "shoulder", 3: "elbow", 4: "wrist flex", 5: "wrist roll", 6: "gripper"}


class Arm:
    def __init__(self, link, keys):
        self.link, self.keys = link, keys
        self.target = {}
        self.pending = []                                   # keys pressed while a move was running, in order
        self.free = False                                   # True = torque off (hand-moving mode)

    def torque(self, on):
        """Q,<id>,1|0 to every joint, wait for the Mega's replies (as autoen's _torque). -> joints that did NOT confirm."""
        joints = list(range(1, 7))
        with self.link.slock:
            for s in joints:
                self.link.torque.pop(s, None)
        missing = joints
        for _ in range(3):                                  # the link drops a few % of lines: ask again
            for s in missing:
                self.link.write(f"Q,{s},{1 if on else 0}")
            end = time.monotonic() + 0.6
            while time.monotonic() < end:
                with self.link.slock:
                    missing = [s for s in joints if self.link.torque.get(s, (None,))[0] != (1 if on else 0)]
                if not missing:
                    break
                time.sleep(0.02)
            if not missing:
                break
        if on and not missing:
            self.free = False
            self.target = {}
        return missing

    def lock(self):
        """Torque back on where the arm IS (the firmware sets goal = present first: no jump)."""
        if not self.free:
            return True
        missing = self.torque(True)
        if missing:
            self.hold()                                     # H also turns a torque-off joint on where it is
            print(f"    lock: no Q reply from {missing}, sent H instead")
        return True

    def key(self, timeout=0.2):
        return self.pending.pop(0) if self.pending else self.keys.get(timeout)

    def pos(self):
        for _ in range(40):
            p = {s: self.link.servo_pos(s) for s in range(1, 7)}
            if all(v is not None for v in p.values()):
                return p
            time.sleep(0.05)
        raise RuntimeError("no fresh servo positions from the Mega")

    def hold(self):
        for s in range(1, 7):
            self.link.write(f"H,{s}")
        self.target = {}
        self.free = False                                   # H turns torque back on where each joint is

    def move(self, targets, speed=TEACH_SPEED):
        """Move the listed joints together; space stops. -> True when all arrived (False = blocked / stopped)."""
        self.lock()                                         # the firmware ignores P while torque is off
        targets = {s: max(SAFE_MIN[s], min(SAFE_MAX[s], int(t))) for s, t in targets.items()}
        for s, t in targets.items():
            self.link.write(f"P,{s},{t},{speed},8")
            self.target[s] = t
        end, stuck = time.monotonic() + MOVE_TIMEOUT_S, {}
        while True:
            k = self.keys.get(0.02)
            if k == " ":
                self.pending.clear()                        # stop also drops the keys queued behind it
                self.hold()
                print("    STOP: holding where it is")
                return False
            if k is not None:
                self.pending.append(k)
            p = self.pos()
            left = {s: t - p[s] for s, t in targets.items() if abs(t - p[s]) > ARRIVE_TICKS}
            if not left:
                return True
            for s, err in left.items():
                if abs(err) > BLOCK_TICKS and abs(self.link.servo_speed(s)) <= 3:
                    stuck.setdefault(s, time.monotonic())
                    if time.monotonic() - stuck[s] >= BLOCK_S:
                        self.link.write(f"P,{s},{p[s]},{speed},8")     # stop pushing: hold where it stopped
                        self.target[s] = p[s]
                        self.pending.clear()                # don't keep pushing into whatever blocked it
                        print(f"    BLOCKED: J{s} {JOINT[s]} stopped {err:+d} ticks short at {p[s]} - holding there")
                        return False
                else:
                    stuck.pop(s, None)
            if time.monotonic() > end:
                self.hold()
                print(f"    TIMEOUT: still off {left} - holding where it is")
                return False

    def play(self, steps, label):
        for i, st in enumerate(steps, 1):
            print(f"  {label} {i}/{len(steps)}: " + ", ".join(f"J{s}->{t}" for s, t in st.items()))
            if not self.move(st):
                return False
        return True


def show(start, waypoints):
    print(f"  start: " + " ".join(f"J{s}={v}" for s, v in start.items()))
    for i, w in enumerate(waypoints, 1):
        print(f"  {i}. " + ", ".join(f"J{s} {JOINT[s]} -> {t}" for s, t in w.items()))


def reverse_steps(start, waypoints):
    """wake = undo each waypoint, last first: its joints go back to where they were before that waypoint."""
    before, prev = [], dict(start)
    for w in waypoints:
        before.append({s: prev[s] for s in w})
        prev.update(w)
    return list(reversed(before))


def build_config(old, start, waypoints):
    final = dict(start)
    for w in waypoints:
        final.update(w)
    cfg = dict(old)
    cfg["rest"] = {str(s): final[s] for s in range(1, 7)}
    cfg["rest_sequence"] = [{"name": "start", "joints": {str(s): start[s] for s in range(1, 7)}}] + \
                           [{"name": f"step{i}", "joints": {str(s): t for s, t in w.items()}}   # "start": autoen goes to the
                            for i, w in enumerate(waypoints, 1)]                            # taught start pose first
    moved = set().union(*waypoints)
    for s in range(1, 6):                                   # autoen wants the sequence to name every J1-5 rest value:
        if s not in moved:                                  # a joint never moved is held at its start value (no motion)
            cfg["rest_sequence"][-1]["joints"][str(s)] = start[s]
    cfg["wake_sequence"] = [{"name": f"unstep{len(waypoints) - i}", "joints": {str(s): t for s, t in w.items()}}
                            for i, w in enumerate(reverse_steps(start, waypoints))]
    cfg["taught_at"] = datetime.now().isoformat(timespec="seconds")
    cfg["taught_start"] = {str(s): start[s] for s in range(1, 7)}
    return cfg


def save(path, cfg):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=1)
    try:
        load_pose_config(tmp)                               # same checks as autoen at startup
    except SystemExit as e:
        os.remove(tmp)
        print(f"  NOT SAVED: {e}")
        return
    if os.path.exists(path):
        bak = f"{path}.before-restteach-{datetime.now():%Y%m%d_%H%M%S}"
        shutil.copy2(path, bak)
        print(f"  backup: {bak}")
    os.replace(tmp, path)
    print(f"  saved {path} -- restart autoen to use it")


def autoen_running():
    try:
        urllib.request.urlopen("http://127.0.0.1:8781/api/state", timeout=1)
        return True
    except Exception:                                       # noqa: BLE001
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description="teach the rest move as keyboard waypoints")
    ap.add_argument("--port", default=os.environ.get("ARM_SERIAL_PORT"), help="default: auto-detect")
    ap.add_argument("--file", default=POSE_CONFIG)
    ap.add_argument("--terminal-keys", action="store_true",
                    help="old 1-6 / =- terminal keys (e.g. over ssh) instead of arm_node's A-H + Z/X jog")
    a = ap.parse_args(argv)
    if autoen_running():
        raise SystemExit("autoen is running (it owns the Mega port): stop it first")
    old = {}
    if os.path.exists(a.file):
        with open(a.file, encoding="utf-8") as f:
            old = json.load(f)
        print(f"current rest_sequence in {a.file}: {[s['name'] for s in old.get('rest_sequence', [])]}")

    port = a.port or find_serial_port()
    print(f"Mega port: {port} (opening it resets the Mega; wait a few s)")
    link = Link(port)
    try:
        link.wait_ready()
        input("The arm moves ONLY on your keys, slowly. Space = stop. Hands clear? Enter = start: ")
        with Keys() as term:
            keys = term
            if not a.terminal_keys and arm_node is not None:
                try:
                    keys = EvKeys(term)
                except SystemExit as e:                     # no keyboard on the Pi / not in the input group
                    print(f"{e}\n-> using the terminal keys")
            jog = isinstance(keys, EvKeys)
            arm = Arm(link, keys)
            speed, rates, jogging = JOG_SPEED, {s: 0.0 for s in range(1, 7)}, False
            last = time.monotonic()
            jog_ids = set()                                 # joints jogging right now
            link_write = link.write

            def write(line, hb=False):                      # the Link's heartbeat sends J,<id>,0 every 0.2 s, and a 0
                if hb and line.startswith("J,") and int(line.split(",")[1]) in jog_ids:
                    return                                  # stops a jog at once: skip it (the jog feeds the watchdog)
                link_write(line, hb)
            link.write = write
            start = arm.pos()
            off = {s: start[s] - HOME[s] for s in HOME if abs(start[s] - HOME[s]) > 40}
            print("at home" if not off else f"NOT at home (off by {off}): press {'3' if jog else 'h'} to go home first")
            waypoints, joint = [], 2
            if not jog:
                print(f"joint J{joint} {JOINT[joint]} selected.  1-6 pick, =/- 10, +/_ 50, Enter waypoint, s save, q quit")
                print("by hand: f = free the arm (torque off), move it, Enter = remember each pose, f = lock")
            else:
                print("ARM_NODE KEYS (Pi keyboard): hold A S D F G H (J1..J6) + Z / X = move, 5-8 / = - speed, "
                      "Enter = remember pose,\n  Backspace undo, W wake test, R rest test, L list, K save (Y = yes), "
                      "3 home, Space/Q stop, Esc quit")
            while True:
                if jog:
                    now = time.monotonic()
                    dt, last = min(now - last, 0.1), now
                    want = arm_node.wanted_directions(keys.held())
                    for s in rates:                         # same ramp as arm_node (JOINT_COMMAND_ACCEL)
                        goal = want[s] * arm_node.JOINT_MAX_COMMAND * speed
                        stepmax = arm_node.JOINT_COMMAND_ACCEL * dt
                        rates[s] += max(-stepmax, min(stepmax, goal - rates[s]))
                    jog_ids.clear()
                    jog_ids.update(s for s, r in rates.items() if int(r))
                    if jog_ids:
                        for s in jog_ids:                   # one line each: the Link paces lines 10 ms apart
                            link.write(f"J,{s},{int(rates[s])}")
                        jogging = True
                    elif jogging:                           # keys released: hold where it stopped
                        arm.hold()
                        jogging = False
                        p = arm.pos()
                        print("  now " + " ".join(f"J{s}={p[s]}" for s in range(1, 7)))
                k = arm.key(0.02) if jog else arm.key()
                if k is not None and k.startswith("spd:"):
                    v = k[4:]
                    speed = {"+": min(1.0, speed + 0.25), "-": max(0.25, speed - 0.25)}.get(v) or float(v)
                    print(f"  jog speed {speed:.0%}")
                    continue
                if k is not None and jogging:
                    jog_ids.clear()
                    arm.hold()                              # never record / play while a joint is still moving
                    jogging, rates = False, {s: 0.0 for s in range(1, 7)}
                    time.sleep(0.2)
                if k is None:
                    continue
                if k == "q":
                    raise KeyboardInterrupt
                if k in "123456":
                    joint = int(k)
                    print(f"  J{joint} {JOINT[joint]} selected (now {arm.pos()[joint]})")
                elif k in "=-+_":
                    step = {"=": 10, "-": -10, "+": 50, "_": -50}[k]
                    base = arm.target.get(joint, arm.pos()[joint])
                    arm.move({joint: base + step})
                    print(f"  J{joint} {JOINT[joint]} = {arm.pos()[joint]}")
                elif k == " ":
                    arm.hold()
                    print("  holding")
                elif k == "f":
                    if arm.free:
                        arm.lock()
                        print("  locked where it is. f = free again")
                        continue
                    missing = arm.torque(False)
                    arm.free = True                         # even partly: lock() turns every joint back on
                    if missing:
                        arm.lock()
                        print(f"  NOT freed: no Q reply from {missing} (flash the rest-pose firmware first). Locked again")
                    else:
                        print("  FREE (it drops if you let go): move it by hand, Enter = remember this pose, f = lock")
                elif k == "h":
                    if waypoints:
                        print("  h refused: waypoints recorded (use w to go back the taught way)")
                    elif arm.move(HOME, speed=TEACH_SPEED):
                        start = arm.pos()
                        print("  at home: this is the start")
                elif k in ("\n", "\r"):
                    now, prev = arm.pos(), dict(start)
                    for w in waypoints:
                        prev.update(w)
                    w = {s: now[s] for s in range(1, 7) if abs(now[s] - prev[s]) > MOVED_TICKS}
                    if not w:
                        print("  nothing moved since the last waypoint: not recorded")
                        continue
                    waypoints.append(w)
                    print(f"  waypoint {len(waypoints)}: " + ", ".join(f"J{s} -> {t}" for s, t in w.items()))
                    if arm.free:
                        print("  still free: move to the next pose, Enter again (f = lock)")
                elif k == "u" and waypoints:
                    back = reverse_steps(start, waypoints)[0]
                    if arm.move(back):
                        waypoints.pop()
                        print(f"  undone: {len(waypoints)} waypoint(s) left")
                elif k == "l":
                    show(start, waypoints)
                elif k == "w" and waypoints:
                    if arm.play(reverse_steps(start, waypoints), "wake"):
                        print("  back at the start. r = play the rest move forward")
                elif k == "r" and waypoints:
                    if arm.play(waypoints, "rest"):
                        print("  on the rest pose")
                elif k == "s":
                    if not waypoints:
                        print("  no waypoints yet")
                        continue
                    show(start, waypoints)
                    print("  wake (reverse):")
                    for i, st in enumerate(reverse_steps(start, waypoints), 1):
                        print(f"   {i}. " + ", ".join(f"J{s} -> {t}" for s, t in st.items()))
                    print("  save this as the rest move? y = save, anything else = keep teaching")
                    while (c := arm.key()) is None:
                        pass
                    if c == "y":
                        save(a.file, build_config(old, start, waypoints))
                        print("  the arm stays where it is. w = go back to the start the taught way, q = quit")
                    else:
                        print("  not saved")
    except KeyboardInterrupt:
        print("\nquit (anything saved with s stays saved)")
    finally:
        try:
            for s in range(1, 7):
                link.write(f"H,{s}")                       # leave every joint holding where it is
        except Exception:                                   # noqa: BLE001
            pass
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
