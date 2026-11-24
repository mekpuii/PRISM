#!/usr/bin/env python3
"""
mega_link.py -- Raspberry Pi side of the SO-101 Mega link (arm protocol + conveyor module).

One serial port, one owner: close the Xbox dashboard before running anything that uses this.

Lines the Mega sends (parsed here):
  H,SO101_MEGA_V1            hello (1 Hz)
  R,<0|1>                    1 = all six STS3215 answered in position mode
  S,<id>,<mode>,<pos>        boot status per servo
  T,<id>,<pos>,<speed>,<cur> servo telemetry (one servo every 10 ms, round robin)
  IR,<raw>,<detected>        IR sensor (D22)
  G,<state>,<pos>            gripper crawl: CLOSING / CLOSED / ERROR
  X,OK | X,ERR,<why> | X,PONG            replies to X commands
  X,E,<NAME>,<ms>[,<arg>]                belt events ARRIVED / FEED_DONE / FAULT
  X,T,...                                belt telemetry (17 values, see belt_module.h)

Writes are paced >= 10 ms apart: the Mega has a 64-byte serial RX buffer and its arm loop
blocks on servo reads, so bursts of commands get dropped.
Keep-alives (the Mega freezes an arm target if it hears nothing for 500 ms):
  J,<id>,0 for ids 1..6 every 0.2 s, and X,HB every 0.2 s for the belt watchdog.
"""

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Union

import serial

MODE_NAMES = {0: "COAST", 1: "PWM", 2: "RPM", 3: "BRAKE"}
CONV_NAMES = {0: "IDLE", 1: "RUN", 2: "OFFSET", 3: "ARRIVED"}
FEED_NAMES = {0: "IDLE", 1: "PUSH", 2: "RETURN"}
FAULT_NAMES = {0: "NONE", 1: "WATCHDOG", 2: "OVERTEMP", 3: "TEMP_SENSOR", 4: "STALL"}

N_BELT_FIELDS = 17               # values after "X,T" (must match belt_module.h printTelemetry())
MIN_WRITE_GAP_S = 0.010          # spacing between any two lines sent to the Mega
KEEPALIVE_S = 0.2
SERVO_IDS = range(1, 7)


class MegaFault(RuntimeError):
    """The belt module latched a fault (motor already coasted). Send stop() to clear."""

    def __init__(self, code: int):
        self.code = code
        super().__init__(f"Mega fault {code} ({FAULT_NAMES.get(code, '?')})")


@dataclass
class BeltTelemetry:
    ms: int              # Mega uptime
    enc: int             # encoder counts (x4 decoded)
    rpm: float           # output-shaft rpm (filtered)
    pwm: int             # last PWM written, -255..255
    mode: int            # MODE_NAMES
    belt_ma: float       # INA260 #1 (0x40) belt motor
    belt_mv: float
    belt_mw: float
    arm_ma: float        # INA260 #2 (0x41) arm rail
    arm_mv: float
    arm_mw: float
    temp_c: float        # DS18B20 on the motor (nan if missing)
    ir: bool             # True = block present at the IR (debounced, shared with the arm)
    feeder: int          # last feeder servo angle
    fault: int           # FAULT_NAMES
    conv: int            # CONV_NAMES
    feed: int            # FEED_NAMES


@dataclass
class ServoState:
    pos: Optional[int] = None
    speed_raw: int = 0
    current_raw: int = 0
    t: float = 0.0       # time.monotonic() of the last update
    temp_c: Optional[int] = None     # from the TT lines (slow: about once every few seconds per servo)
    volt_dv: Optional[int] = None    # servo supply voltage, 0.1 V units (TT lines)
    tv_t: float = 0.0                # time.monotonic() of the last TT line


@dataclass
class Event:
    name: str
    ms: int
    args: List[str] = field(default_factory=list)


class MegaLink:
    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200):
        # timeout keeps readline() from blocking forever so threads can exit
        self.ser = serial.Serial(port, baud, timeout=0.1)
        # Opening the port resets the Mega. Do NOT write anything until the sketch itself is running:
        # bytes sent while the bootloader is still listening can confuse it. Wait for the boot banner.
        self.boot_lines: List[str] = []
        boot_end = time.monotonic() + 12.0
        saw_boot = False
        while time.monotonic() < boot_end and not saw_boot:
            raw = self.ser.readline()
            if raw:
                text = raw.decode("ascii", errors="ignore").strip()
                self.boot_lines.append(text)
                if "BOOT_DONE" in text or text.startswith("H,SO101"):   # sketch is running
                    saw_boot = True
        self.saw_boot = saw_boot
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self._wlock = threading.Lock()  # serialises + paces every write
        self._last_write = 0.0
        self._running = True

        # parsed state (written by the rx thread, read by everyone else)
        self._slock = threading.Lock()
        self.firmware_ready: Optional[bool] = None
        self.last_hello = 0.0
        self.servo: Dict[int, ServoState] = {i: ServoState() for i in SERVO_IDS}
        self.ir_detected = False
        self.ir_seen = 0.0
        self.gripper_state = "IDLE"
        self.gripper_pos: Optional[int] = None
        self._belt: Optional[BeltTelemetry] = None

        self._replies: "queue.Queue[str]" = queue.Queue()
        self._events: "queue.Queue[Event]" = queue.Queue()
        self._gripper_events: "queue.Queue[tuple]" = queue.Queue()

        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._hb = threading.Thread(target=self._hb_loop, daemon=True)
        self._rx.start()
        self._hb.start()

    # ------------------------------------------------------------ writing
    def write(self, line: str):
        """Send one line (no newline needed), at least 10 ms after the previous one."""
        with self._wlock:
            wait = self._last_write + MIN_WRITE_GAP_S - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            self.ser.write((line + "\n").encode("ascii"))
            self._last_write = time.monotonic()

    def _hb_loop(self):
        while self._running:
            try:
                self.write("X,HB")                      # belt watchdog, silent
                for sid in SERVO_IDS:                   # arm command watchdog, harmless at rate 0
                    self.write(f"J,{sid},0")
            except serial.SerialException:
                break
            time.sleep(KEEPALIVE_S)

    # ------------------------------------------------------------ reading
    def _rx_loop(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if line:
                self._parse(line)

    def _parse(self, line: str):
        p = line.split(",")
        k = p[0]
        try:
            if k == "X" and len(p) >= 2:
                self._parse_x(p)
            elif k == "T" and len(p) == 5:
                sid = int(p[1])
                pos = int(p[2])
                with self._slock:
                    if sid in self.servo:
                        old = self.servo[sid]                     # keep the slow temperature / voltage values
                        self.servo[sid] = ServoState(pos if 0 <= pos <= 4095 else None,
                                                     int(p[3]), int(p[4]), time.monotonic(),
                                                     old.temp_c, old.volt_dv, old.tv_t)
            elif k == "TT" and len(p) == 4:                       # TT,<id>,<temp_C>,<volt_x10>
                sid = int(p[1])
                with self._slock:
                    if sid in self.servo:
                        st = self.servo[sid]
                        st.temp_c = int(p[2]) if int(p[2]) >= 0 else None
                        st.volt_dv = int(p[3]) if int(p[3]) >= 0 else None
                        st.tv_t = time.monotonic()
            elif k == "IR" and len(p) >= 3 and p[1] in ("0", "1") and p[2] in ("0", "1"):
                with self._slock:
                    self.ir_detected = p[2] == "1"
                    self.ir_seen = time.monotonic()
            elif k == "G" and len(p) >= 3:
                pos = int(p[2]) if p[2].lstrip("-").isdigit() else None
                with self._slock:
                    self.gripper_state, self.gripper_pos = p[1], pos
                self._gripper_events.put((p[1], pos))
            elif k == "R" and len(p) >= 2:
                with self._slock:
                    self.firmware_ready = int(p[1]) == 1
            elif k == "H" and len(p) >= 2 and p[1] == "SO101_MEGA_V1":
                with self._slock:
                    self.last_hello = time.monotonic()
        except ValueError:
            pass                                        # corrupted line: drop it

    def _parse_x(self, p: List[str]):
        sub = p[1]
        if sub == "T":
            f = p[2:]
            if len(f) != N_BELT_FIELDS:
                return
            t = BeltTelemetry(
                ms=int(f[0]), enc=int(f[1]), rpm=float(f[2]), pwm=int(f[3]), mode=int(f[4]),
                belt_ma=float(f[5]), belt_mv=float(f[6]), belt_mw=float(f[7]),
                arm_ma=float(f[8]), arm_mv=float(f[9]), arm_mw=float(f[10]),
                temp_c=float(f[11]),                    # float("nan") works
                ir=f[12] == "1", feeder=int(f[13]), fault=int(f[14]),
                conv=int(f[15]), feed=int(f[16]))
            with self._slock:
                self._belt = t
        elif sub == "E" and len(p) >= 4:
            self._events.put(Event(name=p[2], ms=int(p[3]), args=p[4:]))
        elif sub in ("OK", "ERR", "PONG"):
            self._replies.put(",".join(p[1:]))          # "OK", "ERR,<why>", "PONG"

    # ------------------------------------------------------------ belt commands
    def belt_command(self, cmd: str, timeout: float = 1.0) -> str:
        """Send 'X,<cmd>' and return the reply ('OK', 'ERR,why', 'PONG')."""
        while not self._replies.empty():                # drop stale replies
            self._replies.get_nowait()
        self.write("X," + cmd)
        try:
            return self._replies.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"no reply to 'X,{cmd}'")

    def _ok(self, cmd: str):
        # 'ERR,unknown_cmd' means the Mega did NOT run the line (bytes were lost/garbled on the way in, e.g. while
        # the sketch was busy with a slow servo read), so asking again is safe. Real refusals (ERR,fault_...) are not retried.
        r = self.belt_command(cmd)
        for _ in range(3):
            if r != "ERR,unknown_cmd":
                break
            time.sleep(0.1)
            r = self.belt_command(cmd)
        if r != "OK":
            raise RuntimeError(f"'X,{cmd}' -> {r}")

    def belt_telemetry(self) -> Optional[BeltTelemetry]:
        with self._slock:
            return self._belt

    def clear_events(self):
        """Call BEFORE sending the command whose event you will wait for."""
        while not self._events.empty():
            self._events.get_nowait()

    def wait_event(self, names: Union[str, Sequence[str]], timeout: float) -> Event:
        """Block until one of `names` arrives. A FAULT event raises MegaFault."""
        if isinstance(names, str):
            names = (names,)
        end = time.monotonic() + timeout
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no {'/'.join(names)} event within {timeout}s")
            try:
                ev = self._events.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError(f"no {'/'.join(names)} event within {timeout}s")
            if ev.name == "FAULT":
                raise MegaFault(int(ev.args[0]) if ev.args else -1)
            if ev.name in names:
                return ev

    def set_pwm(self, pwm: int):        self._ok(f"PWM,{int(pwm)}")
    def set_rpm(self, rpm: float):      self._ok(f"RPM,{rpm:.2f}")
    def conveyor_run(self, rpm: float): self._ok(f"CONV,{rpm:.2f}")    # run until IR, brake, ARRIVED
    def set_stop_offset(self, counts: int): self._ok(f"OFFSET,{int(counts)}")
    def stop(self):                     self._ok("STOP")     # coast + clear fault
    def brake(self):                    self._ok("BRAKE")
    def zero_encoder(self):             self._ok("ZERO")
    def feed(self):                     self._ok("FEED")
    def configure_feeder(self, home: int, push: int, push_ms: int = 400, return_ms: int = 400):
        self._ok(f"FEEDCFG,{int(home)},{int(push)},{int(push_ms)},{int(return_ms)}")
    def set_feeder_angle(self, angle: int): self._ok(f"SERVO,{int(angle)}")
    def set_pid(self, kp, ki, kd):      self._ok(f"PID,{kp},{ki},{kd}")
    def set_cpr(self, cpr: float):      self._ok(f"CPR,{cpr}")
    def set_temp_max(self, c: float):   self._ok(f"TEMPMAX,{c}")
    def set_watchdog(self, ms: int):    self._ok(f"WDT,{int(ms)}")
    def set_stream(self, ms: int):      self._ok(f"STREAM,{int(ms)}")

    # ------------------------------------------------------------ arm helpers
    def servo_pos(self, sid: int, max_age: float = 1.0) -> Optional[int]:
        """Latest position of a servo, or None if unknown/stale."""
        with self._slock:
            s = self.servo[sid]
        if s.pos is None or time.monotonic() - s.t > max_age:
            return None
        return s.pos

    def clear_gripper_events(self):
        while not self._gripper_events.empty():
            self._gripper_events.get_nowait()

    def wait_gripper(self, states: Sequence[str], timeout: float):
        """Wait for a G,<state> line in `states`; returns (state, position)."""
        end = time.monotonic() + timeout
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no gripper {'/'.join(states)} within {timeout}s")
            try:
                st, pos = self._gripper_events.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError(f"no gripper {'/'.join(states)} within {timeout}s")
            if st in states:
                return st, pos

    def wait_ready(self, timeout: float = 20.0):
        """Block until the Mega says R,1, every servo reports a position, and the belt module answers."""
        end = time.monotonic() + timeout
        why = "nothing received yet"
        while time.monotonic() < end:
            with self._slock:
                ready = self.firmware_ready
            missing = [i for i in SERVO_IDS if self.servo_pos(i, 1.5) is None]
            if ready is not True:
                why = f"no 'R,1' from the arm firmware (R state: {ready})"
            elif missing:
                why = f"no fresh position for servo(s) {missing}"
            else:
                try:
                    reply = self.belt_command("PING")
                    if reply == "PONG":
                        return
                    why = f"X,PING answered '{reply}' instead of PONG"
                except TimeoutError:
                    why = "no reply to X,PING (belt_module.h not uploaded?)"
            time.sleep(0.2)
        raise TimeoutError(f"Mega not ready after {timeout:.0f}s: {why}. "
                           f"saw BOOT_DONE at open: {self.saw_boot}; first lines at open: {self.boot_lines[:6]}")

    def close(self):
        try:
            self.stop()                 # never leave the belt running
        except Exception:
            pass
        self._running = False
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
