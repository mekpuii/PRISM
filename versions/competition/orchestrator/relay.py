# relay.py - the cell's power relay on the Raspberry Pi's GPIO (2026-11-12, the operator: "when no cubes come for
# 5-10 minutes, cut the power with the relay, and switch it on again after some time to check").
#
# Wiring (the operator): relay signal on GPIO 19 = header pin 35, ground on header pin 34. It switches the cell's power
# (12 V: arm servos, belt, feeder) - NOT the Pi. The Mega stays on through its USB cable, so its sensors (feeder IR)
# and the camera keep working while the power is off.
# relay.json says which pin and which level means POWER ON. It stays DISABLED until cal/relay_test.py has been run once
# with the operator watching (a relay board can be active-low) - disabled = every call is a no-op that says so.
# Switching uses `pinctrl` (Raspberry Pi OS); the user must be in the gpio group. No Python GPIO package needed.
import json
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "relay.json")
DEFAULT = {"pin": 19, "power_on_level": 1, "enabled": False,
           "_about": "cell power relay (GPIO 19 = header pin 35, GND pin 34). enabled only after cal/relay_test.py."}


def load():
    try:
        with open(CONFIG) as fh:
            return {**DEFAULT, **json.load(fh)}
    except (OSError, ValueError):
        return dict(DEFAULT)


def save(cfg):
    tmp = CONFIG + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(cfg, fh, indent=1)
    os.replace(tmp, CONFIG)


class Relay:
    def __init__(self, cfg=None, run=subprocess.run):
        self.cfg = cfg or load()
        self._run = run                                     # (tests pass a fake)
        self.last = None                                    # "on" / "off" as last commanded

    @property
    def enabled(self):
        return bool(self.cfg.get("enabled"))

    def _level(self, power_on):
        lvl = int(self.cfg["power_on_level"]) if power_on else 1 - int(self.cfg["power_on_level"])
        return "dh" if lvl else "dl"

    def set(self, power_on, force=False):
        """Power the cell on / off. Returns a short text. Not enabled -> nothing is switched (unless force: the test)."""
        if not (self.enabled or force):
            return "relay not enabled (run cal/relay_test.py once) - power not switched"
        r = self._run(["pinctrl", "set", str(int(self.cfg["pin"])), "op", self._level(power_on)],
                      capture_output=True, text=True, timeout=5)
        if r.returncode != 0:
            raise RuntimeError(f"pinctrl: {(r.stderr or r.stdout).strip()}")
        self.last = "on" if power_on else "off"
        return f"cell power {self.last} (GPIO {self.cfg['pin']} {self._level(power_on)[1].upper()})"

    def on(self, force=False):
        return self.set(True, force)

    def off(self, force=False):
        return self.set(False, force)

    def read(self):
        """'on' / 'off' / None (the pin is not an output yet = nobody switched it)."""
        r = self._run(["pinctrl", "get", str(int(self.cfg["pin"]))], capture_output=True, text=True, timeout=5)
        out = (r.stdout or "").lower()
        if " op " not in out and "= output" not in out:
            return None
        high = " hi " in out or "| hi" in out
        return "on" if high == bool(int(self.cfg["power_on_level"])) else "off"


if __name__ == "__main__":                                  # python3 relay.py on | off | status
    import sys
    rl = Relay()
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "on":
        print(rl.on())
    elif cmd == "off":
        print(rl.off())
    else:
        print(f"relay: pin {rl.cfg['pin']}, power on = {'HIGH' if rl.cfg['power_on_level'] else 'LOW'}, "
              f"{'enabled' if rl.enabled else 'NOT enabled (run cal/relay_test.py)'}, now: {rl.read() or 'not switched yet'}")
