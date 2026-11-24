#!/usr/bin/env python3
"""relay_test.py - teach PRISM its power relay, ONCE, with the operator watching (2026-11-12).

The relay (GPIO 19 = header pin 35, GND = pin 34) cuts the cell's power while PRISM sleeps. A relay board may switch on
with the pin HIGH or with it LOW - this asks you which, and only then enables the relay (orchestrator/relay.json).

BEFORE: stop the cell (./stop_cell.sh --all) - the arm MUST be resting on something (it drops when its power goes),
hands away from the belt and the feeder.
    cd ~/Desktop/PRISM/orchestrator && ~/orch-env/bin/python cal/relay_test.py
It drives the pin HIGH, asks "is the power on?", then LOW, asks again, saves the answer and leaves the power ON.
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import relay as R                                           # noqa: E402


def ask(q):
    while True:
        a = input(q + " [y/n] ").strip().lower()
        if a in ("y", "n"):
            return a == "y"


def main():
    cfg = R.load()
    print(f"Relay on GPIO {cfg['pin']} (header pin 35).")
    print("The cell must be stopped, the arm RESTING on something (it drops without power), hands clear.")
    if not ask("Ready?"):
        return
    rl = R.Relay(dict(cfg, power_on_level=1))
    seen = {}
    for lvl in (1, 0):
        rl.set(power_on=bool(lvl), force=True)              # power_on_level=1: on = HIGH, off = LOW
        print(f"\nGPIO {cfg['pin']} is now {'HIGH' if lvl else 'LOW'}. Wait 2 s, then look: servo LEDs, belt driver "
              "light, the feeder servo, a meter on the 12 V ...")
        time.sleep(2.0)
        seen[lvl] = ask("Is the cell POWER ON now?")
    if seen[1] == seen[0]:
        print("\nThe same answer for HIGH and LOW: the relay does not switch the power. Check its wiring (VCC, GND on "
              "pin 34, signal on pin 35) - the relay stays DISABLED.")
        rl.set(power_on=True, force=True)
        return
    cfg["power_on_level"] = 1 if seen[1] else 0
    cfg["enabled"] = True
    cfg["tested"] = time.strftime("%Y-%m-%d %H:%M")
    R.save(cfg)
    rl = R.Relay(cfg)
    print("\n" + rl.on())
    print(f"Saved: power ON = GPIO {'HIGH' if cfg['power_on_level'] else 'LOW'}; the relay is ENABLED "
          f"(orchestrator/relay.json). PRISM may now cut the power while it sleeps.")


if __name__ == "__main__":
    main()
