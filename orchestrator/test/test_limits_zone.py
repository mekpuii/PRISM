# test_limits_zone.py - run: ~/db-env/bin/python test/test_limits_zone.py
# The Overview's "Limits" zone (2026-11-12, the operator: "a zone with every threshold, so we know why the machine
# rests"). dashboard/overview.limits_zone() on a stand-in store. Checks: every box carries the optimizer's OWN limit
# (orchestrator/limits.py), and the one line "why it rests" names the real cause.
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PRISM = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(PRISM, "dashboard"))
sys.path.insert(0, os.path.join(PRISM, "orchestrator"))

import overview as OV                                       # noqa: E402
import limits as L                                          # noqa: E402


class Store:
    def __init__(self, temps=None, belt=None, orch=None, cur=50.0):
        t = temps or {}
        self.arm = {"joints": [{"temp": t.get(i, 35.0), "cur_mA": cur} for i in range(1, 7)]} if temps is not None else None
        self.d = {"belt/telemetry": belt, "orchestrator/status": orch}

    def get(self, k):
        return self.d.get(k)

    def age(self, k):
        if k == "arm/telemetry":
            return 0.5 if self.arm else None
        return 0.5 if self.d.get(k) is not None else None


def zone(**k):
    return OV.limits_zone(Store(**k), {"temp_c": k.pop("pi", 40.0) if False else 40.0})


def row(z, key):
    return next(r for r in z["rows"] if r["key"] == key)


def main():
    belt = {"temp": 25.0, "voltage": 12.1}
    z = OV.limits_zone(Store(temps={}, belt=belt, orch={"mode": "RUN", "reason": "x", "score": {"flow": {"pushes_last_min": 4}}}),
                       {"temp_c": 40.0})
    assert [r["key"] for r in z["rows"]] == ["j1", "j2", "j3", "j4", "j5", "j6", "belt", "pi", "strain", "rail", "feeder",
                                             "sleep"], [r["key"] for r in z["rows"]]
    assert not z["resting"] and not z["warm"] and z["live"], z
    assert row(z, "j3")["hot"] == L.SERVO_HOT_C and row(z, "belt")["hot"] == L.BELT_HOT_C and row(z, "pi")["hot"] == L.PI_HOT_C
    assert row(z, "feeder")["now"] == 4 and row(z, "feeder")["state"] == "ok"
    print("1  all cool: 12 boxes, every limit is the optimizer's own, nothing resting   OK")

    z = OV.limits_zone(Store(temps={2: 58.0}, belt=belt, orch={"mode": "RUN", "reason": "warm", "score": {}}), {"temp_c": 40.0})
    assert row(z, "j2")["state"] == "warn" and z["warm"] == ["J2 shoulder"] and not z["resting"], z
    print("2  J2 at 58 C: amber, 'warm: J2 shoulder', still running   OK")

    reason = "too hot / straining: J3 elbow 66 C (limit 65) - resting at home until cool"
    z = OV.limits_zone(Store(temps={3: 66.0}, belt=belt, orch={"mode": "COOL", "reason": reason, "score": {}}), {"temp_c": 40.0})
    assert row(z, "j3")["state"] == "bad" and z["resting"] and z["why"] == "J3 elbow 66 C (limit 65)", z["why"]
    print(f"3  J3 at 66 C: red, resting because: {z['why']}   OK")

    z = OV.limits_zone(Store(temps={}, belt=belt, orch={"mode": "RUN", "reason": "x", "score": {
        "flow": {"pushes_last_min": 13, "feeder_rest_s": 40, "feeder_limit": 12}}}), {"temp_c": 40.0})
    assert z["resting"] and "feeder servo cooling" in z["why"] and row(z, "feeder")["state"] == "bad", z
    print(f"4  feeder over its push limit: resting because: {z['why']}   OK")

    z = OV.limits_zone(Store(temps={}, belt=belt, orch={"mode": "SLEEP", "reason": "sleeping", "score": {"idle_s": 400}}),
                       {"temp_c": 40.0})
    assert z["resting"] and "asleep" in z["why"] and abs(row(z, "sleep")["now"] - 6.7) < 0.1, z
    print(f"5  asleep: resting because: {z['why']}   OK")

    z = OV.limits_zone(Store(temps={}, belt=belt, orch={"mode": "IDLE", "reason": "feeder empty: ...", "score": {"idle_s": 90}}),
                       {"temp_c": 40.0})
    assert z["resting"] and "feeder is empty" in z["why"] and row(z, "sleep")["now"] == 1.5, z
    print(f"6  empty feeder: resting because: {z['why']} (no cubes for 1.5 min of 5)   OK")

    z = OV.limits_zone(Store(temps={}, belt={"temp": 25.0, "voltage": 11.2}, orch=None), {"temp_c": 82.0})
    assert row(z, "rail")["state"] == "warn" and row(z, "pi")["state"] == "bad", z
    print("7  rail 11.2 V: amber (pushes wait); Pi 82 C: red   OK")

    z = OV.limits_zone(Store(temps=None, belt=None, orch=None), {"temp_c": 40.0})
    assert not z["live"] and all(row(z, k)["state"] == "off" for k in ("j1", "belt", "rail", "strain")), z
    assert row(z, "pi")["state"] == "ok", "the Pi is always readable"
    print("8  the cell is not running: every box '-', the Pi still read, the limits still shown   OK")
    z = OV.limits_zone(Store(temps={}, belt=belt, orch={"mode": "FAULT", "reason": "FAULT: x", "score": {"fault": {
        "text": ["the camera is covered or its lens is blocked"]}}}), {"temp_c": 40.0})
    assert z["resting"] and z["why"].startswith("FAULT: the camera is covered"), z
    print(f"9  FAULT mode: resting because: {z['why']}   OK")
    print("\nall limits zone tests passed")


if __name__ == "__main__":
    main()
