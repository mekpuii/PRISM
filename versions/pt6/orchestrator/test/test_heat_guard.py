# test_heat_guard.py - run: ~/orch-env/bin/python test/test_heat_guard.py
# 2026-11-12, the operator: "a threshold for EVERY machine, and rest when it is reached". The optimizer's safety governor
# on made-up readings, one part at a time: every arm servo J1-J6, the belt motor, the Raspberry Pi, a straining servo.
# Checks: the hot part sends the cell to COOL and is NAMED; warm = gentler only; it rests at least COOL_MIN_S and until
# every part is COOL_HYST_C below its warm limit; then it runs again by itself.
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import forecast as F                                        # noqa: E402
import optimize as O                                        # noqa: E402
from observe import ArmStatus, BeltObs, JointObs, WorldState   # noqa: E402

TMP = tempfile.mkdtemp(prefix="prism_heat_")
PI = [45.0]
O.Optimize._pi_temp = lambda self, now: PI[0]               # the Pi's own reading, made up here


def world(t, temps=None, belt=30.0, cur=None, pi=45.0):
    PI[0] = pi
    temps = temps or {}
    cur = cur or {}
    joints = {str(i): JointObs(pos=2048, speed=0, cur_ma=cur.get(i, 50.0), temp_c=temps.get(i, 35.0))
              for i in range(1, 7)}
    return WorldState(t=t, connected=True, joints=joints, joints_fresh=True, arm=ArmStatus(), arm_fresh=True,
                      gripper=None, belt=BeltObs(temp_c=belt, live_t=t), belt_fresh=True, cubes=[],
                      camera_fresh=False, cam_seq=0, events=[])


def fresh():
    o = O.Optimize(forecast=F.Forecast(path=os.path.join(TMP, "f.json")), learned_path=os.path.join(TMP, "l.json"))
    o.mode = "RUN"
    return o


def run_case(name, hot_world, want_word):
    o = fresh()
    t = 1000.0
    assert o._safety(world(t), t) is None and o.mode == "RUN"
    act = o._safety(hot_world(t + 1), t + 1)
    assert act == "idle" and o.mode == "COOL", (name, act, o.mode, o.reason)
    assert want_word in o.reason, (name, o.reason)
    # still hot / warm: stays resting
    assert o._safety(hot_world(t + 5), t + 5) is None and o.mode == "COOL"
    # cool again but too early (< COOL_MIN_S): still resting
    assert o._safety(world(t + 10), t + 10) is None and o.mode == "COOL", (name, "left COOL too early")
    act = o._safety(world(t + 2 + O.COOL_MIN_S), t + 2 + O.COOL_MIN_S)
    assert act == "resume" and o.mode == "RUN", (name, act, o.mode, o.reason)
    print(f"{name:22s} -> COOL: {o.story['saw'][:70] if o.story else ''}  ... back to RUN   OK")


def main():
    for j in range(1, 7):
        run_case(f"servo J{j} 66 C", lambda t, j=j: world(t, temps={j: 66.0}), O.JOINT_NAME[str(j)])
    run_case("belt motor 58 C", lambda t: world(t, belt=58.0), "belt motor")

    run_case("Raspberry Pi 81 C", lambda t: world(t, pi=81.0), "Raspberry Pi")

    o = fresh()
    t = 2000.0
    o._safety(world(t, cur={3: 1800.0}), t)
    assert o.mode == "RUN", "one strained reading must not stop it"
    act = o._safety(world(t + 2.5, cur={3: 1800.0}), t + 2.5)
    assert act == "idle" and o.mode == "COOL" and "straining" in (o.story or {}).get("problem", ""), o.reason
    print(f"{'J3 1800 mA for 2.5 s':22s} -> COOL: {o.story['saw'][:70]}   OK")

    o = fresh()
    act = o._safety(world(3000.0, temps={2: 58.0}), 3000.0)
    assert act is None and o.mode == "RUN" and o.heat == "warm" and "J2" in o.reason, o.reason
    print(f"{'J2 58 C (warm)':22s} -> keeps running, gentler settings: {o.reason[:60]}   OK")
    print("\nall heat guard tests passed")


if __name__ == "__main__":
    main()
