# test_observe.py - tests for observe.py. No camera, no robot, no broker needed.
# Run:  python3 test_observe.py     Expected last line: "all observe tests passed"
#
# HOW THE TESTS WORK
#   inject() puts a message in the queue exactly as the broker callback would.
#   step() drains the queue. _snapshot(now) builds a WorldState at a time we choose,
#   so freshness ("is this data still live?") can be tested without waiting.

import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
import time

from observe import Observe, MA_PER_COUNT, STALE_S

T_ARM_TEL   = "arm/telemetry"
T_ARM_STATE = "arm/state"
T_GRIPPER   = "arm/gripper"
T_BELT_TEL  = "belt/telemetry"
T_BELT_EVT  = "belt/event"
T_CAM_PERC  = "camera/perception"


def arm_telemetry(base_pos=2000, cur=10):
    """A complete six-joint telemetry message, as autoen_node.py publishes it."""
    return {"ts": 0, "joints": {
        str(i): {"pos": base_pos + i, "speed": 0, "cur": cur, "temp": 35}
        for i in range(1, 7)
    }}


def camera_msg(tracks, axis="u", direction=1):
    """A camera/perception message. tracks = list of dicts with at least id, u, v."""
    return {"ts": 0, "axis": axis, "dir": direction, "tracks": tracks}


def track(tid, u, v=200, vu=0.0, vv=0.0, hits=10, color="white"):
    """One track entry, shaped like publish_perception() in the camera node."""
    return {"id": tid, "u": u, "v": v, "vu": vu, "vv": vv, "hits": hits,
            "color": color, "dist_m": 0.4, "w": 40, "h": 40}


def run_tests():
    o = Observe()
    try:
        # ------------------------------------------------------------
        # 1. Six joints are parsed. A complete set becomes joints 1..6.
        # ------------------------------------------------------------
        o.inject(T_ARM_TEL, arm_telemetry(base_pos=2000, cur=10))
        s = o.step()
        assert len(s.joints) == 6, f"expected 6 joints, got {len(s.joints)}"
        assert s.joints[1].pos == 2001
        # current is converted from counts to mA
        assert abs(s.joints[1].cur_ma - 10 * MA_PER_COUNT) < 1e-9
        print("1. six joints parsed, current in mA            OK")

        # ------------------------------------------------------------
        # 2. An incomplete joint set must NOT replace the previous complete set.
        # ------------------------------------------------------------
        partial = arm_telemetry(base_pos=5000)
        del partial["joints"]["3"]
        o.inject(T_ARM_TEL, partial)
        s = o.step()
        assert s.joints[1].pos == 2001, "a partial set overwrote the good one"
        print("2. partial joint set keeps previous good set   OK")

        # ------------------------------------------------------------
        # 3. Gripper and belt events are handed over EXACTLY ONCE.
        # ------------------------------------------------------------
        o.inject(T_GRIPPER, {"state": "CLOSED", "pos": 1900, "reason": None})
        o.inject(T_BELT_EVT, {"ts": 0, "name": "ARRIVED", "args": ["12345"]})
        s = o.step()
        names = sorted(e.name for e in s.events)
        assert names == ["ARRIVED", "CLOSED"], f"events wrong: {names}"
        assert o.step().events == [], "events were delivered twice"
        print("3. events delivered once, then cleared         OK")

        # ------------------------------------------------------------
        # 4. Camera: hits is read, and the cube list is REPLACED not merged.
        # ------------------------------------------------------------
        o.inject(T_CAM_PERC, camera_msg([track(1, 100, hits=3), track(2, 200, hits=9)]))
        s = o.step()
        assert {c.id for c in s.cubes} == {1, 2}
        assert {c.id: c.hits for c in s.cubes} == {1: 3, 2: 9}
        o.inject(T_CAM_PERC, camera_msg([track(2, 210, hits=10)]))
        s = o.step()
        assert [c.id for c in s.cubes] == [2], "old cube 1 was merged back in"
        print("4. hits parsed, cube list replaced (not merged) OK")

        # ------------------------------------------------------------
        # 5. Camera sign convention: dir = -1 flips the along-belt axis.
        #    A cube at u = 320 px must come out as along_px = -320.
        # ------------------------------------------------------------
        o.inject(T_CAM_PERC, camera_msg([track(7, 320, vu=12.0)], axis="u", direction=-1))
        s = o.step()
        c = s.cubes[0]
        assert c.along_px == -320, f"sign wrong: {c.along_px}"
        assert c.along_vel_px_s == -12.0, f"velocity sign wrong: {c.along_vel_px_s}"
        print("5. dir=-1 flips along-belt sign (pos and vel) OK")

        # ------------------------------------------------------------
        # 6. Camera axis v: along-belt coordinate is v, not u.
        # ------------------------------------------------------------
        o.inject(T_CAM_PERC, camera_msg([{"id": 8, "u": 111, "v": 222, "vu": 1.0, "vv": 5.0,
                                          "hits": 10, "color": "red"}], axis="v", direction=1))
        s = o.step()
        assert s.cubes[0].along_px == 222 and s.cubes[0].along_vel_px_s == 5.0
        assert s.cubes[0].across_px == 111
        print("6. axis=v uses v as along-belt                 OK")

        # ------------------------------------------------------------
        # 7. cam_seq counts LIVE snapshots only.
        #    A retained copy (sent when we reconnect) must NOT count as a new snapshot,
        #    or predict.py would correct its filters with stale data.
        # ------------------------------------------------------------
        before = o.step().cam_seq
        o.inject(T_CAM_PERC, camera_msg([track(9, 50)]), retained=True)
        after_retained = o.step().cam_seq
        assert after_retained == before, "a retained copy bumped cam_seq"
        o.inject(T_CAM_PERC, camera_msg([track(9, 50)]))
        after_live = o.step().cam_seq
        assert after_live == before + 1, "a live snapshot did not bump cam_seq"
        print("7. cam_seq counts live snapshots, not retained OK")

        # ------------------------------------------------------------
        # 8. Freshness uses OUR receive time, and goes stale after the limit.
        #    _snapshot(now) lets us ask "is it fresh at time now?" directly.
        # ------------------------------------------------------------
        o.inject(T_CAM_PERC, camera_msg([track(10, 60)]))
        s_now = o.step()
        t_rx = s_now.t
        assert o._snapshot(t_rx + 0.5).camera_fresh, "camera should be fresh 0.5 s after receipt"
        assert not o._snapshot(t_rx + STALE_S["camera"] + 0.5).camera_fresh, \
            "camera should be stale after its limit"
        print(f"8. camera fresh within {STALE_S['camera']:.0f} s, stale after   OK")

        # ------------------------------------------------------------
        # 9. A retained arm/state must not make the arm look alive.
        # ------------------------------------------------------------
        o.inject(T_ARM_STATE, {"mode": "auto", "status": "running", "phase": "HOME", "cycle": 4},
                 retained=True)
        s = o.step()
        assert s.arm.phase == "HOME", "retained state was not read"
        assert not s.arm_fresh, "a retained arm/state was treated as live"
        print("9. retained arm/state is read but not 'live'   OK")

        # ------------------------------------------------------------
        # 10. Bad messages are counted and skipped, and never crash the loop.
        # ------------------------------------------------------------
        before_bad = o.bad_messages
        o.inject(T_ARM_TEL, [1, 2, 3])                  # not a JSON object
        o.inject(T_ARM_TEL, {"ts": 0})                  # no "joints" key
        o.inject(T_CAM_PERC, {"ts": 0, "tracks": [{"u": 1}]})   # track without id -> skipped, not counted
        s = o.step()
        assert o.bad_messages - before_bad == 2, f"bad count wrong: {o.bad_messages - before_bad}"
        print("10. malformed messages counted and skipped     OK")

        print("\nall observe tests passed")
    finally:
        o.shutdown()


if __name__ == "__main__":
    run_tests()