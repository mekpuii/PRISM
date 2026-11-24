# test_guardian.py - run: python3 test/test_guardian.py   (no robot, no broker)
# The fault detector: debounce, clearing, "never seen = no fault", the hand check only while the arm is parked and only
# with a reference photo.
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import guardian as G                                             # noqa: E402
import limits as L                                               # noqa: E402


def st(cam=True, belt=True, arm=True, light=None):
    return NS(camera_fresh=cam, belt_fresh=belt, joints_fresh=arm, light=light)


def run(g, state, secs, t0, phase="TRANSPORT", dead=False, step=0.1):
    t = t0
    while t < t0 + secs:
        g.update(state, phase, t, dead)
        t += step
    return t


def main():
    g = G.Guardian()
    t = run(g, st(cam=False, belt=False, arm=False), 30, 0.0)
    assert not g.active, "never-seen sources are no fault (blind tests, start-up)"
    print("1  nothing ever seen: no fault   OK")

    g = G.Guardian()
    t = run(g, st(), 5, 0.0)
    t = run(g, st(cam=False), L.CAMERA_LOST_S - 1, t)
    assert not g.active, "flicker"
    t = run(g, st(cam=False), 2, t)
    assert "camera_lost" in g.active
    t = run(g, st(), L.CLEAR_S - 1, t)
    assert "camera_lost" in g.active, "cleared too early"
    t = run(g, st(), 2, t)
    assert not g.active
    print("2  camera lost: needs 5 s, ends after 10 s of pictures   OK")

    g = G.Guardian()
    dark = {"v_mean": 5.0, "dark_pct": 99.0}
    t = run(g, st(light=dark), L.COVER_S + 0.5, 0.0)
    assert "camera_covered" in g.active
    t = run(g, st(light={"v_mean": 120.0, "dark_pct": 2.0}), L.CLEAR_S + 1, t)
    assert not g.active
    print("3  camera covered (black belt): fault, ends 10 s after the light is back   OK")

    g = G.Guardian()
    hand = {"v_mean": 120.0, "dark_pct": 2.0, "obj_pct": 20.0, "ref": True}
    t = run(g, st(light=hand), 3, 0.0, phase="GRIP")
    assert not g.active, "the arm is moving in the picture: no hand check"
    t = run(g, st(light=hand), L.OBJECT_S + 0.5, t, phase="TRANSPORT")
    assert "foreign" in g.active
    t = run(g, st(light={**hand, "obj_pct": 0.5}), L.OBJECT_CLEAR_S - 1, t)
    assert "foreign" in g.active
    t = run(g, st(light={**hand, "obj_pct": 0.5}), 2, t)
    assert not g.active
    g = G.Guardian()
    run(g, st(light={**hand, "ref": False}), 5, 0.0)
    assert not g.active, "no reference photo: no hand check"
    print("4  hand: only with a reference photo and a parked arm, 1 s to detect, gone 3 s to clear   OK")

    g = G.Guardian()
    t = run(g, st(), 2, 0.0)
    t = run(g, st(belt=False), L.LINK_LOST_S + 0.5, t)
    assert "feeder_link" in g.active
    t = run(g, st(arm=False), L.LINK_LOST_S + 0.5, t)
    assert "arm_link" in g.active
    print("5  the Mega link and the arm link: 3 s of silence = fault   OK")

    g = G.Guardian()
    run(g, st(), 1, 0.0, dead=True)
    assert "feeder" in g.active
    g.resolve("feeder")
    assert not g.active
    print("6  feeder dead: fault at once, ended by resolve() (a retry push drew current)   OK")

    g = G.Guardian()
    g.update(st(cam=False), "IDLE", 1.0, enabled=False)
    run(g, st(), 1, 0.0)
    t = run(g, st(cam=False), 10, 1.0)
    assert "camera_lost" in g.active
    g.update(st(), "IDLE", t, enabled=False)
    assert not g.active, "disabled (asleep): everything resets"
    print("7  disabled while asleep: no fault, the list resets   OK")
    print("\nall guardian tests passed")


if __name__ == "__main__":
    main()
