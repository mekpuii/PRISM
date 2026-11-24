# test_replay.py - run: python3 test_replay.py
# Builds a fake recording whose truth we know, then checks replay.py can TELL a correct belt
# sign / scale from a wrong one. A replay tool that cannot tell them apart would be useless.
import random

import predict as P
import replay as R

MM_PER_PX_TRUE = 0.95
P.MM_PER_PX = MM_PER_PX_TRUE
RPM = 30.0
TRUE_MM_S = RPM * P.BELT_MM_S_PER_RPM            # 82.5 mm/s: the belt really moves at the theory speed
SPEED_PX_S = TRUE_MM_S / MM_PER_PX_TRUE
random.seed(7)


def fake_run(dir_value=1):
    """Cubes move downstream at SPEED_PX_S. Camera 2 Hz, noisy u (+-1.5 px) and vu (+-6 px/s)."""
    cam, belt = [], []
    for k in range(200):                         # 100 s
        t = 1000.0 + k * 0.5
        tracks = []
        for cid, u0 in ((1, 100.0), (2, 260.0)):
            u = u0 + SPEED_PX_S * (k * 0.5)      # no wrap: a cube must not teleport
            vu = SPEED_PX_S + random.uniform(-6, 6)
            # camera convention: dir=-1 means coordinate SHRINKS downstream, so store it mirrored
            if dir_value < 0:
                u, vu = 20000 - u, -vu
            tracks.append({"id": cid, "u": int(round(u + random.uniform(-1.5, 1.5))), "v": 200,
                           "color": "WHITE", "w": 42, "h": 42, "vu": round(vu, 1), "vv": 0.0, "hits": 10})
        cam.append((t, {"axis": "u", "dir": dir_value, "tracks": tracks}))
        belt.append((t - 0.1, {"velocity": RPM, "fault": 0}))
    return cam, belt


def mae(cam, belt, sign):
    P.BELT_SIGN = sign
    s = R.summarize(R.replay(cam, belt, hide_every=3))
    assert s, "replay produced no comparisons"
    return s[2]


def test_sign_is_detectable():
    cam, belt = fake_run(dir_value=1)
    good, bad = mae(cam, belt, +1), mae(cam, belt, -1)
    print(f"sign +1 (right): MAE {good:.2f} mm   sign -1 (wrong): MAE {bad:.2f} mm")
    assert good < 5.0, "correct sign should track within a few mm"
    assert bad > 2 * good, "wrong sign must be clearly worse, or replay cannot find sign errors"


def test_dir_minus_one_handled():
    """Same physics, camera axis mirrored (dir=-1). observe flips it, so BELT_SIGN=+1 must still fit."""
    cam, belt = fake_run(dir_value=-1)
    good, bad = mae(cam, belt, +1), mae(cam, belt, -1)
    print(f"dir=-1: sign +1 MAE {good:.2f} mm   sign -1 MAE {bad:.2f} mm")
    assert good < 5.0 and bad > 2 * good


def test_wrong_belt_speed_is_visible():
    """Real belt 10x slower than the pulley theory (the suspected discrepancy): replay must show it."""
    cam, belt = fake_run()
    belt = [(t, {"velocity": RPM * 10, "fault": 0}) for t, _ in belt]    # prior says 825 mm/s, truth 82
    P.BELT_SIGN = +1
    s = R.summarize(R.replay(cam, belt, hide_every=3))
    print(f"prior 10x too fast: MAE {s[2]:.1f} mm")
    assert s[2] > 5.0


def test_no_belt_data_still_runs():
    cam, _ = fake_run()
    P.BELT_SIGN = +1
    assert R.summarize(R.replay(cam, [], hide_every=3)) is not None


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"{name}: OK")
    print("\nall replay tests passed")
