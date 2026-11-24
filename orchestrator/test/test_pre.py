import os as _os; _os.environ.setdefault("PRISM_CAMGEO", _os.devnull)   # these tests use the D415 picture
from observe import WorldState, BeltObs, ArmStatus, CubeObs
import predict as P

# This file checks the Kalman filter's tracking math over an arbitrary synthetic distance/
# duration, not the real belt's physical length, so it must not be cut short by the real
# BELT_LENGTH_MM (300 mm) -- a real cube at belt-theory speed would cross that in under 2 s.
P.BELT_LENGTH_MM = None


def make_state(t, cubes, seq, rpm=60.0):
    """A WorldState as observe.py would hand it to predict.py."""
    return WorldState(
        t=t, connected=True, joints={}, joints_fresh=False,
        arm=ArmStatus(), arm_fresh=False, gripper=None,
        belt=BeltObs(rpm=rpm, fault=0, live_t=t), belt_fresh=True,
        cubes=cubes, camera_fresh=True, events=[], cam_seq=seq,
    )


def camera_cube(x_mm, v_mm_s, cube_id=1):
    """What observe.py produces for a confirmed cube: px values, already sign-corrected."""
    return CubeObs(
        id=cube_id, color="white",
        along_px=x_mm / P.MM_PER_PX, across_px=0.0,
        along_vel_px_s=v_mm_s / P.MM_PER_PX,
        dist_m=0.4, w_px=P.CUBE_EDGE_MM / P.MM_PER_PX, h_px=P.CUBE_EDGE_MM / P.MM_PER_PX,
        hits=10,
    )


def simulate(t_end, camera_off=None, x0=100.0, dt=0.05):
    """Run the predictor against a perfect belt. camera_off = (start, end) seconds with no snapshots."""
    rpm = 60.0
    v = rpm * P.BELT_MM_S_PER_RPM               # ground truth: 165 mm/s
    pred = P.Predict()
    seq, next_snap = 0, 0.0
    log = []                                    # (t, error_mm or None, cube_count)
    t = 0.0
    while t <= t_end + 1e-9:
        cubes = []
        off = camera_off is not None and camera_off[0] <= t < camera_off[1]
        if t + 1e-9 >= next_snap:
            next_snap += 0.5                    # camera publishes every 0.5 s
            if not off:
                seq += 1
                cubes = [camera_cube(x0 + v * t, v)]
        out = pred.step(make_state(t, cubes, seq, rpm))
        truth = x0 + v * t
        if out.cubes:
            log.append((t, abs(out.cubes[0].pos_mm - truth), len(out.cubes)))
        else:
            log.append((t, None, 0))
        t = round(t + dt, 6)
    return log, pred


def main():
    # 1. Steady state: tracks the belt within a few mm.
    log, _ = simulate(10.0)
    settled = [err for (t, err, n) in log if t >= 2.0 and err is not None]
    assert settled, "no estimate produced"
    assert max(settled) < 8.0, f"steady-state error too large: {max(settled):.1f} mm"
    print(f"steady state: max error {max(settled):.2f} mm   OK")

    # 2. Blink: camera silent for 1 s (longer than a normal 0.5 s gap). Must keep tracking.
    log, _ = simulate(10.0, camera_off=(3.0, 4.0))
    blink = [err for (t, err, n) in log if 3.0 <= t < 4.0 and err is not None]
    assert blink and max(blink) < 15.0, f"blink error too large: {max(blink):.1f} mm"
    print(f"1 s blink:    max error {max(blink):.2f} mm (coasting)   OK")

    # 3. Long silence: camera silent for 3 s (longer than COAST_MAX_S). Cube must be dropped.
    log, pred = simulate(8.0, camera_off=(3.0, 6.0))
    at_5_5 = [n for (t, err, n) in log if abs(t - 5.5) < 1e-6]
    assert at_5_5 and at_5_5[0] == 0, "cube should be dropped after COAST_MAX_S"
    print(f"3 s silence:  cube dropped after {P.COAST_MAX_S:.0f} s   OK")

    # 4. Gap maths: two cubes 100 mm apart centre-to-centre -> 60 mm free between edges.
    pred = P.Predict()
    cubes = [camera_cube(200.0, 0.0, cube_id=1), camera_cube(100.0, 0.0, cube_id=2)]
    out = pred.step(make_state(0.0, cubes, seq=1))
    out = pred.step(make_state(0.0, cubes, seq=1))   # second tick: no new snapshot
    gap = out.gaps[0]["edge_gap_mm"] if out.gaps else None
    assert gap is not None and abs(gap - 60.0) < 0.5, f"gap wrong: {gap}"
    print(f"gap maths:    100 mm centre-to-centre -> {gap} mm edge gap   OK")

    print("\nall predict tests passed")


if __name__ == "__main__":
    main()