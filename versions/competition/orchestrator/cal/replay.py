# replay.py - replay a recorded run through observe.py's parser and predict.py.
#
# Usage:
#   python3 replay.py runs/run3.jsonl                 (record_run.py file: camera + belt in one file)
#   python3 replay.py run2.jsonl belt2.jsonl          (old style: two mosquitto_sub files)
#   add  --try-sign  to compare BELT_SIGN = +1 against -1 and say which fits the data.
#
# Every HIDE_EVERY-th camera snapshot is hidden from the filter. The filter's prediction for that
# moment is compared with the camera's own measurement of it.
#   bias = average error    -> same sign every time = drift (scale or belt sign wrong)
#   MAE  = average |error|  -> how far off the coasting filter is
# LIMIT: the camera's measurement is not ground truth. This checks consistency, not accuracy.

import bisect
import json
import sys

from observe import Observe, WorldState, BeltObs, ArmStatus   # reuses the real parser
import predict as P

HIDE_EVERY = 3        # hide snapshot 3, 6, 9, ... from the filter (~1 s gap at 2 Hz)
BELT_FRESH_S = 1.0    # a belt sample older than this counts as "not fresh"


class _Stub:
    """Only the fields Observe._camera writes. Lets us call the real parser without MQTT."""
    def __init__(self):
        self._cubes, self._cam_live_t, self._cam_seq = [], None, 0


def load(path):
    """Read a JSONL file. Returns (camera rows, belt rows), each [(time, payload)], sorted by time.
    Understands both formats: record_run.py lines ({"rt","topic","data"}) and raw mosquitto_sub lines."""
    cam, belt = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if "header" in o:
                continue
            if "topic" in o and "rt" in o:                  # record_run.py format
                if o.get("retained") or not isinstance(o["data"], dict):
                    continue
                if o["topic"] == "camera/perception":
                    cam.append((o["rt"], o["data"]))
                elif o["topic"] == "belt/telemetry":
                    belt.append((o["rt"], o["data"]))
            elif "tracks" in o:                             # raw camera line
                cam.append((o["ts"], o))
            elif "velocity" in o:                           # raw belt line
                belt.append((o["ts"], o))
    return sorted(cam, key=lambda r: r[0]), sorted(belt, key=lambda r: r[0])


def cubes_from(data):
    """Parse one camera payload exactly as observe.py does."""
    stub = _Stub()
    Observe._camera(stub, data, False, 0.0)
    return stub._cubes


class BeltIndex:
    """Latest belt sample at or before t, in O(log n)."""
    def __init__(self, rows):
        self.rows = rows
        self.ts = [r[0] for r in rows]

    def at(self, t):
        i = bisect.bisect_right(self.ts, t) - 1
        return self.rows[i] if i >= 0 else None


def make_state(t, cubes, seq, belt_row):
    """Build the WorldState predict.py expects, from one recorded moment."""
    belt, fresh = BeltObs(), False
    if belt_row is not None and t - belt_row[0] <= BELT_FRESH_S:
        belt = BeltObs(rpm=belt_row[1].get("velocity"),
                       fault=int(belt_row[1].get("fault") or 0), live_t=t)
        fresh = True
    return WorldState(t=t, connected=True, joints={}, joints_fresh=False,
                      arm=ArmStatus(), arm_fresh=False, gripper=None,
                      belt=belt, belt_fresh=fresh, cubes=cubes,
                      camera_fresh=True, cam_seq=seq, events=[])


def replay(cam_rows, belt_rows, hide_every=HIDE_EVERY):
    """Run the filter over the recording. Returns [(cube id, error_mm)]."""
    pred = P.Predict()
    belts = BeltIndex(belt_rows)
    seq, errors = 0, []
    for i, (t, data) in enumerate(cam_rows):
        cubes = cubes_from(data)
        belt_row = belts.at(t)
        if (i % hide_every) == hide_every - 1:
            # The filter gets no new snapshot (same seq), so it can only coast here.
            out = pred.step(make_state(t, [], seq, belt_row))
            est = {c.id: c.pos_mm for c in out.cubes}
            for c in cubes:
                if c.id in est and c.hits >= P.MIN_HITS:
                    truth = c.along_px * P.MM_PER_PX      # the camera's own measurement
                    errors.append((c.id, est[c.id] - truth))
        else:
            seq += 1
            pred.step(make_state(t, cubes, seq, belt_row))
    return errors


def summarize(errors):
    """(n, bias, MAE) over all cubes, or None."""
    if not errors:
        return None
    es = [e for _, e in errors]
    return len(es), sum(es) / len(es), sum(abs(e) for e in es) / len(es)


def report(errors):
    """Print bias and mean absolute error per cube."""
    by_id = {}
    for cid, e in errors:
        by_id.setdefault(cid, []).append(e)
    if not by_id:
        print("no comparisons made: no cube reached MIN_HITS while hidden")
        return
    for cid, es in sorted(by_id.items()):
        print(f"cube {cid:>3}: n={len(es):3d}  bias={sum(es)/len(es):+7.1f} mm  "
              f"MAE={sum(abs(e) for e in es)/len(es):6.1f} mm")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    try_sign = "--try-sign" in sys.argv
    if not args:
        sys.exit(__doc__)
    cam, belt = load(args[0])
    if len(args) > 1:
        belt = load(args[1])[1] or belt
    print(f"camera snapshots: {len(cam)}   belt samples: {len(belt)}   MM_PER_PX={P.MM_PER_PX}")
    if not belt:
        print("WARNING: no belt data. The belt prior is 0, so the result is not meaningful.")
    if try_sign:
        for sign in (+1, -1):
            P.BELT_SIGN = sign
            s = summarize(replay(cam, belt))
            print(f"BELT_SIGN={sign:+d}: " + (f"n={s[0]} bias={s[1]:+.1f} mm MAE={s[2]:.1f} mm" if s else "no data"))
        print("lower MAE = the sign that matches the belt. If they are equal, the prior barely matters here.")
    else:
        report(replay(cam, belt))
