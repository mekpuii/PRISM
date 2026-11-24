# test_tuner.py - run (orch-env, from orchestrator/):  python3 test/test_tuner.py
# tuner.py: the optimizer experiments on its own flow knobs and keeps only what is measurably better.
# A made-up cell: +1 cube in flight is really faster (9 s instead of 10 s per pick); spacing 0.20 jams (1 pick in 3
# fails); the "late" limit makes no difference. The tuner must keep the first, reject the second, leave the third.
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tuner as TU                                                 # noqa: E402


def pick(tu, rng):
    secs = 10.0 + rng.uniform(-0.3, 0.3)
    if tu.current("in_flight") == 1:
        secs -= 1.0
    if tu.current("in_flight") == -1:
        secs += 2.0
    ok = not (tu.current("spacing") == 0.20 and rng.random() < 0.34)
    return secs, ok


def main():
    rng = random.Random(5)
    tu = TU.Tuner()
    words = []
    for _ in range(400):
        w = tu.record(*pick(tu, rng))
        if w:
            words.append(w)
    assert tu.champ["in_flight"] == 1, (tu.champ, words)
    assert tu.champ["spacing"] != 0.20, (tu.champ, words)
    print(f"kept:     in_flight +1 (really faster) -> champion {tu.champ}   OK")
    kept = [h for h in tu.history if h["kept"]]
    assert all(h["knob"] == "in_flight" for h in kept), kept      # noise never made it keep a neutral / worse value
    print(f"rejected: spacing 0.20 (jams) and every neutral 'late' value; {len(tu.history)} experiments, "
          f"{len(kept)} kept   OK")
    print(f"words:    {[w for w in words if w.startswith('kept')][0]}")
    print(f"          {[w for w in words if w.startswith('back to spacing') and ': 0.2 gave' in w][0]}")
    # memory: a new tuner loads the champions
    tu2 = TU.Tuner()
    tu2.load(tu.state())
    assert tu2.champ == tu.champ and tu2.picks == tu.picks, (tu2.champ, tu.champ)
    # an experiment is dropped on trouble
    tu3 = TU.Tuner()
    for _ in range(TU.WARM):
        tu3.record(10.0, True)
    assert tu3.trial is not None and tu3.abort("ALARM") and tu3.trial is None
    print("memory:   champions kept across restarts; an experiment is dropped on trouble   OK")
    print("\nall tuner tests passed")


if __name__ == "__main__":
    main()
