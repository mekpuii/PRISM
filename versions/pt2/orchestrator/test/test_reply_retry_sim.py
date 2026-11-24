# test_reply_retry_sim.py - run: ~/db-env/bin/python test/test_reply_retry_sim.py   (needs pyserial + paho: db-env)
# autoen WITHOUT the robot (FakeMega, --no-mqtt): the fake Mega carries out a command but LOSES its "X,OK" reply,
# like the real Mega did ~1 in a few hundred commands (2026-10-24..26: "no reply to X,CONV / X,PID / X,FEED").
# Checks: a lost reply to CONV is asked again and the cycle is OK; a lost reply to FEED is NOT re-sent (it may have
# pushed a block) and the cycle ends TimeoutError as before.
import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import test_orch_sim as T                                   # noqa: E402  (sets up sys.path, FeederMega, start_autoen)
from actuate import ApiError                                # noqa: E402

T.TMP = tempfile.mkdtemp(prefix="prism_retry_sim_")


class LossyMega(T.FeederMega):
    """Swallows the X,OK reply to the next command named in self.lose (once each)."""

    def __init__(self, *a, **k):
        self.lose, self._drop_next_ok = [], False
        super().__init__(*a, **k)

    def _cmd(self, line):
        p = line.split(",")
        if p[0] == "X" and len(p) > 1 and p[1] in self.lose:
            self.lose.remove(p[1])
            self._drop_next_ok = True
        travelling = self.arrive_at if p[:2] == ["X", "CONV"] else None
        super()._cmd(line)
        if travelling is not None:                          # like the firmware: CONV while running keeps CONV_RUN,
            self.arrive_at = travelling                     # the block keeps travelling (the fake would forget it)

    def say(self, line):
        if line == "X,OK" and self._drop_next_ok:
            self._drop_next_ok = False
            return
        super().say(line)


def main():
    fake = LossyMega(fast=8.0)
    fake.cubes_left = 20
    api = T.start_autoen(fake)
    api.post("/api/start", {"clear": True, "config": {"sets": 1000, "cycles_per_set": 1000, "pause_before_set": 0,
                                                      "enable_f": 0, "enable_b": 0, "enable_c": 0, "rpm": 20.0, "arm_speed": 1.5,
                                                      "carry_speed": 1.5, "arrive_timeout": 25.0, "enable_verify": 0,
                                                      "gap": 1.0}})

    def wait_cycles(n, timeout=120):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            st = api.state()
            if st["cycle_total"] >= n and st["cycles"] and st["cycles"][-1]["n"] >= n:
                return st
            assert st["status"] not in ("error", "stopped", "finished"), f"{st['status']} {st['message']}"
            time.sleep(0.2)
        raise AssertionError(f"no cycle {n}")

    try:
        st = wait_cycles(1)
        fake.lose.append("CONV")                            # cycle 2 (or 3): the CONV reply goes missing
        st = wait_cycles(st["cycle_total"] + 2)
        res = [c["result"] for c in st["cycles"]]
        assert not fake.lose and all(r == "OK" for r in res), res
        print(f"CONV:  reply lost once -> sent again, all cycles OK {res}   OK")
        fake.lose.append("FEED")
        st = wait_cycles(st["cycle_total"] + 2)
        res = [(c["result"], c["notes"][:24]) for c in st["cycles"]]
        assert any(r == "TimeoutError" and "X,FEED" in n for r, n in res), res
        print("FEED:  reply lost -> NOT sent again (could push two blocks), cycle TimeoutError as before   OK")
    finally:
        try:
            api.post("/api/stop")
        except ApiError:
            pass
    print("\nall reply-retry sim tests passed")


if __name__ == "__main__":
    main()
