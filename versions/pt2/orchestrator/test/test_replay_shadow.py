# test_replay_shadow.py - roadmap C4: replay a recorded experiment session through observe -> predict -> optimize
# in shadow (nothing is sent anywhere, no broker is used). Prints every plan change and checks that on a session
# with only good cycles the optimizer never pauses, never idles and never alerts.
# Run (orch-env, from orchestrator/):
#   python3 test/test_replay_shadow.py [cal/runs/exp_..._vision_check_v1.jsonl]
#
# The session ran under experiment.py (pause_before_set = 1); the replayed AutoenView says 0, as if the
# optimizer were in charge, and takes status / phase from autoen/state and the settings from exp/config.
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ORCH = os.path.dirname(HERE)
sys.path.insert(0, ORCH)

import observe as OB                                        # noqa: E402
import optimize as O                                        # noqa: E402
from actuate import AutoenView                              # noqa: E402
from predict import Predict                                 # noqa: E402

DEFAULT = os.path.join(ORCH, "cal", "runs", "exp_20261025_232408_vision_check_v1.jsonl")
TICK_S = 0.05


class NoBroker:
    """observe.Observe without a broker: replay calls _apply() itself."""
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        return lambda *a, **k: None


def main(path=DEFAULT):
    OB.mqtt.Client = NoBroker
    obs, pred, opt = OB.Observe(), Predict(), O.Optimize()
    status, phase, cfg = "running", "", {"pause_before_set": 0, "rpm": 20.0, "arm_speed": 1.5}
    plans, actions, n_msgs, t_next = [], [], 0, None

    def tick(now):
        st = obs._snapshot(now)
        p = opt.step(st, pred.step(st), AutoenView(ok=True, t=now, status=status, paused=status == "paused",
                                                   phase=phase, config=dict(cfg)))
        if not plans or p.seq != plans[-1].seq:
            plans.append(p)
            print(f"  t={now - t0:7.1f}s  {p.mode:<6} {p.rpm:g}/{p.arm_speed:g}"
                  + (f"  action={p.action}" if p.action else "") + f"  - {p.reason}")
        if p.action:
            actions.append((now - t0, p.action, p.reason))

    print(f"replaying {os.path.basename(path)}")
    t0 = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            topic, data, mono = r.get("topic"), r.get("data") or {}, r.get("mono")
            if mono is None:
                continue
            if t0 is None:
                t0 = t_next = mono
            while t_next <= mono:                         # 20 Hz ticks in recorded time
                tick(t_next)
                t_next += TICK_S
            if topic == "autoen/state":
                status, phase = data.get("status", status), data.get("phase", phase)
            elif topic == "exp/config":
                cfg.update({k: v for k, v in (data.get("posted") or {}).items() if k in ("rpm", "arm_speed")})
            elif topic in (OB.T_ARM_TEL, OB.T_ARM_STATE, OB.T_GRIPPER, OB.T_BELT_TEL, OB.T_BELT_EVT, OB.T_CAM_PERC,
                           OB.T_CYCLE):
                obs._apply(topic, json.dumps(data).encode(), bool(r.get("retained")), mono)
                n_msgs += 1

    good = sum(c["n"] for c in opt.configs.values())
    print(f"\n{n_msgs} messages, {len(plans)} plan changes, {good} cycles counted, camera trusted: {opt.camera_trusted()} "
          f"(saw the cube in {sum(opt._cam_record)}/{len(opt._cam_record)} of the last good cycles)")
    bad = [a for a in actions if a[1] in ("pause", "idle")] + [(0, p.mode, p.reason) for p in plans if p.mode in ("ALERT", "IDLE")]
    assert not bad, f"no pause / idle / alert expected on a session of good cycles: {bad}"
    assert good >= 20, f"cycles were not seen by the optimizer ({good})"
    print("replay: all good cycles counted, no wrong pause / idle / alert   OK")


if __name__ == "__main__":
    main(*sys.argv[1:])
