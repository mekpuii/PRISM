# test_scene.py - the camera as a sensor of the situation (scene.py + its use in optimize.py):
# bad light -> camera not used, colour votes paused; a stuck cube -> brake + ALERT; cubes keep coming -> rush;
# slip warning; one case row per cycle; and an old vision (no light data) changes nothing.
# No robot, no broker, no autoen.
# Run (orch-env, from orchestrator/):  python3 test/test_scene.py     Expected last line: "all scene tests passed"
import csv
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import optimize as O                                               # noqa: E402
import scene as S                                                  # noqa: E402
import study_cases                                                 # noqa: E402
from observe import CubeObs                                        # noqa: E402
from predict import PredictOutput                                  # noqa: E402
from test_opt import cyc, view                                     # noqa: E402
from test_smart import settings, tele                              # noqa: E402

NORMAL = {"v_mean": 75.1, "s_mean": 171.3, "glare_pct": 0.01, "dark_pct": 0.14}   # measured, snap_raw.png
DARK = {"v_mean": 44.7, "s_mean": 173.3, "glare_pct": 0.0, "dark_pct": 66.88}     # lamp at 60 %
GLARE = {"v_mean": 162.9, "s_mean": 120.6, "glare_pct": 12.35, "dark_pct": 0.0}


class Cam:
    """A camera: every call is one new snapshot with these cubes (u, colour, vu) and this light."""

    def __init__(self):
        self.seq = 0

    def state(self, t, cubes=(), light=NORMAL, belt_rpm=20.0):
        self.seq += 1
        s = tele(t, camera=True)
        s.belt.rpm = belt_rpm
        s.cubes = [CubeObs(id=i, color=col, along_px=-u, across_px=200.0, along_vel_px_s=-vu, dist_m=0.59,
                           w_px=43.0, h_px=43.0, hits=10) for i, (u, col, vu) in enumerate(cubes, 1)]
        s.cam_seq, s.light = self.seq, light
        return s


def pred(t, belt=True):
    return PredictOutput(t=t, cubes=[], belt_mm_s=56.0 if belt else 0.0, belt_ok=True, gaps=[])


def run(opt, cam, t, secs, phase="TRANSPORT", cubes=(), light=NORMAL, cycles=(), **v):
    plans = []
    for i in range(int(secs / 0.5)):
        t += 0.5
        plans.append(opt.step(cam.state(t, cubes, light), pred(t), view(t, phase=phase, config=settings(opt),
                                                                         cycles=cycles if i == 0 else (), **v)))
    return t, plans


def trusted(opt):
    opt._cam_record.extend([True] * O.CAM_WINDOW)


def main():
    tmp = tempfile.mkdtemp(prefix="prism_scene_")

    # 1. Light: normal -> ok; the lamp dims to 60 % for 3 s -> "dark": the camera is not used for decisions and the
    #    colour votes stop (positions still tracked); a 1 s shadow changes nothing; light back -> trusted again.
    opt, cam = O.Optimize(), Cam()
    trusted(opt)
    t, _ = run(opt, cam, 0.0, 4)
    assert opt.scene.light == "ok" and opt.camera_trusted()
    t, _ = run(opt, cam, t, 1, light=DARK)
    assert opt.scene.light == "ok", "a 1 s shadow must not count"
    t, _ = run(opt, cam, t, 3.5, light=DARK)
    assert opt.scene.light == "dark" and not opt.camera_trusted(), opt.scene.light
    assert any("light on the belt: dark" in w for w in opt.warnings), opt.warnings
    opt.queue.entries.clear()
    t, _ = run(opt, cam, t, 3, cubes=[(560, "orange", -50)], light=DARK)
    e = opt.queue.head()
    assert e is not None and e.get("orange", 0) == 0, "voted a colour in the dark"
    t, _ = run(opt, cam, t, 4)
    assert opt.scene.light == "ok" and opt.camera_trusted() and "ok again" in opt.reason, opt.reason
    t, _ = run(opt, cam, t, 4, light=GLARE)
    assert opt.scene.light == "glare" and not opt.camera_trusted()
    print("light:     dark (lamp 60 %) / glare -> camera not used, colours not voted; a 1 s shadow ignored; "
          "back to ok -> trusted   OK")

    # 2. Stuck: a cube still at u 400 for 4 s while the belt runs -> brake (abort) + ALERT; its NO_BLOCK cycle is
    #    not taken for an empty feeder. A cube at the IR (belt stopping for it) is never "stuck".
    opt, cam = O.Optimize(), Cam()
    t, _ = run(opt, cam, 0.0, 3)
    t, plans = run(opt, cam, t, 3, cubes=[(400, "white", 0)])
    assert opt.mode == "RUN", "stuck called too early"
    t, plans = run(opt, cam, t, 2, cubes=[(400, "white", 0)])
    assert opt.mode == "ALERT" and any(p.action == "abort" for p in plans) and "stuck" in opt.reason, opt.reason
    t, _ = run(opt, cam, t, 2, phase="IDLE", cycles=[cyc(1, O.NO_BLOCK)], paused=True, status="paused")
    assert opt.mode == "ALERT" and "stuck" in opt.reason, (opt.mode, opt.reason)
    opt, cam = O.Optimize(), Cam()
    t, _ = run(opt, cam, 0.0, 10, cubes=[(260, "white", 0)])
    assert opt.mode == "RUN" and opt.scene.stuck is None, "a cube at the IR is not stuck"
    opt = O.Optimize(stuck_alert=False)
    t, _ = run(opt, Cam(), 0.0, 10, cubes=[(400, "white", 0)])
    assert opt.mode == "RUN" and opt.scene.stuck is not None
    print("stuck:     still 4 s on the running belt -> belt braked + ALERT (not 'feeder empty'); at the IR -> fine; "
          "--no-stuck-alert -> logged only   OK")

    # 3. Cubes keep coming: 2 waiting during a cycle -> rush (fastest safe setting); a clear belt -> energy first.
    opt, cam = O.Optimize(), Cam()
    trusted(opt)
    waiting = [(300, "white", -50), (420, "orange", -50), (520, "white", -50)]
    t, _ = run(opt, cam, 0.0, 3, cubes=waiting)
    t, _ = run(opt, cam, t, 0.5, phase="GRIP", cubes=waiting, cycles=[cyc(1)])
    assert opt.rush and "rush" in opt.reason, opt.reason
    assert opt.cost(opt.configs[(15.0, 1.5)]) > opt.cost(opt.configs[(20.0, 1.5)])
    t, _ = run(opt, cam, t, 3, cubes=[(270, "white", -50)])            # only the one at the IR
    t, _ = run(opt, cam, t, 1, phase="GRIP", cycles=[cyc(2)])
    assert not opt.rush and "energy first" in opt.reason, opt.reason
    print("backlog:   2 cubes waiting -> rush (speed first, safe settings only); belt clear -> energy first again   OK")

    # 4. Touching cubes and the gap; slip (cubes at 40 % of the belt speed for 3 cycles) -> one warning.
    sc = S.Scene(record=False)
    s = Cam().state(1.0, [(300, "white", -50), (342, "orange", -50)])
    sc.step(s, pred(1.0), "TRANSPORT", 1.0)
    assert sc.train and sc.min_gap_mm is not None and sc.min_gap_mm < S.TRAIN_MM, sc.min_gap_mm
    opt, cam = O.Optimize(), Cam()
    for n in range(1, 4):
        t, _ = run(opt, cam, t, 2, cubes=[(450, "white", -0.4 * 56 / S.MM_PER_PX)])
        t, _ = run(opt, cam, t, 0.5, phase="GRIP", cycles=[cyc(n)])
    assert any("slip" in w for w in opt.warnings), opt.warnings
    print(f"train:     two cubes {sc.min_gap_mm} mm apart -> touching; slip 40 % over 3 cycles -> warning   OK")

    # 5. The case array: one row per cycle, read back by study_cases.
    S.CASES_DIR = study_cases.CASES_DIR = tmp
    opt, cam = O.Optimize(scene=S.Scene(cases_dir=tmp)), Cam()
    trusted(opt)
    for n in range(1, 5):
        light = DARK if n == 4 else NORMAL
        t, _ = run(opt, cam, t, 4, cubes=[(450, "orange", -59)], light=light)
        t, _ = run(opt, cam, t, 0.5, phase="GRIP", cycles=[cyc(n, "OK" if n < 4 else "GRIP_MISS")], light=light)
    path = [f for f in os.listdir(tmp) if f.startswith("cases_")]
    rows = list(csv.DictReader(open(os.path.join(tmp, path[0]))))
    assert len(rows) == 4 and rows[0]["light"] == "ok" and rows[-1]["light"] == "dark", [r["light"] for r in rows]
    assert rows[0]["slip"] and abs(float(rows[0]["slip"]) - 1.0) < 0.05, rows[0]["slip"]
    tab = {r["situation"]: r for r in study_cases.table(rows, study_cases.GROUPS["light"])}
    assert tab["ok"]["ok_pct"] == 100 and tab["dark"]["ok_pct"] == 0, tab
    print(f"cases:     {len(rows)} rows ({len(S.COLUMNS)} columns each) -> study: light ok 100 % OK, dark 0 %   OK")

    # 6. Old vision (no light data): unknown, nothing changes.
    opt, cam = O.Optimize(), Cam()
    trusted(opt)
    t, _ = run(opt, cam, 0.0, 5, light=None)
    assert opt.scene.light == "unknown" and opt.camera_trusted() and opt.mode == "RUN"
    print("old vision: no light data -> 'unknown', behaviour as before   OK")

    print("\nall scene tests passed")


if __name__ == "__main__":
    main()
