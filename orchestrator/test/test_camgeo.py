# test_camgeo.py - run (orch-env, from orchestrator/):  python3 test/test_camgeo.py
# camgeo.py (2026-11-12): the scene (backlog, stuck) and the colour queue work for a camera that looks the OTHER way.
# The Astro HD camera sees the belt left -> right (dir +1, feeder exit at the left edge ~65 px, the IR at the right
# ~815 px); the D415 saw it right -> left. Each case runs in its own python with PRISM_CAMGEO set (module constants).
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ORCH = os.path.dirname(HERE)

CASE = r'''
import sys
sys.path[:0] = [ORCH, HERE]
import camgeo, scene as S, sortq as Q
from observe import CubeObs
def cube(i, along, col="white", hits=5):
    return CubeObs(id=i, color=col, along_px=along, across_px=250.0, along_vel_px_s=0.0, dist_m=None, w_px=60, h_px=45, hits=hits)
q = Q.SortQueue()
# three cubes come out of the feeder one after the other (new camera: they APPEAR at the left edge ~70 px)
q.update([cube(1, 70, "orange")]); q.update([cube(1, 75, "orange")]); q.update([cube(1, 80, "orange")])
q.update([cube(1, 300, "orange"), cube(2, 70, "white")]); q.update([cube(1, 310, "orange"), cube(2, 72, "white")])
q.update([cube(1, 320, "orange"), cube(2, 74, "white")])
order = [q.colour(e) for e in q.entries]
# the scene: one cube at the IR (815), two waiting upstream (300, 500) -> backlog 2
inside = [c for c in [cube(1, 815), cube(2, 500), cube(3, 300)] if -S.AT_IR_PX <= camgeo.dist(c.along_px) <= camgeo.ENTRY_DIST]
backlog = sum(camgeo.dist(c.along_px) > S.AT_IR_PX for c in inside)
print(order, backlog, camgeo.ACTIVE)
'''


def run(geo):
    env = dict(os.environ)
    if geo is None:
        env.pop("PRISM_CAMGEO", None)
        env["PRISM_CAMGEO"] = os.path.join(tempfile.mkdtemp(), "none.json")        # no file: the D415 defaults
    else:
        path = os.path.join(tempfile.mkdtemp(), "geo.json")
        with open(path, "w") as f:
            json.dump(geo, f)
        env["PRISM_CAMGEO"] = path
    code = f"ORCH, HERE = {ORCH!r}, {HERE!r}\n" + CASE
    out = subprocess.run([sys.executable, "-c", code], env=env, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    return out.stdout.strip()


def main():
    new = run({"ir_along": 815.0, "entry_along": 65.0, "mm_per_px": 0.7})
    assert new == "['orange', 'white'] 2 True", new
    print(f"new camera (dir +1): cubes listed in feeder order orange, white; 2 waiting before the IR   OK")
    old = run(None)
    assert old.endswith("0 False"), old
    print(f"old D415 numbers on the new picture: the backlog sees nothing ({old}) - why camgeo was needed   OK")
    print("\nall camgeo tests passed")


if __name__ == "__main__":
    main()
