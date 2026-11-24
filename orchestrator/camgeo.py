# camgeo.py - where things are in the camera picture, for scene.py / sortq.py / predict.py (PRISM orchestrator).
#
# All positions are observe.py's "along" values: pixels along the belt, sign-corrected by the camera's own "dir",
# so LARGER = FURTHER DOWNSTREAM whichever way the camera looks. Rules are written as "pixels upstream of the IR"
# (dist = IR_ALONG - along: > 0 before the IR, < 0 past it), so they do not care about the camera's direction.
#
# Without camera_geometry.json: the RealSense D415 numbers of 2026-10-26 (cubes moved toward smaller u, dir -1, so
# along = -u: IR at u 245 -> along -245, feeder end u 600 -> along -600, 0.95 mm/px) - exactly the behaviour before.
# camera_geometry.json (the operator's OK; the measured proposal is camera_geometry.proposed.json):
#   {"ir_along": <px>, "entry_along": <px>, "mm_per_px": <mm>, "note": "..."}
# Pixel thresholds elsewhere (AT_IR_PX, LINK_PX, STUCK_PX ...) were tuned on the D415 picture; SCALE = this camera's
# feeder-to-IR distance in px / the D415's (355 px), so they keep their size on the belt.
import json
import os

_PATH = os.environ.get("PRISM_CAMGEO") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_geometry.json")
D415 = {"ir_along": -245.0, "entry_along": -600.0, "mm_per_px": 0.95, "note": "RealSense D415, 2026-10-26"}
D415_ENTRY_DIST = 355.0

try:
    with open(_PATH) as _f:
        GEO = {**D415, **json.load(_f)}
    ACTIVE = True
except (OSError, ValueError):
    GEO, ACTIVE = dict(D415), False

IR_ALONG = float(GEO["ir_along"])
ENTRY_ALONG = float(GEO["entry_along"])
MM_PER_PX = float(GEO["mm_per_px"])
ENTRY_DIST = IR_ALONG - ENTRY_ALONG          # px from the feeder end to the IR (D415: 355)
SCALE = ENTRY_DIST / D415_ENTRY_DIST         # 1.0 for the D415


def dist(along):
    """Pixels upstream of the IR (> 0: still coming, < 0: past it)."""
    return IR_ALONG - along
