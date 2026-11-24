#!/usr/bin/env python3
"""apply_fixes.py - apply the three pending PRISM fixes automatically.

Usage:  python3 apply_fixes.py <camera_file.py> <predict.py>
   e.g. python3 apply_fixes.py ~/Desktop/PRISM/DCAM/vision_live_tracks.py ~/Desktop/PRISM/orchestrator/predict.py

Fixes
  1. camera: add shape-gate constants (MIN_SIDE_PX / MAX_SIDE_PX / MAX_ASPECT)
  2. camera: reject non-cube blobs inside detect_objects()
  3. camera: add "hits" to the camera/perception payload (observe/predict need it)
  4. predict: MM_PER_PX = 0.95
  5. DISABLED (2026-10-24): BELT_MM_S_PER_RPM = 0.2823 was measured in the old CPR-1320 rpm units. After the
     CPR fix (11880) it is set from experiment.py calibrate instead; this script never touches it.

Safety: every file is backed up to <name>.bak_<timestamp> first, the patch is
checked with py_compile, and on any failure the original is restored.
Re-running is safe: a fix that is already present is skipped.
"""
import py_compile
import re
import shutil
import sys
import time

STAMP = time.strftime("%Y%m%d_%H%M%S")


def backup(path):
    """Copy the file aside so any fix can be undone."""
    bak = f"{path}.bak_{STAMP}"
    shutil.copy2(path, bak)
    return bak


def compiles(path):
    """True if the file is still valid Python."""
    try:
        py_compile.compile(path, doraise=True)
        return True
    except py_compile.PyCompileError as e:
        print("   syntax error:", e.msg)
        return False


def patch_camera(src):
    """Return (new_source, list of messages). Each fix is skipped if already applied."""
    msgs = []

    # --- Fix 1: constants, placed right after MAX_AREA = ... --------------------
    if "MAX_ASPECT" in src:
        msgs.append("skip  shape-gate constants (already present)")
    else:
        m = re.search(r"^MAX_AREA\s*=.*$", src, re.M)
        if not m:
            raise RuntimeError("could not find 'MAX_AREA = ...' to anchor the constants")
        block = (
            "\n\n# Shape gate: a cube seen from above is ~square and ~40 mm wide (28..65 px here).\n"
            "# Blobs outside this are reflections, the arm, or a cube cut by the ROI edge.\n"
            "MIN_SIDE_PX = 28\nMAX_SIDE_PX = 65\nMAX_ASPECT = 1.6   # max(w,h)/min(w,h)"
        )
        src = src[:m.end()] + block + src[m.end():]
        msgs.append("ok    shape-gate constants added")

    # --- Fix 2: gate inside detect_objects, after the 'w <= 0 or h <= 0' check --
    if "MIN_SIDE_PX <= w" in src:
        msgs.append("skip  shape gate in detect_objects (already present)")
    else:
        fn = re.search(r"^def detect_objects\(.*?(?=^def |\Z)", src, re.M | re.S)
        if not fn:
            raise RuntimeError("could not find detect_objects()")
        body = fn.group(0)
        pat = re.compile(r"(\n([ \t]+)if w <= 0 or h <= 0:\n[ \t]+continue[ \t]*\n)")
        m = pat.search(body)
        if not m:
            raise RuntimeError("could not find 'if w <= 0 or h <= 0: continue' in detect_objects")
        ind = m.group(2)
        gate = (
            f"\n{ind}# shape gate (see MIN_SIDE_PX / MAX_SIDE_PX / MAX_ASPECT)\n"
            f"{ind}if not (MIN_SIDE_PX <= w <= MAX_SIDE_PX and MIN_SIDE_PX <= h <= MAX_SIDE_PX):\n"
            f"{ind}    continue   # wrong size for a cube\n"
            f"{ind}if max(w, h) / min(w, h) > MAX_ASPECT:\n"
            f"{ind}    continue   # too long/thin for a cube\n"
        )
        body = body[:m.end()] + gate + body[m.end():]
        src = src[:fn.start()] + body + src[fn.end():]
        msgs.append("ok    shape gate added to detect_objects")

    # --- Fix 3: "hits" in the perception payload ---------------------------------
    pf = re.search(r"def publish_perception\(.*?(?=\n    def |\nclass |\ndef |\Z)", src, re.S)
    if not pf:
        raise RuntimeError("could not find publish_perception()")
    if '"hits"' in pf.group(0):
        msgs.append('skip  "hits" in payload (already present)')
    else:
        body = pf.group(0)
        pat = re.compile(r'(\n([ \t]+)"vv": round\(t\.vv, 1\),[ \t]*\n)')
        m = pat.search(body)
        if not m:
            raise RuntimeError('could not find the "vv" line in publish_perception')
        ind = m.group(2)
        body = body[:m.end()] + f'{ind}"hits": t.hits,\n' + body[m.end():]
        src = src[:pf.start()] + body + src[pf.end():]
        msgs.append('ok    "hits" added to payload')
    return src, msgs


def patch_predict(src):
    """MM_PER_PX = 0.95. Only the assignment changes. (BELT_MM_S_PER_RPM: fix #5 is disabled.)"""
    msgs = []
    m = re.search(r"^MM_PER_PX\s*=\s*([0-9.]+)[^\n]*$", src, re.M)
    if not m:
        raise RuntimeError("could not find 'MM_PER_PX = ...' in predict.py")
    if float(m.group(1)) == 0.95:
        msgs.append("skip  MM_PER_PX already 0.95")
    else:
        src = src[:m.start()] + "MM_PER_PX = 0.95   # w 43 px = 40 mm; depth 0.59 m. Verify with replay.py" + src[m.end():]
        msgs.append(f"ok    MM_PER_PX {m.group(1)} -> 0.95")

    msgs.append("skip  BELT_MM_S_PER_RPM: fix #5 disabled (0.2823 was old CPR-1320 units; use the calibration)")
    return src, msgs


def apply(path, patcher):
    """Backup -> patch -> compile-check -> restore on failure."""
    print(f"\n{path}")
    with open(path, encoding="utf-8") as f:
        original = f.read()
    try:
        new, msgs = patcher(original)
    except RuntimeError as e:
        print("   FAILED, file untouched:", e)
        return False
    for m in msgs:
        print("  ", m)
    if new == original:
        print("   nothing to change")
        return True
    bak = backup(path)
    with open(path, "w", encoding="utf-8") as f:
        f.write(new)
    if not compiles(path):
        shutil.copy2(bak, path)
        print("   restored original from backup")
        return False
    print(f"   written. backup: {bak}")
    return True


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    ok1 = apply(sys.argv[1], patch_camera)
    ok2 = apply(sys.argv[2], patch_predict)
    sys.exit(0 if ok1 and ok2 else 2)
