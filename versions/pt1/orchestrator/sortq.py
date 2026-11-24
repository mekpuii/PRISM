# sortq.py - the order of the cubes on the belt, for colour sorting (PRISM orchestrator).
#
# THE IDEA (the operator's, 2026-10-27)
#   Cubes cannot overtake each other on a belt. So the order in which NEW cubes appear on the camera at the feeder end
#   is exactly the order in which the arm will pick them. Keep them in a list: [W, O, O, ...]. The FRONT of the list
#   is the next cube to pick, so its bin is known long before it reaches the IR. After each pick, drop the front.
#
# WHAT MAKES IT SAFE AGAINST A REAL CAMERA  (numbers from the 2026-10-26 recordings)
#   - only a track that APPEARS at the feeder end (u >= ENTRY_U, real cubes: u 540-590) is a new cube; a track that
#     starts mid-belt (7 of 47 did: the camera lost and re-found a cube) is linked to the nearest cube already listed
#   - each cube's colour is the MAJORITY over all its camera frames, across track restarts
#   - objects past the IR (u < IGNORE_U: bins, gripper) are ignored
#   - RE-SYNC at the IR: when the belt stops, the cube at the IR is the one picked; cubes listed before it were
#     phantoms and are removed, so one mistake cannot shift every bin after it
#
# Positions are observe.py's "along" values (sign-corrected: larger = further downstream; with the camera's
# dir = -1, along = -u). Thresholds are written in u for readability.

ENTRY_U = 520.0         # first seen at u >= this = a new cube from the feeder (real: 540-590)
IGNORE_U = 180.0        # past the IR (real IR ~245): bins / gripper, never a cube on the belt
LINK_PX = 90.0          # a mid-belt track this close to a listed cube is that cube, re-found
MIN_HITS = 2            # camera frames before a new track counts (first-seen hits were 1-3)
VOTE_MIN = 3            # frames with a colour before a cube's colour is trusted
RELINK_FRAMES = 3       # a feeder-end track joins the last listed cube only if that one was seen this recently


class SortQueue:
    def __init__(self, colours=("orange", "white")):
        self.colours = tuple(colours)
        self.entries = []       # oldest (= next to pick) first: {"no", "tracks", "pos", "n", <colour>: frames}
        self.track = {}         # camera track id -> its entry
        self.first = {}         # camera track id -> along position when first seen
        self.present = set()    # track ids in the latest snapshot
        self.next_no = 1
        self.snap = 0           # snapshots seen so far
        self._orphans = set()   # mid-belt tracks that matched no listed cube (counted once each)
        self.stats = {"cubes": 0, "relinked": 0, "orphans": 0, "phantoms_dropped": 0}

    # ----- input -----

    def update(self, cubes, vote=True):
        """One camera snapshot (observe.CubeObs list). vote=False: positions only (bad light: colours not counted)."""
        self.present = set()
        self.snap += 1
        for c in cubes:
            if c.color not in self.colours or -c.along_px < IGNORE_U:
                continue
            self.first.setdefault(c.id, c.along_px)
            e = self.track.get(c.id)
            if e is None:
                if c.hits < MIN_HITS:
                    continue
                if -self.first[c.id] >= ENTRY_U:
                    e = self._near(c.along_px, tail_only=True) or self._new()   # a feeder-end break-up re-links
                else:
                    e = self._near(c.along_px)
                    if e is None:
                        if c.id not in self._orphans:
                            self._orphans.add(c.id)
                            self.stats["orphans"] += 1                   # placed by hand mid-belt, or noise
                        continue
                    self.stats["relinked"] += 1
                e["tracks"].add(c.id)
                self.track[c.id] = e
            self.present.add(c.id)
            e["pos"], e["n"], e["seen"] = c.along_px, e["n"] + 1, self.snap
            if vote:
                e[c.color] = e.get(c.color, 0) + 1

    def _new(self):
        e = {"no": self.next_no, "tracks": set(), "pos": None, "n": 0, "seen": self.snap}
        self.next_no += 1
        self.entries.append(e)
        self.stats["cubes"] += 1
        return e

    def _near(self, along, tail_only=False):
        cands = self.entries[-1:] if tail_only else self.entries
        if tail_only:                           # only a break-up of a cube seen a moment ago, not a vanished one
            cands = [e for e in cands if self.snap - e["seen"] <= RELINK_FRAMES]
        best = min(cands, key=lambda e: abs(e["pos"] - along), default=None) if cands else None
        if best is not None and best["pos"] is not None and abs(best["pos"] - along) <= LINK_PX:
            return best
        return None

    # ----- questions -----

    def colour(self, e):
        """Majority colour of a cube, or None while it is too uncertain."""
        if e is None or e["n"] < VOTE_MIN:
            return None
        best = max(self.colours, key=lambda col: e.get(col, 0))
        return best if e.get(best, 0) * 2 > e["n"] else None

    def head(self):
        return self.entries[0] if self.entries else None

    def at_ir(self):
        """The listed cube nearest the IR among those the camera sees now (the belt has stopped)."""
        seen = [self.track[t] for t in self.present if t in self.track]
        return max(seen, key=lambda e: e["pos"]) if seen else None

    # ----- the pick -----

    def resync(self, e):
        """The cube at the IR is `e`: anything listed before it was a phantom. Returns how many were dropped."""
        if e not in self.entries:
            return 0
        i = self.entries.index(e)
        for gone in self.entries[:i]:
            self._forget(gone)
        self.entries = self.entries[i:]
        self.stats["phantoms_dropped"] += i
        return i

    def pop(self, e=None):
        """The arm took `e` (default: the front of the list)."""
        e = e or self.head()
        if e in self.entries:
            self.entries.remove(e)
            self._forget(e)

    def _forget(self, e):
        for t in e["tracks"]:
            self.track.pop(t, None)

    def letters(self):
        """The list as the operator thinks of it: 'O W ?' (front first; ? = colour not sure yet)."""
        return " ".join((self.colour(e) or "?")[0].upper() for e in self.entries)
