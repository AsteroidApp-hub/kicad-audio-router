"""Audio-aware grid router (pure Python).

A* on a uniform grid over the routing layers (inner plane layers excluded), 8 directions + vias.

  * occupancy: every cell holds 0 (free), net-id + 1 (copper of that net or its clearance halo),
    or BLOCK (two nets' halos overlap, board edge). A net may use free cells and its own cells.
  * a track of half-width hw needs every cell within hw of its centre line passable;
    a via needs that within its radius on every routing layer.
  * cost fields: analog nets pay for running near digital copper (weighted by activity),
    digital nets pay for running near analog copper. The second side of a differential pair pays
    for leaving the band next to the first side, so the two run together.
  * multi-pin nets grow a tree: repeatedly connect the tree to the nearest remaining pad.
  * order: differential pairs, other analog, power, digital, other. Ground nets are left to
    planes / `kar ground`. Nets that fail are reported (hand them to Freerouting).
Deterministic: no randomness, ties broken by insertion order.
"""
from __future__ import annotations

import heapq
import math
from array import array
from dataclasses import dataclass, field

from .classify import Classification
from .geometry import Seg
from .model import Board, Pad

BLOCK = 0xFFFF
SQ2 = math.sqrt(2.0)
DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]


@dataclass
class RouteParams:
    res: float = 0.25                 # grid pitch (mm)
    clearance: float = 0.2
    edge_clearance: float = 0.4
    via_d: float = 0.6
    via_drill: float = 0.3
    via_cost: float = 12.0            # in grid steps
    analog_via_cost: float = 30.0
    bend_cost: float = 0.4
    width: dict = field(default_factory=lambda: {"analog": 0.25, "power": 0.5, "digital": 0.2, "other": 0.25,
                                                 "gnd": 0.4})
    narrow_width: dict = field(default_factory=lambda: {"power": 0.3, "gnd": 0.25})   # where the wide one fails
    near_mm: float = 3.0              # reach of the proximity cost fields
    near_cost: float = 6.0            # extra cost per step right next to the other kind (falls off linearly)
    near_min_weight: float = 0.0      # routing: digital nets avoid analog at least this much, however static
    pair_gap: float = 0.25            # target edge gap between the two sides of a pair
    pair_cost: float = 1.5            # extra cost per step for the second side away from the first
    pair_retry_below: float = 0.5     # if the sides run together less than this, try other orders/costs
    pair_room_cost: float = 0.0       # first side of a pair pays for crowded corridors (off: no gain on test boards)
    window_mm: float = 12.0           # search window around the net, widened once on failure
    rip_rounds: int = 3               # rip-up and reroute rounds for nets that failed
    fine_fallback: bool = True        # nets that still fail are tried again on a grid twice as fine
    phase: tuple[float, float] | None = None   # grid offset; default: aligned to most fine-pitch pads
    window: tuple[float, float, float, float] | None = None   # route only inside this box (mm)
    net_width: dict = field(default_factory=dict)   # net -> track width from its net class (project file)
    net_via: dict = field(default_factory=dict)     # net -> (via diameter, drill) from its net class

    def full_width(self, net: str, kind: str) -> float:
        return self.net_width.get(net) or self.width.get(kind, 0.25)

    def widths_for(self, net: str, kind: str) -> list[float]:
        full = self.full_width(net, kind)
        nw = self.narrow_width.get(kind)
        return [full] + ([nw] if nw and nw < full - 1e-9 else [])

    def via_of(self, net: str) -> tuple[float, float]:
        return self.net_via.get(net, (self.via_d, self.via_drill))
    max_expand: int = 400_000         # node budget per search (fail fast instead of flooding the board)
    near_targets: int = 6             # heuristic looks at this many nearest remaining pads
    rip_radius: float = 2.0           # routed nets passing this close to a failed net's pads are ripped up
    rip_time_s: float = 300.0         # time budget for rip-up and reroute
    push_cost: float = 20.0           # per step through another net's track when pushing (digital/other)
    push_cost_analog: float = 60.0    # ... through an analog track
    push_limit: int = 4               # how many times one net may push others
    push_total: int = 0               # pushes in all (0: 3 x the nets that failed, at least 20)
    widen_push_s: float = 60.0        # time for re-routing narrowed power nets wide by pushing others
    widen_push_victims: int = 6       # ... giving up on a net if it would move more routes than this
    local_rip: bool = False           # also try ripping up everything near a failed net's pads first
    h_weight: dict = field(default_factory=lambda: {"analog": 1.0, "power": 1.5, "digital": 1.2, "other": 1.5,
                                                    "gnd": 1.5})
    # >1 makes A* greedier (much faster on long nets, paths slightly longer than optimal)
    layers: list[str] | None = None   # routing layers (default: copper minus plane layers)
    classes: tuple[str, ...] = ("analog", "power", "digital", "other")


@dataclass
class RoutedNet:
    net: str
    segs: list[Seg]
    vias: list[tuple[float, float]]
    width: float


@dataclass
class RouteResult:
    routed: list[RoutedNet]
    failed: list[str]
    skipped: list[str]
    stats: dict = field(default_factory=dict)      # seconds per phase, pushes, reroutes


def _grid_phase(board: Board, res: float, nets=None) -> tuple[float, float]:
    """Grid offset that puts as many fine-pitch pad centres as possible exactly on grid points
    (only pads of `nets` when given)."""
    from collections import Counter
    pads = [p for fp in board.footprints for p in fp.pads if p.smd and (nets is None or p.net in nets)]
    fine = [p for p in pads if min(p.box()[2] - p.box()[0], p.box()[3] - p.box()[1]) < 0.45] or pads
    if not fine:
        return 0.0, 0.0
    q = 0.005
    cx = Counter(round((p.x % res) / q) * q for p in fine)
    cy = Counter(round((p.y % res) / q) * q for p in fine)
    return cx.most_common(1)[0][0], cy.most_common(1)[0][0]


class Copper:
    """Copper objects bucketed by layer and 1 mm cell; each object is put in every bucket its
    clearance reach touches, so a query looks at one bucket only."""

    def __init__(self, reach: float, cell: float = 1.0):
        self.reach, self.cell = reach, cell
        self.b: dict[tuple[int, int, int], list] = {}

    def add(self, li: int, obj, bbox):
        c, r = self.cell, self.reach
        for i in range(math.floor((bbox[0] - r) / c), math.floor((bbox[2] + r) / c) + 1):
            for j in range(math.floor((bbox[1] - r) / c), math.floor((bbox[3] + r) / c) + 1):
                self.b.setdefault((li, i, j), []).append(obj)

    def at(self, li: int, x: float, y: float):
        return self.b.get((li, math.floor(x / self.cell), math.floor(y / self.cell)), ())

    def clone(self) -> "Copper":
        c = Copper(self.reach, self.cell)
        c.b = {k: list(v) for k, v in self.b.items()}
        return c


def _obj_dist(obj, x: float, y: float) -> float:
    """Distance from a point to an object's copper. obj = (kind, net_id, fixed, data)."""
    k, _, _, d = obj
    if k == "S":
        x1, y1, x2, y2, hw = d
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - x1) * vx + (y - y1) * vy) / l2))
        return math.hypot(x1 + t * vx - x, y1 + t * vy - y) - hw
    if k == "V":
        return math.hypot(d[0] - x, d[1] - y) - d[2]
    return d.dist(x, y)          # pad (real outline)


class Grid:
    def __init__(self, board: Board, prm: RouteParams):
        self.prm = prm
        self.res = prm.res
        xs = [x for poly in board.outline for x, _ in poly] or [p.x for fp in board.footprints for p in fp.pads]
        ys = [y for poly in board.outline for _, y in poly] or [p.y for fp in board.footprints for p in fp.pads]
        if prm.window:
            wx0, wy0, wx1, wy1 = prm.window
            xs = [max(min(xs), wx0) + 1.0, min(max(xs), wx1) - 1.0]
            ys = [max(min(ys), wy0) + 1.0, min(max(ys), wy1) - 1.0]
        rx, ry = prm.phase if prm.phase is not None else _grid_phase(board, self.res)
        self.ox = math.floor((min(xs) - 1.0 - rx) / self.res) * self.res + rx
        self.oy = math.floor((min(ys) - 1.0 - ry) / self.res) * self.res + ry
        self.W = int(math.ceil((max(xs) + 1.0 - self.ox) / self.res)) + 1
        self.H = int(math.ceil((max(ys) + 1.0 - self.oy) / self.res)) + 1
        self.layers = prm.layers or [l for l in board.copper_layers if l not in board.plane_layers]
        self.L = len(self.layers)
        self.lidx = {n: i for i, n in enumerate(self.layers)}
        n = self.W * self.H
        self.occ = [array("H", bytes(2 * n)) for _ in range(self.L)]     # copper owner (net id) / BLOCK
        self.band = [bytearray(n) for _ in range(self.L)]                # near some copper: check exactly
        self.near_dig = [array("f", bytes(4 * n)) for _ in range(self.L)]
        self.near_ana = [array("f", bytes(4 * n)) for _ in range(self.L)]
        self.novia = bytearray(n)
        self.net_id: dict[str, int] = {}
        self.clr: dict[int, float] = {}                                  # net id -> class clearance
        self.margin = self.res * 0.8
        cmax = max([prm.clearance] + list(board.net_clearance.values()))
        hwmax = max(list(prm.width.values()) + list(prm.net_width.values()) + [prm.via_d]
                    + [v[0] for v in prm.net_via.values()]) / 2
        self.reach = math.sqrt((cmax + hwmax) ** 2 + 0.5 * self.res ** 2) + 0.05
        self.cu = Copper(self.reach)

    def clone(self) -> "Grid":
        """Copy with fresh occupancy / band / copper index sharing ids, fixed map and cost fields."""
        import copy
        g = copy.copy(self)
        g.occ = [array("H", o) for o in self.occ]
        g.band = [bytearray(b) for b in self.band]
        g.cu = self.cu.clone()
        return g

    # ---------------------------------------------------------------- coordinates
    def cell(self, x: float, y: float) -> tuple[int, int]:
        return int(round((x - self.ox) / self.res)), int(round((y - self.oy) / self.res))

    def pos(self, cx: int, cy: int) -> tuple[float, float]:
        return self.ox + cx * self.res, self.oy + cy * self.res

    def nid(self, net: str) -> int:
        if net not in self.net_id:
            self.net_id[net] = len(self.net_id) + 1
        return self.net_id[net]

    # ---------------------------------------------------------------- rasterising
    def _stamp(self, li: int, cx: int, cy: int, val: int):
        if 0 <= cx < self.W and 0 <= cy < self.H:
            o = self.occ[li]
            i = cy * self.W + cx
            cur = o[i]
            if cur == 0 or cur == val:
                o[i] = val
            elif val == BLOCK or cur != BLOCK:
                o[i] = BLOCK

    def _force(self, li: int, cx: int, cy: int, val: int):
        if 0 <= cx < self.W and 0 <= cy < self.H:
            self.occ[li][cy * self.W + cx] = val

    def stamp_box(self, li, box, val, grow, force=False):
        x0, y0 = self.cell(box[0] - grow, box[1] - grow)
        x1, y1 = self.cell(box[2] + grow, box[3] + grow)
        f = self._force if force else self._stamp
        for cy in range(y0, y1 + 1):
            for cx in range(x0, x1 + 1):
                if grow > 0:
                    x, y = self.pos(cx, cy)
                    dx = max(box[0] - x, 0.0, x - box[2])
                    dy = max(box[1] - y, 0.0, y - box[3])
                    if dx * dx + dy * dy > grow * grow:
                        continue
                f(li, cx, cy, val)

    def stamp_pad(self, li, pad, val, grow, force=False):
        """Stamp the pad copper (grow=0) or its halo, using the real pad outline when known."""
        b = pad.box()
        x0, y0 = self.cell(b[0] - grow, b[1] - grow)
        x1, y1 = self.cell(b[2] + grow, b[3] + grow)
        f = self._force if force else self._stamp
        for cy in range(y0, y1 + 1):
            for cx in range(x0, x1 + 1):
                x, y = self.pos(cx, cy)
                if pad.dist(x, y) <= grow + 1e-9:
                    f(li, cx, cy, val)

    def stamp_capsule(self, li, x1, y1, x2, y2, r, val, force=False):
        f = self._force if force else self._stamp
        gx0, gy0 = self.cell(min(x1, x2) - r, min(y1, y2) - r)
        gx1, gy1 = self.cell(max(x1, x2) + r, max(y1, y2) + r)
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        for cy in range(gy0, gy1 + 1):
            for cx in range(gx0, gx1 + 1):
                x, y = self.pos(cx, cy)
                t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - x1) * vx + (y - y1) * vy) / l2))
                if (x1 + t * vx - x) ** 2 + (y1 + t * vy - y) ** 2 <= r * r:
                    f(li, cx, cy, val)

    def field_capsule(self, fld, x1, y1, x2, y2, reach, weight):
        gx0, gy0 = self.cell(min(x1, x2) - reach, min(y1, y2) - reach)
        gx1, gy1 = self.cell(max(x1, x2) + reach, max(y1, y2) + reach)
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        W, H = self.W, self.H
        for cy in range(max(gy0, 0), min(gy1, H - 1) + 1):
            for cx in range(max(gx0, 0), min(gx1, W - 1) + 1):
                x, y = self.pos(cx, cy)
                t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - x1) * vx + (y - y1) * vy) / l2))
                d = math.hypot(x1 + t * vx - x, y1 + t * vy - y)
                if d < reach:
                    v = weight * (1.0 - d / reach)
                    i = cy * W + cx
                    if v > fld[i]:
                        fld[i] = v

    def field_ring(self, fld, x1, y1, x2, y2, centre, falloff, weight=1.0):
        """Field that peaks at distance `centre` from the segment and falls to 0 at centre +- falloff."""
        reach = centre + falloff
        gx0, gy0 = self.cell(min(x1, x2) - reach, min(y1, y2) - reach)
        gx1, gy1 = self.cell(max(x1, x2) + reach, max(y1, y2) + reach)
        vx, vy = x2 - x1, y2 - y1
        l2 = vx * vx + vy * vy
        W, H = self.W, self.H
        for cy in range(max(gy0, 0), min(gy1, H - 1) + 1):
            for cx in range(max(gx0, 0), min(gx1, W - 1) + 1):
                x, y = self.pos(cx, cy)
                t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - x1) * vx + (y - y1) * vy) / l2))
                d = math.hypot(x1 + t * vx - x, y1 + t * vy - y)
                v = weight * (1.0 - abs(d - centre) / falloff)
                if v > 0:
                    i = cy * W + cx
                    if v > fld[i]:
                        fld[i] = v

    def block_outside(self, outline):
        """BLOCK cells outside the board and within edge clearance of it."""
        if not outline:
            return
        for cy in range(self.H):
            _, y = self.pos(0, cy)
            xs = []
            for poly in outline:
                n = len(poly)
                for i in range(n):
                    (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
                    if (ay > y) != (by > y):
                        xs.append(ax + (y - ay) * (bx - ax) / (by - ay))
            xs.sort()
            inside = [False] * self.W
            for k in range(0, len(xs) - 1, 2):
                c0 = int(math.ceil((xs[k] - self.ox) / self.res))
                c1 = int(math.floor((xs[k + 1] - self.ox) / self.res))
                for cx in range(max(c0, 0), min(c1, self.W - 1) + 1):
                    inside[cx] = True
            for li in range(self.L):
                o = self.occ[li]
                base = cy * self.W
                for cx in range(self.W):
                    if not inside[cx]:
                        o[base + cx] = BLOCK
        r = self.prm.edge_clearance + self.margin
        for poly in outline:
            n = len(poly)
            for i in range(n):
                (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
                for li in range(self.L):
                    self.stamp_capsule(li, ax, ay, bx, by, r, BLOCK, force=True)

    # ---------------------------------------------------------------- queries
    def disk(self, radius_mm: float) -> list[tuple[int, int]]:
        r = radius_mm / self.res
        k = int(math.floor(r))
        return [(dx, dy) for dx in range(-k, k + 1) for dy in range(-k, k + 1) if dx * dx + dy * dy <= r * r + 1e-9]


def _pad_layers(grid: Grid, p: Pad) -> list[int]:
    return [grid.lidx[l] for l in p.layers if l in grid.lidx]


def _stamp_copper(g: Grid, li: int, val: int, inside, bbox, fixed: bool, obj):
    """Mark copper cells (owner), the band within reach, and index the object."""
    o, band, W, H = g.occ[li], g.band[li], g.W, g.H
    r = g.reach
    x0, y0 = g.cell(bbox[0] - r, bbox[1] - r)
    x1, y1 = g.cell(bbox[2] + r, bbox[3] + r)
    for cy in range(max(y0, 0), min(y1, H - 1) + 1):
        for cx in range(max(x0, 0), min(x1, W - 1) + 1):
            x, y = g.pos(cx, cy)
            d = inside(x, y)
            if d < r:
                i = cy * W + cx
                band[i] = 1
                if d <= 1e-9:
                    cur = o[i]
                    o[i] = val if cur in (0, val) else BLOCK
    g.cu.add(li, obj, bbox)


def _add_pad(g: Grid, p: Pad, fixed: bool = True):
    v = g.nid(p.net) if p.net else BLOCK
    obj = ("P", v, fixed, p)
    for li in _pad_layers(g, p):
        _stamp_copper(g, li, v, p.dist, p.box(), fixed, obj)


def _add_seg(g: Grid, li: int, s: Seg, fixed: bool):
    v = g.nid(s.net)
    hw = s.width / 2
    obj = ("S", v, fixed, (s.x1, s.y1, s.x2, s.y2, hw))
    _stamp_copper(g, li, v, lambda x, y: _obj_dist(obj, x, y), s.bbox(hw), fixed, obj)


def _add_via(g: Grid, x: float, y: float, d: float, net: str, fixed: bool):
    v = g.nid(net)
    obj = ("V", v, fixed, (x, y, d / 2))
    for li in range(g.L):
        _stamp_copper(g, li, v, lambda px, py: math.hypot(px - x, py - y) - d / 2,
                      (x - d / 2, y - d / 2, x + d / 2, y + d / 2), fixed, obj)


def _grid_box(g: Grid) -> tuple[float, float, float, float]:
    r = g.reach + 1.0
    x1, y1 = g.pos(g.W - 1, g.H - 1)
    return (g.ox - r, g.oy - r, x1 + r, y1 + r)


def _touches(b, w) -> bool:
    return b[0] <= w[2] and b[2] >= w[0] and b[1] <= w[3] and b[3] >= w[1]


def build_grid(board: Board, cls: Classification, prm: RouteParams, weights: dict) -> Grid:
    g = Grid(board, prm)
    g.ncl = board.net_clearance
    pads = [p for fp in board.footprints for p in fp.pads]
    win = _grid_box(g)
    for p in pads:
        if _touches(p.box(), win):
            _add_pad(g, p)
    for s in board.segs:
        if s.layer in g.lidx and _touches(s.bbox(s.width / 2), win):
            _add_seg(g, g.lidx[s.layer], s, True)
    for v in board.vias:
        if _touches((v.x, v.y, v.x, v.y), win):
            _add_via(g, v.x, v.y, v.diameter, v.net, True)
    for n, i in g.net_id.items():
        g.clr[i] = board.clearance(n, default=prm.clearance)
    g.clr[BLOCK] = board.clearance("", default=prm.clearance)     # pads without a net (holes)
    g.block_outside(board.outline)
    # rule areas (keep-outs)
    from .geometry import point_in_poly
    for ko in board.keepouts:
        xs = [q[0] for q in ko["poly"]]
        ys = [q[1] for q in ko["poly"]]
        x0, y0 = g.cell(min(xs), min(ys))
        x1, y1 = g.cell(max(xs), max(ys))
        for cy in range(max(y0, 0), min(y1, g.H - 1) + 1):
            for cx in range(max(x0, 0), min(x1, g.W - 1) + 1):
                if point_in_poly(*g.pos(cx, cy), ko["poly"]):
                    i = cy * g.W + cx
                    if ko["no_vias"]:
                        g.novia[i] = 1
                    if ko["no_tracks"]:
                        for l in ko["layers"]:
                            if l in g.lidx:
                                g.occ[g.lidx[l]][i] = BLOCK
    g.fixed = [array("H", o) for o in g.occ]      # pads, existing copper, edges: never pushed aside
    # proximity fields from pads and existing copper
    reach = prm.near_mm
    for p in pads:
        k = cls.kind.get(p.net)
        b = p.box()
        cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        rr = reach + max(b[2] - b[0], b[3] - b[1]) / 2
        for li in _pad_layers(g, p):
            if k == "digital":
                g.field_capsule(g.near_dig[li], cx, cy, cx, cy, rr, weights.get(cls.activity.get(p.net, "data"), 1.0))
            elif k == "analog":
                g.field_capsule(g.near_ana[li], cx, cy, cx, cy, rr, 1.0)
    for s in board.segs:
        if s.layer not in g.lidx:
            continue
        k = cls.kind.get(s.net)
        if k == "digital":
            g.field_capsule(g.near_dig[g.lidx[s.layer]], s.x1, s.y1, s.x2, s.y2, reach,
                            weights.get(cls.activity.get(s.net, "data"), 1.0))
        elif k == "analog":
            g.field_capsule(g.near_ana[g.lidx[s.layer]], s.x1, s.y1, s.x2, s.y2, reach, 1.0)
    return g


OK, FIXED = 0, -1


class _Search:
    """One A* search for one net between its current tree and the remaining pads.

    Clearance is checked exactly: a cell near copper (the band) is compared with the real shapes
    around it. The check keeps sqrt(need^2 + res^2 / 2) from other copper, so the straight run
    between two neighbouring cells (whose middle is up to 0.71 res away from both ends) also keeps
    `need` = class clearance + half the track width."""

    def __init__(self, g: Grid, net: str, kind: str, width: float, prm: RouteParams,
                 pair_field: list[array] | None, act_w: float, push: dict[int, float] | None = None):
        self.g, self.net, self.kind, self.prm = g, net, kind, prm
        self.push = push           # net id -> cost per step to cross that net's routed track (None: hard)
        self.hw = prm.h_weight.get(kind, 1.0)      # A* greediness
        self.half = width / 2
        self.val = g.nid(net)
        self.own_cl = max(prm.clearance, getattr(g, "ncl", {}).get(net, 0.0))
        self.via_r = prm.via_of(net)[0] / 2
        self.via_cost = prm.analog_via_cost if kind == "analog" else prm.via_cost
        self.pair_field = pair_field
        if kind == "analog":
            self.field, self.fw = g.near_dig, prm.near_cost
        elif kind == "digital":
            self.field, self.fw = g.near_ana, prm.near_cost * max(act_w, prm.near_min_weight)
        else:
            self.field, self.fw = None, 0.0
        self.room = None                     # set for the first side of a pair: cost of crowded cells
        self._room: dict[int, float] = {}
        # check caches: byte per cell, 0 unknown, 1 OK, 2 FIXED, 3 crossing routed nets (set kept in a dict)
        self._tc = bytearray(g.L * g.W * g.H)
        self._vc = bytearray(g.W * g.H)
        self._tset: dict[int, frozenset] = {}
        self._vset: dict[int, frozenset] = {}
        self._need: dict[tuple[int, float], float] = {}

    def _need_e(self, nid: int, half: float) -> float:
        key = (nid, half)
        r = self._need.get(key)
        if r is None:
            c = max(self.own_cl, self.g.clr.get(nid, self.prm.clearance)) + half
            r = math.sqrt(c * c + 0.5 * self.g.res ** 2)
            self._need[key] = r
        return r

    def _check(self, li: int, cx: int, cy: int, half: float, step_err: bool = True):
        g = self.g
        i = cy * g.W + cx
        c = g.occ[li][i]
        v = self.val
        if c == BLOCK:
            return FIXED
        viol = None
        if c != 0 and c != v:
            if g.fixed[li][i] == c:
                return FIXED
            viol = {c}
        if not g.band[li][i]:
            return frozenset(viol) if viol else OK
        x, y = g.pos(cx, cy)
        for obj in g.cu.at(li, x, y):
            nid = obj[1]
            if nid == v:
                continue
            need = self._need_e(nid, half) if step_err else \
                max(self.own_cl, g.clr.get(nid, self.prm.clearance)) + half
            if obj[0] == "P":
                b = obj[3].box()
                dx = max(b[0] - x, 0.0, x - b[2])
                dy = max(b[1] - y, 0.0, y - b[3])
                if dx * dx + dy * dy >= need * need:
                    continue
            if _obj_dist(obj, x, y) < need + 0.005:     # 5 um margin for arc approximation
                if obj[2]:
                    return FIXED
                if viol is None:
                    viol = set()
                viol.add(nid)
        return frozenset(viol) if viol else OK

    def crowd(self, li: int, cx: int, cy: int) -> float:
        """Share of cells around (within one pair pitch) that hold other copper: 0 open .. 1 walled in."""
        key = (li * self.g.H + cy) * self.g.W + cx
        r = self._room.get(key)
        if r is None:
            g = self.g
            o, W, H, v = g.occ[li], g.W, g.H, self.val
            k = self._room_k
            n = bad = 0
            for dx in range(-k, k + 1):
                for dy in range(-k, k + 1):
                    if dx * dx + dy * dy > k * k:
                        continue
                    x, y = cx + dx, cy + dy
                    n += 1
                    if not (0 <= x < W and 0 <= y < H):
                        bad += 1
                        continue
                    c = o[y * W + x]
                    if c != 0 and c != v:
                        bad += 1
            r = bad / n
            self._room[key] = r
        return r

    def track_state(self, li: int, cx: int, cy: int):
        key = (li * self.g.H + cy) * self.g.W + cx
        c = self._tc[key]
        if c == 1:
            return OK
        if c == 2:
            return FIXED
        if c == 3:
            return self._tset[key]
        r = self._check(li, cx, cy, self.half)
        if r is OK:
            self._tc[key] = 1
        elif r is FIXED:
            self._tc[key] = 2
        else:
            self._tc[key] = 3
            self._tset[key] = r
        return r

    def via_state(self, cx: int, cy: int):
        g = self.g
        key = cy * g.W + cx
        c = self._vc[key]
        if c == 1:
            return OK
        if c == 2:
            return FIXED
        if c == 3:
            return self._vset[key]
        r = OK
        # keep-outs: no via whose copper reaches a no-via cell
        k = int(math.ceil(self.via_r / g.res))
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                x, y = cx + dx, cy + dy
                if not (0 <= x < g.W and 0 <= y < g.H) or (g.novia[y * g.W + x] and dx * dx + dy * dy <= k * k):
                    r = FIXED
        if r is OK:
            acc = set()
            for li in range(g.L):
                st = self._check(li, cx, cy, self.via_r, step_err=False)
                if st is FIXED:
                    r = FIXED
                    break
                if st is not OK:
                    acc |= st
            if r is OK and acc:
                r = frozenset(acc)
        if r is OK:
            self._vc[key] = 1
        elif r is FIXED:
            self._vc[key] = 2
        else:
            self._vc[key] = 3
            self._vset[key] = r
        return r

    def ok(self, li: int, cx: int, cy: int) -> bool:
        return self.track_state(li, cx, cy) is OK

    def via_ok(self, cx: int, cy: int) -> bool:
        return self.via_state(cx, cy) is OK

    def _cost(self, st) -> float | None:
        if st is OK:
            return 0.0
        if st is FIXED or self.push is None:
            return None
        return max(self.push.get(n, self.prm.push_cost_analog) for n in st)

    def push_step(self, li, cx, cy) -> float | None:
        return self._cost(self.track_state(li, cx, cy))

    def push_via(self, cx, cy) -> float | None:
        return self._cost(self.via_state(cx, cy))

    def run(self, sources: list[tuple[int, int, int]], targets: set[tuple[int, int, int]],
            tcenters: list[tuple[float, float]], window) -> list[tuple[int, int, int]] | None:
        """A* with integer node ids (layer, x, y) -> (layer * H + y) * W + x and flat caches."""
        g, prm = self.g, self.prm
        W, H, L = g.W, g.H, g.L
        WH = W * H
        wx0, wy0, wx1, wy1 = window
        hw = self.hw
        sq2m1 = SQ2 - 1
        tc = tcenters
        push = self.push
        fields = self.field
        fw = self.fw
        pfs = self.pair_field
        pair_cost = prm.pair_cost
        room = self.room
        bend = prm.bend_cost
        via_cost = self.via_cost
        tgt = {(li * H + y) * W + x for li, x, y in targets}
        track_state = self.track_state
        tcache = self._tc
        vcache = self._vc

        def h(x, y):
            best = 1e18
            for tx, ty in tc:
                dx = x - tx if x > tx else tx - x
                dy = y - ty if y > ty else ty - y
                d = dx + sq2m1 * dy if dx > dy else dy + sq2m1 * dx
                if d < best:
                    best = d
            return best * hw

        def cell_ok(nid, li, x, y):
            return track_state(li, x, y) is OK

        gcost: dict[int, float] = {}
        came: dict[int, int] = {}
        cdir: dict[int, int] = {}
        heap = []
        cnt = 0
        for li, x, y in sources:
            nid = (li * H + y) * W + x
            gcost[nid] = 0.0
            came[nid] = -1
            cdir[nid] = -1
            heap.append((h(x, y), cnt, nid))
            cnt += 1
        heapq.heapify(heap)
        closed = bytearray(L * WH)
        # pushing searches cross the board; finer grids have more cells per mm
        budget = int(prm.max_expand * (4 if push is not None else 1) * max(1.0, (0.25 / g.res) ** 2))
        pop, push_ = heapq.heappop, heapq.heappush
        found = -1
        while heap:
            _, _, cur = pop(heap)
            if closed[cur]:
                continue
            if cur in tgt:
                found = cur
                break
            budget -= 1
            if budget < 0:
                return None
            closed[cur] = 1
            li, rem = divmod(cur, WH)
            cy, cx = divmod(rem, W)
            gc = gcost[cur]
            pdir = cdir[cur]
            fld = fields[li] if fields is not None else None
            pf = pfs[li] if pfs is not None else None
            base = li * WH
            for d in range(8):
                dx, dy, step = DIRS[d]
                nx, ny = cx + dx, cy + dy
                if nx < wx0 or nx > wx1 or ny < wy0 or ny > wy1:
                    continue
                i = ny * W + nx
                nid = base + i
                if closed[nid]:
                    continue
                c = step
                tcv = tcache[nid]
                if tcv == 0:
                    track_state(li, nx, ny)
                    tcv = tcache[nid]
                if tcv != 1:
                    if push is None:
                        continue
                    pc = self.push_step(li, nx, ny)
                    if pc is None:
                        continue
                    c += pc * step
                if fld is not None:
                    c += step * fw * fld[i]
                if pf is not None:
                    c += step * pair_cost * (1.0 - pf[i])
                if room:
                    c += step * room * self.crowd(li, nx, ny)
                if pdir >= 0 and pdir != d:
                    c += bend
                ng = gc + c
                if ng < gcost.get(nid, 1e18):
                    gcost[nid] = ng
                    came[nid] = cur
                    cdir[nid] = d
                    push_(heap, (ng + h(nx, ny), cnt, nid))
                    cnt += 1
            if L > 1:
                vcv = vcache[rem]
                if vcv == 1:
                    vs = OK
                elif vcv == 2 and push is None:
                    continue
                else:
                    vs = self.via_state(cx, cy)
                if vs is OK:
                    vextra = 0.0
                else:
                    pv = self._cost(vs)
                    if pv is None:
                        continue
                    vextra = pv * 3
                hh = h(cx, cy)
                for lj in range(L):
                    if lj == li:
                        continue
                    nid = lj * WH + rem
                    if closed[nid]:
                        continue
                    ng = gc + via_cost + vextra
                    if ng < gcost.get(nid, 1e18):
                        gcost[nid] = ng
                        came[nid] = cur
                        cdir[nid] = -1
                        push_(heap, (ng + hh, cnt, nid))
                        cnt += 1
        if found < 0:
            return None
        path = []
        n = found
        while n >= 0:
            li, rem = divmod(n, WH)
            y, x = divmod(rem, W)
            path.append((li, x, y))
            n = came[n]
        return path[::-1]


def _pad_cells(g: Grid, p: Pad) -> set[tuple[int, int, int]]:
    """Grid cells on the pad copper (a track ending there is connected)."""
    out = set()
    b = p.box()
    x0, y0 = g.cell(b[0], b[1])
    x1, y1 = g.cell(b[2], b[3])
    cells = [(cx, cy) for cy in range(y0, y1 + 1) for cx in range(x0, x1 + 1)
             if p.contains(*g.pos(cx, cy), inset=0.02)]
    if not cells:
        cells = [g.cell(p.x, p.y)]
    for li in _pad_layers(g, p):
        for cx, cy in cells:
            out.add((li, cx, cy))
    return out


def _path_to_segs(g: Grid, path, net: str, width: float) -> tuple[list[Seg], list[tuple[float, float]]]:
    segs, vias = [], []
    run = [path[0]]

    def flush(run):
        if len(run) < 2:
            return
        pts = [run[0]]
        for k in range(1, len(run) - 1):
            a, b, c = pts[-1], run[k], run[k + 1]
            if (b[1] - a[1]) * (c[2] - b[2]) != (b[2] - a[2]) * (c[1] - b[1]):
                pts.append(b)
        pts.append(run[-1])
        for a, b in zip(pts, pts[1:]):
            x1, y1 = g.pos(a[1], a[2])
            x2, y2 = g.pos(b[1], b[2])
            segs.append(Seg(round(x1, 4), round(y1, 4), round(x2, 4), round(y2, 4), width, g.layers[a[0]], net))

    for n in path[1:]:
        if n[0] != run[-1][0]:
            flush(run)
            vias.append(g.pos(n[1], n[2]))
            run = [n]
        else:
            run.append(n)
    flush(run)
    return segs, vias


def _commit(g: Grid, net: str, kind: str, segs: list[Seg], vias, prm: RouteParams, fields: bool = True,
            fixed: bool = False):
    win = _grid_box(g) if prm.window else None
    for s in segs:
        if win and not _touches(s.bbox(s.width / 2), win):
            continue
        li = g.lidx[s.layer]
        _add_seg(g, li, s, fixed)
        if kind == "analog" and fields:
            g.field_capsule(g.near_ana[li], s.x1, s.y1, s.x2, s.y2, prm.near_mm, 1.0)
    for x, y in vias:
        if win and not _touches((x, y, x, y), win):
            continue
        _add_via(g, x, y, prm.via_of(net)[0], net, fixed)
    nid = g.nid(net)
    if nid not in g.clr:
        g.clr[nid] = max(prm.clearance, getattr(g, "ncl", {}).get(net, 0.0))


def _uncommit(g: Grid, net: str, rn: RoutedNet, prm: RouteParams):
    """Take a routed net's tracks and vias out of the grid: drop its objects from the copper index and
    recompute owner / band for the cells they could have touched."""
    nid = g.nid(net)
    gone_s = {(s.x1, s.y1, s.x2, s.y2, s.width / 2) for s in rn.segs}
    gone_v = {(x, y, prm.via_of(net)[0] / 2) for x, y in rn.vias}
    regions = []   # (layer, bbox)
    for s in rn.segs:
        regions.append((g.lidx[s.layer], s.bbox(s.width / 2)))
    for x, y in rn.vias:
        r = prm.via_of(net)[0] / 2
        for li in range(g.L):
            regions.append((li, (x - r, y - r, x + r, y + r)))

    def doomed(o):
        return o[1] == nid and not o[2] and ((o[0] == "S" and o[3] in gone_s) or (o[0] == "V" and o[3] in gone_v))
    c, R = g.cu.cell, g.cu.reach
    for li, bb in regions:
        for i in range(math.floor((bb[0] - R) / c), math.floor((bb[2] + R) / c) + 1):
            for j in range(math.floor((bb[1] - R) / c), math.floor((bb[3] + R) / c) + 1):
                lst = g.cu.b.get((li, i, j))
                if lst:
                    g.cu.b[(li, i, j)] = [o for o in lst if not doomed(o)]
    W, H = g.W, g.H
    seen = set()
    for li, bb in regions:
        o_, band, fx = g.occ[li], g.band[li], g.fixed[li]
        x0, y0 = g.cell(bb[0] - R, bb[1] - R)
        x1, y1 = g.cell(bb[2] + R, bb[3] + R)
        for cy in range(max(y0, 0), min(y1, H - 1) + 1):
            for cx in range(max(x0, 0), min(x1, W - 1) + 1):
                i = cy * W + cx
                if (li, i) in seen:
                    continue
                seen.add((li, i))
                x, y = g.pos(cx, cy)
                own = fx[i]
                near = 0
                for ob in g.cu.at(li, x, y):
                    d = _obj_dist(ob, x, y)
                    if d < R:
                        near = 1
                        if d <= 1e-9 and not ob[2] and fx[i] == 0:
                            own = ob[1] if own in (0, ob[1]) else BLOCK
                o_[i] = own
                band[i] = near        # the index holds fixed and routed copper alike


def _pair_band(g: Grid, segs: list[Seg], width: float, prm: RouteParams) -> list[array]:
    """1.0 on the line where the second side of a pair should run (one track pitch beside the first
    side, on the same layer), falling to 0 within ~1 mm either side of it."""
    n = g.W * g.H
    out = [array("f", bytes(4 * n)) for _ in range(g.L)]
    # nearest legal centre line: halo of the first side (clearance + margin) + both half widths
    centre = width + prm.clearance + g.margin
    for s in segs:
        li = g.lidx[s.layer]
        g.field_ring(out[li], s.x1, s.y1, s.x2, s.y2, centre, 1.0)
    return out


class Terminal:
    """Something the router connects: a pad, or a group of copper already joined (pads, tracks, vias)."""

    def __init__(self, key, cells: set, x: float, y: float):
        self.key, self.cells, self.x, self.y = key, cells, x, y

    @staticmethod
    def of_pad(g: "Grid", p: Pad) -> "Terminal":
        return Terminal((p.ref, p.number, p.x, p.y), _pad_cells(g, p), p.x, p.y)

    @staticmethod
    def of_group(g: "Grid", grp: dict) -> "Terminal":
        cells: set = set()
        for p in grp["pads"]:
            cells |= _pad_cells(g, p)
        for sg in grp["segs"]:
            if sg.layer not in g.lidx:
                continue
            li = g.lidx[sg.layer]
            n = max(1, int(sg.length / (g.res / 2)))
            for k in range(n + 1):
                t = k / n
                cx, cy = g.cell(sg.x1 + t * (sg.x2 - sg.x1), sg.y1 + t * (sg.y2 - sg.y1))
                if 0 <= cx < g.W and 0 <= cy < g.H:
                    cells.add((li, cx, cy))
        for v in grp["vias"]:
            cx, cy = g.cell(v.x, v.y)
            if 0 <= cx < g.W and 0 <= cy < g.H:
                cells |= {(li, cx, cy) for li in range(g.L)}
        if grp["pads"]:
            p = sorted(grp["pads"], key=lambda q: (q.ref, q.number))[0]
            key, x, y = (p.ref, p.number, p.x, p.y), p.x, p.y
        elif grp["segs"]:
            sg = grp["segs"][0]
            key, x, y = ("~", "", sg.x1, sg.y1), sg.x1, sg.y1
        else:
            v = grp["vias"][0]
            key, x, y = ("~", "", v.x, v.y), v.x, v.y
        return Terminal(key, cells, x, y)


def route_net(g: Grid, board_pads: list, net: str, kind: str, width, prm: RouteParams,
              pair_field=None, act_w: float = 1.0, push: dict[int, float] | None = None,
              first_of_pair: bool = False, why: list | None = None) -> RoutedNet | None:
    """Grow a tree over the net's pads. `width` may be a list: each connection tries the widths in
    order (e.g. a power net goes narrow only where the wide track does not fit)."""
    widths = list(width) if isinstance(width, (list, tuple)) else [width]
    # pads, or Terminals (groups of copper already joined, for nets that are partly routed)
    pads = sorted((t if isinstance(t, Terminal) else Terminal.of_pad(g, t) for t in board_pads),
                  key=lambda t: t.key)
    pads = [t for t in pads if t.cells]
    if len(pads) < 2:
        return None
    searches = [_Search(g, net, kind, w, prm, pair_field, act_w, push) for w in widths]
    if first_of_pair and prm.pair_room_cost:
        for sr, w in zip(searches, widths):
            sr.room = prm.pair_room_cost
            sr._room_k = max(1, int(round((w + prm.clearance + g.margin + w / 2) / g.res)))
    tree: set[tuple[int, int, int]] = set(pads[0].cells)
    remaining = list(pads[1:])
    all_segs, all_vias = [], []
    while remaining:
        targets: dict[tuple[int, int, int], int] = {}
        for k, p in enumerate(remaining):
            for c in p.cells:
                targets.setdefault(c, k)
        xs = [c[1] for c in tree]
        ys = [c[2] for c in tree]
        tx0, tx1, ty0, ty1 = min(xs), max(xs), min(ys), max(ys)

        def gap(p):
            cx, cy = g.cell(p.x, p.y)
            return max(tx0 - cx, 0, cx - tx1) + max(ty0 - cy, 0, cy - ty1)
        near = sorted(remaining, key=gap)[:prm.near_targets]
        tcent = [g.cell(p.x, p.y) for p in near]
        near_ids = {remaining.index(p) for p in near}
        # aim at the nearest pads first; the others stay valid targets
        txs = xs + [c[1] for c, k in targets.items() if k in near_ids]
        tys = ys + [c[2] for c, k in targets.items() if k in near_ids]
        path, used_w = None, widths[0]
        # a pad with no cell this track can stand on cannot be reached on this grid: give up at once
        # (fine-pitch pins off the grid phase; the fine-grid fallback handles them)
        if push is None:
            # each pad's own cells (pads of one net may overlap, e.g. mounting holes, USB-C VBUS/GND pins)
            own_cells = {k: remaining[k].cells for k in near_ids}
            reachable = lambda srch, k: any(srch.track_state(*c) is OK for c in own_cells[k])
            if not all(any(reachable(srch, k) for srch in searches) for k in near_ids):
                if why is not None:
                    why.append("pin")       # no grid cell on a pin: a finer grid is needed, not pushing
                return None
        # small window first with every width (a narrow track through a pin field beats a wide one that
        # floods the board looking for room; narrowed stretches are widened again afterwards), then large
        tset = set(targets)
        srcs = {}
        for grow in (prm.window_mm, prm.window_mm * 4):
            m = int(grow / g.res)
            win = (max(min(txs) - m, 0), max(min(tys) - m, 0), min(max(txs) + m, g.W - 1), min(max(tys) + m, g.H - 1))
            for srch, w in zip(searches, widths):
                if w not in srcs:
                    # start only where this track width keeps its clearance (a wide track starting at a
                    # pad's edge cell would stick out of the pad towards its neighbours)
                    srcs[w] = sorted([c for c in tree if srch.track_state(*c) is OK] or tree)
                path = srch.run(srcs[w], tset, tcent, win)
                if path:
                    used_w = w
                    break
            if path:
                break
        if not path:
            return None
        segs, vias = _path_to_segs(g, path, net, used_w)
        all_segs += segs
        all_vias += vias
        tree.update(path)
        hit = remaining[targets[path[-1]]]
        tree.update(hit.cells)
        remaining.remove(hit)
    return RoutedNet(net, all_segs, all_vias, widths[0])


def _crossed(new: RoutedNet, done: dict[str, RoutedNet], prm: RouteParams) -> list[str]:
    """Routed nets whose copper the new route violates (clearance), found geometrically."""
    from .geometry import SegIndex, edge_gap
    out = set()
    vr = prm.via_d / 2
    new_vias = [Seg(x, y, x, y, prm.via_of(new.net)[0], "*", new.net) for x, y in new.vias]
    for n, rn in done.items():
        if n == new.net:
            continue
        idx = SegIndex(rn.segs, cell=2.0)
        other_vias = [Seg(x, y, x, y, prm.via_of(n)[0], "*", n) for x, y in rn.vias]
        hit = False
        for s in new.segs:
            for t in idx.near(s.layer, s.bbox(s.width / 2 + prm.clearance + 1.0)):
                if edge_gap(s, t) < prm.clearance:
                    hit = True
                    break
            if not hit:
                hit = any(edge_gap(s, v) < prm.clearance for v in other_vias)
            if hit:
                break
        if not hit:
            for v in new_vias:
                if any(edge_gap(v, t) < prm.clearance for t in rn.segs) or \
                        any(edge_gap(v, w) < prm.clearance for w in other_vias):
                    hit = True
                    break
        if hit:
            out.add(n)
    return sorted(out)


def route(board: Board, cls: Classification, prm: RouteParams, weights: dict, nets: list[str] | None = None,
          progress=None, prerouted: dict | None = None) -> RouteResult:
    """Route `nets` (default: all of prm.classes). `prerouted` routes are laid down first as ordinary
    (pushable) tracks; with it, the result lists every route, rerouted ones included."""
    import time as _time
    t_start = _time.time()
    for n_ in board.nets:                       # net classes from the project file
        c_ = board.class_of(n_)
        if c_:
            if c_.get("track_width"):
                prm.net_width.setdefault(n_, float(c_["track_width"]))
            if c_.get("via_diameter") and c_.get("via_drill"):
                prm.net_via.setdefault(n_, (float(c_["via_diameter"]), float(c_["via_drill"])))
    base = build_grid(board, cls, prm, weights)     # pads, existing copper, edges, fields (built once)
    g = base.clone()
    pads_by_net = board.pads_by_net()
    already = {s.net for s in board.segs}
    pairs = [(a, b) for a, b in cls.pairs if (nets is None or (a in nets and b in nets))]
    paired = {n for ab in pairs for n in ab}
    order: list[tuple[str, str | None]] = []           # (net, partner it should run beside)
    for a, b in pairs:
        order += [(a, None), (b, a)]

    def span(n):
        ps = pads_by_net.get(n, [])
        return (max(p.x for p in ps) - min(p.x for p in ps) + max(p.y for p in ps) - min(p.y for p in ps)) if ps else 0

    for kind in prm.classes:
        rest = [n for n in cls.nets_of(kind) if n not in paired and (nets is None or n in nets)]
        order += [(n, None) for n in sorted(rest, key=lambda n: (span(n), n))]
    failed, skipped = [], []
    done: dict[str, RoutedNet] = {}
    partner_of = {b: a for a, b in pairs}
    pin_bound: set[str] = set()     # nets that failed because no grid cell sits on one of their pins
    partial: dict[str, list] = {}   # nets partly routed already -> their groups of joined copper
    frozen: dict[str, RoutedNet] = {}
    wbox = prm.window
    for n, rn in (prerouted or {}).items():
        inside = wbox is None or all(wbox[0] <= p.x <= wbox[2] and wbox[1] <= p.y <= wbox[3]
                                     for p in pads_by_net.get(n, []))
        if inside:
            _commit(g, n, cls.kind.get(n, "other"), rn.segs, rn.vias, prm, fields=False)
            done[n] = rn
        else:
            # reaches outside the window: kept as it is, cannot be pushed aside here
            _commit(g, n, cls.kind.get(n, "other"), rn.segs, rn.vias, prm, fields=False, fixed=True)
            frozen[n] = rn

    def terms(g, net):
        """What to connect: the net's pads, or, for a net that is partly routed already, the groups of
        copper that are joined so far."""
        if net in partial:
            return [Terminal.of_group(g, grp) for grp in partial[net]]
        return [p for p in pads_by_net.get(net, []) if any(l in g.lidx for l in p.layers)]

    def attempt(g, net, partner, first_of_pair=False):
        kind = cls.kind.get(net, "other")
        pads = terms(g, net)
        widths = prm.widths_for(net, kind)
        pf = _pair_band(g, done[partner].segs, done[partner].width, prm) if partner in done else None
        why: list = []
        res = route_net(g, pads, net, kind, widths, prm, pf, weights.get(cls.activity.get(net, "data"), 1.0),
                        first_of_pair=first_of_pair, why=why)
        if why:
            pin_bound.add(net)
        if res:
            _commit(g, net, kind, res.segs, res.vias, prm)
            done[net] = res
            return True
        return False

    todo = []
    for net, partner in order:
        pads = [p for p in pads_by_net.get(net, []) if any(l in g.lidx for l in p.layers)]
        if len(pads) < 2:
            skipped.append(net)
            continue
        if net in already and cls.kind.get(net) != "gnd":
            # partly routed (by hand, or by an earlier run): join the groups of copper that exist
            from .connect import groups as copper_groups
            grs = [gr for gr in copper_groups(board, net)
                   if any(any(l in g.lidx for l in p.layers) for p in gr["pads"]) or gr["segs"] or gr["vias"]]
            if len(grs) <= 1:
                skipped.append(net)          # already complete
                continue
            partial[net] = grs
        todo.append((net, partner))
    def together(a: RoutedNet, b: RoutedNet) -> float:
        from .geometry import SegIndex, coupled_length
        la = sum(s.length for s in a.segs) or 1.0
        idx = SegIndex(b.segs, cell=2.0)
        return sum(coupled_length(s, idx, 0.6)[0] for s in a.segs) / la

    def pair_attempt(g0, a, b):
        """Route both sides of a pair; if they do not run together, try a stronger pull and the other
        order, keep the best. Returns (grid, ok_a, ok_b)."""
        best = None
        variants = [((a, b), prm.pair_cost), ((a, b), prm.pair_cost * 3), ((b, a), prm.pair_cost * 2)]
        for k, ((first, second), pc) in enumerate(variants):
            g1 = g0.clone()
            saved = prm.pair_cost
            prm.pair_cost = pc
            ok1 = attempt(g1, first, None, first_of_pair=True)
            ok2 = attempt(g1, second, first) if ok1 else False
            prm.pair_cost = saved
            r1, r2 = done.pop(first, None), done.pop(second, None)
            score = (int(ok1) + int(ok2), together(r1, r2) if ok1 and ok2 else 0.0)
            if best is None or score > best[0]:
                best = (score, g1, {first: r1, second: r2})
            if score[0] == 2 and score[1] >= prm.pair_retry_below:
                break
        _, g1, res = best
        for n, r in res.items():
            if r is not None:
                done[n] = r
        return g1, res[a] is not None, res[b] is not None

    # nets with a pin that no cell of this grid can reach (fine-pitch parts off the grid phase) go first,
    # on a fine grid around their pads, before other tracks fill the space around those pins
    early_stats: dict = {}
    if prm.fine_fallback and prm.res > 0.15 and prerouted is None:
        def pin_bound_net(n):
            if n in partial:
                return False
            kind = cls.kind.get(n, "other")
            w = min(prm.widths_for(n, kind))   # narrowest it may use
            srch = _Search(g, n, kind, w, prm, None, 1.0)
            for p in pads_by_net.get(n, []):
                if any(l in g.lidx for l in p.layers) and \
                        not any(srch.track_state(*c) is OK for c in _pad_cells(g, p)):
                    return True
            return False
        early = [n for n, _ in todo if pin_bound_net(n)]
        if early:
            import copy
            t_e = _time.time()
            bxs = [x for poly in board.outline for x, _ in poly]
            bys = [y for poly in board.outline for _, y in poly]
            if progress:
                progress(f"fine grid first for {len(early)} pin-bound nets", True)
            got: dict[str, RoutedNet] = {}
            efail = []
            for n in early:          # one net at a time, each in a small window around its own pads
                pe = copy.copy(prm)
                pe.res, pe.fine_fallback = prm.res / 2, False
                pe.phase = _grid_phase(board, pe.res, {n})
                ps = pads_by_net.get(n, [])
                m = 5.0
                pe.window = (max(min(p.x for p in ps) - m, min(bxs or [0])), max(min(p.y for p in ps) - m, min(bys or [0])),
                             min(max(p.x for p in ps) + m, max(bxs or [1e9])), min(max(p.y for p in ps) + m, max(bys or [1e9])))
                re_ = route(board, cls, pe, weights, nets=[n], progress=progress, prerouted=dict(got))
                got = {rn.net: rn for rn in re_.routed}
                efail += re_.failed
            for rn in got.values():
                _commit(g, rn.net, cls.kind.get(rn.net, "other"), rn.segs, rn.vias, prm)
                done[rn.net] = rn
            todo = [(n, pt) for n, pt in todo if n not in done]
            early_stats = {"early_nets": len(early), "t_early": round(_time.time() - t_e, 1), "early_failed": efail,
                           "events": [{"type": "fine_first", "nets": early, "failed": efail,
                                       "why": "a pin no coarse grid cell can reach (fine pitch)"}]}

    todo_all = list(order)
    handled = set()
    for k, (net, partner) in enumerate(todo):
        if net in handled:
            continue
        nxt = todo[k + 1] if k + 1 < len(todo) else None
        if partner is None and nxt is not None and nxt[1] == net:
            g, ok_a, ok_b = pair_attempt(g, net, nxt[0])
            handled.add(nxt[0])
            for n, ok in ((net, ok_a), (nxt[0], ok_b)):
                if not ok:
                    failed.append(n)
                if progress:
                    progress(n, ok)
            continue
        ok = attempt(g, net, partner)
        if not ok:
            failed.append(net)
        if progress:
            progress(net, ok)

    import time
    t_end = time.time() + prm.rip_time_s
    stats = {"t_main": round(time.time() - t_start, 1), "failed_main": len(failed), "pushes": 0,
             "rerouted": 0, "events": [], **early_stats}
    if failed:
        stats["events"].append({"type": "failed_first_pass", "nets": list(failed)})

    def rebuild(keep: dict[str, RoutedNet]):
        # occupancy from scratch; the proximity fields keep ripped analog tracks (harmless, slightly cautious)
        g2 = base.clone()
        for n, rn in keep.items():
            _commit(g2, n, cls.kind.get(n, "other"), rn.segs, rn.vias, prm, fields=False)
        return g2

    def with_partners(names):
        names = set(names)
        names |= {partner_of[v] for v in names if v in partner_of}
        names |= {b for a, b in pairs if a in names}
        return sorted(names)

    def reroute(g, seq):
        """Route `seq` (first sides of pairs before second sides); return the nets that failed."""
        seq = [v for v in seq if v not in partner_of] + [v for v in seq if v in partner_of]
        bad = []
        for n in seq:
            ok = attempt(g, n, partner_of.get(n))
            if progress:
                progress(n, ok)
            if not ok:
                bad.append(n)
        return bad

    # 1) local rip-up (optional): free the area around a failed net's pads
    still = []
    for net in (list(failed) if prm.local_rip else []):
        if time.time() > t_end:
            still.append(net)
            continue
        rr = prm.rip_radius
        boxes = [(p.box()[0] - rr, p.box()[1] - rr, p.box()[2] + rr, p.box()[3] + rr) for p in pads_by_net.get(net, [])]

        def near(rn):
            return any(any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in boxes
                           for x, y in ((s.x1, s.y1), (s.x2, s.y2), ((s.x1 + s.x2) / 2, (s.y1 + s.y2) / 2)))
                       for s in rn.segs)
        victims = with_partners(n for n, rn in done.items() if near(rn))
        for v in victims:
            done.pop(v, None)
        g = rebuild(done)
        still += reroute(g, [net] + [v for v in victims if v != net])
    if prm.local_rip:
        failed = sorted(set(still))

    # 2) push: route a failed net through others' tracks at a price, then reroute the ones it crossed
    pushes: dict[str, int] = {}
    queue = list(failed)
    failed = []
    t_push = time.time()
    total = prm.push_total or max(20, 3 * len(queue))
    while queue:
        net = queue.pop(0)
        if net in done:
            continue
        if time.time() > t_end or pushes.get(net, 0) >= prm.push_limit or stats["pushes"] >= total \
                or net in pin_bound:
            failed.append(net)
            continue
        pushes[net] = pushes.get(net, 0) + 1
        stats["pushes"] += 1
        kind = cls.kind.get(net, "other")
        # pushing a net means re-routing it: big nets (many pads) are dearer, analog dearer still
        id_cost = {g.nid(n): (prm.push_cost_analog if cls.kind.get(n) == "analog" else prm.push_cost)
                   * max(1.0, len(pads_by_net.get(n, [])) / 4.0) for n in done}
        pads = terms(g, net)
        pf = _pair_band(g, done[partner_of[net]].segs, done[partner_of[net]].width, prm) \
            if partner_of.get(net) in done else None
        res = route_net(g, pads, net, kind, prm.widths_for(net, kind)[-1], prm, pf,
                        weights.get(cls.activity.get(net, "data"), 1.0), id_cost)
        if not res:
            failed.append(net)
            continue
        victims = with_partners(_crossed(res, done, prm))
        victims = [v for v in victims if v != net]
        for v in victims:
            rv = done.pop(v, None)
            if rv is not None:
                _uncommit(g, v, rv, prm)
        done[net] = res
        _commit(g, net, kind, res.segs, res.vias, prm)
        stats["events"].append({"type": "push", "net": net, "moved": victims})
        if progress:
            progress(f"{net} (pushed {len(victims)}: {', '.join(victims[:4])})", True)
        stats["rerouted"] += len(victims)
        queue += reroute(g, victims)
    failed = sorted(set(failed))
    stats["t_push"] = round(time.time() - t_push, 1)
    stats["failed_push"] = len(failed)
    t_fine = time.time()

    # 3) finer grid for what is left (fine-pitch parts such as USB-C, SSOP), one net at a time with the
    #    grid aligned to its pads; the routes so far stay pushable so a walled-in pin can be freed
    if failed and prm.fine_fallback and prm.res > 0.15:
        import copy
        p2 = copy.copy(prm)
        p2.res, p2.fine_fallback, p2.rip_time_s = prm.res / 2, False, min(prm.rip_time_s, 120.0)
        if progress:
            progress(f"fine grid {p2.res} mm for {len(failed)} nets", True)
        bxs = [x for poly in board.outline for x, _ in poly]
        bys = [y for poly in board.outline for _, y in poly]

        def attempt_fine(nets_, phase_nets, done_, failed_, windowed=True):
            """Route `nets_` on the fine grid, in a window around their pads; keep the result only if
            fewer nets end up unrouted."""
            p3 = copy.copy(p2)
            p3.phase = _grid_phase(board, p2.res, set(phase_nets))
            ps = [p for n in nets_ for p in pads_by_net.get(n, [])]
            if windowed and ps and bxs:
                m = 10.0
                p3.window = (max(min(p.x for p in ps) - m, min(bxs)), max(min(p.y for p in ps) - m, min(bys)),
                             min(max(p.x for p in ps) + m, max(bxs)), min(max(p.y for p in ps) + m, max(bys)))
            r = route(board, cls, p3, weights, nets=list(nets_), progress=progress, prerouted=dict(done_))
            new_done = {rn.net: rn for rn in r.routed}
            lost = [n for n in done_ if n not in new_done]        # pushed aside and not rerouted
            new_failed = sorted(set(r.failed) | set(lost) | (set(failed_) - set(nets_)))
            if len(new_failed) < len(failed_):
                return new_done, new_failed
            return done_, failed_

        before_fine = list(failed)
        # all failed nets together first (one fine grid, aligned to their pads) ...
        done, failed = attempt_fine(failed, failed, done, failed)
        # ... then one at a time, each with the grid on its own pads
        for net in list(failed):
            done, failed = attempt_fine([net], [net], done, failed)
        # ... and, for what is still left, on the whole board (long routes outside the window can move)
        for net in list(failed):
            done, failed = attempt_fine([net], [net], done, failed, windowed=False)
        stats["events"].append({"type": "fine_fallback", "nets": before_fine, "still_failed": list(failed)})
    stats["t_fine"] = round(time.time() - t_fine, 1)

    # 4) widen what fits; power nets still narrow away from fine-pitch pins are then re-routed wide,
    #    pushing other routes aside, and kept only if the narrow length halves and every pushed route
    #    finds a new way
    if prerouted is None:
        stats["widened"] = widen(board, list(done.values()), cls, prm)
        if stats["widened"]:
            # the grid still holds the narrow copies: lay the routes down again as they are now
            g = base.clone()
            for n_, rn_ in done.items():
                _commit(g, n_, cls.kind.get(n_, "other"), rn_.segs, rn_.vias, prm, fields=False)
    if prm.widen_push_s > 0 and prerouted is None and not failed:
        t_w = time.time()
        full_of = prm.width
        widened = 0

        def narrow_len(n, rn):
            kind = cls.kind.get(n, "other")
            full = prm.full_width(n, kind)
            small = [p for p in pads_by_net.get(n, []) if min(p.box()[2] - p.box()[0], p.box()[3] - p.box()[1]) < full]
            return sum(sg.length for sg in rn.segs if sg.width < full - 1e-9 and not any(
                min(p.dist(sg.x1, sg.y1), p.dist(sg.x2, sg.y2)) < 1.5 for p in small))
        cands = sorted(((narrow_len(n, rn), n) for n, rn in done.items() if cls.kind.get(n) == "power"), reverse=True)
        for before, n in cands:
            if before < 1.0:
                break
            if time.time() - t_w > prm.widen_push_s:
                break
            kind = cls.kind.get(n, "other")
            snap_g, snap_done = g.clone(), dict(done)
            old = done.pop(n)
            _uncommit(g, n, old, prm)
            id_cost = {g.nid(m): (prm.push_cost_analog if cls.kind.get(m) == "analog" else prm.push_cost)
                       * max(1.0, len(pads_by_net.get(m, [])) / 4.0) for m in done}
            pads = terms(g, n)
            res = route_net(g, pads, n, kind, prm.widths_for(n, kind), prm,
                            None, weights.get(cls.activity.get(n, "data"), 1.0), id_cost)
            why = "no route" if res is None else ""
            ok = not why
            victims = []
            if ok:
                victims = [v for v in with_partners(_crossed(res, done, prm)) if v != n]
                ok = len(victims) <= prm.widen_push_victims
                why = "" if ok else f"would move {len(victims)} routes"
            if ok:
                for v in victims:
                    _uncommit(g, v, done.pop(v), prm)
                done[n] = res
                _commit(g, n, kind, res.segs, res.vias, prm)
                bad = reroute(g, victims)
                ok = not bad
                why = "" if ok else f"could not re-route {bad}"
            if ok:
                narrow_copy = RoutedNet(n, list(done[n].segs), list(done[n].vias), done[n].width)
                if widen(board, list(done.values()), cls, prm, only={n}):
                    _uncommit(g, n, narrow_copy, prm)
                    _commit(g, n, kind, done[n].segs, done[n].vias, prm, fields=False)
                after = narrow_len(n, done[n])
                ok = after < 0.5 * before
                why = "" if ok else f"narrow {before:.1f} -> {after:.1f} mm"
            stats.setdefault("widen_push_log", []).append(f"{n}: {'ok' if ok else why}")
            stats["events"].append({"type": "rewiden", "net": n, "ok": ok, "moved": victims if ok else [],
                                    "why": why, "narrow_before_mm": round(before, 1)})
            if ok:
                widened += 1
                if progress:
                    progress(f"{n} wide (moved {len(victims)})", True)
            else:
                g, done = snap_g, snap_done          # roll back
        stats["power_rewidened"] = widened
        stats["t_widen_push"] = round(time.time() - t_w, 1)
    stats["t_total"] = round(time.time() - t_start, 1)
    if prerouted is not None:
        return RouteResult(list(done.values()) + list(frozen.values()), failed, skipped, stats)
    routed = [done[n] for n, _ in todo_all if n in done]
    return RouteResult(routed, failed, skipped, stats)


def escape_to_via(board: Board, cls: Classification, prm: RouteParams, weights: dict, net: str,
                  pad_names: list[str], max_len: float = 15.0, width: float = 0.3) -> tuple[list[RoutedNet], list[str]]:
    """For pads that need a via (e.g. ground pads with no via site in reach): walk from the pad on its
    own layer, around other copper, to the nearest spot where a via fits. Returns (routes, failed)."""
    g = build_grid(board, cls, prm, weights)
    pads = {f"{p.ref}.{p.number}": p for fp in board.footprints for p in fp.pads}
    out, failed = [], []
    for name in pad_names:
        p = pads.get(name)
        if p is None or not any(l in g.lidx for l in p.layers):
            failed.append(name)
            continue
        srch = _Search(g, net, "other", width, prm, None, 1.0)
        start = _pad_cells(g, p)
        li0 = next(iter(start))[0]
        limit = max_len / g.res
        best = None
        dist: dict[tuple[int, int, int], float] = {c: 0.0 for c in start}
        came: dict[tuple[int, int, int], tuple[int, int, int] | None] = {c: None for c in start}
        heap = [(0.0, k, c) for k, c in enumerate(sorted(start))]
        heapq.heapify(heap)
        cnt = len(heap)
        seen = set()
        fixed = g.fixed[li0]
        while heap:
            d, _, cur = heapq.heappop(heap)
            if cur in seen:
                continue
            seen.add(cur)
            _, cx, cy = cur
            # a via site: clear of everything fixed (not on any pad), and legal on all layers
            if cur not in start and fixed[cy * g.W + cx] == 0 and srch.via_ok(cx, cy) and \
                    p.dist(*g.pos(cx, cy)) >= prm.via_d / 2 + 0.1:
                best = cur
                break
            if d > limit:
                continue
            for dx, dy, step in DIRS:
                nxt = (li0, cx + dx, cy + dy)
                if nxt in seen or not (0 <= nxt[1] < g.W and 0 <= nxt[2] < g.H) or not srch.ok(*nxt):
                    continue
                nd = d + step
                if nd < dist.get(nxt, math.inf):
                    dist[nxt] = nd
                    came[nxt] = cur
                    heapq.heappush(heap, (nd, cnt, nxt))
                    cnt += 1
        if best is None:
            failed.append(name)
            continue
        path = []
        n = best
        while n is not None:
            path.append(n)
            n = came[n]
        path.reverse()
        segs, _ = _path_to_segs(g, path, net, width)
        vx, vy = g.pos(best[1], best[2])
        rn = RoutedNet(net, segs, [(vx, vy)], width)
        _commit(g, net, "gnd", rn.segs, rn.vias, prm)
        out.append(rn)
    return out, failed


def widen(board: Board, routes: list[RoutedNet], cls: Classification, prm: RouteParams,
          kinds=("power", "gnd"), only: set | None = None) -> int:
    """After routing: tracks of power/ground nets that had to go narrow are widened back where the
    copper around allows it (checked against real shapes, clearances per net class). Returns the
    number of segments widened. Routes are updated in place."""
    from .geometry import SegIndex, point_seg_dist, seg_seg_dist
    from .geometry import point_in_poly, point_poly_dist
    all_segs = list(board.segs) + [s for r in routes for s in r.segs]
    idx = SegIndex(all_segs, cell=2.0)
    vias = [(v.x, v.y, v.diameter / 2, v.net) for v in board.vias] + \
           [(x, y, prm.via_of(r.net)[0] / 2, r.net) for r in routes for x, y in r.vias]
    pads = [p for fp in board.footprints for p in fp.pads]
    count = 0

    def fits(s: Seg, w: float) -> bool:
        hw = w / 2
        bb = s.bbox(hw + 1.0)
        for layer_seg in idx.near(s.layer, bb):
            if layer_seg.net == s.net or layer_seg is s:
                continue
            c = board.clearance(s.net, layer_seg.net, default=prm.clearance)
            if seg_seg_dist(s, layer_seg) - layer_seg.width / 2 - hw < c + 0.005:
                return False
        for x, y, r, n in vias:
            if n != s.net and bb[0] - r <= x <= bb[2] + r and bb[1] - r <= y <= bb[3] + r:
                c = board.clearance(s.net, n, default=prm.clearance)
                if point_seg_dist(x, y, s.x1, s.y1, s.x2, s.y2) - r - hw < c + 0.005:
                    return False
        steps = max(2, int(s.length / 0.1))
        near_pads = [p for p in pads if p.net != s.net and s.layer in p.layers
                     and p.box()[0] <= bb[2] and p.box()[2] >= bb[0] and p.box()[1] <= bb[3] and p.box()[3] >= bb[1]]
        for p in near_pads:
            c = board.clearance(s.net, p.net, default=prm.clearance)
            for k in range(steps + 1):
                t = k / steps
                if p.dist(s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1)) - hw < c + 0.005:
                    return False
        if board.outline:
            for k in range(steps + 1):
                t = k / steps
                px, py = s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1)
                if not any(point_in_poly(px, py, poly) for poly in board.outline) or \
                        min(point_poly_dist(px, py, poly) for poly in board.outline) < prm.edge_clearance + hw:
                    return False
        for ko in board.keepouts:
            if ko.get("no_tracks") and s.layer in ko["layers"]:
                for k in range(steps + 1):
                    t = k / steps
                    px, py = s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1)
                    if point_in_poly(px, py, ko["poly"]) or point_poly_dist(px, py, ko["poly"]) < hw:
                        return False
        return True

    for r in routes:
        kind = cls.kind.get(r.net, "other")
        if kind not in kinds or (only is not None and r.net not in only):
            continue
        full = prm.full_width(r.net, kind)
        targets = [full] + [w for w in (0.4,) if w < full]
        new = []
        for s in r.segs:
            if s.width >= full - 1e-9:
                new.append(s)
                continue
            for w in targets:
                if w <= s.width + 1e-9:
                    break
                if fits(s, w):
                    ns = Seg(s.x1, s.y1, s.x2, s.y2, w, s.layer, s.net)
                    # the index holds the old segment: swap it for the wider one
                    for key in idx._cells(s.layer, s.bbox()):
                        lst = idx.grid.get(key)
                        if lst and s in lst:
                            lst.remove(s)
                    for key in idx._cells(ns.layer, ns.bbox()):
                        idx.grid[key].append(ns)
                    s = ns
                    count += 1
                    break
            new.append(s)
        r.segs = new
    return count
