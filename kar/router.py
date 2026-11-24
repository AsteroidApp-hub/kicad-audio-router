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
    width: dict = field(default_factory=lambda: {"analog": 0.25, "power": 0.5, "digital": 0.2, "other": 0.25})
    narrow_width: dict = field(default_factory=lambda: {"power": 0.3})   # retry width when the wide one fails
    near_mm: float = 3.0              # reach of the proximity cost fields
    near_cost: float = 3.0            # extra cost per step right next to the other kind (falls off linearly)
    pair_gap: float = 0.25            # target edge gap between the two sides of a pair
    pair_cost: float = 1.5            # extra cost per step for the second side away from the first
    window_mm: float = 12.0           # search window around the net, widened once on failure
    rip_rounds: int = 3               # rip-up and reroute rounds for nets that failed
    rip_radius: float = 2.0           # routed nets passing this close to a failed net's pads are ripped up
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


class Grid:
    def __init__(self, board: Board, prm: RouteParams):
        self.prm = prm
        self.res = prm.res
        xs = [x for poly in board.outline for x, _ in poly] or [p.x for fp in board.footprints for p in fp.pads]
        ys = [y for poly in board.outline for _, y in poly] or [p.y for fp in board.footprints for p in fp.pads]
        self.ox, self.oy = min(xs) - 1.0, min(ys) - 1.0
        self.W = int(math.ceil((max(xs) + 1.0 - self.ox) / self.res)) + 1
        self.H = int(math.ceil((max(ys) + 1.0 - self.oy) / self.res)) + 1
        self.layers = prm.layers or [l for l in board.copper_layers if l not in board.plane_layers]
        self.L = len(self.layers)
        self.lidx = {n: i for i, n in enumerate(self.layers)}
        n = self.W * self.H
        self.occ = [array("H", bytes(2 * n)) for _ in range(self.L)]
        self.near_dig = [array("f", bytes(4 * n)) for _ in range(self.L)]
        self.near_ana = [array("f", bytes(4 * n)) for _ in range(self.L)]
        self.net_id: dict[str, int] = {}
        self.margin = self.res * 0.71        # discretisation safety

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


def build_grid(board: Board, cls: Classification, prm: RouteParams, weights: dict) -> Grid:
    g = Grid(board, prm)
    halo = prm.clearance + g.margin
    pads = [p for fp in board.footprints for p in fp.pads]
    # halos first (conflicts become BLOCK), then the copper itself (owned by its net)
    for p in pads:
        val = g.nid(p.net) if p.net else BLOCK
        for li in _pad_layers(g, p):
            g.stamp_box(li, p.box(), val, halo)
    for s in board.segs:
        if s.layer in g.lidx:
            g.stamp_capsule(g.lidx[s.layer], s.x1, s.y1, s.x2, s.y2, s.width / 2 + halo, g.nid(s.net))
    for v in board.vias:
        for li in range(g.L):
            g.stamp_capsule(li, v.x, v.y, v.x, v.y, v.diameter / 2 + halo, g.nid(v.net))
    for p in pads:
        if p.net:
            for li in _pad_layers(g, p):
                g.stamp_box(li, p.box(), g.nid(p.net), 0.0, force=True)
    g.block_outside(board.outline)
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


class _Search:
    """One A* search for one net between its current tree and the remaining pads."""

    def __init__(self, g: Grid, net: str, kind: str, width: float, prm: RouteParams,
                 pair_field: list[array] | None, act_w: float):
        self.g, self.net, self.kind, self.prm = g, net, kind, prm
        self.val = g.nid(net)
        self.track_disk = g.disk(width / 2)
        self.via_disk = g.disk(prm.via_d / 2)
        self.via_cost = prm.analog_via_cost if kind == "analog" else prm.via_cost
        self.pair_field = pair_field
        if kind == "analog":
            self.field, self.fw = g.near_dig, prm.near_cost
        elif kind == "digital":
            self.field, self.fw = g.near_ana, prm.near_cost * act_w
        else:
            self.field, self.fw = None, 0.0
        self._ok_cache: dict[int, bool] = {}
        self._via_cache: dict[int, bool] = {}

    def ok(self, li: int, cx: int, cy: int) -> bool:
        g = self.g
        key = (li * g.H + cy) * g.W + cx
        r = self._ok_cache.get(key)
        if r is not None:
            return r
        o, W, H, v = g.occ[li], g.W, g.H, self.val
        r = True
        for dx, dy in self.track_disk:
            x, y = cx + dx, cy + dy
            if not (0 <= x < W and 0 <= y < H):
                r = False
                break
            c = o[y * W + x]
            if c != 0 and c != v:
                r = False
                break
        self._ok_cache[key] = r
        return r

    def via_ok(self, cx: int, cy: int) -> bool:
        g = self.g
        key = cy * g.W + cx
        r = self._via_cache.get(key)
        if r is not None:
            return r
        W, H, v = g.W, g.H, self.val
        r = True
        for li in range(g.L):
            o = g.occ[li]
            for dx, dy in self.via_disk:
                x, y = cx + dx, cy + dy
                if not (0 <= x < W and 0 <= y < H):
                    r = False
                    break
                c = o[y * W + x]
                if c != 0 and c != v:
                    r = False
                    break
            if not r:
                break
        self._via_cache[key] = r
        return r

    def run(self, sources: list[tuple[int, int, int]], targets: set[tuple[int, int, int]],
            tcenters: list[tuple[float, float]], window) -> list[tuple[int, int, int]] | None:
        g, prm = self.g, self.prm
        W = g.W
        wx0, wy0, wx1, wy1 = window

        def h(cx, cy):
            best = math.inf
            for tx, ty in tcenters:
                dx, dy = abs(cx - tx), abs(cy - ty)
                d = max(dx, dy) + (SQ2 - 1) * min(dx, dy)
                if d < best:
                    best = d
            return best

        gcost: dict[tuple[int, int, int], float] = {}
        came: dict[tuple[int, int, int], tuple[tuple[int, int, int] | None, int]] = {}
        heap = []
        cnt = 0
        for s in sources:
            gcost[s] = 0.0
            came[s] = (None, -1)
            heapq.heappush(heap, (h(s[1], s[2]), cnt, s))
            cnt += 1
        closed = set()
        while heap:
            _, _, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            if cur in targets:
                path = []
                n = cur
                while n is not None:
                    path.append(n)
                    n = came[n][0]
                return path[::-1]
            closed.add(cur)
            li, cx, cy = cur
            gc = gcost[cur]
            pdir = came[cur][1]
            fld = self.field[li] if self.field is not None else None
            pf = self.pair_field[li] if self.pair_field is not None else None
            for d, (dx, dy, step) in enumerate(DIRS):
                nx, ny = cx + dx, cy + dy
                if not (wx0 <= nx <= wx1 and wy0 <= ny <= wy1):
                    continue
                nxt = (li, nx, ny)
                if nxt in closed or not self.ok(li, nx, ny):
                    continue
                i = ny * W + nx
                c = step
                if fld is not None:
                    c += step * self.fw * fld[i]
                if pf is not None:
                    c += step * prm.pair_cost * (1.0 - pf[i])
                if pdir >= 0 and pdir != d:
                    c += prm.bend_cost
                ng = gc + c
                if ng < gcost.get(nxt, math.inf):
                    gcost[nxt] = ng
                    came[nxt] = (cur, d)
                    heapq.heappush(heap, (ng + h(nx, ny), cnt, nxt))
                    cnt += 1
            if g.L > 1 and self.via_ok(cx, cy):
                for lj in range(g.L):
                    if lj == li:
                        continue
                    nxt = (lj, cx, cy)
                    if nxt in closed:
                        continue
                    ng = gc + self.via_cost
                    if ng < gcost.get(nxt, math.inf):
                        gcost[nxt] = ng
                        came[nxt] = (cur, -1)
                        heapq.heappush(heap, (ng + h(cx, cy), cnt, nxt))
                        cnt += 1
        return None


def _pad_cells(g: Grid, p: Pad) -> set[tuple[int, int, int]]:
    out = set()
    b = p.box()
    x0, y0 = g.cell(b[0], b[1])
    x1, y1 = g.cell(b[2], b[3])
    for li in _pad_layers(g, p):
        for cy in range(y0, y1 + 1):
            for cx in range(x0, x1 + 1):
                x, y = g.pos(cx, cy)
                if b[0] - 1e-9 <= x <= b[2] + 1e-9 and b[1] - 1e-9 <= y <= b[3] + 1e-9:
                    out.add((li, cx, cy))
        if not out:
            cx, cy = g.cell(p.x, p.y)
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


def _commit(g: Grid, net: str, kind: str, segs: list[Seg], vias, prm: RouteParams):
    val = g.nid(net)
    halo = prm.clearance + g.margin
    for s in segs:
        li = g.lidx[s.layer]
        g.stamp_capsule(li, s.x1, s.y1, s.x2, s.y2, s.width / 2 + halo, val)
        g.stamp_capsule(li, s.x1, s.y1, s.x2, s.y2, s.width / 2, val, force=True)
        if kind == "analog":
            g.field_capsule(g.near_ana[li], s.x1, s.y1, s.x2, s.y2, prm.near_mm, 1.0)
    for x, y in vias:
        for li in range(g.L):
            g.stamp_capsule(li, x, y, x, y, prm.via_d / 2 + halo, val)
            g.stamp_capsule(li, x, y, x, y, prm.via_d / 2, val, force=True)


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


def route_net(g: Grid, board_pads: list[Pad], net: str, kind: str, width: float, prm: RouteParams,
              pair_field=None, act_w: float = 1.0) -> RoutedNet | None:
    pads = sorted(board_pads, key=lambda p: (p.ref, p.number))
    if len(pads) < 2:
        return None
    srch = _Search(g, net, kind, width, prm, pair_field, act_w)
    tree: set[tuple[int, int, int]] = set(_pad_cells(g, pads[0]))
    remaining = list(pads[1:])
    all_segs, all_vias = [], []
    while remaining:
        targets: dict[tuple[int, int, int], int] = {}
        for k, p in enumerate(remaining):
            for c in _pad_cells(g, p):
                targets.setdefault(c, k)
        tcent = [g.cell(p.x, p.y) for p in remaining]
        xs = [c[1] for c in tree] + [c[1] for c in targets]
        ys = [c[2] for c in tree] + [c[2] for c in targets]
        path = None
        for grow in (prm.window_mm, prm.window_mm * 4):
            m = int(grow / g.res)
            win = (max(min(xs) - m, 0), max(min(ys) - m, 0), min(max(xs) + m, g.W - 1), min(max(ys) + m, g.H - 1))
            path = srch.run(sorted(tree), set(targets), tcent, win)
            if path:
                break
        if not path:
            return None
        segs, vias = _path_to_segs(g, path, net, width)
        all_segs += segs
        all_vias += vias
        tree.update(path)
        hit = remaining[targets[path[-1]]]
        tree.update(_pad_cells(g, hit))
        remaining.remove(hit)
    return RoutedNet(net, all_segs, all_vias, width)


def route(board: Board, cls: Classification, prm: RouteParams, weights: dict, nets: list[str] | None = None,
          progress=None) -> RouteResult:
    g = build_grid(board, cls, prm, weights)
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

    def attempt(g, net, partner):
        kind = cls.kind.get(net, "other")
        pads = [p for p in pads_by_net.get(net, []) if any(l in g.lidx for l in p.layers)]
        widths = [prm.width.get(kind, 0.25)]
        if kind in prm.narrow_width:
            widths.append(prm.narrow_width[kind])
        pf = _pair_band(g, done[partner].segs, done[partner].width, prm) if partner in done else None
        for w in widths:
            res = route_net(g, pads, net, kind, w, prm, pf, weights.get(cls.activity.get(net, "data"), 1.0))
            if res:
                _commit(g, net, kind, res.segs, res.vias, prm)
                done[net] = res
                return True
        return False

    todo = []
    for net, partner in order:
        pads = [p for p in pads_by_net.get(net, []) if any(l in g.lidx for l in p.layers)]
        if len(pads) < 2 or net in already:
            skipped.append(net)
        else:
            todo.append((net, partner))
    for net, partner in todo:
        ok = attempt(g, net, partner)
        if not ok:
            failed.append(net)
        if progress:
            progress(net, ok)

    # rip-up and reroute: free the area around a failed net's pads, route it first, then the others again
    for _ in range(prm.rip_rounds):
        if not failed:
            break
        still = []
        for net in list(failed):
            pads = pads_by_net.get(net, [])
            rr = prm.rip_radius
            boxes = [(p.box()[0] - rr, p.box()[1] - rr, p.box()[2] + rr, p.box()[3] + rr) for p in pads]

            def near(rn):
                return any(any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for b in boxes
                               for x, y in ((s.x1, s.y1), (s.x2, s.y2), ((s.x1 + s.x2) / 2, (s.y1 + s.y2) / 2)))
                           for s in rn.segs)
            victims = sorted(n for n, rn in done.items() if near(rn))
            # keep pairs together
            victims = sorted(set(victims) | {partner_of[v] for v in victims if v in partner_of}
                             | {b for a, b in pairs if a in victims})
            kept = {n: rn for n, rn in done.items() if n not in victims}
            for v in victims:
                done.pop(v, None)
            g = build_grid(board, cls, prm, weights)
            for n, rn in kept.items():
                _commit(g, n, cls.kind.get(n, "other"), rn.segs, rn.vias, prm)
            seq = [net] + [v for v in victims if v not in partner_of] + [v for v in victims if v in partner_of]
            for n in seq:
                ok = attempt(g, n, partner_of.get(n))
                if progress:
                    progress(n, ok)
                if not ok:
                    still.append(n)
        failed = sorted(set(still))
    routed = [done[n] for n, _ in todo if n in done]
    return RouteResult(routed, failed, skipped)
