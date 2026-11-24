"""Ground handling, planned in pure Python (applied to KiCad by kar.kicad_io).

  plan_fanout   SMD pads of a ground net with no ground via / through-hole ground pad within
                `max_dist` get a short track + via to the inner plane. Search goes outwards from the
                pad, preferring the direction away from the part body, and checks clearance to all
                other-net copper (a via goes through every layer).
  plan_islands  pour islands that contain no via / through-hole pad of the net get one via at the
                point deepest inside the island that is clear of other copper.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import Seg, SegIndex, point_seg_dist
from .model import Board, Pad


@dataclass
class GroundParams:
    net: str = "GND"
    max_dist: float = 3.0         # a pad is OK if a ground via / THT pad is this close
    via_d: float = 0.6
    via_drill: float = 0.3
    track_w: float = 0.3
    clearance: float = 0.2        # to other-net copper
    edge_clearance: float = 0.5   # via edge to board edge
    max_radius: float = 6.0       # how far from the pad to look for a via site
    step: float = 0.3
    directions: int = 24


@dataclass
class ViaPlan:
    x: float
    y: float
    pad: str = ""                       # "U1.4" when it fans out a pad, "" for an island via
    track: tuple[float, float, float, float, str] | None = None   # x1, y1, x2, y2, layer


def _box_dist(b, x, y) -> float:
    dx = max(b[0] - x, 0.0, x - b[2])
    dy = max(b[1] - y, 0.0, y - b[3])
    return math.hypot(dx, dy)


def _point_in_poly(x, y, poly) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _poly_edge_dist(x, y, poly) -> float:
    n = len(poly)
    return min(point_seg_dist(x, y, *poly[i], *poly[(i + 1) % n]) for i in range(n))


class Obstacles:
    """All copper of a board, for clearance queries."""

    def __init__(self, board: Board, cell: float = 2.0):
        self.board = board
        self.cell = cell
        self.segs = SegIndex(board.segs, cell)
        self.layers = list(board.copper_layers)
        self.vias: dict[tuple[int, int], list[tuple[float, float, float, str]]] = {}
        for v in board.vias:
            self._add_via(v.x, v.y, v.diameter / 2, v.net)
        self.pads: dict[tuple[int, int], list[Pad]] = {}
        for fp in board.footprints:
            for p in fp.pads:
                b = p.box()
                for i in range(math.floor(b[0] / cell), math.floor(b[2] / cell) + 1):
                    for j in range(math.floor(b[1] / cell), math.floor(b[3] / cell) + 1):
                        self.pads.setdefault((i, j), []).append(p)

    def _add_via(self, x, y, r, net):
        self.vias.setdefault((math.floor(x / self.cell), math.floor(y / self.cell)), []).append((x, y, r, net))

    def add_via(self, x, y, r, net):
        self._add_via(x, y, r, net)

    def add_seg(self, s: Seg):
        for key in self.segs._cells(s.layer, s.bbox()):
            self.segs.grid[key].append(s)

    def _near_keys(self, x, y, reach):
        c = self.cell
        for i in range(math.floor((x - reach) / c), math.floor((x + reach) / c) + 1):
            for j in range(math.floor((y - reach) / c), math.floor((y + reach) / c) + 1):
                yield (i, j)

    def via_ok(self, x, y, r, net, clearance, own_pad: Pad | None = None) -> bool:
        for ko in self.board.keepouts:
            if ko.get("no_vias") and (_point_in_poly(x, y, ko["poly"]) or _poly_edge_dist(x, y, ko["poly"]) < r):
                return False
        reach = r + clearance + 2.0
        for key in self._near_keys(x, y, reach):
            for p in self.pads.get(key, ()):
                d = p.dist(x, y) - r
                if d < (self.cl(clearance, net, p.net) if p.net != net else 0.05):
                    return False
            for (vx, vy, vr, vn) in self.vias.get(key, ()):
                if math.hypot(vx - x, vy - y) - r - vr < (self.cl(clearance, net, vn) if vn != net else 0.1):
                    return False
        bb = (x - reach, y - reach, x + reach, y + reach)
        for layer in self.layers:
            for s in self.segs.near(layer, bb):
                if s.net != net and point_seg_dist(x, y, s.x1, s.y1, s.x2, s.y2) - s.width / 2 - r < \
                        self.cl(clearance, net, s.net):
                    return False
        return True

    def cl(self, base: float, a: str, b: str) -> float:
        return self.board.clearance(a, b, default=base)

    def track_ok(self, x1, y1, x2, y2, w, layer, net, clearance, own_pad: Pad) -> bool:
        L = math.hypot(x2 - x1, y2 - y1)
        for ko in self.board.keepouts:
            if ko.get("no_tracks") and layer in ko["layers"] and any(
                    _point_in_poly(x1 + (x2 - x1) * t / 8, y1 + (y2 - y1) * t / 8, ko["poly"]) for t in range(9)):
                return False
        n = max(2, int(L / 0.1))
        bb = (min(x1, x2) - 2.5, min(y1, y2) - 2.5, max(x1, x2) + 2.5, max(y1, y2) + 2.5)
        segs = [s for s in self.segs.near(layer, bb) if s.net != net]
        pads = {id(p): p for k in self._near_keys((x1 + x2) / 2, (y1 + y2) / 2, L / 2 + 2.5)
                for p in self.pads.get(k, ()) if p is not own_pad and p.net != net and layer in p.layers}
        vias = [v for k in self._near_keys((x1 + x2) / 2, (y1 + y2) / 2, L / 2 + 2.5)
                for v in self.vias.get(k, ()) if v[3] != net]
        for k in range(n + 1):
            t = k / n
            px, py = x1 + t * (x2 - x1), y1 + t * (y2 - y1)
            for s in segs:
                if point_seg_dist(px, py, s.x1, s.y1, s.x2, s.y2) - s.width / 2 - w / 2 < self.cl(clearance, net, s.net):
                    return False
            for p in pads.values():
                if p.dist(px, py) - w / 2 < self.cl(clearance, net, p.net):
                    return False
            for (vx, vy, vr, vn) in vias:
                if math.hypot(vx - px, vy - py) - vr - w / 2 < self.cl(clearance, net, vn):
                    return False
        return True

    def inside_board(self, x, y, margin) -> bool:
        if not self.board.outline:
            return True
        polys = self.board.outline
        return any(_point_in_poly(x, y, p) for p in polys) and \
            min(_poly_edge_dist(x, y, p) for p in polys) >= margin


def anchors_by_net(board: Board, nets) -> dict[str, list[tuple[float, float]]]:
    """Points where a net reaches the inner layers: its vias and through-hole pads."""
    out: dict[str, list[tuple[float, float]]] = {n: [] for n in nets}
    for v in board.vias:
        if v.net in out:
            out[v.net].append((v.x, v.y))
    for fp in board.footprints:
        for p in fp.pads:
            if p.net in out and not p.smd:
                out[p.net].append((p.x, p.y))
    return out


def _anchors(board: Board, net: str) -> list[tuple[float, float]]:
    return anchors_by_net(board, [net])[net]


def anchor_circles(board: Board, net: str) -> list[tuple[float, float, float]]:
    """(x, y, copper radius) of the net's vias and through-hole pads."""
    out = [(v.x, v.y, v.diameter / 2) for v in board.vias if v.net == net]
    for fp in board.footprints:
        for p in fp.pads:
            if p.net == net and not p.smd:
                b = p.box()
                out.append((p.x, p.y, max(b[2] - b[0], b[3] - b[1]) / 2))
    return out


def island_anchored(poly, circles, tol: float = 0.6) -> bool:
    """True when a via / THT pad lies in the island or right at its edge. Filled-zone polygons are
    'fractured' (holes such as thermal-relief gaps are cut into the outline), so a pad sitting in a
    thermal hole is outside the polygon but within `radius + tol` of its edge."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    for x, y, r in circles:
        if x < x0 - r - tol or x > x1 + r + tol or y < y0 - r - tol or y > y1 + r + tol:
            continue
        if _point_in_poly(x, y, poly) or _poly_edge_dist(x, y, poly) <= r + tol:
            return True
    return False


def anchor_dist(pad: Pad, anchors: list[tuple[float, float]]) -> float:
    """Distance from the pad's copper edge to the nearest anchor point."""
    b = pad.box()
    return min((_box_dist(b, x, y) for x, y in anchors), default=math.inf)


def plan_fanout(board: Board, prm: GroundParams, obst: Obstacles | None = None) -> tuple[list[ViaPlan], list[str]]:
    obst = obst or Obstacles(board)
    anchors = _anchors(board, prm.net)
    r = prm.via_d / 2
    plans, failed = [], []
    for fp in sorted(board.footprints, key=lambda f: f.ref):
        cx = sum(p.x for p in fp.pads) / max(len(fp.pads), 1)
        cy = sum(p.y for p in fp.pads) / max(len(fp.pads), 1)
        for p in sorted(fp.pads, key=lambda q: q.number):
            if p.net != prm.net or not p.smd:
                continue
            if anchor_dist(p, anchors) <= prm.max_dist:
                continue
            layer = p.layers[0] if p.layers else "F.Cu"
            out = math.atan2(p.y - cy, p.x - cx) if (p.x, p.y) != (cx, cy) else 0.0
            angs = sorted((2 * math.pi * k / prm.directions for k in range(prm.directions)),
                          key=lambda a: (abs(math.remainder(a - out, 2 * math.pi)), a))
            b = p.box()
            r0 = max(b[2] - b[0], b[3] - b[1]) / 2 + r + 0.1
            done = None
            rr = r0
            while rr <= prm.max_radius + 1e-9 and not done:
                for a in angs:
                    x, y = p.x + rr * math.cos(a), p.y + rr * math.sin(a)
                    if not obst.inside_board(x, y, r + prm.edge_clearance):
                        continue
                    if not obst.via_ok(x, y, r, prm.net, prm.clearance, p):
                        continue
                    if not obst.track_ok(p.x, p.y, x, y, prm.track_w, layer, prm.net, prm.clearance, p):
                        continue
                    done = ViaPlan(round(x, 4), round(y, 4), f"{p.ref}.{p.number}", (p.x, p.y, x, y, layer))
                    break
                rr += prm.step
            if done:
                plans.append(done)
                obst.add_via(done.x, done.y, r, prm.net)
                obst.add_seg(Seg(p.x, p.y, done.x, done.y, prm.track_w, layer, prm.net))
                anchors.append((done.x, done.y))
            else:
                failed.append(f"{p.ref}.{p.number}")
    return plans, failed


def plan_islands(board: Board, islands: list[list[tuple[float, float]]], prm: GroundParams,
                 obst: Obstacles | None = None, grid: float = 0.1,
                 failed_polys: list | None = None) -> tuple[list[ViaPlan], int]:
    """`islands`: polygons of pour islands of prm.net. Returns (vias, islands that could not get one);
    the polygons that could not get one are appended to `failed_polys` when given."""
    obst = obst or Obstacles(board)
    circles = anchor_circles(board, prm.net)
    r = prm.via_d / 2
    plans, failed = [], 0
    for poly in islands:
        if island_anchored(poly, circles):
            continue
        xs, ys = [p[0] for p in poly], [p[1] for p in poly]
        best = None
        x = min(xs)
        while x <= max(xs):
            y = min(ys)
            while y <= max(ys):
                if _point_in_poly(x, y, poly):
                    d = _poly_edge_dist(x, y, poly)
                    if d >= r and (best is None or d > best[0]) and obst.via_ok(x, y, r, prm.net, prm.clearance):
                        best = (d, x, y)
                y += grid
            x += grid
        if best:
            plans.append(ViaPlan(round(best[1], 4), round(best[2], 4)))
            obst.add_via(best[1], best[2], r, prm.net)
            circles.append((best[1], best[2], r))
        else:
            failed += 1
            if failed_polys is not None:
                failed_polys.append(poly)
    return plans, failed


def pads_on_island(board: Board, poly, net: str) -> list[str]:
    """SMD pads of `net` lying on (or touching) a pour island."""
    out = []
    for fp in board.footprints:
        for p in fp.pads:
            if p.net == net and p.smd and (_point_in_poly(p.x, p.y, poly) or _poly_edge_dist(p.x, p.y, poly) < 0.3):
                out.append(f"{p.ref}.{p.number}")
    return out
