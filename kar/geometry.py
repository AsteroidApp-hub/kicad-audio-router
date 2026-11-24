"""Pure-Python 2D geometry helpers (no KiCad dependency, unit-testable).

All coordinates are in millimetres.
"""
from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Iterable, Iterator


@dataclass(frozen=True)
class Seg:
    """A straight copper segment on one layer."""
    x1: float
    y1: float
    x2: float
    y2: float
    width: float
    layer: str
    net: str

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def bbox(self, pad: float = 0.0) -> tuple[float, float, float, float]:
        return (min(self.x1, self.x2) - pad, min(self.y1, self.y2) - pad,
                max(self.x1, self.x2) + pad, max(self.y1, self.y2) + pad)


def point_seg_dist(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Distance from point to segment (centre lines)."""
    vx, vy = x2 - x1, y2 - y1
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / l2))
    return math.hypot(x1 + t * vx - px, y1 + t * vy - py)


def _ccw(ax, ay, bx, by, cx, cy) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def segs_intersect(a: Seg, b: Seg) -> bool:
    d1 = _ccw(b.x1, b.y1, b.x2, b.y2, a.x1, a.y1)
    d2 = _ccw(b.x1, b.y1, b.x2, b.y2, a.x2, a.y2)
    d3 = _ccw(a.x1, a.y1, a.x2, a.y2, b.x1, b.y1)
    d4 = _ccw(a.x1, a.y1, a.x2, a.y2, b.x2, b.y2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def seg_seg_dist(a: Seg, b: Seg) -> float:
    """Centre-line distance between two segments."""
    if segs_intersect(a, b):
        return 0.0
    return min(point_seg_dist(a.x1, a.y1, b.x1, b.y1, b.x2, b.y2),
               point_seg_dist(a.x2, a.y2, b.x1, b.y1, b.x2, b.y2),
               point_seg_dist(b.x1, b.y1, a.x1, a.y1, a.x2, a.y2),
               point_seg_dist(b.x2, b.y2, a.x1, a.y1, a.x2, a.y2))


def edge_gap(a: Seg, b: Seg) -> float:
    """Copper edge-to-edge gap between two segments (negative = overlap)."""
    return seg_seg_dist(a, b) - (a.width + b.width) / 2


class SegIndex:
    """Uniform-grid spatial index for segments, per layer."""

    def __init__(self, segs: Iterable[Seg], cell: float = 2.0):
        self.cell = cell
        self.grid: dict[tuple[str, int, int], list[Seg]] = defaultdict(list)
        for s in segs:
            for key in self._cells(s.layer, s.bbox()):
                self.grid[key].append(s)

    def _cells(self, layer: str, bb) -> Iterator[tuple[str, int, int]]:
        c = self.cell
        for i in range(math.floor(bb[0] / c), math.floor(bb[2] / c) + 1):
            for j in range(math.floor(bb[1] / c), math.floor(bb[3] / c) + 1):
                yield (layer, i, j)

    def near(self, layer: str, bb) -> set[Seg]:
        out: set[Seg] = set()
        for key in self._cells(layer, bb):
            out.update(self.grid.get(key, ()))
        return out


def coupled_length(victim: Seg, aggressors: SegIndex, gap: float, step: float = 0.25) -> tuple[float, set[str]]:
    """Length of `victim` that runs within `gap` mm (edge to edge) of any aggressor
    segment on the same layer. Returns (length_mm, aggressor nets involved)."""
    L = victim.length
    if L == 0:
        return 0.0, set()
    reach = gap + victim.width / 2 + 1.0   # +1mm: generous for wide aggressors
    cands = [s for s in aggressors.near(victim.layer, victim.bbox(reach)) if s.net != victim.net]
    if not cands:
        return 0.0, set()
    n = max(1, int(math.ceil(L / step)))
    dl = L / n
    total, nets = 0.0, set()
    for k in range(n):
        t = (k + 0.5) / n
        px = victim.x1 + t * (victim.x2 - victim.x1)
        py = victim.y1 + t * (victim.y2 - victim.y1)
        for s in cands:
            if point_seg_dist(px, py, s.x1, s.y1, s.x2, s.y2) - (s.width + victim.width) / 2 <= gap:
                total += dl
                nets.add(s.net)
                break
    return total, nets
