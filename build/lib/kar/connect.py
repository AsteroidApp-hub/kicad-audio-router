"""Which copper of a net is already connected: groups of pads, tracks and vias that touch.

Used to finish nets that are partly routed (by hand, or left over from an earlier run): the router then
joins the groups instead of starting from the bare pads. Pure Python (zones are not considered).
"""
from __future__ import annotations

from .geometry import Seg, point_seg_dist, seg_seg_dist
from .model import Board, Pad, Via

EPS = 0.01


def _seg_pad(s: Seg, p: Pad) -> bool:
    if s.layer not in p.layers:
        return False
    b = p.box()
    hw = s.width / 2
    if max(s.x1, s.x2) < b[0] - hw or min(s.x1, s.x2) > b[2] + hw or \
            max(s.y1, s.y2) < b[1] - hw or min(s.y1, s.y2) > b[3] + hw:
        return False
    n = max(2, int(s.length / 0.2))
    for k in range(n + 1):
        t = k / n
        if p.dist(s.x1 + t * (s.x2 - s.x1), s.y1 + t * (s.y2 - s.y1)) <= hw + EPS:
            return True
    return False


def _touch(a, b) -> bool:
    if isinstance(a, Seg) and isinstance(b, Seg):
        return a.layer == b.layer and seg_seg_dist(a, b) <= (a.width + b.width) / 2 + EPS
    if isinstance(a, Seg) and isinstance(b, Pad):
        return _seg_pad(a, b)
    if isinstance(a, Pad) and isinstance(b, Seg):
        return _seg_pad(b, a)
    if isinstance(a, Via) and isinstance(b, Seg):
        a, b = b, a
    if isinstance(a, Seg) and isinstance(b, Via):
        return point_seg_dist(b.x, b.y, a.x1, a.y1, a.x2, a.y2) <= b.diameter / 2 + a.width / 2 + EPS
    if isinstance(a, Via) and isinstance(b, Pad):
        a, b = b, a
    if isinstance(a, Pad) and isinstance(b, Via):
        return a.dist(b.x, b.y) <= b.diameter / 2 + EPS
    if isinstance(a, Via) and isinstance(b, Via):
        return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5 <= (a.diameter + b.diameter) / 2 + EPS
    if isinstance(a, Pad) and isinstance(b, Pad):
        if not set(a.layers) & set(b.layers):
            return False
        return a.dist(b.x, b.y) <= EPS or b.dist(a.x, a.y) <= EPS
    return False


def groups(board: Board, net: str) -> list[dict]:
    """Connected groups of the net's copper: [{"pads": [...], "segs": [...], "vias": [...]}, ...].
    A pad with nothing attached is a group of its own."""
    items: list = [p for fp in board.footprints for p in fp.pads if p.net == net]
    items += [s for s in board.segs if s.net == net]
    items += [v for v in board.vias if v.net == net]
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    boxes = []
    for it in items:
        if isinstance(it, Pad):
            boxes.append(it.box())
        elif isinstance(it, Seg):
            boxes.append(it.bbox(it.width / 2 + EPS))
        else:
            r = it.diameter / 2 + EPS
            boxes.append((it.x - r, it.y - r, it.x + r, it.y + r))
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = boxes[i], boxes[j]
            if a[0] > b[2] + 0.5 or b[0] > a[2] + 0.5 or a[1] > b[3] + 0.5 or b[1] > a[3] + 0.5:
                continue
            if find(i) != find(j) and _touch(items[i], items[j]):
                parent[find(i)] = find(j)
    out: dict[int, dict] = {}
    for i, it in enumerate(items):
        g = out.setdefault(find(i), {"pads": [], "segs": [], "vias": []})
        key = "pads" if isinstance(it, Pad) else ("segs" if isinstance(it, Seg) else "vias")
        g[key].append(it)
    # groups without a pad (stray copper) are kept: joining them is harmless and they may carry a pad's link
    return list(out.values())
