"""SVG picture of a board's routing, one panel per copper layer, tracks coloured by net class.
No dependencies: the SVG is written by hand."""
from __future__ import annotations

from xml.sax.saxutils import escape

from .classify import Classification
from .model import Board

COLOURS = {"analog": "#d62728", "digital": "#1f77b4", "power": "#ff7f0e", "gnd": "#2ca02c", "other": "#7f7f7f",
           "ignore": "#bbbbbb"}


def svg(board: Board, cls: Classification, layers: list[str] | None = None, scale: float = 6.0,
        title: str = "") -> str:
    layers = layers or [l for l in board.copper_layers if l not in board.plane_layers]
    xs = [x for poly in board.outline for x, _ in poly] or [p.x for fp in board.footprints for p in fp.pads]
    ys = [y for poly in board.outline for _, y in poly] or [p.y for fp in board.footprints for p in fp.pads]
    x0, y0, x1, y1 = min(xs) - 1, min(ys) - 1, max(xs) + 1, max(ys) + 1
    w, h = (x1 - x0), (y1 - y0)
    gap = 4.0
    head = 10.0 if title else 6.0
    total_w = len(layers) * w + (len(layers) - 1) * gap
    total_h = h + head + 8
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{total_w * scale:.0f}" height="{total_h * scale:.0f}" '
           f'viewBox="0 0 {total_w:.2f} {total_h:.2f}" font-family="sans-serif">',
           f'<rect width="100%" height="100%" fill="#ffffff"/>']
    if title:
        out.append(f'<text x="0.5" y="3.2" font-size="3">{escape(title)}</text>')
    for k, layer in enumerate(layers):
        ox = k * (w + gap) - x0
        oy = head - y0
        out.append(f'<g transform="translate({ox:.3f},{oy:.3f})">')
        out.append(f'<text x="{x0 + 0.5:.2f}" y="{y0 - 1:.2f}" font-size="2.6">{escape(layer)}</text>')
        for poly in board.outline:
            pts = " ".join(f"{x:.3f},{y:.3f}" for x, y in poly)
            out.append(f'<polygon points="{pts}" fill="#fbfaf6" stroke="#333" stroke-width="0.15"/>')
        for fp in board.footprints:
            for p in fp.pads:
                if layer not in p.layers:
                    continue
                if p.poly:
                    pts = " ".join(f"{x:.3f},{y:.3f}" for x, y in p.poly)
                    out.append(f'<polygon points="{pts}" fill="#d4d4d4"/>')
                else:
                    b = p.box()
                    out.append(f'<rect x="{b[0]:.3f}" y="{b[1]:.3f}" width="{b[2] - b[0]:.3f}" '
                               f'height="{b[3] - b[1]:.3f}" fill="#d4d4d4"/>')
        for s in board.segs:
            if s.layer != layer:
                continue
            c = COLOURS.get(cls.kind.get(s.net, "other"), "#7f7f7f")
            out.append(f'<line x1="{s.x1:.3f}" y1="{s.y1:.3f}" x2="{s.x2:.3f}" y2="{s.y2:.3f}" stroke="{c}" '
                       f'stroke-width="{max(s.width, 0.12):.3f}" stroke-linecap="round"/>')
        for v in board.vias:
            c = COLOURS.get(cls.kind.get(v.net, "other"), "#7f7f7f")
            out.append(f'<circle cx="{v.x:.3f}" cy="{v.y:.3f}" r="{v.diameter / 2:.3f}" fill="{c}" '
                       f'stroke="#fff" stroke-width="0.08"/>')
        out.append("</g>")
    # legend
    lx, ly = 0.5, total_h - 2.5
    for name, c in (("analog", COLOURS["analog"]), ("digital", COLOURS["digital"]), ("power", COLOURS["power"]),
                    ("ground", COLOURS["gnd"]), ("other", COLOURS["other"])):
        out.append(f'<rect x="{lx:.2f}" y="{ly - 1.6:.2f}" width="2" height="2" fill="{c}"/>')
        out.append(f'<text x="{lx + 2.6:.2f}" y="{ly:.2f}" font-size="2.2">{name}</text>')
        lx += 14
    out.append("</svg>")
    return "\n".join(out) + "\n"
