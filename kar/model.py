"""Plain data model of a board. Everything except `kar.kicad_io` works on this model,
so classification and reports can be tested without KiCad."""
from __future__ import annotations

from dataclasses import dataclass, field

from .geometry import Seg


@dataclass
class Pad:
    ref: str
    number: str
    net: str
    x: float
    y: float
    smd: bool
    layers: tuple[str, ...]          # copper layers the pad is on, e.g. ("F.Cu",) or all for THT
    pin_function: str = ""           # pin name from the schematic, if any
    pin_type: str = ""               # electrical type from the schematic, if any
    bbox: tuple[float, float, float, float] | None = None   # copper extent (x0, y0, x1, y1)
    poly: list[tuple[float, float]] | None = None             # copper outline (mm), if known

    def box(self) -> tuple[float, float, float, float]:
        return self.bbox or (self.x - 0.3, self.y - 0.3, self.x + 0.3, self.y + 0.3)

    def contains(self, x: float, y: float, inset: float = 0.0) -> bool:
        """Is (x, y) on the pad copper, at least `inset` inside its edge?"""
        b = self.box()
        if not (b[0] + inset <= x <= b[2] - inset and b[1] + inset <= y <= b[3] - inset):
            return False
        if not self.poly:
            return True
        from .geometry import point_in_poly, point_poly_dist
        return point_in_poly(x, y, self.poly) and (inset <= 0 or point_poly_dist(x, y, self.poly) >= inset)

    def dist(self, x: float, y: float) -> float:
        """Distance from (x, y) to the pad copper (0 inside)."""
        b = self.box()
        dx = max(b[0] - x, 0.0, x - b[2])
        dy = max(b[1] - y, 0.0, y - b[3])
        if not self.poly:
            return (dx * dx + dy * dy) ** 0.5
        from .geometry import edges_dist2, point_in_poly, poly_edges
        e = self.__dict__.get("_edges")
        if e is None:
            e = self.__dict__["_edges"] = poly_edges(self.poly)
        if (dx > 0 or dy > 0) or not point_in_poly(x, y, self.poly):
            return edges_dist2(x, y, e) ** 0.5
        return 0.0


@dataclass
class Footprint:
    ref: str
    value: str
    fp_name: str                     # library footprint name, e.g. "SOIC-8_3.9x4.9mm_P1.27mm"
    pads: list[Pad] = field(default_factory=list)

    @property
    def prefix(self) -> str:
        return "".join(ch for ch in self.ref if ch.isalpha())


@dataclass
class Via:
    x: float
    y: float
    net: str
    diameter: float
    drill: float


@dataclass
class Board:
    footprints: list[Footprint]
    segs: list[Seg]
    vias: list[Via]
    copper_layers: list[str]
    zones: list[tuple[str, str]] = field(default_factory=list)   # (net, layer) of filled zones
    unrouted: int | None = None
    source: str = ""
    outline: list[list[tuple[float, float]]] = field(default_factory=list)   # board outline polygon(s)
    plane_layers: list[str] = field(default_factory=list)   # inner layers typed "power" in the board setup
    net_clearance: dict[str, float] = field(default_factory=dict)   # net -> clearance from its net class
    keepouts: list[dict] = field(default_factory=list)   # rule areas: {"poly", "layers", "no_tracks", "no_vias"}
    hole_clearance: float = 0.0      # board rule: copper to holes (used for pads without a net, e.g. NPTH)
    net_class: dict[str, str] = field(default_factory=dict)        # net -> net class name (project file)
    netclasses: dict[str, dict] = field(default_factory=dict)      # class -> {"track_width", "via_diameter", ...}
    zone_polys: list = field(default_factory=list)   # (net, layer, outline polygon) of copper zones

    def class_of(self, net: str) -> dict | None:
        """The net's own net class settings, None for nets in the Default class (no explicit intent)."""
        c = self.net_class.get(net)
        if not c or c == "Default":
            return None
        return self.netclasses.get(c)

    def clearance(self, *nets: str, default: float = 0.2) -> float:
        """Clearance needed between copper of these nets (the larger rule wins). A pad without a net
        (mounting / NPTH hole) needs the board's hole clearance."""
        return max([default] + [self.net_clearance.get(n, 0.0) if n else self.hole_clearance for n in nets])

    @property
    def nets(self) -> set[str]:
        out = {p.net for fp in self.footprints for p in fp.pads if p.net}
        out |= {s.net for s in self.segs if s.net}
        return out

    def pads_by_net(self) -> dict[str, list[Pad]]:
        d: dict[str, list[Pad]] = {}
        for fp in self.footprints:
            for p in fp.pads:
                if p.net:
                    d.setdefault(p.net, []).append(p)
        return d

    def fp_by_ref(self) -> dict[str, Footprint]:
        return {fp.ref: fp for fp in self.footprints}
