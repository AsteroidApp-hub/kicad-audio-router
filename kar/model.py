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

    def box(self) -> tuple[float, float, float, float]:
        return self.bbox or (self.x - 0.3, self.y - 0.3, self.x + 0.3, self.y + 0.3)


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
