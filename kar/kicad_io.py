"""KiCad (pcbnew) <-> kar.model. The only module that imports pcbnew.

Run with the Python bundled with KiCad (7, 8, 9 or 10), e.g. on macOS:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
"""
from __future__ import annotations

from .geometry import Seg
from .model import Board, Footprint, Pad, Via

try:
    import pcbnew  # type: ignore
except ImportError as e:   # pragma: no cover
    raise SystemExit("pcbnew が見つかりません。KiCad 同梱の Python で実行してください "
                     "(pcbnew not found: run with the Python bundled with KiCad).") from e


def _mm(v) -> float:
    return pcbnew.ToMM(v)


def _via_width(v) -> float:
    try:
        return _mm(v.GetWidth(pcbnew.F_Cu))
    except TypeError:
        return _mm(v.GetWidth())


def load(path: str) -> tuple[Board, "pcbnew.BOARD"]:
    kb = pcbnew.LoadBoard(path)
    return from_pcbnew(kb, path), kb


def from_pcbnew(kb, source: str = "") -> Board:
    copper = [kb.GetLayerName(l) for l in kb.GetEnabledLayers().CuStack()]
    fps = []
    for fp in kb.GetFootprints():
        f = Footprint(fp.GetReference(), fp.GetValue(), str(fp.GetFPID().GetLibItemName()))
        for p in fp.Pads():
            pos = p.GetPosition()
            layers = tuple(kb.GetLayerName(l) for l in p.GetLayerSet().CuStack())
            pf = p.GetPinFunction() if hasattr(p, "GetPinFunction") else ""
            pt = p.GetPinType() if hasattr(p, "GetPinType") else ""
            f.pads.append(Pad(fp.GetReference(), p.GetNumber(), p.GetNetname(), _mm(pos.x), _mm(pos.y),
                              p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD, layers, pf, pt))
        fps.append(f)
    segs, vias = [], []
    for t in kb.GetTracks():
        cls = t.GetClass()
        if cls == "PCB_VIA":
            p = t.GetPosition()
            vias.append(Via(_mm(p.x), _mm(p.y), t.GetNetname(), _via_width(t), _mm(t.GetDrillValue())))
        elif cls in ("PCB_TRACK", "PCB_ARC"):
            # arcs are approximated by their chord (good enough for metrics)
            a, b = t.GetStart(), t.GetEnd()
            segs.append(Seg(_mm(a.x), _mm(a.y), _mm(b.x), _mm(b.y), _mm(t.GetWidth()),
                            kb.GetLayerName(t.GetLayer()), t.GetNetname()))
    zones = []
    for z in kb.Zones():
        if z.GetIsRuleArea():
            continue
        for l in z.GetLayerSet().CuStack():
            zones.append((z.GetNetname(), kb.GetLayerName(l)))
    return Board(fps, segs, vias, copper, zones, _unrouted(kb), source)


def _unrouted(kb) -> int | None:
    try:
        kb.BuildConnectivity()
        conn = kb.GetConnectivity()
        for args in ((False,), ()):
            try:
                return int(conn.GetUnconnectedCount(*args))
            except TypeError:
                continue
    except Exception:   # noqa: BLE001
        return None
    return None
