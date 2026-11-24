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
    b = from_pcbnew(kb, path)
    read_netclasses(b, path)
    return b, kb


def read_netclasses(board: Board, pcb_path: str) -> None:
    """Net classes from the project file next to the board (<name>.kicad_pro): widths and via sizes,
    explicit assignments and wildcard patterns."""
    import fnmatch
    import json
    import os
    pro = os.path.splitext(pcb_path)[0] + ".kicad_pro"
    if not os.path.exists(pro):
        return
    try:
        with open(pro, encoding="utf-8") as f:
            ns = json.load(f).get("net_settings", {})
    except Exception:   # noqa: BLE001
        return
    for c in ns.get("classes", []):
        board.netclasses[c.get("name", "")] = {k: c.get(k) for k in ("track_width", "clearance", "via_diameter",
                                                                     "via_drill", "diff_pair_width", "diff_pair_gap")}
    assign = ns.get("netclass_assignments") or {}
    pats = ns.get("netclass_patterns") or []
    for n in board.nets:
        a = assign.get(n) if isinstance(assign, dict) else None
        if isinstance(a, list):
            a = a[0] if a else None
        if not a:
            for pt in pats:
                if fnmatch.fnmatchcase(n, pt.get("pattern", "")):
                    a = pt.get("netclass")
                    break
        if a:
            board.net_class[n] = a


def from_pcbnew(kb, source: str = "") -> Board:
    copper = [kb.GetLayerName(l) for l in kb.GetEnabledLayers().CuStack()]
    fps = []
    ncl: dict[str, float] = {}
    for fp in kb.GetFootprints():
        f = Footprint(fp.GetReference(), fp.GetValue(), str(fp.GetFPID().GetLibItemName()))
        for p in fp.Pads():
            pos = p.GetPosition()
            layers = tuple(n for n in (kb.GetLayerName(l) for l in p.GetLayerSet().CuStack()) if n in copper)
            pf = p.GetPinFunction() if hasattr(p, "GetPinFunction") else ""
            pt = p.GetPinType() if hasattr(p, "GetPinType") else ""
            if p.GetNetname():
                try:
                    c = _mm(p.GetOwnClearance(p.GetLayerSet().CuStack()[0]))
                    ncl[p.GetNetname()] = max(ncl.get(p.GetNetname(), 0.0), c)
                except Exception:   # noqa: BLE001
                    pass
            bb = p.GetBoundingBox()
            f.pads.append(Pad(fp.GetReference(), p.GetNumber(), p.GetNetname(), _mm(pos.x), _mm(pos.y),
                              p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD, layers, pf, pt,
                              (_mm(bb.GetLeft()), _mm(bb.GetTop()), _mm(bb.GetRight()), _mm(bb.GetBottom())),
                              _pad_poly(p)))
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
    zones, keepouts = [], []
    for z in kb.Zones():
        if z.GetIsRuleArea():
            o = z.Outline()
            for i in range(o.OutlineCount()):
                ol = o.Outline(i)
                keepouts.append({"poly": [(_mm(ol.CPoint(k).x), _mm(ol.CPoint(k).y)) for k in range(ol.PointCount())],
                                 "layers": [kb.GetLayerName(l) for l in z.GetLayerSet().CuStack()],
                                 "no_tracks": bool(z.GetDoNotAllowTracks()), "no_vias": bool(z.GetDoNotAllowVias()),
                                 "no_pads": bool(z.GetDoNotAllowPads())})
            continue
        for l in z.GetLayerSet().CuStack():
            zones.append((z.GetNetname(), kb.GetLayerName(l)))
    # planes: inner layers whose type in the board setup is "power" (Board Setup > Layers)
    planes = []
    for l in kb.GetEnabledLayers().CuStack():
        try:
            if kb.GetLayerType(l) == pcbnew.LT_POWER and kb.GetLayerName(l) in copper[1:-1]:
                planes.append(kb.GetLayerName(l))
        except Exception:   # noqa: BLE001
            pass
    try:
        hole_cl = _mm(kb.GetDesignSettings().m_HoleClearance)
    except Exception:   # noqa: BLE001
        hole_cl = 0.0
    return Board(fps, segs, vias, copper, zones, _unrouted(kb), source, _outline(kb), planes, ncl, keepouts,
                 hole_cl)


def _area(poly) -> float:
    n = len(poly)
    return abs(sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))) / 2


def _pad_poly(p) -> list[tuple[float, float]] | None:
    """Copper outline of a pad (first layer), or None if this KiCad version cannot give it.
    Arcs are approximated on the outside (ERROR_OUTSIDE), so distances to it are never overestimated."""
    ps = None
    err = getattr(pcbnew, "ERROR_OUTSIDE", pcbnew.ERROR_INSIDE)
    for args in ((p.GetLayerSet().CuStack()[0] if p.GetLayerSet().CuStack() else pcbnew.F_Cu, err),
                 (err,)):
        try:
            ps = p.GetEffectivePolygon(*args)
            break
        except Exception:   # noqa: BLE001
            continue
    try:
        if ps is None or ps.OutlineCount() == 0:
            return None
        o = ps.Outline(0)
        return [(_mm(o.CPoint(k).x), _mm(o.CPoint(k).y)) for k in range(o.PointCount())]
    except Exception:   # noqa: BLE001
        return None


def _outline(kb) -> list[list[tuple[float, float]]]:
    ps = pcbnew.SHAPE_POLY_SET()
    try:
        kb.GetBoardPolygonOutlines(ps)
    except TypeError:
        kb.GetBoardPolygonOutlines(ps, False)
    out = []
    for i in range(ps.OutlineCount()):
        o = ps.Outline(i)
        out.append([(_mm(o.CPoint(k).x), _mm(o.CPoint(k).y)) for k in range(o.PointCount())])
    return out


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


# ---------------------------------------------------------------- writing

def _from_mm(v: float) -> int:
    return pcbnew.FromMM(v)


def layer_id(kb, name: str) -> int:
    lid = kb.GetLayerID(name)
    if lid < 0:
        raise ValueError(f"unknown layer {name}")
    return lid


def add_pour(kb, net: str, layer: str, outline: list[tuple[float, float]], *, name: str = "",
             clearance: float = 0.3, min_width: float = 0.25, priority: int = 0,
             tht_thermal: bool = True, inset: float = 0.3):
    """Pour `net` on `layer` over the board outline (shrunk by `inset`). SMD pads connect solid,
    through-hole pads with thermal spokes when tht_thermal."""
    z = pcbnew.ZONE(kb)
    z.SetLayer(layer_id(kb, layer))
    z.SetNet(kb.FindNet(net))
    z.SetZoneName(name or f"{net}_{layer.split('.')[0]}")
    ps = pcbnew.SHAPE_POLY_SET()
    ps.NewOutline()
    for x, y in outline:
        ps.Append(_from_mm(x), _from_mm(y))
    if inset:
        try:
            ps.Inflate(-_from_mm(inset), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, _from_mm(0.01))
        except Exception:   # noqa: BLE001  (API differs between KiCad versions)
            try:
                ps.Inflate(-_from_mm(inset), 16)
            except Exception:   # noqa: BLE001
                pass
    z.Outline().RemoveAllContours()
    for i in range(ps.OutlineCount()):
        z.Outline().AddOutline(ps.Outline(i))
    z.SetLocalClearance(_from_mm(clearance))
    z.SetMinThickness(_from_mm(min_width))
    z.SetThermalReliefGap(_from_mm(0.3))
    z.SetThermalReliefSpokeWidth(_from_mm(0.4))
    z.SetAssignedPriority(priority)
    for setter, val in (("SetIslandRemovalMode", "ISLAND_REMOVAL_MODE_ALWAYS"),
                        ("SetPadConnection", "ZONE_CONNECTION_THT_THERMAL" if tht_thermal else "ZONE_CONNECTION_FULL")):
        try:
            getattr(z, setter)(getattr(pcbnew, val))
        except Exception:   # noqa: BLE001
            pass
    kb.Add(z)
    return z


def refill(kb) -> None:
    pcbnew.ZONE_FILLER(kb).Fill(kb.Zones())


def islands(kb, net: str, layer: str) -> list[list[tuple[float, float]]]:
    """Filled-copper islands of `net` on `layer` (outer contours, mm)."""
    lid = layer_id(kb, layer)
    out = []
    for z in kb.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != net or not z.IsOnLayer(lid):
            continue
        poly = z.GetFilledPolysList(lid)
        for i in range(poly.OutlineCount()):
            o = poly.Outline(i)
            out.append([(_mm(o.CPoint(k).x), _mm(o.CPoint(k).y)) for k in range(o.PointCount())])
    return out


def apply_vias(kb, plans, net: str, via_d: float, drill: float, track_w: float,
               group_name: str | None = "kar: ground") -> int:
    n = kb.FindNet(net)
    grp = group(kb, group_name) if group_name and plans else None
    for p in plans:
        v = pcbnew.PCB_VIA(kb)
        v.SetPosition(pcbnew.VECTOR2I(_from_mm(p.x), _from_mm(p.y)))
        v.SetDrill(_from_mm(drill))
        try:
            v.SetWidth(pcbnew.F_Cu, _from_mm(via_d))
        except TypeError:
            v.SetWidth(_from_mm(via_d))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(n)
        kb.Add(v)
        if grp is not None:
            grp.AddItem(v)
        if p.track:
            x1, y1, x2, y2, layer = p.track
            t = pcbnew.PCB_TRACK(kb)
            t.SetStart(pcbnew.VECTOR2I(_from_mm(x1), _from_mm(y1)))
            t.SetEnd(pcbnew.VECTOR2I(_from_mm(x2), _from_mm(y2)))
            t.SetWidth(_from_mm(track_w))
            t.SetLayer(layer_id(kb, layer))
            t.SetNet(n)
            kb.Add(t)
            if grp is not None:
                grp.AddItem(t)
    return len(plans)


def save(kb, path: str, src: str | None = None) -> None:
    """Save the board. The project file (design rules, net classes) of `src` is copied next to it, so
    the saved board is checked against the same rules (SaveBoard alone may write default settings)."""
    import os
    import shutil
    pcbnew.SaveBoard(path, kb)
    if src:
        sp = os.path.splitext(src)[0] + ".kicad_pro"
        dp = os.path.splitext(path)[0] + ".kicad_pro"
        if os.path.exists(sp) and os.path.abspath(sp) != os.path.abspath(dp):
            shutil.copyfile(sp, dp)


def group(kb, name: str):
    """A PCB group by name (created if missing), so kar's copper can be selected in one click."""
    for g in kb.Groups():
        if g.GetName() == name:
            return g
    g = pcbnew.PCB_GROUP(kb)
    g.SetName(name)
    kb.Add(g)
    return g


def apply_routes(kb, routed, via_d: float, drill: float, lock: bool = True, via_of=None,
                 group_of=None) -> tuple[int, int]:
    """Write RoutedNet results (kar.router) to the board. Locked tracks are kept by Freerouting
    (KiCad exports them as protected wiring). `group_of(net)` names the PCB group for the net's copper
    (e.g. "kar: analog"); None leaves it ungrouped."""
    nt = nv = 0
    for r in routed:
        n = kb.FindNet(r.net)
        grp = group(kb, group_of(r.net)) if group_of else None
        for s in r.segs:
            t = pcbnew.PCB_TRACK(kb)
            t.SetStart(pcbnew.VECTOR2I(_from_mm(s.x1), _from_mm(s.y1)))
            t.SetEnd(pcbnew.VECTOR2I(_from_mm(s.x2), _from_mm(s.y2)))
            t.SetWidth(_from_mm(s.width))
            t.SetLayer(layer_id(kb, s.layer))
            t.SetNet(n)
            t.SetLocked(lock)
            kb.Add(t)
            if grp is not None:
                grp.AddItem(t)
            nt += 1
        vd, vdr = via_of(r.net) if via_of else (via_d, drill)
        for x, y in r.vias:
            v = pcbnew.PCB_VIA(kb)
            v.SetPosition(pcbnew.VECTOR2I(_from_mm(x), _from_mm(y)))
            v.SetDrill(_from_mm(vdr))
            try:
                v.SetWidth(pcbnew.F_Cu, _from_mm(vd))
            except TypeError:
                v.SetWidth(_from_mm(vd))
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNet(n)
            v.SetLocked(lock)
            kb.Add(v)
            if grp is not None:
                grp.AddItem(v)
            nv += 1
    return nt, nv


def strip(kb, keep_locked: bool = False) -> int:
    """Remove tracks and vias (for re-routing experiments)."""
    gone = [t for t in kb.GetTracks() if not (keep_locked and t.IsLocked())]
    for t in gone:
        kb.Delete(t)
    return len(gone)
