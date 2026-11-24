"""Tests that run without KiCad:  python3 -m pytest tests  (or python3 tests/test_core.py)"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kar import rules as R                               # noqa: E402
from kar.classify import classify                        # noqa: E402
from kar.geometry import Seg, SegIndex, coupled_length, edge_gap, seg_seg_dist  # noqa: E402
from kar.model import Board, Footprint, Pad, Via         # noqa: E402
from kar.report import build                             # noqa: E402


def S(x1, y1, x2, y2, net, w=0.2, layer="F.Cu"):
    return Seg(x1, y1, x2, y2, w, layer, net)


def test_seg_distance():
    assert seg_seg_dist(S(0, 0, 10, 0, "a"), S(0, 1, 10, 1, "b")) == 1.0
    assert seg_seg_dist(S(0, 0, 10, 10, "a"), S(0, 10, 10, 0, "b")) == 0.0       # crossing
    assert math.isclose(edge_gap(S(0, 0, 10, 0, "a", 0.2), S(0, 1, 10, 1, "b", 0.4)), 0.7)


def test_coupled_length_parallel_and_layer():
    idx = SegIndex([S(2, 0.5, 8, 0.5, "dig"), S(0, 0.5, 10, 0.5, "dig2", layer="B.Cu")])
    l, nets = coupled_length(S(0, 0, 10, 0, "ana"), idx, gap=0.31)   # edge gap is 0.3
    assert math.isclose(l, 6.0, abs_tol=0.3) and nets == {"dig"}               # other layer ignored
    l, _ = coupled_length(S(0, 0, 10, 0, "ana"), idx, gap=0.1)
    assert l == 0.0


def _pad(ref, num, net, x=0.0, y=0.0, smd=True, fn=""):
    return Pad(ref, num, net, x, y, smd, ("F.Cu",), fn)


def _board():
    u1 = Footprint("U1", "OPA1612", "SOIC-8", [_pad("U1", "1", "/OUT"), _pad("U1", "4", "/-15V"),
                                               _pad("U1", "3", "/IN_P")])
    u2 = Footprint("U2", "74HC595", "SOIC-16", [_pad("U2", "11", "/SCLK"), _pad("U2", "15", "/QA")])
    r1 = Footprint("R1", "10k", "R_0805", [_pad("R1", "1", "/OUT"), _pad("R1", "2", "/NEXT")])
    j1 = Footprint("J1", "XLR", "Jack_XLR_Neutrik", [_pad("J1", "2", "/IN_H", smd=False),
                                                       _pad("J1", "3", "/IN_C", smd=False)])
    c1 = Footprint("C1", "100n", "C_0805", [_pad("C1", "1", "/+15V", 5, 5), _pad("C1", "2", "GND", 5, 6)])
    segs = [S(0, 0, 20, 0, "/OUT"), S(0, 0.6, 20, 0.6, "/SCLK"), S(0, 5, 20, 5, "/IN_H"),
            S(0, 5.5, 20, 5.5, "/IN_C"), S(0, 9, 10, 9, "/+15V", w=0.2)]
    return Board([u1, u2, r1, j1, c1], segs, [Via(5, 6.5, "GND", 0.6, 0.3)], ["F.Cu", "B.Cu"], unrouted=0)


def test_classify():
    cls = classify(_board(), R.load(None))
    k = cls.kind
    assert k["/OUT"] == "analog" and k["/IN_P"] == "analog"
    assert k["/NEXT"] == "analog"                       # spread through R1
    assert k["/SCLK"] == "digital" and k["/QA"] == "digital"
    assert k["/-15V"] == "power" and k["GND"] == "gnd"
    assert k["/IN_H"] == "analog" and ("/IN_H", "/IN_C") in cls.pairs
    assert cls.activity["/SCLK"] == "clock" and cls.activity["/QA"] == "data"


def test_rules_override():
    rules = R.load(None)
    rules["classes"]["digital"] = ["/NEXT"]
    assert classify(_board(), rules).kind["/NEXT"] == "digital"


def test_report():
    b = _board()
    rep = build(b, classify(b, R.load(None)), R.load(None)).data
    assert rep["coupling"]["worst"][0]["net"] == "/OUT"
    assert rep["coupling"]["total_mm"]["1.0"] > 19
    assert rep["coupling"]["by_activity_mm"]["clock"]["1.0"] > 19     # SCLK is a clock
    assert rep["coupling"]["weighted_mm"]["1.0"] == rep["coupling"]["total_mm"]["1.0"]   # weight 1.0
    assert rep["pairs"]["items"][0]["parallel_pct"] > 90
    assert rep["power"]["narrow"][0]["net"] == "/+15V"
    assert rep["gnd"]["far"] == []                      # GND pad 0.5 mm from the via
    assert 0 <= rep["score"] <= 100


def _ground_board():
    from kar.model import Board, Footprint, Pad
    sq = [(0, 0), (40, 0), (40, 40), (0, 40)]
    c1 = Footprint("C1", "100n", "C_0805", [
        Pad("C1", "1", "/V", 20.0, 20.0, True, ("F.Cu",), bbox=(19.5, 19.4, 20.5, 20.6)),
        Pad("C1", "2", "GND", 21.9, 20.0, True, ("F.Cu",), bbox=(21.4, 19.4, 22.4, 20.6))])
    # a signal track hugging the right of the GND pad on every layer would block the via; leave room below
    segs = [S(23.4, 15, 23.4, 25, "/SIG", layer=l) for l in ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")]
    return Board([c1], segs, [], ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"], outline=[sq])


def test_ground_fanout():
    from kar.ground import GroundParams, Obstacles, plan_fanout, _box_dist
    b = _ground_board()
    plans, failed = plan_fanout(b, GroundParams())
    assert not failed and len(plans) == 1
    v = plans[0]
    assert v.pad == "C1.2" and v.track[4] == "F.Cu"
    # clearance to the signal track (x=23.4, w=0.2) and to the other pad
    assert abs(v.x - 23.4) - 0.1 - 0.3 >= 0.2 - 1e-9 or not (15 <= v.y <= 25)
    assert _box_dist((19.5, 19.4, 20.5, 20.6), v.x, v.y) - 0.3 >= 0.2 - 1e-9
    # a second run with the new via present needs nothing
    from kar.model import Via
    b.vias.append(Via(v.x, v.y, "GND", 0.6, 0.3))
    assert plan_fanout(b, GroundParams())[0] == []


def test_island_anchor_in_thermal_hole():
    from kar.ground import island_anchored
    ring_cut = [(0, 0), (3, 0), (3, 3), (0, 3)]          # island whose outline does not contain the pad
    assert island_anchored(ring_cut, [(3.6, 1.5, 0.8)])   # THT pad just outside, inside tolerance
    assert not island_anchored(ring_cut, [(6.0, 1.5, 0.8)])


def _route_board():
    from kar.model import Board, Footprint, Pad

    def pad(ref, num, net, x, y):
        return Pad(ref, num, net, x, y, True, ("F.Cu",), bbox=(x - 0.5, y - 0.5, x + 0.5, y + 0.5))
    j1 = Footprint("J1", "XLR", "Jack_XLR_Neutrik", [pad("J1", "2", "/IN_H", 5, 10), pad("J1", "3", "/IN_C", 5, 12)])
    u1 = Footprint("U1", "OPA1612", "SOIC-8", [pad("U1", "3", "/IN_H", 35, 10), pad("U1", "2", "/IN_C", 35, 12)])
    # a wall of foreign copper across the direct path on F.Cu, with a gap far below
    wall = [Seg(20, 0.5, 20, 18, 0.5, "F.Cu", "/OTHER")]
    sq = [(0, 0), (40, 0), (40, 30), (0, 30)]
    return Board([j1, u1], wall, [], ["F.Cu", "B.Cu"], outline=[sq])


def test_router_pair():
    from kar.router import RouteParams, route
    from kar.geometry import edge_gap
    b = _route_board()
    rules = R.load(None)
    cls = classify(b, rules)
    res = route(b, cls, RouteParams(layers=["F.Cu", "B.Cu"]), rules["activity_weight"], nets=["/IN_H", "/IN_C"])
    assert res.failed == [] and {r.net for r in res.routed} == {"/IN_H", "/IN_C"}
    segs = [s for r in res.routed for s in r.segs]
    # no clearance violation against the wall on its layer
    assert all(edge_gap(s, b.segs[0]) >= 0.2 - 1e-6 for s in segs if s.layer == "F.Cu")
    # the pair runs side by side for most of its length
    b.segs += segs
    rep = build(b, classify(b, rules), rules).data
    assert rep["pairs"]["items"][0]["parallel_pct"] > 60, rep["pairs"]["items"]


def test_drop_nets():
    from kar.freerouting import drop_nets
    dsn = """(pcb x (network
    (net GND (pins U1-4 C1-2))
    (net "/SIG A" (pins U1-1 R1-1))
    (net /CHASSIS (pins H1-1))
    (class kicad_default "" GND "/SIG A" /CHASSIS (rule (width 200)))
  ))"""
    out, dropped = drop_nets(dsn, {"GND", "/CHASSIS"})
    assert dropped == ["GND", "/CHASSIS"]
    assert "U1-4" not in out and "H1-1" not in out and '"/SIG A" (pins' in out
    assert "(class kicad_default \"\" \"/SIG A\" (rule" in out
    assert out.count("(") == out.count(")")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
