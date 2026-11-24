"""Routing quality report for audio boards.

Metrics (all deterministic, mm):
  coupling   analog copper running within N mm (edge to edge, same layer) of digital copper
  pairs      differential pairs: length of each side, mismatch, share of length routed side by side
  gnd        SMD pads on GND: distance to the nearest GND via / through-hole GND pad
  power      narrowest track on each power net
  analog     per analog net: track length and via count
  unrouted   unconnected items reported by KiCad
A single score (0-100, higher is better) summarises the board so routing runs can be compared.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .classify import Classification
from .geometry import Seg, SegIndex, coupled_length, point_seg_dist
from .model import Board


@dataclass
class Report:
    data: dict

    def to_markdown(self) -> str:
        d = self.data
        L = [f"# Routing quality report", "", f"source: `{d['source']}`", "",
             f"**score {d['score']:.1f} / 100**  (higher is better; see `score_parts`)", "",
             "| item | value |", "|---|---|"]
        s = d["summary"]
        L += [f"| nets (analog / digital / power / gnd / other) | {s['analog']} / {s['digital']} / {s['power']} / {s['gnd']} / {s['other']} |",
              f"| unrouted | {d['unrouted']} |",
              f"| analog track length | {s['analog_length_mm']:.0f} mm, {s['analog_vias']} vias |"]
        c = d["coupling"]
        for g, v in c["total_mm"].items():
            parts = ", ".join(f"{a} {c['by_activity_mm'][a][g]:.0f}" for a in c["by_activity_mm"])
            L.append(f"| analog running within {g} mm of digital | {v:.1f} mm (weighted {c['weighted_mm'][g]:.1f}; {parts}) |")
        p = d["pairs"]
        L += [f"| differential pairs | {len(p['items'])} (mismatch max {p['max_mismatch_mm']:.1f} mm, side-by-side {p['avg_parallel_pct']:.0f} %, away from pins {p['avg_parallel_away_pct']:.0f} %) |",
              f"| GND pads farther than {d['gnd']['limit_mm']} mm from a GND via | {len(d['gnd']['far'])} / {d['gnd']['pads']} |",
              f"| power nets narrower than their width (net class, else {d['power']['min_width_mm']} mm) | {len(d['power']['narrow'])} "
              f"(neck-downs into fine-pitch pins not counted: {d['power'].get('neck_downs', 0)}) |", ""]
        worst = d["coupling"]["worst"][:15]
        if worst:
            L += ["## Analog nets closest to digital", "",
                  f"| analog net | within {d['coupling']['gaps'][0]} mm (weighted) | within {d['coupling']['gaps'][-1]} mm (weighted) | digital nets |",
                  "|---|---:|---:|---|"]
            for w in worst:
                L.append(f"| {w['net']} | {w['near'][0]:.1f} ({w['weighted'][0]:.1f}) | {w['near'][-1]:.1f} ({w['weighted'][-1]:.1f}) | {', '.join(w['aggressors'][:5])} |")
            L.append("")
        if p["items"]:
            L += ["## Differential pairs", "", "| pair | length | mismatch | side-by-side | away from pins |",
                  "|---|---:|---:|---:|---:|"]
            for it in sorted(p["items"], key=lambda x: -x["mismatch_mm"]):
                aw = "-" if it["parallel_away_pct"] is None else f"{it['parallel_away_pct']:.0f} %"
                L.append(f"| {it['a']} / {it['b']} | {it['len_a']:.1f} / {it['len_b']:.1f} | {it['mismatch_mm']:.1f} | "
                         f"{it['parallel_pct']:.0f} % | {aw} |")
            L.append("")
        if d["gnd"]["far"]:
            L += ["## GND pads far from a via", "", "| pad | distance |", "|---|---:|"]
            for f in d["gnd"]["far"][:30]:
                L.append(f"| {f['pad']} | {f['dist_mm']:.1f} |")
            L.append("")
        if d["power"]["narrow"]:
            L += ["## Narrow power tracks", "", "| net | narrowest | narrow length |", "|---|---:|---:|"]
            for f in d["power"]["narrow"]:
                L.append(f"| {f['net']} | {f['min_width_mm']:.2f} | {f.get('length_mm', 0):.1f} mm |")
            L.append("")
        return "\n".join(L) + "\n"


def build(board: Board, cls: Classification, rules: dict) -> Report:
    m = rules["metrics"]
    gaps = sorted(m["coupling_gap_mm"])
    kind = cls.kind
    analog_segs = [s for s in board.segs if kind.get(s.net) == "analog"]
    digital_segs = [s for s in board.segs if kind.get(s.net) == "digital"]
    dig_idx = SegIndex(digital_segs, cell=2.0)

    # coupling, split by aggressor activity (clock / data / static) and weighted
    weights = rules["activity_weight"]
    acts = sorted(set(cls.activity.values()) | set(weights))
    idx = {a: SegIndex([s for s in digital_segs if cls.activity.get(s.net, "data") == a], cell=2.0) for a in acts}
    per_net: dict[str, dict] = {}
    by_act = {a: [0.0] * len(gaps) for a in acts}
    for s in analog_segs:
        e = per_net.setdefault(s.net, {"near": [0.0] * len(gaps), "weighted": [0.0] * len(gaps), "aggr": set()})
        for a in acts:
            if not idx[a].grid:
                continue
            for i, g in enumerate(gaps):
                l, nets = coupled_length(s, idx[a], g)
                e["near"][i] += l
                e["weighted"][i] += l * weights.get(a, 1.0)
                by_act[a][i] += l
                e["aggr"] |= nets
    worst = sorted(({"net": n, "near": e["near"], "weighted": e["weighted"], "aggressors": sorted(e["aggr"])}
                    for n, e in per_net.items() if e["near"][-1] > 0),
                   key=lambda x: (-x["weighted"][0], -x["weighted"][-1]))
    coupling = {"gaps": gaps,
                "total_mm": {str(g): sum(e["near"][i] for e in per_net.values()) for i, g in enumerate(gaps)},
                "weighted_mm": {str(g): sum(e["weighted"][i] for e in per_net.values()) for i, g in enumerate(gaps)},
                "by_activity_mm": {a: {str(g): v[i] for i, g in enumerate(gaps)} for a, v in by_act.items()},
                "weights": weights, "worst": worst}

    # analog length / vias
    alen: dict[str, float] = {}
    for s in analog_segs:
        alen[s.net] = alen.get(s.net, 0.0) + s.length
    avias: dict[str, int] = {}
    for v in board.vias:
        if kind.get(v.net) == "analog":
            avias[v.net] = avias.get(v.net, 0) + 1

    # differential pairs
    items = []
    by_net: dict[str, list[Seg]] = {}
    for s in analog_segs:
        by_net.setdefault(s.net, []).append(s)
    pads_by = board.pads_by_net()
    for a, b in cls.pairs:
        la, lb = alen.get(a, 0.0), alen.get(b, 0.0)
        idx_b = SegIndex(by_net.get(b, []), cell=2.0)
        par = sum(coupled_length(s, idx_b, 0.6)[0] for s in by_net.get(a, []))
        # the same, leaving out the stretches next to the pins (pin positions decide those)
        pins = pads_by.get(a, []) + pads_by.get(b, [])
        free = free_par = 0.0
        for sg in by_net.get(a, []):
            n = max(1, int(sg.length / 0.25))
            dl = sg.length / n
            near_b = idx_b.near(sg.layer, sg.bbox(1.2))
            for k in range(n):
                t = (k + 0.5) / n
                px, py = sg.x1 + t * (sg.x2 - sg.x1), sg.y1 + t * (sg.y2 - sg.y1)
                if any(p.dist(px, py) < 2.0 for p in pins):
                    continue
                free += dl
                if any(point_seg_dist(px, py, o.x1, o.y1, o.x2, o.y2) - (o.width + sg.width) / 2 <= 0.6 for o in near_b):
                    free_par += dl
        items.append({"a": a, "b": b, "len_a": la, "len_b": lb, "mismatch_mm": abs(la - lb),
                      "parallel_pct": 100.0 * par / la if la else 0.0,
                      "parallel_away_pct": 100.0 * free_par / free if free > 1.0 else None})
    away = [i["parallel_away_pct"] for i in items if i["parallel_away_pct"] is not None]
    pairs = {"items": items,
             "max_mismatch_mm": max((i["mismatch_mm"] for i in items), default=0.0),
             "avg_parallel_pct": sum(i["parallel_pct"] for i in items) / len(items) if items else 0.0,
             "avg_parallel_away_pct": sum(away) / len(away) if away else 100.0}

    # GND return: SMD pads of each ground net -> nearest via / THT pad of the same net (from the pad edge)
    from .ground import anchor_dist, anchors_by_net
    gnd_nets = {n for n, k in kind.items() if k == "gnd"}
    anchors = anchors_by_net(board, gnd_nets)
    far, npads = [], 0
    for fp in board.footprints:
        for p in fp.pads:
            if p.net in gnd_nets and p.smd:
                npads += 1
                d = anchor_dist(p, anchors.get(p.net, []))
                if d > m["gnd_via_max_mm"]:
                    far.append({"pad": f"{p.ref}.{p.number}", "net": p.net, "dist_mm": round(d, 2)})
    far.sort(key=lambda f: -f["dist_mm"])
    gnd = {"limit_mm": m["gnd_via_max_mm"], "pads": npads, "far": far}

    # power width
    # narrow power tracks; a narrow stretch within 1.5 mm of a pad of the same net that is narrower than a
    # full-width power track is a neck-down into a fine-pitch pin (necessary), counted apart
    narrow, neck = [], 0
    lim = m["power_min_width_mm"]
    pads_of = board.pads_by_net()
    for n in cls.nets_of("power"):
        # a net class with its own track width sets the expectation for that net (the designer's choice)
        nc = board.class_of(n) if hasattr(board, "class_of") else None
        own = float(nc["track_width"]) if nc and nc.get("track_width") else None
        full = own or m.get("power_full_width_mm", 0.5)
        want = own or lim
        small = [p for p in pads_of.get(n, []) if min(p.box()[2] - p.box()[0], p.box()[3] - p.box()[1]) < full]
        bad_w, bad_len = None, 0.0
        for sg in board.segs:
            if sg.net != n or sg.width >= want - 1e-6:
                continue
            if any(min(p.dist(sg.x1, sg.y1), p.dist(sg.x2, sg.y2)) < 1.5 for p in small):
                neck += 1
                continue
            bad_w = sg.width if bad_w is None else min(bad_w, sg.width)
            bad_len += sg.length
        if bad_w is not None:
            narrow.append({"net": n, "min_width_mm": bad_w, "length_mm": round(bad_len, 1)})
    power = {"min_width_mm": lim, "narrow": narrow, "neck_downs": neck}

    summary = {k: len(cls.nets_of(k)) for k in ("analog", "digital", "power", "gnd", "other")}
    summary["analog_length_mm"] = sum(alen.values())
    summary["analog_vias"] = sum(avias.values())

    score, parts = _score(board, summary, coupling, pairs, gnd, power)
    data = {"source": board.source, "unrouted": board.unrouted, "summary": summary, "coupling": coupling,
            "pairs": pairs, "gnd": gnd, "power": power,
            "analog_nets": {n: {"length_mm": round(alen.get(n, 0.0), 2), "vias": avias.get(n, 0)}
                            for n in cls.nets_of("analog")},
            "classes": {n: [cls.kind[n], cls.why[n]] + ([cls.activity[n]] if n in cls.activity else [])
                        for n in sorted(cls.kind)},
            "pairs_found": cls.pairs, "score": score, "score_parts": parts}
    return Report(data)


def _score(board, summary, coupling, pairs, gnd, power) -> tuple[float, dict]:
    """0-100. Each part loses points smoothly; weights reflect audio priorities."""
    alen = max(summary["analog_length_mm"], 1.0)
    g0 = coupling["weighted_mm"][str(coupling["gaps"][0])]
    g1 = coupling["weighted_mm"][str(coupling["gaps"][-1])]
    parts = {
        # share of analog length close to digital, weighted by aggressor activity (tight gap counts 3x)
        "coupling": 35 * math.exp(-(3 * g0 + g1) / alen * 10),
        # side by side where the pins allow it (stretches next to the pins are left out)
        "pairs": 15 * (pairs["avg_parallel_away_pct"] / 100 if pairs["items"] else 1.0)
                 * math.exp(-pairs["max_mismatch_mm"] / 50),
        "gnd": 20 * (1 - len(gnd["far"]) / gnd["pads"] if gnd["pads"] else 1.0),
        # by how much narrow power track there is (a 0.6 mm neck is not a 20 mm bottleneck)
        "power": 10 * math.exp(-sum(f.get("length_mm", 5.0) for f in power["narrow"]) / 30.0),
        "vias": 10 * math.exp(-summary["analog_vias"] / max(summary["analog"], 1) / 2),
        "routed": 10 * (1.0 if not board.unrouted else 0.0),
    }
    if board.unrouted:
        parts = {k: v * 0.5 for k, v in parts.items()}
    return round(sum(parts.values()), 1), {k: round(v, 2) for k, v in parts.items()}
