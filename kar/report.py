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
from .geometry import Seg, SegIndex, coupled_length
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
        for g, v in d["coupling"]["total_mm"].items():
            L.append(f"| analog running within {g} mm of digital | {v:.1f} mm |")
        p = d["pairs"]
        L += [f"| differential pairs | {len(p['items'])} (mismatch max {p['max_mismatch_mm']:.1f} mm, side-by-side {p['avg_parallel_pct']:.0f} %) |",
              f"| GND pads farther than {d['gnd']['limit_mm']} mm from a GND via | {len(d['gnd']['far'])} / {d['gnd']['pads']} |",
              f"| power nets narrower than {d['power']['min_width_mm']} mm | {len(d['power']['narrow'])} |", ""]
        worst = d["coupling"]["worst"][:15]
        if worst:
            L += ["## Analog nets closest to digital", "",
                  f"| analog net | within {d['coupling']['gaps'][0]} mm | within {d['coupling']['gaps'][-1]} mm | digital nets |",
                  "|---|---:|---:|---|"]
            for w in worst:
                L.append(f"| {w['net']} | {w['near'][0]:.1f} | {w['near'][-1]:.1f} | {', '.join(w['aggressors'][:5])} |")
            L.append("")
        if p["items"]:
            L += ["## Differential pairs", "", "| pair | length | mismatch | side-by-side |", "|---|---:|---:|---:|"]
            for it in sorted(p["items"], key=lambda x: -x["mismatch_mm"]):
                L.append(f"| {it['a']} / {it['b']} | {it['len_a']:.1f} / {it['len_b']:.1f} | {it['mismatch_mm']:.1f} | {it['parallel_pct']:.0f} % |")
            L.append("")
        if d["gnd"]["far"]:
            L += ["## GND pads far from a via", "", "| pad | distance |", "|---|---:|"]
            for f in d["gnd"]["far"][:30]:
                L.append(f"| {f['pad']} | {f['dist_mm']:.1f} |")
            L.append("")
        if d["power"]["narrow"]:
            L += ["## Narrow power tracks", "", "| net | narrowest |", "|---|---:|"]
            for f in d["power"]["narrow"]:
                L.append(f"| {f['net']} | {f['min_width_mm']:.2f} |")
            L.append("")
        return "\n".join(L) + "\n"


def build(board: Board, cls: Classification, rules: dict) -> Report:
    m = rules["metrics"]
    gaps = sorted(m["coupling_gap_mm"])
    kind = cls.kind
    analog_segs = [s for s in board.segs if kind.get(s.net) == "analog"]
    digital_segs = [s for s in board.segs if kind.get(s.net) == "digital"]
    dig_idx = SegIndex(digital_segs, cell=2.0)

    # coupling
    per_net: dict[str, dict] = {}
    for s in analog_segs:
        e = per_net.setdefault(s.net, {"near": [0.0] * len(gaps), "aggr": set()})
        for i, g in enumerate(gaps):
            l, nets = coupled_length(s, dig_idx, g)
            e["near"][i] += l
            if i == 0 or l:
                e["aggr"] |= nets
    worst = sorted(({"net": n, "near": e["near"], "aggressors": sorted(e["aggr"])} for n, e in per_net.items()
                    if e["near"][-1] > 0), key=lambda x: (-x["near"][0], -x["near"][-1]))
    coupling = {"gaps": gaps, "total_mm": {str(g): sum(e["near"][i] for e in per_net.values()) for i, g in enumerate(gaps)},
                "worst": worst}

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
    for a, b in cls.pairs:
        la, lb = alen.get(a, 0.0), alen.get(b, 0.0)
        idx_b = SegIndex(by_net.get(b, []), cell=2.0)
        par = sum(coupled_length(s, idx_b, 0.6)[0] for s in by_net.get(a, []))
        items.append({"a": a, "b": b, "len_a": la, "len_b": lb, "mismatch_mm": abs(la - lb),
                      "parallel_pct": 100.0 * par / la if la else 0.0})
    pairs = {"items": items,
             "max_mismatch_mm": max((i["mismatch_mm"] for i in items), default=0.0),
             "avg_parallel_pct": sum(i["parallel_pct"] for i in items) / len(items) if items else 0.0}

    # GND return: SMD GND pads -> nearest GND via / THT GND pad
    gnd_nets = {n for n, k in kind.items() if k == "gnd"}
    anchors = [(v.x, v.y) for v in board.vias if v.net in gnd_nets]
    anchors += [(p.x, p.y) for fp in board.footprints for p in fp.pads if p.net in gnd_nets and not p.smd]
    far, npads = [], 0
    for fp in board.footprints:
        for p in fp.pads:
            if p.net in gnd_nets and p.smd:
                npads += 1
                d = min((math.hypot(p.x - x, p.y - y) for x, y in anchors), default=math.inf)
                if d > m["gnd_via_max_mm"]:
                    far.append({"pad": f"{p.ref}.{p.number}", "dist_mm": round(d, 2)})
    far.sort(key=lambda f: -f["dist_mm"])
    gnd = {"limit_mm": m["gnd_via_max_mm"], "pads": npads, "far": far}

    # power width
    narrow = []
    for n in cls.nets_of("power"):
        ws = [s.width for s in board.segs if s.net == n]
        if ws and min(ws) < m["power_min_width_mm"] - 1e-6:
            narrow.append({"net": n, "min_width_mm": min(ws)})
    power = {"min_width_mm": m["power_min_width_mm"], "narrow": narrow}

    summary = {k: len(cls.nets_of(k)) for k in ("analog", "digital", "power", "gnd", "other")}
    summary["analog_length_mm"] = sum(alen.values())
    summary["analog_vias"] = sum(avias.values())

    score, parts = _score(board, summary, coupling, pairs, gnd, power)
    data = {"source": board.source, "unrouted": board.unrouted, "summary": summary, "coupling": coupling,
            "pairs": pairs, "gnd": gnd, "power": power,
            "analog_nets": {n: {"length_mm": round(alen.get(n, 0.0), 2), "vias": avias.get(n, 0)}
                            for n in cls.nets_of("analog")},
            "classes": {n: [cls.kind[n], cls.why[n]] for n in sorted(cls.kind)},
            "pairs_found": cls.pairs, "score": score, "score_parts": parts}
    return Report(data)


def _score(board, summary, coupling, pairs, gnd, power) -> tuple[float, dict]:
    """0-100. Each part loses points smoothly; weights reflect audio priorities."""
    alen = max(summary["analog_length_mm"], 1.0)
    g0 = coupling["total_mm"][str(coupling["gaps"][0])]
    g1 = coupling["total_mm"][str(coupling["gaps"][-1])]
    parts = {
        # share of analog length close to digital (tight gap counts 3x)
        "coupling": 35 * math.exp(-(3 * g0 + g1) / alen * 10),
        "pairs": 15 * (pairs["avg_parallel_pct"] / 100 if pairs["items"] else 1.0)
                 * math.exp(-pairs["max_mismatch_mm"] / 50),
        "gnd": 20 * (1 - len(gnd["far"]) / gnd["pads"] if gnd["pads"] else 1.0),
        "power": 10 * (1.0 if not power["narrow"] else math.exp(-len(power["narrow"]) / 3)),
        "vias": 10 * math.exp(-summary["analog_vias"] / max(summary["analog"], 1) / 2),
        "routed": 10 * (1.0 if not board.unrouted else 0.0),
    }
    if board.unrouted:
        parts = {k: v * 0.5 for k, v in parts.items()}
    return round(sum(parts.values()), 1), {k: round(v, 2) for k, v in parts.items()}
