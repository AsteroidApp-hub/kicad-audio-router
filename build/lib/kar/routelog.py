"""Route log: what kar did, per net and as a list of events, for review in KiCad.

Written by `kar route --log DIR` and by `kar auto` as <board>_route.md / .json."""
from __future__ import annotations

import json
import os

from .classify import Classification

WHY = {
    "fine_first": "routed first on a 0.125 mm grid (a pin the 0.25 mm grid cannot reach)",
    "failed_first_pass": "did not fit on the first pass",
    "push": "routed through other tracks; the tracks it crossed were routed again",
    "fine_fallback": "tried again on a 0.125 mm grid",
    "rewiden": "power net routed again at full width, pushing other tracks aside",
}


def build(routes, cls: Classification, stats: dict, full_width) -> dict:
    nets = []
    for r in routes:
        kind = cls.kind.get(r.net, "other")
        widths = sorted({round(s.width, 3) for s in r.segs})
        full = full_width(r.net, kind)
        narrow = sum(s.length for s in r.segs if s.width < full - 1e-9)
        nets.append({"net": r.net, "kind": kind, "length_mm": round(sum(s.length for s in r.segs), 1),
                     "vias": len(r.vias), "widths_mm": widths, "narrow_mm": round(narrow, 1),
                     "layers": sorted({s.layer for s in r.segs})})
    nets.sort(key=lambda n: (n["kind"], n["net"]))
    events = stats.get("events", [])
    timing = {k: v for k, v in stats.items() if k.startswith("t_")}
    return {"nets": nets, "events": events, "timing_s": timing,
            "summary": {"nets": len(nets), "pushes": stats.get("pushes", 0), "rerouted": stats.get("rerouted", 0),
                        "widened_segments": stats.get("widened", 0)}}


def to_markdown(log: dict, title: str) -> str:
    L = [f"# Route log: {title}", "",
         "Tracks and vias placed by kar are in PCB groups named `kar: <kind>` (select a group in KiCad to see",
         "them at once). Ground vias added by `kar ground` are in `kar: ground`.", ""]
    s = log["summary"]
    L += [f"{s['nets']} nets routed; {s['pushes']} pushes moved {s['rerouted']} routes; "
          f"{s['widened_segments']} narrowed segments widened again.", ""]
    t = log["timing_s"]
    if t:
        L += ["| phase | seconds |", "|---|---:|"] + [f"| {k[2:]} | {v} |" for k, v in t.items()] + [""]
    if log["events"]:
        L += ["## What needed more than a plain route", "", "| event | nets | detail |", "|---|---|---|"]
        for e in log["events"]:
            nets = e.get("nets") or [e.get("net")]
            detail = WHY.get(e["type"], "")
            if e["type"] == "push":
                detail += f" (moved: {', '.join(e.get('moved', [])) or '-'})"
            if e["type"] == "rewiden":
                detail += (f" (moved: {', '.join(e.get('moved', [])) or '-'})" if e.get("ok")
                           else f" — kept as it was: {e.get('why')}")
            if e.get("still_failed") or e.get("failed"):
                detail += f" — still failed: {', '.join(e.get('still_failed') or e.get('failed'))}"
            shown = ", ".join(n for n in nets if n)
            L.append(f"| {e['type']} | {shown} | {detail} |")
        L.append("")
    narrow = [n for n in log["nets"] if n["narrow_mm"] > 0]
    if narrow:
        L += ["## Nets narrowed below their width", "", "| net | kind | narrow length | widths |", "|---|---|---:|---|"]
        for n in sorted(narrow, key=lambda n: -n["narrow_mm"]):
            L.append(f"| {n['net']} | {n['kind']} | {n['narrow_mm']} mm | {', '.join(map(str, n['widths_mm']))} |")
        L.append("")
    L += ["## All nets", "", "| net | kind | length | vias | widths | layers |", "|---|---|---:|---:|---|---|"]
    for n in log["nets"]:
        L.append(f"| {n['net']} | {n['kind']} | {n['length_mm']} mm | {n['vias']} | "
                 f"{', '.join(map(str, n['widths_mm']))} | {', '.join(n['layers'])} |")
    return "\n".join(L) + "\n"


def write(log: dict, out_dir: str, stem: str) -> tuple[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    j = os.path.join(out_dir, f"{stem}_route.json")
    m = os.path.join(out_dir, f"{stem}_route.md")
    with open(j, "w", encoding="utf-8") as f:
        json.dump(log, f, ensure_ascii=False, indent=1)
    with open(m, "w", encoding="utf-8") as f:
        f.write(to_markdown(log, stem))
    return m, j
