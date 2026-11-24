"""Classify nets into gnd / power / digital / analog / other.

Order of decision (first wins):
  1. explicit rules["classes"] globs
  2. gnd / power by name
  3. digital: by name, or touching a digital IC, or touching a digital pin of a mixed IC
  4. analog: touching an analog IC (non-digital pin) or an audio connector, then spread through
     two-terminal passives (R, C, L, FB) and relay contacts to neighbouring nets that are not yet decided
  5. everything else: other
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

from . import rules as R
from .model import Board

PASSIVE_PREFIX = {"R", "C", "L", "FB", "RN"}


@dataclass
class Classification:
    kind: dict[str, str]                       # net -> kind
    why: dict[str, str]                        # net -> short reason
    pairs: list[tuple[str, str]] = field(default_factory=list)   # differential pairs (analog)
    activity: dict[str, str] = field(default_factory=dict)       # digital net -> clock / data / static

    def nets_of(self, kind: str) -> list[str]:
        return sorted(n for n, k in self.kind.items() if k == kind)


def _any(patterns: list[str], text: str) -> bool:
    return any(re.search(p, text, re.I) for p in patterns)


def classify(board: Board, rules: dict) -> Classification:
    pads = board.pads_by_net()
    fps = board.fp_by_ref()
    nets = sorted(n for n in board.nets if n and not n.startswith("unconnected-"))
    kind: dict[str, str] = {}
    why: dict[str, str] = {}

    def setk(n, k, w):
        if n not in kind:
            kind[n] = k
            why[n] = w

    # 1. explicit
    for k, pats in rules["classes"].items():
        for n in nets:
            if pats and R.glob_match(n, pats):
                setk(n, k, "rules")
    # 2. names
    for n in nets:
        if _any(rules["gnd_names"], n):
            setk(n, "gnd", "name")
    for n in nets:
        if _any(rules["power_names"], n):
            setk(n, "power", "name")
    # 3. digital
    for n in nets:
        if _any(rules["digital_names"], n.lstrip("/")):
            setk(n, "digital", "name")
    for n in nets:
        for p in pads.get(n, []):
            fp = fps[p.ref]
            if fp.prefix == "U" and _any(rules["digital_parts"], fp.value):
                setk(n, "digital", f"{fp.ref} {fp.value}")
                break
            if fp.prefix == "U" and _any(rules["analog_parts"], fp.value) and p.pin_function \
                    and _any(rules["digital_pins"], p.pin_function):
                setk(n, "digital", f"{fp.ref}.{p.number} {p.pin_function}")
                break
    # 4. analog seeds
    for n in nets:
        if n in kind:
            continue
        for p in pads.get(n, []):
            fp = fps[p.ref]
            if fp.prefix == "U" and _any(rules["analog_parts"], fp.value):
                setk(n, "analog", f"{fp.ref} {fp.value}")
                break
            if fp.prefix in ("J", "P", "CN") and _any(rules["audio_connectors"], fp.fp_name + " " + fp.value):
                setk(n, "analog", f"{fp.ref} {fp.fp_name}")
                break
    # spread through passives and relays (contacts only: a relay pin on a digital net is its coil)
    changed = True
    while changed:
        changed = False
        for fp in board.footprints:
            if fp.prefix in PASSIVE_PREFIX and len({p.net for p in fp.pads if p.net}) == 2:
                a, b = [p.net for p in fp.pads if p.net][:2]
                for x, y in ((a, b), (b, a)):
                    if kind.get(x) == "analog" and y not in kind and not y.startswith("unconnected-"):
                        setk(y, "analog", f"via {fp.ref}")
                        changed = True
            elif fp.prefix == "K":
                an = [p.net for p in fp.pads if kind.get(p.net) == "analog"]
                if an:
                    for p in fp.pads:
                        if p.net and p.net not in kind and not p.net.startswith("unconnected-"):
                            setk(p.net, "analog", f"via {fp.ref} contact")
                            changed = True
    for n in nets:
        setk(n, "other", "-")
    return Classification(kind, why, find_pairs(kind, rules), digital_activity(board, kind, rules))


def digital_activity(board: Board, kind: dict[str, str], rules: dict) -> dict[str, str]:
    """clock / data / static for each digital net (see rules["activity_names"])."""
    fps = board.fp_by_ref()
    pads = board.pads_by_net()
    out = {}
    for n, k in kind.items():
        if k != "digital":
            continue
        bare = n.lstrip("/")
        for act in ("clock", "data", "static"):
            if _any(rules["activity_names"].get(act, []), bare):
                out[n] = act
                break
        else:
            touches_static = any(_any(rules["static_parts"], fps[p.ref].value) or fps[p.ref].prefix == "K"
                                 for p in pads.get(n, []))
            out[n] = "static" if touches_static else "data"
    return out


def find_pairs(kind: dict[str, str], rules: dict) -> list[tuple[str, str]]:
    analog = {n for n, k in kind.items() if k == "analog"}
    out = []
    for n in sorted(analog):
        for a, b in rules["diff_pairs"]:
            if n.endswith(a):
                m = n[: len(n) - len(a)] + b
                if m in analog and (n, m) not in out:
                    out.append((n, m))
    return out
