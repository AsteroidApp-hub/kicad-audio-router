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
# multi-pin parts that carry the analog signal through them (relay: contacts only, see below)
THROUGH_PREFIX = {"Q", "SW", "T", "TR", "RV", "VR", "D", "TH"}


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


STANDARD = {"R", "C", "L", "FB", "RN", "D", "Q", "U", "IC", "J", "P", "CN", "SW", "S", "T", "TR", "RV", "VR",
            "K", "RY", "F", "Y", "X", "LED", "TP", "MH", "H", "Z", "ZD", "TH", "BT"}
RELAY_RE = r"relay|HFD\d|G5V|G6K|G6[AHS]|TQ2|AGN|IM0\d|TX2|DS2E|NA05|OMRON|^RL\b|SRD-"
FIRST = {"R": "R", "C": "C", "L": "L", "D": "D", "Z": "D", "Q": "Q", "U": "U", "I": "U", "J": "J", "P": "J",
         "S": "SW", "T": "T", "K": "K", "V": "VR", "F": "F", "Y": "Y", "X": "Y", "M": "MH", "H": "MH"}


def part_kind(fp) -> str:
    """Normalised part type from the reference designator, value and footprint name. Handles
    house styles like 'RGaR' (resistor), 'DBL' (diode), 'JINL' (connector), 'RLL' + value HFD27 (relay)."""
    if re.search(RELAY_RE, fp.value + " " + fp.fp_name, re.I):
        return "K"
    p = fp.prefix
    if p in STANDARD:
        return {"IC": "U", "RY": "K", "Z": "D", "ZD": "D", "S": "SW", "P": "J", "CN": "J", "X": "Y",
                "LED": "D"}.get(p, p)
    return FIRST.get(p[:1].upper(), p)


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
    # supply by what the net touches: regulator / DC-DC pins, supply connectors (value like "+20", "16VAC")
    for n in nets:
        for p in pads.get(n, []):
            fp = fps[p.ref]
            pk = part_kind(fp)
            if pk == "U" and _any(rules.get("power_parts", []), fp.value):
                setk(n, "power", f"{fp.ref} {fp.value}")
                break
            # a supply connector has few pins (a USB-C receptacle with CC/data pins is not one)
            if pk in ("J", "K") and len(fp.pads) <= 4 and _any(rules.get("power_connectors", []), fp.value):
                setk(n, "power", f"{fp.ref} {fp.value}")
                break
    # 3. digital
    for n in nets:
        if _any(rules["digital_names"], n.lstrip("/")):
            setk(n, "digital", "name")
    for n in nets:
        for p in pads.get(n, []):
            fp = fps[p.ref]
            if part_kind(fp) == "U" and _any(rules["digital_parts"], fp.value):
                setk(n, "digital", f"{fp.ref} {fp.value}")
                break
            if part_kind(fp) == "U" and _any(rules["analog_parts"], fp.value) and p.pin_function \
                    and _any(rules["digital_pins"], p.pin_function):
                setk(n, "digital", f"{fp.ref}.{p.number} {p.pin_function}")
                break
    # 4. analog seeds
    for n in nets:
        if n in kind:
            continue
        for p in pads.get(n, []):
            fp = fps[p.ref]
            if part_kind(fp) == "U" and _any(rules["analog_parts"], fp.value):
                setk(n, "analog", f"{fp.ref} {fp.value}")
                break
            if part_kind(fp) == "J" and (_any(rules["audio_connectors"], fp.fp_name + " " + fp.value)
                                         or re.search(r"^J.*(IN|OUT|MIC|HP|LINE|XLR|TRS)", fp.ref, re.I)):
                setk(n, "analog", f"{fp.ref} {fp.fp_name}")
                break
    # spread through passives and relays (contacts only: a relay pin on a digital net is its coil)
    changed = True
    while changed:
        changed = False
        for fp in board.footprints:
            pk = part_kind(fp)
            if pk in PASSIVE_PREFIX and len({p.net for p in fp.pads if p.net}) == 2:
                a, b = [p.net for p in fp.pads if p.net][:2]
                for x, y in ((a, b), (b, a)):
                    if kind.get(x) == "analog" and y not in kind and not y.startswith("unconnected-"):
                        setk(y, "analog", f"via {fp.ref}")
                        changed = True
            elif pk in THROUGH_PREFIX:
                if any(kind.get(p.net) == "analog" for p in fp.pads):
                    for p in fp.pads:
                        if p.net and p.net not in kind and not p.net.startswith("unconnected-"):
                            setk(p.net, "analog", f"via {fp.ref}")
                            changed = True
            elif pk == "K":
                an = [p.net for p in fp.pads if kind.get(p.net) == "analog"]
                if an:
                    for p in fp.pads:
                        if p.net and p.net not in kind and not p.net.startswith("unconnected-"):
                            setk(p.net, "analog", f"via {fp.ref} contact")
                            changed = True
    for n in nets:
        setk(n, "other", "-")
    return Classification(kind, why, find_pairs(kind, rules, board), digital_activity(board, kind, rules))


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
            touches_static = any(_any(rules["static_parts"], fps[p.ref].value) or part_kind(fps[p.ref]) == "K"
                                 for p in pads.get(n, []))
            out[n] = "static" if touches_static else "data"
    return out


def find_pairs(kind: dict[str, str], rules: dict, board: Board | None = None) -> list[tuple[str, str]]:
    """Analog nets whose names differ only by a suffix / word pair. With a board, the two nets must also
    connect to mostly the same kinds of parts (by type and value, at least half in common), otherwise they
    cannot run together anyway (e.g. HOT also goes to a test clip far away)."""
    from collections import Counter
    analog = {n for n, k in kind.items() if k == "analog"}
    parts = {}
    if board is not None:
        fps = board.fp_by_ref()
        for n, ps in board.pads_by_net().items():
            # what the net connects to, as (part type, value) per part: mirror-image halves of a balanced
            # circuit use different parts of the same kind and value
            parts[n] = Counter((part_kind(fps[r]), fps[r].value) for r in {p.ref for p in ps})

    def same_parts(a, b):
        if board is None:
            return True
        pa, pb = parts.get(a, Counter()), parts.get(b, Counter())
        union = sum((pa | pb).values())
        return union > 0 and sum((pa & pb).values()) / union >= 0.5
    out = []
    for n in sorted(analog):
        for a, b in rules["diff_pairs"]:
            if n.endswith(a):
                m = n[: len(n) - len(a)] + b
                if m in analog and (n, m) not in out and same_parts(n, m):
                    out.append((n, m))
        for a, b in rules.get("diff_pairs_word", []):
            for mt in re.finditer(re.escape(a), n, re.I):
                m = n[:mt.start()] + (b if n[mt.start()].isupper() else b.lower()) + n[mt.end():]
                if m in analog and (n, m) not in out and (m, n) not in out and same_parts(n, m):
                    out.append((n, m))
    return out
