"""Command line entry point.

  python -m kar classify BOARD.kicad_pcb [--rules R.json]          # how nets were classified
  python -m kar report   BOARD.kicad_pcb [--rules R.json] [--out DIR] [--json]

Run with KiCad's bundled Python (needs pcbnew).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from . import __version__
from . import rules as R


def _load(args):
    from . import kicad_io
    from .classify import classify
    board, _ = kicad_io.load(args.board)
    rules = R.load(args.rules)
    return board, rules, classify(board, rules)


def cmd_classify(args) -> int:
    board, rules, cls = _load(args)
    if args.json:
        json.dump({"classes": {n: [cls.kind[n], cls.why[n]] for n in sorted(cls.kind)}, "pairs": cls.pairs},
                  sys.stdout, ensure_ascii=False, indent=1)
        print()
        return 0
    for k in ("analog", "digital", "power", "gnd", "other", "ignore"):
        nets = cls.nets_of(k)
        if not nets:
            continue
        print(f"## {k} ({len(nets)})")
        for n in nets:
            print(f"  {n:28s} {cls.why[n]}")
    print(f"## differential pairs ({len(cls.pairs)})")
    for a, b in cls.pairs:
        print(f"  {a} / {b}")
    return 0


def cmd_report(args) -> int:
    from .report import build
    board, rules, cls = _load(args)
    rep = build(board, cls, rules)
    if args.out:
        os.makedirs(args.out, exist_ok=True)
        stem = os.path.splitext(os.path.basename(args.board))[0]
        with open(os.path.join(args.out, f"{stem}_quality.json"), "w", encoding="utf-8") as f:
            json.dump(rep.data, f, ensure_ascii=False, indent=1)
        with open(os.path.join(args.out, f"{stem}_quality.md"), "w", encoding="utf-8") as f:
            f.write(rep.to_markdown())
    if args.json:
        json.dump({k: rep.data[k] for k in ("score", "score_parts", "unrouted", "summary")}, sys.stdout, indent=1)
        print()
    else:
        print(rep.to_markdown())
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="kar", description="kicad-audio-router: audio-aware routing helpers for KiCad")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, fn, hlp in (("classify", cmd_classify, "show net classification"),
                          ("report", cmd_report, "routing quality report")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("board")
        p.add_argument("--rules", help="rules JSON (see kar/rules.py)")
        p.add_argument("--json", action="store_true", help="machine-readable output")
        if name == "report":
            p.add_argument("--out", help="write <board>_quality.json / .md here")
        p.set_defaults(fn=fn)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
