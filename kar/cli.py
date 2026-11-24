"""Command line entry point.

  python -m kar classify BOARD.kicad_pcb [--rules R.json]          # how nets were classified
  python -m kar report   BOARD.kicad_pcb [--rules R.json] [--out DIR] [--json]
  python -m kar route    BOARD.kicad_pcb [--classes analog,power,digital,other] [--nets A,B] [-o OUT]
  python -m kar strip    BOARD.kicad_pcb -o OUT.kicad_pcb            # remove tracks/vias (experiments)
  python -m kar ground   BOARD.kicad_pcb [--net GND,/CHASSIS] [--pour F.Cu,B.Cu] [--max-dist 3] [-o OUT.kicad_pcb]

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


def cmd_ground(args) -> int:
    from . import kicad_io
    from .ground import GroundParams, Obstacles, plan_fanout, plan_islands
    from .model import Via
    _, kb = kicad_io.load(args.board)
    nets = [n.strip() for n in args.net.split(",") if n.strip()]
    results, bad = [], False
    for net in nets:
        prm = GroundParams(net=net, max_dist=args.max_dist, via_d=args.via, via_drill=args.drill,
                           clearance=args.clearance, max_radius=args.max_radius)
        if kb.FindNet(net) is None:
            raise SystemExit(f"net {net!r} not found")
        res = {"net": net, "pours_added": [], "fanout_vias": 0, "fanout_failed": [], "island_vias": 0,
               "islands_failed": 0}
        model = kicad_io.from_pcbnew(kb, args.board)
        if args.pour and net == nets[0]:          # pours only for the first (main ground) net
            have = {(n, l) for n, l in model.zones}
            for layer in [l.strip() for l in args.pour.split(",") if l.strip()]:
                if (net, layer) in have:
                    continue
                for poly in model.outline:
                    kicad_io.add_pour(kb, net, layer, poly, clearance=args.pour_clearance)
                res["pours_added"].append(layer)
        kicad_io.refill(kb)
        model = kicad_io.from_pcbnew(kb, args.board)
        plans, failed = plan_fanout(model, prm, Obstacles(model))
        res["fanout_vias"] = kicad_io.apply_vias(kb, plans, net, prm.via_d, prm.via_drill, prm.track_w)
        res["fanout_failed"] = failed
        kicad_io.refill(kb)
        model = kicad_io.from_pcbnew(kb, args.board)
        obst = Obstacles(model)
        for layer in sorted({l for n, l in model.zones if n == net}):
            iplans, ifail = plan_islands(model, kicad_io.islands(kb, net, layer), prm, obst)
            res["island_vias"] += kicad_io.apply_vias(kb, iplans, net, prm.via_d, prm.via_drill, prm.track_w)
            res["islands_failed"] += ifail
            model.vias += [Via(p.x, p.y, net, prm.via_d, prm.via_drill) for p in iplans]   # seen by next layer
        kicad_io.refill(kb)
        bad |= bool(failed or res["islands_failed"])
        results.append(res)
    out = args.output or args.board
    kicad_io.save(kb, out)
    print(json.dumps({"saved": out, "nets": results}, ensure_ascii=False, indent=1))
    return 2 if bad else 0


def cmd_route(args) -> int:
    import time
    from . import kicad_io
    from .classify import classify
    from .router import RouteParams, route
    board, kb = kicad_io.load(args.board)
    rules = R.load(args.rules)
    cls = classify(board, rules)
    prm = RouteParams(res=args.grid, clearance=args.clearance)
    if args.layers:
        prm.layers = [l.strip() for l in args.layers.split(",")]
    prm.classes = tuple(c.strip() for c in args.classes.split(",") if c.strip())
    nets = [n.strip() for n in args.nets.split(",")] if args.nets else None
    t0 = time.time()

    def prog(net, ok):
        if not args.json:
            print(f"  {'ok  ' if ok else 'FAIL'} {net}  ({time.time() - t0:.0f}s)", file=sys.stderr)
    res = route(board, cls, prm, rules["activity_weight"], nets, prog)
    nt, nv = kicad_io.apply_routes(kb, res.routed, prm.via_d, prm.via_drill, lock=not args.no_lock)
    kicad_io.refill(kb)
    out = args.output or args.board
    kicad_io.save(kb, out)
    summary = {"saved": out, "routed": len(res.routed), "failed": res.failed, "tracks": nt, "vias": nv,
               "layers": prm.layers or [l for l in board.copper_layers if l not in board.plane_layers],
               "seconds": round(time.time() - t0, 1)}
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0 if not res.failed else 2


def cmd_strip(args) -> int:
    from . import kicad_io
    _, kb = kicad_io.load(args.board)
    n = kicad_io.strip(kb, keep_locked=args.keep_locked)
    kicad_io.save(kb, args.output)
    print(json.dumps({"removed": n, "saved": args.output}))
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
    r = sub.add_parser("route", help="audio-aware routing (pairs, analog, power, digital)")
    r.add_argument("board")
    r.add_argument("-o", "--output")
    r.add_argument("--rules")
    r.add_argument("--classes", default="analog", help="net classes to route, in order (default: analog)")
    r.add_argument("--nets", help="only these nets (comma-separated)")
    r.add_argument("--layers", help="routing layers (default: copper layers except power planes)")
    r.add_argument("--grid", type=float, default=0.25)
    r.add_argument("--clearance", type=float, default=0.2)
    r.add_argument("--no-lock", action="store_true", help="do not lock the new tracks")
    r.add_argument("--json", action="store_true")
    r.set_defaults(fn=cmd_route)
    st = sub.add_parser("strip", help="remove tracks and vias")
    st.add_argument("board")
    st.add_argument("-o", "--output", required=True)
    st.add_argument("--keep-locked", action="store_true")
    st.set_defaults(fn=cmd_strip)
    g = sub.add_parser("ground", help="ground pours, fan-out vias and island repair")
    g.add_argument("board")
    g.add_argument("-o", "--output", help="write here instead of overwriting the board")
    g.add_argument("--net", default="GND", help="ground net(s), comma-separated, e.g. GND,/CHASSIS")
    g.add_argument("--pour", default="", help="comma-separated layers to pour the first net on, e.g. F.Cu,B.Cu")
    g.add_argument("--pour-clearance", type=float, default=0.3)
    g.add_argument("--max-dist", type=float, default=3.0, help="pads with a ground via this close are left alone")
    g.add_argument("--max-radius", type=float, default=6.0, help="how far from a pad to search for a via site")
    g.add_argument("--via", type=float, default=0.6, help="via diameter")
    g.add_argument("--drill", type=float, default=0.3)
    g.add_argument("--clearance", type=float, default=0.2)
    g.set_defaults(fn=cmd_ground)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
