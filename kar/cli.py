"""Command line entry point.

  python -m kar classify BOARD.kicad_pcb [--rules R.json]          # how nets were classified
  python -m kar report   BOARD.kicad_pcb [--rules R.json] [--out DIR] [--json]
  python -m kar route    BOARD.kicad_pcb [--classes analog,power,digital,other] [--nets A,B] [-o OUT]
  python -m kar finish   BOARD.kicad_pcb --jar freerouting.jar [--java java] [--passes 30] [-o OUT]
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
        if failed:
            # no straight shot to a via site: walk around other copper with the router
            from .classify import classify
            from .router import RouteParams, escape_to_via
            model = kicad_io.from_pcbnew(kb, args.board)
            rules = R.load(None)
            routes, failed = escape_to_via(model, classify(model, rules), RouteParams(clearance=prm.clearance,
                                           via_d=prm.via_d, via_drill=prm.via_drill), rules["activity_weight"],
                                           net, failed, width=prm.track_w)
            kicad_io.apply_routes(kb, routes, prm.via_d, prm.via_drill, lock=False)
            res["escape_vias"] = len(routes)
        res["fanout_failed"] = failed
        kicad_io.refill(kb)
        model = kicad_io.from_pcbnew(kb, args.board)
        obst = Obstacles(model)
        stuck = []
        for layer in sorted({l for n, l in model.zones if n == net}):
            iplans, ifail = plan_islands(model, kicad_io.islands(kb, net, layer), prm, obst, failed_polys=stuck)
            res["island_vias"] += kicad_io.apply_vias(kb, iplans, net, prm.via_d, prm.via_drill, prm.track_w)
            res["islands_failed"] += ifail
            model.vias += [Via(p.x, p.y, net, prm.via_d, prm.via_drill) for p in iplans]   # seen by next layer
        if stuck:
            # an island too small or crowded for a via: walk from a pad on it to a via site
            from .classify import classify
            from .ground import pads_on_island
            from .router import RouteParams, escape_to_via
            kicad_io.refill(kb)
            model = kicad_io.from_pcbnew(kb, args.board)
            names = sorted({n for poly in stuck for n in pads_on_island(model, poly, net)})
            rules = R.load(None)
            routes, efail = escape_to_via(model, classify(model, rules), RouteParams(clearance=prm.clearance,
                                          via_d=prm.via_d, via_drill=prm.via_drill), rules["activity_weight"],
                                          net, names, width=prm.track_w)
            kicad_io.apply_routes(kb, routes, prm.via_d, prm.via_drill, lock=False)
            res["island_escapes"] = len(routes)
            res["islands_failed"] -= sum(1 for poly in stuck
                                         if any(n not in efail for n in pads_on_island(model, poly, net)))
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
    # ground fan-out first (like a designer would): every SMD ground pad gets its via before signals
    # are routed, so signal tracks cannot wall it in
    fan = {}
    if not args.no_fanout:
        from .geometry import Seg
        from .ground import GroundParams, Obstacles, plan_fanout
        from .model import Via
        for gnet in cls.nets_of("gnd"):
            # a ground via "nearby" is not enough here: signal tracks may later run between the pad and it,
            # so (almost) every SMD ground pad gets its own via
            gp = GroundParams(net=gnet, clearance=args.clearance, max_dist=args.fanout_dist)
            plans, _ = plan_fanout(board, gp, Obstacles(board))
            kicad_io.apply_vias(kb, plans, gnet, gp.via_d, gp.via_drill, gp.track_w)
            for pl in plans:
                board.vias.append(Via(pl.x, pl.y, gnet, gp.via_d, gp.via_drill))
                if pl.track:
                    x1, y1, x2, y2, layer = pl.track
                    board.segs.append(Seg(x1, y1, x2, y2, gp.track_w, layer, gnet))
            fan[gnet] = len(plans)

    def prog(net, ok):
        if not args.json:
            print(f"  {'ok  ' if ok else 'FAIL'} {net}  ({time.time() - t0:.0f}s)", file=sys.stderr)
    res = route(board, cls, prm, rules["activity_weight"], nets, prog)
    nt, nv = kicad_io.apply_routes(kb, res.routed, prm.via_d, prm.via_drill, lock=not args.no_lock)
    kicad_io.refill(kb)
    out = args.output or args.board
    kicad_io.save(kb, out)
    summary = {"saved": out, "ground_fanout": fan, "routed": len(res.routed), "failed": res.failed,
               "tracks": nt, "vias": nv,
               "layers": prm.layers or [l for l in board.copper_layers if l not in board.plane_layers],
               "seconds": round(time.time() - t0, 1)}
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0 if not res.failed else 2


def cmd_finish(args) -> int:
    import time
    from . import freerouting as FR
    from . import kicad_io
    from .classify import classify
    jar = args.jar or os.environ.get("KAR_FREEROUTING_JAR")
    java = args.java or os.environ.get("KAR_JAVA", "java")
    if not jar or not os.path.exists(jar):
        raise SystemExit("Freerouting jar not found: pass --jar or set KAR_FREEROUTING_JAR")
    board, kb = kicad_io.load(args.board)
    cls = classify(board, R.load(args.rules))
    exclude = set(cls.nets_of("gnd")) | set(cls.nets_of("ignore"))
    if args.route_ground:
        exclude -= set(cls.nets_of("gnd"))
    out = args.output or args.board
    work = os.path.join(os.path.dirname(os.path.abspath(out)), "freerouting_work")
    os.makedirs(work, exist_ok=True)
    stem = os.path.splitext(os.path.basename(out))[0]
    dsn, ses, log = (os.path.join(work, stem + e) for e in (".dsn", ".ses", ".log"))
    locked_before = sum(1 for t in kb.GetTracks() if t.IsLocked())
    t0 = time.time()
    dropped = FR.export_dsn(kb, dsn, exclude)
    FR.run(dsn, ses, jar, java, args.passes, args.timeout, log)
    FR.import_ses(kb, ses)
    locked_after = sum(1 for t in kb.GetTracks() if t.IsLocked())
    removed = FR.cleanup_dangling(kb)
    kicad_io.refill(kb)
    kicad_io.save(kb, out)
    after = kicad_io.from_pcbnew(kb, out)
    print(json.dumps({"saved": out, "excluded_nets": dropped, "locked_tracks": [locked_before, locked_after],
                      "dangling_removed": removed, "unrouted": after.unrouted, "log": log,
                      "seconds": round(time.time() - t0, 1)}, ensure_ascii=False, indent=1))
    return 0


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
    r.add_argument("--no-fanout", action="store_true", help="do not place ground vias before routing")
    r.add_argument("--fanout-dist", type=float, default=1.0,
                   help="before routing, ground pads without a ground via this close get one (default 1 mm)")
    r.add_argument("--json", action="store_true")
    r.set_defaults(fn=cmd_route)
    fz = sub.add_parser("finish", help="route what is left with Freerouting, keeping locked tracks")
    fz.add_argument("board")
    fz.add_argument("-o", "--output")
    fz.add_argument("--rules")
    fz.add_argument("--jar", help="freerouting.jar (or env KAR_FREEROUTING_JAR)")
    fz.add_argument("--java", help="java executable (or env KAR_JAVA; Freerouting 2.1 needs Java 21)")
    fz.add_argument("--passes", type=int, default=30)
    fz.add_argument("--timeout", type=int, default=3600)
    fz.add_argument("--route-ground", action="store_true", help="also let Freerouting route ground nets")
    fz.set_defaults(fn=cmd_finish)
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
