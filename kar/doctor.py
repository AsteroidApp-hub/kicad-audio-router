"""`kar doctor`: check that this KiCad installation offers everything kar uses, then (optionally)
build and route the demo board. Meant for checking new KiCad versions and platforms."""
from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
import tempfile


def _check(name, fn):
    try:
        detail = fn()
        return {"check": name, "ok": True, "detail": detail}
    except Exception as e:   # noqa: BLE001
        return {"check": name, "ok": False, "detail": f"{type(e).__name__}: {e}"}


def checks() -> list[dict]:
    import pcbnew
    from . import kicad_io
    res = []
    ver = getattr(pcbnew, "GetBuildVersion", lambda: "?")()
    res.append({"check": "KiCad version", "ok": True, "detail": ver})
    res.append({"check": "Python", "ok": sys.version_info >= (3, 9), "detail": platform.python_version()})
    res.append({"check": "platform", "ok": True, "detail": f"{platform.system()} {platform.machine()}"})

    b = pcbnew.BOARD()
    b.SetCopperLayerCount(4)
    fp = pcbnew.FOOTPRINT(b)
    b.Add(fp)
    pad = pcbnew.PAD(fp)
    pad.SetShape(getattr(pcbnew, "PAD_SHAPE_ROUNDRECT", pcbnew.PAD_SHAPE_RECT))
    pad.SetSize(pcbnew.VECTOR2I(pcbnew.FromMM(1.0), pcbnew.FromMM(0.6)))
    pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
    pad.SetLayerSet(pad.SMDMask())
    fp.Add(pad)

    res.append(_check("pad outline (GetEffectivePolygon)",
                      lambda: f"{len(kicad_io._pad_poly(pad) or [])} points"))
    res.append(_check("pad clearance (GetOwnClearance)",
                      lambda: f"{pcbnew.ToMM(pad.GetOwnClearance(pcbnew.F_Cu)):.3f} mm"))
    res.append(_check("layer types (LT_POWER)", lambda: str(b.GetLayerType(pcbnew.In1_Cu))))
    res.append(_check("hole clearance setting",
                      lambda: f"{pcbnew.ToMM(b.GetDesignSettings().m_HoleClearance):.3f} mm"))
    res.append(_check("board outline polygons", lambda: str(len(kicad_io._outline(b)))))

    def zone():
        z = pcbnew.ZONE(b)
        z.SetLayer(pcbnew.F_Cu)
        z.Outline().NewOutline()
        for x, y in ((0, 0), (10, 0), (10, 10), (0, 10)):
            z.Outline().Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks(True)
        b.Add(z)
        return f"rule area: tracks={z.GetDoNotAllowTracks()} vias={z.GetDoNotAllowVias()}"
    res.append(_check("zones and rule areas", zone))
    res.append(_check("zone filler", lambda: str(pcbnew.ZONE_FILLER(b).Fill(b.Zones()))))

    def track():
        t = pcbnew.PCB_TRACK(b)
        t.SetWidth(pcbnew.FromMM(0.25))
        t.SetLocked(True)
        v = pcbnew.PCB_VIA(b)
        try:
            v.SetWidth(pcbnew.F_Cu, pcbnew.FromMM(0.6))
        except TypeError:
            v.SetWidth(pcbnew.FromMM(0.6))
        return "tracks, vias, locking"
    res.append(_check("tracks and vias", track))
    res.append(_check("connectivity", lambda: str(kicad_io._unrouted(b))))
    from .cli import _kicad_cli
    cli = _kicad_cli()
    res.append({"check": "kicad-cli (for DRC)", "ok": cli is not None, "detail": cli or "not found (set KICAD_CLI)"})
    return res


def demo_run() -> dict:
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    py = sys.executable
    subprocess.run([py, os.path.join(root, "examples", "make_demo.py")], cwd=root, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "demo.kicad_pcb")
        r = subprocess.run([py, "-m", "kar", "auto", os.path.join(root, "examples", "demo_receiver.kicad_pcb"),
                            "-o", out], cwd=root, capture_output=True, text=True)
        t = r.stdout
        i = t.rfind('{\n "saved"')
        if i < 0:
            return {"check": "demo board routed", "ok": False, "detail": (r.stderr or t)[-400:]}
        s = json.loads(t[i:])
        d_ = s.get("drc") or {}
        ok = s.get("unrouted") == 0 and not d_.get("problems") and d_.get("unconnected", 0) == 0
        return {"check": "demo board routed", "ok": ok,
                "detail": f"score {s.get('score')}, unrouted {s.get('unrouted')}, drc {d_.get('problems')}, "
                          f"{s.get('seconds')} s"}


def main(run_demo: bool, as_json: bool) -> int:
    res = checks()
    if run_demo:
        res.append(demo_run())
    if as_json:
        print(json.dumps(res, ensure_ascii=False, indent=1))
    else:
        for r in res:
            print(f"{'ok ' if r['ok'] else 'NG '} {r['check']:36s} {r['detail']}")
    return 0 if all(r["ok"] for r in res) else 1
