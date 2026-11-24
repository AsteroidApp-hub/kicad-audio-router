"""End-to-end test with KiCad: build the demo board, route it, check DRC.
Skipped when KiCad's Python is not available. Run:  python3 tests/test_kicad.py
(set KICAD_PYTHON to KiCad's python if it is not in the usual place)"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def kicad_python():
    for c in (os.environ.get("KICAD_PYTHON"),
              "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
              r"C:\Program Files\KiCad\9.0\bin\python.exe", r"C:\Program Files\KiCad\10.0\bin\python.exe"):
        if c and os.path.exists(c):
            return c
    try:
        import pcbnew  # noqa: F401
        return sys.executable
    except ImportError:
        return None


def _demo(py, d):
    """Build the demo board in a temporary folder (the copy in examples/ stays untouched)."""
    path = os.path.join(d, "demo_receiver.kicad_pcb")
    subprocess.run([py, "-c", f"from kar.demo import build; build({path!r})"], cwd=ROOT, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return path


def test_demo_auto():
    py = kicad_python()
    if not py:
        print("skip: KiCad Python not found")
        return
    with tempfile.TemporaryDirectory() as d:
        demo = _demo(py, d)
        out = os.path.join(d, "routed", "demo.kicad_pcb")
        r = subprocess.run([py, "-m", "kar", "auto", demo, "-o", out], cwd=ROOT, capture_output=True, text=True)
        t = r.stdout
        s = json.loads(t[t.rfind('{\n "saved"'):])
        assert s["unrouted"] == 0, s
        if s["drc"] is not None:
            assert s["drc"]["unconnected"] == 0 and not s["drc"]["problems"], s["drc"]
        assert s["score"] > 70, s


EDIT = r"""
import sys, shutil, pcbnew
src, dst = sys.argv[1], sys.argv[2]
b = pcbnew.LoadBoard(src)
for g in b.Groups():                       # keep /BUF: take its copper out of the kar group
    for it in list(g.GetItems()):
        if it.GetNetname() == "/BUF":
            g.RemoveItem(it)
for fp in b.GetFootprints():               # a hand-drawn stub on /QA
    if fp.GetReference() == "U2":
        p = [q for q in fp.Pads() if q.GetNumber() == "15"][0]
        t = pcbnew.PCB_TRACK(b); t.SetStart(p.GetPosition())
        t.SetEnd(pcbnew.VECTOR2I(p.GetPosition().x, p.GetPosition().y + pcbnew.FromMM(2.0)))
        t.SetWidth(pcbnew.FromMM(0.2)); t.SetLayer(pcbnew.F_Cu); t.SetNet(p.GetNet()); b.Add(t)
pcbnew.SaveBoard(dst, b)
shutil.copyfile(src[:-10] + ".kicad_pro", dst[:-10] + ".kicad_pro")
"""

DUMP = r"""
import sys, json, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
print(json.dumps(sorted([t.GetNetname(), t.GetStart().x, t.GetStart().y, t.GetEnd().x, t.GetEnd().y]
                        for t in b.GetTracks() if t.GetClass() == "PCB_TRACK" and t.GetNetname() in ("/BUF", "/QA"))))
"""


def test_reroute_keeps_hand_copper():
    py = kicad_python()
    if not py:
        print("skip: KiCad Python not found")
        return
    with tempfile.TemporaryDirectory() as d:
        demo = _demo(py, d)
        r1, ed, r2 = (os.path.join(d, n + ".kicad_pcb") for n in ("r1", "edited", "r2"))
        subprocess.run([py, "-m", "kar", "auto", demo, "-o", r1], cwd=ROOT, check=True, capture_output=True)
        subprocess.run([py, "-c", EDIT, r1, ed], check=True, capture_output=True)
        out = subprocess.run([py, "-m", "kar", "auto", ed, "-o", r2, "--rip-kar"], cwd=ROOT, capture_output=True,
                             text=True).stdout
        s = json.loads(out[out.rfind('{\n "saved"'):])
        assert s["unrouted"] == 0 and (s["drc"] is None or (s["drc"]["unconnected"] == 0 and not s["drc"]["problems"]))
        before = json.loads(subprocess.run([py, "-c", DUMP, ed], capture_output=True, text=True).stdout.splitlines()[-1])
        after = json.loads(subprocess.run([py, "-c", DUMP, r2], capture_output=True, text=True).stdout.splitlines()[-1])
        kept = [t for t in before if t[0] == "/BUF"] + [t for t in before if t[0] == "/QA" and t[1] == t[3]]
        assert kept and all(t in after for t in kept), "hand copper was changed"


if __name__ == "__main__":
    test_demo_auto()
    print("ok test_demo_auto")
    test_reroute_keeps_hand_copper()
    print("ok test_reroute_keeps_hand_copper")
