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


def test_demo_auto():
    py = kicad_python()
    if not py:
        print("skip: KiCad Python not found")
        return
    subprocess.run([py, os.path.join(ROOT, "examples", "make_demo.py")], cwd=ROOT, check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "demo.kicad_pcb")
        r = subprocess.run([py, "-m", "kar", "auto", os.path.join(ROOT, "examples", "demo_receiver.kicad_pcb"),
                            "-o", out], cwd=ROOT, capture_output=True, text=True)
        t = r.stdout
        s = json.loads(t[t.rfind('{\n "saved"'):])
        assert s["unrouted"] == 0, s
        if s["drc"] is not None:
            assert s["drc"]["unconnected"] == 0 and not s["drc"]["problems"], s["drc"]
        assert s["score"] > 70, s


if __name__ == "__main__":
    test_demo_auto()
    print("ok test_demo_auto")
