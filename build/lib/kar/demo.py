"""Demo board: a balanced line receiver next to a shift-register LED driver, parts placed, nothing
routed. Footprints come from KiCad's own libraries. Used by examples/make_demo.py and `kar doctor --demo`."""
import os
import sys

import pcbnew


def fp_dir() -> str:
    for c in (os.environ.get("KICAD_FOOTPRINT_DIR"), os.environ.get("KICAD9_FOOTPRINT_DIR"),
              os.environ.get("KICAD10_FOOTPRINT_DIR"),
              "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints",
              "/usr/share/kicad/footprints", r"C:\Program Files\KiCad\9.0\share\kicad\footprints",
              r"C:\Program Files\KiCad\10.0\share\kicad\footprints"):
        if c and os.path.isdir(c):
            return c
    sys.exit("KiCad footprint libraries not found; set KICAD_FOOTPRINT_DIR")


mm = pcbnew.FromMM
W, H = 80.0, 60.0

# ref, value, library, footprint, x, y, rotation, {pad: net}
PARTS = [
    # analog: XLR in -> RF caps -> difference amplifier -> follower -> output header
    ("J1", "XLR IN", "Connector_Audio", "Jack_XLR_Neutrik_NC3FAAH_Horizontal", 10, 16.5, 90,
     {"1": "GND", "2": "/IN_H", "3": "/IN_C"}),
    ("C1", "470p", "Capacitor_SMD", "C_0805_2012Metric", 30, 10, 0, {"1": "/IN_H", "2": "GND"}),
    ("C2", "470p", "Capacitor_SMD", "C_0805_2012Metric", 30, 22, 0, {"1": "/IN_C", "2": "GND"}),
    ("R1", "10k", "Resistor_SMD", "R_0805_2012Metric", 36, 12, 0, {"1": "/IN_H", "2": "/RX_P"}),
    ("R2", "10k", "Resistor_SMD", "R_0805_2012Metric", 36, 8, 0, {"1": "/RX_P", "2": "GND"}),
    ("R3", "10k", "Resistor_SMD", "R_0805_2012Metric", 36, 20, 0, {"1": "/IN_C", "2": "/RX_N"}),
    ("R4", "10k", "Resistor_SMD", "R_0805_2012Metric", 46, 25, 0, {"1": "/RX_N", "2": "/RX_OUT"}),
    ("U1", "OPA1612", "Package_SO", "SOIC-8_3.9x4.9mm_P1.27mm", 47, 15, 0,
     {"1": "/RX_OUT", "2": "/RX_N", "3": "/RX_P", "4": "/-15V", "5": "/RX_OUT", "6": "/BUF", "7": "/BUF",
      "8": "/+15V"}),
    ("C3", "100n", "Capacitor_SMD", "C_0805_2012Metric", 53, 10, 90, {"1": "/+15V", "2": "GND"}),
    ("C4", "100n", "Capacitor_SMD", "C_0805_2012Metric", 41, 16, 90, {"1": "GND", "2": "/-15V"}),
    ("R5", "47", "Resistor_SMD", "R_0805_2012Metric", 60, 15, 0, {"1": "/BUF", "2": "/OUT"}),
    ("J2", "OUT", "Connector_PinHeader_2.54mm", "PinHeader_1x02_P2.54mm_Vertical", 73, 8, 0,
     {"1": "/OUT", "2": "GND"}),
    # power in
    ("J3", "PWR", "Connector_PinHeader_2.54mm", "PinHeader_1x04_P2.54mm_Vertical", 76, 20, 0,
     {"1": "/+15V", "2": "GND", "3": "/-15V", "4": "/+5V"}),
    # digital: control header on the left, shift register and LEDs bottom right
    ("J4", "CTRL", "Connector_PinHeader_2.54mm", "PinHeader_1x03_P2.54mm_Vertical", 6, 38, 0,
     {"1": "/SCLK", "2": "/SDATA", "3": "/LATCH"}),
    ("U2", "74HC595", "Package_SO", "SOIC-16_3.9x9.9mm_P1.27mm", 52, 44, 90,
     {"8": "GND", "16": "/+5V", "13": "GND", "10": "/+5V", "11": "/SCLK", "14": "/SDATA", "12": "/LATCH",
      "15": "/QA", "1": "/QB", "2": "/QC"}),
    ("C5", "100n", "Capacitor_SMD", "C_0805_2012Metric", 44, 38, 90, {"1": "/+5V", "2": "GND"}),
    ("R6", "1k", "Resistor_SMD", "R_0805_2012Metric", 64, 40, 0, {"1": "/QA", "2": "/LA"}),
    ("R7", "1k", "Resistor_SMD", "R_0805_2012Metric", 64, 45, 0, {"1": "/QB", "2": "/LB"}),
    ("R8", "1k", "Resistor_SMD", "R_0805_2012Metric", 64, 50, 0, {"1": "/QC", "2": "/LC"}),
    ("D1", "LED", "LED_SMD", "LED_0805_2012Metric", 72, 40, 0, {"1": "GND", "2": "/LA"}),
    ("D2", "LED", "LED_SMD", "LED_0805_2012Metric", 72, 45, 0, {"1": "GND", "2": "/LB"}),
    ("D3", "LED", "LED_SMD", "LED_0805_2012Metric", 72, 50, 0, {"1": "GND", "2": "/LC"}),
    ("H1", "", "MountingHole", "MountingHole_3.2mm_M3", 4, 56, 0, {}),
    ("H2", "", "MountingHole", "MountingHole_3.2mm_M3", 76, 56, 0, {}),
]


def build(out: str) -> str:
    """Write the demo board (and its project file) to `out`."""
    LIB = fp_dir()
    b = pcbnew.BOARD()
    b.SetCopperLayerCount(4)
    try:
        b.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)     # ground plane
    except Exception:   # noqa: BLE001
        pass
    nets = {}

    def net(name):
        if name not in nets:
            n = pcbnew.NETINFO_ITEM(b, name)
            b.Add(n)
            nets[name] = n
        return nets[name]
    for ref, val, lib, name, x, y, rot, pads in PARTS:
        fp = pcbnew.FootprintLoad(os.path.join(LIB, lib + ".pretty"), name)
        if fp is None:
            sys.exit(f"footprint not found: {lib}:{name}")
        fp.SetReference(ref)
        fp.SetValue(val)
        b.Add(fp)
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        fp.SetOrientationDegrees(rot)
        for p in fp.Pads():
            if p.GetNumber() in pads:
                p.SetNet(net(pads[p.GetNumber()]))
    pts = [(0, 0), (W, 0), (W, H), (0, H)]
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pcbnew.VECTOR2I(mm(x1), mm(y1)))
        s.SetEnd(pcbnew.VECTOR2I(mm(x2), mm(y2)))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        b.Add(s)
    # ground plane on In1
    z = pcbnew.ZONE(b)
    z.SetLayer(pcbnew.In1_Cu)
    z.SetNet(net("GND"))
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((0.3, 0.3), (W - 0.3, 0.3), (W - 0.3, H - 0.3), (0.3, H - 0.3)):
        ol.Append(mm(x), mm(y))
    z.SetLocalClearance(mm(0.3))
    z.SetMinThickness(mm(0.25))
    b.Add(z)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(out, b)
    return out
