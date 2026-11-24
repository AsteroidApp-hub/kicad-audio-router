"""Build the demo board examples/demo_receiver.kicad_pcb (parts placed, nothing routed).

A balanced line receiver next to a little shift-register LED driver, laid out so that the digital lines
have to get past the analog path. Footprints come from KiCad's own libraries.

Run with KiCad's Python:   $KPY examples/make_demo.py
Then route it:             $KPY -m kar auto examples/demo_receiver.kicad_pcb -o out/demo_routed.kicad_pcb
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kar.demo import build  # noqa: E402

if __name__ == "__main__":
    print("written", build(os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_receiver.kicad_pcb")))
