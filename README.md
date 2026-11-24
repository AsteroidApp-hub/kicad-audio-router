# kicad-audio-router

Audio-aware autorouter and routing checks for KiCad. **Status: early development (not released).**

[日本語](README.ja.md)

General-purpose autorouters connect everything but know nothing about audio: they run analog signals
next to clocks, split balanced pairs, and leave ground pads far from a via. kicad-audio-router reads your
board, works out which nets are analog / digital / power / ground, and routes it with audio in mind:

- analog tracks keep away from digital copper (weighted by how active the digital net is)
- the two sides of a balanced pair run side by side
- every SMD ground pad gets its own via to the plane before signals are routed
- clearances are checked against the real copper shapes, per net class, including hole clearance
  and keep-out areas, so the result passes KiCad's DRC

It is pure Python on top of KiCad's own `pcbnew` module (tested on KiCad 10, KiCad 9 in CI; see
[docs/compatibility.md](docs/compatibility.md)): no Java, no extra packages,
deterministic (same board + rules = same result), with JSON output for scripted or AI-driven flows.

## Try it

```sh
$KPY examples/make_demo.py                                   # builds examples/demo_receiver.kicad_pcb
$KPY -m kar auto examples/demo_receiver.kicad_pcb -o out/demo.kicad_pcb
$KPY -m kar plot out/demo.kicad_pcb -o out/demo.svg
```

The demo is a balanced line receiver (XLR, OPA1612) next to a 74HC595 LED driver whose control lines have
to get past the analog path. Routed in about 4 s:

![demo board routed by kar](docs/demo_routed.svg)

## Results so far

Seven real audio boards (monitor controller, mic preamps, phantom supplies, VU meter, tape saturator),
stripped of all tracks and routed from scratch with `kar auto`, compared with their original routing
(hand-routed or Freerouting). Score: `kar report` (0–100, higher is better).

| board | layers | original | kar | time |
|---|---|---|---|---|
| monitor controller (~270 parts, relays, MUSES72320) | 4 | 52.4 | 85.9 | 143 s |
| VU meter (USB-C) | 2 | 75.7 | 94.1 | 24 s |
| tape saturator | 4 | 81.1 | 91.0 | 59 s |
| P48 phantom supply | 4 | 82.5 | 85.0 | 23 s |
| 2-ch mic preamp (discrete) | 4 | 79.6 | 82.5 | 28 s |
| mic preamp (op-amp) | 2 | 78.5 | 86.0 | 81 s |
| phantom power unit | 2 | 99.3 | 100.0 | 8 s |

All boards fully routed with no DRC problems introduced by kar (only issues already present in the
originals, such as overlapping courtyards, remain).

## Usage

Run with the Python that ships with KiCad (it contains `pcbnew`):

```sh
# macOS
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
# Windows:  "C:\Program Files\KiCad\9.0\bin\python.exe"     Linux: python3 (with KiCad installed)

$KPY -m kar auto     board.kicad_pcb -o routed.kicad_pcb      # route + ground + DRC + report
$KPY -m kar classify board.kicad_pcb                         # how each net was classified, and why
$KPY -m kar report   board.kicad_pcb --out out/              # quality report (Markdown + JSON)
```

Place the parts first (kar does not move footprints). Existing tracks are kept; `kar strip` removes them
if you want to start over. Always review the result in KiCad and run its DRC before ordering.

### Commands

| command | what it does |
|---|---|
| `auto` | `route` → `ground` → KiCad DRC (kicad-cli) → `report`, one summary in JSON |
| `route` | routes the nets (`--classes analog,power,digital,other`); ground fan-out first |
| `ground` | ground vias for pads far from one, island repair, optional pours (`--pour F.Cu,B.Cu`) |
| `report` | routing quality report and score |
| `classify` | net classification with reasons |
| `bench DIR` | strips and re-routes every board in a folder, compares with the original routing |
| `plot` | SVG picture of the routing, one panel per layer, tracks coloured by net class |
| `doctor` | checks that your KiCad offers what kar needs (`--demo`: also routes the demo board) |
| `strip` | removes tracks and vias |
| `finish` | optional: hands nets kar could not route to Freerouting (DSN/SES); not needed so far |

### How it routes

- A* search on a 0.25 mm grid over the routing layers (inner layers typed *power* in the board setup are
  left to planes), 8 directions plus vias; the grid is aligned to fine-pitch pads.
- Clearance is checked exactly near copper: the distance from a grid point to the real pad outline /
  track / via must leave the net-class clearance plus half the track width, with an allowance for the
  straight run to the next grid point.
- Cost fields: analog nets pay for running near digital copper, digital nets for running near analog.
  The second side of a pair pays for leaving the line one track pitch beside the first side; if the
  sides do not run together, a stronger pull and the reverse order are tried.
- Order: pairs, analog, power (wide, narrowed only where needed), digital, other.
- Nets that fail are routed again through other routes at a price ("push"), and the nets they cross are
  re-routed; what still fails is tried on a grid twice as fine, aligned to its pads.

### What the report measures

| metric | meaning |
|---|---|
| coupling | analog copper within 1 mm / 3 mm (edge to edge, same layer) of digital copper, weighted by how active the digital net is (clock 1.0, data 0.6, static 0.15) |
| differential pairs | length of each side, mismatch, share routed side by side |
| ground return | SMD pads of each ground net farther than 3 mm (from the pad edge) from a via / through-hole pad of the same net |
| power width | power nets with tracks narrower than the limit |
| analog vias | vias on analog nets |
| unrouted | unconnected items reported by KiCad |

The score weights these for audio work so that two routings of the same board can be compared.
It is a comparison aid, not an absolute grade.

### How nets are classified

1. explicit globs in the rules file
2. ground / power by name (`GND`, `AGND`, `CHASSIS`, `+15V`, `VCC`, …)
3. digital by name (`SCLK`, `SDA`, `LATCH`, …), by touching a digital IC (`74*`, `ULN*`, MCU, …),
   or a digital pin of a mixed-signal IC
4. analog: pins of analog ICs (`OPA*`, `NE5532`, `NJM*`, `MUSES*`, …) and audio connectors (by footprint,
   value or reference: XLR, jack, `MIC IN`, `OUT L`, `JINL`), then spread through resistors, capacitors,
   transistors, switches, transformers, pots, diodes and relay contacts
5. pairs: `_H`/`_C`, `_P`/`_N`, `+`/`-` suffixes and `HOT`/`COLD`, `POS`/`NEG` anywhere in the name

House-style designators (`RGaR` resistor, `DBL` diode, `RLL` + value `HFD27` relay) are recognised.
Override anything with a rules file (`--rules rules.json`, see `examples/rules.example.json` and
`kar/rules.py`).

## Development

```sh
python3 tests/test_core.py        # core tests, no KiCad needed
python3 tests/test_kicad.py       # builds and routes the demo board with KiCad (skipped without KiCad)
$KPY -m kar bench path/to/boards  # regression run on a folder of your own boards
```

## License

MIT
