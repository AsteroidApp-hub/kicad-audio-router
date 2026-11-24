# kicad-audio-router

Audio-aware routing helpers for KiCad. **Status: early development (not released).**

General-purpose autorouters (e.g. Freerouting) connect everything but know nothing about audio:
they run analog signals next to clocks, split balanced pairs, and leave ground pads far from a via.
kicad-audio-router reads your board, works out which nets are analog / digital / power / ground,
and measures (and, in later versions, improves) the routing with audio in mind.

- Pure Python on top of KiCad's own `pcbnew` module (KiCad 7–10). No Java, no extra packages.
- Deterministic: the same board and rules give the same result.
- Machine-readable output (JSON) so it fits scripted / AI-driven design flows.

## Status

| feature | state |
|---|---|
| net classification (analog / digital / power / gnd, differential pairs) | ✅ v0.1 |
| routing quality report (Markdown + JSON, 0–100 score) | ✅ v0.1 |
| audio pre-router (analog nets, pairs side by side, away from digital) | planned |
| ground handling (pours, fan-out vias, island repair) | ✅ v0.1 |
| hand-off of the remaining nets to Freerouting (DSN/SES) | planned |

## Usage

Run with the Python that ships with KiCad (it contains `pcbnew`):

```sh
# macOS
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
# Windows:  "C:\Program Files\KiCad\9.0\bin\python.exe"     Linux: python3 (with KiCad installed)

$KPY -m kar classify board.kicad_pcb                 # how each net was classified, and why
$KPY -m kar report   board.kicad_pcb --out out/      # writes board_quality.md / .json
$KPY -m kar report   board.kicad_pcb --json          # score only, for scripts
$KPY -m kar ground   board.kicad_pcb --net GND,/CHASSIS -o out.kicad_pcb
                                                     # vias for ground pads far from one, island repair
$KPY -m kar ground   board.kicad_pcb --pour F.Cu,B.Cu   # also pour GND on the outer layers first
```

`ground` works on a routed board. For every SMD pad of the ground net(s) with no via / through-hole pad
of the same net within `--max-dist` (3 mm, measured from the pad edge) it adds a short track and a via,
searching outwards from the pad (away from the part body first) and keeping clearance to all other copper
on every layer. Pour islands that reach no via / through-hole pad get one via at their deepest point.
Always run KiCad's DRC afterwards.

### What the report measures

| metric | meaning |
|---|---|
| coupling | analog copper running within 1 mm / 3 mm (edge to edge, same layer) of digital copper, weighted by how active the digital net is (clock 1.0, data 0.6, static 0.15) |
| differential pairs | length of each side, mismatch, share routed side by side |
| ground return | SMD pads of each ground net farther than 3 mm (from the pad edge) from a via / through-hole pad of the same net |
| power width | power nets with tracks narrower than the limit |
| analog vias | vias on analog nets |
| unrouted | unconnected items reported by KiCad |

The score (0–100) weights these for audio work so that two routing runs of the same board can be compared.
It is a comparison aid, not an absolute grade.

### How nets are classified

1. explicit globs in the rules file
2. ground / power by name (`GND`, `AGND`, `+15V`, `VCC`, …)
3. digital by name (`SCLK`, `SDA`, `LATCH`, …), by touching a digital IC (`74*`, `ULN*`, MCU, …),
   or a digital pin of a mixed-signal IC (e.g. the `CLOCK` pin of an electronic volume)
4. analog: pins of analog ICs (`OPA*`, `NE5532`, `MUSES*`, …) and audio connectors, then spread through
   resistors, capacitors and relay contacts
5. pairs: analog nets whose names differ only by a suffix pair (`_H`/`_C`, `_P`/`_N`, `+`/`-`)

Override anything with a rules file (`--rules rules.json`, see `examples/rules.example.json`
and the docstring in `kar/rules.py`).

## Development

```sh
python3 tests/test_core.py        # core tests, no KiCad needed
```

## License

MIT
