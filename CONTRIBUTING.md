# Contributing

Thank you for helping. Bug reports with a board that shows the problem are the most useful thing.

## Reporting a problem

Please include:

- the output of `python -m kar doctor --json` (run with KiCad's Python),
- the command you ran and its JSON output,
- if you can share it, the board (`.kicad_pcb` and `.kicad_pro`), or a cut-down board that shows the
  problem. Do not attach boards you are not allowed to share.

## Working on the code

```sh
python3 tests/test_core.py        # pure-Python tests (no KiCad)
$KPY tests/test_kicad.py          # builds and routes the demo board, re-route after hand edits
$KPY -m kar bench path/to/boards  # re-route a folder of boards and compare with their routing
```

- Only `kar/kicad_io.py`, `kar/doctor.py` and `kar/demo.py` import `pcbnew`; keep the rest pure Python so
  it can be tested without KiCad.
- Results must stay deterministic: the same board and rules give the same output.
- A change to the router should come with a `kar bench` run on a few boards: no new unconnected items, no
  new DRC problems, and the score not lower.
- Code, comments and documentation are in English; `README.ja.md` is the Japanese README.
