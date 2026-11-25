# Compatibility

| KiCad | status |
|---|---|
| 10.0 (macOS, Python 3.9) | tested: all checks pass, seven real boards and the demo route with clean DRC |
| 9.0 (Linux, Python 3.11) | tested in the official `kicad/kicad:9.0` Docker image (9.0.9): `kar doctor --demo` and the end-to-end tests pass, same result as on KiCad 10 |
| 8.0 | not tested |

Boards saved by a newer KiCad cannot be opened by an older one (KiCad's own rule): a board saved with
KiCad 10 does not load in KiCad 9. kar keeps the file format of the KiCad it runs in.

kar talks to KiCad only through the `pcbnew` Python module and `kicad-cli` (for DRC). Every call that
changed between versions has a fallback (via width per layer, `GetEffectivePolygon` arguments,
`GetUnconnectedCount` arguments, zone outline inflation).

## Checking your installation

```sh
$KPY -m kar doctor          # API checks only
$KPY -m kar doctor --demo   # also builds the demo board, routes it and runs KiCad's DRC
```

Every line should start with `ok`. If a line says `NG`, please open an issue with the full output
(`--json`), your KiCad version and your operating system.

## Continuous integration

`.github/workflows/tests.yml` runs the core tests on Python 3.9–3.12 and the whole flow (`kar doctor --demo`,
`tests/test_kicad.py`) inside the official `kicad/kicad:9.0` and `kicad/kicad:10.0` Docker images.
The image tags follow KiCad's Docker Hub naming; adjust them if a tag is renamed.
