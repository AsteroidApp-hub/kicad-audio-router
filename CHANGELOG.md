# Changelog

## 0.1.0 (unreleased)

First public version.

### Routing
- A* grid router over the routing layers (power-plane layers left to planes), 8 directions plus vias,
  grid aligned to fine-pitch pads.
- Exact clearance checks against real pad outlines, tracks and vias, per net class, including hole
  clearance and keep-out (rule) areas.
- Audio cost fields: analog keeps away from digital copper (weighted by clock / data / static activity),
  digital keeps away from analog; the second side of a balanced pair follows the first.
- Ground fan-out first: every SMD ground pad gets its own via before signals are routed.
- Push-and-reroute for nets that do not fit; pins off the coarse grid routed first on a finer grid;
  finer-grid fallback for what is left.
- Power and ground tracks narrowed only where needed, widened again afterwards; power nets still narrow
  are routed again at full width by pushing other routes aside (kept only if every pushed route fits).
- Net classes from the project file: track width and via size per class.
- Re-routing after hand edits: `--rip-kar` removes only what kar placed (PCB groups `kar: ...`) and
  completes nets that are partly routed.

### Checks and output
- Net classification (analog / digital / power / ground / other, balanced pairs) with reasons;
  house-style designators recognised; rules file to override.
- Routing quality report with a 0–100 score and board coordinates of the places to review.
- Route log: per-net table and every step that needed more than a plain route.
- kar's copper grouped in KiCad by kind (`kar: analog`, `kar: power`, …).
- `kar plot` (SVG), `kar bench` (re-route a folder of boards and compare), `kar doctor` (KiCad checks).

### Tested
- KiCad 10.0 on macOS: seven real audio boards and the demo board, all fully routed, no DRC problems
  introduced by kar. KiCad 9.0 and 10.0 in CI (Docker images) once published.
