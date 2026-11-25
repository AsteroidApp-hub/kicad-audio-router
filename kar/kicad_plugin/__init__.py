"""KiCad action plugin for kicad-audio-router. Installed by `kar plugin install`."""
from .action import KarAction

try:
    KarAction().register()          # works inside KiCad's PCB editor only
except Exception:   # noqa: BLE001
    pass
