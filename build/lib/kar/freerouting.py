"""Hand the nets kar did not route to Freerouting (Specctra DSN/SES), keeping kar's tracks.

Freerouting is run as an external program (`java -jar freerouting.jar`); none of its code is used here.
Tested with Freerouting 2.1 (Java 21). Locked tracks are exported as protected wiring, so Freerouting
routes around them.
"""
from __future__ import annotations

import os
import re
import subprocess



def _pcbnew():
    from .kicad_io import pcbnew
    return pcbnew


def _find_block(text: str, start: int) -> int:
    """Index just after the s-expression starting at text[start] == '('."""
    depth = 0
    i = start
    in_str = False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    raise ValueError("unbalanced DSN")


def drop_nets(dsn_text: str, nets: set[str]) -> tuple[str, list[str]]:
    """Remove `(net NAME ...)` blocks of the network section for the given nets (and their
    mentions in `(class ...)` lists), so Freerouting leaves them alone."""
    dropped = []
    out = []
    pos = 0
    for m in re.finditer(r'\(net\s+("([^"]*)"|[^\s()]+)', dsn_text):
        name = m.group(2) if m.group(2) is not None else m.group(1)
        if name not in nets or m.start() < pos:
            continue
        end = _find_block(dsn_text, m.start())
        out.append(dsn_text[pos:m.start()])
        pos = end
        dropped.append(name)
    out.append(dsn_text[pos:])
    text = "".join(out)
    for n in dropped:
        q = re.escape(n)
        text = re.sub(r'(\(class\s+[^\s()]+[^()]*?)\s("%s"|%s)(?=[\s)])' % (q, q), r"\1", text)
    return text, dropped


def export_dsn(kb, path: str, exclude: set[str]) -> list[str]:
    if not _pcbnew().ExportSpecctraDSN(kb, path):
        raise RuntimeError("DSN export failed")
    with open(path, encoding="utf-8") as f:
        text = f.read()
    text, dropped = drop_nets(text, exclude)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return dropped


def run(dsn: str, ses: str, jar: str, java: str = "java", passes: int = 30, timeout: int = 3600,
        log: str | None = None) -> int:
    home = os.path.join(os.path.dirname(os.path.abspath(dsn)), ".frhome")   # never read the user's settings
    os.makedirs(home, exist_ok=True)
    p = str(passes)
    cmd = [java, "-Djava.awt.headless=true", "-Duser.home=" + home, "-jar", jar, "-de", dsn, "-do", ses,
           "-mp", p, "--gui.enabled=false", "--router.max_passes=" + p, "--router.stop_pass_no=" + p,
           "--router.optimizer.enabled=false"]
    env = dict(os.environ, FREEROUTING__ROUTER__MAX_PASSES=p, FREEROUTING__ROUTER__STOP_PASS_NO=p,
               FREEROUTING__GUI__ENABLED="false")
    with open(log or os.devnull, "w") as lf:
        r = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT, env=env, timeout=timeout)
    if not os.path.exists(ses):
        raise RuntimeError(f"Freerouting produced no SES (exit {r.returncode}); see {log}")
    return r.returncode


def import_ses(kb, ses: str) -> None:
    if not _pcbnew().ImportSpecctraSES(kb, ses):
        raise RuntimeError("SES import failed")


def cleanup_dangling(kb, rounds: int = 20) -> int:
    """Repeatedly delete tracks with a dangling end (left over by the autorouter)."""
    removed = 0
    for _ in range(rounds):
        kb.BuildConnectivity()
        conn = kb.GetConnectivity()
        dead = []
        for t in kb.GetTracks():
            if t.GetClass() != "PCB_TRACK" or t.IsLocked():
                continue
            try:
                d = conn.TestTrackEndpointDangling(t, False)
            except TypeError:
                d = conn.TestTrackEndpointDangling(t)
            if d:
                dead.append(t)
        if not dead:
            break
        for t in dead:
            kb.Delete(t)
        removed += len(dead)
    return removed
