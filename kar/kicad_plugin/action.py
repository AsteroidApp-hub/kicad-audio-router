"""Toolbar button in the KiCad PCB editor: saves the board, runs `kar auto` on it in a separate process
and writes the result next to it as <board>_kar.kicad_pcb (the open board is not changed)."""
import json
import os
import subprocess
import sys
import threading

import pcbnew
import wx

HERE = os.path.dirname(os.path.abspath(__file__))


def _settings():
    try:
        with open(os.path.join(HERE, "kar_location.json"), encoding="utf-8") as f:
            return json.load(f)
    except Exception:   # noqa: BLE001
        return {}


def _python():
    """The Python that comes with KiCad (inside KiCad, sys.executable is KiCad itself)."""
    s = _settings()
    cands = [s.get("python"), os.path.join(sys.prefix, "bin", "python3"), os.path.join(sys.prefix, "python.exe"),
             os.path.join(os.path.dirname(sys.executable), "python.exe"), os.path.join(sys.prefix, "bin", "python")]
    for c in cands:
        if c and os.path.exists(c):
            return c
    return None


class KarAction(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "kicad-audio-router"
        self.category = "Routing"
        self.description = "Route the board for audio: analog away from digital, pairs side by side (kar auto)"
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(HERE, "icon.png")

    def Run(self):
        board = pcbnew.GetBoard()
        path = board.GetFileName()
        parent = wx.GetActiveWindow()
        if not path:
            wx.MessageBox("Save the board first.", "kicad-audio-router", wx.OK | wx.ICON_INFORMATION, parent)
            return
        choice = wx.SingleChoiceDialog(
            parent, "kar saves this board, routes a copy and writes it next to it as <board>_kar.kicad_pcb.\n"
                    "The board open here is not changed.",
            "kicad-audio-router",
            ["Route (keep tracks that are already there)",
             "Re-route: remove what kar placed before, keep hand-routed tracks"])
        if choice.ShowModal() != wx.ID_OK:
            return
        rip = choice.GetSelection() == 1
        py = _python()
        if not py:
            wx.MessageBox("KiCad's Python was not found. Run 'kar plugin install' again.", "kicad-audio-router",
                          wx.OK | wx.ICON_ERROR, parent)
            return
        pcbnew.SaveBoard(path, board)
        stem = os.path.splitext(path)[0]
        out = stem + "_kar.kicad_pcb"
        cmd = [py, "-m", "kar", "auto", path, "-o", out, "--out", os.path.join(os.path.dirname(path), "kar_out")]
        if rip:
            cmd.append("--rip-kar")
        env = dict(os.environ)
        loc = _settings().get("pythonpath")
        if loc:
            env["PYTHONPATH"] = loc + os.pathsep + env.get("PYTHONPATH", "")
        result = {}

        def work():
            r = subprocess.run(cmd, capture_output=True, text=True, env=env)
            result["out"], result["err"], result["rc"] = r.stdout, r.stderr, r.returncode
        th = threading.Thread(target=work, daemon=True)
        th.start()
        busy = wx.ProgressDialog("kicad-audio-router", "Routing ... (a large board takes a few minutes)",
                                 parent=parent, style=wx.PD_APP_MODAL | wx.PD_AUTO_HIDE)
        while th.is_alive():
            busy.Pulse()
            wx.MilliSleep(200)
            wx.Yield()
        busy.Destroy()
        t = result.get("out", "")
        i = t.rfind('{\n "saved"')
        if i < 0:
            wx.MessageBox("kar did not finish:\n\n" + (result.get("err", "") or t)[-1500:], "kicad-audio-router",
                          wx.OK | wx.ICON_ERROR, parent)
            return
        s = json.loads(t[i:])
        d = s.get("drc") or {}
        msg = (f"Written: {out}\n\n"
               f"Score: {s.get('score')} / 100\n"
               f"Unconnected: {d.get('unconnected', '?')}\n"
               f"DRC problems: {d.get('problems') or 'none'}\n"
               f"Time: {s.get('seconds')} s\n\n"
               f"Report and route log: {s.get('outputs')}\n\n"
               "Open the written board (File > Open) to look at it and run the DRC.")
        wx.MessageBox(msg, "kicad-audio-router", wx.OK | wx.ICON_INFORMATION, parent)
