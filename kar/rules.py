"""Rules: how nets are classified and what each class needs.

A rules file is JSON. Every key is optional; missing keys fall back to DEFAULTS.

{
  "classes": {                     # force classes by net-name glob (with or without leading "/")
    "analog":  ["/L_*", "/R_*"],
    "digital": ["/KD*"],
    "power":   [], "gnd": [], "ignore": []
  },
  "diff_pairs": [["_H", "_C"], ["+", "-"]],   # suffix pairs that make a differential pair
  "analog_parts":  ["^OPA", "^NE55", "^MUSES"],  # regex on footprint value (U*) -> analog IC
  "digital_parts": ["^74", "^ULN", "^RP2040"],   # regex on footprint value  -> digital IC
  "metrics": { "coupling_gap_mm": [1.0, 3.0], "gnd_via_max_mm": 3.0, "power_min_width_mm": 0.4 }
}
"""
from __future__ import annotations

import copy
import fnmatch
import json

DEFAULTS: dict = {
    "classes": {"analog": [], "digital": [], "power": [], "gnd": [], "ignore": []},
    "diff_pairs": [["_H", "_C"], ["_HOT", "_COLD"], ["_P", "_N"], ["+", "-"]],
    # name patterns (regex, case-insensitive) used when nothing else decides
    "gnd_names": [r"^/?(A|D|P|S)?GND\d*$", r"^/?GND_", r"^/?CHASSIS$", r"^/?EARTH$", r"^/?VSS$", r"^/?0V$"],
    "power_names": [r"^/?[+-]\d+(\.\d+)?V", r"^/?[+-]?V(CC|DD|EE|IN|BAT|BUS)", r"^/?[+-]?\d+V\d*",
                    r"RAW$", r"^/?V\d+", r"_?V(CC|DD|EE)$"],
    "digital_names": [r"CLK|SCK|SCL|SDA|MOSI|MISO|SDATA|SDI|SDO|DATA$|LAT|LATCH|_CS$|^/?CS|_EN(_|$)|^/?EN|"
                      r"OE(_|$)|PGOOD|RESET|RST|^/?(UART_)?(TX|RX)D?\d*$|PWM|INT$|IRQ|SPI|I2C|UART|GPIO|LED"],
    "analog_parts": [r"^OPA\d", r"^NE55", r"^NJM", r"^LM4562", r"^LME", r"^TL07", r"^AD8\d", r"^THAT",
                     r"^INA\d", r"^MUSES", r"^PGA2", r"^BUF6", r"^LT1\d", r"^TPA6", r"^RC4580"],
    "digital_parts": [r"^74", r"^CD40", r"^ULN", r"^TPIC", r"^RP2040", r"^Pico", r"^ATmega", r"^ATtiny",
                      r"^STM32", r"^ESP32", r"^PIC", r"^SN74", r"^MCP23", r"^PCA9"],
    # footprint-name regex for audio connectors (seeds for analog)
    "audio_connectors": [r"Neutrik", r"XLR", r"Jack", r"TRS", r"RCA", r"Phone", r"Audio", r"Combo",
                         r"NC3", r"NJ", r"NMJ", r"NRJ"],
    # pin-function regex on mixed-signal ICs: these pins are digital even on an analog IC
    "digital_pins": [r"CLK|SCK|SCL|SDA|DATA|SDI|SDO|LATCH|LAT|CS|^EN|RESET|RST|MUTE|ZC|DGND|DVDD|ADR|ADDR"],
    # activity of digital nets: how harmful it is to run next to analog copper.
    # checked in order clock -> data -> static; digital nets that match nothing are "data".
    "activity_names": {
        "clock": [r"CLK|SCK|SCL$|SCL_|BCLK|MCLK|LRCK|LRCLK|WCLK|(^|_)WS$|XTAL|OSC"],
        "data": [r"DATA|SDA|MOSI|MISO|SDI|SDO|DIN|DOUT|(^|_)(TX|RX)D?\d*$|USB|D[+-]$"],
        "static": [r"LAT|(^|_)EN(_|$)|OE(_|$)|PGOOD|RESET|RST|LED|MUTE|SEL|DRV|KD\d|COIL|RLY|INT$|IRQ"],
    },
    # parts whose outputs switch rarely (relay drivers, shift registers that set relays)
    "static_parts": [r"^ULN", r"^TPIC", r"^MIC2981", r"^TBD62"],
    "activity_weight": {"clock": 1.0, "data": 0.6, "static": 0.15},
    "metrics": {"coupling_gap_mm": [1.0, 3.0], "gnd_via_max_mm": 3.0, "power_min_width_mm": 0.4},
}

KINDS = ("gnd", "power", "digital", "analog", "other", "ignore")


def load(path: str | None) -> dict:
    rules = copy.deepcopy(DEFAULTS)
    if not path:
        return rules
    with open(path, encoding="utf-8") as f:
        user = json.load(f)
    for k, v in user.items():
        if isinstance(v, dict) and isinstance(rules.get(k), dict):
            rules[k].update(v)
        else:
            rules[k] = v
    return rules


def glob_match(net: str, patterns: list[str]) -> bool:
    bare = net.lstrip("/")
    return any(fnmatch.fnmatchcase(net, p) or fnmatch.fnmatchcase(bare, p.lstrip("/")) for p in patterns)
