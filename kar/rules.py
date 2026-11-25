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
    # word pairs anywhere in the name, e.g. HOTL / COLDL, IN_POS_L / IN_NEG_L
    "diff_pairs_word": [["HOT", "COLD"], ["POS", "NEG"]],
    # name patterns (regex, case-insensitive) used when nothing else decides
    "gnd_names": [r"^/?(A|D|P|S)?GND\d*$", r"^/?GND_", r"^/?GND(REF|PWR|A|D|S)$", r"^/?[AD]GND_", r"^/?CHASSIS$",
                  r"^/?EARTH$", r"^/?VSS$", r"^/?0V$"],
    "power_names": [r"^/?[+-]\d+(\.\d+)?V", r"^/?[+-]?V(CC|DD|EE|IN|BAT|BUS)", r"^/?[+-]?\d+V\d*",
                    r"RAW$", r"^/?V\d+", r"_?V(CC|DD|EE)$"],
    "digital_names": [r"CLK|SCK|SCL|SDA|MOSI|MISO|SDATA|SDI|SDO|DATA$|LAT|LATCH|_CS$|^/?CS|_EN(_|$)|^/?EN|"
                      r"(^|_)(BCK|LRCK|WCK|MCK|DATAIN|DATAOUT|DIN|DOUT|SDIN|SDOUT|I2S|SPDIF|DSD)|"
                      r"OE(_|$)|PGOOD|RESET|RST|^/?(UART_)?(TX|RX)D?\d*$|PWM|INT$|IRQ|SPI|I2C|UART|GPIO|LED"],
    "analog_parts": [r"^OPA\d", r"^NE55", r"^NJM(?!7[89])", r"^LM4562", r"^LME", r"^TL0[78]", r"^AD8\d",
                     r"^AD7[0-9]{2}(?!\d)", r"^THAT", r"^INA\d", r"^MUSES", r"^PGA2", r"^BUF6", r"^LT1\d",
                     r"^TPA6", r"^RC4580", r"^MCP60", r"^LM358", r"^LM324", r"^LM833", r"^JRC", r"^OP\d",
                     r"^SSM2", r"^DRV13", r"^LM386", r"^TDA",
                     # audio converters: their digital pins are caught by name first, the rest is analog
                     r"^PCM1[0-9]", r"^PCM5", r"^PCM18", r"^ES9", r"^AK4[0-9]", r"^AK5", r"^CS4[0-9]", r"^CS5[0-9]",
                     r"^WM8[0-9]", r"^AD19", r"^TPA3", r"^PAM8"],
    "digital_parts": [r"^74", r"^CD40", r"^ULN", r"^TPIC", r"^RP2040", r"^Pico", r"^ATmega", r"^ATtiny",
                      r"^STM32", r"^ESP32", r"^PIC", r"^SN74", r"^MCP23", r"^PCA9"],
    # footprint-name regex for audio connectors (seeds for analog)
    "audio_connectors": [r"Neutrik", r"XLR", r"Jack", r"TRS", r"RCA", r"Phone", r"Audio", r"Combo",
                         r"NC3", r"NJ", r"NMJ", r"NRJ", r"MIC", r"LINE", r"INPUT", r"OUTPUT", r"HEADPHONE",
                         r"^(?!.*\bDC\b)(?!.*USB).*\b(IN|OUT)\b",
                         # eurorack / small jacks, and values like "In1", "Out 2"
                         r"PJ-?3[0-9]{2}", r"PJ398", r"Thonk", r"WQP", r"3\.5\s*mm", r"6\.35\s*mm",
                         r"(^|\s)(IN|OUT)\s*\d+$"],
    # parts whose in/out pins carry supply: regulators, DC/DC modules
    "power_parts": [r"^(L|LM|MC|UA|NJM|KA)?78[0-9LMS]", r"^(L|LM|MC|UA|NJM|KA)?79[0-9LMS]", r"^LM31[17]", r"^LM33[37]",
                    r"^TL783", r"^AMS1117", r"^LD1117", r"^LM1117", r"^NMA", r"^R-78", r"^TPS7", r"^LT30", r"^LP29",
                    r"^MIC5", r"DC-?DC", r"^TSR"],
    # connector values that mark a supply input
    "power_connectors": [r"^[+-]?\d+(\.\d+)?\s*V(AC|DC)?\b", r"^[+-]\d+$", r"\bUSB\b", r"\bPWR\b", r"POWER"],
    # pin-function regex on mixed-signal ICs: these pins are digital even on an analog IC
    "digital_pins": [r"CLK|SCK|SCL|SDA|DATA|SDI|SDO|LATCH|LAT|CS|^EN|RESET|RST|MUTE|ZC|DGND|DVDD|ADR|ADDR"],
    # activity of digital nets: how harmful it is to run next to analog copper.
    # checked in order clock -> data -> static; digital nets that match nothing are "data".
    "activity_names": {
        "clock": [r"CLK|SCK|SCL$|SCL_|BCLK|MCLK|LRCK|LRCLK|WCLK|(^|_)(BCK|WCK|MCK)|(^|_)WS$|XTAL|OSC"],
        "data": [r"DATA|SDA|MOSI|MISO|SDI|SDO|DIN|DOUT|(^|_)(TX|RX)D?\d*$|USB|D[+-]$"],
        "static": [r"LAT|(^|_)EN(_|$)|OE(_|$)|PGOOD|RESET|RST|LED|MUTE|SEL|DRV|KD\d|COIL|RLY|INT$|IRQ"],
    },
    # parts whose outputs switch rarely (relay drivers, shift registers that set relays)
    "static_parts": [r"^ULN", r"^TPIC", r"^MIC2981", r"^TBD62"],
    "activity_weight": {"clock": 1.0, "data": 0.6, "static": 0.15},
    "metrics": {"coupling_gap_mm": [1.0, 3.0], "gnd_via_max_mm": 3.0, "power_min_width_mm": 0.4,
                "power_full_width_mm": 0.5},
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
