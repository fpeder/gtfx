#!/usr/bin/env python3
"""
efxctrl.py — ncurses controller for GTFX FPGA guitar effects processor.

Architecture
────────────
  Routing   Declarative route table mirroring the FPGA's shadow_route[0..8].
  Model     Param / Param16 / EffectSlot — pure data, no I/O.
  Serial    SerialLink wraps pyserial; every public method maps 1-to-1
            to a cmd_proc_v2 CLI command.
  UI        Renderer (stateless draw helpers) + InputHandler (modal state
            machine: NORMAL → RAW_WRITE | CONNECTION).

Adding a new effect
───────────────────
  1. Add a PORT_xxx constant and extend PORT_TAG / EFFECT_DEFS.
  2. Add a new EffectSlot entry in EFFECT_DEFS.
  3. Done — routing, display order, and serial commands derive automatically.

Usage
─────
    python3 efx_control.py [--port /dev/ttyUSB0] [--baud 115200] [--offline]

Keys (normal mode)
──────────────────
    TAB / Shift-TAB    Cycle effect slots (chain order)
    h / j / k / l      Vim-style navigation (left/down/up/right column)
    UP / DOWN          Navigate parameters (wraps across slots)
    LEFT / RIGHT       Coarse adjust ±16  (±0x1000 for 16-bit)
    Shift-LEFT/RIGHT   Fine adjust  ±1  (±0x100 for 16-bit params)
    PgUp / PgDn        Coarse adjust ±16  (±0x1000 for 16-bit)
    SPACE / ENTER      Toggle bypass on/off
    c                  Enter connection editor
    s                  Request hardware status dump
    w                  Enter raw register write mode
    r                  Refresh display
    q / ESC            Quit
"""

from __future__ import annotations

import argparse
import curses
import os
import sys
import time
import threading
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    try:
        import tomli as tomllib  # type: ignore[no-redef]
    except ModuleNotFoundError:
        tomllib = None  # type: ignore[assignment]
from dataclasses import dataclass, field
from enum import IntEnum, auto
from typing import Optional

try:
    import serial as pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ╔════════════════════════════════════════════════════════════════════════════════╗
# ║  Routing model — mirrors cmd_proc shadow_route[0..8]                         ║
# ║                                                                              ║
# ║  Port indices: 0=ADC 1=TRM 2=PHA 3=CHO 4=DLY 5=FLN 6=REV 7=CMP 8=WAH      ║
# ║  route[sink] = source_port   (who feeds into *sink*)                         ║
# ║  DAC = sink index 0 ;  ADC = ultimate source (port 0)                       ║
# ╚════════════════════════════════════════════════════════════════════════════════╝


class Port(IntEnum):
    """Hardware port indices (must match cmd_proc)."""
    ADC = 0
    TRM = 1
    PHA = 2
    CHO = 3
    DLY = 4
    FLN = 5
    REV = 6
    CMP = 7
    WAH = 8


PORT_TAG: dict[int, str] = {p.value: p.name.lower() for p in Port}
TAG_PORT: dict[str, int] = {v: k for k, v in PORT_TAG.items()}
TAG_PORT["dac"] = 99  # virtual display-only node (not a real port)

# Nodes available as source / sink in the connection editor
SOURCE_NODES = [p.name.lower() for p in Port]  # adc … fln
SINK_NODES = ["dac"] + [p.name.lower() for p in Port if p != Port.ADC]
SINK_ROUTE_IDX = {tag: i for i, tag in enumerate(SINK_NODES)}

# Default route: ADC→WAH→CMP→PHA→FLN→CHO→TRM→DLY→REV→DAC
DEFAULT_ROUTE: list[int] = [
    Port.REV,  # route[0]   DAC ← REV
    Port.CHO,  # route[1]   TRM ← CHO
    Port.CMP,  # route[2]   PHA ← CMP
    Port.FLN,  # route[3]   CHO ← FLN
    Port.TRM,  # route[4]   DLY ← TRM
    Port.PHA,  # route[5]   FLN ← PHA
    Port.DLY,  # route[6]   REV ← DLY
    Port.WAH,  # route[7]   CMP ← WAH
    Port.ADC,  # route[8]   WAH ← ADC
]

ROUTE_ADDR_BASE = 0x60  # ctrl_bus base address for route registers


def trace_chain(route: list[int]) -> list[int]:
    """Walk *route* from DAC back to ADC.

    Returns port indices in signal-flow order (ADC-side first).
    A visited set prevents infinite loops from bad route tables.
    """
    chain: list[int] = []
    visited: set[int] = set()
    port = route[0]
    while port != Port.ADC and port not in visited:
        visited.add(port)
        chain.append(port)
        port = route[port] if port < len(route) else Port.ADC
    chain.reverse()
    return chain


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Data model                                                             ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


@dataclass
class Param:
    """Single 8-bit register parameter."""
    name: str  # 3-char CLI mnemonic  (e.g. "rat")
    label: str  # Human-readable label (e.g. "Rate")
    addr: int  # ctrl_bus address
    default: int
    lo: int = 0x00
    hi: int = 0xFF
    value: int = -1  # set to *default* in __post_init__
    fmt: str = "hex"

    def __post_init__(self):
        if self.value == -1:
            self.value = self.default

    @property
    def step_fine(self) -> int:
        return 1

    @property
    def step_coarse(self) -> int:
        return 0x10

    def format_value(self) -> str:
        return f"  {self.value:02X}"

    def format_cmd_value(self) -> str:
        return f"{self.value:02X}"


@dataclass
class Param16(Param):
    """16-bit little-endian parameter (e.g. delay time)."""
    lo: int = 0x0000
    hi: int = 0xFFFF
    fmt: str = "hex16"

    @property
    def step_fine(self) -> int:
        return 0x100

    @property
    def step_coarse(self) -> int:
        return 0x1000

    def format_value(self) -> str:
        return f"{self.value:04X}"

    def format_cmd_value(self) -> str:
        return f"{self.value:04X}"


@dataclass
class ParamPacked(Param):
    """Param that owns a bit-range within a shared register.

    Two ParamPacked siblings compose into one hardware byte.
    """
    bit_lo: int = 0
    bit_hi: int = 7
    sibling: ParamPacked | None = field(default=None, repr=False)

    def __post_init__(self):
        width = self.bit_hi - self.bit_lo + 1
        self.lo = 0
        self.hi = (1 << width) - 1
        if self.value == -1:
            mask = self.hi
            self.value = (self.default >> self.bit_lo) & mask

    @property
    def step_fine(self) -> int:
        return 1

    @property
    def step_coarse(self) -> int:
        return max(1, (self.hi + 1) // 8)

    def _compose(self) -> int:
        """Build the full hardware byte from self + sibling."""
        val = self.value << self.bit_lo
        if self.sibling is not None:
            val |= self.sibling.value << self.sibling.bit_lo
        return val & 0xFF

    def format_value(self) -> str:
        return f"  {self.value:02X}"

    def format_cmd_value(self) -> str:
        return f"{self._compose():02X}"


MODE_LABELS = {0: "DIG", 1: "TAPE", 2: "ANA"}


class ParamPackedMode(ParamPacked):
    """Mode selector with named display values."""

    def __post_init__(self):
        super().__post_init__()
        self.hi = len(MODE_LABELS) - 1  # 0..2

    is_enum = True

    def format_value(self) -> str:
        return f" {MODE_LABELS.get(self.value, '???')}"


@dataclass
class ParamEnum(Param):
    """Mode selector param with named labels instead of a bar."""
    labels: dict[int, str] = field(default_factory=dict)

    is_enum = True

    def __post_init__(self):
        super().__post_init__()
        if self.labels:
            self.hi = len(self.labels) - 1

    @property
    def step_coarse(self) -> int:
        return 1

    def format_value(self) -> str:
        return f" {self.labels.get(self.value, '???')}"


@dataclass
class EffectSlot:
    """One effect pedal: parameters + bypass state."""
    tag: str  # 3-char CLI tag  (e.g. "trm")
    full_name: str  # Display name    (e.g. "Tremolo")
    port: int  # Routing port index (Port enum value)
    params: list[Param]
    bypass_addr: int
    bypassed: bool = True
    disabled: bool = False
    selected_param: int = 0

    @property
    def height(self) -> int:
        """Rows consumed when drawn (header + params + footer)."""
        return 1 + len(self.params) + 1


# ── Effect definitions (extend here to add new pedals) ──────────────────────

EFFECT_DEFS: list[EffectSlot] = [
    EffectSlot("trm",
               "Tremolo",
               Port.TRM, [
                   Param("rat", "Rate", 0x00, 0x3C),
                   Param("dep", "Depth", 0x01, 0xB4),
                   Param("shp", "Shape", 0x02, 0x00),
               ],
               bypass_addr=0x07),
    EffectSlot("pha",
               "Phaser",
               Port.PHA, [
                   Param("spd", "Speed", 0x08, 0x50),
                   Param("fbn", "Feedback En", 0x09, 0x00),
               ],
               bypass_addr=0x0F),
    EffectSlot("cho",
               "Chorus",
               Port.CHO, [
                   Param("rat", "Rate", 0x10, 0x50),
                   Param("dep", "Depth", 0x11, 0x64),
                   Param("efx", "Effect", 0x12, 0x80),
                   Param("eqh", "EQ High", 0x13, 0xC8),
                   Param("eql", "EQ Low", 0x14, 0x80),
               ],
               bypass_addr=0x17),
    EffectSlot("dly",
               "Delay",
               Port.DLY, [
                   Param("rpt", "Repeats", 0x18, 0x4C),
                   Param("mix", "Mix", 0x19, 0x60),
                   Param("flt", "Filter", 0x1A, 0xB0),
                   Param16("tim", "Time", 0x1B, 0x5DC0),
                   Param("mod", "Mod", 0x1D, 0x00),
                   ParamPackedMode("grt", "Mode", 0x1E, 0x00,
                                   bit_lo=0, bit_hi=1),
                   ParamPacked("grt", "Saturation", 0x1E, 0x00,
                               bit_lo=2, bit_hi=7),
               ],
               bypass_addr=0x1F),
    EffectSlot("fln",
               "Flanger",
               Port.FLN, [
                   Param("man", "Manual", 0x20, 0x40),
                   Param("wid", "Width", 0x21, 0x80),
                   Param("spd", "Speed", 0x22, 0x30),
                   Param("reg", "Regen", 0x23, 0x90),
                   Param("mix", "Mix", 0x24, 0x60),
               ],
               bypass_addr=0x27),
    EffectSlot("rev",
               "Reverb",
               Port.REV, [
                   Param("dec", "Decay", 0x28, 0x80),
                   Param("dmp", "Damping", 0x29, 0x60),
                   Param("mix", "Mix", 0x2A, 0x60),
                   Param("pre", "Pre-delay", 0x2B, 0x20),
                   Param("ton", "Tone", 0x2C, 0x80),
                   Param("lvl", "Level", 0x2D, 0x80),
               ],
               bypass_addr=0x2F),
    EffectSlot("cmp",
               "Compressor",
               Port.CMP, [
                   Param("thr", "Threshold", 0x30, 0x60),
                   Param("rat", "Ratio", 0x31, 0x40),
                   Param("atk", "Attack", 0x32, 0x20),
                   Param("rel", "Release", 0x33, 0x40),
                   Param("mak", "Makeup", 0x34, 0x40),
               ],
               bypass_addr=0x37),
    EffectSlot("wah",
               "Wah / Auto-Wah",
               Port.WAH, [
                   Param("frq", "Freq/Sens", 0x38, 0x30),
                   Param("res", "Resonance", 0x39, 0x60),
                   Param("dpt", "Depth", 0x3A, 0xFF),
                   ParamEnum("mod", "Mode", 0x3B, 0x02,
                             labels={0: "MAN", 1: "LFO", 2: "ENV"}),
                   Param("mix", "Mix", 0x3C, 0xFF),
                   Param("dec", "Decay", 0x3D, 0x60),
               ],
               bypass_addr=0x3F),
]

# Link ParamPacked siblings sharing the same address within each slot
for _slot in EFFECT_DEFS:
    _packed = [p for p in _slot.params if isinstance(p, ParamPacked)]
    for i, a in enumerate(_packed):
        for b in _packed[i + 1:]:
            if a.addr == b.addr:
                a.sibling = b
                b.sibling = a


def build_slot_dict() -> dict[str, EffectSlot]:
    """Create a fresh tag→EffectSlot mapping from EFFECT_DEFS."""
    return {s.tag: s for s in EFFECT_DEFS}


def ordered_slots(slot_dict: dict[str, EffectSlot],
                  route: list[int]) -> list[EffectSlot]:
    """Return slots ordered by signal chain (ADC-side first)."""
    chain_ports = trace_chain(route)
    ordered = [
        slot_dict[PORT_TAG[p]] for p in chain_ports
        if PORT_TAG.get(p, "adc") in slot_dict
    ]
    # Append orphaned slots (disconnected from DAC chain)
    seen = {s.tag for s in ordered}
    for tag, slot in slot_dict.items():
        if tag not in seen:
            ordered.append(slot)
    return ordered


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Serial interface                                                       ║
# ║                                                                         ║
# ║  Every public method corresponds 1-to-1 to a cmd_proc_v2 CLI command.  ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

LOG_MAX = 200
LOG_TRIM = 100


class SerialLink:
    """Thread-safe serial transport for cmd_proc_v2."""

    def __init__(self, port: str, baud: int, timeout: float = 0.5):
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self._ser: Optional[
            pyserial.Serial] = None  # type: ignore[name-defined]
        self._lock = threading.Lock()
        self.rx_log: list[str] = []
        self.connected = False

    # ── lifecycle ────────────────────────────────────────────────────────

    def open(self) -> bool:
        if not HAS_SERIAL:
            return False
        try:
            self._ser = pyserial.Serial(  # type: ignore[attr-defined]
                self.port_name,
                self.baud,
                timeout=self.timeout)
            self.connected = True
            time.sleep(0.1)
            self._ser.reset_input_buffer()
            return True
        except Exception as exc:
            self._log(f"Open error: {exc}")
            return False

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.connected = False

    # ── low-level ────────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        self.rx_log.append(msg)
        if len(self.rx_log) > LOG_MAX:
            self.rx_log[:] = self.rx_log[-LOG_TRIM:]

    def _send(self, cmd: str) -> str:
        """Send *cmd* + CR, waiting for echo of each character.

        The FPGA cmd_proc echoes every character through its TX state
        machine before returning to S_IDLE to accept the next one.
        If we blast the whole string at once, characters arriving while
        the FPGA is still echoing get overwritten in rx_latch and lost.
        Sending one byte at a time and waiting for its echo avoids this.
        """
        with self._lock:
            if not self.connected:
                self._log(f"[offline] {cmd}")
                return ""
            assert self._ser is not None

            # Drain any stale RX data
            if self._ser.in_waiting:
                self._ser.read(self._ser.in_waiting)

            full = cmd + "\r"
            for ch in full:
                self._ser.write(ch.encode("ascii"))
                # Wait for echo (the FPGA echoes every char, CR→CRLF)
                time.sleep(0.002)  # 2ms >> 87µs per byte at 115200

            # Read back any remaining response (echo + prompt)
            time.sleep(0.02)
            resp = ""
            try:
                resp = self._ser.read(self._ser.in_waiting
                                      or 1).decode("ascii", errors="replace")
            except Exception:
                pass
            if resp:
                self._log(resp.strip())
            return resp

    # ── cmd_proc_v2 commands ─────────────────────────────────────────────
    #  set <efx> <prm> <hex>
    #  set <efx> on|off
    #  con <src> <snk>
    #  wr <addr><data>
    #  ;                       (status dump)

    def set_param(self, slot: EffectSlot, param: Param) -> str:
        return self._send(
            f"set {slot.tag} {param.name} {param.format_cmd_value()}")

    def set_bypass(self, slot: EffectSlot, active: bool) -> str:
        return self._send(f"set {slot.tag} {'on' if active else 'off'}")

    def connect(self, src_tag: str, snk_tag: str) -> str:
        """Send ``con <src> <snk>``."""
        return self._send(f"con {src_tag} {snk_tag}")

    def raw_write(self, addr: int, data: int) -> str:
        return self._send(f"wr {addr:02X}{data:02X}")

    def chain(self, tags: list[str]) -> str:
        """Send ``chain <tag1> <tag2> ...`` — redefine signal chain."""
        return self._send(f"chain {' '.join(tags)}")

    def request_status(self) -> str:
        """Send ';' and read the full status dump until the '>' prompt.

        The FPGA sends the status line-by-line through its TX state
        machine.  We keep reading until we see the '>' prompt or
        hit a timeout, to ensure we capture all lines including the
        route table and every active effect's parameters.
        """
        with self._lock:
            if not self.connected:
                self._log("[offline] status request")
                return ""
            assert self._ser is not None

            # Drain stale RX
            if self._ser.in_waiting:
                self._ser.read(self._ser.in_waiting)

            self._ser.write(b";")
            resp = ""
            deadline = time.time() + 2.0  # 2s overall timeout
            while time.time() < deadline:
                time.sleep(0.05)
                n = self._ser.in_waiting
                if n:
                    chunk = self._ser.read(n).decode("ascii", errors="replace")
                    resp += chunk
                    # Status dump ends with the prompt '> '
                    if ">" in chunk:
                        break
            for line in resp.splitlines():
                stripped = line.strip()
                if stripped and stripped != ">":
                    self._log(stripped)
            return resp


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Status parser — decode the `;` status dump from cmd_proc_v2            ║
# ║                                                                         ║
# ║  Status format (example):                                               ║
# ║    DAC<DLY TRM<CHO PHA<TUB CHO<FLN DLY<TRM TUB<BMF FLN<PHA BMF<ADC    ║
# ║    BMF sus:80 ton:80 vol:A0                                             ║
# ║    TUB gai:40 bas:80 mid:80 tre:80 lvl:A0                              ║
# ║    PHA spd:50 fbn:00                                                    ║
# ║    FLN man:80 wid:C0 spd:30 reg:A0                                     ║
# ║    CHO rat:50 dep:64 efx:80 eqh:C8 eql:80                              ║
# ║    TRM rat:3C dep:B4 shp:00                                             ║
# ║    DLY rpt:64 mix:80 flt:FF tim:3000 mod:00 grt:00                      ║
# ║                                                                         ║
# ║  The route line is always present.  Effect lines are only printed       ║
# ║  for effects that are NOT bypassed — omitted effects are bypassed.      ║
# ║  Params of bypassed effects keep their last known (or default) values.  ║
# ╚═══════════════════════════════════════════════════════════════════════════╝

# Maps 3-char route name → port index (for parsing the route line)
_ROUTE_NAME_PORT: dict[str, int] = {
    "ADC": Port.ADC,
    "TRM": Port.TRM,
    "PHA": Port.PHA,
    "CHO": Port.CHO,
    "DLY": Port.DLY,
    "FLN": Port.FLN,
    "REV": Port.REV,
    "CMP": Port.CMP,
    "WAH": Port.WAH,
    "DAC": 0,
}


def parse_status(raw: str, slot_dict: dict[str, EffectSlot],
                 route: list[int]) -> bool:
    """Parse a status dump and update *slot_dict* and *route* in-place.

    Returns True if at least the route line was successfully parsed.
    """
    lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
    if not lines:
        return False

    # ── Parse route line ──
    # Format: "DAC<DLY TRM<TUB PHA<TRM CHO<PHA DLY<CHO TUB<FLN FLN<ADC"
    route_parsed = False
    for line in lines:
        if "<" in line and not any(c == ":" for c in line):
            pairs = line.split()
            for pair in pairs:
                if "<" not in pair:
                    continue
                snk_name, src_name = pair.split("<", 1)
                snk_name = snk_name.strip().upper()
                src_name = src_name.strip().upper()
                if snk_name in _ROUTE_NAME_PORT and src_name in _ROUTE_NAME_PORT:
                    snk_idx = SINK_ROUTE_IDX.get(snk_name.lower())
                    if snk_idx is not None and snk_idx < len(route):
                        route[snk_idx] = _ROUTE_NAME_PORT[src_name]
                        route_parsed = True
            break  # route line is always first

    # ── Determine which effects appeared (= active / not bypassed) ──
    seen_tags: set[str] = set()
    disabled_tags: set[str] = set()

    for line in lines:
        if "<" in line and ":" not in line:
            continue  # skip route line

        # "--- TAG" → disabled effect (not in chain)
        if line.startswith("---"):
            tag = line[4:].strip().lower()
            if tag in slot_dict:
                disabled_tags.add(tag)
                seen_tags.add(tag)
            continue

        if ":" not in line:
            continue

        # Line format: "TRM rat:3C dep:B4 shp:00"
        parts = line.split()
        if not parts:
            continue

        tag = parts[0].strip().lower()
        if tag not in slot_dict:
            continue

        seen_tags.add(tag)
        slot = slot_dict[tag]

        # Build param lookup for this slot
        param_by_name: dict[str, Param] = {p.name: p for p in slot.params}

        for token in parts[1:]:
            if ":" not in token:
                continue
            pname, hexval = token.split(":", 1)
            pname = pname.strip().lower()
            hexval = hexval.strip().upper()
            if pname not in param_by_name:
                continue
            try:
                param_by_name[pname].value = int(hexval, 16)
            except ValueError:
                pass

    # ── Update bypass / disabled state ──
    for tag, slot in slot_dict.items():
        if tag in disabled_tags:
            slot.disabled = True
            slot.bypassed = False
        elif tag in seen_tags:
            slot.disabled = False
            slot.bypassed = False
        else:
            slot.disabled = False
            slot.bypassed = True

    return route_parsed


def sync_from_hardware(link: SerialLink, slot_dict: dict[str, EffectSlot],
                       route: list[int]) -> bool:
    """Request status from hardware and sync local state.

    Returns True on successful sync.
    """
    if not link.connected:
        return False
    raw = link.request_status()
    if not raw:
        return False
    ok = parse_status(raw, slot_dict, route)
    if ok:
        link._log("Synced from hardware")
    else:
        link._log("Status parse failed — using defaults")
    return ok


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Renderer — stateless drawing helpers                                   ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


class Color(IntEnum):
    """Curses color-pair IDs."""
    NORMAL = 1
    HEADER = 2
    ACTIVE = 3
    BYPASSED = 4
    SELECTED = 5
    BAR = 6
    TITLE = 7
    LOG = 8
    HELP = 9
    WARN = 10


SLOT_WIDTH = 40
NUM_COLS = 2
COL_GAP = 1  # horizontal gap between columns


ANSI_COLORS = {
    "black": curses.COLOR_BLACK, "red": curses.COLOR_RED,
    "green": curses.COLOR_GREEN, "yellow": curses.COLOR_YELLOW,
    "blue": curses.COLOR_BLUE, "magenta": curses.COLOR_MAGENTA,
    "cyan": curses.COLOR_CYAN, "white": curses.COLOR_WHITE,
    "default": -1,
}


def _hex_to_curses(h: str) -> tuple[int, int, int]:
    h = h.lstrip('#')
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (r * 1000 // 255, g * 1000 // 255, b * 1000 // 255)


def _load_theme(override: str | None = None) -> dict | None:
    if tomllib is None:
        return None
    candidates: list[Path] = []
    if override:
        candidates.append(Path(override))
    else:
        env = os.environ.get("EFXCTRL_THEME")
        if env:
            candidates.append(Path(env))
        candidates.append(Path.home() / ".config" / "efxctrl" / "theme.toml")
        candidates.append(Path(__file__).resolve().parent / "efxctrl.rc")
    for p in candidates:
        if p.is_file():
            try:
                with open(p, "rb") as f:
                    return tomllib.load(f)
            except Exception:
                return None
    return None


def _init_colors(theme: dict | None = None) -> None:
    curses.start_color()
    curses.use_default_colors()

    pair_names = ["normal", "header", "active", "bypassed", "selected",
                  "bar", "title", "log", "help", "warn"]
    pair_ids = {name: getattr(Color, name.upper()) for name in pair_names}

    if curses.can_change_color() and theme and "colors" in theme:
        # Allocate curses color slots from theme
        color_map: dict[str, int] = {"default": -1}
        slot = 17
        for name, hexval in theme["colors"].items():
            r, g, b = _hex_to_curses(hexval)
            curses.init_color(slot, r, g, b)
            color_map[name] = slot
            slot += 1
        pairs = theme.get("pairs", {})
        for name in pair_names:
            if name in pairs:
                fg_name, bg_name = pairs[name]
                fg = color_map.get(fg_name, -1)
                bg = color_map.get(bg_name, -1)
                curses.init_pair(pair_ids[name], fg, bg)

    elif curses.can_change_color() and not theme:
        # Hard-coded Tokyo Night fallback
        curses.init_color(17, 663, 694, 839)    # #a9b1d6 fg
        curses.init_color(18, 478, 635, 969)    # #7aa2f7 blue
        curses.init_color(19, 753, 788, 961)    # #c0caf5 bright fg
        curses.init_color(20, 255, 282, 408)    # #414868 dark blue bg
        curses.init_color(21, 157, 208, 341)    # #283457 selection bg
        curses.init_color(22, 337, 373, 537)    # #565f89 comment gray
        curses.init_color(23, 82,  86,  149)    # #15161e dark bg
        curses.init_color(24, 620, 808, 416)    # #9ece6a green
        curses.init_color(25, 878, 686, 408)    # #e0af68 yellow
        curses.init_color(26, 969, 463, 557)    # #f7768e red/pink

        curses.init_pair(Color.NORMAL,   17, -1)
        curses.init_pair(Color.HEADER,   19, 20)
        curses.init_pair(Color.ACTIVE,   18, -1)
        curses.init_pair(Color.BYPASSED, 22, -1)
        curses.init_pair(Color.SELECTED, 19, 21)
        curses.init_pair(Color.BAR,      18, -1)
        curses.init_pair(Color.TITLE,    23, 18)
        curses.init_pair(Color.LOG,      24, -1)
        curses.init_pair(Color.HELP,     25, -1)
        curses.init_pair(Color.WARN,     26, -1)

    elif not curses.can_change_color() and theme and "fallback" in theme:
        # Use fallback ANSI names from theme
        fallback = theme["fallback"]
        for name in pair_names:
            if name in fallback:
                fg_name, bg_name = fallback[name]
                fg = ANSI_COLORS.get(fg_name, -1)
                bg = ANSI_COLORS.get(bg_name, -1)
                curses.init_pair(pair_ids[name], fg, bg)

    else:
        # Hard-coded ANSI fallback
        curses.init_pair(Color.NORMAL,   curses.COLOR_WHITE, -1)
        curses.init_pair(Color.HEADER,   curses.COLOR_BLACK, curses.COLOR_CYAN)
        curses.init_pair(Color.ACTIVE,   curses.COLOR_BLUE, -1)
        curses.init_pair(Color.BYPASSED, curses.COLOR_BLACK, -1)
        curses.init_pair(Color.SELECTED, curses.COLOR_BLACK, curses.COLOR_BLUE)
        curses.init_pair(Color.BAR,      curses.COLOR_CYAN, -1)
        curses.init_pair(Color.TITLE,    curses.COLOR_BLACK, curses.COLOR_BLUE)
        curses.init_pair(Color.LOG,      curses.COLOR_GREEN, -1)
        curses.init_pair(Color.HELP,     curses.COLOR_YELLOW, -1)
        curses.init_pair(Color.WARN,     curses.COLOR_RED, -1)


def _cp(c: Color, bold: bool = False) -> int:
    attr = curses.color_pair(c)
    if bold:
        attr |= curses.A_BOLD
    return attr


def _safe(win, *args, **kwargs) -> None:
    """Swallow curses.error so writes past the edge don't crash."""
    try:
        win.addstr(*args, **kwargs)
    except curses.error:
        pass


# ── bar ──────────────────────────────────────────────────────────────────


def draw_bar(win, y: int, x: int, value: int, hi: int, width: int,
             cp: Color) -> None:
    filled = max(0, min(int(value / max(hi, 1) * width), width))
    _safe(win, y, x, "█" * filled, _cp(cp, bold=True))
    _safe(win, "░" * (width - filled), _cp(Color.NORMAL))


# ── effect slot ──────────────────────────────────────────────────────────

_LABEL_W = 10
_VAL_W = 4
_BAR_W = SLOT_WIDTH - 2 - 1 - _LABEL_W - 1 - _VAL_W - 1 - 1  # = 16


def draw_slot(win, slot: EffectSlot, y: int, x: int, is_active: bool) -> int:
    """Draw one slot onto *win* at row *y*, column *x*.  Returns next free row."""
    w = SLOT_WIDTH

    # Determine header style
    if slot.disabled:
        hdr = _cp(Color.BYPASSED)
    elif is_active:
        hdr = _cp(Color.HEADER, bold=True)
    elif slot.bypassed:
        hdr = _cp(Color.BYPASSED)
    else:
        hdr = _cp(Color.ACTIVE, bold=True)

    if slot.disabled:
        byp_text = "---"
        byp_attr = _cp(Color.BYPASSED)
    elif not slot.bypassed:
        byp_text = " ON"
        byp_attr = _cp(Color.ACTIVE, bold=True)
    else:
        byp_text = "OFF"
        byp_attr = _cp(Color.WARN)

    # Header row:  ╭ NAME               ON ╮
    name_w = w - 2 - 4  # 4 chars for bypass tag + space
    _safe(win, y, x, "╭" + "─" * (w - 2) + "╮", hdr)
    _safe(win, y, x + 1, f" {slot.full_name.upper():<{name_w}}", hdr)
    _safe(win, y, x + 1 + name_w, f"{byp_text} ", byp_attr)
    y += 1

    # Parameter rows
    for i, p in enumerate(slot.params):
        is_sel = is_active and i == slot.selected_param and not slot.disabled
        if is_sel:
            attr = _cp(Color.SELECTED, bold=True)
            marker = "▸"
        elif slot.disabled or slot.bypassed:
            attr = _cp(Color.BYPASSED)
            marker = " "
        else:
            attr = _cp(Color.NORMAL)
            marker = " "

        body = f"{marker}{p.label:<{_LABEL_W}} {p.format_value()} "
        border_attr = attr if is_sel else hdr

        _safe(win, y, x, "│", border_attr)
        _safe(win, y, x + 1, body, attr)
        bar_x = x + 1 + len(body)
        bar_cp = Color.SELECTED if is_sel else \
                 (Color.BAR if not slot.bypassed else Color.BYPASSED)
        if getattr(p, 'is_enum', False):
            _safe(win, y, bar_x, " " * _BAR_W, attr)
        else:
            draw_bar(win, y, bar_x, p.value, p.hi, _BAR_W, bar_cp)
        _safe(win, y, x + w - 1, "│", border_attr)
        y += 1

    # Footer
    _safe(win, y, x, "╰" + "─" * (w - 2) + "╯", hdr)
    return y + 1


# ── chrome (title, chain, status, help bar) ──────────────────────────────

FIXED_TOP = 3  # rows reserved above the slot viewport
FIXED_BOT = 2  # status line + help bar


def draw_title(scr, max_x: int, port: str, baud: int, connected: bool) -> None:
    title = " GTFX "
    conn = f" {port}@{baud}" if connected else " OFFLINE"
    line = f"{title}{conn:>{max_x - len(title) - 1}}"
    _safe(scr, 0, 0, line[:max_x - 1].ljust(max_x - 1), _cp(Color.TITLE))


def draw_chain(scr, slots: list[EffectSlot]) -> None:
    chain = "ADC"
    for s in slots:
        if not s.disabled and not s.bypassed:
            chain += f"→{s.tag}"
    chain += "→DAC"
    _safe(scr, 1, 1, "Chain: ", _cp(Color.HELP))
    _safe(scr, chain, _cp(Color.ACTIVE, bold=True))


def draw_connection_editor(scr, con_sink: int, con_src: int,
                           route: list[int]) -> None:
    # Row 1 — sink selector
    _safe(scr, 1, 1, "ROUTE EDIT ", _cp(Color.WARN, bold=True))
    cx = 13
    for si, snk in enumerate(SINK_NODES):
        attr = _cp(Color.SELECTED, bold=True) if si == con_sink \
               else _cp(Color.NORMAL)
        _safe(scr, 1, cx, f" {snk.upper()} ", attr)
        cx += 5

    # Row 2 — source selector for the chosen sink
    snk_tag = SINK_NODES[con_sink]
    ridx = SINK_ROUTE_IDX[snk_tag]
    cur_src = PORT_TAG.get(route[ridx], "?") if ridx < len(route) else "?"
    _safe(scr, 2, 1, f"{snk_tag.upper()} src: ", _cp(Color.HELP))
    sx = 11
    for ni, node in enumerate(SOURCE_NODES):
        if ni == con_src:
            attr = _cp(Color.SELECTED, bold=True)
        elif node == cur_src:
            attr = _cp(Color.ACTIVE, bold=True)
        else:
            attr = _cp(Color.NORMAL)
        _safe(scr, 2, sx, f" {node.upper()} ", attr)
        sx += 5


def draw_chain_editor(scr, st: "AppState") -> None:
    """Draw the chain editor on rows 1-2."""
    _safe(scr, 1, 1, "CHAIN EDIT ", _cp(Color.WARN, bold=True))
    cx = 13
    all_tags = [s.tag for s in EFFECT_DEFS]
    # Show chain effects in order, then disabled ones
    active = [t for t in st.chain_order if t not in st.chain_disabled_set]
    disabled = [t for t in all_tags if t in st.chain_disabled_set]
    display = active + disabled
    for i, tag in enumerate(display):
        is_disabled = tag in st.chain_disabled_set
        is_cursor = (i == st.chain_cursor)
        if is_cursor:
            attr = _cp(Color.SELECTED, bold=True)
        elif is_disabled:
            attr = _cp(Color.BYPASSED)
        else:
            attr = _cp(Color.ACTIVE, bold=True)
        _safe(scr, 1, cx, f" {tag.upper()} ", attr)
        cx += 5
        if i == len(active) - 1 and disabled:
            _safe(scr, 1, cx, "│", _cp(Color.NORMAL))
            cx += 2
    _safe(scr, 2, 1, " UP/DN:sel  u/d:move  SPC:toggle  Enter:apply  ESC:cancel ",
          _cp(Color.HELP))


def compute_two_col_layout(
        slots: list[EffectSlot]) -> list[tuple[int, int, int]]:
    """Compute (col, x, y) for each slot in a 2-column layout.

    Fills left column top-to-bottom first, then right column,
    so signal chain reads top-to-bottom, left then right.
    """
    col_x = [1, 1 + SLOT_WIDTH + COL_GAP]

    # Split: first half → left column, second half → right column
    mid = (len(slots) + 1) // 2  # left gets the extra slot if odd
    layout: list[tuple[int, int, int]] = []
    col_y = [0, 0]
    for i, s in enumerate(slots):
        col = 0 if i < mid else 1
        layout.append((col, col_x[col], col_y[col]))
        col_y[col] += s.height
    return layout


def compute_scroll_2col(layout: list[tuple[int, int, int]],
                        slots: list[EffectSlot], active_idx: int,
                        viewport_h: int, scroll_y: int) -> int:
    """Adjust scroll_y so the active slot is fully visible."""
    if active_idx >= len(layout):
        return 0
    _, _, y = layout[active_idx]
    h = slots[active_idx].height
    top, bot = y, y + h
    if top < scroll_y:
        scroll_y = top
    if bot > scroll_y + viewport_h:
        scroll_y = bot - viewport_h
    return max(0, scroll_y)


def total_height_2col(layout: list[tuple[int, int, int]],
                      slots: list[EffectSlot]) -> int:
    """Maximum height across both columns."""
    max_h = 0
    for (col, _, y), s in zip(layout, slots):
        max_h = max(max_h, y + s.height)
    return max_h


def draw_scroll_indicators(scr, max_x: int, total_h: int, viewport_h: int,
                           scroll_y: int) -> None:
    if total_h <= viewport_h:
        return
    ind_x = min(1 + SLOT_WIDTH * NUM_COLS + COL_GAP + 1, max_x - 1)
    _safe(scr, FIXED_TOP, ind_x, "▲" if scroll_y > 0 else " ", _cp(Color.HELP))
    _safe(scr, FIXED_TOP + viewport_h - 1, ind_x,
          "▼" if scroll_y + viewport_h < total_h else " ", _cp(Color.HELP))


def draw_status(scr, y: int, max_x: int, msg: str, msg_time: float,
                rx_log: list[str]) -> None:
    if msg and (time.time() - msg_time < 4.0):
        _safe(scr, y, 1, f" {msg[:max_x - 3]} ", _cp(Color.HELP))
    elif rx_log:
        _safe(scr, y, 1, rx_log[-1][:max_x - 3], _cp(Color.LOG))


def draw_help_bar(scr, max_y: int, max_x: int, mode: "InputMode", raw_buf: str,
                  raw_field: int) -> None:
    if mode == InputMode.CONNECTION:
        text = " CON │ </>:sink  UP/DN:src  Enter:apply  ESC:cancel "
    elif mode == InputMode.CHAIN_EDIT:
        text = " CHAIN │ UP/DN:sel  u/d:move  SPC:toggle  Enter:apply  ESC:cancel "
    elif mode == InputMode.RAW_WRITE:
        if raw_field == 0:
            text = f" RAW │ Addr: {raw_buf}_ │ 2 hex, Enter=next, ESC=cancel "
        else:
            text = f" RAW │ {raw_buf[:2]}:{raw_buf[2:]}_ │ 2 hex, Enter=send "
    else:
        text = (" TAB:slot hjkl:nav UP/DN:prm <>:±16 S<S>:±1"
                "  SPC:byp c:con o:chain s:stat w:raw q:quit ")
    _safe(scr, max_y - 1, 0, text[:max_x - 1].ljust(max_x - 1),
          _cp(Color.TITLE))


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Input handler — modal state machine                                    ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


class InputMode(IntEnum):
    NORMAL = auto()
    RAW_WRITE = auto()
    CONNECTION = auto()
    CHAIN_EDIT = auto()


@dataclass
class AppState:
    """All mutable application state in one place."""
    slot_dict: dict[str, EffectSlot]
    route: list[int]
    link: SerialLink
    active_tag: Optional[str] = None
    scroll_y: int = 0
    mode: InputMode = InputMode.NORMAL

    # status flash
    status_msg: str = ""
    status_time: float = 0.0

    # raw write sub-mode
    raw_buf: str = ""
    raw_field: int = 0  # 0 = entering addr, 1 = entering data

    # connection editor sub-mode
    con_sink: int = 0  # index into SINK_NODES
    con_src: int = 0  # index into SOURCE_NODES

    # chain editor sub-mode
    chain_order: list[str] = field(default_factory=list)
    chain_disabled_set: set[str] = field(default_factory=set)
    chain_cursor: int = 0

    running: bool = True

    def __post_init__(self):
        self._sync_chain_from_route()

    # ── helpers ──────────────────────────────────────────────────────

    def _sync_chain_from_route(self):
        chain_ports = trace_chain(self.route)
        self.chain_order = [PORT_TAG[p] for p in chain_ports
                            if PORT_TAG.get(p, "adc") in self.slot_dict]
        all_tags = [s.tag for s in EFFECT_DEFS]
        in_chain = set(self.chain_order)
        self.chain_disabled_set = {t for t in all_tags if t not in in_chain}
        for tag, slot in self.slot_dict.items():
            slot.disabled = tag in self.chain_disabled_set

    def flash(self, msg: str) -> None:
        self.status_msg = msg
        self.status_time = time.time()

    def get_ordered_slots(self) -> list[EffectSlot]:
        return ordered_slots(self.slot_dict, self.route)

    def active_index(self, slots: list[EffectSlot]) -> int:
        for i, s in enumerate(slots):
            if s.tag == self.active_tag:
                return i
        return 0

    def _current_src_for_sink(self) -> int:
        """Return SOURCE_NODES index for the current route value of con_sink."""
        snk_tag = SINK_NODES[self.con_sink]
        ridx = SINK_ROUTE_IDX[snk_tag]
        cur_port = self.route[ridx] if ridx < len(self.route) else 0
        cur_tag = PORT_TAG.get(cur_port, "adc")
        return SOURCE_NODES.index(cur_tag) if cur_tag in SOURCE_NODES else 0

    def enter_connection_mode(self) -> None:
        self.mode = InputMode.CONNECTION
        self.con_sink = 0
        self.con_src = self._current_src_for_sink()
        self.flash("Connection editor: pick sink then source")

    def enter_chain_mode(self) -> None:
        self.mode = InputMode.CHAIN_EDIT
        self._sync_chain_from_route()
        self.chain_cursor = 0
        self.flash("Chain: ^v select, u/d move, SPC toggle, Enter apply, ESC cancel")

    def apply_chain(self) -> None:
        """Send chain command and update local route table."""
        active = [t for t in self.chain_order
                  if t not in self.chain_disabled_set]
        if not active:
            self.flash("Chain must have at least one effect")
            return
        self.link.chain(active)
        # Update local route
        ports = [TAG_PORT[t] for t in active]
        for i, p in enumerate(ports):
            if p < len(self.route):
                self.route[p] = Port.ADC if i == 0 else ports[i - 1]
        self.route[0] = ports[-1]
        # Update disabled state on slots
        for tag, slot in self.slot_dict.items():
            slot.disabled = tag in self.chain_disabled_set
        self.mode = InputMode.NORMAL
        self.flash(f"Chain: {' > '.join(t.upper() for t in active)}")

    def enter_raw_write_mode(self) -> None:
        self.mode = InputMode.RAW_WRITE
        self.raw_buf = ""
        self.raw_field = 0
        self.flash("Raw write mode...")

    def update_route(self, route_idx: int, src_port: int) -> None:
        """Update local route table (called after con or raw write to 0x40-0x47)."""
        if 0 <= route_idx < len(self.route):
            self.route[route_idx] = src_port


# ── Key dispatch helpers ─────────────────────────────────────────────────

HEX_CHARS = set("0123456789abcdefABCDEF")
ENTER_KEYS = {curses.KEY_ENTER, 10, 13}
BACKSPACE_KEYS = {curses.KEY_BACKSPACE, 127, 8}


def _handle_normal(ch: int, st: AppState, slots: list[EffectSlot],
                   ai: int) -> None:
    """Process a keypress in NORMAL mode."""
    slot = slots[ai]
    n = len(slots)

    if ch in (ord('q'), ord('Q'), 27):
        st.running = False

    # ── slot navigation ──
    elif ch == ord('\t'):
        st.active_tag = slots[(ai + 1) % n].tag
    elif ch == curses.KEY_BTAB:
        st.active_tag = slots[(ai - 1) % n].tag

    # ── param navigation (wraps across slots) ──
    elif ch == curses.KEY_UP:
        if slot.selected_param == 0:
            new_ai = (ai - 1) % n
            st.active_tag = slots[new_ai].tag
            slots[new_ai].selected_param = len(slots[new_ai].params) - 1
        else:
            slot.selected_param -= 1

    elif ch == curses.KEY_DOWN:
        if slot.selected_param == len(slot.params) - 1:
            new_ai = (ai + 1) % n
            st.active_tag = slots[new_ai].tag
            slots[new_ai].selected_param = 0
        else:
            slot.selected_param += 1

    # ── value adjust ──
    elif ch in (curses.KEY_RIGHT, curses.KEY_LEFT, curses.KEY_PPAGE,
                curses.KEY_NPAGE, curses.KEY_SRIGHT, curses.KEY_SLEFT):
        if not slot.disabled:
            p = slot.params[slot.selected_param]
            if ch in (curses.KEY_RIGHT, curses.KEY_PPAGE, curses.KEY_SRIGHT):
                step = p.step_fine if ch == curses.KEY_SRIGHT else p.step_coarse
                p.value = min(p.value + step, p.hi)
            else:
                step = p.step_fine if ch == curses.KEY_SLEFT else p.step_coarse
                p.value = max(p.value - step, p.lo)
            st.link.set_param(slot, p)
            st.flash(f"set {slot.tag} {p.name} {p.format_cmd_value()}")

    # ── bypass ──
    elif ch == ord(' ') or ch in ENTER_KEYS:
        if not slot.disabled:
            slot.bypassed = not slot.bypassed
            st.link.set_bypass(slot, not slot.bypassed)
            st.flash(f"{slot.full_name}: {'ON' if not slot.bypassed else 'OFF'}")
        else:
            # Re-enable: remove from disabled set, apply chain
            st.chain_disabled_set.discard(slot.tag)
            if slot.tag not in st.chain_order:
                st.chain_order.append(slot.tag)
            st.apply_chain()
            slot.bypassed = False
            st.link.set_bypass(slot, True)
            st.flash(f"{slot.full_name}: re-enabled in chain")

    # ── hjkl vim navigation (2-column grid) ──
    elif ch in (ord('j'), ord('J'), ord('k'), ord('K'),
                ord('h'), ord('H'), ord('l'), ord('L')):
        mid = (n + 1) // 2  # same split as compute_two_col_layout
        col = 0 if ai < mid else 1
        row = ai if col == 0 else ai - mid
        left_len = mid
        right_len = n - mid

        if ch in (ord('j'), ord('J')):
            col_len = left_len if col == 0 else right_len
            row = (row + 1) % col_len
        elif ch in (ord('k'), ord('K')):
            col_len = left_len if col == 0 else right_len
            row = (row - 1) % col_len
        elif ch in (ord('l'), ord('L')):
            if col == 0 and right_len > 0:
                col = 1
                row = min(row, right_len - 1)
        elif ch in (ord('h'), ord('H')):
            if col == 1:
                col = 0
                row = min(row, left_len - 1)

        new_ai = row if col == 0 else mid + row
        st.active_tag = slots[new_ai].tag

    # ── mode switches ──
    elif ch in (ord('c'), ord('C')):
        st.enter_connection_mode()
    elif ch in (ord('o'), ord('O')):
        st.enter_chain_mode()
    elif ch in (ord('w'), ord('W')):
        st.enter_raw_write_mode()
    elif ch in (ord('s'), ord('S')):
        if sync_from_hardware(st.link, st.slot_dict, st.route):
            st.flash("Synced from hardware")
        else:
            st.flash("Status sync failed")
    elif ch in (ord('r'), ord('R')):
        st.flash("Refreshed")


def _handle_connection(ch: int, st: AppState) -> None:
    """Process a keypress in CONNECTION editor mode."""
    if ch == 27:
        st.mode = InputMode.NORMAL
        st.flash("Connection edit cancelled")

    elif ch == curses.KEY_LEFT:
        st.con_sink = (st.con_sink - 1) % len(SINK_NODES)
        st.con_src = st._current_src_for_sink()

    elif ch == curses.KEY_RIGHT:
        st.con_sink = (st.con_sink + 1) % len(SINK_NODES)
        st.con_src = st._current_src_for_sink()

    elif ch == curses.KEY_UP:
        st.con_src = (st.con_src - 1) % len(SOURCE_NODES)

    elif ch == curses.KEY_DOWN:
        st.con_src = (st.con_src + 1) % len(SOURCE_NODES)

    elif ch in ENTER_KEYS:
        src_tag = SOURCE_NODES[st.con_src]
        snk_tag = SINK_NODES[st.con_sink]
        ridx = SINK_ROUTE_IDX[snk_tag]
        src_port = TAG_PORT.get(src_tag, 0)
        st.link.connect(src_tag, snk_tag)
        st.update_route(ridx, src_port)
        st.flash(f"con {src_tag} {snk_tag}  (route[{ridx}]={src_port})")
        st.mode = InputMode.NORMAL


def _handle_raw_write(ch: int, st: AppState) -> None:
    """Process a keypress in RAW_WRITE mode."""
    if ch == 27:
        st.mode = InputMode.NORMAL
        st.flash("Raw write cancelled")

    elif ch in ENTER_KEYS:
        if len(st.raw_buf) == 4:
            try:
                addr = int(st.raw_buf[:2], 16)
                data = int(st.raw_buf[2:], 16)
            except ValueError:
                st.flash("Invalid hex")
                st.mode = InputMode.NORMAL
                return
            st.link.raw_write(addr, data)
            # Keep local route table in sync
            if ROUTE_ADDR_BASE <= addr <= ROUTE_ADDR_BASE + 8:
                st.update_route(addr - ROUTE_ADDR_BASE, data)
                st.flash(f"wr {addr:02X}{data:02X} (route updated)")
            else:
                st.flash(f"wr {addr:02X}{data:02X}")
            st.mode = InputMode.NORMAL
        elif len(st.raw_buf) == 2:
            st.raw_field = 1
        else:
            st.flash("Need 2+2 hex digits")

    elif ch in BACKSPACE_KEYS:
        if st.raw_buf:
            st.raw_buf = st.raw_buf[:-1]
            if len(st.raw_buf) < 2:
                st.raw_field = 0

    else:
        c = chr(ch) if 0 <= ch < 256 else ""
        if c in HEX_CHARS and len(st.raw_buf) < 4:
            st.raw_buf += c.upper()


def _handle_chain_edit(ch: int, st: AppState) -> None:
    """Process a keypress in CHAIN_EDIT mode."""
    all_tags = [s.tag for s in EFFECT_DEFS]
    active = [t for t in st.chain_order if t not in st.chain_disabled_set]
    disabled = [t for t in all_tags if t in st.chain_disabled_set]
    display = active + disabled
    n = len(display)
    if n == 0:
        st.mode = InputMode.NORMAL
        return

    if ch == 27:
        st.mode = InputMode.NORMAL
        st.flash("Chain edit cancelled")

    elif ch == curses.KEY_UP:
        st.chain_cursor = (st.chain_cursor - 1) % n

    elif ch == curses.KEY_DOWN:
        st.chain_cursor = (st.chain_cursor + 1) % n

    elif ch in (ord('u'), ord('U')):
        if st.chain_cursor >= len(active):
            # Re-enable disabled effect, insert at end of active
            tag = display[st.chain_cursor]
            st.chain_disabled_set.discard(tag)
            if tag not in st.chain_order:
                st.chain_order.append(tag)
            new_active = [t for t in st.chain_order if t not in st.chain_disabled_set]
            st.chain_cursor = len(new_active) - 1
        elif st.chain_cursor > 0:
            tag = display[st.chain_cursor]
            idx = st.chain_order.index(tag)
            if idx > 0:
                st.chain_order[idx - 1], st.chain_order[idx] = \
                    st.chain_order[idx], st.chain_order[idx - 1]
                st.chain_cursor -= 1

    elif ch in (ord('d'), ord('D')):
        if st.chain_cursor >= len(active):
            # Re-enable disabled effect, insert at end of active
            tag = display[st.chain_cursor]
            st.chain_disabled_set.discard(tag)
            if tag not in st.chain_order:
                st.chain_order.append(tag)
            new_active = [t for t in st.chain_order if t not in st.chain_disabled_set]
            st.chain_cursor = len(new_active) - 1
        elif st.chain_cursor < len(active) - 1:
            tag = display[st.chain_cursor]
            idx = st.chain_order.index(tag)
            if idx < len(st.chain_order) - 1:
                st.chain_order[idx], st.chain_order[idx + 1] = \
                    st.chain_order[idx + 1], st.chain_order[idx]
                st.chain_cursor += 1

    elif ch == ord(' '):
        # Toggle effect in/out of chain
        tag = display[st.chain_cursor]
        if tag in st.chain_disabled_set:
            st.chain_disabled_set.discard(tag)
            if tag not in st.chain_order:
                st.chain_order.append(tag)
        else:
            if len(active) > 1:  # keep at least one
                st.chain_disabled_set.add(tag)
        # Recompute display and follow the item
        new_active = [t for t in st.chain_order if t not in st.chain_disabled_set]
        new_disabled = [t for t in all_tags if t in st.chain_disabled_set]
        new_display = new_active + new_disabled
        if tag in new_display:
            st.chain_cursor = new_display.index(tag)

    elif ch in ENTER_KEYS:
        st.apply_chain()


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Main loop                                                              ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


def main(stdscr, args: argparse.Namespace) -> None:
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(100)
    theme = _load_theme(getattr(args, 'theme', None))
    _init_colors(theme)

    # ── state ──
    link = SerialLink(args.port, args.baud)
    if not args.offline:
        if link.open():
            link.rx_log.append(f"Connected: {args.port} @ {args.baud}")
        else:
            link.rx_log.append(f"FAILED to open {args.port} — offline")
    else:
        link.rx_log.append("Offline mode (--offline)")

    st = AppState(
        slot_dict=build_slot_dict(),
        route=list(DEFAULT_ROUTE),
        link=link,
    )

    # ── sync local state from hardware shadow registers ──
    if link.connected:
        if sync_from_hardware(link, st.slot_dict, st.route):
            st.flash("Synced from hardware")
        else:
            st.flash("Sync failed — using defaults")

    # ── loop ──
    while st.running:
        slots = st.get_ordered_slots()
        if not slots:
            break
        if st.active_tag is None:
            st.active_tag = slots[0].tag
        ai = st.active_index(slots)

        max_y, max_x = stdscr.getmaxyx()
        viewport_h = max(max_y - FIXED_TOP - FIXED_BOT, 3)
        stdscr.erase()

        # ── draw chrome ──
        draw_title(stdscr, max_x, args.port, args.baud, link.connected)

        if st.mode == InputMode.CONNECTION:
            draw_connection_editor(stdscr, st.con_sink, st.con_src, st.route)
        elif st.mode == InputMode.CHAIN_EDIT:
            draw_chain_editor(stdscr, st)
        else:
            draw_chain(stdscr, slots)

        # ── draw slots in 2 columns directly on stdscr ──
        layout = compute_two_col_layout(slots)
        total_h = total_height_2col(layout, slots)
        st.scroll_y = compute_scroll_2col(layout, slots, ai, viewport_h,
                                          st.scroll_y)

        for idx, slot in enumerate(slots):
            col, cx, cy = layout[idx]
            screen_y = FIXED_TOP + cy - st.scroll_y
            # Skip slots entirely above or below the viewport
            if screen_y + slot.height <= FIXED_TOP:
                continue
            if screen_y >= FIXED_TOP + viewport_h:
                continue
            draw_slot(stdscr, slot, screen_y, cx, idx == ai)

        draw_scroll_indicators(stdscr, max_x, total_h, viewport_h, st.scroll_y)
        draw_status(stdscr, max_y - 2, max_x, st.status_msg, st.status_time,
                    link.rx_log)
        draw_help_bar(stdscr, max_y, max_x, st.mode, st.raw_buf, st.raw_field)

        # ── refresh ──
        stdscr.noutrefresh()
        curses.doupdate()

        # ── input dispatch ──
        ch = stdscr.getch()
        if ch == -1:
            continue

        if st.mode == InputMode.CONNECTION:
            _handle_connection(ch, st)
        elif st.mode == InputMode.RAW_WRITE:
            _handle_raw_write(ch, st)
        elif st.mode == InputMode.CHAIN_EDIT:
            _handle_chain_edit(ch, st)
        else:
            _handle_normal(ch, st, slots, ai)

    link.close()


# ╔═══════════════════════════════════════════════════════════════════════════╗
# ║  Entry point                                                            ║
# ╚═══════════════════════════════════════════════════════════════════════════╝


def run() -> None:
    parser = argparse.ArgumentParser(
        description="ncurses GUI for GTFX FPGA effects processor")
    parser.add_argument("--port",
                        "-p",
                        default="/dev/ttyUSB0",
                        help="Serial port  (default: /dev/ttyUSB0)")
    parser.add_argument("--baud",
                        "-b",
                        type=int,
                        default=115200,
                        help="Baud rate  (default: 115200)")
    parser.add_argument("--offline",
                        action="store_true",
                        help="Run without serial (demo / preview mode)")
    parser.add_argument("--theme",
                        "-t",
                        default=None,
                        help="Path to theme .toml file")
    args = parser.parse_args()

    if not HAS_SERIAL and not args.offline:
        print("pyserial not installed.  pip install pyserial")
        print("Or run with --offline for preview mode.")
        sys.exit(1)

    curses.wrapper(lambda scr: main(scr, args))


if __name__ == "__main__":
    run()
