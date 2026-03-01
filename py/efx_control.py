#!/usr/bin/env python3
"""
efx_control.py - ncurses GUI for FPGA guitar effects command processor (cmd_proc_v2)

Sends commands over serial to set effect parameters, toggle bypass, and view status.
Slot display order reflects the shadow_route table and updates at runtime.

Usage:
    python3 efx_control.py [--port /dev/ttyUSB0] [--baud 115200]

Keys:
    TAB / Shift-TAB   Move between effect slots (in chain order)
    UP / DOWN         Move between parameters (wraps across slots)
    LEFT / RIGHT      Adjust value (-1 / +1)
    PgUp / PgDn       Adjust value (+16 / -16)
    SPACE / ENTER     Toggle effect bypass on/off
    o / f             Force ON / OFF
    s                 Request status dump from hardware
    r                 Refresh display from local state
    w                 Raw write mode (prompts for addr and data)
    q / ESC           Quit
"""

import argparse
import curses
import sys
import time
import threading

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# =============================================================================
# Routing model  (mirrors cmd_proc_v2 shadow_route)
#
#   Port indices:  0=ADC  1=TRM  2=PHA  3=CHO  4=DLY  5=TUB  6=FLN
#   route[sink] = source   (who feeds into sink)
#   DAC is implicit sink index 0 (route[0] = who feeds DAC)
#   ADC is the ultimate source (port 0), never a sink.
# =============================================================================

PORT_ADC = 0
PORT_TRM = 1
PORT_PHA = 2
PORT_CHO = 3
PORT_DLY = 4
PORT_TUB = 5
PORT_FLN = 6

PORT_TAG = {
    PORT_ADC: "adc",
    PORT_TRM: "trm",
    PORT_PHA: "pha",
    PORT_CHO: "cho",
    PORT_DLY: "dly",
    PORT_TUB: "tub",
    PORT_FLN: "fln",
}

# Reverse: tag -> port index
TAG_PORT = {v: k for k, v in PORT_TAG.items()}
TAG_PORT["dac"] = 7  # virtual, used for display only

# All nodes that can be a source (output side)
ALL_NODES = ["adc", "trm", "pha", "cho", "dly", "tub", "fln"]

# All nodes that can be a sink (input side) -- these have route[] entries
# route[0]=DAC, route[1]=TRM, ..., route[6]=FLN
SINK_NODES = ["dac", "trm", "pha", "cho", "dly", "tub", "fln"]

# Map sink tag to route[] index
SINK_ROUTE_IDX = {
    "dac": 0, "trm": 1, "pha": 2, "cho": 3,
    "dly": 4, "tub": 5, "fln": 6,
}

# Default route: ADC->FLN->TUB->TRM->PHA->CHO->DLY->DAC
DEFAULT_ROUTE = [
    4,  # route[0] DAC  <- DLY
    5,  # route[1] TRM  <- TUB
    1,  # route[2] PHA  <- TRM
    2,  # route[3] CHO  <- PHA
    3,  # route[4] DLY  <- CHO
    6,  # route[5] TUB  <- FLN
    0,  # route[6] FLN  <- ADC
]


def trace_chain(route):
    """Walk route[] from DAC backwards to ADC, return list of port indices
    in signal-flow order (first = right after ADC, last = feeds DAC).
    Protects against loops with a visited set."""
    chain = []
    visited = set()
    port = route[0]          # start: who feeds DAC
    while port != PORT_ADC and port not in visited:
        visited.add(port)
        chain.append(port)
        if port < len(route):
            port = route[port]
        else:
            break
    chain.reverse()           # now first element is closest to ADC
    return chain


# =============================================================================
# Data model
# =============================================================================

class Param:
    def __init__(self, name, label, addr, default, lo=0x00, hi=0xFF, fmt="hex"):
        self.name  = name
        self.label = label
        self.addr  = addr
        self.value = default
        self.default = default
        self.lo = lo
        self.hi = hi
        self.fmt = fmt

class Param16:
    def __init__(self, name, label, addr, default, lo=0x0000, hi=0xFFFF):
        self.name  = name
        self.label = label
        self.addr  = addr
        self.value = default
        self.default = default
        self.lo = lo
        self.hi = hi
        self.fmt = "hex16"

class EffectSlot:
    def __init__(self, tag, full_name, port_idx, params, bypass_addr,
                 bypass_default=0x01):
        self.tag       = tag
        self.full_name = full_name
        self.port_idx  = port_idx        # routing port index
        self.params    = params
        self.bypass_addr = bypass_addr
        self.bypassed  = bool(bypass_default)
        self.selected_param = 0


def build_effects():
    """Return dict keyed by tag for O(1) lookup."""
    defs = [
        EffectSlot("trm", "Tremolo", PORT_TRM, [
            Param("rat", "Rate",  0x00, 0x3C),
            Param("dep", "Depth", 0x01, 0xB4),
            Param("shp", "Shape", 0x02, 0x00),
        ], bypass_addr=0x07),

        EffectSlot("pha", "Phaser", PORT_PHA, [
            Param("spd", "Speed",       0x08, 0x50),
            Param("fbn", "Feedback En", 0x09, 0x00),
        ], bypass_addr=0x0F),

        EffectSlot("cho", "Chorus", PORT_CHO, [
            Param("rat", "Rate",    0x10, 0x50),
            Param("dep", "Depth",   0x11, 0x64),
            Param("efx", "Effect",  0x12, 0x80),
            Param("eqh", "EQ High", 0x13, 0xC8),
            Param("eql", "EQ Low",  0x14, 0x80),
        ], bypass_addr=0x17),

        EffectSlot("dly", "Delay (DD3)", PORT_DLY, [
            Param("ton",   "Tone",     0x18, 0xFF),
            Param("lvl",   "Level",    0x19, 0x80),
            Param("fdb",   "Feedback", 0x1A, 0x64),
            Param16("tim", "Time",     0x1B, 0x3000),
        ], bypass_addr=0x1F),

        EffectSlot("tub", "Tube Distortion", PORT_TUB, [
            Param("gai", "Gain",        0x20, 0x40),
            Param("bas", "Tone Bass",   0x21, 0x80),
            Param("mid", "Tone Mid",    0x22, 0x80),
            Param("tre", "Tone Treble", 0x23, 0x80),
            Param("lvl", "Level",       0x24, 0xA0),
        ], bypass_addr=0x27),

        EffectSlot("fln", "Flanger", PORT_FLN, [
            Param("man", "Manual",  0x28, 0x80),
            Param("wid", "Width",   0x29, 0xC0),
            Param("spd", "Speed",   0x2A, 0x30),
            Param("reg", "Regen",   0x2B, 0xA0),
        ], bypass_addr=0x2F),
    ]
    return {s.tag: s for s in defs}


def ordered_slots(slot_dict, route):
    """Return slots list in signal-chain order derived from route table."""
    chain = trace_chain(route)     # port indices, ADC-side first
    ordered = []
    for port in chain:
        tag = PORT_TAG.get(port)
        if tag and tag != "adc" and tag in slot_dict:
            ordered.append(slot_dict[tag])
    # Append any slots not found in the chain (shouldn't happen, but safety)
    seen = {s.tag for s in ordered}
    for tag, s in slot_dict.items():
        if tag not in seen:
            ordered.append(s)
    return ordered


# =============================================================================
# Serial interface
# =============================================================================

class SerialLink:
    def __init__(self, port, baud, timeout=0.5):
        self.port_name = port
        self.baud      = baud
        self.timeout   = timeout
        self.ser       = None
        self.lock      = threading.Lock()
        self.rx_log    = []
        self.connected = False

    def open(self):
        if not HAS_SERIAL:
            return False
        try:
            self.ser = serial.Serial(self.port_name, self.baud,
                                     timeout=self.timeout)
            self.connected = True
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            self.rx_log.append(f"Open error: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False

    def _send_cmd(self, cmd_str):
        with self.lock:
            if not self.connected:
                self.rx_log.append(f"[offline] {cmd_str}")
                return ""
            self.ser.write((cmd_str + "\r").encode("ascii"))
            time.sleep(0.05)
            resp = ""
            try:
                resp = self.ser.read(self.ser.in_waiting or 1) \
                           .decode("ascii", errors="replace")
            except Exception:
                pass
            if resp:
                self.rx_log.append(resp.strip())
            if len(self.rx_log) > 200:
                self.rx_log = self.rx_log[-100:]
            return resp

    def set_param(self, tag, name, val):
        return self._send_cmd(f"set {tag} {name} {val:02X}")

    def set_param16(self, tag, name, val):
        return self._send_cmd(f"set {tag} {name} {val:04X}")

    def set_bypass(self, tag, active):
        return self._send_cmd(f"set {tag} {'on' if active else 'off'}")

    def connect_route(self, src_tag, snk_tag):
        return self._send_cmd(f"con {src_tag} {snk_tag}")

    def raw_write(self, addr, data):
        return self._send_cmd(f"wr {addr:02X}{data:02X}")

    def request_status(self):
        with self.lock:
            if not self.connected:
                self.rx_log.append("[offline] status request")
                return ""
            self.ser.write(b";")
            time.sleep(0.15)
            resp = ""
            try:
                resp = self.ser.read(self.ser.in_waiting or 1) \
                           .decode("ascii", errors="replace")
            except Exception:
                pass
            if resp:
                for line in resp.split("\n"):
                    s = line.strip()
                    if s:
                        self.rx_log.append(s)
            if len(self.rx_log) > 200:
                self.rx_log = self.rx_log[-100:]
            return resp


# =============================================================================
# ncurses GUI
# =============================================================================

CP_NORMAL   = 1
CP_HEADER   = 2
CP_ACTIVE   = 3
CP_BYPASSED = 4
CP_SELECTED = 5
CP_BAR      = 6
CP_TITLE    = 7
CP_LOG      = 8
CP_HELP     = 9
CP_WARN     = 10

SLOT_W = 36


def draw_bar(win, y, x, value, max_val, width, color_pair):
    filled = int((value / max(max_val, 1)) * width)
    filled = max(0, min(filled, width))
    try:
        win.addstr(y, x, "#" * filled,
                   curses.color_pair(color_pair) | curses.A_BOLD)
        win.addstr("." * (width - filled), curses.color_pair(CP_NORMAL))
    except curses.error:
        pass


def draw_slot(pad, slot, y, is_active):
    w = SLOT_W
    x = 0

    if is_active:
        hdr_attr = curses.color_pair(CP_HEADER) | curses.A_BOLD
    elif slot.bypassed:
        hdr_attr = curses.color_pair(CP_BYPASSED)
    else:
        hdr_attr = curses.color_pair(CP_ACTIVE) | curses.A_BOLD

    bypass_tag = " ON" if not slot.bypassed else "OFF"
    byp_attr = (curses.color_pair(CP_ACTIVE) | curses.A_BOLD) if not slot.bypassed \
               else curses.color_pair(CP_WARN)
    name = slot.full_name.upper()

    # Header
    try:
        inner = w - 2
        name_field = inner - 4
        pad.addstr(y, x, "+" + "-" * (w - 2) + "+", hdr_attr)
        pad.addstr(y, x + 1, f" {name:<{name_field}}", hdr_attr)
        pad.addstr(y, x + 1 + name_field, f"{bypass_tag} ", byp_attr)
    except curses.error:
        pass
    y += 1

    LABEL_W = 10
    VAL_W   = 4
    bar_w   = w - 2 - 1 - LABEL_W - 1 - VAL_W - 1 - 1
    if bar_w < 2:
        bar_w = 2

    for i, p in enumerate(slot.params):
        is_sel = is_active and i == slot.selected_param
        if is_sel:
            attr = curses.color_pair(CP_SELECTED) | curses.A_BOLD
            mk = ">"
        elif slot.bypassed:
            attr = curses.color_pair(CP_BYPASSED)
            mk = " "
        else:
            attr = curses.color_pair(CP_NORMAL)
            mk = " "

        val_str = f"{p.value:04X}" if p.fmt == "hex16" else f"  {p.value:02X}"
        body = f"{mk}{p.label:<{LABEL_W}} {val_str} "

        try:
            pad.addstr(y, x, "|", hdr_attr if not is_sel else attr)
            pad.addstr(body, attr)
            bar_x = x + 1 + len(body)
            bar_cp = CP_SELECTED if is_sel else (CP_BAR if not slot.bypassed else CP_BYPASSED)
            draw_bar(pad, y, bar_x, p.value, p.hi, bar_w, bar_cp)
            pad.addstr(y, x + w - 1, "|", hdr_attr if not is_sel else attr)
        except curses.error:
            pass
        y += 1

    try:
        pad.addstr(y, x, "+" + "-" * (w - 2) + "+", hdr_attr)
    except curses.error:
        pass
    y += 1
    return y


def calc_total_height(slots):
    h = 0
    for s in slots:
        h += 1 + len(s.params) + 1
    return h


def compute_scroll(slots, active_idx, viewport_h, scroll_y):
    y = 0
    for idx, s in enumerate(slots):
        slot_h = 1 + len(s.params) + 1
        if idx == active_idx:
            top = y
            bot = y + slot_h
            if top < scroll_y:
                scroll_y = top
            if bot > scroll_y + viewport_h:
                scroll_y = bot - viewport_h
            break
        y += slot_h
    return max(0, scroll_y)


# -----------------------------------------------------------------------------

def main(stdscr, args):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(100)
    curses.start_color()
    curses.use_default_colors()

    curses.init_pair(CP_NORMAL,   curses.COLOR_WHITE,  -1)
    curses.init_pair(CP_HEADER,   curses.COLOR_BLACK,  curses.COLOR_CYAN)
    curses.init_pair(CP_ACTIVE,   curses.COLOR_GREEN,  -1)
    curses.init_pair(CP_BYPASSED, curses.COLOR_BLACK,  -1)
    curses.init_pair(CP_SELECTED, curses.COLOR_BLACK,  curses.COLOR_YELLOW)
    curses.init_pair(CP_BAR,      curses.COLOR_GREEN,  -1)
    curses.init_pair(CP_TITLE,    curses.COLOR_BLACK,  curses.COLOR_WHITE)
    curses.init_pair(CP_LOG,      curses.COLOR_GREEN,  -1)
    curses.init_pair(CP_HELP,     curses.COLOR_YELLOW, -1)
    curses.init_pair(CP_WARN,     curses.COLOR_RED,    -1)

    # -- data model --
    slot_dict = build_effects()
    route = list(DEFAULT_ROUTE)        # mutable copy of shadow_route
    active_tag = None                  # tag of currently selected slot

    # -- serial --
    link = SerialLink(args.port, args.baud)
    if not args.offline:
        if link.open():
            link.rx_log.append(f"Connected: {args.port} @ {args.baud}")
        else:
            link.rx_log.append(f"FAILED to open {args.port} - running offline")
    else:
        link.rx_log.append("Offline mode (--offline)")

    status_msg  = ""
    status_time = 0
    scroll_y    = 0
    raw_mode    = False
    raw_buf     = ""
    raw_field   = 0
    con_mode    = False        # connection editor active
    con_sink    = 0            # index into SINK_NODES
    con_src     = 0            # index into ALL_NODES

    def set_status(msg):
        nonlocal status_msg, status_time
        status_msg = msg
        status_time = time.time()

    def get_slots():
        """Ordered slot list from current route table."""
        return ordered_slots(slot_dict, route)

    def active_index(slots):
        """Find index of active_tag in the current ordered list."""
        for i, s in enumerate(slots):
            if s.tag == active_tag:
                return i
        return 0

    def send_current_param(slots, ai):
        s = slots[ai]
        p = s.params[s.selected_param]
        if p.fmt == "hex16":
            link.set_param16(s.tag, p.name, p.value)
        else:
            link.set_param(s.tag, p.name, p.value)
        set_status(f"set {s.tag} {p.name} "
                   + (f"{p.value:04X}" if p.fmt == "hex16" else f"{p.value:02X}"))

    def toggle_bypass(slots, ai):
        s = slots[ai]
        s.bypassed = not s.bypassed
        link.set_bypass(s.tag, not s.bypassed)
        set_status(f"{s.full_name}: {'ON' if not s.bypassed else 'OFF'}")

    # -- main loop --
    while True:
        slots = get_slots()
        if not slots:
            break
        if active_tag is None:
            active_tag = slots[0].tag
        ai = active_index(slots)

        max_y, max_x = stdscr.getmaxyx()
        stdscr.erase()

        FIXED_TOP = 3
        FIXED_BOT = 2
        viewport_h = max(max_y - FIXED_TOP - FIXED_BOT, 3)

        # -- title --
        title = " EFX Control | cmd_proc_v2 "
        conn  = f" {args.port}@{args.baud}" if link.connected else " OFFLINE"
        tline = f"{title}{conn:>{max_x - len(title) - 1}}"
        try:
            stdscr.addstr(0, 0, tline[:max_x-1].ljust(max_x-1),
                          curses.color_pair(CP_TITLE))
        except curses.error:
            pass

        # -- chain string / connection editor --
        if con_mode:
            # Show route table editor on rows 1-2
            # Row 1: sink labels with highlight on selected
            # Row 2: source for each sink (current route value)
            try:
                stdscr.addstr(1, 1, "ROUTE EDITOR  ", curses.color_pair(CP_WARN) | curses.A_BOLD)
                cx = 16
                for si, snk in enumerate(SINK_NODES):
                    if si == con_sink:
                        attr = curses.color_pair(CP_SELECTED) | curses.A_BOLD
                    else:
                        attr = curses.color_pair(CP_NORMAL)
                    stdscr.addstr(1, cx, f" {snk.upper()} ", attr)
                    cx += 5
            except curses.error:
                pass
            # Row 2: show "src -> snk" for selected, all current assignments
            try:
                snk_tag = SINK_NODES[con_sink]
                ridx = SINK_ROUTE_IDX[snk_tag]
                cur_src_port = route[ridx] if ridx < len(route) else 0
                cur_src_tag = PORT_TAG.get(cur_src_port, "???")
                # Show source selector
                stdscr.addstr(2, 1, f"{snk_tag.upper()} input: ", curses.color_pair(CP_HELP))
                sx = 14
                for ni, node in enumerate(ALL_NODES):
                    if ni == con_src:
                        attr = curses.color_pair(CP_SELECTED) | curses.A_BOLD
                    elif node == cur_src_tag:
                        attr = curses.color_pair(CP_ACTIVE) | curses.A_BOLD
                    else:
                        attr = curses.color_pair(CP_NORMAL)
                    stdscr.addstr(2, sx, f" {node.upper()} ", attr)
                    sx += 5
            except curses.error:
                pass
        else:
            chain = "ADC"
            for s in slots:
                if not s.bypassed:
                    chain += f">{s.tag}"
            chain += ">DAC"
            try:
                stdscr.addstr(1, 1, "Chain: ", curses.color_pair(CP_HELP))
                stdscr.addstr(chain, curses.color_pair(CP_ACTIVE) | curses.A_BOLD)
            except curses.error:
                pass

        # -- build pad --
        total_h = calc_total_height(slots)
        pad_h   = max(total_h + 2, viewport_h + 2)
        pad_w   = SLOT_W + 2
        pad     = curses.newpad(pad_h, pad_w)

        py = 0
        for idx, slot in enumerate(slots):
            py = draw_slot(pad, slot, py, idx == ai)

        scroll_y = compute_scroll(slots, ai, viewport_h, scroll_y)
        blit_bottom = min(FIXED_TOP + viewport_h - 1, max_y - FIXED_BOT - 1)
        blit_right  = min(1 + pad_w - 1, max_x - 1)

        # -- scroll indicator --
        if total_h > viewport_h:
            try:
                stdscr.addstr(FIXED_TOP, SLOT_W + 3,
                              "^" if scroll_y > 0 else " ",
                              curses.color_pair(CP_HELP))
                stdscr.addstr(FIXED_TOP + viewport_h - 1, SLOT_W + 3,
                              "v" if scroll_y + viewport_h < total_h else " ",
                              curses.color_pair(CP_HELP))
            except curses.error:
                pass

        # -- status --
        sty = max_y - 2
        if status_msg and (time.time() - status_time < 4.0):
            try:
                stdscr.addstr(sty, 1, f" {status_msg[:max_x-3]} ",
                              curses.color_pair(CP_HELP))
            except curses.error:
                pass
        elif link.rx_log:
            try:
                stdscr.addstr(sty, 1, link.rx_log[-1][:max_x-3],
                              curses.color_pair(CP_LOG))
            except curses.error:
                pass

        # -- help bar --
        if con_mode:
            hstr = " CON MODE | </>:sink  ^/v:source  Enter:apply  ESC:cancel "
        elif raw_mode:
            if raw_field == 0:
                hstr = f" RAW | Addr: {raw_buf}_ | 2 hex, Enter=next, ESC=cancel "
            else:
                hstr = f" RAW | {raw_buf[:2]}:{raw_buf[2:]}_ | 2 hex, Enter=send "
        else:
            hstr = " TAB:slot ^v:prm <>:+-1 PgU/D:+-16 SPC:byp o:ON f:OFF c:con s:stat w:raw q:quit "
        try:
            stdscr.addstr(max_y - 1, 0, hstr[:max_x-1].ljust(max_x-1),
                          curses.color_pair(CP_TITLE))
        except curses.error:
            pass

        # -- refresh: stdscr first, pad overlay second --
        stdscr.noutrefresh()
        try:
            pad.noutrefresh(scroll_y, 0,
                            FIXED_TOP, 1,
                            blit_bottom, blit_right)
        except curses.error:
            pass
        curses.doupdate()

        # -- input --
        ch = stdscr.getch()
        if ch == -1:
            continue

        if con_mode:
            if ch == 27:
                con_mode = False
                set_status("Connection edit cancelled")
            elif ch == curses.KEY_LEFT:
                con_sink = (con_sink - 1) % len(SINK_NODES)
                # Pre-select current source for this sink
                snk_tag = SINK_NODES[con_sink]
                ridx = SINK_ROUTE_IDX[snk_tag]
                cur_port = route[ridx] if ridx < len(route) else 0
                cur_tag = PORT_TAG.get(cur_port, "adc")
                con_src = ALL_NODES.index(cur_tag) if cur_tag in ALL_NODES else 0
            elif ch == curses.KEY_RIGHT:
                con_sink = (con_sink + 1) % len(SINK_NODES)
                snk_tag = SINK_NODES[con_sink]
                ridx = SINK_ROUTE_IDX[snk_tag]
                cur_port = route[ridx] if ridx < len(route) else 0
                cur_tag = PORT_TAG.get(cur_port, "adc")
                con_src = ALL_NODES.index(cur_tag) if cur_tag in ALL_NODES else 0
            elif ch == curses.KEY_UP:
                con_src = (con_src - 1) % len(ALL_NODES)
            elif ch == curses.KEY_DOWN:
                con_src = (con_src + 1) % len(ALL_NODES)
            elif ch in (curses.KEY_ENTER, 10, 13):
                src_tag = ALL_NODES[con_src]
                snk_tag = SINK_NODES[con_sink]
                link.connect_route(src_tag, snk_tag)
                # Update local route table
                ridx = SINK_ROUTE_IDX[snk_tag]
                src_port = TAG_PORT.get(src_tag, 0)
                route[ridx] = src_port
                set_status(f"con {src_tag} {snk_tag} (route[{ridx}]={src_port})")
                con_mode = False
            continue

        if raw_mode:
            if ch == 27:
                raw_mode = False; raw_buf = ""; raw_field = 0
                set_status("Raw write cancelled")
            elif ch in (curses.KEY_ENTER, 10, 13):
                if len(raw_buf) == 4:
                    try:
                        a = int(raw_buf[:2], 16)
                        d = int(raw_buf[2:], 16)
                        link.raw_write(a, d)
                        # Update local route table if it was a route write
                        if 0x40 <= a <= 0x46:
                            route[a - 0x40] = d
                            set_status(f"wr {a:02X}{d:02X} (route updated)")
                        else:
                            set_status(f"wr {a:02X}{d:02X}")
                    except ValueError:
                        set_status("Invalid hex")
                    raw_mode = False; raw_buf = ""; raw_field = 0
                elif len(raw_buf) == 2:
                    raw_field = 1
                else:
                    set_status("Need 2+2 hex digits")
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                if raw_buf:
                    raw_buf = raw_buf[:-1]
                    if len(raw_buf) < 2:
                        raw_field = 0
            else:
                c = chr(ch) if 0 <= ch < 256 else ""
                if c in "0123456789abcdefABCDEF" and len(raw_buf) < 4:
                    raw_buf += c.upper()
            continue

        if ch in (ord('q'), ord('Q'), 27):
            break

        elif ch == ord('\t'):
            ai = (ai + 1) % len(slots)
            active_tag = slots[ai].tag
        elif ch == curses.KEY_BTAB:
            ai = (ai - 1) % len(slots)
            active_tag = slots[ai].tag

        elif ch == curses.KEY_UP:
            s = slots[ai]
            if s.selected_param == 0:
                ai = (ai - 1) % len(slots)
                active_tag = slots[ai].tag
                slots[ai].selected_param = len(slots[ai].params) - 1
            else:
                s.selected_param -= 1
        elif ch == curses.KEY_DOWN:
            s = slots[ai]
            if s.selected_param == len(s.params) - 1:
                ai = (ai + 1) % len(slots)
                active_tag = slots[ai].tag
                slots[ai].selected_param = 0
            else:
                s.selected_param += 1

        elif ch == curses.KEY_RIGHT:
            s = slots[ai]; p = s.params[s.selected_param]
            step = 0x100 if p.fmt == "hex16" else 1
            p.value = min(p.value + step, p.hi)
            send_current_param(slots, ai)
        elif ch == curses.KEY_LEFT:
            s = slots[ai]; p = s.params[s.selected_param]
            step = 0x100 if p.fmt == "hex16" else 1
            p.value = max(p.value - step, p.lo)
            send_current_param(slots, ai)
        elif ch == curses.KEY_PPAGE:
            s = slots[ai]; p = s.params[s.selected_param]
            step = 0x1000 if p.fmt == "hex16" else 0x10
            p.value = min(p.value + step, p.hi)
            send_current_param(slots, ai)
        elif ch == curses.KEY_NPAGE:
            s = slots[ai]; p = s.params[s.selected_param]
            step = 0x1000 if p.fmt == "hex16" else 0x10
            p.value = max(p.value - step, p.lo)
            send_current_param(slots, ai)

        elif ch in (ord(' '),):
            toggle_bypass(slots, ai)
        elif ch in (ord('o'), ord('O')):
            s = slots[ai]
            if s.bypassed:
                s.bypassed = False; link.set_bypass(s.tag, True)
                set_status(f"{s.full_name}: ON")
            else:
                set_status(f"{s.full_name}: already ON")
        elif ch in (ord('f'), ord('F')):
            s = slots[ai]
            if not s.bypassed:
                s.bypassed = True; link.set_bypass(s.tag, False)
                set_status(f"{s.full_name}: OFF")
            else:
                set_status(f"{s.full_name}: already OFF")
        elif ch in (curses.KEY_ENTER, 10, 13):
            toggle_bypass(slots, ai)

        elif ch in (ord('s'), ord('S')):
            link.request_status()
            set_status("Status requested (;)")
        elif ch in (ord('c'), ord('C')):
            con_mode = True
            con_sink = 0
            # Pre-select current source for the first sink
            snk_tag = SINK_NODES[con_sink]
            ridx = SINK_ROUTE_IDX[snk_tag]
            cur_port = route[ridx] if ridx < len(route) else 0
            cur_tag = PORT_TAG.get(cur_port, "adc")
            con_src = ALL_NODES.index(cur_tag) if cur_tag in ALL_NODES else 0
            set_status("Connection editor: pick sink, then source")
        elif ch in (ord('r'), ord('R')):
            set_status("Refreshed")
        elif ch in (ord('w'), ord('W')):
            raw_mode = True; raw_buf = ""; raw_field = 0
            set_status("Raw write mode...")

    link.close()


def run():
    parser = argparse.ArgumentParser(
        description="ncurses GUI for FPGA effects processor (cmd_proc_v2)")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--offline", action="store_true",
                        help="Run without serial (demo/preview mode)")
    args = parser.parse_args()

    if not HAS_SERIAL and not args.offline:
        print("pyserial not installed. Install with: pip install pyserial")
        print("Or run with --offline for preview mode.")
        sys.exit(1)

    curses.wrapper(lambda stdscr: main(stdscr, args))


if __name__ == "__main__":
    run()
