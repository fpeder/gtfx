#!/usr/bin/env python3
"""
efx_control.py - ncurses GUI for FPGA guitar effects command processor (cmd_proc_v2)

Sends commands over serial to set effect parameters, toggle bypass, and view status.

Usage:
    python3 efx_control.py [--port /dev/ttyUSB0] [--baud 115200]

Keys:
    TAB / Shift-TAB   Move between effect slots
    UP / DOWN         Move between parameters within a slot
    LEFT / RIGHT      Adjust value (-1 / +1)
    SHIFT+LEFT/RIGHT  Adjust value (-16 / +16)  (PgDn/PgUp also work)
    SPACE / ENTER     Toggle effect bypass on/off
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

# ---------------------------------------------------------------------------
# Try to import pyserial; provide a stub for dry-run / demo mode
# ---------------------------------------------------------------------------
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False


# ═══════════════════════════════════════════════════════════════════════════════
# Data model - mirrors the cmd_proc_v2 address map exactly
# ═══════════════════════════════════════════════════════════════════════════════

class Param:
    """Single register parameter."""
    def __init__(self, name, label, addr, default, lo=0x00, hi=0xFF, fmt="hex"):
        self.name = name        # 3-char CLI name (e.g. "rat")
        self.label = label      # Human-readable label
        self.addr = addr        # Bus address
        self.value = default
        self.default = default
        self.lo = lo
        self.hi = hi
        self.fmt = fmt          # "hex" or "int"


class Param16:
    """16-bit LE parameter (delay time)."""
    def __init__(self, name, label, addr, default, lo=0x0000, hi=0xFFFF):
        self.name = name
        self.label = label
        self.addr = addr
        self.value = default
        self.default = default
        self.lo = lo
        self.hi = hi
        self.fmt = "hex16"


class EffectSlot:
    """One effect pedal with its parameters and bypass state."""
    def __init__(self, tag, full_name, params, bypass_addr, bypass_default=0x01):
        self.tag = tag              # 3-char CLI tag (e.g. "trm")
        self.full_name = full_name
        self.params = params        # list of Param / Param16
        self.bypass_addr = bypass_addr
        self.bypassed = bool(bypass_default)
        self.selected_param = 0


def build_effects():
    """Build the effect chain in default route order: ADC→FLN→TUB→TRM→PHA→CHO→DLY→DAC."""
    slots = [
        EffectSlot("fln", "Flanger", [
            Param("man", "Manual",  0x28, 0x80),
            Param("wid", "Width",   0x29, 0xC0),
            Param("spd", "Speed",   0x2A, 0x30),
            Param("reg", "Regen",   0x2B, 0xA0),
        ], bypass_addr=0x2F),

        EffectSlot("tub", "Tube Distortion", [
            Param("gai", "Gain",        0x20, 0x40),
            Param("bas", "Tone Bass",   0x21, 0x80),
            Param("mid", "Tone Mid",    0x22, 0x80),
            Param("tre", "Tone Treble", 0x23, 0x80),
            Param("lvl", "Level",       0x24, 0xA0),
        ], bypass_addr=0x27),

        EffectSlot("trm", "Tremolo", [
            Param("rat", "Rate",  0x00, 0x3C),
            Param("dep", "Depth", 0x01, 0xB4),
            Param("shp", "Shape", 0x02, 0x00),
        ], bypass_addr=0x07),

        EffectSlot("pha", "Phaser", [
            Param("spd", "Speed",       0x08, 0x50),
            Param("fbn", "Feedback En", 0x09, 0x00),
        ], bypass_addr=0x0F),

        EffectSlot("cho", "Chorus", [
            Param("rat", "Rate",    0x10, 0x50),
            Param("dep", "Depth",   0x11, 0x64),
            Param("efx", "Effect",  0x12, 0x80),
            Param("eqh", "EQ High", 0x13, 0xC8),
            Param("eql", "EQ Low",  0x14, 0x80),
        ], bypass_addr=0x17),

        EffectSlot("dly", "Delay (DD3)", [
            Param("ton",   "Tone",     0x18, 0xFF),
            Param("lvl",   "Level",    0x19, 0x80),
            Param("fdb",   "Feedback", 0x1A, 0x64),
            Param16("tim", "Time",     0x1B, 0x3000),
        ], bypass_addr=0x1F),
    ]
    return slots


# ═══════════════════════════════════════════════════════════════════════════════
# Serial interface
# ═══════════════════════════════════════════════════════════════════════════════

class SerialLink:
    """Wraps pyserial with command helpers matching cmd_proc_v2 CLI protocol."""

    def __init__(self, port, baud, timeout=0.5):
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()
        self.rx_log = []           # last N lines received
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
        """Send a command string followed by CR."""
        with self.lock:
            if not self.connected:
                self.rx_log.append(f"[offline] {cmd_str}")
                return ""
            full = cmd_str + "\r"
            self.ser.write(full.encode("ascii"))
            time.sleep(0.05)
            resp = ""
            try:
                resp = self.ser.read(self.ser.in_waiting or 1).decode("ascii", errors="replace")
            except Exception:
                pass
            if resp:
                self.rx_log.append(resp.strip())
            # Keep log bounded
            if len(self.rx_log) > 200:
                self.rx_log = self.rx_log[-100:]
            return resp

    def set_param(self, slot_tag, param_name, value):
        """Send: set <efx> <prm> <hex>"""
        cmd = f"set {slot_tag} {param_name} {value:02X}"
        return self._send_cmd(cmd)

    def set_param16(self, slot_tag, param_name, value):
        """Send: set <efx> <prm> <hex16> (e.g. delay time)."""
        cmd = f"set {slot_tag} {param_name} {value:04X}"
        return self._send_cmd(cmd)

    def set_bypass(self, slot_tag, active):
        """Send: set <efx> on/off"""
        state = "on" if active else "off"
        cmd = f"set {slot_tag} {state}"
        return self._send_cmd(cmd)

    def raw_write(self, addr, data):
        """Send: wr <addr><data>"""
        cmd = f"wr {addr:02X}{data:02X}"
        return self._send_cmd(cmd)

    def request_status(self):
        """Send ';' to trigger status dump."""
        with self.lock:
            if not self.connected:
                self.rx_log.append("[offline] status request")
                return ""
            self.ser.write(b";")
            time.sleep(0.15)
            resp = ""
            try:
                resp = self.ser.read(self.ser.in_waiting or 1).decode("ascii", errors="replace")
            except Exception:
                pass
            if resp:
                for line in resp.split("\n"):
                    stripped = line.strip()
                    if stripped:
                        self.rx_log.append(stripped)
            if len(self.rx_log) > 200:
                self.rx_log = self.rx_log[-100:]
            return resp


# ═══════════════════════════════════════════════════════════════════════════════
# ncurses GUI  (scrollable single-column, compact width)
# ═══════════════════════════════════════════════════════════════════════════════

CP_NORMAL    = 1
CP_HEADER    = 2
CP_ACTIVE    = 3
CP_BYPASSED  = 4
CP_SELECTED  = 5
CP_BAR       = 6
CP_TITLE     = 7
CP_LOG       = 8
CP_HELP      = 9
CP_WARN      = 10

SLOT_W = 36          # fixed compact width


def draw_bar(win, y, x, value, max_val, width, color_pair):
    """Draw a visual bar ▓▓▓▓░░░░ for a parameter value."""
    filled = int((value / max(max_val, 1)) * width)
    filled = max(0, min(filled, width))
    try:
        win.addstr(y, x, "▓" * filled, curses.color_pair(color_pair) | curses.A_BOLD)
        win.addstr("░" * (width - filled), curses.color_pair(CP_NORMAL))
    except curses.error:
        pass


def draw_slot(pad, slot, y, is_active_slot):
    """Draw one compact effect slot panel onto a pad. Returns next y."""
    w = SLOT_W
    x = 0

    # Colours
    if is_active_slot:
        hdr_attr = curses.color_pair(CP_HEADER) | curses.A_BOLD
    elif slot.bypassed:
        hdr_attr = curses.color_pair(CP_BYPASSED)
    else:
        hdr_attr = curses.color_pair(CP_ACTIVE) | curses.A_BOLD

    byp_attr = (curses.color_pair(CP_ACTIVE) | curses.A_BOLD) if not slot.bypassed \
               else curses.color_pair(CP_WARN)

    bypass_tag = " ON" if not slot.bypassed else "OFF"
    name = slot.full_name.upper()

    # Header: ┌ NAME              ON ┐
    try:
        inner = w - 2
        name_field = inner - 4  # 4 = bypass_tag(3) + space(1)
        hdr_text = f" {name:<{name_field}}{bypass_tag} "
        pad.addstr(y, x, "+" + "-" * (w - 2) + "+", hdr_attr)
        pad.addstr(y, x + 1, hdr_text[:inner], hdr_attr)
        # overwrite bypass portion with bypass colour
        byp_start = x + 1 + name_field
        pad.addstr(y, byp_start, f"{bypass_tag} ", byp_attr)
    except curses.error:
        pass
    y += 1

    # Parameters
    # Layout per line: |>Label     XX ########|
    #                  1+1+10+1+4+1+bar+1 = w
    # bar_w = w - 19
    LABEL_W = 10
    VAL_W = 4
    bar_w = w - 2 - 1 - LABEL_W - 1 - VAL_W - 1 - 1  # = w - 20
    if bar_w < 2:
        bar_w = 2

    for i, p in enumerate(slot.params):
        is_sel = is_active_slot and i == slot.selected_param
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

        # Build the full line as a fixed-width string
        body = f"{mk}{p.label:<{LABEL_W}} {val_str} "
        # We'll write body, then the bar, then pad to border
        try:
            pad.addstr(y, x, "|", hdr_attr if not is_sel else attr)
            pad.addstr(body, attr)
            # draw bar
            bar_x = x + 1 + len(body)
            draw_bar(pad, y, bar_x, p.value, p.hi, bar_w,
                     CP_SELECTED if is_sel else (CP_BAR if not slot.bypassed else CP_BYPASSED))
            # right border
            pad.addstr(y, x + w - 1, "|", hdr_attr if not is_sel else attr)
        except curses.error:
            pass
        y += 1

    # Bottom border
    try:
        pad.addstr(y, x, "+" + "-" * (w - 2) + "+", hdr_attr)
    except curses.error:
        pass
    y += 1
    return y


def calc_total_height(slots):
    """Calculate total pad height needed for all slots."""
    h = 0
    for s in slots:
        h += 1 + len(s.params) + 1   # header(1) + params + bottom(1)
    return h


def compute_scroll(slots, active_slot, viewport_h, scroll_y):
    """Ensure the active slot is visible; return adjusted scroll_y."""
    y = 0
    for idx, s in enumerate(slots):
        slot_h = 1 + len(s.params) + 1  # header + params + bottom
        if idx == active_slot:
            slot_top = y
            slot_bot = y + slot_h
            # Adjust scroll so the full slot is visible
            if slot_top < scroll_y:
                scroll_y = slot_top
            if slot_bot > scroll_y + viewport_h:
                scroll_y = slot_bot - viewport_h
            break
        y += slot_h
    return max(0, scroll_y)


def main(stdscr, args):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(100)
    curses.start_color()
    curses.use_default_colors()

    curses.init_pair(CP_NORMAL,   curses.COLOR_WHITE,   -1)
    curses.init_pair(CP_HEADER,   curses.COLOR_BLACK,   curses.COLOR_CYAN)
    curses.init_pair(CP_ACTIVE,   curses.COLOR_GREEN,   -1)
    curses.init_pair(CP_BYPASSED, curses.COLOR_BLACK,   -1)
    curses.init_pair(CP_SELECTED, curses.COLOR_BLACK,   curses.COLOR_YELLOW)
    curses.init_pair(CP_BAR,      curses.COLOR_CYAN,    -1)
    curses.init_pair(CP_TITLE,    curses.COLOR_BLACK,   curses.COLOR_WHITE)
    curses.init_pair(CP_LOG,      curses.COLOR_BLUE,    -1)
    curses.init_pair(CP_HELP,     curses.COLOR_YELLOW,  -1)
    curses.init_pair(CP_WARN,     curses.COLOR_RED,     -1)

    slots = build_effects()
    active_slot = 0
    scroll_y = 0

    link = SerialLink(args.port, args.baud)
    if not args.offline:
        if link.open():
            link.rx_log.append(f"Connected: {args.port} @ {args.baud}")
        else:
            link.rx_log.append(f"FAILED to open {args.port} - running offline")
    else:
        link.rx_log.append("Offline mode (--offline)")

    status_msg = ""
    status_time = 0
    raw_mode = False
    raw_buf = ""
    raw_field = 0

    def set_status(msg):
        nonlocal status_msg, status_time
        status_msg = msg
        status_time = time.time()

    def send_current_param():
        s = slots[active_slot]
        p = s.params[s.selected_param]
        if p.fmt == "hex16":
            link.set_param16(s.tag, p.name, p.value)
        else:
            link.set_param(s.tag, p.name, p.value)
        set_status(f"set {s.tag} {p.name} {p.value:02X}" if p.fmt != "hex16"
                   else f"set {s.tag} {p.name} {p.value:04X}")

    def toggle_bypass():
        s = slots[active_slot]
        s.bypassed = not s.bypassed
        link.set_bypass(s.tag, not s.bypassed)
        state = "ON" if not s.bypassed else "OFF"
        set_status(f"{s.full_name}: {state}")

    # ── Main loop ──
    while True:
        max_y, max_x = stdscr.getmaxyx()
        stdscr.erase()

        # ── Fixed rows: title (0), chain (1), separator (2) ──
        FIXED_TOP = 3        # rows reserved at top
        FIXED_BOT = 2        # help bar + status line
        viewport_h = max_y - FIXED_TOP - FIXED_BOT
        if viewport_h < 3:
            viewport_h = 3

        # Title bar
        title = " EFX Control │ cmd_proc_v2 "
        conn = f" {args.port}@{args.baud}" if link.connected else " OFFLINE"
        tline = f"{title}{conn:>{max_x - len(title) - 1}}"
        try:
            stdscr.addstr(0, 0, tline[:max_x-1].ljust(max_x-1),
                          curses.color_pair(CP_TITLE))
        except curses.error:
            pass

        # Chain
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

        # ── Build pad with all slots ──
        total_h = calc_total_height(slots)
        pad_h = max(total_h + 2, viewport_h + 2)
        pad_w = SLOT_W + 2
        pad = curses.newpad(pad_h, pad_w)

        py = 0
        for idx, slot in enumerate(slots):
            py = draw_slot(pad, slot, py, idx == active_slot)

        # Auto-scroll to keep active slot in view
        scroll_y = compute_scroll(slots, active_slot, viewport_h, scroll_y)

        # Blit pad into viewport (done after stdscr.noutrefresh below)
        blit_bottom = min(FIXED_TOP + viewport_h - 1, max_y - FIXED_BOT - 1)
        blit_right = min(1 + pad_w - 1, max_x - 1)

        # ── Scroll indicator (written to stdscr) ──
        if total_h > viewport_h:
            try:
                stdscr.addstr(FIXED_TOP, SLOT_W + 3, "^" if scroll_y > 0 else " ",
                              curses.color_pair(CP_HELP))
                stdscr.addstr(FIXED_TOP + viewport_h - 1, SLOT_W + 3,
                              "v" if scroll_y + viewport_h < total_h else " ",
                              curses.color_pair(CP_HELP))
            except curses.error:
                pass

        # ── Status line ──
        status_y = max_y - 2
        if status_msg and (time.time() - status_time < 4.0):
            try:
                stdscr.addstr(status_y, 1, f" {status_msg[:max_x-3]} ",
                              curses.color_pair(CP_HELP))
            except curses.error:
                pass
        elif link.rx_log:
            try:
                stdscr.addstr(status_y, 1, link.rx_log[-1][:max_x-3],
                              curses.color_pair(CP_LOG))
            except curses.error:
                pass

        # ── Help bar ──
        if raw_mode:
            if raw_field == 0:
                hstr = f" RAW | Addr: {raw_buf}_ | 2 hex, Enter=next, ESC=cancel "
            else:
                hstr = f" RAW | {raw_buf[:2]}:{raw_buf[2:]}_ | 2 hex, Enter=send, ESC=cancel "
        else:
            hstr = " TAB:slot ^v:prm <>:+-1 PgU/D:+-16 SPC:byp o:ON f:OFF s:stat w:raw q:quit "
        try:
            stdscr.addstr(max_y - 1, 0, hstr[:max_x-1].ljust(max_x-1),
                          curses.color_pair(CP_TITLE))
        except curses.error:
            pass

        # ── Refresh: stdscr FIRST, then pad overlay, then
        #    doupdate to push both to the physical screen ──
        stdscr.noutrefresh()
        try:
            pad.noutrefresh(scroll_y, 0,
                            FIXED_TOP, 1,
                            blit_bottom, blit_right)
        except curses.error:
            pass
        curses.doupdate()

        # ── Input ──
        ch = stdscr.getch()
        if ch == -1:
            continue

        if raw_mode:
            if ch == 27:
                raw_mode = False; raw_buf = ""; raw_field = 0
                set_status("Raw write cancelled")
            elif ch in (curses.KEY_ENTER, 10, 13):
                if len(raw_buf) == 4:
                    try:
                        a = int(raw_buf[:2], 16); d = int(raw_buf[2:], 16)
                        link.raw_write(a, d)
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
                    if len(raw_buf) < 2: raw_field = 0
            else:
                c = chr(ch) if 0 <= ch < 256 else ""
                if c in "0123456789abcdefABCDEF" and len(raw_buf) < 4:
                    raw_buf += c.upper()
            continue

        if ch in (ord('q'), ord('Q'), 27):
            break

        elif ch == ord('\t'):
            active_slot = (active_slot + 1) % len(slots)
        elif ch == curses.KEY_BTAB:
            active_slot = (active_slot - 1) % len(slots)

        elif ch == curses.KEY_UP:
            s = slots[active_slot]
            if s.selected_param == 0:
                # wrap to previous slot's last param
                active_slot = (active_slot - 1) % len(slots)
                slots[active_slot].selected_param = len(slots[active_slot].params) - 1
            else:
                s.selected_param -= 1
        elif ch == curses.KEY_DOWN:
            s = slots[active_slot]
            if s.selected_param == len(s.params) - 1:
                # wrap to next slot's first param
                active_slot = (active_slot + 1) % len(slots)
                slots[active_slot].selected_param = 0
            else:
                s.selected_param += 1

        elif ch == curses.KEY_RIGHT:
            s = slots[active_slot]; p = s.params[s.selected_param]
            step = 0x100 if p.fmt == "hex16" else 1
            p.value = min(p.value + step, p.hi); send_current_param()
        elif ch == curses.KEY_LEFT:
            s = slots[active_slot]; p = s.params[s.selected_param]
            step = 0x100 if p.fmt == "hex16" else 1
            p.value = max(p.value - step, p.lo); send_current_param()

        elif ch == curses.KEY_NPAGE:
            s = slots[active_slot]; p = s.params[s.selected_param]
            step = 0x1000 if p.fmt == "hex16" else 0x10
            p.value = max(p.value - step, p.lo); send_current_param()
        elif ch == curses.KEY_PPAGE:
            s = slots[active_slot]; p = s.params[s.selected_param]
            step = 0x1000 if p.fmt == "hex16" else 0x10
            p.value = min(p.value + step, p.hi); send_current_param()

        elif ch in (ord(' '),):
            toggle_bypass()
        elif ch in (ord('o'), ord('O')):
            s = slots[active_slot]
            if s.bypassed:
                s.bypassed = False; link.set_bypass(s.tag, True)
                set_status(f"{s.full_name}: ON")
            else:
                set_status(f"{s.full_name}: already ON")
        elif ch in (ord('f'), ord('F')):
            s = slots[active_slot]
            if not s.bypassed:
                s.bypassed = True; link.set_bypass(s.tag, False)
                set_status(f"{s.full_name}: OFF")
            else:
                set_status(f"{s.full_name}: already OFF")
        elif ch in (curses.KEY_ENTER, 10, 13):
            toggle_bypass()

        elif ch in (ord('s'), ord('S')):
            link.request_status(); set_status("Status requested (;)")
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
