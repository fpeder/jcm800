#!/usr/bin/env python3
"""
jcm800_tui.py — three-knob curses TUI for the JCM800 FPGA build

Talks to rtl/uart_pot_regs.sv over the Arty A7 USB-UART bridge
(FT2232H interface 1, typically /dev/ttyUSB1 on Linux). Wire
protocol is two raw bytes per update:

    [0x01, pot]  → gain_pot_pos
    [0x02, pot]  → master_pot_pos
    [0x03, pot]  → presence_pot_pos
    [0x04, pot]  → bass_pot_pos
    [0x05, pot]  → mid_pot_pos
    [0x06, pot]  → treble_pot_pos

No framing, no ACK; the parser resyncs on any valid channel byte.
"""

from __future__ import annotations

import argparse
import curses
import math
import sys

import serial
from serial.tools import list_ports

CH_GAIN     = 0x01
CH_MASTER   = 0x02
CH_PRESENCE = 0x03
CH_BASS     = 0x04
CH_MID      = 0x05
CH_TREBLE   = 0x06
GAIN_DEFAULT     = 0x80     # matches uart_pot_regs.sv reset defaults
MASTER_DEFAULT   = 0x80     # matches uart_pot_regs.sv reset defaults
PRESENCE_DEFAULT = 0x80
BASS_DEFAULT     = 0x80
MID_DEFAULT      = 0x80
TREBLE_DEFAULT   = 0x80
FT2232H_VID_PID = "0403:6010"   # FTDI FT2232H on the Arty A7

MIN_PANEL_W    = 60
MIN_BAR_WIDTH  = 10
MAX_BAR_WIDTH  = 120
# Width of everything in a pot row except the bar itself:
# "  " + marker(1) + "  " + label(8) + "  " + value(3) + "  " + trailer(8) + "  "
ROW_FIXED_COLS = 32
FILL_CH   = "█"
EMPTY_CH  = "░"

# Marshall front-panel palette. 256-colour indices first, 8-colour ANSI
# fallback second, plus any extra attribute (bold/dim). Populated by
# init_palette() at run-time into PAIRS: name -> (pair_id, extra_attr).
PALETTE_SPEC = (
    # name,        id, fg_256, fg_8_fallback,       extra_attr_if_8col
    ("gold",        1, 220, "COLOR_YELLOW",         "A_BOLD"),
    ("gold_bold",   2, 220, "COLOR_YELLOW",         "A_BOLD"),
    ("cream",       3, 230, "COLOR_WHITE",          None),
    ("bar_fill",    4, 214, "COLOR_YELLOW",         "A_BOLD"),
    ("bar_empty",   5,  94, "COLOR_YELLOW",         "A_DIM"),
    ("red",         6, 196, "COLOR_RED",            "A_BOLD"),
    ("dim",         7, 240, "COLOR_WHITE",          "A_DIM"),
    ("value",       8, 255, "COLOR_WHITE",          "A_BOLD"),
    ("footer",      9, 136, "COLOR_YELLOW",         None),
)
PAIRS: dict[str, tuple[int, int]] = {}


def init_palette() -> None:
    curses.start_color()
    try:
        curses.use_default_colors()
    except curses.error:
        pass
    has_256 = curses.COLORS >= 256
    bg = curses.COLOR_BLACK
    for name, pid, fg256, fg8_name, extra_name in PALETTE_SPEC:
        fg8 = getattr(curses, fg8_name)
        fg = fg256 if has_256 else fg8
        extra = getattr(curses, extra_name) if extra_name else 0
        try:
            curses.init_pair(pid, fg, bg)
        except curses.error:
            curses.init_pair(pid, fg8, bg)
        PAIRS[name] = (pid, extra)


def attr(name: str) -> int:
    pid, extra = PAIRS[name]
    return curses.color_pair(pid) | extra


def safe_addnstr(stdscr, y: int, x: int, text: str, maxlen: int, a: int) -> None:
    if maxlen <= 0 or not text:
        return
    try:
        stdscr.addnstr(y, x, text, maxlen, a)
    except curses.error:
        pass


def pot_to_db(pot: int) -> str:
    if pot <= 0:
        return "  mute"
    if pot >= 255:
        return " +0.0 dB"
    db = (pot / 255.0 - 1.0) * 60.0
    return f"{db:+5.1f} dB"


def autodetect_port() -> str:
    matches = sorted(
        list_ports.grep(FT2232H_VID_PID),
        key=lambda p: p.device,
    )
    if not matches:
        sys.exit(
            "error: no FT2232H USB-UART found (VID:PID 0403:6010). "
            "Pass --port explicitly."
        )
    # Vivado's hw_server holds interface 0 (ttyUSB0) for JTAG; the
    # UART is exposed on the higher-numbered interface.
    return matches[-1].device


def clamp(v: int) -> int:
    return max(0, min(255, v))


def _pot_row_segments(key: str, label: str, pots: dict, focus: str,
                      bar_width: int, kind: str = "audio"):
    focused = key == focus
    marker = "●" if focused else "·"
    marker_pair = "red" if focused else "dim"
    label_pair = "gold" if focused else "cream"
    filled = round(pots[key] / 255.0 * bar_width)
    empty = bar_width - filled
    if kind == "tone":
        # Tonestack ROM is addressed by the top 3 bits of each pot
        # (8 positions per knob), so show the quantised index.
        trailer = f"pos {pots[key] >> 5}/7"
    else:
        trailer = pot_to_db(pots[key])
    return [
        ("  ", "cream"),
        (marker, marker_pair),
        ("  ", "cream"),
        (label, label_pair),
        ("  ", "cream"),
        (FILL_CH * filled, "bar_fill"),
        (EMPTY_CH * empty, "bar_empty"),
        ("  ", "cream"),
        (f"{pots[key]:3d}", "value"),
        ("  ", "cream"),
        (f"{trailer:>8}", "value"),
        ("  ", "cream"),
    ]


def draw(stdscr, port: str, baud: int, pots: dict, focus: str) -> None:
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    panel_w = max(MIN_PANEL_W, max_x - 2)
    inner_w = panel_w - 4     # │ + pad + content + pad + │
    bar_width = max(MIN_BAR_WIDTH,
                    min(MAX_BAR_WIDTH, inner_w - ROW_FIXED_COLS))

    body = [
        [(" M A R S H A L L ", "gold_bold")],
        [("JCM800 2203", "cream")],
        [(f"── {port} @ {baud} ──", "footer")],
        [("", "cream")],
        _pot_row_segments("gain",     "Gain    ", pots, focus, bar_width),
        _pot_row_segments("master",   "Master  ", pots, focus, bar_width),
        _pot_row_segments("presence", "Presence", pots, focus, bar_width),
        [("", "cream")],
        _pot_row_segments("bass",     "Bass    ", pots, focus, bar_width, kind="tone"),
        _pot_row_segments("mid",      "Middle  ", pots, focus, bar_width, kind="tone"),
        _pot_row_segments("treble",   "Treble  ", pots, focus, bar_width, kind="tone"),
        [("", "cream")],
        [(" ↑/↓ select   ←/→ ±1   ⇧←/→ ±8   q quit ", "footer")],
    ]

    def line_w(segs): return sum(len(s) for s, _ in segs)
    panel_h = max(len(body) + 2, max_y - 2)

    y0 = max(0, (max_y - panel_h) // 2)
    x0 = max(0, (max_x - panel_w) // 2)

    gold = attr("gold")
    top = "┌" + "─" * (panel_w - 2) + "┐"
    bot = "└" + "─" * (panel_w - 2) + "┘"
    safe_addnstr(stdscr, y0, x0, top, max_x - x0, gold)
    safe_addnstr(stdscr, y0 + panel_h - 1, x0, bot, max_x - x0, gold)
    for i in range(1, panel_h - 1):
        safe_addnstr(stdscr, y0 + i, x0, "│", max_x - x0, gold)
        safe_addnstr(stdscr, y0 + i, x0 + panel_w - 1, "│",
                     max_x - (x0 + panel_w - 1), gold)

    for idx, segs in enumerate(body):
        y = y0 + 1 + idx
        pad_left = (inner_w - line_w(segs)) // 2
        x = x0 + 2 + pad_left
        for text, pname in segs:
            if not text:
                continue
            safe_addnstr(stdscr, y, x, text, max(0, max_x - x), attr(pname))
            x += len(text)

    stdscr.refresh()


def send(ser: serial.Serial, channel: int, pot: int) -> None:
    ser.write(bytes([channel, pot]))
    ser.flush()


def run(stdscr, ser: serial.Serial, port: str, baud: int, pots: dict) -> None:
    curses.curs_set(0)
    stdscr.keypad(True)
    init_palette()
    stdscr.bkgd(" ", attr("cream"))

    # Sync the board to whatever the TUI thinks the values are.
    send(ser, CH_GAIN,     pots["gain"])
    send(ser, CH_MASTER,   pots["master"])
    send(ser, CH_PRESENCE, pots["presence"])
    send(ser, CH_BASS,     pots["bass"])
    send(ser, CH_MID,      pots["mid"])
    send(ser, CH_TREBLE,   pots["treble"])

    order = ["gain", "master", "presence", "bass", "mid", "treble"]
    focus = "gain"
    draw(stdscr, port, baud, pots, focus)

    channel_of = {
        "gain":     CH_GAIN,
        "master":   CH_MASTER,
        "presence": CH_PRESENCE,
        "bass":     CH_BASS,
        "mid":      CH_MID,
        "treble":   CH_TREBLE,
    }

    while True:
        ch = stdscr.getch()
        prev = pots[focus]

        if ch in (ord("q"), ord("Q"), 27):          # 27 = ESC
            return
        elif ch == curses.KEY_UP:
            focus = order[max(0, order.index(focus) - 1)]
        elif ch == curses.KEY_DOWN:
            focus = order[min(len(order) - 1, order.index(focus) + 1)]
        elif ch == curses.KEY_LEFT:
            pots[focus] = clamp(pots[focus] - 1)
        elif ch == curses.KEY_RIGHT:
            pots[focus] = clamp(pots[focus] + 1)
        elif ch == curses.KEY_SLEFT:
            pots[focus] = clamp(pots[focus] - 8)
        elif ch == curses.KEY_SRIGHT:
            pots[focus] = clamp(pots[focus] + 8)
        elif ch == curses.KEY_HOME:
            pots[focus] = 0
        elif ch == curses.KEY_END:
            pots[focus] = 255
        elif ch == curses.KEY_RESIZE:
            pass
        else:
            continue

        if pots[focus] != prev:
            send(ser, channel_of[focus], pots[focus])
        draw(stdscr, port, baud, pots, focus)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("--port", help="serial device (default: auto-detect FT2232H)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--gain",     type=lambda s: int(s, 0), default=GAIN_DEFAULT,
                    help=f"initial gain pot 0..255 (default 0x{GAIN_DEFAULT:02X})")
    ap.add_argument("--master",   type=lambda s: int(s, 0), default=MASTER_DEFAULT,
                    help=f"initial master pot 0..255 (default 0x{MASTER_DEFAULT:02X})")
    ap.add_argument("--presence", type=lambda s: int(s, 0), default=PRESENCE_DEFAULT,
                    help=f"initial presence pot 0..255 (default 0x{PRESENCE_DEFAULT:02X})")
    ap.add_argument("--bass",     type=lambda s: int(s, 0), default=BASS_DEFAULT,
                    help=f"initial bass pot 0..255 (default 0x{BASS_DEFAULT:02X})")
    ap.add_argument("--mid",      type=lambda s: int(s, 0), default=MID_DEFAULT,
                    help=f"initial mid pot 0..255 (default 0x{MID_DEFAULT:02X})")
    ap.add_argument("--treble",   type=lambda s: int(s, 0), default=TREBLE_DEFAULT,
                    help=f"initial treble pot 0..255 (default 0x{TREBLE_DEFAULT:02X})")
    args = ap.parse_args()

    for name, val in (("gain",     args.gain),
                      ("master",   args.master),
                      ("presence", args.presence),
                      ("bass",     args.bass),
                      ("mid",      args.mid),
                      ("treble",   args.treble)):
        if not 0 <= val <= 255:
            sys.exit(f"error: --{name} must be in 0..255, got {val}")

    port = args.port or autodetect_port()
    pots = {
        "gain":     args.gain,
        "master":   args.master,
        "presence": args.presence,
        "bass":     args.bass,
        "mid":      args.mid,
        "treble":   args.treble,
    }

    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {port}: {e}")

    try:
        curses.wrapper(run, ser, port, args.baud, pots)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
