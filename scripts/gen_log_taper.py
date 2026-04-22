#!/usr/bin/env python3
"""
gen_log_taper.py — 256-entry audio-taper coefficient LUT

Produces lut_out/log_taper_256.mem for rtl/level_ctrl.sv.  One entry per
pot position (0..255).  Format: 24-bit unsigned Q0.24 hex, one per line,
loadable with $readmemh.

Curve: pure audio (log) taper attenuator, span −60..0 dB.

    dB(i)      = (i/255 − 1) · 60           # i=0 → −60 dB, i=255 → 0 dB
    gain(i)    = 10^(dB(i) / 20)
    coef(i)    = round(gain(i) · 2^24)      # Q0.24 in a 24-bit word

Endpoints are forced for clean behaviour:
    coef[0]   = 0x000000   (true mute)
    coef[255] = 0xFFFFFF   (1.0 − 1 ulp, clamped from 2^24)

The RTL multiplier treats the 24-bit LUT value as unsigned, sign-extends
for the signed×unsigned product, and right-shifts by 24 after rounding.
"""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

N          = 256                               # pot positions (8-bit)
DB_MIN     = -60.0                             # dB at pot_pos = 0
DB_MAX     =   0.0                             # dB at pot_pos = 255
Q_FRAC     = 24                                # Q0.24
SCALE      = 1 << Q_FRAC                       # 2^24
OUT_PATH   = Path(__file__).resolve().parent.parent / "lut_out" / "log_taper_256.mem"


def taper_db(i: int) -> float:
    """Linear-in-dB mapping from pot index to attenuation."""
    return DB_MIN + (DB_MAX - DB_MIN) * (i / (N - 1))


def coef_q024(i: int) -> int:
    if i == 0:
        return 0
    if i == N - 1:
        return SCALE - 1
    g = 10.0 ** (taper_db(i) / 20.0)
    v = round(g * SCALE)
    return max(0, min(SCALE - 1, v))


def main() -> None:
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"// JCM800 log-taper pot LUT — {N} entries × 24-bit Q0.24")
    lines.append(f"// Generated: {stamp}  by scripts/gen_log_taper.py")
    lines.append(f"// Span:      {DB_MIN:.1f} dB (pot=0) .. {DB_MAX:.1f} dB (pot={N - 1})")
    lines.append( "// Curve:     linear-in-dB (audio/log taper)")
    lines.append( "// Spot:")
    for i in (0, 32, 64, 96, 128, 160, 192, 224, 255):
        c = coef_q024(i)
        g = c / SCALE if c else 0.0
        db = -math.inf if c == 0 else 20.0 * math.log10(g)
        db_s = f"{db:+6.2f} dB" if math.isfinite(db) else "  mute "
        lines.append(f"//   pot={i:3d}  coef=0x{c:06X}  gain={g:.6f}  {db_s}")
    lines.append("")

    for i in range(N):
        lines.append(f"{coef_q024(i):06x}")

    OUT_PATH.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_PATH} ({N} entries)")


if __name__ == "__main__":
    main()
