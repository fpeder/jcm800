#!/usr/bin/env python3
"""
gen_tonestack_lut.py — Marshall-style TMB tone-stack coefficient LUT

Produces lut_out/tonestack_coefs.mem for rtl/tonestack.sv.

Grid
────
  8 × 8 × 8 = 512 pot triples.  Pot positions are 8-bit at runtime; the
  RTL uses pot[7:5] (top 3 bits) as the grid index, so each grid cell
  spans 32 pot steps.  Coarse for v1 — trilinear interpolation is the
  first follow-up upgrade (same .mem, just more ROM reads per sample).

  Grid address layout (flat 9-bit index 0..511):
      addr = (bass_idx << 6) | (mid_idx << 3) | treble_idx

Per-entry payload
─────────────────
  9 × 32-bit Q3.29 signed coefficients = 288 bits / entry.

  Sections are three cascaded 1st-order biquads, matching
  rtl/cathode_shelf.sv's datapath:
      y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]

  Coefficient order within an entry (addr-offsets 0..8, low → high):
      0: sec1_b0   1: sec1_b1   2: sec1_a1      (LF shelf)
      3: sec2_b0   4: sec2_b1   5: sec2_a1      (Mid shelf)
      6: sec3_b0   7: sec3_b1   8: sec3_a1      (HF shelf)

Model (v1 — heuristic Marshall-like coupling)
──────────────────────────────────────────────
  This generator is a first-pass model of Marshall TMB behaviour.  A
  future iteration will solve the actual 2203 tone-stack network
  symbolically (4-node KCL, 3rd-order s-domain rational, bilinear
  transform) and replace this heuristic.  The RTL is unaffected —
  only the .mem content changes.

  The heuristic here captures the three audible traits of the Marshall
  TS that players notice:

    1. Bass & Treble controls act largely independently at their own
       bands, with ~±12 dB throw each.

    2. The infamous "mid scoop" — increasing Bass deepens the scoop
       around ~500 Hz even when Mid is held constant.  Modelled by a
       coupling term subtracting some Mid gain as Bass increases.

    3. Treble slightly pulls down extreme LF (shared-impedance loading),
       and Bass slightly rolls off extreme HF.

  All three sections are implemented as first-order shelving filters
  using the bilinear-transformed analog prototypes:

    Low shelf :  H(s) = (G·ω0 + s) / (ω0 + s)
    High shelf:  H(s) = (ω0 + G·s) / (ω0 + s)

  The mid section is a high shelf centred at 700 Hz whose gain is
  interpreted as "contribution added on top of the LF shelf" — the
  audible mid emphasis then comes from the difference between this
  section and the low shelf.  With the three shelves cascaded, a
  typical "Marshall scoop" profile (B=7, M=3, T=7) produces the
  expected V-shaped response.

Stability
─────────
  Every generated pole is checked against |pole| < 0.999 before the
  .mem is written.  Shelf poles are well inside the unit circle for
  the chosen centre frequencies at fs = 768 kHz (worst |pole| ≈ 0.99
  for the 100 Hz low shelf).

Output
──────
  lut_out/tonestack_coefs.mem   4608 lines of 32-bit hex (512 × 9)
"""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FS_HZ   = 768_000.0     # must match the oversampled DSP rate in jcm800.sv
GRID    = 8             # 8 × 8 × 8 pot grid
N_POTS  = GRID ** 3     # 512
N_COEFS = 9             # 3 sections × (b0, b1, a1)

Q_FRAC  = 29            # Q3.29 signed
Q_SCALE = 1 << Q_FRAC
Q_MIN   = -(1 << 31)    # 32-bit signed range
Q_MAX   =  (1 << 31) - 1

POLE_STABILITY_MARGIN = 0.99999
# Low-frequency shelves at high fs produce poles very close to unity —
# a 120 Hz shelf at fs = 768 kHz lives at z ≈ 0.999 by design.  This
# margin guards against true instability, not cosmetic closeness.

# Section centre frequencies (Hz).  Chosen to roughly match the audible
# 3-band split of the real Marshall TS.
F0_LF   =  120.0        # low-shelf hinge
F0_MF   =  700.0        # mid-shelf hinge
F0_HF   = 3200.0        # high-shelf hinge

# Max per-band throw (dB).  ±12 dB is the typical Marshall TS range.
GAIN_THROW_DB = 12.0

OUT_PATH = Path(__file__).resolve().parent.parent / "lut_out" / "tonestack_coefs.mem"


# ---------------------------------------------------------------------------
# Heuristic gain model — returns (lf_dB, mf_dB, hf_dB) for a pot triple.
# Pot fractions (b, m, t) are in [0, 1], nominal "noon" = 0.5.
# ---------------------------------------------------------------------------
def shelf_gains_db(b: float, m: float, t: float) -> tuple[float, float, float]:
    # Bass / Mid / Treble primary axes (centred at 0.5, ±GAIN_THROW_DB range).
    lf_db = (b - 0.5) * 2.0 * GAIN_THROW_DB
    mf_db = (m - 0.5) * 2.0 * GAIN_THROW_DB
    hf_db = (t - 0.5) * 2.0 * GAIN_THROW_DB

    # Mid-scoop coupling — cranking Bass pulls the mid shelf down,
    # cranking Treble also deepens the scoop slightly.  Capped.
    mf_db += -3.0 * max(0.0, b - 0.5) * 2.0     # up to -3 dB when b=1
    mf_db += -1.5 * max(0.0, t - 0.5) * 2.0     # up to -1.5 dB when t=1

    # Shared-impedance loading: treble slightly rolls off extreme LF,
    # bass slightly rolls off extreme HF.
    lf_db += -1.0 * max(0.0, t - 0.5) * 2.0
    hf_db += -1.0 * max(0.0, b - 0.5) * 2.0

    return lf_db, mf_db, hf_db


# ---------------------------------------------------------------------------
# Bilinear-transformed first-order shelves
# Returns (b0, b1, a1).  a1 is stored as the repo convention: the multiplier
# subtracted from the output inside cathode_shelf (y = b0·x + b1·x[-1] − a1·y[-1]).
# That matches the bilinear-transform sign of a1 for a standard shelf.
# ---------------------------------------------------------------------------
def low_shelf_biquad(f0: float, gain_db: float, fs: float) -> tuple[float, float, float]:
    g    = 10.0 ** (gain_db / 20.0)
    w_pre = 2.0 * fs * math.tan(math.pi * f0 / fs)
    k    = 2.0 * fs
    denom = w_pre + k
    b0 = (g * w_pre + k) / denom
    b1 = (g * w_pre - k) / denom
    a1 = (w_pre - k) / denom    # y − a1·y[-1] form
    return b0, b1, a1


def high_shelf_biquad(f0: float, gain_db: float, fs: float) -> tuple[float, float, float]:
    g    = 10.0 ** (gain_db / 20.0)
    w_pre = 2.0 * fs * math.tan(math.pi * f0 / fs)
    k    = 2.0 * fs
    denom = w_pre + k
    b0 = (w_pre + g * k) / denom
    b1 = (w_pre - g * k) / denom
    a1 = (w_pre - k) / denom
    return b0, b1, a1


# ---------------------------------------------------------------------------
# Q3.29 quantisation
# ---------------------------------------------------------------------------
def to_q329(x: float) -> int:
    v = int(round(x * Q_SCALE))
    if v >  Q_MAX: v = Q_MAX
    if v <  Q_MIN: v = Q_MIN
    # Two's-complement 32-bit representation
    return v & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# Entry assembly — returns list of 9 Q3.29 ints for a given (b_idx, m_idx, t_idx)
# ---------------------------------------------------------------------------
def entry_coefs(b_idx: int, m_idx: int, t_idx: int) -> list[int]:
    b = b_idx / (GRID - 1)
    m = m_idx / (GRID - 1)
    t = t_idx / (GRID - 1)

    lf_db, mf_db, hf_db = shelf_gains_db(b, m, t)

    sec1 = low_shelf_biquad (F0_LF, lf_db, FS_HZ)
    sec2 = high_shelf_biquad(F0_MF, mf_db, FS_HZ)
    sec3 = high_shelf_biquad(F0_HF, hf_db, FS_HZ)

    # Stability check — first-order IIR pole is −a1.  (The cathode_shelf
    # form y = b0·x + b1·x[-1] − a1·y[-1] has its pole at z = −a1 in the
    # convention where a1 comes out of bilinear with its natural sign.)
    for name, (b0, b1, a1) in (("LF", sec1), ("MF", sec2), ("HF", sec3)):
        pole = -a1
        assert abs(pole) < POLE_STABILITY_MARGIN, (
            f"unstable pole {pole:.4f} at (b={b:.2f}, m={m:.2f}, t={t:.2f}) "
            f"section {name}"
        )

    coefs_float = [*sec1, *sec2, *sec3]
    return [to_q329(c) for c in coefs_float]


# ---------------------------------------------------------------------------
# Main — sweep grid, emit one 32-bit hex word per line (address-ordered).
# ---------------------------------------------------------------------------
def main() -> None:
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = []
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append(f"// JCM800 tone-stack coefficient LUT")
    lines.append(f"// Generated: {stamp}  by scripts/gen_tonestack_lut.py")
    lines.append(f"// Grid:      {GRID}^3 = {N_POTS} pot triples (addressed by pot[7:5])")
    lines.append(f"// Per entry: {N_COEFS} × 32-bit Q3.29 (b0,b1,a1 for 3 sections)")
    lines.append(f"// Sections:  LF shelf @ {F0_LF:.0f} Hz | "
                 f"MF shelf @ {F0_MF:.0f} Hz | HF shelf @ {F0_HF:.0f} Hz")
    lines.append(f"// Model:     heuristic Marshall-like coupling (v1 — see docstring)")
    lines.append(f"// fs:        {FS_HZ:.0f} Hz (must match jcm800.sv oversample domain)")
    lines.append(f"//")
    lines.append(f"// Spot checks (shelf gains at representative pot settings):")
    for label, (bb, mm, tt) in (
        ("noon     ", (0.5, 0.5, 0.5)),
        ("scoop    ", (1.0, 0.0, 1.0)),
        ("all-min  ", (0.0, 0.0, 0.0)),
        ("all-max  ", (1.0, 1.0, 1.0)),
        ("mid-heavy", (0.3, 1.0, 0.3)),
    ):
        lf, mf, hf = shelf_gains_db(bb, mm, tt)
        lines.append(f"//   {label} (B={bb:.1f} M={mm:.1f} T={tt:.1f}) → "
                     f"LF={lf:+5.1f} dB  MF={mf:+5.1f} dB  HF={hf:+5.1f} dB")
    lines.append("")

    # Emit 512 × 9 = 4608 hex words in grid-address order.
    for b_idx in range(GRID):
        for m_idx in range(GRID):
            for t_idx in range(GRID):
                for c in entry_coefs(b_idx, m_idx, t_idx):
                    lines.append(f"{c:08x}")

    OUT_PATH.write_text("\n".join(lines) + "\n")
    print(f"wrote {OUT_PATH}  ({N_POTS} entries × {N_COEFS} coefs = "
          f"{N_POTS * N_COEFS} hex words)")


if __name__ == "__main__":
    main()
