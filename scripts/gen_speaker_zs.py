#!/usr/bin/env python3
"""Generate speaker_zs.mem — a 2nd-order biquad that shapes v_sec before the
NFB loop sees it, modelling a guitar speaker's reactive impedance instead of
the flat 8 Ω assumption baked into the OT + NFB wiring.

What this filter represents
---------------------------
The output transformer steps Ip_A − Ip_B down through a turns ratio and
delivers it to the speaker.  Today, `nfb_network.sv` takes that secondary
voltage `v_sec` and feeds it back to the PI's shared cathode as if the load
were an ideal 8 Ω resistor.  A real guitar speaker's impedance Zs(f) is not
flat — it has a pronounced resonance peak near 80 Hz (Q ≈ 5, +15 dB-ish
above nominal) and an inductive rise at HF.  The NFB loop "sees" Zs(f), not
a flat 8 Ω, and that frequency-dependent divider is part of the amp's tonal
character (bass resonance slightly darker and tighter, presence sweep
slightly smoother at the top).

Rather than regenerating the OT subsystem, we insert a biquad between
`output_transformer.v_sec` and `nfb_network.v_sec` whose transfer function
is H(ω) = Zs(ω) / Z_ideal, where Z_ideal = 8 Ω.  The DAC path continues to
tap the raw `v_sec` (the `ir_cab` already bakes in a full miked-cabinet
response; adding Zs(f) on top would double-count the speaker).

Design
------
Full physical Zs(s) = Re + s·Le + R_res·(s·ω_s/Q) / (s² + s·ω_s/Q + ω_s²)
is 3rd-order (the VC inductance Le adds a trailing zero).  We drop Le for
the first pass — the bass resonance is the dominant NFB-relevant feature,
and a 2nd-order biquad captures it exactly as a standard RBJ peaking-EQ
filter (Audio EQ Cookbook, R. Bristow-Johnson):

    H(s) = Re/Z_ideal · (s² + (Re+R_res)/Re · s·ω_s/Q + ω_s²)
                       ─────────────────────────────────────
                             (s² + s·ω_s/Q + ω_s²)

  peak_gain  = (Re + R_res) / Re         (linear at f = f_s)
  baseline   = Re / Z_ideal              (plateau at f ≪ f_s and f ≫ f_s)

The HF inductive rise (Le) is a smaller effect and can be added later as a
second biquad section if wanted — see the parked second-stage slot below.

Output file layout (matches `cathode_shelf.sv`'s extended biquad form the
new `speaker_load.sv` consumes — 5 × 32-bit Q3.29 hex at addresses 0..4):
    addr 0: b0   (Q3.29)
    addr 1: b1   (Q3.29)
    addr 2: b2   (Q3.29)
    addr 3: a1   (Q3.29)          (a0 is normalised to 1 — not stored)
    addr 4: a2   (Q3.29)
"""

import argparse
import datetime
import math
import pathlib


# ──────────────────────────────────────────────────────────────────────────
# Speaker parameters — roughly a Celestion-style 12" guitar speaker on a
# closed-back 4×12, or an open-back 2×12.  Tweak these for different cabs.
# ──────────────────────────────────────────────────────────────────────────
FS_HZ        = 768_000.0       # oversampled sample rate the biquad runs at
Z_IDEAL      = 8.0             # Ω — flat load baked into the current NFB
SPEAKER_RE   = 6.0             # Ω — DC voice-coil resistance
SPEAKER_FS   = 80.0            # Hz — cone/suspension resonance
SPEAKER_Q    = 5.0             # Q — resonance sharpness (5 is a moderate peak)
SPEAKER_RRES = 40.0            # Ω — peak above Re at resonance
                               # (peak-to-baseline factor = (Re+Rres)/Re)

# HF inductive rise (VC inductance).  Parked — not synthesised into the
# biquad yet.  Noted here so a future second-section regeneration can pick
# these up without hunting through the script.
SPEAKER_LE_HENRIES = 0.4e-3    # ~0.4 mH — typical 12" VC inductance


# ──────────────────────────────────────────────────────────────────────────
# Bristow-Johnson peaking-EQ biquad
#     A      = sqrt(peak_linear)
#     ω0     = 2π·f0/fs
#     α      = sin(ω0) / (2Q)
#   pre-normalised coefficients (a0 = 1 + α/A):
#     b0 = 1 + α·A
#     b1 = -2·cos(ω0)
#     b2 = 1 − α·A
#     a1 = -2·cos(ω0)
#     a2 = 1 − α/A
# ──────────────────────────────────────────────────────────────────────────

def peaking_eq(f0, q, peak_linear, fs):
    A      = math.sqrt(peak_linear)
    ω0     = 2.0 * math.pi * f0 / fs
    cos_ω0 = math.cos(ω0)
    sin_ω0 = math.sin(ω0)
    α      = sin_ω0 / (2.0 * q)
    a0     = 1.0 + α / A
    b0     = (1.0 + α * A) / a0
    b1     = (-2.0 * cos_ω0) / a0
    b2     = (1.0 - α * A) / a0
    a1     = (-2.0 * cos_ω0) / a0
    a2     = (1.0 - α / A)   / a0
    return b0, b1, b2, a1, a2


def to_q_signed(val, int_bits, frac_bits, width):
    """Signed fixed-point conversion with saturation.  Matches the helper
    in gen_pentode_lut.py but local here so this script has no
    cross-dependency on that file."""
    scale = 1 << frac_bits
    q     = int(round(val * scale))
    max_v = (1 << (width - 1)) - 1
    min_v = -(1 << (width - 1))
    if q > max_v:
        q = max_v
    elif q < min_v:
        q = min_v
    return q & ((1 << width) - 1)


def write_mem(path, coeffs, header_lines):
    hdr = ['// ' + line for line in header_lines]
    body = '\n'.join(f'{c & 0xFFFFFFFF:08X}' for c in coeffs)
    path.write_text('\n'.join(hdr) + '\n' + body + '\n')


def main():
    ap = argparse.ArgumentParser(
        description='Generate speaker_zs.mem biquad coefficients')
    ap.add_argument('--out', type=str, default='lut_out/speaker_zs.mem')
    args = ap.parse_args()

    peak_linear  = (SPEAKER_RE + SPEAKER_RRES) / SPEAKER_RE
    baseline     = SPEAKER_RE / Z_IDEAL
    peak_absolute = peak_linear * baseline   # at-resonance gain vs flat 8 Ω

    b0, b1, b2, a1, a2 = peaking_eq(SPEAKER_FS, SPEAKER_Q, peak_linear, FS_HZ)

    # Apply baseline makeup (Re/Z_ideal) to the numerator — the biquad's
    # peaking-EQ form is normalised to unity DC gain, but the physical
    # divider plateau sits at Re/Z_ideal, not 1.0.
    b0 *= baseline
    b1 *= baseline
    b2 *= baseline

    # Q3.29 fixed point — range ±4 safely holds all coefficients.  |b1| and
    # |a1| are both ≈ 2·baseline ≈ 1.5, |b0| ≈ |b2| ≈ 0.75, |a2| ≈ 1.0.
    q_b0 = to_q_signed(b0, 3, 29, 32)
    q_b1 = to_q_signed(b1, 3, 29, 32)
    q_b2 = to_q_signed(b2, 3, 29, 32)
    q_a1 = to_q_signed(a1, 3, 29, 32)
    q_a2 = to_q_signed(a2, 3, 29, 32)

    path = pathlib.Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    peak_db = 20.0 * math.log10(peak_absolute)
    write_mem(
        path,
        [q_b0, q_b1, q_b2, q_a1, q_a2],
        [
            f'JCM800 speaker-impedance load biquad (Q3.29, 2nd-order)',
            f'Generated: {timestamp}',
            f'Physical params: Re={SPEAKER_RE} Ω, fs={SPEAKER_FS} Hz, '
            f'Q={SPEAKER_Q}, R_res={SPEAKER_RRES} Ω',
            f'  baseline (Re/Z_ideal) = {baseline:.4f} '
            f'({20*math.log10(baseline):+.2f} dB)',
            f'  peak-at-resonance gain ×{peak_absolute:.3f} '
            f'({peak_db:+.2f} dB)',
            f'Layout: addr 0..4 = b0 b1 b2 a1 a2 (a0 = 1; not stored)',
            f'HF inductive rise (Le = {SPEAKER_LE_HENRIES*1e3:.2f} mH) — '
            f'not synthesised into this stage.',
            f'Float coeffs:',
            f'  b0={b0:+.9f}  b1={b1:+.9f}  b2={b2:+.9f}',
            f'  a1={a1:+.9f}  a2={a2:+.9f}',
        ],
    )

    print(f'Wrote {path.resolve()}')
    print(f'  f0 = {SPEAKER_FS} Hz   Q = {SPEAKER_Q}   '
          f'peak vs flat 8Ω = {peak_db:+.2f} dB   '
          f'baseline = {20*math.log10(baseline):+.2f} dB')


if __name__ == '__main__':
    main()
