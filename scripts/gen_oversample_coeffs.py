"""
gen_oversample_coeffs.py — 16× minimum-phase polyphase oversampling coefficients
================================================================================

Designs four 2× halfband-style lowpass FIRs for the cascaded
48→96→192→384→768 kHz oversampler that wraps the JCM800 preamp.  Each
prototype is designed as a
Kaiser-windowed sinc lowpass (equiripple / PM would alternatively work, but
Kaiser is numerically robust at the tight −120 dB stopband of stage 1), then
converted to minimum phase by homomorphic spectral factorization.  The resulting
FIR has the SAME length and SAME magnitude response as the linear-phase
prototype, but all zeros are inside the unit circle — group delay is packed at
the front of the filter, eliminating pre-ringing on transients (the "pick
attack" artifact that makes linear-phase oversamplers sound unnatural on
guitars).

Each min-phase FIR is then polyphase-split into its even-indexed and
odd-indexed branches (E0, E1), quantized to Q1.17 signed 18-bit (matching the
Xilinx DSP48 coefficient port), and emitted as six .mem files readable by
$readmemh.  A SystemVerilog package (rtl/oversample/oversample_coeffs_pkg.sv) is
generated alongside containing the tap counts and .mem filenames.

Gain handling
─────────────
The prototype lowpass is normalised to unity DC gain.  Polyphase upsampling by
2 reduces amplitude by a factor of 2 (half the input samples are inserted
zeros).  This ×2 compensation is applied IN HARDWARE inside halfband_up2.sv by
a final left-shift at the output rounder, not by scaling the coefficients — so
the same .mem files are shared by upsampler and downsampler stages.

Quantisation
────────────
Q1.17 signed 18-bit: scale = 2^17, range [−131072, +131071], hex width = 5
digits.  $readmemh reads 5 hex chars into an 18-bit logic vector directly; the
consumer casts to signed [17:0].  Typical peak |coeff| for a min-phase halfband
is ~0.7, well inside Q1.17 range.

Verification
────────────
For each stage the script reports:
  - Actual passband ripple (dB)
  - Actual stopband attenuation (dB)
  - Peak coefficient magnitude (for headroom check)
  - Integer range and saturation count
If stopband attenuation falls short of spec by more than 3 dB, the script
prints a warning (does not fail — the generated files are still usable while
you iterate).
"""

import numpy as np
import pathlib
import datetime
from scipy import signal


# ─────────────────────────────────────────────────────────────────────────────
# Stage specifications — the four 2× halfband stages of the 16× cascade
# ─────────────────────────────────────────────────────────────────────────────
#
#   fs_high : output sample rate of this stage (kHz)
#   passband: passband edge (kHz) — must cover guitar audio bandwidth
#   stopband: stopband edge (kHz) — set to fs_in/2 to reject images/aliases
#   atten   : target stopband attenuation (dB)
#
# The ratio (passband / stopband) tightens as you move down the cascade —
# stage 1 is the hardest (20→24 kHz transition), later stages get very loose
# because the input Nyquist is far above 20 kHz.

STAGES = [
    dict(name='hb1', fs_low=48,  fs_high=96,   passband=20.0, stopband=24.0,  atten=120),
    dict(name='hb2', fs_low=96,  fs_high=192,  passband=20.0, stopband=48.0,  atten=100),
    dict(name='hb3', fs_low=192, fs_high=384,  passband=20.0, stopband=96.0,  atten=100),
    dict(name='hb4', fs_low=384, fs_high=768,  passband=20.0, stopband=192.0, atten=100),
]

COEF_W = 18           # total bits per coefficient (fits Xilinx DSP48 B-port)
COEF_Q = 17           # fractional bits (Q1.17)
HEX_W  = (COEF_W + 3) // 4   # 5 hex digits per entry
MEM_MASK = (1 << COEF_W) - 1


# ─────────────────────────────────────────────────────────────────────────────
# Filter design
# ─────────────────────────────────────────────────────────────────────────────

def design_linear_phase(spec):
    """
    Kaiser-windowed sinc lowpass meeting the per-stage passband/stopband/atten
    spec.  Length is forced odd and rounded up from kaiserord() estimate, with
    a small margin so the min-phase factorisation has headroom.
    """
    fs      = spec['fs_high']
    pb      = spec['passband']
    sb      = spec['stopband']
    A       = spec['atten']

    # Kaiser design: returns minimum taps and beta for given ripple + trans width.
    # Transition width is given relative to Nyquist = fs/2.
    trans_w = (sb - pb) / (fs / 2.0)
    N_est, beta = signal.kaiserord(A, trans_w)

    # Force odd length (cleaner polyphase split, symmetric about centre).
    if N_est % 2 == 0:
        N_est += 1
    # Add 10% margin — min-phase factorisation sometimes shows a small
    # attenuation degradation near the stopband edge due to finite n_fft.
    N = N_est + 1 if N_est % 2 == 0 else N_est
    N = int(np.ceil(N * 1.10))
    if N % 2 == 0:
        N += 1

    cutoff = (pb + sb) / 2.0
    h = signal.firwin(N, cutoff, window=('kaiser', beta), fs=fs)
    return h, beta, N_est


def to_minimum_phase(h_lp):
    """
    Homomorphic spectral factorisation.  Returns a FIR of the same length as
    h_lp with the same magnitude response but minimum-phase.  Uses a large
    n_fft for accuracy (the default is too small for long filters).
    """
    n_fft = max(1 << 16, 4 * len(h_lp))
    try:
        return signal.minimum_phase(h_lp, method='homomorphic', n_fft=n_fft, half=False)
    except TypeError:
        # Older scipy: no `half` kwarg, default behaviour returned same-length filter
        return signal.minimum_phase(h_lp, method='homomorphic', n_fft=n_fft)


def measure_response(h, spec, n_fft=1 << 14):
    """
    Compute passband ripple (dB, peak-to-peak) and stopband attenuation
    (dB, worst-case max) against the stage spec.
    """
    fs = spec['fs_high']
    w, H = signal.freqz(h, worN=n_fft, fs=fs)
    mag  = np.abs(H)

    pb_mask = w <= spec['passband']
    sb_mask = w >= spec['stopband']

    pb_max = mag[pb_mask].max()
    pb_min = mag[pb_mask].min()
    # pp ripple in dB (positive = how much passband gain varies)
    pb_ripple_dB = 20.0 * np.log10(pb_max / max(pb_min, 1e-30))
    sb_atten_dB  = -20.0 * np.log10(max(mag[sb_mask].max(), 1e-30))
    return pb_ripple_dB, sb_atten_dB


# ─────────────────────────────────────────────────────────────────────────────
# Quantisation
# ─────────────────────────────────────────────────────────────────────────────

def quantise_q1_17(h):
    """
    Quantise float coefficients to Q1.17 signed 18-bit int64.  Clips at ±full
    scale and reports the saturation count.
    """
    scale = 1 << COEF_Q          # 131072
    lo    = -(1 << (COEF_W - 1)) # -131072
    hi    =  (1 << (COEF_W - 1)) - 1   # +131071
    q_unclipped = np.round(h * scale).astype(np.int64)
    q = np.clip(q_unclipped, lo, hi)
    sat_count = int(np.sum(q_unclipped != q))
    return q, sat_count


# ─────────────────────────────────────────────────────────────────────────────
# File writers
# ─────────────────────────────────────────────────────────────────────────────

def write_mem(path, coeffs_q, stage, branch, K, spec, pb_rip, sb_att, timestamp):
    """Emit a $readmemh-compatible .mem with a descriptive header."""
    path.parent.mkdir(parents=True, exist_ok=True)
    header = (
        f"// {stage.upper()} polyphase branch {branch.upper()} — "
        f"stage {spec['fs_low']}→{spec['fs_high']} kHz\n"
        f"// Generated: {timestamp}\n"
        f"// Entries:   {K}   (polyphase branch of min-phase halfband)\n"
        f"// Format:    Q1.{COEF_Q} signed, {COEF_W}-bit ({HEX_W} hex digits), $readmemh\n"
        f"// Filter:    passband 0–{spec['passband']:.0f} kHz, "
        f"stopband ≥{spec['stopband']:.0f} kHz\n"
        f"// Achieved:  passband ripple {pb_rip:.4f} dB, "
        f"stopband atten {sb_att:.2f} dB\n"
        f"// Note:      consumed by rtl/oversample/hb_coef_rom.sv via $readmemh\n"
    )
    lines = [f"{int(v) & MEM_MASK:0{HEX_W}X}" for v in coeffs_q]
    path.write_text(header + "\n".join(lines) + "\n")


def write_pkg(path, stages_info, script_name, timestamp):
    """
    Emit the SystemVerilog coefficients package.  Matches the
    jcm800_lut_pkg.sv style (header block → package → localparams).
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "//===========================================================================",
        "// oversample_coeffs_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.",
        f"//   Produced by: scripts/{script_name}",
        f"//   Timestamp:   {timestamp}",
        "//",
        "//   Polyphase halfband coefficient lengths and .mem filenames for the 16×",
        "//   minimum-phase oversampling cascade.  Regenerate via:",
        "//     python scripts/gen_oversample_coeffs.py",
        "//===========================================================================",
        "package oversample_coeffs_pkg;",
        "",
        "    // Fixed-point widths shared across all stages",
        f"    localparam int COEF_W = {COEF_W};",
        f"    localparam int COEF_Q = {COEF_Q};",
        "",
    ]
    for s in stages_info:
        up = s['name'].upper()
        lines += [
            f"    // {up}: {s['fs_low']}→{s['fs_high']} kHz  "
            f"(passband {s['passband']:.0f} kHz, achieved atten {s['sb_att']:.1f} dB)",
            f"    localparam int    {up}_N        = {s['N']};",
            f"    localparam int    {up}_K0       = {s['K0']};   // even-index branch length",
            f"    localparam int    {up}_K1       = {s['K1']};   // odd-index branch length",
            f'    localparam string {up}_E0_FILE  = "{s["name"]}_e0.mem";',
            f'    localparam string {up}_E1_FILE  = "{s["name"]}_e1.mem";',
            "",
        ]
    lines.append("endpackage")
    path.write_text("\n".join(lines) + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    repo    = pathlib.Path(__file__).resolve().parent.parent
    lut_dir = repo / 'lut_out'
    pkg_pth = repo / 'rtl' / 'oversample' / 'oversample_coeffs_pkg.sv'
    lut_dir.mkdir(parents=True, exist_ok=True)

    timestamp   = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    script_name = pathlib.Path(__file__).name

    print("Designing minimum-phase polyphase halfband cascade (16× oversampling)\n")
    print(f"  {'stage':<5}  {'rate':<13}  {'N':>4}  {'peak':>6}  "
          f"{'pb ripple':>10}  {'sb atten':>9}  {'target':>7}  status")
    print("  " + "─" * 78)

    stages_info = []
    any_warning = False

    for spec in STAGES:
        h_lp, beta, N_est = design_linear_phase(spec)
        h_min             = to_minimum_phase(h_lp)

        pb_rip, sb_att = measure_response(h_min, spec)
        peak           = float(np.max(np.abs(h_min)))

        # Polyphase split
        e0 = h_min[0::2]   # even indices
        e1 = h_min[1::2]   # odd indices
        K0, K1 = len(e0), len(e1)
        N      = len(h_min)

        # Quantise each branch separately (some coeffs may saturate on one
        # branch and not the other; track them independently).
        e0_q, sat0 = quantise_q1_17(e0)
        e1_q, sat1 = quantise_q1_17(e1)

        spec_ok = sb_att >= spec['atten'] - 3.0
        status  = "ok" if spec_ok else "SHORT"
        if not spec_ok:
            any_warning = True

        print(f"  {spec['name']:<5}  {spec['fs_low']:>3}→{spec['fs_high']:<3} kHz  "
              f"{N:>4}  {peak:6.3f}  {pb_rip:>9.4f} dB  {sb_att:>7.2f} dB  "
              f"{spec['atten']:>5} dB   {status}")
        if sat0 + sat1:
            print(f"       warning: {sat0+sat1} coefficient(s) saturated at Q1.{COEF_Q} range")

        # Write .mem files
        write_mem(lut_dir / f"{spec['name']}_e0.mem",
                  e0_q, spec['name'], 'e0', K0, spec, pb_rip, sb_att, timestamp)
        write_mem(lut_dir / f"{spec['name']}_e1.mem",
                  e1_q, spec['name'], 'e1', K1, spec, pb_rip, sb_att, timestamp)

        stages_info.append(dict(
            name     = spec['name'],
            fs_low   = spec['fs_low'],
            fs_high  = spec['fs_high'],
            passband = spec['passband'],
            N        = N,
            K0       = K0,
            K1       = K1,
            pb_rip   = pb_rip,
            sb_att   = sb_att,
            peak     = peak,
        ))

    # Write the SystemVerilog package
    write_pkg(pkg_pth, stages_info, script_name, timestamp)

    print()
    print(f"  wrote {len(stages_info) * 2} .mem files to {lut_dir.relative_to(repo)}/")
    print(f"  wrote {pkg_pth.relative_to(repo)}")
    if any_warning:
        print("\n  ! one or more stages fell short of target stopband attenuation")
        print("    consider relaxing transition bandwidth or increasing tap count")
    else:
        print("\n  all stages meet stopband spec within 3 dB margin ✓")


if __name__ == '__main__':
    main()
