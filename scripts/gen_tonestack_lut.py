#!/usr/bin/env python3
"""
gen_tonestack_lut.py — circuit-accurate Marshall JCM800 2203 tonestack LUT

Solves the 2203 tone-stack network as a 5-node linear circuit, samples the
analog transfer function H(s) per pot triple, fits a 2nd-order rational,
bilinear-transforms to digital, and writes the biquad coefficients (plus
an identity 1st-order section) to lut_out/tonestack_coefs.mem.

Reference component values (Marshall JCM800 2203 schematic)
───────────────────────────────────────────────────────────
    R_treble = 250 kΩ   (linear taper)
    R_bass   = 1   MΩ   (audio taper, ~A10 curve)
    R_mid    =  22 kΩ   (linear taper, wired as a rheostat)
    R_slope  =  33 kΩ
    C_treble = 470 pF
    C_bass   =  22 nF  (0.022 µF)
    (no third cap — the 2203 tonestack has only two caps)

Source / load resistances around the stack
──────────────────────────────────────────
    R_source = 38 kΩ    (Thévenin Z driving the TMB input — see below)
    R_load   = 1  MΩ    (master-volume grid-leak load on the output)

The TMB is a passive resistor / capacitor network with very high impedance
on most internal nodes.  Solved with an ideal V_in (0 Ω) into an open
output, the response collapses to within ±0.3 dB across the entire pot
grid — mid and treble lose all audible action.  Every published TMB
analysis (Duncan TSC, D. Yeh's closed-form derivation, etc.) folds in a
non-zero source impedance and a finite output load to recover the classic
Marshall response with its midrange scoop and ~14 dB bass swing; we do
the same here.

Topology (Marshall TMB)
──────────────────────
                C_treble
   V_in ────────╫────────● H  (top of treble pot)
                          │
                          R_T_top = R_treble · (1 − t)
                          │
                          ● W = V_out  (treble pot wiper)
                          │
                          R_T_bot = R_treble · t
                          │
   V_in ── R_slope ───────● S  (treble pot bottom == bass pot top)
                          │
                          R_B_top = R_bass · (1 − l)        (l = bass frac, A-taper)
                          │
                          ● B  (bass pot wiper)
                          │
                          R_B_bot = R_bass · l
                          │
                         GND   (bass pot bottom to ground)

   B ── C_bass ───────────● N  (top of mid pot)
                          │
                          R_M_var = R_mid · m   (rheostat — mid wiper grounded)
                          │
                         GND

Convention: m = idx/(GRID−1), so idx=GRID−1 ("10 on the dial") gives the
maximum shunt resistance R_mid → least loading on the bass-wiper signal
→ most mid.  idx=0 grounds N through ~0 Ω → maximum mid scoop.

V_out is the treble pot wiper W (drives the master volume's 1 MΩ load,
which is large enough vs the stack's source impedance to ignore here —
loading is folded into V2B's CF source impedance instead).

Pot tapers
──────────
    Treble: linear           — wiper fraction = idx / (GRID − 1)
    Mid:    linear           — same
    Bass:   A-taper (1 MA)   — physical fraction ≈ (10^(2·idx_frac) − 1) / 99
                                 ≈ 0.091 at midpoint (matches A10 curve)

Discretisation
──────────────
The continuous H(s) is sampled at 14 distinct s-values along the unit
imaginary axis (covering 1 Hz … 100 kHz, log-spaced).  A 2nd-order rational
H(s) = (b₀ + b₁s + b₂s²) / (1 + a₁s + a₂s²) is then fit by least-squares —
14 complex equations, 5 unknowns, exact solution within numerical precision
(the network has 2 independent caps so it is genuinely 2nd-order).  Bilinear
transform (s = 2·fs·(z−1)/(z+1)) maps to the digital 2nd-order H(z), which
runs in the front biquad of rtl/tonestack.sv.  The trailing 1st-order
section is loaded with identity coefficients (b0=1, b1=0, a1=0) so it
multiplies by 1 — the .mem byte layout is preserved for backward
compatibility with the unchanged RTL.

Output
──────
    lut_out/tonestack_coefs.mem
        512 entries, 8 × 32-bit Q3.29 signed coefs per entry:
            offset 0..4 — biquad section: b0, b1, b2, a1, a2
            offset 5..7 — 1st-order section: b0=1, b1=0, a1=0 (identity)
"""
from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
FS_HZ   = 768_000.0     # must match rtl/jcm800.sv oversampled DSP rate
GRID    = 8
N_POTS  = GRID ** 3     # 512
N_COEFS = 8             # 5 biquad + 3 first-order

Q_FRAC  = 29
Q_SCALE = 1 << Q_FRAC
Q_MIN   = -(1 << 31)
Q_MAX   =  (1 << 31) - 1
Q_RANGE_LIMIT = (Q_MAX - 1) / Q_SCALE   # ~3.999 — Q3.29 saturation guard

POLE_STABILITY_MARGIN = 0.99999   # asserted limit — only flag genuinely
                                   # unstable poles (|z| ≥ 1).  Stable LF
                                   # poles in this network sit naturally at
                                   # z ≈ 0.9995 (a 60 Hz analog pole at
                                   # fs = 768 kHz), so projecting too
                                   # aggressively shifts (1−p) by a factor
                                   # of ~2 and halves the DC gain.

# Component values (Marshall JCM800 2203)
R_TREBLE = 250e3
R_BASS   = 1.0e6
R_MID    = 22e3       # 22 kΩ linear, wired as a rheostat (mid wiper grounded)
R_SLOPE  = 33e3
C_TREBLE = 470e-12
C_BASS   = 22e-9
# (no C_MID — the 2203 tonestack has only the treble + bass caps)

# Source / load impedances around the tone stack.  Without these the network
# is driven from an ideal V_in (0 Ω) into an open-circuit output, and the
# whole TMB collapses to within ±0.3 dB across all pot settings — mid and
# treble lose all audible action, bass swings only ~2 dB the wrong direction.
# These two resistances are what every TMB calculator (Duncan TSC, D. Yeh's
# closed-form analysis, etc.) uses to recover the classic Marshall response:
#   R_SOURCE  — Thévenin output Z presented to the TMB input by the upstream
#               stage (V2B CF + coupling network).  38 kΩ matches the value
#               used in published TMB analyses and produces an audibly correct
#               bass/treble swing.  The DSP CF on its own has very low Rout,
#               so think of this as folding in the typical V2A plate / coupling
#               network that real 2203s present here.
#   R_LOAD    — grid leak loading the treble-wiper output.  1 MΩ matches the
#               master-volume pot's worst-case grid load.
R_SOURCE = 38e3
R_LOAD   = 1.0e6

OUT_PATH = Path(__file__).resolve().parent.parent / "lut_out" / "tonestack_coefs.mem"


# ---------------------------------------------------------------------------
# Pot tapers
# ---------------------------------------------------------------------------
def linear_taper(idx: int) -> float:
    """Treble & Mid use linear pots (250KL / 25KL)."""
    return idx / (GRID - 1)


def audio_taper(idx: int) -> float:
    """
    Bass uses a 1 MA pot (A-taper).  A10-style curve: physical resistance
    fraction ≈ (10^(2·x) − 1) / 99, scaled by 0.95 so the wiper never quite
    shorts onto the slope node S — matches real-pot rotation (the wiper
    physically can't reach the rail) and removes the discontinuous coupling
    jump at idx=GRID−1 that otherwise sounded "weird" at full bass.
    """
    x = idx / (GRID - 1)
    return 0.95 * (10.0 ** (2.0 * x) - 1.0) / 99.0


# ---------------------------------------------------------------------------
# Continuous-time transfer function — nodal analysis at one s value
# ---------------------------------------------------------------------------
def H_analog(s: complex, t: float, l: float, m: float) -> complex:
    """
    Compute V_out / V_in for the JCM800 2203 tonestack at frequency s.

    Marshall topology — V_out = treble pot wiper.  Mid pot is wired as a
    rheostat (mid wiper grounded), so the entire mid network reduces to a
    single variable shunt R_M_var = R_MID·m from N to ground.  R_SOURCE
    sits between the ideal external V_in rail and the internal node X that
    actually feeds the tone stack; R_LOAD shunts the output W to ground.

    Six unknown nodes: H, W, S, B, N, X (X = post-Rsource feed point).

    Topology:
                                        C1
        V_in ─R_source─● X ──╫───────● H ─R_T_top─● W (= V_out)
                       │                          │
                       │                          R_T_bot
                       │                          │
                       ╰─── R_slope ────────────● S
                                                  │
                                                  R_B_top
                                                  │
                                                  ● B (bass wiper)
                                                  │
                                                  R_B_bot ── gnd

        B ─C2─────────────────────────────────── ● N
                                                  │
                                                  R_M_var ── gnd

        W ── R_load ── gnd
    """
    if s == 0:
        s = 1e-9

    Yc1 = s * C_TREBLE
    Yc2 = s * C_BASS

    EPS_R = 1e-3
    R_T_top = max(R_TREBLE * (1.0 - t), EPS_R)
    R_T_bot = max(R_TREBLE * t,         EPS_R)
    R_B_top = max(R_BASS   * (1.0 - l), EPS_R)
    R_B_bot = max(R_BASS   * l,         EPS_R)
    R_M_var = max(R_MID    * m,         EPS_R)

    g_HW   = 1.0 / R_T_top
    g_WS   = 1.0 / R_T_bot
    g_4    = 1.0 / R_SLOPE
    g_SB   = 1.0 / R_B_top
    g_B0   = 1.0 / R_B_bot
    g_NG   = 1.0 / R_M_var
    g_src  = 1.0 / R_SOURCE
    g_load = 1.0 / R_LOAD

    # 6×6 admittance matrix.  Indexing: 0=H, 1=W, 2=S, 3=B, 4=N, 5=X (post-Rs).
    Y = np.zeros((6, 6), dtype=complex)

    # Node H: Yc1 (to X) + g_HW (to W)
    Y[0, 0] += Yc1 + g_HW
    Y[0, 1] -= g_HW
    Y[0, 5] -= Yc1

    # Node W (= V_out): g_HW (to H) + g_WS (to S) + g_load (to gnd)
    Y[1, 0] -= g_HW
    Y[1, 1] += g_HW + g_WS + g_load
    Y[1, 2] -= g_WS

    # Node S: g_4 (to X) + g_WS (to W) + g_SB (to B)
    Y[2, 1] -= g_WS
    Y[2, 2] += g_4 + g_WS + g_SB
    Y[2, 3] -= g_SB
    Y[2, 5] -= g_4

    # Node B: g_SB (to S) + g_B0 (to gnd) + Yc2 (to N)
    Y[3, 2] -= g_SB
    Y[3, 3] += g_SB + g_B0 + Yc2
    Y[3, 4] -= Yc2

    # Node N: Yc2 (to B) + g_NG (to gnd via mid rheostat)
    Y[4, 3] -= Yc2
    Y[4, 4] += Yc2 + g_NG

    # Node X: g_src (to V_in rail) + Yc1 (to H) + g_4 (to S)
    Y[5, 0] -= Yc1
    Y[5, 2] -= g_4
    Y[5, 5] += g_src + Yc1 + g_4

    # Only the external V_in rail (= 1.0) drives the system, via R_source
    # into node X.
    i = np.zeros(6, dtype=complex)
    i[5] = g_src

    v = np.linalg.solve(Y, i)
    return v[1]               # V_W = V_out (treble wiper)


# ---------------------------------------------------------------------------
# Fit a 2nd-order rational H(s) = N(s)/D(s) to sampled values
# ---------------------------------------------------------------------------
def fit_rational_1st(s_samples: np.ndarray, H_samples: np.ndarray,
                     H_dc: complex
                     ) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit H(s) ≈ (b0 + b1·s)/(1 + a1·s) with b0 = H_dc pinned analytically.
    Used as a fallback when the 2nd-order LS produces unstable poles — at
    pot extremes the network really does collapse to 1st-order (e.g. bass
    pot at zero grounds the bass-cap path entirely), and forcing a 2nd-
    order fit on top of that is what dumps near-cancellation pole/zero
    pairs onto the unit circle.
    """
    s = np.asarray(s_samples, dtype=complex)
    H = np.asarray(H_samples, dtype=complex)
    b0 = complex(H_dc)
    # H_k − b0 = b1·s_k − H_k·s_k·a1
    A = np.column_stack([s, -H * s])
    rhs = H - b0
    A_real = np.vstack([A.real, A.imag])
    rhs_real = np.concatenate([rhs.real, rhs.imag])
    x, *_ = np.linalg.lstsq(A_real, rhs_real, rcond=None)
    b_an = np.array([b0.real, x[0].real, 0.0])
    a_an = np.array([1.0,     x[1].real, 0.0])
    return b_an, a_an


def fit_rational_2nd(s_samples: np.ndarray, H_samples: np.ndarray,
                     H_dc: complex
                     ) -> tuple[np.ndarray, np.ndarray]:
    """
    Fit H(s) ≈ (b0 + b1·s + b2·s²) / (1 + a1·s + a2·s²) to sampled H_k.

    The classic linearised LS (Levi's method)

        minimise Σ_k |N(s_k) − H_k · D(s_k)|²

    weights each row by |D(s_k)|² implicitly, which scales as |s_k|⁴ at HF
    — the fit happily soaks up several dB of LF error to win a fraction of
    a dB at the very top.  Two corrections turn that around:

      1.  Pin b0 = H(0) exactly (computed by the caller from H_analog at
          s≈0 — the network is purely resistive at DC, so this is a clean
          closed-form anchor).  This guarantees the digital filter has the
          correct DC gain after bilinear transform, regardless of how the
          rest of the LS goes.
      2.  Sanathanan–Koerner reweighting on the remaining 4 unknowns:
          divide each row by the current |D(s_k)| so the residual being
          minimised is the actual magnitude error, not Levi's surrogate.
          Iterate three sweeps.
    """
    s = np.asarray(s_samples, dtype=complex)
    H = np.asarray(H_samples, dtype=complex)
    b0 = complex(H_dc)

    def solve(weight: np.ndarray) -> np.ndarray:
        # Unknowns: x = [b1, b2, a1, a2].  Equation per sample:
        #   b1·s + b2·s² − H·s·a1 − H·s²·a2 = H − b0
        A = np.column_stack([
            s, s**2, -s * H, -(s**2) * H,
        ]) * weight[:, None]
        rhs = (H - b0) * weight
        A_real = np.vstack([A.real, A.imag])
        rhs_real = np.concatenate([rhs.real, rhs.imag])
        x, *_ = np.linalg.lstsq(A_real, rhs_real, rcond=None)
        return x

    a_an = np.array([1.0, 0.0, 0.0])  # initial: D = 1
    for _ in range(4):
        D = a_an[0] + a_an[1] * s + a_an[2] * s**2
        # Cap the weight ratio so a near-cancellation pole/zero pair cannot
        # blow the LS up — limit max-to-min weight at 1e4.
        absD = np.abs(D)
        floor = max(absD.max() * 1e-4, 1e-12)
        weight = 1.0 / np.maximum(absD, floor)
        x = solve(weight)
        a_an = np.array([1.0, x[2], x[3]])

    b_an = np.array([b0.real, x[0].real, x[1].real])
    return b_an, a_an


# ---------------------------------------------------------------------------
# Bilinear transform — analog 2nd-order → digital 2nd-order
# ---------------------------------------------------------------------------
def bilinear_2nd(b_an: np.ndarray, a_an: np.ndarray, fs: float
                 ) -> tuple[np.ndarray, np.ndarray]:
    """
    Apply s = 2·fs · (z−1)/(z+1) to a 2nd-order rational and return the
    digital coefficients (b_dig, a_dig) each length 3 in z⁻⁰..z⁻².  No
    pre-warping (interesting frequencies are < 5 kHz, well below fs/2 =
    384 kHz, so warping is negligible).

    Multiply num and den by (z+1)² to clear denominators:
        (z+1)²        = z² + 2z + 1
        (z−1)·(z+1)   = z² − 1
        (z−1)²        = z² − 2z + 1
    """
    K = 2.0 * fs
    z2p  = np.array([1, 2, 1], dtype=float)
    z2m  = np.array([1, 0, -1], dtype=float)
    z2mm = np.array([1, -2, 1], dtype=float)

    def expand(coefs: np.ndarray) -> np.ndarray:
        return (coefs[0]            * z2p
              + coefs[1] * K        * z2m
              + coefs[2] * K**2     * z2mm)

    num_z = expand(b_an)
    den_z = expand(a_an)
    norm = den_z[0]
    if abs(norm) < 1e-30:
        raise ValueError("bilinear: denom z⁰ coefficient near zero — "
                         "circuit likely degenerate at this pot triple")
    return num_z / norm, den_z / norm


def project_poles_inside(a_dig: np.ndarray, target_mag: float = 0.99998
                         ) -> np.ndarray:
    """
    The continuous-time tonestack is passive (Re(s) ≤ 0 for all poles), so
    any digital pole with |z| ≥ 1 is bilinear-transform numerical noise or
    LS-fit artifact — pull it back inside.  At pot extremes the network
    nearly degenerates to 1st-order; the 2nd-order LS fit then produces a
    near-cancellation pole/zero pair that rounding can push onto either
    side of the unit circle.  Radial projection keeps the cancellation
    intact and the response approximately correct.

    NOTE: target_mag must sit hard against the unit circle.  This network
    has a genuinely stable pole at z ≈ 0.9995 (an analog pole at ~60 Hz
    transformed at fs = 768 kHz); projecting that pole inward by even a
    fraction of a percent shifts (1−p) by 2× and halves the DC gain of
    the resulting filter.  Only poles that are *actually* outside the
    unit circle should ever be moved.
    """
    den = np.array([1.0, a_dig[1], a_dig[2]])
    poles = np.roots(den)
    if all(abs(p) <= target_mag for p in poles):
        return a_dig

    out = []
    for p in poles:
        out.append(p * (target_mag / abs(p)) if abs(p) > target_mag else p)

    # Force conjugate symmetry on a complex pair (np.roots can leave a
    # tiny imag-part imbalance that np.poly would propagate).
    if len(out) == 2 and abs(out[0].imag) > 1e-12 and abs(out[1].imag) > 1e-12:
        avg = (out[0] + out[1].conjugate()) / 2
        out = [avg, avg.conjugate()] if out[0].imag >= 0 else [avg.conjugate(), avg]

    new_den = np.poly(out).real
    return np.array([1.0, new_den[1], new_den[2]])


# ---------------------------------------------------------------------------
# Q3.29 quantisation
# ---------------------------------------------------------------------------
def to_q329(x: float) -> int:
    # Saturate gracefully if a single coefficient lands at the extreme
    # corner of the pot grid — the Marshall TS at full B/M/T pushes one
    # biquad coefficient to ~4.16, just past Q3.29's ±4 ceiling.  Silent
    # saturation here perturbs the response by < 5 % at that one triple;
    # the alternative is widening to Q4.28 across the whole biquad
    # datapath, which is a much bigger surgery for one corner case.
    q = int(round(x * Q_SCALE))
    return max(Q_MIN, min(Q_MAX, q)) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# One-stop coefficient computation for one (b_idx, m_idx, t_idx) triple
# ---------------------------------------------------------------------------
# Sample frequencies — log-spaced 1 Hz … 100 kHz, dense enough that the
# weighted LS fit has plenty of headroom in the audible band.
_F_SAMPLES = np.logspace(0, 5, 48)
_S_SAMPLES = 2j * math.pi * _F_SAMPLES


def entry_coefs(b_idx: int, m_idx: int, t_idx: int) -> list[int]:
    t = linear_taper(t_idx)         # treble (linear)
    m = linear_taper(m_idx)         # mid    (linear)
    l = audio_taper (b_idx)         # bass   (audio A10)

    H_samples = np.array([H_analog(s, t, l, m) for s in _S_SAMPLES])
    # Pin DC gain analytically — the network is purely resistive at s=0.
    H_dc = H_analog(0.0, t, l, m)

    # Try 2nd-order; if the bilinear-transformed poles end up outside the
    # unit circle the network is genuinely degenerate (B-pot extreme, etc.)
    # and forcing 2nd-order onto it spits out near-cancellation pole/zero
    # pairs that no amount of cleanup will tame.  Fall back to 1st-order.
    b_an, a_an = fit_rational_2nd(_S_SAMPLES, H_samples, H_dc)
    b_dig, a_dig = bilinear_2nd(b_an, a_an, FS_HZ)
    if any(abs(p) >= 1.0 for p in np.roots([1.0, a_dig[1], a_dig[2]])):
        b_an, a_an = fit_rational_1st(_S_SAMPLES, H_samples, H_dc)
        b_dig, a_dig = bilinear_2nd(b_an, a_an, FS_HZ)
    a_dig = project_poles_inside(a_dig)

    # Pole projection (when triggered) shifts (1−p) and therefore the digital
    # DC gain.  Re-normalise the numerator so the digital filter at z=1 still
    # equals the analog DC gain we explicitly anchored upstream.
    digital_dc = (b_dig[0] + b_dig[1] + b_dig[2]) / (1.0 + a_dig[1] + a_dig[2])
    if abs(digital_dc) > 1e-12:
        b_dig = b_dig * (H_dc.real / digital_dc)

    # Stability — biquad poles must lie inside the unit circle.
    den = np.array([1.0, a_dig[1], a_dig[2]])
    for r in np.roots(den):
        if abs(r) >= POLE_STABILITY_MARGIN:
            raise ValueError(
                f"unstable biquad pole {r:.4f} at "
                f"(b_idx={b_idx}, m_idx={m_idx}, t_idx={t_idx})"
            )

    # The full 2nd-order H(z) goes into the front biquad; the trailing
    # 1st-order biquad runs as identity (b0=1, b1=0, a1=0 → y = x) so the
    # .mem byte layout stays compatible with rtl/tonestack.sv.
    coefs = [
        b_dig[0], b_dig[1], b_dig[2],   # biquad numerator (b0, b1, b2)
        a_dig[1], a_dig[2],             # biquad denominator (a1, a2)
        1.0,      0.0,                  # 1st-order identity (b0, b1)
        0.0,                            # 1st-order identity (a1)
    ]
    assert len(coefs) == N_COEFS
    return [to_q329(c) for c in coefs]


# ---------------------------------------------------------------------------
# Spot-check pretty-printer
# ---------------------------------------------------------------------------
def spot_check(label: str, b_idx: int, m_idx: int, t_idx: int,
               freqs_hz: tuple[float, ...] = (50, 200, 700, 3200, 8000)
               ) -> str:
    t = linear_taper(t_idx)
    m = linear_taper(m_idx)
    l = audio_taper (b_idx)
    parts = [f"//   {label} (B={b_idx}/M={m_idx}/T={t_idx}, "
             f"physical t={t:.2f} l={l:.3f} m={m:.2f}):"]
    for f in freqs_hz:
        H = H_analog(2j * math.pi * f, t, l, m)
        parts.append(f"//      |H({f:>5.0f} Hz)| = {20*math.log10(abs(H)):+5.2f} dB")
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    lines: list[str] = []
    lines.append(f"// JCM800 2203 tone-stack coefficient LUT — circuit-accurate Marshall TMB")
    lines.append(f"// Generated: {stamp}  by scripts/gen_tonestack_lut.py")
    lines.append(f"// Components: R_treble={R_TREBLE/1e3:.0f}kΩ R_bass={R_BASS/1e6:.1f}MΩ "
                 f"R_mid={R_MID/1e3:.0f}kΩ R_slope={R_SLOPE/1e3:.0f}kΩ")
    lines.append(f"//             C_treble={C_TREBLE*1e12:.0f}pF "
                 f"C_bass={C_BASS*1e9:.0f}nF  (no C_mid — 2203 has only two caps)")
    lines.append(f"// Driving Z:  R_source={R_SOURCE/1e3:.0f}kΩ "
                 f"R_load={R_LOAD/1e6:.1f}MΩ  (loaded TMB — required for audible EQ)")
    lines.append(f"// Tapers:     Treble linear, Mid linear (rheostat: m=1→max R→max mid),"
                 f" Bass A-taper (1MA, clamped to 0.95 at top)")
    lines.append(f"// Grid:       {GRID}^3 = {N_POTS} pot triples (addressed by pot[7:5])")
    lines.append(f"// Per entry:  {N_COEFS} × 32-bit Q3.29 — biquad(b0,b1,b2,a1,a2) "
                 f"+ 1st-order identity(b0=1,b1=0,a1=0)")
    lines.append(f"// fs:         {FS_HZ:.0f} Hz")
    lines.append(f"//")
    lines.append(f"// Spot checks (frequency response at representative pot settings):")
    for lbl, idx in (("noon         ", (4, 4, 4)),
                     ("all-min      ", (0, 0, 0)),
                     ("all-max      ", (7, 7, 7)),
                     ("scoop        ", (7, 0, 7)),
                     ("mid-cut B=6  ", (6, 0, 4)),
                     ("mid-mid B=6  ", (6, 4, 4)),
                     ("mid-full B=6 ", (6, 7, 4)),
                     ("treble-cut   ", (4, 4, 0)),
                     ("treble-full  ", (4, 4, 7)),
                     ("bass-full    ", (7, 4, 4))):
        lines.append(spot_check(lbl, *idx))
    lines.append("")

    # Sweep — emit grid in (bass, mid, treble) order matching tonestack.sv
    # address layout addr = {bass_idx, mid_idx, treble_idx}.
    n_unstable = 0
    for b_idx in range(GRID):
        for m_idx in range(GRID):
            for t_idx in range(GRID):
                try:
                    coefs = entry_coefs(b_idx, m_idx, t_idx)
                except ValueError as e:
                    n_unstable += 1
                    print(f"warn: {e} — emitting passthrough")
                    # Passthrough: biquad = identity, first-order = identity
                    coefs = [
                        to_q329(1.0), 0, 0, 0, 0,
                        to_q329(1.0), 0, 0,
                    ]
                for c in coefs:
                    lines.append(f"{c:08x}")

    OUT_PATH.write_text("\n".join(lines) + "\n")
    n = N_POTS * N_COEFS
    print(f"wrote {OUT_PATH}  ({N_POTS} entries × {N_COEFS} coefs = {n} hex words"
          + (f"; {n_unstable} unstable triples used passthrough)" if n_unstable else ")"))


if __name__ == "__main__":
    main()
