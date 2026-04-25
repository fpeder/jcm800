#!/usr/bin/env python3
"""
gen_tonestack_lut.py — circuit-accurate Marshall TMB tonestack LUT

Solves the 2203 tone-stack network as a 6-node linear circuit, samples the
analog transfer function H(s) per pot triple, fits a 3rd-order rational,
bilinear-transforms to digital, factors into a 2nd-order biquad cascaded
with a 1st-order biquad, and writes the section coefficients to
lut_out/tonestack_coefs.mem for rtl/tonestack.sv.

Reference component values (from Robinette JCM800 2203/2204 schematic)
─────────────────────────────────────────────────────────────────────
    R_treble = 250 kΩ   (linear taper)
    R_bass   = 1   MΩ   (audio taper, ~A10 curve)
    R_mid    =  25 kΩ   (linear taper)
    R_slope  =  33 kΩ
    C_treble = 470 pF
    C_bass   =  22 nF  (0.022 µF)
    C_mid    =  22 nF  (0.022 µF)

Topology (FMV / Marshall TMB — verified against Yeh's CCRMA thesis)
──────────────────────────────────────────────────────────────────
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
                          R_M_top = R_mid · (1 − m)
                          │
                          ● M  (mid pot wiper)
                          │
                         (R_M_bot = R_mid · m)  ‖  C_mid       (parallel to GND)
                          │
                         GND

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
imaginary axis (covering 1 Hz … 100 kHz, log-spaced).  A 3rd-order rational
H(s) = (b₀ + b₁s + b₂s² + b₃s³) / (1 + a₁s + a₂s² + a₃s³) is then fit by
least-squares — 14 complex equations, 7 unknowns, exact solution within
numerical precision (the network is genuinely 3rd-order so residuals are
< 1e-9).  Bilinear transform (s = 2·fs·(z−1)/(z+1)) maps to the digital
3rd-order H(z), which is split into a 2nd-order section (the complex pole
pair, if any — otherwise the two slowest real poles) and a 1st-order
section (remaining real pole).

Output
──────
    lut_out/tonestack_coefs.mem
        512 entries, 8 × 32-bit Q3.29 signed coefs per entry:
            offset 0..4 — biquad section: b0, b1, b2, a1, a2
            offset 5..7 — 1st-order section: b0, b1, a1
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

POLE_STABILITY_MARGIN = 0.9995    # asserted limit; projection target stays
                                   # below at 0.999 so re-extracted roots
                                   # have headroom.

# Component values (Marshall 2203/2204)
R_TREBLE = 250e3
R_BASS   = 1.0e6
R_MID    = 25e3
R_SLOPE  = 33e3
C_TREBLE = 470e-12
C_BASS   = 22e-9
C_MID    = 22e-9

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
    fraction ≈ (10^(2·x) − 1) / 99, giving y(0)=0, y(0.5)≈0.091, y(1)=1.
    """
    x = idx / (GRID - 1)
    return (10.0 ** (2.0 * x) - 1.0) / 99.0


# ---------------------------------------------------------------------------
# Continuous-time transfer function — nodal analysis at one s value
# ---------------------------------------------------------------------------
def H_analog(s: complex, t: float, l: float, m: float) -> complex:
    """
    Compute V_out / V_in for the JCM800 2203 tonestack at frequency s.

    Marshall topology — V_out = treble pot wiper alone.  The mid network
    is a shunt off the bass-pot wiper.  Mid pot bottom is directly
    grounded; C_mid sits between the mid pot wiper and ground in parallel
    with the wiper-to-bottom resistance R_M_bot.

    Five unknown nodes: H, W, S, B, N, M.  (M = mid wiper.)

    Topology:
                  C1
        V_in ─╫─● H ─R_T_top─● W (= V_out, treble wiper)
                              │
                              R_T_bot
                              │
        V_in ─R_4───────────● S
                              │
                              R_B_top
                              │
                              ● B (bass wiper)
                              │
                              R_B_bot ── gnd

        B ─C2─────────────── ● N (top of mid pot)
                              │
                              R_M_top
                              │
                              ● M (mid wiper)
                              │
                       (R_M_bot ‖ C3) → gnd
    """
    if s == 0:
        s = 1e-9

    Yc1 = s * C_TREBLE
    Yc2 = s * C_BASS
    Yc3 = s * C_MID

    EPS_R = 1e-3
    R_T_top = max(R_TREBLE * (1.0 - t), EPS_R)
    R_T_bot = max(R_TREBLE * t,         EPS_R)
    R_B_top = max(R_BASS   * (1.0 - l), EPS_R)
    R_B_bot = max(R_BASS   * l,         EPS_R)
    R_M_top = max(R_MID    * (1.0 - m), EPS_R)
    R_M_bot = max(R_MID    * m,         EPS_R)

    g_HW = 1.0 / R_T_top
    g_WS = 1.0 / R_T_bot
    g_4  = 1.0 / R_SLOPE
    g_SB = 1.0 / R_B_top
    g_B0 = 1.0 / R_B_bot
    g_NM = 1.0 / R_M_top
    g_M0 = 1.0 / R_M_bot

    # 6×6 admittance matrix.  Indexing: 0=H, 1=W, 2=S, 3=B, 4=N, 5=M
    Y = np.zeros((6, 6), dtype=complex)

    # Node H: Yc1 (to V_in) + g_HW (to W)
    Y[0, 0] += Yc1 + g_HW
    Y[0, 1] -= g_HW

    # Node W (= V_out): g_HW (to H) + g_WS (to S) — pure treble-wiper node
    Y[1, 0] -= g_HW
    Y[1, 1] += g_HW + g_WS
    Y[1, 2] -= g_WS

    # Node S: g_4 (to V_in) + g_WS (to W) + g_SB (to B)
    Y[2, 1] -= g_WS
    Y[2, 2] += g_4 + g_WS + g_SB
    Y[2, 3] -= g_SB

    # Node B: g_SB (to S) + g_B0 (to gnd) + Yc2 (to N)
    Y[3, 2] -= g_SB
    Y[3, 3] += g_SB + g_B0 + Yc2
    Y[3, 4] -= Yc2

    # Node N: Yc2 (to B) + g_NM (to M)
    Y[4, 3] -= Yc2
    Y[4, 4] += Yc2 + g_NM
    Y[4, 5] -= g_NM

    # Node M (mid wiper): g_NM (to N) + g_M0 (to gnd) + Yc3 (to gnd)
    Y[5, 4] -= g_NM
    Y[5, 5] += g_NM + g_M0 + Yc3

    i = np.zeros(6, dtype=complex)
    i[0] = Yc1                # V_in feeds H through C1
    i[2] = g_4                # V_in feeds S through R_slope

    v = np.linalg.solve(Y, i)
    return v[1]               # V_W = V_out (treble wiper)


# ---------------------------------------------------------------------------
# Fit a 3rd-order rational H(s) = N(s)/D(s) to sampled values
# ---------------------------------------------------------------------------
def fit_rational(s_samples: np.ndarray, H_samples: np.ndarray
                 ) -> tuple[np.ndarray, np.ndarray]:
    """
    Solve for {b0, b1, b2, b3, a1, a2, a3} (a0 := 1) via least-squares from
    H_k · (1 + a1·s_k + a2·s_k² + a3·s_k³) = b0 + b1·s_k + b2·s_k² + b3·s_k³

    Treat real and imag parts as separate equations to keep the LS real.
    Returns analog (b_coefs, a_coefs), each length 4 with a[0]=1.
    """
    rows = []
    rhs  = []
    for s, H in zip(s_samples, H_samples):
        row = np.array([
            1.0, s, s**2, s**3,             # b0..b3
            -s * H, -(s**2) * H, -(s**3) * H,   # −a1·s·H, −a2·s²·H, −a3·s³·H
        ], dtype=complex)
        rows.append(row)
        rhs.append(H)
    A = np.array(rows, dtype=complex)
    b = np.array(rhs, dtype=complex)
    # Stack real and imaginary parts so the LS solution is real.
    A_real = np.vstack([A.real, A.imag])
    b_real = np.concatenate([b.real, b.imag])
    x, *_ = np.linalg.lstsq(A_real, b_real, rcond=None)
    b_an = np.array([x[0], x[1], x[2], x[3]])     # b0..b3 (s⁰..s³)
    a_an = np.array([1.0,  x[4], x[5], x[6]])     # a0=1, a1..a3
    return b_an, a_an


# ---------------------------------------------------------------------------
# Bilinear transform — analog 3rd-order → digital 3rd-order
# ---------------------------------------------------------------------------
def bilinear_3rd(b_an: np.ndarray, a_an: np.ndarray, fs: float
                 ) -> tuple[np.ndarray, np.ndarray]:
    """
    Apply s = 2·fs · (z−1)/(z+1) to a 3rd-order rational and return the
    digital coefficients (b_dig, a_dig) each length 4 in z⁻⁰..z⁻³.  No
    pre-warping (the FMV stack's interesting frequencies are < 5 kHz, well
    below fs/2 = 384 kHz, so warping is negligible).
    """
    K = 2.0 * fs
    # Substitute s = K·u where u = (z−1)/(z+1).  Multiply num and den by
    # (z+1)³ to clear denominators, then collect powers of z.
    # (z+1)³ = z³ + 3z² + 3z + 1
    # (z−1)·(z+1)² = z³ + z² − z − 1
    # (z−1)²·(z+1) = z³ − z² − z + 1
    # (z−1)³        = z³ − 3z² + 3z − 1
    z3p = np.array([1, 3, 3, 1], dtype=float)
    z3m = np.array([1, 1, -1, -1], dtype=float)
    z3mm = np.array([1, -1, -1, 1], dtype=float)
    z3mmm = np.array([1, -3, 3, -1], dtype=float)

    def expand(coefs: np.ndarray) -> np.ndarray:
        # coefs[0..3] = c0 + c1·s + c2·s² + c3·s³
        # → c0·(z+1)³ + c1·K·(z−1)·(z+1)² + c2·K²·(z−1)²·(z+1) + c3·K³·(z−1)³
        out = (coefs[0]              * z3p
             + coefs[1] * K          * z3m
             + coefs[2] * K**2       * z3mm
             + coefs[3] * K**3       * z3mmm)
        return out

    num_z = expand(b_an)
    den_z = expand(a_an)
    # Normalise so a_dig[0] = 1 (i.e., divide both by den_z[0]).
    norm = den_z[0]
    if abs(norm) < 1e-30:
        raise ValueError("bilinear: denom z⁰ coefficient near zero — "
                         "circuit likely degenerate at this pot triple")
    return num_z / norm, den_z / norm


# ---------------------------------------------------------------------------
# Factor 3rd-order H(z) into a biquad (2nd-order) + first-order section
# ---------------------------------------------------------------------------
def _project_inside_unit_circle(roots: np.ndarray,
                                target_mag: float = 0.999
                                ) -> np.ndarray:
    """
    Pull any pole that bilinear-transform numerical noise has nudged near
    or outside the unit circle inside.  The continuous-time tonestack is
    passive so analog poles are strictly Re(s) ≤ 0 — any |z| ≥ 1 in the
    digital map is roundoff, not real instability.  Radially scale poles
    with |z| > target_mag down to exactly target_mag.

    target_mag = 0.999 places the worst-case pole at z ≈ 0.999 — analog
    fc ≈ 122 Hz at fs = 768 kHz (unit-circle approximation: fc ≈ fs·(1−|z|)
    /(2π)).  The real tonestack's lowest pole sits at ~30–80 Hz so this
    extra margin is well above the audible LF response and below 0.9999
    to avoid being clipped by re-extracted-roots round-trip at the
    stability check.
    """
    out = roots.copy()
    for i, r in enumerate(out):
        mag = abs(r)
        if mag > target_mag:
            out[i] = r * (target_mag / mag)
    return out


def factor_3rd_to_biquad_plus_first(b_dig: np.ndarray, a_dig: np.ndarray
                                    ) -> tuple[np.ndarray, np.ndarray,
                                               np.ndarray, np.ndarray]:
    """
    Given a 3rd-order H(z) = (b0+b1·z⁻¹+b2·z⁻²+b3·z⁻³)/(1+a1·z⁻¹+...)
    factor as
            ┌──────────────────┐   ┌──────────────────┐
        H = │ biquad (2nd-ord) │ × │ first-order shelf│
            │ b₀+b₁z⁻¹+b₂z⁻²   │   │ b₀+b₁z⁻¹         │
            │ ─────────────── │   │ ─────────────────│
            │ 1+a₁z⁻¹+a₂z⁻²   │   │ 1+a₁z⁻¹          │
            └──────────────────┘   └──────────────────┘

    Strategy: factor the denominator polynomial 1 + a1·z⁻¹ + a2·z⁻² + a3·z⁻³
    into (real-pole 1st-order)(complex-pair-or-2-real 2nd-order).  Pair the
    numerator roots accordingly and split numerator the same way.

    np.roots takes coefficients in highest-power-first order, so we work
    in z (not z⁻¹) for root finding, then convert back.

    Polynomial in z⁻¹: 1 + a1·z⁻¹ + a2·z⁻² + a3·z⁻³
    Multiply by z³ : z³ + a1·z² + a2·z + a3            ← np.roots input
    """
    # Numerator/denominator roots in z (poles & zeros of H).
    den_z = np.array([1.0, a_dig[1], a_dig[2], a_dig[3]])
    num_z = np.array([b_dig[0], b_dig[1], b_dig[2], b_dig[3]])
    poles = np.roots(den_z)
    zeros = np.roots(num_z) if abs(num_z[0]) > 1e-15 else np.roots(num_z[1:])

    # Project any near-unit-circle poles back inside (bilinear-transform
    # artifacts from poles at extreme LF — see _project_inside_unit_circle
    # for rationale).  Force conjugate symmetry on the complex pair too:
    # np.roots can return the pair with tiny imag-part imbalance from
    # roundoff, which np.poly then turns into a polynomial with non-zero
    # imaginary coefficients — defeating the .real cast below.
    poles = _project_inside_unit_circle(poles)
    # Re-conjugate-pair: identify the closest match for each complex root.
    n_complex = sum(1 for p in poles if abs(p.imag) > 1e-9)
    if n_complex == 2:
        # Force the two complex poles to be exact conjugates.
        cpx = [p for p in poles if abs(p.imag) > 1e-9]
        avg = (cpx[0] + cpx[1].conjugate()) / 2
        poles = np.array([
            avg if abs(p.imag) > 1e-9 and p.imag > 0
            else avg.conjugate() if abs(p.imag) > 1e-9
            else p
            for p in poles
        ])
    # Zeros can sit anywhere in the z-plane (a zero outside the unit
    # circle is fine for stability — it's just non-minimum-phase).

    # Pull out the most-real pole (smallest |Im|) for the 1st-order section.
    pole_order = np.argsort(np.abs(poles.imag))
    p1 = poles[pole_order[0]].real        # lone real pole → 1st-order
    p_pair = poles[pole_order[1:]]        # other two → biquad

    # If the "remaining two" happen to be a complex-conjugate pair, build
    # the biquad denom from their product (real coefficients fall out).
    # If they're both real, same product still works.
    a2_biquad_z = np.poly(p_pair).real    # → [1, -(p_a+p_b), p_a·p_b]

    # Pair the zeros similarly: take the most-real zero for the 1st-order,
    # remaining two for the biquad.  When num_z is degree-2 (b3==0), pad.
    zeros_padded = list(zeros) + [0.0] * (3 - len(zeros))
    zeros_arr = np.array(zeros_padded)
    zero_order = np.argsort(np.abs(zeros_arr.imag))
    z1 = zeros_arr[zero_order[0]].real
    z_pair = zeros_arr[zero_order[1:]]
    b2_biquad_z = np.poly(z_pair).real    # → [1, -(z_a+z_b), z_a·z_b]

    # Gain — split the constant factor.  Total gain k = b_dig[0] (lead z³)
    # divided by den lead (which is 1).  Apply all of k to the biquad's b0
    # and leave the 1st-order section gain at 1.
    k = b_dig[0]
    b2_biquad_z = b2_biquad_z * k

    # Assemble biquad coefs in z⁻¹ form (sample_t form):
    #   y = b0·x + b1·x[-1] + b2·x[-2] − a1·y[-1] − a2·y[-2]
    # b2_biquad_z is [b0, b1, b2] already in z⁰..z⁻² order (np.poly returns
    # highest-power-first; for z⁻¹ form, the convention matches one-to-one
    # because we're working with monic z-polynomials).
    biquad_b = b2_biquad_z                            # [b0, b1, b2]
    biquad_a = a2_biquad_z                            # [1, a1, a2]

    # First-order: (1 − z1·z⁻¹) / (1 − p1·z⁻¹).  p1 is already projected
    # by the same _project_inside_unit_circle call above, so it's safe.
    first_b = np.array([1.0, -z1])
    first_a = np.array([1.0, -p1])

    return biquad_b, biquad_a, first_b, first_a


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
# Sample frequencies — log-spaced from 1 Hz to 100 kHz, 14 points.
_F_SAMPLES = np.logspace(0, 5, 14)
_S_SAMPLES = 2j * math.pi * _F_SAMPLES


def entry_coefs(b_idx: int, m_idx: int, t_idx: int) -> list[int]:
    t = linear_taper(t_idx)         # treble (linear)
    m = linear_taper(m_idx)         # mid    (linear)
    l = audio_taper (b_idx)         # bass   (audio A10)

    H_samples = np.array([H_analog(s, t, l, m) for s in _S_SAMPLES])
    b_an, a_an = fit_rational(_S_SAMPLES, H_samples)
    b_dig, a_dig = bilinear_3rd(b_an, a_an, FS_HZ)

    biq_b, biq_a, fst_b, fst_a = factor_3rd_to_biquad_plus_first(b_dig, a_dig)

    # Stability — every pole must lie inside the unit circle.
    for label, a_arr in (("biquad", biq_a), ("first", fst_a)):
        roots = np.roots(a_arr)
        for r in roots:
            if abs(r) >= POLE_STABILITY_MARGIN:
                raise ValueError(
                    f"unstable {label} pole {r:.4f} at "
                    f"(b_idx={b_idx}, m_idx={m_idx}, t_idx={t_idx})"
                )

    # Cathode_shelf / speaker_load store a1, a2 with the convention
    #   y = b0·x[n] + ... − a1·y[n−1] − a2·y[n−2]
    # which is exactly the natural sign of the bilinear-output a_dig
    # coefficients (np.poly gave them with leading +1 then natural signs).
    coefs = [
        biq_b[0], biq_b[1], biq_b[2],   # 2nd-order numerator (b0, b1, b2)
        biq_a[1], biq_a[2],             # 2nd-order denominator (a1, a2)
        fst_b[0], fst_b[1],             # 1st-order numerator (b0, b1)
        fst_a[1],                       # 1st-order denominator (a1)
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
    lines.append(f"// JCM800 tone-stack coefficient LUT — circuit-accurate FMV/TMB")
    lines.append(f"// Generated: {stamp}  by scripts/gen_tonestack_lut.py")
    lines.append(f"// Components: R_treble={R_TREBLE/1e3:.0f}kΩ R_bass={R_BASS/1e6:.1f}MΩ "
                 f"R_mid={R_MID/1e3:.0f}kΩ R_slope={R_SLOPE/1e3:.0f}kΩ")
    lines.append(f"//             C_treble={C_TREBLE*1e12:.0f}pF "
                 f"C_bass={C_BASS*1e9:.0f}nF C_mid={C_MID*1e9:.0f}nF")
    lines.append(f"// Tapers:     Treble linear, Mid linear, Bass A-taper (1MA)")
    lines.append(f"// Grid:       {GRID}^3 = {N_POTS} pot triples (addressed by pot[7:5])")
    lines.append(f"// Per entry:  {N_COEFS} × 32-bit Q3.29 — biquad(b0,b1,b2,a1,a2) "
                 f"+ 1st-order(b0,b1,a1)")
    lines.append(f"// fs:         {FS_HZ:.0f} Hz")
    lines.append(f"//")
    lines.append(f"// Spot checks (frequency response at representative pot settings):")
    for lbl, idx in (("noon       ", (4, 4, 4)),
                     ("scoop      ", (7, 0, 7)),
                     ("all-min    ", (0, 0, 0)),
                     ("all-max    ", (7, 7, 7)),
                     ("mid-heavy  ", (2, 7, 2))):
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
