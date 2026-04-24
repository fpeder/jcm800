"""
gen_pentode_lut.py  —  JCM800 2203 power-amp LUT + coefficient generator
========================================================================

Companion to scripts/gen_triode_lut.py.  Produces every .mem consumed by
the new EL34 push-pull power stage (pentode_stage.sv, screen_supply.sv,
output_transformer.sv, nfb_network.sv) and emits a standalone SV package
rtl/jcm800_power_pkg.sv so the preamp-side generator (and its existing
jcm800_lut_pkg.sv output) stays untouched.

Design summary
──────────────
  EL34 modelled with Norman Koren's pentode equations.  The Ip surface is
  3-D (Vgk, Vak, Vg2); we collapse it to a 1-D LUT at a nominal (Vak, Vg2)
  and recover dynamic screen sag at RUNTIME via the factorisation

     Ik(Vgk, k·Vg2_nom) ≈ (k^Ex) · Ik(Vgk / k, Vg2_nom)

  so the RTL only needs
      lut input scale : Vgk_eff = Vgk · (Vg2_nom / Vg2)
      lut output scale: Ik_eff  = Ik_lut · (Vg2 / Vg2_nom)^Ex
  both driven by the slow vg2_ratio state from screen_supply.sv.

  Push-pull anti-phase comes from subtracting tube-B's LUT output from
  tube-A's in the output transformer; no LUT content inversion needed —
  the two tubes are identical and the PI already delivered anti-phase
  grid drives (y_pos, y_neg) out of phase_inverter.sv.

  The output transformer is modelled as
      i_prim = Ip_A − Ip_B
      i_hp   = hpf_prim(i_prim)       (1st-order, fc ≈ 10 Hz — Lp/Raa)
      s_sat  = ot_sat_lut(i_hp)       (soft-clip PCHIP curve, no flux state)
      v_sec  = lpf_leak(s_sat)        (1st-order, fc ≈ 15 kHz — Lleak/Raa)
  and the sat LUT is a compressive tanh-shaped curve whose knee lands
  near the push-pull peak current the tubes can produce (≈ 2·Ip_peak).
  This is a deliberate simplification: the flat-knee soft-clip
  approximates the audible character of core saturation without carrying
  a separate flux state through the pipeline.

  NFB / Presence: secondary voltage is registered once (1-sample delay,
  breaks the algebraic loop), scaled by a fixed feedback ratio, tilted
  by a first-order presence shelf, and injected into the PI shared
  cathode in phase_inverter.sv.  Presence pot runs the same level_ctrl
  + log_taper_256.mem as the other 2203 pots.

Output artifacts (lut_out/)
───────────────────────────
  el34_lut.mem / el34_tan.mem         4097 × 24-bit Q1.23  Ik(Vgk) @ nom
  pentode_eg_scale_lut.mem /
  pentode_eg_scale_tan.mem            4097 × 24-bit Q1.23  (k)^Ex scale
                                       indexed by vg2_ratio ∈ [0.25, 1.25]
  ot_sat_lut.mem / ot_sat_tan.mem     4097 × 24-bit Q1.23  core-sat soft-clip
  hpf_prim.mem                        2 × 32-bit Q1.31   (α, γ=0)
  lpf_leak.mem                        1 × 32-bit Q1.31   (β)
  shelf_presence.mem                  3 × 32-bit Q3.29   (b0, b1, a1)

SystemVerilog artifact
──────────────────────
  rtl/jcm800_power_pkg.sv             constants + filenames for the RTL
"""

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq
import pathlib
import datetime
import math


# ─────────────────────────────────────────────────────────────────────────────
# Sample rate (must match preamp's oversampled domain)
# ─────────────────────────────────────────────────────────────────────────────
FS_HZ = 768_000.0


# ─────────────────────────────────────────────────────────────────────────────
# EL34 Koren pentode parameters (common simulator defaults; Duncan's tubes)
# ─────────────────────────────────────────────────────────────────────────────
_EL34 = dict(
    mu  = 11.0,
    Ex  = 1.35,
    Kg1 = 650.0,    # plate current
    Kg2 = 4500.0,   # screen current
    Kp  = 60.0,
    Kvb = 24.0,     # plate-knee (pentode saturation)
)


def koren_pentode(Vgk, Vak, Vg2,
                  mu=11.0, Ex=1.35, Kg1=650.0, Kg2=4500.0, Kp=60.0, Kvb=24.0):
    """
    Return (Ip, Ig2) in Amps.  Numerically stable softplus form.

      E1 = (Vg2/Kp)·ln(1 + exp(Kp·(1/mu + Vgk/Vg2)))
      Ik = E1^Ex / Kg1   (cathode current, pre plate/screen split)
      Ip = Ik · plate_knee(Vak/Kvb)
      Ig2 = Ik · (Kg1/Kg2) · (2 − plate_knee)      # screen grabs more when plate starves

    The 2−plate_knee factor for Ig2 ensures that below the plate knee
    (Vak ≪ Kvb) the screen draws the tube's current instead of the plate
    — the standard SPICE pentode behaviour.
    """
    arg = np.clip(Kp * (1.0 / mu + Vgk / Vg2), -500.0, 500.0)
    E1 = (Vg2 / Kp) * np.log1p(np.exp(arg))
    E1 = np.maximum(E1, 0.0)
    Ik = (E1 ** Ex) / Kg1 * (1.0 + np.sign(E1))
    plate_knee = 1.0 - np.exp(-np.maximum(Vak, 0.0) / Kvb)
    Ip  = Ik * plate_knee
    Ig2 = Ik * (Kg1 / Kg2) * (2.0 - plate_knee)
    return Ip, Ig2


# ─────────────────────────────────────────────────────────────────────────────
# JCM800 2203 power-amp circuit constants
# ─────────────────────────────────────────────────────────────────────────────
#
# Rail voltages from the 2203 schematic; fixed-bias topology.  Screen stoppers
# are per-tube (1 kΩ) to a shared node bypassed by a 47 µF cap — the common
# screen bypass that provides the sag character.  Raa (plate-to-plate primary
# impedance) is 3.4 kΩ as labelled on typical 2203 OT specs.
B_PLUS        = 460.0     # V — main plate supply (loaded)
VG2_NOM       = 440.0     # V — nominal screen voltage (at idle)
VG1_BIAS      = -38.0     # V — fixed-bias grid voltage (idle)
R_SCREEN      = 1.0e3     # Ω — per-tube screen stopper
C_SCREEN      = 47.0e-6   # F  — shared screen bypass cap
RAA           = 3.4e3     # Ω — OT primary (plate-to-plate)
R_PLATE_LOAD  = RAA / 4.0 # Ω — each tube's effective AC load in AB1

# Main HT (B+) rail dynamics.  The 2203's B+ comes through the rectifier,
# choke, and reservoir/filter caps; under sustained class-AB drive the rail
# droops because the reservoir cap can't refill fast enough through the
# series DCR.  Dominant time constant is R_HT · C_HT — we model the whole
# supply network as a single leaky integrator of that τ, driven by the
# push-pull plate-current sum |Ip_a|+|Ip_b|.  PEAK_IP_A sets the
# "ip_sum_norm = 1.0" reference so steady-state sag at peak drive lands at
# R_HT · PEAK_IP_A / B_PLUS.  Default numbers target τ ≈ 140 ms and
# ≈ 10 % peak droop — the canonical "cranked 2203 bloom".
R_HT          = 300.0     # Ω — effective choke + rectifier series DCR
C_HT          = 470.0e-6  # F — reservoir/filter cap
PEAK_IP_A     = 0.15      # A — sum-of-Ip full-scale reference (class-AB peak)

# Output transformer (approximate; these drive the filter coefficients)
LP_PRIMARY    = 50.0      # H  — primary inductance (off-load)
LLEAK         = 30.0e-3   # H  — total leakage referred to primary
TURNS_RATIO   = math.sqrt(RAA / 8.0)   # 3.4k : 8 Ω → Nps ≈ 20.6

# NFB loop
NFB_RATIO     = 0.045     # dimensionless — secondary voltage fed back to PI
                          # cathode.  Chosen so the 2203's characteristic
                          # ~27 dB loop gain lands at reasonable signal level.

# Presence shelf: peaks HF in the NFB path.  Maximum shelf ratio ~6 dB.
PRESENCE_FC_HZ = 800.0    # presence corner in the NFB return path
PRESENCE_MAX_LIFT = 2.0   # HF/LF ratio at presence=max → 6 dB

# Grid-diode soft-clip on the EL34 grid (same model as preamp).  Is and Vt
# are lumped-model defaults — the 220 kΩ source impedance from the PI
# dominates the clip softness under hard overdrive.
GRID_IS_A = 1.0e-6
GRID_VT_V = 0.6
GRID_DRIVE_Z = 220.0e3    # V3 coupling-cap / EL34 grid-leak impedance

# LUT sweep extent: drive ±dV past bias.  EL34 enters grid conduction at
# Vgk ≈ 0, so from a −38 V bias we want dV ≳ 42 V to capture a couple of
# Vt of post-rail shoulder (same philosophy as the preamp LUTs).
D_VGK_PAST_ZERO = 3.0     # V past Vgk=0 to capture grid-diode shoulder


# ─────────────────────────────────────────────────────────────────────────────
# DC operating point (single-ended bias-class-AB idle)
# ─────────────────────────────────────────────────────────────────────────────

def solve_el34_dc():
    """
    Compute EL34 idle current given fixed-bias Vg1 and nominal rails.
    Returns (Ip_q, Ig2_q, Vak_q, Ik_q) at Vgk = VG1_BIAS, Vg2 = VG2_NOM.
    """
    Vak_q = B_PLUS - _estimate_idle_ir_drop()       # rough (no OT DC drop)
    Ip_q, Ig2_q = koren_pentode(VG1_BIAS, Vak_q, VG2_NOM, **_EL34)
    Ik_q = Ip_q + Ig2_q
    return Ip_q, Ig2_q, Vak_q, Ik_q


def _estimate_idle_ir_drop():
    """
    OT primary DC resistance + speaker return is negligible at idle; we
    take Vak_q ≈ B+ for the DC solve.  Audio-band behaviour is governed
    by R_PLATE_LOAD, not by this constant.
    """
    return 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Large-signal solver — one EL34, full load line, grid-diode soft clip
# ─────────────────────────────────────────────────────────────────────────────

def _apply_grid_diode(Vgk_drive, grid_R=GRID_DRIVE_Z,
                      Is=GRID_IS_A, Vt=GRID_VT_V, iters=14):
    """Soft-clip Vgk through the EL34 grid-leak drive network."""
    if Vgk_drive <= 0.0:
        return Vgk_drive
    Vgk = Vgk_drive
    for _ in range(iters):
        x = min(max(Vgk / Vt, -30.0), 30.0)
        Ig = Is * (math.exp(x) - 1.0)
        Vgk_new = Vgk_drive - Ig * grid_R
        if abs(Vgk_new - Vgk) < 1e-7:
            Vgk = Vgk_new
            break
        Vgk = 0.5 * Vgk + 0.5 * Vgk_new
    return Vgk


def solve_ip_at_vin(Vin, Ip_q, Vak_q, Ik_q):
    """
    Return (Ip, Ig2) when the grid is driven by Vin volts above the
    fixed bias.  AC load line on the plate: ΔVak = −(Ip − Ip_q)·R_PLATE_LOAD.
    Screen is frozen at VG2_NOM for the LUT sweep (runtime scaling handles sag).
    """
    Vgk_drive = VG1_BIAS + Vin
    Vgk = _apply_grid_diode(Vgk_drive)

    def residual(Ip):
        Vak = Vak_q - (Ip - Ip_q) * R_PLATE_LOAD
        if Vak < 5.0:
            Vak = 5.0
        Ip_calc, _ = koren_pentode(Vgk, Vak, VG2_NOM, **_EL34)
        return Ip_calc - Ip

    Ip_lo = 1e-9
    Ip_hi = max(Ip_q * 20.0, 1e-2)
    for _ in range(8):
        if residual(Ip_lo) * residual(Ip_hi) < 0.0:
            break
        Ip_hi *= 2.0
    try:
        Ip = brentq(residual, Ip_lo, Ip_hi, xtol=1e-10, rtol=1e-9)
    except ValueError:
        Ip = Ip_q
    Ip = max(Ip, 0.0)
    Vak = max(Vak_q - (Ip - Ip_q) * R_PLATE_LOAD, 5.0)
    _, Ig2 = koren_pentode(Vgk, Vak, VG2_NOM, **_EL34)
    return Ip, Ig2


# ─────────────────────────────────────────────────────────────────────────────
# Non-uniform sample grid (same as preamp)
# ─────────────────────────────────────────────────────────────────────────────

def nonuniform_grid(N=32769, alpha=0.55):
    assert N % 2 == 1
    u = np.linspace(0., 1., N)
    return np.sign(u - 0.5) * np.abs(2. * u - 1.) ** alpha


# ─────────────────────────────────────────────────────────────────────────────
# LUT builder (bias-folded Q1.23 + PCHIP tangents, pattern from gen_triode_lut.py)
# ─────────────────────────────────────────────────────────────────────────────

def _fine_to_lut(x_fine, y_fine, y_bias, label,
                 N_lut=4097, out_bits=24, force_anti_phase=False,
                 pre_scale_by_gss=False):
    """
    Build a bias-folded Q1.23 LUT + PCHIP tangents from a fine sweep.

    pre_scale_by_gss=False (default for the pentode LUT): store the raw
    span-normalised curve (peak ≈ ±0.5) and let the post-LUT G_stage pick
    up the small-signal digital gain.  This lets the EL34 path run with
    a non-saturating LUT rail (plenty of headroom into the PCHIP) and
    recovers audible loudness — with the Gss pre-scaling inherited from
    the preamp generator, each tube attenuated by ≈ 5× and the push-pull
    pair + OT filters stacked ~30 dB of net loss into v_sec.

    pre_scale_by_gss=True: legacy preamp behaviour; |Gss| is baked into
    the LUT slope so G_STAGE collapses to the phase sign only.  Kept
    here for future use if someone wants a preamp-style pentode LUT.
    """
    Nm1 = N_lut - 1
    assert (Nm1 & (Nm1 - 1)) == 0

    span_raw = float(y_fine[-1] - y_fine[0])
    if abs(span_raw) < 1e-12:
        raise ValueError(f'{label}: degenerate span {span_raw:.3e}')
    span_abs = abs(span_raw)

    h_linear = (y_fine - y_bias) / span_abs
    mid = len(x_fine) // 2
    h_linear[mid] = 0.0
    slope_linear = float(PchipInterpolator(x_fine, h_linear).derivative()(0.0))
    Gss = abs(slope_linear)     # slope already per-unit-of-x (x ∈ [−1, +1])

    if pre_scale_by_gss:
        h_fine = h_linear * Gss
    else:
        h_fine = h_linear.copy()
    h_fine[mid] = 0.0
    if force_anti_phase:
        h_fine = -h_fine

    pchip = PchipInterpolator(x_fine, h_fine)
    x_lut = np.linspace(-1.0, 1.0, N_lut)
    h_lut = pchip(x_lut)
    step = 2.0 / (N_lut - 1)
    dh_lut = pchip.derivative()(x_lut) * step

    slope_at_zero = float(pchip.derivative()(0.0))
    # When pre_scale is off, pick G_stage so digital gain at Q-point is ≈1.
    # When on, G collapses to the phase sign (same as preamp).
    if pre_scale_by_gss:
        G_stage = float(np.sign(slope_at_zero)) if slope_at_zero != 0.0 else 1.0
    else:
        G_stage = 1.0 / slope_at_zero if abs(slope_at_zero) > 1e-9 else 1.0
    tube_Gss = Gss * np.sign(slope_linear) * (1.0 if G_stage >= 0 else -1.0)

    scale = (1 << (out_bits - 1)) - 1
    lut_q = np.clip(np.round(h_lut  * scale), -(scale + 1), scale).astype(np.int32)
    tan_q = np.clip(np.round(dh_lut * scale), -(scale + 1), scale).astype(np.int32)

    flat_hi = lut_q == scale
    flat_lo = lut_q == -(scale + 1)
    tan_q[flat_hi] = 0
    tan_q[flat_lo] = 0
    for idx in np.where(np.diff(flat_hi.astype(np.int8)) == 1)[0]:
        tan_q[idx] = 0
    for idx in np.where(np.diff(flat_lo.astype(np.int8)) == -1)[0]:
        if idx + 1 < N_lut:
            tan_q[idx + 1] = 0
    for idx in np.where(np.diff(flat_hi.astype(np.int8)) == -1)[0]:
        if idx + 1 < N_lut:
            tan_q[idx + 1] = 0
    for idx in np.where(np.diff(flat_lo.astype(np.int8)) == 1)[0]:
        tan_q[idx] = 0

    mid_lut = N_lut // 2
    assert lut_q[mid_lut] == 0, f'{label}: bias-fold broken at mid ({lut_q[mid_lut]})'

    meta = dict(
        label=label, N_lut=N_lut, out_bits=out_bits,
        span_abs=span_abs, G_stage=G_stage, slope_at_zero=slope_at_zero,
        tube_Gss=tube_Gss, digital_gain=slope_at_zero * G_stage,
        flat_hi_entries=int(np.sum(flat_hi)),
        flat_lo_entries=int(np.sum(flat_lo)),
    )
    return lut_q, tan_q, meta


# ─────────────────────────────────────────────────────────────────────────────
# Q-format helpers (duplicated from gen_triode_lut.py for standalone usability)
# ─────────────────────────────────────────────────────────────────────────────

def _to_q_signed(x, q_int_bits, q_frac_bits, total_bits):
    scale = 1 << q_frac_bits
    vmax = (1 << (total_bits - 1)) - 1
    vmin = -(1 << (total_bits - 1))
    v = int(round(float(x) * scale))
    return max(vmin, min(vmax, v))


def _hex_signed(v, total_bits):
    mask = (1 << total_bits) - 1
    w = (total_bits + 3) // 4
    return f'{v & mask:0{w}X}'


def _write_lut_mem(path, data, bits, header):
    mask = (1 << bits) - 1
    w = (bits + 3) // 4
    lines = [f'{int(v) & mask:0{w}X}' for v in data]
    path.write_text(header + '\n'.join(lines) + '\n')


# ─────────────────────────────────────────────────────────────────────────────
# Pentode LUT (Ip transfer, bias-folded)
# ─────────────────────────────────────────────────────────────────────────────

def generate_el34_lut(N_lut=4097, N_fine=32769):
    """
    Build Ip(Vin) LUT for ONE EL34 at nominal (Vak, Vg2).  Same LUT is used
    for tube A and tube B in the push-pull — the grids are driven anti-phase
    by the PI, so subtracting Ip_B from Ip_A in output_transformer.sv
    recovers push-pull operation without any LUT content inversion.
    """
    Ip_q, Ig2_q, Vak_q, Ik_q = solve_el34_dc()
    # Grid-swing half-range: bias voltage magnitude + headroom past grid-diode
    dV = abs(VG1_BIAS) + D_VGK_PAST_ZERO
    x_fine = nonuniform_grid(N_fine)
    Vin_fine = x_fine * dV
    Ip_fine  = np.empty_like(Vin_fine)
    for i, vin in enumerate(Vin_fine):
        Ip_fine[i], _ = solve_ip_at_vin(float(vin), Ip_q, Vak_q, Ik_q)
    # Bias-fold at Vin=0 (Ip_q).  Ip rises with Vgk, so span is positive.
    # pre_scale_by_gss=True bakes |Gss| into the LUT slope (same convention
    # as the triode preamp), so the post-LUT G_stage collapses to ±1.0 and
    # the LUT output is naturally bounded by the physical pentode knee
    # rather than overrun-and-saturate at the Q1.23 digital rail.  This
    # fixes (a) hard-clipping fart at high master volume and (b) the
    # |ip_out|-driven Ig2 over-estimate that was causing excessive screen
    # sag — both consequences of the previous pre_scale=False rail-overrun.
    lut_q, tan_q, meta = _fine_to_lut(
        x_fine, Ip_fine, Ip_q, 'el34_ip', N_lut=N_lut,
        pre_scale_by_gss=True,
    )
    meta.update(dV=dV, Ip_q=Ip_q, Ig2_q=Ig2_q, Vak_q=Vak_q, Ik_q=Ik_q)
    return lut_q, tan_q, meta


# ─────────────────────────────────────────────────────────────────────────────
# Screen-scaling side table (k^Ex for k ∈ [0.25, 1.25])
# ─────────────────────────────────────────────────────────────────────────────
#
# Indexed at runtime by vg2_ratio (Q2.14).  For a practical BRAM load we
# reuse triode_bram (4097 × Q1.23) and map vg2_ratio ∈ [0.25, 1.25] onto
# addr [0, 4096].  The table holds the output scale k^Ex in Q1.23 — which
# exceeds 1.0 at k > 1, so values are clamped at full-scale (the dynamic
# range above 1 is compressed gracefully by the flat LUT shoulder; 1.0
# is the common case and lands safely at ≈ 0x7FFFFF).

def generate_eg_scale_lut(N_lut=4097, k_min=0.25, k_max=1.25):
    Ex = _EL34['Ex']
    x_fine = np.linspace(-1.0, 1.0, 32769)
    k_fine = k_min + (k_max - k_min) * (x_fine + 1.0) * 0.5
    scale_fine = k_fine ** Ex
    # Bias-fold around k=1.0 (addr 2048) so the runtime table acts like the
    # other LUTs: f(k=1.0) = 0.  We add 1.0 back into the scaling in RTL.
    bias_idx = np.argmin(np.abs(k_fine - 1.0))
    scale_bias = float(scale_fine[bias_idx])
    # Force exact zero at the midpoint
    shifted = scale_fine - scale_bias
    # Resample via PCHIP for smoothness
    pchip = PchipInterpolator(x_fine, shifted)
    x_lut = np.linspace(-1.0, 1.0, N_lut)
    h_lut = pchip(x_lut)
    step = 2.0 / (N_lut - 1)
    dh_lut = pchip.derivative()(x_lut) * step

    # The LUT holds delta = (k^Ex − 1.0).  Peak magnitude at k=1.25 is
    # 1.25^1.35 − 1 ≈ 0.354; at k=0.25 is 0.25^1.35 − 1 ≈ −0.846.  Both
    # fit inside Q1.23 without saturation.
    scale = (1 << 23) - 1
    lut_q = np.clip(np.round(h_lut  * scale), -(scale + 1), scale).astype(np.int32)
    tan_q = np.clip(np.round(dh_lut * scale), -(scale + 1), scale).astype(np.int32)
    # Force bias-fold at centre
    lut_q[N_lut // 2] = 0
    meta = dict(
        k_min=k_min, k_max=k_max, Ex=Ex, scale_bias=scale_bias,
        peak_pos=float(np.max(h_lut)),
        peak_neg=float(np.min(h_lut)),
    )
    return lut_q, tan_q, meta


# ─────────────────────────────────────────────────────────────────────────────
# Output transformer core-saturation LUT
# ─────────────────────────────────────────────────────────────────────────────
#
# Soft-clip shape (tanh-like) applied to the current-difference drive, with
# knee placed near the push-pull peak swing.  The real OT saturates flux,
# not current, but at this level of modelling the compressive character is
# similar and avoids a separate flux state.  Knee tuned by hand so a 0.75
# fractional drive input compresses to ≈ 0.70 output — audibly breaks up
# at extreme volumes, transparent at normal listening levels.

def generate_ot_sat_lut(N_lut=4097, knee=0.85):
    x_fine = np.linspace(-1.0, 1.0, 32769)
    # tanh normalised so dy/dx(0) = 1 and asymptote at ±knee
    y_fine = knee * np.tanh(x_fine / knee)
    pchip = PchipInterpolator(x_fine, y_fine)
    x_lut = np.linspace(-1.0, 1.0, N_lut)
    h_lut = pchip(x_lut)
    step = 2.0 / (N_lut - 1)
    dh_lut = pchip.derivative()(x_lut) * step

    scale = (1 << 23) - 1
    lut_q = np.clip(np.round(h_lut  * scale), -(scale + 1), scale).astype(np.int32)
    tan_q = np.clip(np.round(dh_lut * scale), -(scale + 1), scale).astype(np.int32)
    lut_q[N_lut // 2] = 0
    meta = dict(knee=knee, peak=float(np.max(h_lut)))
    return lut_q, tan_q, meta


# ─────────────────────────────────────────────────────────────────────────────
# IIR coefficient emitters (same on-disk format as gen_triode_lut.py)
# ─────────────────────────────────────────────────────────────────────────────

def lpf_beta(fc, fs=FS_HZ):
    return 1.0 - math.exp(-2.0 * math.pi * fc / fs)


def hpf_alpha(tau_s, fs=FS_HZ):
    return 1.0 - 1.0 / (tau_s * fs)


def write_lpf_mem(path, fc, timestamp, note):
    β = lpf_beta(fc)
    β_q = _to_q_signed(β, 1, 31, 32)
    hdr = (
        f'// JCM800 power-amp LPF — {note}\n'
        f'// Generated: {timestamp}\n'
        f'// Formula:   y[n] = y[n−1] + β·(x[n] − y[n−1])\n'
        f'// fc = {fc:.2f} Hz   fs = {FS_HZ:.0f} Hz\n'
        f'// β = {β:.12f}   Q1.31 signed 32-bit → 0x{_hex_signed(β_q, 32)}\n'
        f'// Width: 32-bit\n'
        f'//\n'
    )
    path.write_text(hdr + _hex_signed(β_q, 32) + '\n')


def write_hpf_mem(path, tau_s, timestamp, note):
    """Emit iir_hpf1-compatible file: α then γ_atk=γ_rel=0 (bias tracker off)."""
    α = hpf_alpha(tau_s)
    α_q = _to_q_signed(α, 1, 31, 32)
    γ_q = 0
    fc = 1.0 / (2.0 * math.pi * tau_s)
    hdr = (
        f'// JCM800 power-amp HPF — {note}\n'
        f'// Generated: {timestamp}\n'
        f'// Formula:   y[n] = α·(y[n−1] + x[n] − x[n−1])     (γ_atk=γ_rel=0: bias tracker off)\n'
        f'// τ = {tau_s*1e3:.4f} ms   fc = {fc:.4f} Hz   fs = {FS_HZ:.0f} Hz\n'
        f'// α = {α:.12f}   Q1.31 signed 32-bit → 0x{_hex_signed(α_q, 32)}\n'
        f'// γ_atk = γ_rel = 0 (disabled; OT primary has no grid-diode bias pumping)\n'
        f'// Width: 32-bit.  Addr 0: α, Addr 1: γ_atk, Addr 2: γ_rel\n'
        f'//\n'
    )
    path.write_text(hdr
                    + _hex_signed(α_q, 32) + '\n'
                    + _hex_signed(γ_q, 32) + '\n'
                    + _hex_signed(γ_q, 32) + '\n')


def shelf_biquad_presence(fc, max_lift, fs=FS_HZ):
    """
    First-order high-shelf in biquad form (b2=a2=0).  Lift in LINEAR ratio
    (2.0 ≈ +6 dB).  Bilinear from H(s) = (1 + s·τz)/(1 + s·τp):
        τz = 1 / (2π·fc)
        τp = τz / max_lift
    Post-shelf gain  : DC=1.0, HF=max_lift.
    Presence pot modulates the shelf magnitude via the log_taper_256.mem
    attenuator OUTSIDE this block, so we bake the MAXIMUM presence lift
    into the shelf coefficients.
    """
    tau_z = 1.0 / (2.0 * math.pi * fc)
    tau_p = tau_z / max_lift
    K = 2.0 * fs
    num0 = 1.0 + K * tau_z
    num1 = 1.0 - K * tau_z
    den0 = 1.0 + K * tau_p
    den1 = 1.0 - K * tau_p
    b0 = num0 / den0
    b1 = num1 / den0
    a1 = den1 / den0
    return b0, b1, a1


def write_shelf_mem(path, b0, b1, a1, timestamp, note, meta_lines):
    b0_q = _to_q_signed(b0, 3, 29, 32)
    b1_q = _to_q_signed(b1, 3, 29, 32)
    a1_q = _to_q_signed(a1, 3, 29, 32)
    hdr = [
        f'// JCM800 presence shelf — {note}',
        f'// Generated: {timestamp}',
        f'// Formula:   y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]   (a0=1, b2=a2=0)',
    ] + [f'// {m}' for m in meta_lines] + [
        f'// Q3.29 signed 32-bit; addr order: b0, b1, a1',
        f'// b0 = {b0:+.12f} → 0x{_hex_signed(b0_q, 32)}',
        f'// b1 = {b1:+.12f} → 0x{_hex_signed(b1_q, 32)}',
        f'// a1 = {a1:+.12f} → 0x{_hex_signed(a1_q, 32)}',
        f'// Width: 32-bit',
        f'//',
    ]
    body = '\n'.join([_hex_signed(b0_q, 32),
                      _hex_signed(b1_q, 32),
                      _hex_signed(a1_q, 32)])
    path.write_text('\n'.join(hdr) + '\n' + body + '\n')


# ─────────────────────────────────────────────────────────────────────────────
# Screen-supply RC coefficient computation
# ─────────────────────────────────────────────────────────────────────────────
#
# Discrete leaky integrator at fs for the shared screen supply:
#   vg2_ratio[n+1] = vg2_ratio[n] + α · (1 − vg2_ratio[n]) − β · Ig2_sum_norm
#
# α comes straight from the screen-stopper RC:
#   α = 1 / (fs · R_SCREEN · C_SCREEN)
#
# β converts a Q1.23 "normalised sum-of-screen-currents" back into the
# vg2_ratio domain.  With the convention that Ig2_sum_norm = 1.0 corresponds
# to PEAK_IG2_A physical amps, the steady-state sag at peak current is
# ΔVg2 = Ig2·R_SCREEN; we want β = α · (R_SCREEN · PEAK_IG2_A / VG2_NOM)
# so that setting Ig2_sum_norm=1.0 drives vg2_ratio to the expected sag.

# The RTL estimates ig2_tube ≈ IG2_RATIO · |Ip_lut_q23| where Ip_lut_q23 is
# the bias-folded plate-current LUT output (Q1.23 ∈ [−1, +1]).  Full-scale
# LUT output equals the span_abs physical plate-current swing — so per-tube
# ig2 norm peaks at IG2_RATIO·1 ≈ 0.144, and the two-tube sum peaks around
# 0.29 in normal drive.  We pick PEAK_IG2_A so that Ig2_sum_norm = 1.0 maps
# to a "fully clipping" sag of ≈ 25% — leaves steady-state sag at ≈ 7% under
# typical overdrive.  The earlier 0.22 value (β/α ≈ 0.5) produced audible
# dropouts on attack transients because Ig2_sum could momentarily drag Vg2
# down 50 %+ within an attack window; halving PEAK_IG2_A halves β so sag
# stays present (still audible sustain compression) without collapsing the
# pentode gain on fast transients — matching the tight, steep 2203 feel.
# β/α = R_SCREEN · PEAK_IG2_A / VG2_NOM, so PEAK_IG2_A = 0.11 A gives ≈ 25%
# sag at Ig2_norm=1.0.
PEAK_IG2_A = 0.11         # A — nominal "sum" full-scale screen current


def screen_supply_coeffs():
    α = 1.0 / (FS_HZ * R_SCREEN * C_SCREEN)
    # Discrete steady-state of  vg2[n+1] = vg2[n] + α(1−vg2) − β·ig2_norm  is
    # vg2_ss = 1 − (β/α)·ig2_norm, so to land at vg2_ss = 1−ratio_sag_full
    # when ig2_norm = 1 we want β/α = ratio_sag_full directly.
    ratio_sag_full = (R_SCREEN * PEAK_IG2_A) / VG2_NOM
    β = α * ratio_sag_full
    return α, β


def ht_supply_coeffs():
    """HT (B+) rail sag integrator.  Same α/β factorisation as the screen
    supply: α fixes the recovery RC, β/α is the steady-state droop fraction
    at ip_sum_norm = 1.0.  Input to the integrator is |Ip_a|+|Ip_b| summed
    at Q1.23, so "ip_sum_norm = 1.0" means the 25-bit sum has hit Q1.23
    full-scale — the same convention screen_supply.sv uses for its
    ig2_sum_q1_23 port."""
    α = 1.0 / (FS_HZ * R_HT * C_HT)
    ratio_sag_full = (R_HT * PEAK_IP_A) / B_PLUS
    β = α * ratio_sag_full
    return α, β


# ─────────────────────────────────────────────────────────────────────────────
# SV package emitter
# ─────────────────────────────────────────────────────────────────────────────

def write_power_pkg(pkg_path, timestamp, dc, el34_meta, eg_meta, ot_meta,
                    alpha_screen, beta_screen, alpha_ht, beta_ht,
                    fc_prim, fc_leak, presence_fc):
    pkg_path = pathlib.Path(pkg_path)
    pkg_path.parent.mkdir(parents=True, exist_ok=True)

    # Screen supply: α and β in Q1.31 (they are tiny fractional values).
    alpha_q = _to_q_signed(alpha_screen, 1, 31, 32)
    beta_q  = _to_q_signed(beta_screen,  1, 31, 32)

    # HT supply: same format as the screen pair.
    ht_alpha_q = _to_q_signed(alpha_ht, 1, 31, 32)
    ht_beta_q  = _to_q_signed(beta_ht,  1, 31, 32)

    # Ig2 estimate: Kg1/Kg2 ≈ 0.144 — fraction of folded |Ip| that we treat
    # as Ig2 for the screen-supply feedback.  Q1.23.
    ig2_ratio = _EL34['Kg1'] / _EL34['Kg2']
    ig2_ratio_q = _to_q_signed(ig2_ratio, 1, 23, 24)

    # Push-pull current peak — used by output_transformer to know what "1.0"
    # means in i_prim (Ip_A − Ip_B).  Here we don't need it at compile time;
    # the LUT already bakes the span into Q1.23.

    # NFB factor in Q4.20 (matches G_STAGE convention; the gain_stage ×G
    # multiplier is proven code, so nfb_network.sv uses the same block).
    nfb_q = _to_q_signed(NFB_RATIO, 4, 20, 32)

    # Presence bias: when presence_pot_pos = 0, the shelf output is scaled
    # down to near-zero so the NFB is effectively flat-shelved (no HF lift).
    # We use level_ctrl.sv + log_taper_256.mem to attenuate the shelf output.

    # PI NFB drive scale: the secondary Q1.23 sample enters nfb_network scaled
    # by NFB_RATIO; shelf+level_ctrl then modulate it.

    # Post-LUT output gain for pentode_stage.  The EL34 LUT is stored raw
    # (no Gss pre-scaling — see _fine_to_lut with pre_scale_by_gss=False),
    # so G_stage carries the small-signal digital-gain normalisation.
    # meta['G_stage'] computes this as 1/slope_at_zero so small-signal LUT
    # output passes with unity gain; hard drive still clips at the LUT rails
    # (|h_linear| ≈ 0.5 · G ≤ 1.0 = Q1.23 rail).  Both tubes share the same
    # LUT — the PI delivers anti-phase grid drives and Ip_A − Ip_B in the OT
    # recovers push-pull operation.
    g_el34_q = _to_q_signed(el34_meta['G_stage'], 4, 20, 32)

    # Push-pull sum scale: with the EL34 LUT now Gss-pre-scaled, each tube's
    # |ip_out| peaks at ~|Gss|·max(h_linear) ≈ 0.4·0.92 ≈ 0.37 in Q1.23,
    # so the anti-phase diff peaks at ≈ ±0.40.  We scale by 2.0 (not 0.5)
    # to put the post-OT signal back near the Q1.23 rail with comfortable
    # headroom (peak ≈ 0.80) — recovers most of the loudness lost to the
    # LUT pre-scale change AND restores NFB loop gain so the 11 Hz OT-
    # primary HPF stays well-damped (low-frequency thump on transients
    # was the residual "fart" character).
    pp_halfer_q = _to_q_signed(2.0, 4, 20, 32)

    src = pathlib.Path(__file__).name

    body = f"""//===========================================================================
// jcm800_power_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.
//   Produced by: scripts/{src}
//   Timestamp:   {timestamp}
//
//   Power-stage constants + .mem filenames for the new EL34 push-pull +
//   output transformer + NFB/Presence subtree.  Generated alongside the
//   preamp-side jcm800_lut_pkg; the two packages are independent (no
//   cross-references) so either can be regenerated without touching the
//   other.
//
//   DC operating point (nominal rails B+={B_PLUS:.1f} V, Vg2={VG2_NOM:.1f} V,
//   fixed bias Vg1={VG1_BIAS:.1f} V):
//     Ip_q  = {dc[0]*1e3:.3f} mA        Vak_q = {dc[2]:.2f} V
//     Ig2_q = {dc[1]*1e3:.3f} mA        Ik_q  = {dc[3]*1e3:.3f} mA
//   EL34 span_abs = {el34_meta['span_abs']*1e3:.3f} mA (bias-fold peak-to-peak)
//   Tube |Gss| = {abs(el34_meta['tube_Gss']):.3f} (baked into LUT slope)
//===========================================================================
package jcm800_power_pkg;

    // ----- EL34 push-pull grid→plate-current LUT (shared for both tubes) ----
    localparam string EL34_LUT_FILE = "el34_lut.mem";
    localparam string EL34_TAN_FILE = "el34_tan.mem";

    // Post-LUT gain (Q4.20).  LUT already bakes span and natural slope
    // into Q1.23; G collapses to the phase sign.  Both tubes use +1.0 —
    // the PI delivers anti-phase grid drives, so Ip_A − Ip_B in the OT
    // gives the correct push-pull difference without LUT inversion.
    localparam int                 SHIFT_G_EL34 = 20;
    localparam logic signed [31:0] G_EL34_Q4_20 = 32'h{g_el34_q & 0xFFFFFFFF:08X};

    // ----- Screen-scaling side table (Vg2/Vg2_nom)^Ex − 1 ------------------
    // Emitted for future use; the current pentode_stage.sv implementation
    // applies a linear vg2_ratio multiplier instead of k^Ex for simplicity.
    //   localparam string EG_SCALE_LUT_FILE = "pentode_eg_scale_lut.mem";
    //   localparam string EG_SCALE_TAN_FILE = "pentode_eg_scale_tan.mem";
    //   localparam real   EG_K_MIN          = {eg_meta['k_min']};
    //   localparam real   EG_K_MAX          = {eg_meta['k_max']};

    // ----- OT core-saturation LUT -----------------------------------------
    localparam string OT_SAT_LUT_FILE = "ot_sat_lut.mem";
    localparam string OT_SAT_TAN_FILE = "ot_sat_tan.mem";

    // ----- Screen supply RC (shared Vg2 node) -----------------------------
    // vg2_ratio[n+1] = vg2_ratio[n] + α·(1 − vg2_ratio[n]) − β·ig2_sum
    // α, β are Q1.31 signed (tiny fractional values).
    localparam logic signed [31:0] SCREEN_ALPHA_Q1_31 = 32'h{alpha_q & 0xFFFFFFFF:08X};  // α = {alpha_screen:.6e}
    localparam logic signed [31:0] SCREEN_BETA_Q1_31  = 32'h{beta_q  & 0xFFFFFFFF:08X};  // β = {beta_screen:.6e}

    // ----- HT (B+) supply RC ----------------------------------------------
    // ht_ratio[n+1] = ht_ratio[n] + α_HT·(1 − ht_ratio[n]) − β_HT·ip_sum
    //   Driven by |Ip_a|+|Ip_b| at Q1.23 — identical integrator to the
    //   screen supply but with its own τ (R_HT·C_HT) and peak-reference
    //   scaling (R_HT·PEAK_IP_A / B_PLUS).  ht_ratio drops from 1.0 toward
    //   (1 − R_HT·PEAK_IP_A/B_PLUS) under sustained drive and is combined
    //   with vg2_ratio in power_amp.sv to form the pentode output scale.
    localparam logic signed [31:0] HT_ALPHA_Q1_31 = 32'h{ht_alpha_q & 0xFFFFFFFF:08X};  // α_HT = {alpha_ht:.6e}
    localparam logic signed [31:0] HT_BETA_Q1_31  = 32'h{ht_beta_q  & 0xFFFFFFFF:08X};  // β_HT = {beta_ht:.6e}

    // Ig2 estimate: Ig2_tube ≈ IG2_RATIO_Q1_23 · |Ip_tube| (Q1.23 signed).
    // Derived from Koren Kg1/Kg2 for EL34 ≈ {ig2_ratio:.4f}.
    localparam logic signed [23:0] IG2_RATIO_Q1_23 = 24'h{ig2_ratio_q & 0xFFFFFF:06X};

    // ----- Output transformer filter coefficient files --------------------
    localparam string OT_HPF_FILE = "hpf_prim.mem";    // fc ≈ {fc_prim:.1f} Hz (Lp/Raa)
    localparam string OT_LPF_FILE = "lpf_leak.mem";    // fc ≈ {fc_leak:.0f} Hz (Lleak/Raa)

    // Push-pull difference halver: i_prim = (Ip_A − Ip_B) · PP_HALF so the
    // Q1.23 range is preserved when the two tubes are driven to opposite rails.
    localparam int                 SHIFT_PP = 20;
    localparam logic signed [31:0] PP_HALF_Q4_20 = 32'h{pp_halfer_q & 0xFFFFFFFF:08X};

    // ----- NFB / Presence --------------------------------------------------
    // v_sec → [1-sample z⁻¹] → ×NFB_Q4_20 → presence shelf → level_ctrl
    //     → injected at V3 shared cathode inside phase_inverter.sv.
    localparam logic signed [31:0] NFB_Q4_20         = 32'h{nfb_q & 0xFFFFFFFF:08X};
    localparam int                 SHIFT_NFB         = 20;
    localparam string              PRESENCE_SHELF_FILE = "shelf_presence.mem";
    localparam string              PRESENCE_POT_FILE   = "log_taper_256.mem";
    localparam real                PRESENCE_FC_HZ      = {presence_fc:.2f};

endpackage
"""
    pkg_path.write_text(body)
    return pkg_path


# ─────────────────────────────────────────────────────────────────────────────
# Top-level export driver
# ─────────────────────────────────────────────────────────────────────────────

def export_all(out_dir='lut_out', pkg_path='rtl/jcm800_power_pkg.sv', verbose=True):
    out_path = pathlib.Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # EL34 Ip LUT
    lut_q, tan_q, el34_meta = generate_el34_lut()
    dc = (el34_meta['Ip_q'], el34_meta['Ig2_q'], el34_meta['Vak_q'], el34_meta['Ik_q'])
    lut_hdr = (
        f'// JCM800 EL34 — Ip(Vin) LUT, bias-folded Q1.23\n'
        f'// Generated: {timestamp}\n'
        f'// DC bias: Vg1={VG1_BIAS:.2f} V  Vg2={VG2_NOM:.1f} V  Vak={el34_meta["Vak_q"]:.2f} V\n'
        f'//   Ip_q  = {el34_meta["Ip_q"]*1e3:.4f} mA  '
        f'Ig2_q = {el34_meta["Ig2_q"]*1e3:.4f} mA  '
        f'Ik_q  = {el34_meta["Ik_q"]*1e3:.4f} mA\n'
        f'// Sweep:  Vin ∈ ±{el34_meta["dV"]:.2f} V; dV includes {D_VGK_PAST_ZERO} V past Vgk=0.\n'
        f'// |ΔIp_span| = {el34_meta["span_abs"]*1e3:.4f} mA   |Gss| = {abs(el34_meta["tube_Gss"]):.3f}\n'
        f'// Digital gain at Q-point = {el34_meta["digital_gain"]:+.3f}\n'
        f'// Rails: flat_hi={el34_meta["flat_hi_entries"]}  flat_lo={el34_meta["flat_lo_entries"]}\n'
        f'// addr 2048 ↔ Vin=0 ↔ lut=0  (bias-fold)\n'
        f'//\n'
    )
    tan_hdr = lut_hdr.replace('Ip(Vin) LUT', 'Ip PCHIP tangents')
    _write_lut_mem(out_path / 'el34_lut.mem', lut_q, 24, lut_hdr)
    _write_lut_mem(out_path / 'el34_tan.mem', tan_q, 24, tan_hdr)

    # Screen scaling side table
    eg_q, eg_tan_q, eg_meta = generate_eg_scale_lut()
    eg_hdr = (
        f'// JCM800 EL34 — (Vg2/Vg2_nom)^Ex − 1 side table, Q1.23\n'
        f'// Generated: {timestamp}\n'
        f'// k ∈ [{eg_meta["k_min"]}, {eg_meta["k_max"]}], Ex = {eg_meta["Ex"]}\n'
        f'// Peak positive delta = {eg_meta["peak_pos"]:+.4f}\n'
        f'// Peak negative delta = {eg_meta["peak_neg"]:+.4f}\n'
        f'// addr 2048 ↔ k=1.0 ↔ lut=0 (bias-fold)\n'
        f'//\n'
    )
    _write_lut_mem(out_path / 'pentode_eg_scale_lut.mem', eg_q,     24, eg_hdr)
    _write_lut_mem(out_path / 'pentode_eg_scale_tan.mem', eg_tan_q, 24,
                   eg_hdr.replace('side table', 'side tangents'))

    # OT saturation LUT
    ot_q, ot_tan_q, ot_meta = generate_ot_sat_lut()
    ot_hdr = (
        f'// JCM800 OT core saturation — soft-clip tanh(x/knee)·knee, Q1.23\n'
        f'// Generated: {timestamp}\n'
        f'// knee = {ot_meta["knee"]}   peak output = {ot_meta["peak"]:+.4f}\n'
        f'// addr 2048 ↔ x=0 ↔ lut=0 (bias-fold)\n'
        f'//\n'
    )
    _write_lut_mem(out_path / 'ot_sat_lut.mem', ot_q,     24, ot_hdr)
    _write_lut_mem(out_path / 'ot_sat_tan.mem', ot_tan_q, 24,
                   ot_hdr.replace('soft-clip', 'soft-clip tangents'))

    # Filter coefficients
    fc_prim = RAA / (2.0 * math.pi * LP_PRIMARY)        # primary-inductance LF pole
    tau_prim = 1.0 / (2.0 * math.pi * fc_prim)
    write_hpf_mem(out_path / 'hpf_prim.mem', tau_prim, timestamp,
                  f'OT primary-inductance roll-off (Lp={LP_PRIMARY}H, Raa={RAA:.0f}Ω)')
    fc_leak = RAA / (2.0 * math.pi * LLEAK)             # leakage-inductance HF pole
    fc_leak = min(fc_leak, FS_HZ * 0.49)
    write_lpf_mem(out_path / 'lpf_leak.mem', fc_leak, timestamp,
                  f'OT leakage-inductance roll-off (Lleak={LLEAK*1e3:.1f}mH, Raa={RAA:.0f}Ω)')

    # Presence shelf (NFB path)
    b0, b1, a1 = shelf_biquad_presence(PRESENCE_FC_HZ, PRESENCE_MAX_LIFT)
    write_shelf_mem(out_path / 'shelf_presence.mem', b0, b1, a1, timestamp,
                    f'NFB presence shelf',
                    [f'fc={PRESENCE_FC_HZ:.1f} Hz  max lift={PRESENCE_MAX_LIFT:.2f}× '
                     f'(≈ +{20*math.log10(PRESENCE_MAX_LIFT):.1f} dB)',
                     f'Pre-LUT / shelf placement: NFB path only',
                     f'Presence pot attenuates the shelf output (level_ctrl '
                     f'+ log_taper_256.mem) so pos=0 → flat NFB, pos=255 → max lift'])

    # Screen supply coefficients
    α_screen, β_screen = screen_supply_coeffs()

    # HT supply coefficients
    α_ht, β_ht = ht_supply_coeffs()

    # SV package
    write_power_pkg(pkg_path, timestamp, dc, el34_meta, eg_meta, ot_meta,
                    α_screen, β_screen, α_ht, β_ht,
                    fc_prim, fc_leak, PRESENCE_FC_HZ)

    if verbose:
        print(f'\nJCM800 power-amp artifacts → {out_path.resolve()}')
        print(f'  fs={FS_HZ:.0f} Hz')
        print(f'\nEL34 DC bias (fixed Vg1={VG1_BIAS:.1f} V, Vg2={VG2_NOM:.1f} V):')
        print(f'  Ip_q={dc[0]*1e3:.3f} mA   Ig2_q={dc[1]*1e3:.3f} mA   '
              f'Vak_q={dc[2]:.2f} V   Ik_q={dc[3]*1e3:.3f} mA')
        print(f'\nEL34 LUT:  |span|={el34_meta["span_abs"]*1e3:.3f} mA   '
              f'|Gss|={abs(el34_meta["tube_Gss"]):.3f}   '
              f'G_stage={el34_meta["G_stage"]:+.2f}   '
              f'flat_hi={el34_meta["flat_hi_entries"]}  flat_lo={el34_meta["flat_lo_entries"]}')
        print(f'OT sat LUT: knee={ot_meta["knee"]}')
        print(f'OT HPF (Lp):     fc={fc_prim:.3f} Hz')
        print(f'OT LPF (Lleak):  fc={fc_leak:.1f} Hz')
        ss_full = β_screen / α_screen
        print(f'Screen supply:   α={α_screen:.4e}  β={β_screen:.4e}   '
              f'(steady-state sag at Ig2_norm=1.0 ≈ {ss_full*100:.1f}%)')
        ht_ss_full = β_ht / α_ht
        ht_tau = R_HT * C_HT
        print(f'HT supply:       α={α_ht:.4e}  β={β_ht:.4e}   '
              f'τ={ht_tau*1e3:.1f} ms   '
              f'(steady-state sag at Ip_norm=1.0 ≈ {ht_ss_full*100:.1f}%)')
        print(f'Presence shelf:  fc={PRESENCE_FC_HZ:.1f} Hz   '
              f'max lift ×{PRESENCE_MAX_LIFT:.2f}')


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='JCM800 power-amp LUT + coefficient generator')
    ap.add_argument('--out', type=str, default='lut_out')
    ap.add_argument('--pkg', type=str, default='rtl/jcm800_power_pkg.sv')
    args = ap.parse_args()
    export_all(out_dir=args.out, pkg_path=args.pkg)
