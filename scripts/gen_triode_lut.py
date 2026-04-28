"""
gen_triode_lut.py  —  JCM800 2203/2204 preamp LUT + coefficient generator
=========================================================================

Single source of truth for all DSP tables and coefficients consumed by the
redesigned gain_stage.sv / iir_hpf1 / iir_lpf1 / cathode_shelf modules.

Design summary (see /home/fab/.claude/plans/redesign-completely-gain-stage-*.md)
────────────────────────────────────────────────────────────────────────────
  Intra-stage order:
    x → (input-AA once at top) → Vgk clamp → LUT(f_norm) → ×G_stage
        → plate LPF → cathode shelf → coupling HPF → next stage

  LUT stores bias-folded NORMALISED PLATE VOLTAGE (CC) or CATHODE VOLTAGE
  (CF), evaluated on a DYNAMIC AC load line (Rp ∥ next-stage input network).
  Quiescence maps to exactly zero, so the ADC noise floor is not amplified
  by a DC residue through the cascade.

  Per-stage gain G_stage = −d(Vout)/d(Vgk)|_Q-point is stored as Q4.20 and
  applied post-LUT as (G · f_norm) >> 20.  Moving the gain out of the LUT
  input scale removes the pre-clip noise-gain that the old current-LUT
  design suffered from (big INPUT_SCALE × ADC LSB dither).

Output artefacts
────────────────
  lut_out/{v1a,v1b,v2a,v2b}_lut.mem    4097 × 24-bit  Q1.23  f_norm(x)
  lut_out/{v1a,v1b,v2a,v2b}_tan.mem    4097 × 24-bit  Q1.23  PCHIP tangents
  lut_out/hpf_{v1a,v1b,v2a,v2b}_out.mem    1 × 32-bit Q1.31  α  (τ ≈ 10 ms)
  lut_out/lpf_{v1a,v1b,v2a,v2b}.mem        1 × 32-bit Q1.31  β
  lut_out/shelf_{v1a,v1b,v2a,v2b}.mem      3 × 32-bit Q3.29  (b0, b1, a1)
  rtl/jcm800_lut_pkg.sv                 G_STAGE_Q4_20 + filenames + INPUT_SCALE

FPGA numeric formats
────────────────────
  Signals           : Q1.23  sample_t  (unchanged; matches CS5343/CS4344)
  LUT f_norm        : Q1.23  stored in BRAM (range ≈ ±0.5, room for asymmetry)
  PCHIP tangent     : Q1.23  stored in second BRAM (same format as before)
  G_stage           : Q4.20  signed 32-bit  (range ±2048, covers |G|≤128)
  α / β             : Q1.31  signed 32-bit  (1 − α ≈ 2 × 10⁻³ resolvable)
  Shelf (b0,b1,a1)  : Q3.29  signed 32-bit  (each ∈ (−4, +4); a1 ∈ (−1, +1))
  Input-scale       : Q16.16 signed 32-bit  (single top-level tunable)

Coefficient widths are ENUMERATED in the written header of every .mem file so
the RTL can load them via $readmemh into matching-width distributed ROMs
without guesswork.
"""

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq
import pathlib
import datetime
import math


# ─────────────────────────────────────────────────────────────────────────────
# Sample rate & tube model
# ─────────────────────────────────────────────────────────────────────────────

FS_HZ = 768_000.0    # preamp internal rate (16× oversampled; ADC/DAC still 48 kHz)

# Grid-diode lumped model used inside _solve_vin when Vgk drives positive.
# Is ≈ 1 µA, Vt ≈ 0.6 V is the usual amp-modelling shorthand for the
# triode grid-cathode junction; pulled through the previous-stage drive
# impedance (mixer_R) this produces soft positive-rail compression that
# is baked straight into the LUT curve.
GRID_IS_A  = 1.0e-6
GRID_VT_V  = 0.6

# 12AX7 Koren parameters (unchanged from prior model)
_KOREN = dict(mu=100., Ex=1.4, Kg1=2977.6, Kp=600., Kvb=300.)


def koren_Ip(Vgk, Vak, mu=100., Ex=1.4, Kg1=2977.6, Kp=600., Kvb=300.):
    """Plate current in Amps, numerically stable softplus form."""
    E1 = (Vak / Kp) * np.log1p(
        np.exp(np.clip(Kp * (1.0 / mu + Vgk / np.sqrt(Kvb + Vak**2)), -500., 500.))
    )
    E1 = np.maximum(E1, 0.0)
    return (E1**Ex / Kg1) * (1.0 + np.sign(E1))


# ─────────────────────────────────────────────────────────────────────────────
# JCM800 preamp circuit constants
# ─────────────────────────────────────────────────────────────────────────────

B_B3_ANCHOR = 390.0          # V — schematic B+3, anchor for downstream cascade
R_DROP      = 10e3           # Ω — each voltage-drop R in the B+ cascade

# Capacitance constants used for LPF / shelf coefficients
CGP_TUBE    = 1.6e-12        # 12AX7 grid-plate capacitance (datasheet)
C_STRAY_PLT = 5e-12          # plate-node wiring stray (estimate)
C_STRAY_CAT = 5e-12          # cathode-node wiring stray (CF only)
R_PA_INPUT  = 500e3          # approx. power-amp / tonestack input load on V2B

# Per-stage topology:
#   topology : 'cc' common-cathode, output at plate; 'cf' cathode follower
#   Rp       : plate resistor (0 for CF since plate is at B+)
#   Rk       : cathode resistor
#   Ck_F     : cathode bypass cap in farads, or None if unbypassed
#   B_node   : 'B4' or 'B5' — B+ tap that feeds this stage
#   Rg       : this stage's grid-leak (to GND)
#   mixer_R  : series R between prev stage's coupling cap and this grid
#   pot_shunt: optional shunt pot at the coupling-cap node (None if absent)
#   Cc_F     : coupling cap at this stage's OUTPUT (drives next-stage grid);
#              τ = Cc · Z_next_grid determines the per-stage coupling-HPF.
#
# V1A Ck_F tightened from 0.68 µF to 0.22 µF: moves the cathode shelf from
# ~87 Hz to ~270 Hz so guitar-range low end no longer gets V1A's full ~−42×
# gain hammered into the high-gain later stages.
# V1A Cc_F tightened from 22 nF to 10 nF: shifts the first inter-stage
# coupling HPF toward ~20 Hz (was a uniform 16 Hz), preventing sub-guitar
# energy from IMD-ing upward through the cascade.
#
# Stage names below are SIGNAL-CHAIN POSITIONS (gain pot sits between
# v1a and v1b), not physical valve assignments. Mapping to canonical
# Marshall JCM800 2203 roles:
#   v1a — input gain stage, bypassed cathode      (2.7 kΩ ‖ 0.22 µF)
#   v1b — "cold clipper" after the gain pot       (10 kΩ unbypassed)
#   v2a — recovery stage after the cold clipper   (820 Ω unbypassed)
#   v2b — cathode follower driving master vol/PI  (Rp=0, Rk=100 kΩ)
# i.e. v1b is the 10 kΩ slot, v2a is the 820 Ω slot — do not compare
# against the wrong row when cross-checking a schematic.
_CIRCUIT = {
    'v1a': dict(topology='cc', Rp=100e3, Rk=2.7e3, Ck_F=0.22e-6, B_node='B5',
                Rg=1e6,   mixer_R=68e3,  pot_shunt=None, Cc_F=10e-9),
    'v1b': dict(topology='cc', Rp=100e3, Rk=10e3,  Ck_F=None,    B_node='B5',
                Rg=470e3, mixer_R=470e3, pot_shunt=1e6,  Cc_F=22e-9),
    'v2a': dict(topology='cc', Rp=100e3, Rk=820.,  Ck_F=None,    B_node='B4',
                Rg=470e3, mixer_R=470e3, pot_shunt=None, Cc_F=22e-9),
    'v2b': dict(topology='cf', Rp=0.0,   Rk=100e3, Ck_F=None,    B_node='B4',
                Rg=470e3, mixer_R=100e3, pot_shunt=None, Cc_F=22e-9),
}
_STAGE_ORDER = ['v1a', 'v1b', 'v2a', 'v2b']


# ─────────────────────────────────────────────────────────────────────────────
# Phase-inverter (long-tailed-pair) circuit constants — JCM800 2203 V3
# ─────────────────────────────────────────────────────────────────────────────
#
# LTP arm A (V3A) takes the signal from V2B via the master volume; arm B
# (V3B) has its grid AC-grounded via a 0.1 µF cap and DC-referenced to
# ground through a 1 MΩ.  Both cathodes share a single resistor to ground —
# there is no bypass cap, so the LTP common-mode rejects via the tail.
#
# Rp_a (82 kΩ) is deliberately smaller than Rp_b (100 kΩ): the Marshall
# trick for balancing PI plate swings despite the tail-current asymmetry —
# V3A hogs more of the tail swing than V3B does, so its smaller plate load
# gives a matching output swing.  Running the solver confirms this imbalance
# is AUDIBLE as asymmetric clipping (the realism gap the plan called out).
#
# B+ tap for the PI plates is B3 ≈ 320 V in the 2203 (one drop resistor up
# from the preamp B4 rail).  The EL34 grid-leaks at 220 kΩ each form the
# next-stage Z_grid for the two coupling caps.
_PI_CIRCUIT = dict(
    Rp_a      = 82e3,      # V3A plate load (signal arm)
    Rp_b      = 100e3,     # V3B plate load (reference arm)
    Rk        = 10e3,      # shared cathode resistor to ground (no bypass)
    B_pi      = 320.0,     # B+ at the PI plate tap
    mixer_R_a = 220e3,     # V3A grid drive impedance (master vol wiper + grid-leak)
    Cc_F      = 22e-9,     # output coupling caps to each power-tube grid
    R_pa_grid = 220e3,     # EL34 grid-leak (per tube); Z looking INTO power amp
)


# ─────────────────────────────────────────────────────────────────────────────
# DC operating point (coupling caps open at DC)
# ─────────────────────────────────────────────────────────────────────────────

def _solve_cc(Rp, Rk, B):
    """Common-cathode DC: Vgk=−Ip·Rk (grid leak to GND), Vak=B−Ip·(Rp+Rk)."""
    def residual(Ip):
        Vak = B - Ip * (Rp + Rk)
        if Vak < 10.:
            return -1.
        return float(koren_Ip(-Ip * Rk, Vak, **_KOREN)) - Ip
    Ip_q = brentq(residual, 1e-7, B / (Rp + Rk) * 0.99, xtol=1e-12, rtol=1e-10)
    return Ip_q, B - Ip_q * (Rp + Rk), -Ip_q * Rk


def solve_dc_network(tol=1e-9, max_iter=200):
    Ip = {n: 1e-3 for n in _CIRCUIT}
    B_node = {'B4': B_B3_ANCHOR, 'B5': B_B3_ANCHOR}
    results = {}
    for _ in range(max_iter):
        I_B5 = Ip['v1a'] + Ip['v1b']
        I_B4 = Ip['v2a'] + Ip['v2b']
        B_node['B4'] = B_B3_ANCHOR  - (I_B4 + I_B5) * R_DROP
        B_node['B5'] = B_node['B4'] -  I_B5         * R_DROP
        new_Ip = {}
        for n, c in _CIRCUIT.items():
            B_loc = B_node[c['B_node']]
            # CF uses the same CC-form DC solve (grid tied to GND via leak,
            # self-bias via cathode; Rp=0 collapses the equation naturally)
            Ip_q, Vak_q, Vgk_q = _solve_cc(c['Rp'], c['Rk'], B_loc)
            new_Ip[n] = Ip_q
            results[n] = dict(Ip_q=Ip_q, Vak_q=Vak_q, Vgk_q=Vgk_q, B_loc=B_loc)
        if max(abs(new_Ip[n] - Ip[n]) for n in _CIRCUIT) < tol:
            return results
        Ip = new_Ip
    return results


# ─────────────────────────────────────────────────────────────────────────────
# Interstage loading & AC load resistance
# ─────────────────────────────────────────────────────────────────────────────

def _grid_network_Z(c):
    """
    AC impedance looking INTO a stage's grid from the coupling-cap node.
    With the coupling cap treated as a short (audio band):
        Z_grid = mixer_R + (Rg ∥ pot_shunt)
    """
    rg = c['Rg']
    if c.get('pot_shunt') is not None:
        rg = rg * c['pot_shunt'] / (rg + c['pot_shunt'])
    return c['mixer_R'] + rg


def _next_grid_Z(stage_name):
    """AC Z of the next stage's grid network.  For v2b → power-amp model."""
    idx = _STAGE_ORDER.index(stage_name)
    if idx + 1 < len(_STAGE_ORDER):
        return _grid_network_Z(_CIRCUIT[_STAGE_ORDER[idx + 1]])
    return R_PA_INPUT


def _ac_load_R(stage_name):
    """
    AC load resistance seen at the stage's OUTPUT node (plate for CC,
    cathode for CF) with the coupling cap shorted:

      CC: R_out_ac = Rp ∥ Z_next_grid
      CF: R_out_ac = Rk ∥ Z_next_grid
    """
    c = _CIRCUIT[stage_name]
    z_next = _next_grid_Z(stage_name)
    r_node = c['Rp'] if c['topology'] == 'cc' else c['Rk']
    return r_node * z_next / (r_node + z_next)


# ─────────────────────────────────────────────────────────────────────────────
# Large-signal solve — parameterised by GRID INPUT Vin, not Vgk directly
# ─────────────────────────────────────────────────────────────────────────────
#
# The cascade's natural signal variable is Vin, the AC voltage at the grid
# terminal relative to AC ground (i.e. the previous stage's driving signal
# after the coupling network).  Vgk depends on Vin AND the cathode's own
# motion (unbypassed cathode degenerates; CF cathode is the output and
# moves 1:1-ish with Vin).
#
# Three effective cases, handled uniformly by one self-consistent solver:
#
#   CC bypassed (v1a; Ck shorts cathode to AC GND in the audio band):
#       R_cath_ac = 0,   Vgk = Vgk_Q + Vin,   ΔVak = −ΔIp · R_ac
#       → shelf module downshifts LF gain to the unbypassed value
#
#   CC unbypassed (v1b, v2a; no Ck):
#       R_cath_ac = Rk,  Vgk = Vgk_Q + Vin − ΔIp·Rk
#       ΔVak = −ΔIp · R_ac    (plate AC load)
#       → no shelf (flat frequency response of degeneration; identity shelf)
#
#   CF (v2b; Rp=0 so plate is at B+ directly):
#       R_cath_ac = Rk,  Vgk = Vgk_Q + Vin − ΔIp·Rk
#       Vak = B_loc − (Ip · Rk)   (plate-to-cathode drops with cathode rise)
#       output node = cathode:    Vk = Ip · Rk
#
# The solver finds Ip (and thus ΔVout) satisfying the Koren model AND
# Kirchhoff at the cathode and plate nodes simultaneously.

def _solve_vin(Vin, Ip_Q, Vak_Q, Vgk_Q, B_loc, R_ac, R_cath_ac,
                topology, Rk, mixer_R=None,
                grid_Is=GRID_IS_A, grid_Vt=GRID_VT_V):
    """
    Self-consistent solve: given driving voltage Vin at the grid, find the
    stage's output-node voltage (Vak for CC, Vk for CF) at audio frequencies.

    Grid-conduction soft-clip (Vgk > 0 only):
        Ig   = Is · (exp(Vgk/Vt) − 1)
        Vgk' = Vgk_drive − Ig · mixer_R
    The previous stage's drive impedance (mixer_R) forms a divider with the
    grid-cathode diode so Vgk_eff asymptotes near zero instead of racing past
    it.  This bakes a smooth positive-rail shoulder into the LUT — the main
    reason palm-mute transients currently sound clicky is that the shoulder
    is a hard corner.  Solved by a damped fixed-point iteration; converges in
    a handful of steps under all stage impedances we use.

    Returns (Vout, Ip).
    """
    def _apply_grid_diode(Vgk_drive):
        if mixer_R is None or Vgk_drive <= 0.0:
            return Vgk_drive
        Vgk = Vgk_drive
        for _ in range(12):
            x = min(max(Vgk / grid_Vt, -30.), 30.)
            Ig = grid_Is * (math.exp(x) - 1.0)
            Vgk_new = Vgk_drive - Ig * mixer_R
            if abs(Vgk_new - Vgk) < 1e-7:
                Vgk = Vgk_new
                break
            # Damped update — the exponential is stiff, plain fixed-point
            # oscillates for mixer_R in the 470 kΩ range.
            Vgk = 0.5 * Vgk + 0.5 * Vgk_new
        return Vgk

    def residual_Ip(Ip):
        # Cathode motion (zero for bypassed CC)
        dVk = (Ip - Ip_Q) * R_cath_ac
        Vgk_drive = Vgk_Q + Vin - dVk
        Vgk = _apply_grid_diode(Vgk_drive)
        if topology == 'cc':
            # AC load line on plate
            dVak = -(Ip - Ip_Q) * R_ac
            Vak  = Vak_Q + dVak
        else:
            # CF: plate held at B+, Vak reflects cathode motion
            Vak  = B_loc - (Ip_Q + (Ip - Ip_Q)) * Rk
        if Vak < 5.:
            Vak = 5.
        return float(koren_Ip(Vgk, Vak, **_KOREN)) - Ip

    # Bracket Ip.  Lower bound tiny positive, upper bound somewhat above Q.
    Ip_lo = 1e-9
    Ip_hi = max(Ip_Q * 20., 1e-2)
    # Expand upper if still same sign
    for _ in range(6):
        if residual_Ip(Ip_lo) * residual_Ip(Ip_hi) < 0:
            break
        Ip_hi *= 2.0
    try:
        Ip = brentq(residual_Ip, Ip_lo, Ip_hi, xtol=1e-10, rtol=1e-9)
    except ValueError:
        Ip = Ip_Q  # fallback; rails will clip later
    Ip = max(Ip, 0.0)
    if topology == 'cc':
        Vout = Vak_Q - (Ip - Ip_Q) * R_ac
    else:
        Vout = Ip * Rk
    return Vout, Ip


def _stage_R_cath_ac(stage_name, c):
    """
    Cathode impedance to AC ground at audio frequencies.
      CC with Ck bypass  → 0 (shelf handles LF)
      CC without Ck      → Rk (flat degeneration)
      CF                 → Rk
    """
    if c['topology'] == 'cc' and c['Ck_F'] is not None:
        return 0.0
    return c['Rk']


# ─────────────────────────────────────────────────────────────────────────────
# Long-tailed pair DC and large-signal solvers (Gap 3 — phase inverter)
# ─────────────────────────────────────────────────────────────────────────────
#
# Unlike a preamp stage, the PI's two triodes do NOT run independently: they
# share a common cathode, so the tail current Ip_a + Ip_b sets a single Vk
# that both grids see.  V3A is driven by the signal (V2B output through
# master vol), V3B's grid is AC-grounded.  At DC both grids sit at 0 V via
# 1 MΩ leaks, so the DC solve is symmetric in Vgk (Vgk_a_q = Vgk_b_q = −Vk_q)
# even though the plate currents differ because Rp_a ≠ Rp_b.
#
# Large-signal: as V3A's grid goes positive, it steals tail current from
# V3B (Vk rises, V3B's Vgk goes more negative, V3B cuts off progressively).
# The two plate voltages respond asymmetrically under hard drive — that
# asymmetry is what we want the LUT to capture instead of the current
# "negate-input-and-reuse-V2B-LUT" ideal-LTP approximation.

def _solve_ltp_dc(c):
    """DC operating point of the LTP (both grids at 0 V via 1 MΩ leaks)."""
    Rp_a, Rp_b, Rk, B = c['Rp_a'], c['Rp_b'], c['Rk'], c['B_pi']

    def _ip_for_Rp(Rp, Vk):
        def r(Ip):
            Vak = B - Ip * Rp
            if Vak < 5.0:
                return -1.
            return float(koren_Ip(-Vk, Vak, **_KOREN)) - Ip
        try:
            return brentq(r, 1e-9, B / Rp * 0.99, xtol=1e-12, rtol=1e-10)
        except ValueError:
            return 0.0

    def resid_Vk(Vk):
        Ip_a = _ip_for_Rp(Rp_a, Vk)
        Ip_b = _ip_for_Rp(Rp_b, Vk)
        return (Ip_a + Ip_b) * Rk - Vk

    Vk_q = brentq(resid_Vk, 0.1, 30.0, xtol=1e-9, rtol=1e-10)
    Ip_a_q = _ip_for_Rp(Rp_a, Vk_q)
    Ip_b_q = _ip_for_Rp(Rp_b, Vk_q)
    return dict(
        Ip_a_q=Ip_a_q, Ip_b_q=Ip_b_q, Vk_q=Vk_q,
        Vak_a_q=B - Ip_a_q * Rp_a, Vak_b_q=B - Ip_b_q * Rp_b,
        Vgk_a_q=-Vk_q, Vgk_b_q=-Vk_q,
    )


def _solve_ltp_vin(Vin, dc, c,
                   grid_Is=GRID_IS_A, grid_Vt=GRID_VT_V):
    """
    Self-consistent LTP solve: V3A grid driven by Vin (AC), V3B grid at 0 V.
    Finds the shared cathode Vk and returns both plate voltages.

    The grid-diode soft-clip on V3A (via its mixer_R drive impedance) is baked
    in exactly as in _solve_vin — V3A can enter grid conduction hard under
    cranked-master drive, which is where the 2203's PI nonlinearity lives.
    """
    Rp_a, Rp_b, Rk, B = c['Rp_a'], c['Rp_b'], c['Rk'], c['B_pi']
    mixer_R_a = c.get('mixer_R_a', 0.0)

    def _apply_grid_diode(Vgk_drive):
        if mixer_R_a == 0.0 or Vgk_drive <= 0.0:
            return Vgk_drive
        Vgk = Vgk_drive
        for _ in range(12):
            x = min(max(Vgk / grid_Vt, -30.), 30.)
            Ig = grid_Is * (math.exp(x) - 1.0)
            Vgk_new = Vgk_drive - Ig * mixer_R_a
            if abs(Vgk_new - Vgk) < 1e-7:
                Vgk = Vgk_new
                break
            Vgk = 0.5 * Vgk + 0.5 * Vgk_new
        return Vgk

    def _ip_at(Vgk, Rp):
        def r(Ip):
            Vak = B - Ip * Rp
            if Vak < 5.0:
                return -1.
            return float(koren_Ip(Vgk, Vak, **_KOREN)) - Ip
        # For deep cutoff residual(1e-12) may already be negative (no plate
        # current can satisfy the equation); return 0 rather than erroring.
        try:
            lo = 1e-12
            hi = B / Rp * 0.99
            if r(lo) * r(hi) > 0:
                return 0.0 if abs(r(lo)) < abs(r(hi)) else hi
            return brentq(r, lo, hi, xtol=1e-12, rtol=1e-10)
        except ValueError:
            return 0.0

    def resid_Vk(Vk):
        Vgk_a_drive = Vin - Vk
        Vgk_a = _apply_grid_diode(Vgk_a_drive)
        Vgk_b = -Vk
        Ip_a = _ip_at(Vgk_a, Rp_a)
        Ip_b = _ip_at(Vgk_b, Rp_b)
        return (Ip_a + Ip_b) * Rk - Vk

    # Expand bracket if the quiescent Vk is an outlier (deep clip regions)
    Vk_lo = 1e-4
    Vk_hi = max(dc['Vk_q'] * 5.0, 20.0)
    for _ in range(8):
        if resid_Vk(Vk_lo) * resid_Vk(Vk_hi) < 0:
            break
        Vk_hi *= 2.0
    try:
        Vk = brentq(resid_Vk, Vk_lo, Vk_hi, xtol=1e-9, rtol=1e-10)
    except ValueError:
        Vk = dc['Vk_q']

    Vgk_a_drive = Vin - Vk
    Vgk_a = _apply_grid_diode(Vgk_a_drive)
    Vgk_b = -Vk
    Ip_a = _ip_at(Vgk_a, Rp_a)
    Ip_b = _ip_at(Vgk_b, Rp_b)
    return dict(
        Vout_a=B - Ip_a * Rp_a,
        Vout_b=B - Ip_b * Rp_b,
        Ip_a=Ip_a, Ip_b=Ip_b, Vk=Vk,
    )


def _solve_dV(Vgk_q, Vak_q, Rp, B, headroom=1.10):
    """
    Maximum symmetric grid swing ±dV.  The LUT domain [−dV, +dV] wraps the
    region the solver is asked to tabulate.  With the grid-diode soft-clip
    active in _solve_vin we now deliberately push dV_pos PAST the zero-Vgk
    boundary so the soft-clip shoulder lands INSIDE the LUT (with non-zero
    PCHIP tangents) rather than at its edge.  Previous design capped dV_pos
    at 0.9·|Vgk_Q|, which meant a hard-driven signal hit the flat rail and
    produced the slope-discontinuous shoulder the palm-mute-click symptom
    was tracking.
    """
    Ip_q = float(koren_Ip(Vgk_q, Vak_q, **_KOREN))
    # Reach ~1.2 V past Vgk=0 by default so the grid-diode exponential curve
    # is captured for several time-constants of Vt before the rails kick in.
    dV_pos = abs(Vgk_q) + 1.2
    # Plate-safety brentq (unchanged behaviour, just a wider bracket):
    # if the plate would collapse below 30 V at the proposed dV_pos, back off
    # until it doesn't.  The grid-diode now limits the actual Vgk anyway, so
    # this rarely triggers in practice.
    if Rp > 0 and B - float(koren_Ip(Vgk_q + dV_pos, Vak_q, **_KOREN)) * Rp < 30.:
        try:
            dV_pos = brentq(
                lambda dv: B - float(koren_Ip(Vgk_q + dv, max(Vak_q - dv * 80., 10.), **_KOREN)) * Rp - 30.,
                1e-5, abs(Vgk_q) + 1.5
            )
        except Exception:
            pass
    try:
        dV_neg = brentq(
            lambda dv: float(koren_Ip(Vgk_q - dv, max(B, Vak_q), **_KOREN)) - Ip_q * 0.01,
            1e-5, 15., xtol=1e-7
        )
    except Exception:
        dV_neg = abs(Vgk_q) * 1.5
    return min(dV_pos, dV_neg) * headroom


# ─────────────────────────────────────────────────────────────────────────────
# Per-stage parameter computation  (single source of truth)
# ─────────────────────────────────────────────────────────────────────────────

def compute_stage_params():
    """
    Return ordered dict: stage_name → param_dict, including everything needed
    by the LUT generator and every filter-coefficient emitter.
    """
    dc = solve_dc_network()
    out = {}
    for n in _STAGE_ORDER:
        c = _CIRCUIT[n]
        Ip_q, Vak_q, Vgk_q, B_loc = (dc[n]['Ip_q'], dc[n]['Vak_q'],
                                      dc[n]['Vgk_q'], dc[n]['B_loc'])
        Rp, Rk = c['Rp'], c['Rk']
        topo   = c['topology']

        dV   = _solve_dV(Vgk_q, Vak_q, Rp if topo == 'cc' else 1.0, B_loc)
        R_ac = _ac_load_R(n)
        R_cath_ac = _stage_R_cath_ac(n, c)

        # Vout at the DC bias = Vak_Q (CC) or Ip_Q·Rk (CF).  Exactly reproduced
        # by _solve_vin(Vin=0) by construction of the load line through Q.
        Vout_bias = Vak_q if topo == 'cc' else Ip_q * Rk

        # Small-signal gain probe (used for Miller capacitance estimate).
        # mixer_R passed through so grid-diode branch engages consistently —
        # at ±eps the diode is inactive anyway (Vgk stays negative), so this
        # does not perturb the probe.
        mixer_R = c['mixer_R']
        eps = 1e-4
        Vp, _ = _solve_vin(+eps, Ip_q, Vak_q, Vgk_q, B_loc, R_ac, R_cath_ac,
                           topo, Rk, mixer_R=mixer_R)
        Vn, _ = _solve_vin(-eps, Ip_q, Vak_q, Vgk_q, B_loc, R_ac, R_cath_ac,
                           topo, Rk, mixer_R=mixer_R)
        G_small = (Vp - Vn) / (2. * eps)
        if topo == 'cc':
            C_total = C_STRAY_PLT + CGP_TUBE * (1. + abs(G_small))
        else:
            C_total = C_STRAY_CAT    # CF has no Miller at its output node

        out[n] = dict(
            topology=topo, Rp=Rp, Rk=Rk, Ck_F=c['Ck_F'], B_loc=B_loc,
            Ip_q=Ip_q, Vak_q=Vak_q, Vgk_q=Vgk_q, Vout_bias=Vout_bias,
            dV=dV, R_ac=R_ac, R_cath_ac=R_cath_ac,
            C_plate=C_total, mixer_R=mixer_R, Cc_F=c['Cc_F'],
        )
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Non-uniform sampling grid (preserved from previous design)
# ─────────────────────────────────────────────────────────────────────────────

def nonuniform_grid(N=32769, alpha=0.55):
    assert N % 2 == 1, 'N must be odd'
    u = np.linspace(0., 1., N)
    return np.sign(u - 0.5) * np.abs(2. * u - 1.) ** alpha


# ─────────────────────────────────────────────────────────────────────────────
# Soft-rail asymptote — replaces the flat digital rail that was producing a
# square-wave shoulder (the "brittle/digital" signature at high gain).  For
# |h| ≤ SOFT_RAIL_KNEE the curve is identity (small-signal region bit-exact,
# bias-fold invariant preserved, tube Gss untouched).  Above the knee the
# magnitude is compressed toward 1.0 via a rational  x/(x+decay)  asymptote:
# slope is 1 at the knee (C¹ continuous with the identity region), and the
# asymptote approaches (but never reaches) 1.0 no matter how large |h|.
# Critically, unlike tanh this stays well clear of numerical saturation for
# h_fine up to tens (V2A at peak has |h|≈16 post-Gss_mag scaling), so every
# LUT entry near the rail receives a distinct quantised value and the
# digital-square-wave shoulder disappears.
# ─────────────────────────────────────────────────────────────────────────────

SOFT_RAIL_KNEE = 0.92


def _soft_rail(h, knee=SOFT_RAIL_KNEE):
    k = float(knee)
    a = 1.0 - k
    mag = np.abs(h)
    out = h.copy()
    mask = mag > k
    excess = mag[mask] - k
    # Rational asymptote: shaped → (a - eps)·excess/(excess + a).  Slope at
    # excess=0 is (a - eps)/a ≈ 1, so C¹ with identity at the knee.  The
    # tiny eps (a few LSBs at Q1.23) guarantees the result never reaches ±1
    # exactly, so no entry quantises to the hard rail.
    eps = 2.0 / (2 ** 23)                 # ≈ 2 LSB at Q1.23
    shaped = (a - eps) * excess / (excess + a)
    out[mask] = np.sign(h[mask]) * (k + shaped)
    return out


# ─────────────────────────────────────────────────────────────────────────────
# LUT generator — bias-folded, normalised plate/cathode VOLTAGE
# ─────────────────────────────────────────────────────────────────────────────

def generate_lut(stage_name, p, N_lut=4097, N_fine=32769, out_bits=24):
    """
    Build the Vak-normalised LUT + PCHIP tangents for one stage.

    Indexing
    ────────
      x ∈ [-1, +1]  maps linearly to  Vgk ∈ [Vgk_Q − dV, Vgk_Q + dV]
      idx = round((x + 1) / 2 · (N_lut − 1))
      addr 2048 ↔ x=0 ↔ Vgk=Vgk_Q ↔ f_norm=0   (bias-fold invariant)

    LUT content
    ───────────
      f_raw(Vgk) = Vout_on_AC_loadline(Vgk)    (Vak for CC, Vk for CF)
      ΔVout_span = f_raw(Vgk_max) − f_raw(Vgk_min)     (signed)
      f_norm     = (f_raw − Vout_bias) / |ΔVout_span|

    By construction f_norm(Vgk_Q)=0 exactly (bias fold).  Sign convention:
    a POSITIVE ΔVgk gives f_norm that tracks Vout at the OUTPUT NODE — so
    CC stages produce NEGATIVE f_norm for positive Vgk (plate inverts),
    CF stages produce POSITIVE f_norm (cathode follows).  The sign
    inversion is re-asserted explicitly in G_stage (spec §2).

    Returns
    ───────
      lut_q (int32[N_lut])   Q1.(out_bits-1) quantised f_norm
      tan_q (int32[N_lut])   Q1.(out_bits-1) PCHIP tangents · step
      meta  (dict)           Vout_span, G_stage_small, span sign
    """
    assert N_lut % 2 == 1
    Nm1 = N_lut - 1
    assert (Nm1 & (Nm1 - 1)) == 0, f'N_lut−1={Nm1} must be power of 2'

    Vak_q, Vgk_q, dV      = p['Vak_q'], p['Vgk_q'], p['dV']
    Ip_q, R_ac, Vout_bias = p['Ip_q'], p['R_ac'], p['Vout_bias']
    topo, Rk, B_loc       = p['topology'], p['Rk'], p['B_loc']
    R_cath_ac             = p['R_cath_ac']
    mixer_R               = p['mixer_R']

    # Fine evaluation: output-node voltage as a function of driving Vin.
    # x ∈ [-1,+1] maps to Vin ∈ [−dV, +dV] (grid-swing half-range).
    x_fine = nonuniform_grid(N_fine)
    Vin_fine = x_fine * dV
    Vout_fine = np.empty_like(Vin_fine)
    for i, vin in enumerate(Vin_fine):
        Vout_fine[i], _ = _solve_vin(float(vin), Ip_q, Vak_q, Vgk_q, B_loc,
                                      R_ac, R_cath_ac, topo, Rk,
                                      mixer_R=mixer_R)

    # Bias-fold: Vout_bias at x=0.  Use the span endpoints for normalisation.
    span_raw = float(Vout_fine[-1] - Vout_fine[0])   # signed; CC: negative, CF: positive
    if abs(span_raw) < 1e-6:
        raise ValueError(f'{stage_name}: degenerate span {span_raw:.3e}')
    span_sign = 1 if span_raw > 0 else -1
    span_abs  = abs(span_raw)

    # Linear (span-normalised) transfer — this is the tube's actual I/O curve
    # scaled so peak |h_linear| ≈ 0.5 and slope_at_zero is the tube's linear
    # voltage gain divided by span_abs/dV.  Used only to extract Gss_mag.
    h_linear = (Vout_fine - Vout_bias) / span_abs
    mid_fine = N_fine // 2
    h_linear[mid_fine] = 0.0
    slope_linear_at_zero = float(PchipInterpolator(x_fine, h_linear).derivative()(0.))
    Gss_mag              = abs(slope_linear_at_zero) * span_abs / dV   # tube |Gss|

    # Pre-scale the tube curve by Gss_mag so the LUT's slope at the Q-point
    # matches the previous design's post-G small-signal gain exactly, and
    # G_STAGE_Q4_20 collapses to ±1 (phase-inversion sign only).  Values
    # exceeding ±1 are then soft-asymptoted (not hard-clipped) by _soft_rail
    # below, so the LUT shoulders hold distinct values instead of a flat rail.
    h_fine = h_linear * Gss_mag
    h_fine[mid_fine] = 0.0

    # PCHIP fit + resample to uniform LUT grid
    pchip  = PchipInterpolator(x_fine, h_fine)
    assert abs(float(pchip(0.))) < 1e-12, \
        f'{stage_name}: h(0)={float(pchip(0.)):.3e} — bias-fold not exact'
    x_lut  = np.linspace(-1., 1., N_lut)
    h_lut_raw = pchip(x_lut)
    step   = 2. / (N_lut - 1)
    dh_lut_raw = pchip.derivative()(x_lut) * step

    # New G_stage: ±1 exactly (phase-inversion sign only).  Sign chosen so
    # slope_new × G_new has the same sign as the previous (slope_old × G_old)
    # — the digital cascade's polarity stays the same.
    slope_at_zero = float(pchip.derivative()(0.))
    G_stage = float(np.sign(slope_at_zero))            # ±1.0
    # Stash tube |Gss| for the log/summary so the operator can still see it.
    tube_Gss = Gss_mag * np.sign(slope_linear_at_zero) * np.sign(G_stage)

    # Soft-rail: leave |h| ≤ knee untouched (bit-exact small-signal region),
    # tanh-asymptote above the knee.  Tangents in the shaped region are
    # recomputed via finite-difference of the softened curve; in the linear
    # region the original PCHIP tangent is preserved (C¹ at the boundary).
    h_lut = _soft_rail(h_lut_raw)
    linear_mask = np.abs(h_lut_raw) <= SOFT_RAIL_KNEE
    dh_lut = np.where(linear_mask, dh_lut_raw, np.gradient(h_lut))

    # Quantise
    scale = 2 ** (out_bits - 1) - 1
    lut_q = np.clip(np.round(h_lut  * scale), -(scale + 1), scale).astype(np.int32)
    tan_q = np.clip(np.round(dh_lut * scale), -(scale + 1), scale).astype(np.int32)

    # Zero tangents at clip-rail entries so Hermite cannot overshoot the LUT
    # values at the saturation shoulders (spec §8 rule 3: monotone, flat stays flat).
    rail_hi, rail_lo = scale, -(scale + 1)
    flat_hi = lut_q == rail_hi
    flat_lo = lut_q == rail_lo
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

    # Bias-fold assertion — spec §8 rule 1, zero exceptions
    mid = N_lut // 2
    assert lut_q[mid] == 0, f'{stage_name}: lut[{mid}]={lut_q[mid]} — bias-fold broken'

    meta = dict(
        stage=stage_name, N_lut=N_lut, out_bits=out_bits,
        Vout_span_abs=span_abs, span_sign=span_sign,
        G_stage=G_stage, slope_at_zero=slope_at_zero,
        tube_Gss=tube_Gss,
        digital_gain=slope_at_zero * G_stage,
        flat_hi_entries=int(np.sum(flat_hi)),
        flat_lo_entries=int(np.sum(flat_lo)),
    )
    return lut_q, tan_q, meta


# ─────────────────────────────────────────────────────────────────────────────
# PI LUT generator — both LTP arms share a single Vin sweep
# ─────────────────────────────────────────────────────────────────────────────

def _fine_to_lut(x_fine, Vout_fine, Vout_bias, dV, label,
                 N_lut=4097, out_bits=24, force_anti_phase=False):
    """
    Common tail of generate_lut: fine (Vin, Vout) → quantised Q1.23 LUT +
    PCHIP tangents + meta.  Factored out because the PI arms reuse the same
    pipeline but are solved jointly (so they can't share the _solve_vin call).
    """
    Nm1 = N_lut - 1
    assert (Nm1 & (Nm1 - 1)) == 0
    N_fine = len(x_fine)

    span_raw = float(Vout_fine[-1] - Vout_fine[0])
    if abs(span_raw) < 1e-6:
        raise ValueError(f'{label}: degenerate span {span_raw:.3e}')
    span_abs = abs(span_raw)

    h_linear = (Vout_fine - Vout_bias) / span_abs
    mid_fine = N_fine // 2
    h_linear[mid_fine] = 0.0
    slope_linear_at_zero = float(
        PchipInterpolator(x_fine, h_linear).derivative()(0.0)
    )
    Gss_mag = abs(slope_linear_at_zero) * span_abs / dV

    h_fine = h_linear * Gss_mag
    h_fine[mid_fine] = 0.0

    # force_anti_phase: flip LUT content so post-G×LUT output is inverted
    # relative to the natural tube response.  Used for the PI's B-arm — V3B's
    # plate naturally MOVES WITH V3A drive (Vak_B rises when V3A rises), but
    # the phase inverter's contract is to deliver an ANTI-phase companion to
    # y_pos.  We bake the inversion into LUT_B's contents so downstream G
    # can keep the normal sign-of-slope convention.
    if force_anti_phase:
        h_fine = -h_fine

    pchip = PchipInterpolator(x_fine, h_fine)
    assert abs(float(pchip(0.0))) < 1e-12, f'{label}: bias-fold not exact'
    x_lut = np.linspace(-1., 1., N_lut)
    h_lut_raw = pchip(x_lut)
    step = 2.0 / (N_lut - 1)
    dh_lut_raw = pchip.derivative()(x_lut) * step

    slope_at_zero = float(pchip.derivative()(0.0))
    G_stage = float(np.sign(slope_at_zero))
    if force_anti_phase:
        # The `h_fine = -h_fine` flip above already inverted the LUT slope,
        # but `G_stage = sign(slope)` re-picks the sign of the flipped LUT
        # and cancels the inversion in the post-(LUT×G) output — PI_A and
        # PI_B end up in-phase.  Flip G_stage here so the post-G response
        # really is anti-phase to the natural-slope arm.
        G_stage = -G_stage
    tube_Gss = Gss_mag * np.sign(slope_linear_at_zero) * np.sign(G_stage)

    # Soft-rail shoulder (see _soft_rail docstring) — identical treatment to
    # generate_lut so PI arms get the same non-flat shoulder shape as the
    # preamp stages.
    h_lut = _soft_rail(h_lut_raw)
    linear_mask = np.abs(h_lut_raw) <= SOFT_RAIL_KNEE
    dh_lut = np.where(linear_mask, dh_lut_raw, np.gradient(h_lut))

    scale = 2 ** (out_bits - 1) - 1
    lut_q = np.clip(np.round(h_lut * scale), -(scale + 1), scale).astype(np.int32)
    tan_q = np.clip(np.round(dh_lut * scale), -(scale + 1), scale).astype(np.int32)

    rail_hi, rail_lo = scale, -(scale + 1)
    flat_hi = lut_q == rail_hi
    flat_lo = lut_q == rail_lo
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

    mid = N_lut // 2
    assert lut_q[mid] == 0, f'{label}: bias-fold broken ({lut_q[mid]})'

    meta = dict(
        label=label, N_lut=N_lut, out_bits=out_bits,
        Vout_span_abs=span_abs, G_stage=G_stage, slope_at_zero=slope_at_zero,
        tube_Gss=tube_Gss, digital_gain=slope_at_zero * G_stage,
        flat_hi_entries=int(np.sum(flat_hi)),
        flat_lo_entries=int(np.sum(flat_lo)),
        dV=dV, anti_phase=force_anti_phase,
    )
    return lut_q, tan_q, meta


def generate_pi_luts(dc, c, N_lut=4097, N_fine=32769, out_bits=24):
    """
    Build both phase-inverter arm LUTs from one shared Vin sweep of the LTP.
    Returns two (lut_q, tan_q, meta) tuples for arms A and B respectively.
    """
    # dV: drive V3A well past grid-diode onset, so the LUT captures the
    # positive-shoulder region where the cranked 2203's PI lives.  |Vgk_Q| +
    # 3 V leaves ~2·Vt of headroom into grid conduction.
    dV = abs(dc['Vgk_a_q']) + 3.0

    x_fine = nonuniform_grid(N_fine)
    Vin_fine = x_fine * dV
    Vout_a_fine = np.empty_like(Vin_fine)
    Vout_b_fine = np.empty_like(Vin_fine)
    for i, vin in enumerate(Vin_fine):
        s = _solve_ltp_vin(float(vin), dc, c)
        Vout_a_fine[i] = s['Vout_a']
        Vout_b_fine[i] = s['Vout_b']

    # Arm A: natural CC response (Vak_A falls with Vin) — sign handled by G.
    lut_a, tan_a, meta_a = _fine_to_lut(
        x_fine, Vout_a_fine, dc['Vak_a_q'], dV, 'pi_a',
        N_lut=N_lut, out_bits=out_bits, force_anti_phase=False,
    )
    # Arm B: Vak_B rises with Vin (slope positive).  Flip LUT content so
    # post-G output is anti-phase to V3A — that's the whole point of the PI.
    lut_b, tan_b, meta_b = _fine_to_lut(
        x_fine, Vout_b_fine, dc['Vak_b_q'], dV, 'pi_b',
        N_lut=N_lut, out_bits=out_bits, force_anti_phase=True,
    )
    return (lut_a, tan_a, meta_a), (lut_b, tan_b, meta_b)


# ─────────────────────────────────────────────────────────────────────────────
# Q-format helpers
# ─────────────────────────────────────────────────────────────────────────────

def _to_q_signed(x, q_int_bits, q_frac_bits, total_bits):
    """Round x to Q(q_int_bits).(q_frac_bits), clamp to signed `total_bits`-wide."""
    scale = 1 << q_frac_bits
    vmax  = (1 << (total_bits - 1)) - 1
    vmin  = -(1 << (total_bits - 1))
    v = int(round(float(x) * scale))
    return max(vmin, min(vmax, v))


def _hex_signed(v, total_bits):
    """Format signed integer as zero-padded hex at the word width."""
    mask = (1 << total_bits) - 1
    w    = (total_bits + 3) // 4
    return f'{v & mask:0{w}X}'


# ─────────────────────────────────────────────────────────────────────────────
# IIR coefficient emitters
# ─────────────────────────────────────────────────────────────────────────────

def hpf_alpha(tau_s, fs=FS_HZ):
    """Spec §4: α = 1 − 1/(τ·fs).  τ is now per-stage (Cc · Z_next_grid)."""
    return 1.0 - 1.0 / (tau_s * fs)


# Asymmetric DC-tracking time constants for the coupling-cap bias-shift
# ("blocking distortion" / "CF squish" — Gap 1 of the realism plan).
# A real coupling cap charges fast through the conducting grid-diode on
# positive-going peaks (a few ms) and discharges slowly through the grid-
# leak alone (hundreds of ms).  iir_hpf1.sv selects γ_atk when the input
# is climbing above the envelope (cap charging) and γ_rel otherwise.
#
# Symmetric fallback: set TAU_BIAS_ATK_S = TAU_BIAS_REL_S to reproduce the
# pre-asymmetric behaviour (γ_atk == γ_rel in the .mem file).
TAU_BIAS_ATK_S = 0.005   # s — grid-diode forward conduction time constant
TAU_BIAS_REL_S = 0.300   # s — grid-leak RC (Cc · R_grid_leak)


def bias_gamma(tau_s, fs=FS_HZ):
    """One-pole envelope coefficient for the rectified-positive-input tracker.

    env[n] = env[n−1] + γ · (x_pos[n] − env[n−1])
    → time constant ≈ 1/(γ·fs).
    """
    return 1.0 - math.exp(-1.0 / (tau_s * fs))


def lpf_beta(fc_hz, fs=FS_HZ):
    """Spec §5: β = 1 − exp(−2π·fc/fs)."""
    return 1.0 - math.exp(-2.0 * math.pi * fc_hz / fs)


def shelf_biquad(Rk, Ck_F, Rp, gm, fs=FS_HZ, pre_emphasis=False):
    """
    First-order high-shelf in biquad form (b2=a2=0) from bilinear-transform
    of H(s) = (1 + s·τz) / (1 + s·τp) where
        τz = Rk · Ck
        τp = Rk · Ck / (1 + gm·Rk)

    Returns (b0, b1, a1) with a0 normalised to 1.  Direct-Form-I recurrence:
        y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]

    Default (post-LUT placement): DC gain = 1, HF gain = (1+gm·Rk) — the
    shelf boosts HF to recover what cathode degeneration would attenuate.

    pre_emphasis=True (Gap 2 — shelf moved pre-LUT): scale b0, b1 by
    1/(1+gm·Rk) so the shelf now ATTENUATES LF before the LUT instead of
    boosting HF after it.  Overall small-signal frequency response is the
    same as the post-LUT version (modulo LUT non-linearity), but under
    large signal the LF content hits the LUT with less drive — so LF
    content no longer eats headroom that should have been reserved for
    HF transients.  Perceivable as cleaner LF intermod on heavy chords.
    """
    if Ck_F is None:
        # No bypass cap → identity pass-through (either placement)
        return 1.0, 0.0, 0.0
    tau_z = Rk * Ck_F
    tau_p = Rk * Ck_F / (1.0 + gm * Rk)
    K = 2.0 * fs
    # Bilinear: s → K·(1−z⁻¹)/(1+z⁻¹)
    num0 = 1.0 + K * tau_z
    num1 = 1.0 - K * tau_z
    den0 = 1.0 + K * tau_p
    den1 = 1.0 - K * tau_p
    b0 =  num0 / den0
    b1 =  num1 / den0
    a1 =  den1 / den0
    if pre_emphasis:
        scale = 1.0 / (1.0 + gm * Rk)
        b0 *= scale
        b1 *= scale
    return b0, b1, a1


def write_hpf_mem(path, tau_s, timestamp, note, enable_bias_tracker=True):
    α = hpf_alpha(tau_s)
    α_q = _to_q_signed(α, q_int_bits=1, q_frac_bits=31, total_bits=32)
    fc  = 1.0 / (2.0 * math.pi * tau_s)
    # Asymmetric DC-tracking γ (Gap 1) — see TAU_BIAS_ATK_S / TAU_BIAS_REL_S.
    # iir_hpf1.sv selects γ_atk on rising edges (cap charging through grid
    # diode, fast) and γ_rel on falling edges (cap discharging through grid
    # leak, slow).  enable_bias_tracker=False emits both γ coefficients as 0
    # — used on PI plate HPFs where the large drive levels turned the slow
    # envelope tracker into audible low-frequency "fart" / amplitude
    # modulation against the EL34 LUTs.
    γ_atk = bias_gamma(TAU_BIAS_ATK_S) if enable_bias_tracker else 0.0
    γ_rel = bias_gamma(TAU_BIAS_REL_S) if enable_bias_tracker else 0.0
    γ_atk_q = _to_q_signed(γ_atk, q_int_bits=1, q_frac_bits=31, total_bits=32)
    γ_rel_q = _to_q_signed(γ_rel, q_int_bits=1, q_frac_bits=31, total_bits=32)
    τ_atk_b = TAU_BIAS_ATK_S
    τ_rel_b = TAU_BIAS_REL_S
    hdr = (
        f'// JCM800 coupling HPF + asymmetric bias tracker — {note}\n'
        f'// Generated: {timestamp}\n'
        f'// Formula (HPF):   y[n]   = α·(y[n−1] + x[n] − x[n−1])\n'
        f'// Formula (bias):  γ_sel  = (max(x[n],0) > env[n−1]) ? γ_atk : γ_rel\n'
        f'//                  env[n] = env[n−1] + γ_sel·(max(x[n],0) − env[n−1])\n'
        f'//                  y_out[n] = y_hpf[n] − env[n]\n'
        f'// τ_hpf   = {tau_s*1e3:.4f} ms   fc_hpf  = {fc:.4f} Hz   fs = {FS_HZ:.0f} Hz\n'
        f'// α       = {α:.12f}   Q1.31 signed 32-bit → 0x{_hex_signed(α_q, 32)}\n'
        f'// τ_atk   = {τ_atk_b*1e3:.2f} ms   γ_atk = {γ_atk:.12e}   → 0x{_hex_signed(γ_atk_q, 32)}\n'
        f'// τ_rel   = {τ_rel_b*1e3:.2f} ms   γ_rel = {γ_rel:.12e}   → 0x{_hex_signed(γ_rel_q, 32)}\n'
        f'// Width: 32-bit.  Addr 0: α, Addr 1: γ_atk, Addr 2: γ_rel\n'
        f'//\n'
    )
    path.write_text(hdr + _hex_signed(α_q, 32) + '\n'
                        + _hex_signed(γ_atk_q, 32) + '\n'
                        + _hex_signed(γ_rel_q, 32) + '\n')


def write_lpf_mem(path, fc_hz, timestamp, note):
    β = lpf_beta(fc_hz)
    β_q = _to_q_signed(β, 1, 31, 32)
    hdr = (
        f'// JCM800 plate LPF — {note}\n'
        f'// Generated: {timestamp}\n'
        f'// Formula:   y[n] = y[n−1] + β·(x[n] − y[n−1])\n'
        f'// fc = {fc_hz:.2f} Hz   fs = {FS_HZ:.0f} Hz\n'
        f'// β = {β:.12f}   Q1.31 signed 32-bit → 0x{_hex_signed(β_q, 32)}\n'
        f'// Width: 32-bit\n'
        f'//\n'
    )
    path.write_text(hdr + _hex_signed(β_q, 32) + '\n')


def write_shelf_mem(path, b0, b1, a1, timestamp, note, meta_lines):
    # Q3.29 signed 32-bit (range ±4.0) — bilinear-transform coefficients for a
    # high shelf at high sample rate can exceed ±2, so Q2.30 is too narrow.
    # a1 is in (-1,+1) so Q3.29 still leaves 29 fractional bits — ~1.9 ppm LSB.
    b0_q = _to_q_signed(b0, 3, 29, 32)
    b1_q = _to_q_signed(b1, 3, 29, 32)
    a1_q = _to_q_signed(a1, 3, 29, 32)
    hdr = [
        f'// JCM800 cathode shelf — {note}',
        f'// Generated: {timestamp}',
        f'// Formula:   y[n] = b0·x[n] + b1·x[n−1] − a1·y[n−1]   (a0=1, b2=a2=0)',
    ] + [f'// {ml}' for ml in meta_lines] + [
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
# LUT + tangent .mem writer
# ─────────────────────────────────────────────────────────────────────────────

def _write_lut_mem(path, data, bits, header):
    mask = (1 << bits) - 1
    w    = (bits + 3) // 4
    lines = [f'{int(v) & mask:0{w}X}' for v in data]
    path.write_text(header + '\n'.join(lines) + '\n')


# ─────────────────────────────────────────────────────────────────────────────
# SystemVerilog package emitter
# ─────────────────────────────────────────────────────────────────────────────

def write_lut_pkg(pkg_path, params, lut_meta, input_scale_q16_16,
                  script_path=None, timestamp=None, pi_info=None):
    """
    Emit rtl/jcm800_lut_pkg.sv:
      - stage_id_t enum
      - G_STAGE_Q4_20[4]           signed Q4.20 per-stage gain
      - INPUT_SCALE_DEFAULT_Q16_16 single top-level input sensitivity default
      - LUT_FILE / TAN_FILE / LPF_FILE / SHELF_FILE / HPF_FILE arrays
    """
    pkg_path = pathlib.Path(pkg_path)
    pkg_path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = timestamp or datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    src = pathlib.Path(script_path).name if script_path else 'gen_triode_lut.py'

    enum_names = {'v1a':'STAGE_V1A','v1b':'STAGE_V1B',
                  'v2a':'STAGE_V2A','v2b':'STAGE_V2B'}

    g_lines = []
    for i, s in enumerate(_STAGE_ORDER):
        G  = lut_meta[s]['G_stage']
        Gq = _to_q_signed(G, 4, 20, 32)
        trail = ',' if i < len(_STAGE_ORDER) - 1 else ' '
        g_lines.append(f"        32'h{Gq & 0xFFFFFFFF:08X}{trail}   "
                        f"// {enum_names[s]}  G={G:+.4f}  ({'CC' if params[s]['topology']=='cc' else 'CF'})")
    g_block = '\n'.join(g_lines)

    def mem_list(kind, suffix='.mem'):
        lines = []
        for i, s in enumerate(_STAGE_ORDER):
            trail = ',' if i < len(_STAGE_ORDER) - 1 else ''
            lines.append(f'        "{kind}_{s}{suffix}"{trail}'
                         if kind != 'hpf'
                         else f'        "hpf_{s}_out{suffix}"{trail}')
        return '\n'.join(lines)

    # per-stage-keyed mem files
    lut_block   = '\n'.join(f'        "{s}_lut.mem"{"," if i<3 else ""}'
                            for i, s in enumerate(_STAGE_ORDER))
    tan_block   = '\n'.join(f'        "{s}_tan.mem"{"," if i<3 else ""}'
                            for i, s in enumerate(_STAGE_ORDER))
    lpf_block   = '\n'.join(f'        "lpf_{s}.mem"{"," if i<3 else ""}'
                            for i, s in enumerate(_STAGE_ORDER))
    shelf_block = '\n'.join(f'        "shelf_{s}.mem"{"," if i<3 else ""}'
                            for i, s in enumerate(_STAGE_ORDER))
    hpf_block   = '\n'.join(f'        "hpf_{s}_out.mem"{"," if i<3 else ""}'
                            for i, s in enumerate(_STAGE_ORDER))
    pi_block    = _pi_pkg_block(pi_info) if pi_info is not None else ''

    body = f"""//===========================================================================
// jcm800_lut_pkg — AUTO-GENERATED; DO NOT EDIT BY HAND.
//   Produced by: scripts/{src}
//   Timestamp:   {timestamp}
//
//   Per-stage gain constants + .mem filenames for the redesigned gain_stage
//   (LUT + PCHIP + ×G + plate LPF + cathode shelf + coupling HPF).
//===========================================================================
package jcm800_lut_pkg;

    localparam int STAGE_COUNT = 4;

    typedef enum logic [1:0] {{
        STAGE_V1A = 2'd0,
        STAGE_V1B = 2'd1,
        STAGE_V2A = 2'd2,
        STAGE_V2B = 2'd3
    }} stage_id_t;

    // Signed Q4.20 per-stage gain.  RTL op (Q1.23 in, Q1.23 out, convergent round):
    //   prod52 = $signed(lut_q23) * $signed(G_STAGE_Q4_20[stage])    // Q5.43
    //   y_q23  = convergent_round(prod52, 20)                        // → Q1.23
    // G includes the tube phase inversion sign (CC negative, CF ~+0.95).
    localparam int                 SHIFT_G = 20;
    localparam logic signed [31:0] G_STAGE_Q4_20 [STAGE_COUNT] = '{{
{g_block}
    }};

    // Single top-level input sensitivity default (Q16.16, signed 32-bit).
    // Applied ONCE at the ADC boundary (before V1A), not per-stage.  Intended
    // to become a runtime register ("input drive" knob).  RTL op:
    //   x_q23 = saturate_s24( ($signed(adc_q23) * INPUT_SCALE_Q16_16) >>> 16 )
    localparam int                 SHIFT_IN = 16;
    localparam logic signed [31:0] INPUT_SCALE_DEFAULT_Q16_16 = 32'h{input_scale_q16_16 & 0xFFFFFFFF:08X};

    // ─── DEPRECATED — preserved so the legacy preamp.sv still elaborates. ───
    // The redesigned gain_stage puts all per-stage gain in G_STAGE_Q4_20 and
    // all input sensitivity in INPUT_SCALE_DEFAULT_Q16_16.  This array is
    // an identity (1.0×) placeholder so the old cascade's muxed pre-LUT
    // multiplier compiles; it will disappear when preamp.sv is rewired.
    localparam logic signed [31:0] INPUT_SCALE_Q16_16 [STAGE_COUNT] = '{{
        32'h00010000,   // STAGE_V1A  (deprecated; use G_STAGE_Q4_20 + INPUT_SCALE_DEFAULT)
        32'h00010000,   // STAGE_V1B
        32'h00010000,   // STAGE_V2A
        32'h00010000    // STAGE_V2B
    }};

    // LUT + companion tangent-table filenames (4097 × 24-bit Q1.23).
    localparam string LUT_FILE   [STAGE_COUNT] = '{{
{lut_block}
    }};
    localparam string TAN_FILE   [STAGE_COUNT] = '{{
{tan_block}
    }};

    // Plate LPF coefficient files (1 × 32-bit Q1.31 β).
    localparam string LPF_FILE   [STAGE_COUNT] = '{{
{lpf_block}
    }};

    // Cathode shelf biquad coefficient files (3 × 32-bit Q3.29: b0, b1, a1).
    localparam string SHELF_FILE [STAGE_COUNT] = '{{
{shelf_block}
    }};

    // Coupling HPF coefficient files (1 × 32-bit Q1.31 α, τ ≈ 10 ms).
    localparam string HPF_FILE   [STAGE_COUNT] = '{{
{hpf_block}
    }};
{pi_block}
endpackage
"""
    pkg_path.write_text(body)
    return pkg_path


def _pi_pkg_block(pi_info):
    """
    Phase-inverter constants for jcm800_lut_pkg.sv.  Lives OUTSIDE the
    STAGE_COUNT-sized arrays because the PI arms don't iterate with the
    4-stage preamp enum.  Referenced by phase_inverter.sv as defaults.
    """
    G_a_q = pi_info['G_a_q']
    G_b_q = pi_info['G_b_q']
    m_a   = pi_info['meta_a']
    m_b   = pi_info['meta_b']
    dc    = pi_info['dc']
    return f"""
    // ------------------------------------------------------------------
    // Phase-inverter (V3 long-tailed pair) constants — Gap 3 of the
    // realism plan.  A-arm is the signal side (V3A, Rp_a < Rp_b for
    // balance); B-arm encodes V3B's plate response as a function of V3A
    // drive (shared-tail coupling baked in, not input-negation).  Both
    // arms' LUTs are indexed by V3A's grid voltage on the SAME x axis.
    // ------------------------------------------------------------------
    // DC quiescence: Vk_q = {dc['Vk_q']:.3f} V
    //   Ip_a_q = {dc['Ip_a_q']*1e3:.3f} mA  Vak_a_q = {dc['Vak_a_q']:.2f} V
    //   Ip_b_q = {dc['Ip_b_q']*1e3:.3f} mA  Vak_b_q = {dc['Vak_b_q']:.2f} V
    //   A: |span|={m_a['Vout_span_abs']:.2f} V  |Gss|={abs(m_a['tube_Gss']):.2f}  dV=±{m_a['dV']:.2f} V
    //   B: |span|={m_b['Vout_span_abs']:.2f} V  |Gss|={abs(m_b['tube_Gss']):.2f}  dV=±{m_b['dV']:.2f} V
    // LUT_B has its contents pre-inverted AND G_PI_B carries the opposite
    // sign of G_PI_A — the double sign-flip delivers the anti-phase drive
    // the PI contract requires.  The historical "both G_PI values land
    // at −1.0" convention was a bug: `G_stage = sign(slope_at_zero)` of the
    // flipped LUT coincidentally matched the A-arm's G sign, cancelling
    // the flip and producing in-phase outputs.
    localparam logic signed [31:0] G_PI_A_Q4_20 = 32'h{G_a_q & 0xFFFFFFFF:08X};   // {m_a['G_stage']:+.4f}
    localparam logic signed [31:0] G_PI_B_Q4_20 = 32'h{G_b_q & 0xFFFFFFFF:08X};   // {m_b['G_stage']:+.4f}

    localparam string PI_A_LUT_FILE   = "pi_a_lut.mem";
    localparam string PI_A_TAN_FILE   = "pi_a_tan.mem";
    localparam string PI_A_LPF_FILE   = "lpf_pi_a.mem";
    localparam string PI_A_SHELF_FILE = "shelf_pi_a.mem";
    localparam string PI_A_HPF_FILE   = "hpf_pi_a_out.mem";

    localparam string PI_B_LUT_FILE   = "pi_b_lut.mem";
    localparam string PI_B_TAN_FILE   = "pi_b_tan.mem";
    localparam string PI_B_LPF_FILE   = "lpf_pi_b.mem";
    localparam string PI_B_SHELF_FILE = "shelf_pi_b.mem";
    localparam string PI_B_HPF_FILE   = "hpf_pi_b_out.mem";
"""


# ─────────────────────────────────────────────────────────────────────────────
# Phase-inverter export driver (Gap 3)
# ─────────────────────────────────────────────────────────────────────────────

def _export_pi(out_path, timestamp):
    """
    Generate PI A/B .mem artefacts and return the pi_info dict consumed by
    write_lut_pkg.  Output filenames intentionally parallel the preamp
    convention ({label}_{lut,tan}.mem, lpf_{label}.mem, shelf_{label}.mem,
    hpf_{label}_out.mem) so $readmemh load paths look the same.
    """
    c  = _PI_CIRCUIT
    dc = _solve_ltp_dc(c)
    (lut_a, tan_a, m_a), (lut_b, tan_b, m_b) = generate_pi_luts(dc, c)

    for side, lut_q, tan_q, m, Vak_q, Rp in [
        ('a', lut_a, tan_a, m_a, dc['Vak_a_q'], c['Rp_a']),
        ('b', lut_b, tan_b, m_b, dc['Vak_b_q'], c['Rp_b']),
    ]:
        lut_hdr = (
            f'// JCM800 PI_{side.upper()} — LTP long-tailed-pair LUT, Q1.23\n'
            f'// Generated: {timestamp}\n'
            f'// Arm:       {side.upper()}   Rp={Rp/1e3:.0f}kΩ   '
            f'{"(signal)" if side == "a" else "(reference, LUT pre-inverted for anti-phase)"}\n'
            f'// DC:        Vk_Q={dc["Vk_q"]:.3f}V  Vak_Q={Vak_q:.2f}V   '
            f'Ip_Q={(dc["Ip_a_q"] if side == "a" else dc["Ip_b_q"])*1e3:.3f}mA\n'
            f'// dV=±{m["dV"]:.3f}V   |span|={m["Vout_span_abs"]:.3f}V\n'
            f'// Tube |Gss|={abs(m["tube_Gss"]):.3f}  (baked into LUT slope; '
            f'G_stage={m["G_stage"]:+.3f} post-LUT)\n'
            f'// Digital gain at Q-point = {m["digital_gain"]:+.3f}\n'
            f'// Rails:     flat_hi={m["flat_hi_entries"]}  flat_lo={m["flat_lo_entries"]}\n'
            f'// addr 2048 ↔ Vin=0 ↔ lut=0 (bias-fold invariant)\n'
            f'//\n'
        )
        tan_hdr = lut_hdr.replace('LUT,', 'PCHIP tangents,')\
                          .replace('lut=0 (bias-fold)',
                                   'tan = (df/dx)·step, zeroed at rails')
        _write_lut_mem(out_path / f'pi_{side}_lut.mem', lut_q, 24, lut_hdr)
        _write_lut_mem(out_path / f'pi_{side}_tan.mem', tan_q, 24, tan_hdr)

        # Plate LPF: Miller uses small-signal |G| from the LUT's linear slope.
        G_small = abs(m['tube_Gss'])
        C_plate = C_STRAY_PLT + CGP_TUBE * (1.0 + G_small)
        R_ac    = Rp * c['R_pa_grid'] / (Rp + c['R_pa_grid'])
        fc_lpf  = 1.0 / (2.0 * math.pi * R_ac * C_plate)
        fc_lpf  = max(1e3, min(fc_lpf, FS_HZ * 0.49))
        write_lpf_mem(
            out_path / f'lpf_pi_{side}.mem', fc_lpf, timestamp,
            f'PI_{side.upper()} plate LPF '
            f'(R_ac={R_ac/1e3:.1f}k, C={C_plate*1e12:.1f}pF)',
        )

        # Shelf: identity — shared tail cathode, no per-arm bypass cap.
        write_shelf_mem(
            out_path / f'shelf_pi_{side}.mem', 1.0, 0.0, 0.0, timestamp,
            f'PI_{side.upper()} cathode shelf',
            ['Identity — shared cathode, no per-arm bypass cap'],
        )

        # Coupling HPF: τ = Cc · R_pa_grid (each arm drives one EL34 grid)
        tau_hpf = c['Cc_F'] * c['R_pa_grid']
        fc_hpf  = 1.0 / (2.0 * math.pi * tau_hpf)
        write_hpf_mem(
            out_path / f'hpf_pi_{side}_out.mem', tau_hpf, timestamp,
            f'PI_{side.upper()} output coupling '
            f'(Cc={c["Cc_F"]*1e9:.1f}nF, Z_next={c["R_pa_grid"]/1e3:.0f}kΩ, '
            f'τ={tau_hpf*1e3:.2f}ms, fc={fc_hpf:.2f}Hz)',
            enable_bias_tracker=True,
        )

    return dict(
        G_a_q=_to_q_signed(m_a['G_stage'], 4, 20, 32),
        G_b_q=_to_q_signed(m_b['G_stage'], 4, 20, 32),
        meta_a=m_a, meta_b=m_b, dc=dc,
    )


# ─────────────────────────────────────────────────────────────────────────────
# Top-level export driver
# ─────────────────────────────────────────────────────────────────────────────

def export_all(out_dir='lut_out', pkg_path='rtl/jcm800_lut_pkg.sv',
               verbose=True, self_test=False):
    """
    Compute everything, write every .mem and the SV package.  Returns a
    summary dict the caller can inspect (used by --self-test).
    """
    out_path = pathlib.Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    params   = compute_stage_params()
    lut_meta = {}

    # ── LUTs + tangents ─────────────────────────────────────────────────
    for s, p in params.items():
        lut_q, tan_q, meta = generate_lut(s, p)
        lut_meta[s] = meta

        lut_hdr = (
            f'// JCM800 {s.upper()} — LUT Gss · (Vout−Vout_bias)/|span|, Q1.23\n'
            f'// Generated: {timestamp}\n'
            f'// Topology:  {p["topology"].upper()}   Vgk_Q={p["Vgk_q"]:+.4f} V   '
            f'Vak_Q={p["Vak_q"]:.3f} V   dV=±{p["dV"]:.4f} V\n'
            f'// AC load:   R_ac={p["R_ac"]/1e3:.2f} kΩ   '
            f'|ΔVout_span|={meta["Vout_span_abs"]:.3f} V\n'
            f'// Tube |Gss|={abs(meta["tube_Gss"]):.3f}  (baked into LUT slope; '
            f'G_stage={meta["G_stage"]:+.3f} post-LUT)\n'
            f'// Digital gain at Q-point (slope × G_stage, Q1.23/Q1.23) = '
            f'{meta["digital_gain"]:+.3f}\n'
            f'// Rails:     flat_hi={meta["flat_hi_entries"]}  flat_lo={meta["flat_lo_entries"]}\n'
            f'// addr 2048 ↔ x=0 ↔ Vgk=Vgk_Q ↔ lut=0 (bias-fold)\n'
            f'//\n'
        )
        tan_hdr = lut_hdr.replace('LUT f_norm', 'PCHIP tangents') \
                          .replace('lut=0 (bias-fold)',
                                   'tan = (df/dx)·step, zeroed at rails')
        _write_lut_mem(out_path / f'{s}_lut.mem', lut_q, 24, lut_hdr)
        _write_lut_mem(out_path / f'{s}_tan.mem', tan_q, 24, tan_hdr)

    # ── Coupling HPFs — per-stage τ = Cc · Z_next_grid ──────────────────
    # Old design used a single TAU_COUPLING_S = 10 ms for every stage, leaving
    # every coupling HPF at fc ≈ 16 Hz.  Four cascaded 16-Hz HPFs are still
    # flat at 50 Hz, so sub-guitar energy (pedal-board rumble, string handling
    # noise) was IMD-ing upward through the cascade and producing the flubby
    # low end.  Now each stage's coupling HPF reflects its actual schematic
    # coupling cap loaded by the next stage's grid network.
    # Gap 1 (asymmetric coupling-cap bias tracker) is per-stage opt-in.
    # V1A → V1B is the cold-clipper feed: its grid diode pumps the coupling
    # cap on positive peaks, then the cap discharges slowly through V1B's
    # grid leak — that's the iconic "blocking distortion" / strangled cold
    # clipper recovery.  Other preamp coupling HPFs and the PI output HPF
    # currently stay symmetric (set False) — turn on after listening tests.
    BIAS_TRACKER_STAGES = {'v1a', 'v1b', 'v2a'}
    for s in _STAGE_ORDER:
        Cc      = _CIRCUIT[s]['Cc_F']
        Z_next  = _next_grid_Z(s)
        tau_s   = Cc * Z_next
        fc_s    = 1.0 / (2.0 * math.pi * tau_s)
        write_hpf_mem(out_path / f'hpf_{s}_out.mem',
                      tau_s, timestamp,
                      f'{s} output coupling '
                      f'(Cc={Cc*1e9:.1f}nF, Z_next={Z_next/1e3:.0f}kΩ, '
                      f'τ={tau_s*1e3:.2f}ms, fc={fc_s:.2f}Hz)',
                      enable_bias_tracker=(s in BIAS_TRACKER_STAGES))
        params[s]['fc_hpf'] = fc_s

    # ── Plate LPFs (fc from Rp_ac · C_plate_total; CF → wideband) ───────
    # Small-signal Miller (Cgp·(1+G)) is a worst-case estimate; under large
    # signal the dynamic gain drops and the effective Miller pole sits well
    # above this calc, so high-order harmonics survive into the next stage's
    # grid in real circuits.  Floor CC stages at PLATE_LPF_FLOOR_HZ to
    # restore that headroom.
    PLATE_LPF_FLOOR_HZ = 60_000.0
    for s, p in params.items():
        if p['topology'] == 'cc':
            fc = 1.0 / (2.0 * math.pi * p['R_ac'] * p['C_plate'])
            fc = max(fc, PLATE_LPF_FLOOR_HZ)
        else:
            # CF: cathode output impedance is ~1/gm, so corner is very high;
            # cap at Nyquist·0.95 to stay numerically sane (β → ~1 anyway).
            fc = FS_HZ * 0.45
        # Clamp to safe range
        fc = max(1e3, min(fc, FS_HZ * 0.49))
        p['fc_lpf'] = fc
        write_lpf_mem(out_path / f'lpf_{s}.mem', fc, timestamp,
                      f'{s} plate LPF (R_ac={p["R_ac"]/1e3:.1f}k, '
                      f'C={p["C_plate"]*1e12:.1f}pF)')

    # ── Cathode shelves (Gap 2: V1A uses pre-emphasis form, moved pre-LUT) ─
    # V1A is the ONLY bypassed-cathode stage.  In the old layout the shelf
    # sat post-LPF, boosting HF to recover what cathode degeneration would
    # attenuate at LF — but by then the LUT had already clipped the full-gain
    # signal.  Gap 2: pre-emphasis form (b0, b1 scaled by 1/(1+gm·Rk)) moves
    # the shelf pre-LUT so LF content enters the LUT with its correctly-
    # degenerated gain, and no further shelving is needed post-LUT.  The
    # post-LUT shelf for V1A therefore collapses to identity below.
    for s, p in params.items():
        v1a_pre = (s == 'v1a') and (p['Ck_F'] is not None)
        if p['Ck_F'] is None:
            b0, b1, a1 = 1.0, 0.0, 0.0
            meta_lines = [f'No cathode bypass cap → identity pass-through']
        elif v1a_pre:
            # V1A's shelf moves pre-LUT — this `shelf_v1a.mem` file is now
            # loaded by the pre-LUT slot in gain_stage.sv (SHELF_PRE_LUT=1).
            eps = 1e-5
            gm = float((koren_Ip(p['Vgk_q']+eps, p['Vak_q'], **_KOREN)
                       - koren_Ip(p['Vgk_q']-eps, p['Vak_q'], **_KOREN)) / (2.*eps))
            b0, b1, a1 = shelf_biquad(p['Rk'], p['Ck_F'], p['Rp'], gm,
                                       pre_emphasis=True)
            tau_z = p['Rk'] * p['Ck_F']
            tau_p = tau_z / (1.0 + gm * p['Rk'])
            k_scale = 1.0 / (1.0 + gm * p['Rk'])
            meta_lines = [
                f'Gap 2 — PRE-LUT placement (SHELF_PRE_LUT=1 in preamp.sv)',
                f'Rk={p["Rk"]:.1f}Ω  Ck={p["Ck_F"]*1e6:.3f}μF  gm={gm*1e3:.3f}mA/V',
                f'τ_zero={tau_z*1e3:.4f} ms (fc_z={1/(2*math.pi*tau_z):.2f} Hz)',
                f'τ_pole={tau_p*1e3:.4f} ms (fc_p={1/(2*math.pi*tau_p):.2f} Hz)',
                f'Pre-emphasis scale 1/(1+gm·Rk)={k_scale:.4f}',
                f'DC gain={k_scale:.4f}  HF gain=1.0000  (shelf attenuates LF)',
            ]
        else:
            # (Unreachable today — only V1A has a bypass cap — but kept so
            # future schematic changes gracefully pick up the non-pre form.)
            eps = 1e-5
            gm = float((koren_Ip(p['Vgk_q']+eps, p['Vak_q'], **_KOREN)
                       - koren_Ip(p['Vgk_q']-eps, p['Vak_q'], **_KOREN)) / (2.*eps))
            b0, b1, a1 = shelf_biquad(p['Rk'], p['Ck_F'], p['Rp'], gm)
            tau_z = p['Rk'] * p['Ck_F']
            tau_p = tau_z / (1.0 + gm * p['Rk'])
            meta_lines = [
                f'POST-LUT placement (SHELF_PRE_LUT=0)',
                f'Rk={p["Rk"]:.1f}Ω  Ck={p["Ck_F"]*1e6:.3f}μF  gm={gm*1e3:.3f}mA/V',
                f'τ_zero={tau_z*1e3:.4f} ms (fc_z={1/(2*math.pi*tau_z):.2f} Hz)',
                f'τ_pole={tau_p*1e3:.4f} ms (fc_p={1/(2*math.pi*tau_p):.2f} Hz)',
                f'DC gain=1.0  HF gain={1+gm*p["Rk"]:.4f}  (shelf ratio)',
            ]
        write_shelf_mem(out_path / f'shelf_{s}.mem', b0, b1, a1,
                        timestamp, f'{s} cathode shelf', meta_lines)

    # ── Interstage "Attenuator + Treble Peak" (Bright) pads ──────────────
    # Two of these on the schematic, both 470 kΩ series + 470 pF "Treble
    # Peak" cap in parallel, feeding the next stage's 470 kΩ grid leak:
    #
    #     prev plate ─ Cc ─┬── R_series (470 kΩ) ──┬── next grid
    #                      │                       │
    #                      └─── C_peak (470 pF) ───┘
    #                                              │
    #                                          R_grid_leak (470 kΩ)
    #                                              │
    #                                             GND
    #
    # Continuous transfer:
    #   H(s) = K · (1 + s·τz) / (1 + s·τp)
    #     K   = R_grid_leak / (R_series + R_grid_leak)        (DC gain)
    #     τz  = R_series · C_peak                             (zero)
    #     τp  = (R_series ∥ R_grid_leak) · C_peak             (pole)
    #     HF  = K · τz/τp = 1.0   (R_series=R_grid_leak ⇒ flat above pole)
    #
    # We reuse `shelf_biquad(Rk=R_series, Ck=C_peak, Rp=∅, gm=1/R_grid_leak)`
    # which produces a shelf with τz=Rk·Ck and τp=Rk·Ck/(1+gm·Rk) matching
    # the bright-pad math (the 1+gm·Rk = (R_series+R_grid_leak)/R_grid_leak
    # = 1/K identity).  Its DC gain is 1 and HF gain is 1+gm·Rk = 1/K, so we
    # post-scale b0/b1 by K to land DC=K, HF=1.
    BRIGHT_PADS = {
        'pre_pot':   dict(R_series=470e3, C_peak=470e-12, R_grid_leak=470e3,
                          where='V1A plate → 0.022 µF coupling → THIS PAD → '
                                'top of Preamp Volume pot (1 MA).  '
                                'Schematic: between V1B plate and Preamp '
                                'Volume in real-circuit naming.'),
        'v1b_v2a':   dict(R_series=470e3, C_peak=470e-12, R_grid_leak=470e3,
                          where='V1B plate → 0.022 µF coupling → THIS PAD → '
                                'V2A grid (470 kΩ grid leak).  '
                                'Schematic: between V1A (cold clipper) plate '
                                'and V2A grid.'),
    }
    for tag, bp in BRIGHT_PADS.items():
        Rs, Cp, Rg = bp['R_series'], bp['C_peak'], bp['R_grid_leak']
        K   = Rg / (Rs + Rg)
        b0, b1, a1 = shelf_biquad(Rs, Cp, 0.0, 1.0 / Rg)
        b0 *= K
        b1 *= K
        tau_z = Rs * Cp
        tau_p = (Rs * Rg / (Rs + Rg)) * Cp
        meta_lines = [
            bp['where'],
            f'R_series={Rs/1e3:.1f}kΩ  C_peak={Cp*1e12:.1f}pF  '
            f'R_grid_leak={Rg/1e3:.1f}kΩ',
            f'τ_zero={tau_z*1e6:.2f} µs (fc_z={1/(2*math.pi*tau_z):.1f} Hz)',
            f'τ_pole={tau_p*1e6:.2f} µs (fc_p={1/(2*math.pi*tau_p):.1f} Hz)',
            f'DC gain={K:.4f} ({20*math.log10(K):+.2f} dB)  '
            f'HF gain={K*tau_z/tau_p:.4f} '
            f'({20*math.log10(K*tau_z/tau_p):+.2f} dB)',
        ]
        write_shelf_mem(out_path / f'bright_pad_{tag}.mem', b0, b1, a1,
                        timestamp, f'bright pad ({tag})', meta_lines)

    # ── Input-scale default (maps ADC full-scale → V1A grid-swing domain) ─
    # V1A's LUT bakes the tube's small-signal voltage gain (Gss ≈ 41.5) into
    # the LUT slope, so the LUT's soft-rail knee (h ≈ 0.92) is reached at an
    # input of x ≈ 0.92/Gss ≈ 0.022 — i.e. any LUT input above ~−33 dBFS
    # saturates V1A.  A typical electric-guitar pickup at the Hi jack hits the
    # I2S2 ADC at roughly −20 to −10 dBFS, which without pre-attenuation would
    # leave V1A permanently clipped regardless of the downstream Preamp Volume
    # pot — making the gain pot behave like a volume control.  Pre-attenuating
    # by 0.1 (−20 dB) keeps a typical −20 dBFS guitar signal in V1A's linear
    # region (LUT input ≈ −40 dBFS) while leaving hot humbucker chord-attack
    # peaks (~−10 dBFS) just barely above V1A's soft-knee — so V1A
    # contributes a hint of harmonic colour on transients without smearing
    # the gain-pot's clean window.  Originally tuned at 0.03 because the
    # pre-V1A scaler hadn't been wired into the RTL yet; once
    # rtl/input_scale.sv landed, that left the chain too clean and 0.1
    # restored useful drive.  Users with quiet pickups can override via
    # the runtime input-drive register.
    input_scale_q16_16 = _to_q_signed(0.2, 16, 16, 32)

    # ── Phase inverter (Gap 3) — LTP-aware LUTs + filter coeffs ─────────
    pi_info = _export_pi(out_path, timestamp)

    # ── SystemVerilog package ───────────────────────────────────────────
    write_lut_pkg(pathlib.Path(pkg_path), params, lut_meta,
                  input_scale_q16_16, script_path=__file__,
                  timestamp=timestamp, pi_info=pi_info)

    # ── Human-readable summary ──────────────────────────────────────────
    if verbose:
        print(f'\nJCM800 redesign artifacts → {out_path.resolve()}')
        print(f'  fs={FS_HZ:.0f} Hz   (coupling HPF τ now per-stage)')
        print(f'\n{"stage":<6} {"topo":<3} {"Vgk_Q":>8} {"Vak_Q":>8} {"dV":>8} '
              f'{"R_ac_k":>8} {"span_V":>8} {"|Gss|":>7} {"G_post":>7} '
              f'{"digGain":>8} {"fc_lpf":>9} {"fc_hpf":>8} {"flat_hi":>8}')
        for s, p in params.items():
            m = lut_meta[s]
            print(f'{s:<6} {p["topology"]:<3} {p["Vgk_q"]:>+8.4f} {p["Vak_q"]:>8.2f} '
                  f'{p["dV"]:>8.4f} {p["R_ac"]/1e3:>8.2f} {m["Vout_span_abs"]:>8.3f} '
                  f'{abs(m["tube_Gss"]):>7.3f} {m["G_stage"]:>+7.2f} '
                  f'{m["digital_gain"]:>+8.3f} {p["fc_lpf"]:>9.1f} '
                  f'{p["fc_hpf"]:>8.2f} {m["flat_hi_entries"]:>8d}')
        print(f'\nphase-inverter (LTP, Gap 3):')
        print(f'  DC: Vk_q={pi_info["dc"]["Vk_q"]:.3f} V  '
              f'Ip_a_q={pi_info["dc"]["Ip_a_q"]*1e3:.3f} mA  '
              f'Ip_b_q={pi_info["dc"]["Ip_b_q"]*1e3:.3f} mA  '
              f'Vak_a_q={pi_info["dc"]["Vak_a_q"]:.2f} V  '
              f'Vak_b_q={pi_info["dc"]["Vak_b_q"]:.2f} V')
        for label, m in [('pi_a', pi_info['meta_a']), ('pi_b', pi_info['meta_b'])]:
            print(f'  {label}: |span|={m["Vout_span_abs"]:>7.3f} V  '
                  f'|Gss|={abs(m["tube_Gss"]):>6.3f}  '
                  f'G_stage={m["G_stage"]:+5.2f}  '
                  f'flat_hi={m["flat_hi_entries"]:>4d}  '
                  f'flat_lo={m["flat_lo_entries"]:>4d}  '
                  f'anti_phase={m["anti_phase"]}')

    # ── Self-test assertions ────────────────────────────────────────────
    if self_test:
        scale = (1 << 23) - 1
        for s in _STAGE_ORDER:
            m = lut_meta[s]
            # 1. Bias-fold exact zero at mid index
            lut_q, tan_q, _ = generate_lut(s, params[s])
            assert lut_q[2048] == 0, f'{s}: bias-fold broken'
            # 2. G_stage fits Q4.20
            Gq = _to_q_signed(m['G_stage'], 4, 20, 32)
            assert abs(Gq) < (1 << 27), f'{s}: |G_stage| overflows Q4.20'
            # 3. HPF α resolvable at this stage's actual τ
            tau_s = params[s]['Cc_F'] * _next_grid_Z(s)
            α = hpf_alpha(tau_s)
            α_q = _to_q_signed(α, 1, 31, 32)
            err_ppm = abs(α_q / float(1 << 31) - α) / α * 1e6
            assert err_ppm < 10, f'{s}: HPF α quantisation error {err_ppm:.3f} ppm'
            # 4. LPF β in range
            β = lpf_beta(params[s]['fc_lpf'])
            assert 0.0 < β < 1.0, f'{s}: LPF β={β:.3e} out of range'
        print('\n  self-test: all assertions passed ✓')

    return dict(params=params, lut_meta=lut_meta,
                input_scale_q16_16=input_scale_q16_16)


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='JCM800 LUT + IIR coefficient generator')
    ap.add_argument('--out',        type=str, default='lut_out',
                    help='Output directory for .mem files')
    ap.add_argument('--pkg',        type=str, default='rtl/jcm800_lut_pkg.sv',
                    help='SV package destination')
    ap.add_argument('--self-test',  action='store_true',
                    help='Run sanity assertions after export')
    args = ap.parse_args()

    export_all(out_dir=args.out, pkg_path=args.pkg,
               verbose=True, self_test=args.self_test)
