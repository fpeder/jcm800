"""
probe_attack.py — diagnose "choked attack" on strong single notes

Goal: expose the two candidate envelopes that could be causing the user's
"note briefly swallowed then recovers" percept at high gain + high master:

  (1) preamp coupling-HPF bias-tracker subtract, stacked across 4 stages
  (2) power-amp screen sag (vg2_ratio droop)

We build a floating-point model of the JCM800 signal chain at 768 kHz, load
the same .mem files the RTL consumes (single source of truth for coefficients
and LUT curves), feed a synthetic pick-attack stimulus at several gain/master
settings, record per-sample internal nodes, and emit PNG plots + a numeric
summary into build/probe/.

Read-only w.r.t. rtl/ and lut_out/.  No Vivado.  Run:

    python3 scripts/probe_attack.py
"""

from __future__ import annotations

import pathlib
import re
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False


FS = 768_000.0              # oversampled rate
REPO = pathlib.Path(__file__).resolve().parent.parent
LUT_DIR = REPO / "lut_out"
OUT_DIR = REPO / "build" / "probe"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# -----------------------------------------------------------------------------
# .mem helpers — parse hex, interpret as signed fixed-point
# -----------------------------------------------------------------------------

def _read_hex_lines(path: pathlib.Path) -> list[str]:
    out = []
    with open(path) as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("//"):
                continue
            # occasional @addr markers — ignore; take the hex token
            tok = s.split()[0]
            if tok.startswith("@"):
                continue
            out.append(tok)
    return out


def _s_from_hex(hex_str: str, width: int) -> int:
    v = int(hex_str, 16)
    if v >= (1 << (width - 1)):
        v -= (1 << width)
    return v


def load_q1_23_array(name: str) -> np.ndarray:
    """Load a .mem file as float Q1.23 (divide by 2^23)."""
    rows = _read_hex_lines(LUT_DIR / name)
    vals = np.array([_s_from_hex(r, 24) for r in rows], dtype=np.float64)
    return vals / (1 << 23)


def load_q1_31_scalar(name: str, index: int = 0) -> float:
    rows = _read_hex_lines(LUT_DIR / name)
    return _s_from_hex(rows[index], 32) / (1 << 31)


def load_shelf_q3_29(name: str) -> tuple[float, float, float]:
    rows = _read_hex_lines(LUT_DIR / name)
    b0 = _s_from_hex(rows[0], 32) / (1 << 29)
    b1 = _s_from_hex(rows[1], 32) / (1 << 29)
    a1 = _s_from_hex(rows[2], 32) / (1 << 29)
    return b0, b1, a1


def load_log_taper() -> np.ndarray:
    """256-entry Q0.24 audio taper. Return float in [0, ~1]."""
    rows = _read_hex_lines(LUT_DIR / "log_taper_256.mem")
    # Stored as Q0.24 unsigned (<=1.0).  Read as 24-bit unsigned.
    vals = np.array([int(r, 16) for r in rows[:256]], dtype=np.float64)
    return vals / (1 << 24)


# -----------------------------------------------------------------------------
# LUT interpolation — linear (adequate for envelope diagnosis)
# -----------------------------------------------------------------------------

class ClampLut:
    """
    4097-entry Q1.23 LUT indexed by a Q1.23 signed input.
    addr = clamp(int((x+1)/2 * 4096), 0, 4095); linear interp to addr+1.
    Matches the RTL's bias + 12-LSB-margin clamp closely enough for
    envelope-level diagnosis.
    """
    def __init__(self, data: np.ndarray):
        assert data.size >= 4097, f"LUT size {data.size}"
        self.y = data[:4097]

    def eval(self, x: np.ndarray | float) -> np.ndarray | float:
        x_c = np.clip(x, -0.999, +0.999)
        biased = (x_c + 1.0) * 0.5            # [0,1]
        idx_f = biased * 4096.0
        i0 = np.clip(idx_f.astype(int) if isinstance(idx_f, np.ndarray) else int(idx_f), 0, 4095)
        t = idx_f - i0
        y0 = self.y[i0]
        y1 = self.y[i0 + 1]
        return y0 + (y1 - y0) * t


# -----------------------------------------------------------------------------
# Stage data classes
# -----------------------------------------------------------------------------

class CouplingHPF:
    """
    First-order HPF + asymmetric bias tracker, sample-by-sample.
    Matches rtl/iir_hpf1.sv: y_hpf[n] = α·(y_hpf[n-1] + x[n] - x[n-1]);
    env[n] = env[n-1] + γ·(max(x[n],0) - env[n-1]);
    y[n] = y_hpf[n] - env[n] >> SHIFT_BIAS.

    If `gamma_release` is not None, we use γ_attack when x_pos>env (cap
    charging through grid diode, fast) and γ_release when x_pos<=env (cap
    discharging through grid leak, slow) — matching real coupling-cap
    physics.
    """
    def __init__(self, alpha: float, gamma: float, shift_bias: int = 3,
                 gamma_release: float | None = None):
        self.alpha = alpha
        self.gamma_atk = gamma
        self.gamma_rel = gamma if gamma_release is None else gamma_release
        self.scale = 2.0 ** (-shift_bias)
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.env = 0.0

    def step(self, x: float) -> tuple[float, float]:
        y_hpf = self.alpha * (self.y_prev + x - self.x_prev)
        x_pos = x if x > 0.0 else 0.0
        env_pre = self.env
        g = self.gamma_atk if x_pos > env_pre else self.gamma_rel
        self.env = env_pre + g * (x_pos - env_pre)
        self.x_prev = x
        self.y_prev = y_hpf
        # env subtract uses pre-update env to match RTL state ordering
        y = y_hpf - env_pre * self.scale
        return y, env_pre * self.scale


class PlateLPF:
    """First-order LPF: y[n] = y[n-1] + β·(x[n] - y[n-1])."""
    def __init__(self, beta: float):
        self.beta = beta
        self.y = 0.0

    def step(self, x: float) -> float:
        self.y = self.y + self.beta * (x - self.y)
        return self.y


class ShelfBiquad:
    """y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1]  (first-order biquad)."""
    def __init__(self, b0: float, b1: float, a1: float):
        self.b0, self.b1, self.a1 = b0, b1, a1
        self.x_prev = 0.0
        self.y_prev = 0.0

    def step(self, x: float) -> float:
        y = self.b0 * x + self.b1 * self.x_prev - self.a1 * self.y_prev
        self.x_prev = x
        self.y_prev = y
        return y


class GainStage:
    """
    LUT+PCHIP → ×G → plate LPF → (shelf) → coupling HPF with bias tracker.
    The cathode shelf is applied either pre-LUT (V1A) or post-LPF; for
    envelope diagnosis we skip the shelf (minor on envelopes).  Plate LPF
    is included because it shapes the signal that feeds the HPF.
    """
    def __init__(self, name: str, lut_mem: str, hpf_mem: str, lpf_mem: str,
                 g_stage: float, shift_bias: int = 3):
        self.name = name
        self.lut = ClampLut(load_q1_23_array(lut_mem))
        alpha = load_q1_31_scalar(hpf_mem, 0)
        gamma = load_q1_31_scalar(hpf_mem, 1)
        beta = load_q1_31_scalar(lpf_mem, 0)
        self.hpf = CouplingHPF(alpha, gamma, shift_bias=shift_bias)
        self.lpf = PlateLPF(beta)
        self.g = g_stage
        # Telemetry
        self.env_scaled: list[float] = []   # per-sample subtract magnitude

    def step(self, x: float) -> float:
        f_norm = self.lut.eval(x)
        post_g = self.g * f_norm
        lpf_y = self.lpf.step(post_g)
        y, env_scaled = self.hpf.step(lpf_y)
        self.env_scaled.append(env_scaled)
        # Clip to Q1.23 rails
        if y > 0.999999:
            y = 0.999999
        elif y < -0.999999:
            y = -0.999999
        return y


class PentodeStage:
    """Plate-current LUT + ×G_EL34 + ×vg2_ratio.  Returns (ip, ig2_proxy)."""
    def __init__(self, lut_mem: str, g_el34: float, ig2_ratio: float):
        self.lut = ClampLut(load_q1_23_array(lut_mem))
        self.g = g_el34
        self.ig2_ratio = ig2_ratio

    def step(self, x: float, vg2_ratio: float) -> tuple[float, float]:
        f = self.lut.eval(x)
        ip = vg2_ratio * self.g * f
        ig2 = abs(ip) * self.ig2_ratio
        return ip, ig2


class ScreenSupply:
    """state += α·(1 - state) - β·ig2_sum; output vg2_ratio."""
    def __init__(self, alpha: float, beta: float):
        self.alpha = alpha
        self.beta = beta
        self.state = 1.0

    def step(self, ig2_sum: float) -> float:
        self.state = self.state + self.alpha * (1.0 - self.state) - self.beta * ig2_sum
        # Don't let the state run negative (screen voltage can't invert)
        if self.state < 0.1:
            self.state = 0.1
        return self.state


class OutputTransformer:
    """
    Push-pull diff → 0.5 scale → primary-L HPF (no bias tracker) → core sat LUT
    → leakage LPF → v_sec.
    """
    def __init__(self):
        alpha = load_q1_31_scalar("hpf_prim.mem", 0)
        # γ is 0 for OT path
        self.hpf = CouplingHPF(alpha, 0.0, shift_bias=31)
        beta = load_q1_31_scalar("lpf_leak.mem", 0)
        self.lpf = PlateLPF(beta)
        self.sat_lut = ClampLut(load_q1_23_array("ot_sat_lut.mem"))

    def step(self, ip_a: float, ip_b: float) -> float:
        i_prim = 0.5 * (ip_a - ip_b)
        i_hp, _ = self.hpf.step(i_prim)
        i_sat = self.sat_lut.eval(i_hp)
        v_sec = self.lpf.step(i_sat)
        return v_sec


class NFBNetwork:
    """z⁻¹ → ×NFB_coef → presence shelf → log-taper pot → nfb_inject."""
    def __init__(self, nfb_coef: float, presence_pot_pos: int, log_taper: np.ndarray):
        self.nfb_coef = nfb_coef
        self.shelf = ShelfBiquad(*load_shelf_q3_29("shelf_presence.mem"))
        self.taper = float(log_taper[presence_pot_pos])
        self.prev_v_sec = 0.0

    def step(self, v_sec: float) -> float:
        delayed = self.prev_v_sec
        self.prev_v_sec = v_sec
        scaled = self.nfb_coef * delayed
        shelfed = self.shelf.step(scaled)
        return self.taper * shelfed


# -----------------------------------------------------------------------------
# Full chain
# -----------------------------------------------------------------------------

class JCM800Probe:
    def __init__(self, gain_pot: int, master_pot: int,
                 presence_pot: int = 128,
                 shift_bias: int = 3,
                 tau_bias_s: float | None = None,
                 tau_bias_atk_s: float | None = None,
                 tau_bias_rel_s: float | None = None):
        self.log_taper = load_log_taper()
        self.gain_att = float(self.log_taper[gain_pot])
        self.master_att = float(self.log_taper[master_pot])

        # Per-stage preamp: LUT G=-1 for V1A/V1B/V2A, +1 for V2B
        # (matches jcm800_lut_pkg G_STAGE_Q4_20)
        self.v1a = GainStage("V1A", "v1a_lut.mem", "hpf_v1a_out.mem", "lpf_v1a.mem",
                             g_stage=-1.0, shift_bias=shift_bias)
        self.v1b = GainStage("V1B", "v1b_lut.mem", "hpf_v1b_out.mem", "lpf_v1b.mem",
                             g_stage=-1.0, shift_bias=shift_bias)
        self.v2a = GainStage("V2A", "v2a_lut.mem", "hpf_v2a_out.mem", "lpf_v2a.mem",
                             g_stage=-1.0, shift_bias=shift_bias)
        self.v2b = GainStage("V2B", "v2b_lut.mem", "hpf_v2b_out.mem", "lpf_v2b.mem",
                             g_stage=+1.0, shift_bias=shift_bias)
        # PI arms pull their G from the pkg (LUT_B is pre-inverted by the
        # generator so the two arms can have opposite G signs under a
        # shared natural-slope convention — hardcoding both to −1.0 masked
        # the generator's anti-phase fix).
        g_pi_a = load_q4_20_scalar_pkg_hex("G_PI_A_Q4_20")
        g_pi_b = load_q4_20_scalar_pkg_hex("G_PI_B_Q4_20")
        self.pi_a = GainStage("PI_A", "pi_a_lut.mem", "hpf_pi_a_out.mem", "lpf_pi_a.mem",
                              g_stage=g_pi_a, shift_bias=shift_bias)
        self.pi_b = GainStage("PI_B", "pi_b_lut.mem", "hpf_pi_b_out.mem", "lpf_pi_b.mem",
                              g_stage=g_pi_b, shift_bias=shift_bias)

        # Optionally override bias-tracker τ everywhere (sweep knob)
        if tau_bias_s is not None:
            gamma = 1.0 - np.exp(-1.0 / (FS * tau_bias_s))
            for s in (self.v1a, self.v1b, self.v2a, self.v2b, self.pi_a, self.pi_b):
                s.hpf.gamma_atk = gamma
                s.hpf.gamma_rel = gamma
        if tau_bias_atk_s is not None:
            g_atk = 1.0 - np.exp(-1.0 / (FS * tau_bias_atk_s))
            for s in (self.v1a, self.v1b, self.v2a, self.v2b, self.pi_a, self.pi_b):
                s.hpf.gamma_atk = g_atk
        if tau_bias_rel_s is not None:
            g_rel = 1.0 - np.exp(-1.0 / (FS * tau_bias_rel_s))
            for s in (self.v1a, self.v1b, self.v2a, self.v2b, self.pi_a, self.pi_b):
                s.hpf.gamma_rel = g_rel

        # Power amp: EL34, screen supply, OT, NFB
        alpha_scr = load_q1_31_scalar_pkg_hex("SCREEN_ALPHA_Q1_31")
        beta_scr = load_q1_31_scalar_pkg_hex("SCREEN_BETA_Q1_31")
        ig2_ratio = load_q1_23_scalar_pkg_hex("IG2_RATIO_Q1_23")
        nfb_coef = load_q4_20_scalar_pkg_hex("NFB_Q4_20")
        g_el34 = load_q4_20_scalar_pkg_hex("G_EL34_Q4_20")

        self.pent_a = PentodeStage("el34_lut.mem", g_el34, ig2_ratio)
        self.pent_b = PentodeStage("el34_lut.mem", g_el34, ig2_ratio)
        self.screen = ScreenSupply(alpha_scr, beta_scr)
        self.ot = OutputTransformer()
        self.nfb = NFBNetwork(nfb_coef, presence_pot, self.log_taper)

        # Telemetry
        self.t: list[float] = []
        self.v1a_y: list[float] = []
        self.v1b_y: list[float] = []
        self.v2a_y: list[float] = []
        self.v2b_y: list[float] = []
        self.pi_pos: list[float] = []
        self.pi_neg: list[float] = []
        self.ip_a_hist: list[float] = []
        self.ip_b_hist: list[float] = []
        self.ig2_sum: list[float] = []
        self.vg2_ratio_hist: list[float] = []
        self.v_sec_hist: list[float] = []
        self.nfb_inject_hist: list[float] = []

    def run(self, x_in: np.ndarray) -> None:
        nfb_inject = 0.0
        vg2_ratio = 1.0
        for n, x in enumerate(x_in):
            self.t.append(n / FS)
            # Preamp cascade
            y1a = self.v1a.step(x)
            self.v1a_y.append(y1a)
            y1b = self.v1b.step(self.gain_att * y1a)
            self.v1b_y.append(y1b)
            y2a = self.v2a.step(y1b)
            self.v2a_y.append(y2a)
            y2b = self.v2b.step(y2a)
            self.v2b_y.append(y2b)
            # Master attenuator
            pi_in = self.master_att * y2b - nfb_inject
            # Clamp to rails for PI input
            pi_in = max(-0.999999, min(0.999999, pi_in))
            y_pos = self.pi_a.step(pi_in)
            y_neg = self.pi_b.step(pi_in)
            self.pi_pos.append(y_pos)
            self.pi_neg.append(y_neg)
            # Pentodes use current vg2_ratio (updated below)
            ip_a, ig2_a = self.pent_a.step(y_pos, vg2_ratio)
            ip_b, ig2_b = self.pent_b.step(y_neg, vg2_ratio)
            self.ip_a_hist.append(ip_a)
            self.ip_b_hist.append(ip_b)
            ig2sum = ig2_a + ig2_b
            self.ig2_sum.append(ig2sum)
            vg2_ratio = self.screen.step(ig2sum)
            self.vg2_ratio_hist.append(vg2_ratio)
            # OT
            v_sec = self.ot.step(ip_a, ip_b)
            self.v_sec_hist.append(v_sec)
            # NFB for next sample
            nfb_inject = self.nfb.step(v_sec)
            self.nfb_inject_hist.append(nfb_inject)


# -----------------------------------------------------------------------------
# Package-constant lookup (parse jcm800_power_pkg.sv once)
# -----------------------------------------------------------------------------

_PKG_TEXT = ((REPO / "rtl" / "jcm800_power_pkg.sv").read_text()
             + "\n"
             + (REPO / "rtl" / "jcm800_lut_pkg.sv").read_text())


def _pkg_hex(name: str) -> str:
    m = re.search(rf"{name}\s*=\s*32'h([0-9A-Fa-f_]+)", _PKG_TEXT)
    if not m:
        m = re.search(rf"{name}\s*=\s*24'h([0-9A-Fa-f_]+)", _PKG_TEXT)
    if not m:
        raise KeyError(name)
    return m.group(1).replace("_", "")


def load_q1_31_scalar_pkg_hex(name: str) -> float:
    return _s_from_hex(_pkg_hex(name), 32) / (1 << 31)


def load_q1_23_scalar_pkg_hex(name: str) -> float:
    return _s_from_hex(_pkg_hex(name), 24) / (1 << 23)


def load_q4_20_scalar_pkg_hex(name: str) -> float:
    return _s_from_hex(_pkg_hex(name), 32) / (1 << 20)


# -----------------------------------------------------------------------------
# Stimulus
# -----------------------------------------------------------------------------

def pick_attack(duration_s: float = 0.5,
                f_hz: float = 220.0,
                amp: float = 0.05,
                tau_rise_s: float = 0.002,
                tau_fall_s: float = 0.300,
                hold_s: float = 0.050) -> np.ndarray:
    """Sine with AR envelope mimicking a guitar pick attack."""
    n = int(duration_s * FS)
    t = np.arange(n) / FS
    sine = np.sin(2 * np.pi * f_hz * t)
    env = np.zeros(n)
    rise_end = int(tau_rise_s * 3 * FS)
    hold_end = rise_end + int(hold_s * FS)
    # Exp-rise to 1.0 over tau_rise
    rise_n = np.arange(rise_end)
    env[:rise_end] = 1.0 - np.exp(-rise_n / (tau_rise_s * FS))
    env[rise_end:hold_end] = 1.0
    # Exp-fall
    fall_n = np.arange(n - hold_end)
    env[hold_end:] = np.exp(-fall_n / (tau_fall_s * FS))
    return amp * sine * env


# -----------------------------------------------------------------------------
# Plot helpers
# -----------------------------------------------------------------------------

def summarize(probe: JCM800Probe, x_in: np.ndarray, title: str,
              out_path: pathlib.Path) -> dict:
    t = np.array(probe.t) * 1000.0   # ms
    env_total = (np.array(probe.v1a.env_scaled)
                 + np.array(probe.v1b.env_scaled)
                 + np.array(probe.v2a.env_scaled)
                 + np.array(probe.v2b.env_scaled)
                 + np.array(probe.pi_a.env_scaled)
                 + np.array(probe.pi_b.env_scaled))
    vg2 = np.array(probe.vg2_ratio_hist)
    v_sec = np.array(probe.v_sec_hist)

    # Always emit a decimated CSV (every 16 samples ≈ 48 kHz) for external plotting
    dec = 16
    csv_path = out_path.with_suffix(".csv")
    with open(csv_path, "w") as f:
        f.write("t_ms,input,v1a,v1b,v2a,v2b,env_total,vg2_ratio,v_sec\n")
        for i in range(0, len(t), dec):
            f.write(f"{t[i]:.4f},{x_in[i]:+.6f},"
                    f"{probe.v1a_y[i]:+.6f},{probe.v1b_y[i]:+.6f},"
                    f"{probe.v2a_y[i]:+.6f},{probe.v2b_y[i]:+.6f},"
                    f"{env_total[i]:+.6f},{vg2[i]:.6f},{v_sec[i]:+.6f}\n")

    if not HAVE_MPL:
        # Numeric summary only
        result = {
            "env_peak_pct":   float(100.0 * env_total.max()),
            "env_peak_t_ms":  float(t[int(np.argmax(env_total))]) if env_total.size else 0.0,
            "vg2_min":        float(vg2.min()),
            "vg2_droop_pct":  float(100.0 * (1.0 - vg2.min())),
            "vg2_min_t_ms":   float(t[int(np.argmin(vg2))]) if vg2.size else 0.0,
            "v_sec_peak":     float(np.abs(v_sec).max()),
        }
        return result

    fig, ax = plt.subplots(5, 1, figsize=(11, 11), sharex=True)

    ax[0].plot(t, x_in, color="0.3")
    ax[0].set_ylabel("input")
    ax[0].set_title(title)

    ax[1].plot(t, probe.v1a_y, label="V1A", alpha=0.6)
    ax[1].plot(t, probe.v2a_y, label="V2A", alpha=0.6)
    ax[1].plot(t, probe.v2b_y, label="V2B", alpha=0.6)
    ax[1].legend(loc="upper right", fontsize=8)
    ax[1].set_ylabel("preamp stages")

    ax[2].plot(t, probe.v1a.env_scaled, label="V1A", alpha=0.7)
    ax[2].plot(t, probe.v1b.env_scaled, label="V1B", alpha=0.7)
    ax[2].plot(t, probe.v2a.env_scaled, label="V2A", alpha=0.7)
    ax[2].plot(t, probe.v2b.env_scaled, label="V2B", alpha=0.7)
    ax[2].plot(t, env_total, "k", lw=1.5, label="sum (all 6)")
    ax[2].legend(loc="upper right", fontsize=8)
    ax[2].set_ylabel("bias subtract")

    ax[3].plot(t, vg2, color="tab:red")
    ax[3].axhline(1.0, color="k", lw=0.4, ls=":")
    ax[3].set_ylabel("vg2_ratio")
    ax[3].set_ylim(min(0.8, vg2.min() - 0.02), 1.02)

    ax[4].plot(t, v_sec, color="tab:green", lw=0.6)
    ax[4].set_ylabel("v_sec")
    ax[4].set_xlabel("time (ms)")

    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)

    # Numeric summary
    def peak_time_ms(sig):
        i = int(np.argmax(np.abs(sig - sig[0]) if sig.size else 0))
        return t[i] if sig.size else 0.0

    return {
        "env_peak_pct":   float(100.0 * env_total.max()),
        "env_peak_t_ms":  float(peak_time_ms(env_total)),
        "vg2_min":        float(vg2.min()),
        "vg2_droop_pct":  float(100.0 * (1.0 - vg2.min())),
        "vg2_min_t_ms":   float(t[int(np.argmin(vg2))]),
        "v_sec_peak":     float(np.abs(v_sec).max()),
    }


def plot_overlay(probes: list[tuple[str, JCM800Probe]],
                 out_path: pathlib.Path) -> None:
    if not HAVE_MPL:
        return
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for label, p in probes:
        t = np.array(p.t) * 1000.0
        env_total = (np.array(p.v1a.env_scaled)
                     + np.array(p.v1b.env_scaled)
                     + np.array(p.v2a.env_scaled)
                     + np.array(p.v2b.env_scaled)
                     + np.array(p.pi_a.env_scaled)
                     + np.array(p.pi_b.env_scaled))
        ax[0].plot(t, env_total, label=label)
        ax[1].plot(t, p.vg2_ratio_hist, label=label)
    ax[0].set_ylabel("bias subtract (sum, 6 stages)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("vg2_ratio")
    ax[1].set_xlabel("time (ms)")
    ax[1].axhline(1.0, color="k", lw=0.4, ls=":")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Main sweep
# -----------------------------------------------------------------------------

def main() -> None:
    stim = pick_attack()
    combos = [
        ("gain032_master032", 32,  32),
        ("gain032_master220", 32,  220),
        ("gain128_master032", 128, 32),
        ("gain128_master220", 128, 220),
        ("gain220_master032", 220, 32),
        ("gain220_master220", 220, 220),
    ]

    print(f"{'combo':<24}  env_peak%  env_t_ms  vg2_min  droop%  vg2_t_ms  vsec_pk")
    probes: list[tuple[str, JCM800Probe]] = []
    for name, gp, mp in combos:
        probe = JCM800Probe(gp, mp)
        probe.run(stim)
        out = summarize(probe, stim, f"{name} — shift_bias=3, tau_bias=100ms",
                             OUT_DIR / f"dashboard_{name}.png")
        print(f"{name:<24}  "
              f"{out['env_peak_pct']:8.2f}  "
              f"{out['env_peak_t_ms']:7.1f}  "
              f"{out['vg2_min']:7.3f}  "
              f"{out['vg2_droop_pct']:6.2f}  "
              f"{out['vg2_min_t_ms']:7.1f}  "
              f"{out['v_sec_peak']:7.3f}")
        probes.append((name, probe))

    plot_overlay(probes, OUT_DIR / "overlay_envelopes.png")

    # Python-model A/B: worst-case combo with tuning knobs changed
    print()
    print("A/B (python model only, no RTL change) at gain220_master220:")
    for label, kwargs in [
        ("baseline",                        dict()),
        ("asym5_300_sb5",                   dict(shift_bias=5, tau_bias_atk_s=0.005, tau_bias_rel_s=0.300)),
        ("asym5_300_sb6",                   dict(shift_bias=6, tau_bias_atk_s=0.005, tau_bias_rel_s=0.300)),
        ("asym10_500_sb5",                  dict(shift_bias=5, tau_bias_atk_s=0.010, tau_bias_rel_s=0.500)),
        ("asym10_500_sb6",                  dict(shift_bias=6, tau_bias_atk_s=0.010, tau_bias_rel_s=0.500)),
        ("asym20_500_sb5",                  dict(shift_bias=5, tau_bias_atk_s=0.020, tau_bias_rel_s=0.500)),
    ]:
        p = JCM800Probe(220, 220, **kwargs)
        p.run(stim)
        out = summarize(p, stim, f"gain220_master220 — {label}",
                             OUT_DIR / f"ab_{label.replace(' ', '_').replace('=', '')}.png")
        print(f"  {label:<22}  "
              f"env_peak%={out['env_peak_pct']:6.2f}  "
              f"env_t_ms={out['env_peak_t_ms']:6.1f}  "
              f"vg2_droop%={out['vg2_droop_pct']:5.2f}  "
              f"vg2_t_ms={out['vg2_min_t_ms']:6.1f}")

    print()
    print(f"PNGs and dashboards written to {OUT_DIR}")


if __name__ == "__main__":
    sys.exit(main())
