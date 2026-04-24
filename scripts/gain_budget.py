#!/usr/bin/env python3
"""
gain_budget.py — stage-by-stage dB budget for the JCM800 DSP chain.

Reads the packages + .mem LUTs in this repo and prints the designed
small-signal level at every tap in the pipeline for a given reference
input (default 1 kHz sine at -20 dBFS) and pot positions.  No RTL
simulation; pure offline math.  Used to spot where signal is being lost
when the DAC output is quieter than expected.

    python scripts/gain_budget.py                            # defaults
    python scripts/gain_budget.py --gain 255 --master 255    # pots wide open
    python scripts/gain_budget.py --freq 100                 # probe at 100 Hz
"""

from __future__ import annotations

import argparse
import cmath
import math
import re
from pathlib import Path

ROOT    = Path(__file__).resolve().parent.parent
LUT_DIR = ROOT / "lut_out"

FS_OS  = 768_000         # oversampled rate inside the amp chain
FS_BB  = 48_000          # baseband rate (upsampler in, IR cab, DAC)

# --- constants mirrored from the SV packages ---------------------------
#   rtl/jcm800_lut_pkg.sv:25-30 (G_STAGE_Q4_20, SHIFT_G=20)
#   rtl/jcm800_power_pkg.sv:29-61 (G_EL34, PP_HALF, NFB)
#   Values are Q4.20, re-expressed as floats here.
G_STAGE   = {"V1A": -1.0, "V1B": -1.0, "V2A": -1.0, "V2B": +1.0}
G_PI_A    = -1.0          # jcm800_lut_pkg.sv:102
G_EL34    = +1.0          # jcm800_power_pkg.sv:30
PP_HALF   =  0.5          # jcm800_power_pkg.sv:61
NFB       = 0x0000B852 / (1 << 20)   # jcm800_power_pkg.sv:66 → ≈0.045
INPUT_SCL = 1.0           # jcm800_lut_pkg.sv:37 → 0x00010000 Q16.16

# --- helpers -----------------------------------------------------------
def read_mem(name: str) -> list[int]:
    """Parse a $readmemh-style file into a list of ints."""
    out: list[int] = []
    for raw in (LUT_DIR / name).read_text().splitlines():
        s = raw.split("//", 1)[0].strip()
        if s:
            out.append(int(s, 16))
    return out


def signed(value: int, bits: int) -> int:
    mask = (1 << bits) - 1
    v = value & mask
    if v & (1 << (bits - 1)):
        v -= 1 << bits
    return v


def lut_header_gain(name: str) -> float:
    """
    Extract the 'Digital gain at Q-point' value from a LUT .mem header.
    Every generator (gen_triode_lut.py, gen_pentode_lut.py) prints this
    already — reusing it avoids re-implementing the PCHIP slope math.
    """
    text = (LUT_DIR / name).read_text()
    m = re.search(r"Digital gain at Q-point[^=]*=\s*([+\-]?[0-9.]+)", text)
    if not m:
        raise RuntimeError(f"no digital gain header in {name}")
    return float(m.group(1))


def db(x: float) -> float:
    return -math.inf if x == 0 else 20.0 * math.log10(abs(x))


# --- filter frequency responses @ one probe freq -----------------------
def lpf_mag(beta: float, f: float, fs: float) -> float:
    """y[n] = y[n-1] + β(x[n] - y[n-1])  →  H(z) = β / (1 - (1-β)z⁻¹)"""
    z = cmath.exp(-1j * 2 * math.pi * f / fs)
    return abs(beta / (1 - (1 - beta) * z))


def hpf_mag(alpha: float, f: float, fs: float) -> float:
    """y[n] = α(y[n-1] + x[n] - x[n-1])  →  H(z) = α(1-z⁻¹) / (1-αz⁻¹)"""
    z = cmath.exp(-1j * 2 * math.pi * f / fs)
    return abs(alpha * (1 - z) / (1 - alpha * z))


def shelf_mag(b0: float, b1: float, a1: float, f: float, fs: float) -> float:
    """y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1]"""
    z = cmath.exp(-1j * 2 * math.pi * f / fs)
    return abs((b0 + b1 * z) / (1 + a1 * z))


def load_lpf(name: str) -> float:
    return signed(read_mem(name)[0], 32) / (1 << 31)


def load_hpf(name: str) -> float:
    # First row = α (Q1.31); second row (if present) = γ bias tracker, unused here.
    return signed(read_mem(name)[0], 32) / (1 << 31)


def load_shelf(name: str) -> tuple[float, float, float]:
    words = read_mem(name)
    b0 = signed(words[0], 32) / (1 << 29)
    b1 = signed(words[1], 32) / (1 << 29)
    a1 = signed(words[2], 32) / (1 << 29)
    return b0, b1, a1


def log_taper_db(pot: int) -> float:
    coef = read_mem("log_taper_256.mem")[pot] / (1 << 24)
    return db(coef), coef


def ir_cab_mag(f: float, fs: float = FS_BB) -> tuple[float, float]:
    taps = [signed(v, 18) / (1 << 17) for v in read_mem("ir_cab.mem")]
    dc = sum(taps)
    s = sum(h * cmath.exp(-1j * 2 * math.pi * k * f / fs) for k, h in enumerate(taps))
    return dc, abs(s)


# --- stage aggregations ------------------------------------------------
def preamp_stage_gain(stage: str, f: float) -> float:
    s = stage.lower()
    lut    = lut_header_gain(f"{s}_lut.mem")   # already signed × G_stage
    lpf    = lpf_mag(load_lpf(f"lpf_{s}.mem"), f, FS_OS)
    b0,b1,a1 = load_shelf(f"shelf_{s}.mem")
    shelf  = shelf_mag(b0, b1, a1, f, FS_OS)
    hpf    = hpf_mag(load_hpf(f"hpf_{s}_out.mem"), f, FS_OS)
    return lut * lpf * shelf * hpf        # signed linear gain


def pi_stage_gain(f: float) -> float:
    lut    = lut_header_gain("pi_a_lut.mem")
    lpf    = lpf_mag(load_lpf("lpf_pi_a.mem"), f, FS_OS)
    b0,b1,a1 = load_shelf("shelf_pi_a.mem")
    shelf  = shelf_mag(b0, b1, a1, f, FS_OS)
    hpf    = hpf_mag(load_hpf("hpf_pi_a_out.mem"), f, FS_OS)
    return lut * lpf * shelf * hpf


def power_stage_gain(f: float) -> float:
    """EL34 single-side × PP_HALF × OT filters."""
    lut    = lut_header_gain("el34_lut.mem")
    hpf_ot = hpf_mag(load_hpf("hpf_prim.mem"), f, FS_OS)
    lpf_ot = lpf_mag(load_lpf("lpf_leak.mem"), f, FS_OS)
    return lut * PP_HALF * hpf_ot * lpf_ot


def nfb_loop_correction(el34_open: float, pres_pot: int, f: float) -> float:
    """
    Closed-loop correction at this freq:  1 / (1 + βA).
    βA = NFB · el34_open · presence_shelf(f) · presence_pot.
    Presence pot attenuates the shelf ONLY (pot at 0 → βA≈0, loop open).
    """
    b0,b1,a1 = load_shelf("shelf_presence.mem")
    pres_shelf = shelf_mag(b0, b1, a1, f, FS_OS)
    pres_atten = read_mem("log_taper_256.mem")[pres_pot] / (1 << 24)
    beta_a = NFB * abs(el34_open) * pres_shelf * pres_atten
    return 1.0 / (1.0 + beta_a), beta_a


# --- main --------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--input-dbfs", type=float, default=-20.0)
    ap.add_argument("--freq",       type=float, default=1000.0)
    ap.add_argument("--gain",       type=int,   default=0xC0, help="gain_pot_pos 0..255")
    ap.add_argument("--master",     type=int,   default=0xC0, help="master_pot_pos 0..255")
    ap.add_argument("--pres",       type=int,   default=0x80, help="presence_pot_pos 0..255")
    args = ap.parse_args()

    f = args.freq
    gain_db,  gain_coef  = log_taper_db(args.gain)
    mstr_db,  mstr_coef  = log_taper_db(args.master)
    pres_db,  pres_coef  = log_taper_db(args.pres)

    rows: list[tuple[str, float]] = []   # (label, stage_gain_db)
    rows.append(("ADC input",                 0.0))
    rows.append(("INPUT_SCALE (×1.0)",        db(INPUT_SCL)))
    rows.append(("upsample_16x  (halfband, in-band)", 0.0))  # unity by design
    rows.append((f"V1A  (LUT+LPF+shelf+HPF @ {f:g} Hz)", db(preamp_stage_gain("V1A", f))))
    rows.append((f"Gain pot (pos={args.gain})",  gain_db))
    rows.append(("V1B",                       db(preamp_stage_gain("V1B", f))))
    rows.append(("V2A",                       db(preamp_stage_gain("V2A", f))))
    rows.append(("V2B (CF)",                  db(preamp_stage_gain("V2B", f))))
    rows.append((f"Master pot (pos={args.master})", mstr_db))
    rows.append(("Phase inverter (V3A arm)",  db(pi_stage_gain(f))))
    el34_open = power_stage_gain(f)
    rows.append(("EL34 + PP_HALF + OT (open-loop)", db(el34_open)))
    # Downsampler: halfband, in-band unity by design.
    rows.append(("downsample_16x  (halfband, in-band)", 0.0))
    dc_ir, mag_ir = ir_cab_mag(f)
    rows.append((f"ir_cab FIR (@ {f:g} Hz)",  db(mag_ir)))
    rows.append(("DAC output",                0.0))

    nfb_lin, beta_a = nfb_loop_correction(el34_open, args.pres, f)
    nfb_db = db(nfb_lin)

    # --- print -----------------------------------------------------------
    print(f"JCM800 dB budget  — input: {f:g} Hz sine @ {args.input_dbfs:+.2f} dBFS")
    print(f"gain_pot_pos = {args.gain:3d} ({gain_db:+6.2f} dB)   "
          f"master_pot_pos = {args.master:3d} ({mstr_db:+6.2f} dB)   "
          f"presence_pot_pos = {args.pres:3d} ({pres_db:+6.2f} dB)")
    print()
    print(f"{'#':>2}  {'tap':44s} {'stage (dB)':>12s}  {'cumul (dB)':>12s}  {'abs (dBFS)':>12s}")
    cum = 0.0
    for i, (label, stg) in enumerate(rows):
        cum += stg
        abs_db = args.input_dbfs + cum
        stg_s  = f"{stg:+7.2f}" if math.isfinite(stg) else "  -inf "
        print(f"{i:>2}  {label:44s} {stg_s:>12s}  {cum:>+12.2f}  {abs_db:>+12.2f}")
    print()
    print(f"NFB closed-loop correction @ {f:g} Hz: "
          f"βA = {beta_a:.4f}  →  1/(1+βA) = {nfb_db:+.2f} dB")
    print("   (subtract this from every row ≥ 9 for closed-loop estimate;")
    print("    beta_a goes to 0 as presence_pot_pos → 0, making the loop open)")


if __name__ == "__main__":
    main()
