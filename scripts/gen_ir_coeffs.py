#!/usr/bin/env python3
"""
gen_ir_coeffs.py — IR cabinet-sim FIR coefficient LUT

Reads ir/OD-M212-VINT-DYN-57-P00-00-L.wav (Orange 2x12 + SM57, 44.1 kHz, mono,
24-bit), resamples to 48 kHz, truncates to K_TAPS with a short cosine fade on
the tail, normalizes so the peak of the magnitude response is 0 dB, applies a
user MAKEUP_DB of headroom, and quantizes to signed Q1.17 (18-bit, same format
as lut_out/hb*_e*.mem).

Output: lut_out/ir_cab.mem — hex (5 digits per line), $readmemh-loadable.
Consumed by rtl/ir_cab.sv.
"""

from __future__ import annotations

import argparse
import math
import wave
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly


ROOT         = Path(__file__).resolve().parent.parent
DEFAULT_WAV  = ROOT / "ir" / "OD-M212-VINT-DYN-57-P00-00-L.wav"
DEFAULT_OUT  = ROOT / "lut_out" / "ir_cab.mem"

SRC_RATE     = 44100
DST_RATE     = 48000
K_TAPS       = 1024
FADE_TAPS    = 64
COEF_W       = 18
COEF_Q       = 17
MAKEUP_DB    = +6.0


def read_wav_24bit_mono(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as w:
        if w.getnchannels() != 1:
            raise ValueError(f"{path.name}: expected mono, got {w.getnchannels()} ch")
        if w.getsampwidth() != 3:
            raise ValueError(f"{path.name}: expected 24-bit, got {w.getsampwidth() * 8}-bit")
        rate   = w.getframerate()
        nframe = w.getnframes()
        raw    = w.readframes(nframe)
    b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
    s = (b[:, 0].astype(np.int32)
         | (b[:, 1].astype(np.int32) << 8)
         | (b[:, 2].astype(np.int32) << 16))
    s = np.where(s & 0x800000, s - 0x1000000, s)
    return s.astype(np.float64) / float(1 << 23), rate


def cosine_fade_tail(h: np.ndarray, fade_taps: int) -> np.ndarray:
    if fade_taps <= 0 or fade_taps >= len(h):
        return h
    out = h.copy()
    n = np.arange(fade_taps)
    win = 0.5 * (1.0 + np.cos(math.pi * n / (fade_taps - 1)))  # 1 → 0
    out[-fade_taps:] *= win
    return out


def normalize_peak_response(h: np.ndarray, nfft: int = 4096) -> tuple[np.ndarray, float]:
    H = np.fft.rfft(h, n=nfft)
    mag_peak = float(np.max(np.abs(H)))
    if mag_peak <= 0.0:
        raise ValueError("IR has zero magnitude response")
    return h / mag_peak, mag_peak


def quantize_signed(h: np.ndarray, coef_q: int, coef_w: int) -> np.ndarray:
    scale  = 1 << coef_q
    lo     = -(1 << (coef_w - 1))
    hi     =  (1 << (coef_w - 1)) - 1
    q      = np.round(h * scale).astype(np.int64)
    return np.clip(q, lo, hi)


def spot_db(h: np.ndarray, rate: int, freq_hz: float, nfft: int = 4096) -> float:
    H = np.fft.rfft(h, n=nfft)
    f = np.fft.rfftfreq(nfft, 1.0 / rate)
    k = int(np.argmin(np.abs(f - freq_hz)))
    m = abs(H[k])
    return 20.0 * math.log10(m) if m > 0 else -math.inf


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate ir_cab.mem from a wav IR")
    ap.add_argument("--wav",       type=Path, default=DEFAULT_WAV)
    ap.add_argument("--out",       type=Path, default=DEFAULT_OUT)
    ap.add_argument("--taps",      type=int,  default=K_TAPS)
    ap.add_argument("--fade",      type=int,  default=FADE_TAPS)
    ap.add_argument("--makeup-db", type=float, default=MAKEUP_DB)
    args = ap.parse_args()

    h_src, src_rate = read_wav_24bit_mono(args.wav)
    if src_rate != SRC_RATE:
        raise ValueError(f"expected {SRC_RATE} Hz source, got {src_rate}")

    up, down = 160, 147   # 48000/44100 reduced
    h_48 = resample_poly(h_src, up, down)

    if len(h_48) < args.taps:
        h_48 = np.pad(h_48, (0, args.taps - len(h_48)))
    h_trunc = h_48[: args.taps]
    h_fade  = cosine_fade_tail(h_trunc, args.fade)

    h_norm, peak_unnorm = normalize_peak_response(h_fade)

    makeup_lin = 10.0 ** (args.makeup_db / 20.0)
    h_scaled   = h_norm * makeup_lin

    coef_int = quantize_signed(h_scaled, COEF_Q, COEF_W)

    # Achieved magnitude-response stats on the quantized IR
    h_q_float = coef_int.astype(np.float64) / float(1 << COEF_Q)
    H_q = np.fft.rfft(h_q_float, n=4096)
    peak_q_db = 20.0 * math.log10(float(np.max(np.abs(H_q))))

    spots_hz = (100.0, 250.0, 500.0, 1000.0, 2000.0, 3000.0, 4000.0, 6000.0,
                8000.0, 12000.0)
    spots    = [(f, spot_db(h_q_float, DST_RATE, f)) for f in spots_hz]

    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    header = [
        f"// IR cabinet FIR — {args.taps} taps @ {DST_RATE} Hz",
        f"// Generated: {stamp}  by scripts/gen_ir_coeffs.py",
        f"// Source:    {args.wav.relative_to(ROOT) if args.wav.is_absolute() and ROOT in args.wav.parents else args.wav}",
        f"// Resample:  {SRC_RATE} -> {DST_RATE} via polyphase (up={up}, down={down})",
        f"// Window:    cosine fade on last {args.fade} taps",
        f"// Norm:      peak |H(w)|=1.0 then makeup {args.makeup_db:+.2f} dB",
        f"// Format:    Q{COEF_W - COEF_Q}.{COEF_Q} signed, {COEF_W}-bit "
        f"({(COEF_W + 3) // 4} hex digits), $readmemh",
        f"// Achieved:  peak |H(w)| on quantized IR = {peak_q_db:+.3f} dB",
        f"// Spot (quantized):",
    ]
    for f, db in spots:
        if math.isfinite(db):
            header.append(f"//   {f:>7.0f} Hz  {db:+7.3f} dB")
        else:
            header.append(f"//   {f:>7.0f} Hz       -inf dB")
    header.append("")

    hex_w = (COEF_W + 3) // 4
    mask  = (1 << COEF_W) - 1
    lines = header + [f"{int(v) & mask:0{hex_w}x}".upper() for v in coef_int]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("\n".join(lines) + "\n")

    print(f"wrote {args.out}  ({len(coef_int)} taps, peak |H| = {peak_q_db:+.3f} dB)")


if __name__ == "__main__":
    main()
