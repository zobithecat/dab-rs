#!/usr/bin/env python3
"""Slice-11 Part B: arg(r) heatmap diagnostic.

Reads the per-frame differential spectra dump produced by
`DAB_RS_DUMP_DIFF_SPEC=path dab fic-iq` and analyses the per-carrier
`arg(r)` distribution across the three FIC OFDM symbols of each
frame, looking for the structural signature that distinguishes the
four remaining bug hypotheses on the slice-11 fork list.

Pattern A — linear-in-carrier slope of `arg(r) − Δ_expected`:
            sub-sample timing offset between PRS and data symbols.
            cp.fine_time picked a sample index that's ±1 off,
            introducing a phase ramp `exp(j*2π*k*Δ_sample/T_u)`
            across bins.

Pattern B — uniform constant offset across all carriers, same per
            symbol: fractional-CFO residual / NCO sign error /
            integer-CFO miscount. Every carrier picks up the same
            rotation, so the differential demap quadrants are
            cyclically permuted but the bit pair structure is
            preserved (just at the wrong constellation rotation).

Pattern C — per-symbol varying offset: fractional-CFO drift through
            the FIC region. Frequency tracking loop needs adjustment.

Pattern D — random / non-structural: deeper mismatch, FFT bin
            order / sign / scale is itself suspect.

Real signals don't carry a known `Δ_expected`, so this script clusters
the observed `arg(r)` values onto the four π/4-DQPSK constellation
points (`π/4, 3π/4, 5π/4, 7π/4`) and reports the *residual rotation*
each carrier shows away from the nearest cluster centre. A healthy
chain produces residuals ~ 0 (or ± a few degrees) for active carriers
and noise on the guard bins.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

T_U = 2048
K_CARRIERS = 1536
N_SYMS = 3
HEADER = 4
RECORD = HEADER + N_SYMS * T_U * 8

# π/4-DQPSK target angles in radians.
TARGET_ANGLES = [math.pi / 4, 3 * math.pi / 4, -3 * math.pi / 4, -math.pi / 4]


def bin_of_carrier(c: int) -> int:
    return c if c > 0 else c + T_U


def active_carriers() -> list[int]:
    return list(range(-K_CARRIERS // 2, 0)) + list(range(1, K_CARRIERS // 2 + 1))


def fold_to_nearest(theta: float) -> tuple[float, int]:
    """Return (residual, cluster_idx) where residual = theta − nearest
    target angle, wrapped into [-π/4, π/4]."""
    best_r = 1e9
    best_i = 0
    for i, target in enumerate(TARGET_ANGLES):
        r = theta - target
        # wrap to [-π, π]
        r = (r + math.pi) % (2 * math.pi) - math.pi
        if abs(r) < abs(best_r):
            best_r = r
            best_i = i
    return best_r, best_i


def analyse_frame(buf: bytes) -> None:
    """Print a one-frame summary."""
    (frame_idx,) = struct.unpack_from("<I", buf, 0)
    print(f"Frame {frame_idx}")
    print(f"  {'symbol':<7}  {'mean |arg(r)|':>14}  "
          f"{'mean residual':>14}  {'stddev residual':>16}  "
          f"{'cluster spread (rad)':>22}")

    # Pattern detection accumulators.
    per_symbol_residual_mean = []
    per_symbol_residual_std = []
    per_symbol_slopes = []

    for s in range(N_SYMS):
        off = HEADER + s * T_U * 8
        spec = []
        for k in range(T_U):
            re = struct.unpack_from("<f", buf, off + k * 8)[0]
            im = struct.unpack_from("<f", buf, off + k * 8 + 4)[0]
            spec.append(complex(re, im))

        # Walk active carriers only.
        residuals = []
        args = []
        carriers = active_carriers()
        for c in carriers:
            r = spec[bin_of_carrier(c)]
            if abs(r.real) + abs(r.imag) < 1e-9:
                continue
            theta = math.atan2(r.imag, r.real)
            args.append(theta)
            res, _idx = fold_to_nearest(theta)
            residuals.append((c, res))

        if not residuals:
            print(f"  sym {s + 1}: all-zero, skipping")
            continue

        res_vals = [r for _, r in residuals]
        mean_arg = sum(abs(a) for a in args) / len(args)
        mean_res = sum(res_vals) / len(res_vals)
        var_res = sum((r - mean_res) ** 2 for r in res_vals) / len(res_vals)
        std_res = math.sqrt(var_res)
        cluster_spread = max(res_vals) - min(res_vals)

        # Linear-in-carrier slope test: regression of res vs c.
        n = len(residuals)
        mean_c = sum(c for c, _ in residuals) / n
        mean_r = mean_res
        num = sum((c - mean_c) * (r - mean_r) for c, r in residuals)
        den = sum((c - mean_c) ** 2 for c, _ in residuals)
        slope = num / den if den > 0 else 0.0

        per_symbol_residual_mean.append(mean_res)
        per_symbol_residual_std.append(std_res)
        per_symbol_slopes.append(slope)

        print(f"  sym {s + 1:<3}  {mean_arg:>14.4f}  "
              f"{mean_res:>+14.4f}  {std_res:>16.4f}  "
              f"{cluster_spread:>22.4f}")
        print(f"           linear-in-carrier slope (Pattern A): {slope:+.6e}")

    # Cross-symbol comparison.
    print("\n  Cross-symbol comparison:")
    if per_symbol_residual_mean:
        m_mean = sum(per_symbol_residual_mean) / len(per_symbol_residual_mean)
        spread_means = max(per_symbol_residual_mean) - min(per_symbol_residual_mean)
        print(f"    per-symbol mean residual: "
              f"{per_symbol_residual_mean}")
        print(f"    spread of per-symbol means: {spread_means:.4f}")
        print(f"    per-symbol slopes: {per_symbol_slopes}")

        # Pattern classification.
        max_slope = max(abs(s) for s in per_symbol_slopes)
        avg_std = sum(per_symbol_residual_std) / len(per_symbol_residual_std)
        print()
        print("  PATTERN HINT:")
        if avg_std < 0.05 and max_slope < 1e-5:
            uniform_mag = max(abs(m) for m in per_symbol_residual_mean)
            if uniform_mag < 0.05:
                print("    Pattern (none) — residuals close to 0 across all carriers/symbols.")
                print("    Chain looks structurally correct here.")
            else:
                print("    Pattern B — uniform per-symbol offset; suggests")
                print("    fractional-CFO residual or integer-CFO miscount.")
        elif max_slope > 1e-4:
            print("    Pattern A — linear-in-carrier slope of residual.")
            print("    Sub-sample timing offset between PRS and data symbols.")
            print(f"    Slope magnitude: {max_slope:.3e} rad/carrier")
        elif spread_means > 0.3:
            print("    Pattern C — per-symbol residual drifts across the frame.")
            print("    Fractional-CFO tracking loop / NCO carry issue.")
        else:
            print(f"    Pattern D — non-structural (avg_std={avg_std:.3f},")
            print(f"    max_slope={max_slope:.3e}, spread_means={spread_means:.3f}).")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--diff-spec", type=Path,
                    default=Path("/tmp/dab_rs_diff_spec.bin"))
    ap.add_argument("--frames", type=int, default=3,
                    help="number of frames to analyse")
    args = ap.parse_args()

    raw = args.diff_spec.read_bytes()
    n_frames = len(raw) // RECORD
    print(f"dump: {len(raw)} bytes, {n_frames} frames\n")
    for i in range(min(args.frames, n_frames)):
        buf = raw[i * RECORD:(i + 1) * RECORD]
        analyse_frame(buf)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
