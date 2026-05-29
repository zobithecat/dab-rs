#!/usr/bin/env python3
"""Slice-18 clean FFT bin comparison.

Both files are produced from the SAME 2.048 MSPS cf32 stream:
    /tmp/sim4_fft_live.bin   eti-stuff DAB_RS_DIAG_DUMP_FFT
                             (sync+CFO applied AFTER, all 75 data symbols/frame)
    /tmp/sim4_fft_dab.bin    dab-rs DAB_RS_DUMP_FFT_PRE
                             (sync+CFO applied AFTER, FIC symbols 2,3,4/frame)

Record (both):
    u32 LE frame_idx
    u32 LE ofdmSymbolCount (2..76; FIC = 2,3,4)
    2048 × complex<f32> (re,im LE pair) natural-order FFT bins

Step 1: frame-offset sweep — for each candidate δ ∈ [-8, +8], join records on
        (live.frame + δ, live.sym) ↔ (dab.frame, dab.sym) and compute global
        complex correlation. With identical inputs, exactly one δ should
        produce a clear spike.

Step 2: at the best δ, per-symbol metrics — magnitude_corr, complex_corr.
        magnitude_corr ≈ 1 means same energy lands in same bins (sync within
        ±1 sample). complex_corr ≈ 1 means same phase too.

Step 3: if complex_corr is low while magnitude_corr is high, fit per-bin
        phase difference to a linear ramp arg(dab/live) = 2π·k·δ_samp/T_u +
        const. Slope → sub-sample timing offset.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

TU = 2048
REC_SZ = 4 + 4 + TU * 8


def read_records(path: Path):
    """Yield (frame, sym, [complex]) tuples."""
    data = path.read_bytes()
    n = len(data) // REC_SZ
    for i in range(n):
        off = i * REC_SZ
        frame, sym = struct.unpack_from("<II", data, off)
        re_im = struct.unpack_from(f"<{2*TU}f", data, off + 8)
        spec = [complex(re_im[2*j], re_im[2*j+1]) for j in range(TU)]
        yield (frame, sym, spec)


def active_bins():
    """Mode I active carriers: ±1 .. ±768 → bins [1..768] ∪ [1280..2047]."""
    return list(range(1, 769)) + list(range(1280, 2048))


def complex_correlation(a, b, bins):
    """Normalized complex correlation Σ a·conj(b) / sqrt(Σ|a|²·Σ|b|²)."""
    num = complex(0, 0)
    pa = 0.0
    pb = 0.0
    for k in bins:
        num += a[k] * b[k].conjugate()
        pa += abs(a[k]) ** 2
        pb += abs(b[k]) ** 2
    denom = math.sqrt(pa * pb)
    return num / denom if denom > 0 else complex(0, 0)


def magnitude_correlation(a, b, bins):
    """Correlation of |a[k]| and |b[k]| over `bins`."""
    am = [abs(a[k]) for k in bins]
    bm = [abs(b[k]) for k in bins]
    ma = sum(am) / len(am)
    mb = sum(bm) / len(bm)
    cov = sum((am[i] - ma) * (bm[i] - mb) for i in range(len(am)))
    va = sum((am[i] - ma) ** 2 for i in range(len(am)))
    vb = sum((bm[i] - mb) ** 2 for i in range(len(bm)))
    return cov / math.sqrt(va * vb) if va * vb > 0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", type=Path, default=Path("/tmp/sim4_fft_live.bin"))
    ap.add_argument("--dab",  type=Path, default=Path("/tmp/sim4_fft_dab.bin"))
    ap.add_argument("--offset-range", type=int, default=8,
                    help="frame-offset sweep range (±N)")
    ap.add_argument("--limit-frames", type=int, default=40,
                    help="only use the first N dab frames (speed)")
    args = ap.parse_args()

    bins = active_bins()

    # Index live records by (frame, sym).
    print(f"Reading live FFT dump {args.live} ...")
    live_index = {}
    for frame, sym, spec in read_records(args.live):
        live_index[(frame, sym)] = spec
    print(f"  loaded {len(live_index)} live records")

    print(f"Reading dab FFT dump {args.dab} ...")
    dab_records = list(read_records(args.dab))
    if args.limit_frames > 0:
        max_frame = args.limit_frames
        dab_records = [r for r in dab_records if r[0] <= max_frame]
    print(f"  loaded {len(dab_records)} dab records (frames 1..{args.limit_frames})")

    # ---- Step 1: (frame, sym) joint offset sweep ----
    # If dab-rs's sync picks a different OFDM symbol than eti-stuff, we need
    # to shift `sym` too: dab.sym ↔ live.(sym + sym_off).
    print()
    print("Step 1: (frame, sym) joint offset sweep — mag_corr is key, cc-phase shows random vs structured")
    print(f"{'fr_off':>7} {'sym_off':>7}  {'n':>5}  {'mean mag_corr':>13}  {'mean|cc|':>10}")
    best = (0, 0, -1.0, -1.0)
    for fr_off in range(-args.offset_range, args.offset_range + 1):
        for sym_off in range(-3, 4):
            mag_list = []
            cc_list = []
            for frame, sym, dab_spec in dab_records:
                live_spec = live_index.get((frame + fr_off, sym + sym_off))
                if live_spec is None:
                    continue
                mc = magnitude_correlation(dab_spec, live_spec, bins)
                cc = complex_correlation(dab_spec, live_spec, bins)
                mag_list.append(mc)
                cc_list.append(cc)
            if len(mag_list) < 10:
                continue
            mean_mag = sum(mag_list) / len(mag_list)
            mean_cc = sum(abs(c) for c in cc_list) / len(cc_list)
            # Only print interesting ones (mag_corr > 0.5) and the diagonal.
            if mean_mag > 0.5 or (sym_off == 0 and abs(fr_off) <= 2):
                print(f"  {fr_off:+5d} {sym_off:+5d}  {len(mag_list):>5}  "
                      f"{mean_mag:>13.4f}  {mean_cc:>10.4f}")
            if mean_mag > best[2] or (mean_mag > 0.6 and mean_cc > best[3]):
                best = (fr_off, sym_off, mean_mag, mean_cc)

    print(f"\nBest (fr_off={best[0]}, sym_off={best[1]}) → "
          f"mag_corr={best[2]:.4f}, mean|cc|={best[3]:.4f}")
    best = (best[0], best[2], best[3], best[1])  # reorder for downstream use

    # ---- Step 2: per-symbol detail at best offset ----
    fr_off_b, sym_off_b = best[0], best[3]
    print()
    print(f"Step 2: per-symbol metrics at (fr_off={fr_off_b}, sym_off={sym_off_b})")
    print(f"{'frame':>6} {'sym':>4}  {'mag_corr':>9}  {'|cc|':>7}  {'arg(cc)':>9}")
    shown = 0
    avg_mag = 0.0
    avg_cc = 0.0
    n_used = 0
    for frame, sym, dab_spec in dab_records:
        live_spec = live_index.get((frame + fr_off_b, sym + sym_off_b))
        if live_spec is None:
            continue
        mc = magnitude_correlation(dab_spec, live_spec, bins)
        cc = complex_correlation(dab_spec, live_spec, bins)
        avg_mag += mc
        avg_cc += abs(cc)
        n_used += 1
        if shown < 30:
            print(f"  {frame:>4} {sym:>4}  {mc:>9.4f}  {abs(cc):>7.4f}  "
                  f"{math.atan2(cc.imag, cc.real):>+9.4f}")
            shown += 1
    if n_used:
        print(f"\n  aggregate over {n_used} symbols: mag_corr={avg_mag/n_used:.4f}, "
              f"|cc|={avg_cc/n_used:.4f}")

    # ---- Step 3: per-symbol phase-ramp fit → sub-sample timing offset ----
    # Use a robust circular slope estimator that's immune to 2π wraps:
    # for each adjacent carrier pair, dφ_k = arg(dab[k+1]/live[k+1]) -
    # arg(dab[k]/live[k]) wrapped to (-π, π]; mean dφ is the slope rad/carrier.
    # δ_samp = mean_dφ · T_u / (2π).
    print()
    print(f"Step 3: per-symbol phase-ramp slope → sub-sample δ (first 12 frames)")
    print(f"{'frame':>6} {'sym':>4}  {'δ_samples':>10}  {'|cc|':>7}")

    def per_symbol_delta(dab_spec, live_spec):
        # Compute per-bin (dab/live) phase only on strong bins to reduce noise.
        ratios = []
        carriers = []
        for k in bins:
            l = live_spec[k]
            d = dab_spec[k]
            if abs(l) < 1e-6 or abs(d) < 1e-6:
                continue
            r = d / l
            ph = math.atan2(r.imag, r.real)
            kc = k if k <= TU // 2 else k - TU
            ratios.append(ph)
            carriers.append(kc)
        if len(ratios) < 64:
            return float("nan")
        # Sort by carrier index so adjacent ratios are adjacent carriers.
        order = sorted(range(len(carriers)), key=lambda i: carriers[i])
        # Median of pair-wise wrapped differences (robust to outliers).
        diffs = []
        for i in range(1, len(order)):
            j_prev = order[i - 1]
            j_curr = order[i]
            dk = carriers[j_curr] - carriers[j_prev]
            if dk == 0:
                continue
            d_ph = ratios[j_curr] - ratios[j_prev]
            # Wrap to (-π, π]
            d_ph = (d_ph + math.pi) % (2 * math.pi) - math.pi
            slope = d_ph / dk
            diffs.append(slope)
        if not diffs:
            return float("nan")
        diffs.sort()
        median_slope = diffs[len(diffs) // 2]
        return median_slope * TU / (2 * math.pi)

    deltas_by_frame: dict = {}
    shown = 0
    for frame, sym, dab_spec in dab_records:
        live_spec = live_index.get((frame + fr_off_b, sym + sym_off_b))
        if live_spec is None:
            continue
        ds = per_symbol_delta(dab_spec, live_spec)
        cc = complex_correlation(dab_spec, live_spec, bins)
        deltas_by_frame.setdefault(frame, []).append((sym, ds))
        if shown < 36:
            print(f"  {frame:>4} {sym:>4}  {ds:>+10.3f}  {abs(cc):>7.4f}")
            shown += 1

    # Cross-symbol consistency within frames: if all sym 2/3/4 of a frame have
    # similar δ_samples, the offset is frame-level (constant prs_start error).
    # If they vary widely, the offset is per-symbol (cp_start drift).
    print(f"\nCross-symbol δ spread within each frame:")
    print(f"{'frame':>6}  {'sym2':>8} {'sym3':>8} {'sym4':>8}  {'spread':>8}")
    for frame in sorted(deltas_by_frame.keys())[:20]:
        rows = deltas_by_frame[frame]
        if len(rows) < 3:
            continue
        rows.sort()
        d2, d3, d4 = rows[0][1], rows[1][1], rows[2][1]
        spread = max(d2, d3, d4) - min(d2, d3, d4)
        print(f"  {frame:>4}  {d2:>+8.3f} {d3:>+8.3f} {d4:>+8.3f}  {spread:>8.3f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
