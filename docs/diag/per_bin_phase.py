#!/usr/bin/env python3
"""Slice-20 Part B: per-carrier phase difference at sample-aligned record.

Pick one (live_f, live_sym) and (dab_f, dab_sym) pair at the same
useful_start. Compute phase_diff[k] = arg(dab_fft[k]) − arg(live_fft[k])
over active carriers, then classify:

- linear in k  → fractional CFO mismatch (slope → Δf Hz)
- constant     → trivial phase offset (high |cc| expected — contradicts)
- spectral shift dab[k]≈live[k−m] → integer CFO mismatch
- random       → input difference (already ruled out by Part A) or
                 FFT-internal mismatch
"""
from __future__ import annotations
import math, struct, sys
from pathlib import Path

T_U = 2048
REC_FFT  = 4 + 4 + T_U * 8
REC_SYNC = 4 + 4 + 8

def read_sync(p):
    raw = Path(p).read_bytes()
    out = {}
    for i in range(len(raw)//REC_SYNC):
        f, s, pos = struct.unpack_from('<IIQ', raw, i*REC_SYNC)
        out[(f, s)] = pos
    return out

def read_fft(p):
    data = Path(p).read_bytes()
    n = len(data) // REC_FFT
    for i in range(n):
        off = i * REC_FFT
        f, s = struct.unpack_from('<II', data, off)
        re_im = struct.unpack_from(f'<{2*T_U}f', data, off+8)
        spec = [complex(re_im[2*j], re_im[2*j+1]) for j in range(T_U)]
        yield (f, s, spec)

def active(): return list(range(1, 769)) + list(range(1280, T_U))

def signed_carrier(k):
    return k if k <= T_U//2 else k - T_U

def main():
    live_sync = read_sync('/tmp/sim5_syncpos_live.bin')
    dab_sync  = read_sync('/tmp/sim5_syncpos_dab.bin')
    live_by_pos = {pos: (f, s) for (f, s), pos in live_sync.items()}
    dab_by_pos  = {pos: (f, s) for (f, s), pos in dab_sync.items()}
    matched = sorted(set(live_by_pos) & set(dab_by_pos))
    print(f'exact-position matches: {len(matched)}')

    live_fft = {}
    for f, s, spec in read_fft('/tmp/sim5_fft_live.bin'):
        live_fft[(f, s)] = spec
    dab_fft = {}
    for f, s, spec in read_fft('/tmp/sim5_fft_dab.bin'):
        dab_fft[(f, s)] = spec

    bins = active()

    print('\n=== Per-bin phase scan: take 3 matched records, look at first 24 active carriers ===')
    shown = 0
    for pos in matched[:30]:
        live_fs = live_by_pos[pos]
        dab_fs  = dab_by_pos[pos]
        lspec = live_fft.get(live_fs)
        dspec = dab_fft.get(dab_fs)
        if lspec is None or dspec is None: continue
        # Magnitude on a few bins
        sample_bins = bins[:24] + bins[-4:]
        # Compute global stats
        diffs = []
        mag_ratios = []
        for k in bins:
            if abs(lspec[k]) < 1e-9 or abs(dspec[k]) < 1e-9: continue
            r = dspec[k] / lspec[k]
            diffs.append((signed_carrier(k), math.atan2(r.imag, r.real), abs(r)))
        if not diffs: continue
        # Mean magnitude ratio, slope of phase vs k (using pair-wise wrapped)
        diffs.sort()
        ks = [d[0] for d in diffs]
        phs = [d[1] for d in diffs]
        mrs = [d[2] for d in diffs]
        # Slope by adjacent-pair median (wrap-safe)
        slopes = []
        for i in range(1, len(diffs)):
            dk = ks[i] - ks[i-1]
            if dk == 0: continue
            dph = phs[i] - phs[i-1]
            dph = (dph + math.pi) % (2*math.pi) - math.pi
            slopes.append(dph/dk)
        slopes.sort()
        slope_med = slopes[len(slopes)//2] if slopes else 0
        # Δf Hz from slope: slope rad/carrier ⇒ time domain = slope·T_u/(2π) samples ⇒ Hz = slope·fs/(2π)
        delta_f = slope_med * 2048000 / (2*math.pi)
        mean_mag = sum(mrs)/len(mrs)
        print(f'rec at pos {pos} live{live_fs} dab{dab_fs}: '
              f'mag_ratio_mean={mean_mag:.3f} '
              f'phase_slope={slope_med:+.4e} rad/k  Δf≈{delta_f:+.2f}Hz')
        # Cross-correlation for spectral-shift detection: shift dab spec by m bins,
        # see if any m gives much higher cc
        if shown < 2:
            shown += 1
            print(f'   spectral-shift sweep (cc magnitude vs bin-shift m):')
            for m in [-5, -3, -2, -1, 0, +1, +2, +3, +5]:
                num = 0j; pa = 0.0; pb = 0.0
                for k in bins:
                    kshift = (k + m) % T_U
                    a = dspec[kshift]
                    b = lspec[k]
                    num += a * b.conjugate()
                    pa += abs(a)**2
                    pb += abs(b)**2
                cc = abs(num) / math.sqrt(pa*pb) if pa*pb > 0 else 0
                marker = '  ★' if abs(cc) > 0.5 else ''
                print(f'      m={m:+}: |cc|={cc:.4f}{marker}')

if __name__ == '__main__':
    sys.exit(main())
