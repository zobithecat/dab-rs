#!/usr/bin/env python3
"""Slice-19 absolute-position FFT compare.

Join eti-stuff and dab-rs FFT dumps by their SYNC_POS useful_start
(absolute sample index in the resampled stream), then compare bins.

If both have a record at the same useful_start, their FFT output should be
nearly identical (modulo any NCO mix difference). If they're random
(|cc|≈0), the bug is NOT sync (since input samples are identical) — it's
NCO/FFT/spectral processing.
"""
from __future__ import annotations
import math, struct, sys
from pathlib import Path

REC_FFT  = 4 + 4 + 2048*8
REC_SYNC = 4 + 4 + 8

def read_sync(p):
    raw = Path(p).read_bytes()
    out = {}
    for i in range(len(raw)//REC_SYNC):
        f, s, pos = struct.unpack_from('<IIQ', raw, i*REC_SYNC)
        out[(f, s)] = pos
    return out

def read_fft(p):
    """Yield (frame, sym, spec list[complex])."""
    data = Path(p).read_bytes()
    n = len(data) // REC_FFT
    for i in range(n):
        off = i * REC_FFT
        f, s = struct.unpack_from('<II', data, off)
        re_im = struct.unpack_from(f'<{2*2048}f', data, off+8)
        spec = [complex(re_im[2*j], re_im[2*j+1]) for j in range(2048)]
        yield (f, s, spec)

def active():
    return list(range(1, 769)) + list(range(1280, 2048))

def ccorr(a, b, bins):
    num = 0j; pa = 0.0; pb = 0.0
    for k in bins:
        num += a[k] * b[k].conjugate()
        pa  += abs(a[k])**2
        pb  += abs(b[k])**2
    return num / math.sqrt(pa*pb) if pa*pb > 0 else 0j

def mcorr(a, b, bins):
    am = [abs(a[k]) for k in bins]
    bm = [abs(b[k]) for k in bins]
    ma = sum(am)/len(am); mb = sum(bm)/len(bm)
    cov = sum((am[i]-ma)*(bm[i]-mb) for i in range(len(am)))
    va  = sum((am[i]-ma)**2 for i in range(len(am)))
    vb  = sum((bm[i]-mb)**2 for i in range(len(bm)))
    return cov/math.sqrt(va*vb) if va*vb>0 else 0.0

def main():
    print('Loading sync_pos lookups...')
    live_sync = read_sync('/tmp/sim5_syncpos_live.bin')
    dab_sync  = read_sync('/tmp/sim5_syncpos_dab.bin')
    # invert: useful_start → (frame, sym)
    live_by_pos = {pos: (f, s) for (f, s), pos in live_sync.items()}
    dab_by_pos  = {pos: (f, s) for (f, s), pos in dab_sync.items()}
    # Find positions that exist in both
    matched_pos = set(live_by_pos.keys()) & set(dab_by_pos.keys())
    print(f'  exact-position matches: {len(matched_pos)} symbol-records')

    # Also match within tolerance (±2 samples) for near-matches
    tol_matches = []
    for pos in sorted(dab_by_pos.keys())[:200]:
        for dp in range(-2, 3):
            if (pos + dp) in live_by_pos:
                tol_matches.append((pos, dp))
                break
    print(f'  within ±2 samples: {len(tol_matches)} of first 200 dab records')

    if not matched_pos:
        print('NO EXACT MATCHES — checking why...')
        print(f'live PRS positions (first 5): {sorted([p for (f,s),p in live_sync.items() if s==1])[:5]}')
        print(f'dab  PRS positions (first 5): {sorted([p for (f,s),p in dab_sync.items() if s==1])[:5]}')

    # Load FFT dumps and index by (frame, sym)
    print('Loading FFT dumps...')
    live_fft = {}
    for f, s, spec in read_fft('/tmp/sim5_fft_live.bin'):
        live_fft[(f, s)] = spec
    print(f'  live FFT: {len(live_fft)} records')
    dab_fft = {}
    for f, s, spec in read_fft('/tmp/sim5_fft_dab.bin'):
        dab_fft[(f, s)] = spec
    print(f'  dab  FFT: {len(dab_fft)} records')

    # For each exact-position match, compare the FFT bins
    bins = active()
    print('\n=== Exact-position FFT bin compare ===')
    print(f'{"pos":>10}  {"live(f,s)":>11}  {"dab(f,s)":>11}  {"mag_corr":>8}  {"|cc|":>6}  {"arg(cc)":>8}')
    pairs = sorted(matched_pos)
    avg_mag = 0.0; avg_cc = 0.0; n = 0
    for pos in pairs[:30]:
        live_fs = live_by_pos[pos]
        dab_fs  = dab_by_pos[pos]
        lspec = live_fft.get(live_fs)
        dspec = dab_fft.get(dab_fs)
        if lspec is None or dspec is None:
            continue
        m = mcorr(dspec, lspec, bins)
        c = ccorr(dspec, lspec, bins)
        avg_mag += m; avg_cc += abs(c); n += 1
        print(f'  {pos:>8}  {str(live_fs):>11}  {str(dab_fs):>11}  '
              f'{m:>8.4f}  {abs(c):>6.4f}  {math.atan2(c.imag, c.real):>+8.4f}')
    if n:
        print(f'\n  aggregate over {n}: mag_corr={avg_mag/n:.4f}, |cc|={avg_cc/n:.4f}')

if __name__ == '__main__':
    sys.exit(main())
