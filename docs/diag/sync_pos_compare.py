#!/usr/bin/env python3
"""Slice-19 absolute-sample sync alignment.

Reads two SYNC_POS dumps in the same coordinate system (resampled 2.048 MSPS
sample index) and finds the dab-rs frame offset that best matches eti-stuff,
then reports the per-symbol δ = dab_useful_start − live_useful_start.

Record (both):
    u32 LE frame_idx
    u32 LE ofdmSymbolCount (1=PRS, 2..76=data)
    u64 LE abs_useful_start
"""
from __future__ import annotations
import argparse, struct, sys
from pathlib import Path

REC_SZ = 4 + 4 + 8

def read(path):
    raw = Path(path).read_bytes()
    n = len(raw) // REC_SZ
    rs = []
    for i in range(n):
        rs.append(struct.unpack_from('<IIQ', raw, i*REC_SZ))
    return rs

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--live', default='/tmp/sim5_syncpos_live.bin')
    ap.add_argument('--dab',  default='/tmp/sim5_syncpos_dab.bin')
    args = ap.parse_args()

    live = read(args.live)
    dab  = read(args.dab)
    print(f'live: {len(live)} records (frames {live[0][0]}..{live[-1][0]})')
    print(f'dab : {len(dab)} records  (frames {dab[0][0]}..{dab[-1][0]})')

    # Build a lookup: (live_frame, live_sym) → live_pos
    live_lookup = {(f, s): p for (f, s, p) in live}

    # For each dab record, find live record with matching sym whose pos is
    # closest. Specifically for sym=1 (PRS) entries, compute the implied
    # frame-offset that aligns them.
    print('\n=== dab-rs PRS useful_start vs nearest live PRS useful_start ===')
    print(f'{"dab_f":>5} {"dab_pos":>10}  {"best_live_f":>11} {"live_pos":>10}  {"δ_samples":>10}')
    dab_prs = [(f, p) for (f, s, p) in dab if s == 1]
    live_prs = [(f, p) for (f, s, p) in live if s == 1]
    matches = []  # (dab_f, live_f, δ)
    for dab_f, dab_p in dab_prs[:25]:
        # Find live PRS with closest pos.
        best = None
        for live_f, live_p in live_prs:
            d = dab_p - live_p
            if best is None or abs(d) < abs(best[2]):
                best = (live_f, live_p, d)
        if best is not None:
            matches.append((dab_f, best[0], best[2]))
            print(f'{dab_f:>5} {dab_p:>10}  {best[0]:>11} {best[1]:>10}  {best[2]:>+10}')

    # Show per-symbol δ within matched frames.
    print('\n=== per-symbol δ within frames matched by PRS ===')
    print(f'{"dab_f":>5} {"live_f":>6}  {"sym":>3}  {"δ_samples":>10}')
    for dab_f, live_f, _ in matches[:8]:
        # Get all dab syms for this dab_f
        dab_syms = [(s, p) for (f, s, p) in dab if f == dab_f]
        for s, dab_p in dab_syms:
            live_p = live_lookup.get((live_f, s))
            if live_p is None:
                continue
            print(f'{dab_f:>5} {live_f:>6}  {s:>3}  {dab_p - live_p:>+10}')

    # Cross-frame δ stability check
    print('\n=== PRS δ stability over the run ===')
    deltas = [d for (_, _, d) in matches]
    if deltas:
        print(f'   min={min(deltas)}  max={max(deltas)}  mean={sum(deltas)/len(deltas):.2f}')
        # how many at each integer offset
        from collections import Counter
        c = Counter(deltas)
        for k in sorted(c):
            print(f'   δ={k:+}: {c[k]} frames')

if __name__ == '__main__':
    sys.exit(main())
