#!/usr/bin/env python3
"""Scan dab-rs frame_idx <-> expected_soft frame_idx offset to find best
sign-agreement. If chain is functionally correct, some offset will reveal
>> 50% agreement on a window of matched frames."""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

SOFT_BITS_PER_FICBLOCK = 2304
FICBLOCKS_PER_FRAME = 4
SOFT_BITS_PER_FRAME = SOFT_BITS_PER_FICBLOCK * FICBLOCKS_PER_FRAME


def read_expected(path: Path):
    out = {}
    rec_sz = 4 + 4 + SOFT_BITS_PER_FRAME
    raw = path.read_bytes()
    for i in range(len(raw) // rec_sz):
        r = raw[i * rec_sz : (i + 1) * rec_sz]
        (fi,) = struct.unpack_from("<I", r, 0)
        out[fi] = (r[4], r[8:8 + SOFT_BITS_PER_FRAME])
    return out


def read_dab(path: Path):
    out = {}
    rec_sz = 4 + SOFT_BITS_PER_FRAME
    raw = path.read_bytes()
    for i in range(len(raw) // rec_sz):
        r = raw[i * rec_sz : (i + 1) * rec_sz]
        (fi,) = struct.unpack_from("<I", r, 0)
        out[fi] = r[4:4 + SOFT_BITS_PER_FRAME]
    return out


def agreement(expected: bytes, actual: bytes, valid_mask: int) -> tuple[int, int]:
    match = 0
    total = 0
    for fb in range(FICBLOCKS_PER_FRAME):
        if not (valid_mask & (1 << fb)):
            continue
        off = fb * SOFT_BITS_PER_FICBLOCK
        for i in range(SOFT_BITS_PER_FICBLOCK):
            e = expected[off + i]
            a = actual[off + i]
            e_signed = e - 256 if e >= 128 else e
            a_signed = a - 256 if a >= 128 else a
            if a_signed == 0:
                continue
            if (e_signed > 0) == (a_signed > 0):
                match += 1
            total += 1
    return match, total


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("expected", type=Path)
    ap.add_argument("dab", type=Path)
    ap.add_argument("--offset-range", type=int, default=20,
                    help="search ±N offset between dab and expected indices")
    args = ap.parse_args()

    expected = read_expected(args.expected)
    actual = read_dab(args.dab)

    # offset means: expected[dab_idx + offset] is compared with actual[dab_idx].
    print(f"{'offset':>7}  {'n_blocks':>8}  {'agree %':>8}")
    best = (-1, 0.0, 0)
    for off in range(-args.offset_range, args.offset_range + 1):
        agg_match = 0
        agg_total = 0
        for dab_idx, act in actual.items():
            exp_key = dab_idx + off
            if exp_key not in expected:
                continue
            mask, exp = expected[exp_key]
            m, t = agreement(exp, act, mask)
            agg_match += m
            agg_total += t
        if agg_total == 0:
            continue
        pct = 100.0 * agg_match / agg_total
        print(f"  {off:+5d}  {agg_total // 2304:>8}  {pct:>7.2f}")
        if pct > best[1]:
            best = (off, pct, agg_total)
    print(f"\nbest offset = {best[0]} → {best[1]:.2f}% agreement "
          f"(over {best[2]} bits)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
