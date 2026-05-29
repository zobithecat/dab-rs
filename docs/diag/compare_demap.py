#!/usr/bin/env python3
"""Compare dab-rs DAB_RS_DUMP_DEMAP_OUT against expected soft bits derived
from the LIVE k8b_v4.eti (Slice-14 Path A).

Both files have per-DAB-frame records keyed by 1-based frame_idx:

    expected_soft.bin (output of eti_to_expected_soft.py)
        u32 LE frame_idx
        u8  valid_mask (bit b = ficBlock b has all-3 FIBs CRC-valid in LIVE)
        u8  pad × 3
        i8  expected[9216] (sentinel 0 in invalid ficBlocks)

    dab_demap.bin (DAB_RS_DUMP_DEMAP_OUT, see crates/dab-cli/src/fic_iq.rs)
        u32 LE frame_idx
        i8  actual[9216]

For each (frame_idx, ficBlock) where the LIVE oracle's mask is set:

    - sign_agreement = sum(sign(expected_i) == sign(actual_i)) / 2304
    - mean_abs_actual / mean_abs_expected
    - position histogram of disagreements

A "healthy" dab-rs chain should produce ≥ 90% sign agreement vs the LIVE
ground truth on each valid ficBlock. Lower means a real divergence somewhere
in OFDM (since post-FFT chain is byte-equivalent per slice 12 v2).
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

SOFT_BITS_PER_FICBLOCK = 2304
FICBLOCKS_PER_FRAME = 4
SOFT_BITS_PER_FRAME = SOFT_BITS_PER_FICBLOCK * FICBLOCKS_PER_FRAME  # 9216


def read_expected(path: Path) -> dict[int, tuple[int, bytes]]:
    """Returns {frame_idx: (valid_mask, 9216 i8 bytes)}."""
    out: dict[int, tuple[int, bytes]] = {}
    record_size = 4 + 4 + SOFT_BITS_PER_FRAME  # 9224
    raw = path.read_bytes()
    n = len(raw) // record_size
    for i in range(n):
        record = raw[i * record_size : (i + 1) * record_size]
        (frame_idx,) = struct.unpack_from("<I", record, 0)
        valid_mask = record[4]
        soft = record[8 : 8 + SOFT_BITS_PER_FRAME]
        out[frame_idx] = (valid_mask, soft)
    return out


def read_dab_demap(path: Path) -> dict[int, bytes]:
    """Returns {frame_idx: 9216 i8 bytes}."""
    out: dict[int, bytes] = {}
    record_size = 4 + SOFT_BITS_PER_FRAME
    raw = path.read_bytes()
    n = len(raw) // record_size
    for i in range(n):
        record = raw[i * record_size : (i + 1) * record_size]
        (frame_idx,) = struct.unpack_from("<I", record, 0)
        soft = record[4 : 4 + SOFT_BITS_PER_FRAME]
        out[frame_idx] = soft
    return out


def signed_byte(b: int) -> int:
    return b - 256 if b >= 128 else b


def compare_block(expected: bytes, actual: bytes) -> dict:
    """Compare two 2304-byte ficBlock soft-bit slices."""
    assert len(expected) == SOFT_BITS_PER_FICBLOCK
    assert len(actual) == SOFT_BITS_PER_FICBLOCK

    sign_match = 0
    sign_mismatch = 0
    abs_expected = 0.0
    abs_actual = 0.0
    actual_zero = 0

    for i in range(SOFT_BITS_PER_FICBLOCK):
        e = signed_byte(expected[i])
        a = signed_byte(actual[i])
        abs_expected += abs(e)
        abs_actual += abs(a)
        # Expected is always ±127 (never 0) for valid bits.
        if a == 0:
            actual_zero += 1
            sign_mismatch += 1  # count zero as mismatch
            continue
        if (e > 0) == (a > 0):
            sign_match += 1
        else:
            sign_mismatch += 1

    return {
        "sign_match": sign_match,
        "sign_mismatch": sign_mismatch,
        "agreement": sign_match / SOFT_BITS_PER_FICBLOCK,
        "mean_abs_expected": abs_expected / SOFT_BITS_PER_FICBLOCK,
        "mean_abs_actual": abs_actual / SOFT_BITS_PER_FICBLOCK,
        "actual_zero": actual_zero,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("expected", type=Path, help="output of eti_to_expected_soft.py")
    ap.add_argument("dab_demap", type=Path, help="DAB_RS_DUMP_DEMAP_OUT")
    ap.add_argument("--max-frames", type=int, default=20,
                    help="number of DAB frames to show per-block detail for")
    args = ap.parse_args()

    expected = read_expected(args.expected)
    actual = read_dab_demap(args.dab_demap)

    common = sorted(set(expected.keys()) & set(actual.keys()))
    print(f"Expected: {len(expected)} frames, dab-rs: {len(actual)} frames, "
          f"common: {len(common)}")

    agg_match = 0
    agg_mismatch = 0
    n_blocks = 0
    high_agreement_blocks = 0  # ≥ 90%
    perfect_blocks = 0  # 100%

    detail_shown = 0
    for frame_idx in common:
        valid_mask, exp_soft = expected[frame_idx]
        act_soft = actual[frame_idx]

        for fb in range(FICBLOCKS_PER_FRAME):
            if not (valid_mask & (1 << fb)):
                continue
            off = fb * SOFT_BITS_PER_FICBLOCK
            e = exp_soft[off : off + SOFT_BITS_PER_FICBLOCK]
            a = act_soft[off : off + SOFT_BITS_PER_FICBLOCK]
            r = compare_block(e, a)
            agg_match += r["sign_match"]
            agg_mismatch += r["sign_mismatch"]
            n_blocks += 1
            if r["agreement"] >= 0.90:
                high_agreement_blocks += 1
            if r["agreement"] >= 0.999:
                perfect_blocks += 1

            if detail_shown < args.max_frames * FICBLOCKS_PER_FRAME:
                print(f"  frame {frame_idx} fb {fb}: "
                      f"agree={r['agreement']*100:5.1f}% "
                      f"mean|exp|={r['mean_abs_expected']:.1f} "
                      f"mean|act|={r['mean_abs_actual']:.1f} "
                      f"zeros={r['actual_zero']}")
                detail_shown += 1

    total = agg_match + agg_mismatch
    print()
    print(f"=== Aggregate over {n_blocks} ficBlocks ({total} bits) ===")
    if total > 0:
        print(f"  sign agreement: {agg_match}/{total} = {100*agg_match/total:.2f}%")
    print(f"  high agreement (≥90%) ficBlocks: {high_agreement_blocks}/{n_blocks}")
    print(f"  perfect (≥99.9%) ficBlocks:      {perfect_blocks}/{n_blocks}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
