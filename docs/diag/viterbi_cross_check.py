#!/usr/bin/env python3
"""Slice-7 cross-check: dab-rs vs eti-stuff viterbiSpiral, input and output.

Reads the four dump files produced by the slice-7 instrumentation
(patched `ficHandler::process_ficInput` on the oracle side, env-gated
dumps in `dab fic-iq` on the dab-rs side) and reports whether the two
implementations agree at the Viterbi *input* (depunctured mother
codeword, 3096 soft bits) and at the Viterbi *output* (768 hard bits
per ficBlock, before energy descrambling).

Three judgement classes follow naturally from the two diff rates:

  Result A — Inputs match (≥ 0.99 bit identity), outputs differ.
             Bug confirmed in dab-viterbi's scalar decoder; port
             viterbiSpiral (slice-7 plan fork 1).
  Result B — Inputs differ at any non-trivial offset.
             Bug is upstream of dab-viterbi — likely OFDM bit-ordering
             inside the demap loop or the FIC depuncture geometry.
  Result C — Both match.
             Bug downstream — descrambler or FIB packing.

File formats
============

Oracle (per ficBlock, 4 records per OFDM frame):

  viterbi_in_oracle.bin
      u32 LE frame_idx, u32 LE ficno, i16 LE × 3096

  viterbi_out_oracle.bin
      u32 LE frame_idx, u32 LE ficno, u8 × 768   (bit-per-byte hard bits)

dab-rs (per OFDM frame, 1 record):

  viterbi_in_dab_rs.bin
      u32 LE frame_idx, i16 LE × (4 × 3096) = i16 × 12384

  viterbi_out_dab_rs.bin
      u32 LE frame_idx, u8 × (4 × 768) = u8 × 3072

Aggregation: the script concatenates oracle's 4 ficBlocks per frame to
produce a single per-frame record matching the dab-rs layout, then
sweeps a small frame_idx offset to absorb sync-acquisition skew.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

FIC_VITERBI_LEN = 3072 + 24       # 3096 soft bits per ficBlock
FIC_OUT_BITS = 768                # 768 hard bits per ficBlock
FIC_BLOCKS_PER_FRAME = 4
FRAME_SOFT_LEN = FIC_BLOCKS_PER_FRAME * FIC_VITERBI_LEN  # 12384
FRAME_HARD_LEN = FIC_BLOCKS_PER_FRAME * FIC_OUT_BITS     # 3072


def read_oracle_in(path: Path) -> dict[int, list[int]]:
    """Returns ``{frame_idx → [i16 × 12384]}`` by aggregating each
    frame's 4 ficBlock records."""
    rec_bytes = 4 + 4 + FIC_VITERBI_LEN * 2  # u32 frame, u32 ficno, i16 × 3096
    raw = path.read_bytes()
    n = len(raw) // rec_bytes
    by_frame: dict[int, list[tuple[int, list[int]]]] = {}
    for i in range(n):
        off = i * rec_bytes
        frame_idx, ficno = struct.unpack_from("<II", raw, off)
        soft = list(struct.unpack_from(f"<{FIC_VITERBI_LEN}h", raw, off + 8))
        by_frame.setdefault(frame_idx, []).append((ficno, soft))
    aggregated: dict[int, list[int]] = {}
    for f, recs in by_frame.items():
        if len(recs) != FIC_BLOCKS_PER_FRAME:
            continue  # skip incomplete frames
        recs.sort()  # by ficno
        out: list[int] = []
        for _ficno, soft in recs:
            out.extend(soft)
        aggregated[f] = out
    return aggregated


def read_oracle_out(path: Path) -> dict[int, list[int]]:
    """Returns ``{frame_idx → [u8 × 3072]}``."""
    rec_bytes = 4 + 4 + FIC_OUT_BITS  # u32 frame, u32 ficno, u8 × 768
    raw = path.read_bytes()
    n = len(raw) // rec_bytes
    by_frame: dict[int, list[tuple[int, list[int]]]] = {}
    for i in range(n):
        off = i * rec_bytes
        frame_idx, ficno = struct.unpack_from("<II", raw, off)
        bits = list(raw[off + 8 : off + 8 + FIC_OUT_BITS])
        by_frame.setdefault(frame_idx, []).append((ficno, bits))
    aggregated: dict[int, list[int]] = {}
    for f, recs in by_frame.items():
        if len(recs) != FIC_BLOCKS_PER_FRAME:
            continue
        recs.sort()
        out: list[int] = []
        for _ficno, bits in recs:
            out.extend(bits)
        aggregated[f] = out
    return aggregated


def read_dab_rs_in(path: Path) -> dict[int, list[int]]:
    rec_bytes = 4 + FRAME_SOFT_LEN * 2
    raw = path.read_bytes()
    n = len(raw) // rec_bytes
    out: dict[int, list[int]] = {}
    for i in range(n):
        off = i * rec_bytes
        (frame_idx,) = struct.unpack_from("<I", raw, off)
        soft = list(struct.unpack_from(f"<{FRAME_SOFT_LEN}h", raw, off + 4))
        out[frame_idx] = soft
    return out


def read_dab_rs_out(path: Path) -> dict[int, list[int]]:
    rec_bytes = 4 + FRAME_HARD_LEN
    raw = path.read_bytes()
    n = len(raw) // rec_bytes
    out: dict[int, list[int]] = {}
    for i in range(n):
        off = i * rec_bytes
        (frame_idx,) = struct.unpack_from("<I", raw, off)
        bits = list(raw[off + 4 : off + rec_bytes])
        out[frame_idx] = bits
    return out


def sweep_alignment(
    a: dict[int, list[int]],
    b: dict[int, list[int]],
    metric_eq,
    offset_range: int = 30,
) -> tuple[int, int, int]:
    """Pick the integer offset that maximises agreement between `a` and
    `b` when interpreting `a[frame] ↔ b[frame + offset]`. `metric_eq` is
    a function(a_entry, b_entry) -> (matched, total)."""
    best = (0, 0, 0)  # (offset, matched, total)
    for offset in range(-offset_range, offset_range + 1):
        matched = 0
        total = 0
        for f, va in a.items():
            vb = b.get(f + offset)
            if vb is None:
                continue
            m, t = metric_eq(va, vb)
            matched += m
            total += t
        if matched > best[1]:
            best = (offset, matched, total)
    return best


def i16_match(va, vb):
    if len(va) != len(vb):
        return (0, 0)
    return (sum(1 for x, y in zip(va, vb) if x == y), len(va))


def bit_match(va, vb):
    if len(va) != len(vb):
        return (0, 0)
    return (sum(1 for x, y in zip(va, vb) if x == y), len(va))


def per_position_diff(
    a: dict[int, list[int]],
    b: dict[int, list[int]],
    offset: int,
    payload_len: int,
) -> list[int]:
    counts = [0] * payload_len
    for f, va in a.items():
        vb = b.get(f + offset)
        if vb is None or len(vb) != payload_len:
            continue
        for i, (x, y) in enumerate(zip(va, vb)):
            if x != y:
                counts[i] += 1
    return counts


def show_bucket_histogram(counts: list[int], n_frames: int, label: str,
                          bucket: int = 256) -> None:
    print(f"  per-position diff rate over {n_frames} aligned frames "
          f"(bucket = {bucket} positions):")
    print("    position bucket        diff-rate")
    n = len(counts)
    for start in range(0, n, bucket):
        end = min(start + bucket, n)
        d = sum(counts[start:end])
        if n_frames * (end - start) == 0:
            continue
        rate = d / (n_frames * (end - start))
        bar = "#" * int(rate * 40)
        print(f"      {start:5d}..{end-1:5d}  {rate:6.4f}  {bar}  [{label}]")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-oracle", type=Path, required=True)
    ap.add_argument("--in-dab-rs", type=Path, required=True)
    ap.add_argument("--out-oracle", type=Path, required=True)
    ap.add_argument("--out-dab-rs", type=Path, required=True)
    ap.add_argument("--offset-range", type=int, default=30)
    args = ap.parse_args()

    print(f"reading oracle  Viterbi input  : {args.in_oracle}")
    in_oracle = read_oracle_in(args.in_oracle)
    print(f"reading dab-rs Viterbi input  : {args.in_dab_rs}")
    in_dab_rs = read_dab_rs_in(args.in_dab_rs)
    print(f"reading oracle  Viterbi output : {args.out_oracle}")
    out_oracle = read_oracle_out(args.out_oracle)
    print(f"reading dab-rs Viterbi output : {args.out_dab_rs}")
    out_dab_rs = read_dab_rs_out(args.out_dab_rs)

    print(f"\nframe counts:  oracle in={len(in_oracle)}  out={len(out_oracle)}  "
          f"dab_rs in={len(in_dab_rs)}  out={len(out_dab_rs)}")

    print("\n--- Viterbi input (i16 soft bits) ---")
    in_offset, in_match, in_total = sweep_alignment(
        in_dab_rs, in_oracle, i16_match, args.offset_range)
    in_rate = in_match / in_total if in_total else 0.0
    print(f"best offset (oracle = dab_rs + {in_offset})  "
          f"i16 exact match {in_match}/{in_total} = {in_rate:.4f}")
    in_counts = per_position_diff(in_dab_rs, in_oracle, in_offset, FRAME_SOFT_LEN)
    n_in_frames = sum(1 for f in in_dab_rs if (f + in_offset) in in_oracle)
    show_bucket_histogram(in_counts, n_in_frames, "input", bucket=1024)

    print("\n--- Viterbi output (hard bits 0/1) ---")
    out_offset, out_match, out_total = sweep_alignment(
        out_dab_rs, out_oracle, bit_match, args.offset_range)
    out_rate = out_match / out_total if out_total else 0.0
    print(f"best offset (oracle = dab_rs + {out_offset})  "
          f"bit exact match {out_match}/{out_total} = {out_rate:.4f}")
    out_counts = per_position_diff(out_dab_rs, out_oracle, out_offset, FRAME_HARD_LEN)
    n_out_frames = sum(1 for f in out_dab_rs if (f + out_offset) in out_oracle)
    show_bucket_histogram(out_counts, n_out_frames, "output", bucket=256)

    # ---- Judgement ----
    print("\n========== JUDGEMENT ==========")
    if in_rate >= 0.99 and out_rate < 0.99:
        print("Result A — INPUTS MATCH, OUTPUTS DIFFER")
        print("  → Bug is inside dab-viterbi's scalar Viterbi decoder.")
        print("    Port viterbiSpiral (slice-7 fork 1).")
    elif in_rate < 0.99:
        print("Result B — INPUTS DIFFER")
        print("  → Bug is upstream of dab-viterbi.")
        print("    Likely OFDM bit-ordering inside the demap loop, the")
        print("    FIC depuncture geometry, or the sign convention of soft")
        print("    bits handed to the decoder.")
    else:
        print("Result C — BOTH MATCH")
        print("  → Viterbi chain is byte-identical between dab-rs and oracle;")
        print("    the FIB-CRC failure must be downstream — descrambler PRBS")
        print("    seed/polarity, or the MSB-first FIB packing step.")
    print("==================================")

    return 0


if __name__ == "__main__":
    sys.exit(main())
