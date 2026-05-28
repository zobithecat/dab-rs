#!/usr/bin/env python3
"""Slice-9 transform bisection over the OFDM-to-Viterbi handoff.

Slice 8 cleared `dab-viterbi`'s convention against `viterbiSpiral`:
both decoders are byte-identical on real DAB-encoded soft bits *and*
both perfectly recover the encoder's input. So the 0/2496 FIB CRC
failure on `k8b_v4.iq` must come from a bit-ordering / sign /
permutation mistake somewhere *between* `dqpsk_demap` and the FIC
chain's `FicProtection::deconvolve`.

This script takes one 2304-byte ficBlock pulled from dab-rs's actual
demap output (env var `DAB_RS_DUMP_DEMAP_OUT` on `dab fic-iq`), applies
each candidate transform, pipes the result through `dab viterbi-cli`,
and checks the 96-byte (3-FIB) output for CRC validity. Whichever
transform produces a valid FIB tells us which inverse-transform is
the missing OFDM-side step.

Output format from `dab viterbi-cli`: 768 bytes (bit-per-byte; the
hard bits the Viterbi emits before PRBS energy-dispersal). We
descramble with the FIC PRBS and check three 256-bit FIBs per
transform.

Three judgement classes:

  Result S — Exactly one transform passes CRC → that transform's
             *inverse* is the bug; apply it in
             `dqpsk_demap` / `FreqInterleaver` / `fic_iq`.
  Result M — Multiple transforms pass, or none alone but several
             approach 75 % bit-match against live FIBs → run a
             paired sweep.
  Result N — Nothing comes close to passing → the bug needs more
             surgical bisection (different stage, or multi-transform
             composition).
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
from pathlib import Path

FIC_IN_BITS = 2304
FIC_OUT_BITS = 768
FRAME_SOFT_LEN = 4 * FIC_IN_BITS         # 9216 i8 per frame
DEMAP_RECORD_BYTES = 4 + FRAME_SOFT_LEN  # u32 frame_idx + 9216 i8
ETI_FRAME = 6144
FIB_SIZE = 32
FIB_BITS = 256
FIC_FIBS_PER_ETI = 4

# --- FIC PRBS (matches dab-descramble::prbs_sequence) -----------------

def fic_prbs(n: int) -> list[int]:
    """9-bit shift register init all-ones, taps at positions 8 and 4."""
    sr = [1] * 9
    out = []
    for _ in range(n):
        b = sr[8] ^ sr[4]
        for k in range(8, 0, -1):
            sr[k] = sr[k - 1]
        sr[0] = b
        out.append(b)
    return out


_PRBS_768 = fic_prbs(FIC_OUT_BITS)


# --- CRC-16-CCITT (matches dab-fic::crc16_ccitt) ---------------------

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc ^ 0xFFFF


def fib_crc_ok(fib: bytes) -> bool:
    if len(fib) != FIB_SIZE:
        return False
    expected = (fib[30] << 8) | fib[31]
    return crc16_ccitt(fib[:30]) == expected


def pack_msb_first(bits: list[int]) -> bytes:
    """Pack a 0/1 list into bytes, MSB first per byte. matches
    dab-descramble's `descramble_and_pack` layout."""
    n = len(bits) // 8
    out = bytearray(n)
    for i in range(n):
        v = 0
        for j in range(8):
            v = (v << 1) | (bits[i * 8 + j] & 1)
        out[i] = v
    return bytes(out)


# --- Transform catalog -----------------------------------------------

def t_identity(b: bytes) -> bytes:
    return b


def t_byte_negate(b: bytes) -> bytes:
    # Two's-complement byte-wise negate. Matches negating each i8 soft bit.
    return bytes(((-bytes_to_i8(x)) & 0xFF) for x in b)


def bytes_to_i8(x: int) -> int:
    return x - 256 if x >= 128 else x


def t_bit_reverse_per_byte(b: bytes) -> bytes:
    def rev(x: int) -> int:
        x = ((x & 0x55) << 1) | ((x & 0xAA) >> 1)
        x = ((x & 0x33) << 2) | ((x & 0xCC) >> 2)
        x = ((x & 0x0F) << 4) | ((x & 0xF0) >> 4)
        return x & 0xFF
    return bytes(rev(x) for x in b)


def t_full_reverse(b: bytes) -> bytes:
    return b[::-1]


def t_swap_iq_per_symbol(b: bytes) -> bytes:
    # 2304 = 3 OFDM symbols × 768 (1536 I + 768 Q? no, 1536/2 = wait).
    # Per OFDM symbol, demap writes 1536 I then 1536 Q = 3072 bits.
    # ficBlock spans 2304 bits = part of 3 symbols. Within one ficBlock
    # the I-block-vs-Q-block split isn't symmetric across the 2304-bit
    # span; this transform splits the 2304 in half and swaps.
    half = len(b) // 2
    return b[half:] + b[:half]


def t_interleave_iq_pairs(b: bytes) -> bytes:
    # Treat as two halves; interleave: I0,Q0,I1,Q1,…
    half = len(b) // 2
    a, c = b[:half], b[half:]
    out = bytearray(len(b))
    for i in range(half):
        out[2 * i] = a[i]
        out[2 * i + 1] = c[i]
    return bytes(out)


def t_reverse_first_half(b: bytes) -> bytes:
    half = len(b) // 2
    return b[:half][::-1] + b[half:]


def t_reverse_second_half(b: bytes) -> bytes:
    half = len(b) // 2
    return b[:half] + b[half:][::-1]


def t_negate_first_half(b: bytes) -> bytes:
    half = len(b) // 2
    return t_byte_negate(b[:half]) + b[half:]


def t_negate_second_half(b: bytes) -> bytes:
    half = len(b) // 2
    return b[:half] + t_byte_negate(b[half:])


def t_rotate_left_1bit(b: bytes) -> bytes:
    # Rotate the 2304-bit stream left by 1 bit. Each byte holds one
    # soft bit (the sign matters; magnitude carries the metric). Bit
    # rotation here means rotating the *byte order* by 1 position.
    return b[1:] + b[:1]


def t_rotate_right_1bit(b: bytes) -> bytes:
    return b[-1:] + b[:-1]


def t_byte_pair_swap(b: bytes) -> bytes:
    out = bytearray(b)
    for i in range(0, len(out) - 1, 2):
        out[i], out[i + 1] = out[i + 1], out[i]
    return bytes(out)


def t_reverse_per_768(b: bytes) -> bytes:
    # ficBlock-internal reverse in 768-bit chunks (= 3 FIBs).
    out = bytearray(len(b))
    for s in range(0, len(b), 768):
        out[s:s + 768] = b[s:s + 768][::-1]
    return bytes(out)


def t_negate_alt_bytes(b: bytes) -> bytes:
    out = bytearray(b)
    for i in range(0, len(out), 2):
        out[i] = (-bytes_to_i8(out[i])) & 0xFF
    return bytes(out)


TRANSFORMS = [
    ("T0  identity",                     t_identity),
    ("T1  byte_negate",                  t_byte_negate),
    ("T2  bit_reverse_per_byte",         t_bit_reverse_per_byte),
    ("T3  full_reverse",                 t_full_reverse),
    ("T4  swap_halves",                  t_swap_iq_per_symbol),
    ("T5  interleave_halves",            t_interleave_iq_pairs),
    ("T6  reverse_first_half",           t_reverse_first_half),
    ("T7  reverse_second_half",          t_reverse_second_half),
    ("T8  negate_first_half",            t_negate_first_half),
    ("T9  negate_second_half",           t_negate_second_half),
    ("T10 rotate_left_1byte",            t_rotate_left_1bit),
    ("T11 rotate_right_1byte",           t_rotate_right_1bit),
    ("T12 byte_pair_swap",               t_byte_pair_swap),
    ("T13 reverse_per_768",              t_reverse_per_768),
    ("T14 negate_alt_bytes",             t_negate_alt_bytes),
]


def run_viterbi_cli(cli: Path, soft: bytes) -> bytes:
    r = subprocess.run([str(cli), "viterbi-cli"], input=soft, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"viterbi-cli rc={r.returncode}: "
                           f"{r.stderr.decode(errors='replace')}")
    return r.stdout


def descramble_and_pack(hard_bits: bytes) -> bytes:
    """Hard bits (768 × 0/1) → descrambled, then MSB-first packed into 96 bytes."""
    xored = [hard_bits[i] ^ _PRBS_768[i] for i in range(FIC_OUT_BITS)]
    return pack_msb_first(xored)


def fic_block_pass(fic_bytes: bytes) -> int:
    """Count how many of the 3 FIBs in 96 bytes pass CRC."""
    assert len(fic_bytes) == FIB_SIZE * 3
    return sum(1 for k in range(3) if fib_crc_ok(fic_bytes[k * 32:(k + 1) * 32]))


def load_live_reference(eti_path: Path, frame_index: int, slot: int) -> bytes:
    """Live ETI's real FIB at the given (frame_index, slot ∈ {0,1,2})."""
    with eti_path.open("rb") as f:
        f.seek(frame_index * ETI_FRAME)
        frame = f.read(ETI_FRAME)
    nst = frame[5] & 0x7F
    off = 4 + 4 + 4 * nst + 4  # SYNC + FC + STC + EOH
    return frame[off + slot * FIB_SIZE:off + (slot + 1) * FIB_SIZE]


def bit_match(a: bytes, b: bytes) -> tuple[int, int]:
    n = min(len(a), len(b)) * 8
    if n == 0:
        return 0, 0
    matched = 0
    for byte_a, byte_b in zip(a, b):
        x = byte_a ^ byte_b
        # popcount of inverted bits = mismatches; flip for matches
        matched += 8 - bin(x).count("1")
    return matched, n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--demap", type=Path, default=Path("/tmp/dab_rs_demap_out.bin"))
    ap.add_argument("--eti", type=Path,
                    default=Path("/Users/zobithecat/Documents/projects/etc_projects/"
                                 "airspy-mini-dmb/data/captures/k8b_v4.eti"))
    ap.add_argument("--dab-rs", type=Path,
                    default=Path("/Users/zobithecat/Documents/projects/etc_projects/"
                                 "dab-rs/target/release/dab"))
    ap.add_argument("--frame", type=int, default=0, help="dab-rs frame index (0-based)")
    ap.add_argument("--ficblock", type=int, default=0, help="ficBlock index 0..3")
    ap.add_argument("--ref-frame", type=int, default=0, help="live ETI frame index")
    ap.add_argument("--ref-slot", type=int, default=0, help="live ETI slot 0..2")
    args = ap.parse_args()

    # ---- Pull the chosen ficBlock from the demap dump ----
    raw = args.demap.read_bytes()
    n_frames = len(raw) // DEMAP_RECORD_BYTES
    if args.frame >= n_frames:
        print(f"error: frame {args.frame} out of range [{n_frames} frames]")
        return 2
    base = args.frame * DEMAP_RECORD_BYTES
    frame_idx, = struct.unpack_from("<I", raw, base)
    soft = raw[base + 4:base + 4 + FRAME_SOFT_LEN]
    soft_block = soft[args.ficblock * FIC_IN_BITS:(args.ficblock + 1) * FIC_IN_BITS]
    print(f"dab-rs frame_idx={frame_idx}, ficBlock={args.ficblock}, "
          f"{len(soft_block)} soft bytes "
          f"(mean |b|={sum(abs(bytes_to_i8(x)) for x in soft_block)/len(soft_block):.1f})")

    # ---- Live ETI reference FIB ----
    live_fib = load_live_reference(args.eti, args.ref_frame, args.ref_slot)
    print(f"live ETI FIB (frame {args.ref_frame}, slot {args.ref_slot}): "
          f"CRC ok = {fib_crc_ok(live_fib)}, first 8 bytes = "
          f"{live_fib[:8].hex()}")

    # ---- Sweep ----
    print("\n---- transform sweep ----")
    print(f"  {'transform':<32}  {'CRC pass /3':>11}  {'best FIB bit-match':>18}")
    results = []
    for label, fn in TRANSFORMS:
        try:
            transformed = fn(soft_block)
            assert len(transformed) == FIC_IN_BITS, f"{label} returned {len(transformed)}"
            hard = run_viterbi_cli(args.dab_rs, transformed)
            fic_bytes = descramble_and_pack(hard)
            n_pass = fic_block_pass(fic_bytes)
            best_match = 0
            best_idx = -1
            for k in range(3):
                fib = fic_bytes[k * 32:(k + 1) * 32]
                m, total = bit_match(fib, live_fib)
                if m > best_match:
                    best_match = m
                    best_idx = k
            results.append((label, n_pass, best_match, best_idx))
            print(f"  {label:<32}  {n_pass:>11d}  "
                  f"{best_match:>3d}/256 = {best_match/256:.4f}  (FIB {best_idx})")
        except Exception as e:
            print(f"  {label:<32}  ERROR: {e}")
            results.append((label, -1, -1, -1))

    # ---- Judgement ----
    print("\n========== JUDGEMENT ==========")
    n_passing = sum(1 for _, p, _, _ in results if p > 0)
    best_bit = max((m for _, _, m, _ in results), default=0)
    if n_passing == 1:
        win = next((label, p) for label, p, _, _ in results if p > 0)
        print(f"Result S — Exactly one transform produced a valid FIB: {win}")
        print("  → Locate the inverse of this transform in dab-rs's OFDM /")
        print("    FIC handoff and remove it.")
    elif n_passing > 1:
        print(f"Result M — {n_passing} transforms produced valid FIBs:")
        for label, p, m, k in results:
            if p > 0:
                print(f"    {label}  CRC pass={p}/3  bit-match={m}/256")
        print("  → Inspect the passing set; they may share a common")
        print("    sub-permutation. Consider a paired transform sweep.")
    else:
        print(f"Result N — No transform produced a valid FIB.")
        print(f"  best bit-match against live FIB: {best_bit}/256 = {best_bit/256:.4f}")
        print("  → Bug is deeper than a single byte-level transform.")
        print("    Switch to slice-9 fork 2 (synthetic round-trip via")
        print("    docs/diag/viterbi_unit_diff.py::make_encoded_vector).")
    print("==================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
