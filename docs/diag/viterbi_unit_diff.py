#!/usr/bin/env python3
"""Slice-8 standalone Viterbi unit test.

Generates 5 deterministic 2304-soft-bit test vectors, pipes each one
through the eti-stuff `viterbiSpiral` harness and the dab-rs
`dab viterbi-cli` harness, and bit-XOR's the 768-bit outputs to
distinguish:

  Result X — All 5 outputs bit-identical → dab-viterbi convention
             is correct, bug is upstream (OFDM bit-ordering).
  Result Y — Outputs differ with a systematic pattern across vectors:
               * uniform inversion → soft-bit polarity convention
               * bit-reverse per byte → MSB/LSB packing endianness
               * sliding shift → polynomial index/order
               * block-boundary divergence → trellis termination
  Result Z — Random ≈ 50 % diff with no structure → deeper bisection
             needed (try shorter input length).

Test vectors
------------

1. vec_zeros       — 2304 × `0x00` (all soft bits zero, ambiguous input)
2. vec_plus127     — 2304 × `+127` (all maximally-positive)
3. vec_minus127    — 2304 × `-127` (all maximally-negative)
4. vec_alternating — `+127, -127, +127, -127, …`
5. vec_encoded     — encode a known 768-bit info word with the DAB
                     rate-1/4 mother code + the FIC puncture pattern;
                     map kept bits to ±127. The decoder *must*
                     recover the original info word for this to be a
                     valid round-trip — if it does on either side,
                     that side's polynomial/state convention agrees
                     with the encoder's.

The encoder convention here matches dab-viterbi's `convolutional_encode`:
polynomials `{0o133, 0o171, 0o145, 0o133}`, MSB-newest register, output
4 bits per info bit in poly order. If `viterbiSpiral` uses a different
internal convention (bit-reversed polys + LSB-newest register, per its
own source comments) but is the inverse of the *same* DAB transmitter
encoder, the round-trip will *still* recover the original info word —
because both are valid decoders for the same code, just with different
implementations. A round-trip failure on the *spiral* side under this
encoder is hard evidence that the conventions are not interchangeable.
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
from pathlib import Path

FIC_IN_BITS = 2304
FIC_VITERBI_LEN = 3072 + 24
FIC_OUT_BITS = 768

# DAB rate-1/4 mother code polynomials in MSB-newest convention,
# matching dab-viterbi::POLYS.
POLYS = [0o133, 0o171, 0o145, 0o133]
K = 7  # constraint length


def parity(x: int) -> int:
    x ^= x >> 16
    x ^= x >> 8
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1


def bit_for(state: int, poly: int, bit: int) -> int:
    """Same trellis-output bit computation dab-viterbi uses."""
    reg = state if bit == 0 else state + (1 << (K - 1))
    return parity(reg & poly)


def convolutional_encode(message: list[int]) -> list[int]:
    """rate-1/4 DAB convolutional encode, MSB-newest register. Returns
    `4 * (len(message) + K - 1)` coded bits, matching dab-viterbi's
    `convolutional_encode`."""
    state = 0
    n = len(message)
    total = n + (K - 1)  # 6 tail zeros
    out: list[int] = []
    for step in range(total):
        bit = message[step] if step < n else 0
        for poly in POLYS:
            out.append(bit_for(state, poly, bit))
        # Advance the register: shift right one position, insert bit at the
        # K-2 slot (matches dab-viterbi's `state = (state >> 1) | (bit <<
        # (K - 2))`).
        state = (state >> 1) | (bit << (K - 2))
    return out


def build_fic_puncture_table() -> list[bool]:
    """Replicates the FIC depuncture table dab-viterbi and eti-stuff
    both build (21 × PI_16 + 3 × PI_15 + PI_X). Returns a length-3096
    boolean array."""
    # PCodes patterns: only the *positions* in `[0, 32)` that get
    # transmitted matter. We embed the three FIC PCodes verbatim from
    # the eti-stuff `protTables.cpp`.
    # Verbatim from eti-stuff `protTables.cpp` rows 16, 15, 8 (1-based).
    pi_16 = [1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,
             1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0]
    pi_15 = [1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,
             1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 1, 0,  1, 1, 0, 0]
    pi_x  = [1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0,
             1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0]
    table = [False] * FIC_VITERBI_LEN
    local = 0
    for _ in range(21):
        for k in range(32 * 4):
            if pi_16[k % 32]:
                table[local] = True
            local += 1
    for _ in range(3):
        for k in range(32 * 4):
            if pi_15[k % 32]:
                table[local] = True
            local += 1
    for k in range(24):
        if pi_x[k]:
            table[local] = True
        local += 1
    assert local == FIC_VITERBI_LEN
    return table


def make_encoded_vector(message: list[int]) -> bytes:
    """Encode `message` with the DAB mother code, puncture per the FIC
    pattern, map each kept bit to ±127 soft. Returns 2304 signed bytes."""
    assert len(message) == FIC_OUT_BITS
    coded = convolutional_encode(message)  # 3096 bits
    assert len(coded) == FIC_VITERBI_LEN
    table = build_fic_puncture_table()
    soft = bytearray()
    kept = sum(1 for t in table if t)
    assert kept == FIC_IN_BITS, f"puncture geometry off ({kept} kept)"
    for i, keep in enumerate(table):
        if keep:
            # In the *encoder→decoder* round-trip, dab-viterbi's tests map
            # bit 0 → −255, bit 1 → +255. Here we use ±127 to fit i8 and
            # match what the OFDM demap would produce on a clean channel.
            soft.append(127 if coded[i] == 1 else (256 - 127))  # +127 / −127
    return bytes(soft)


def run_harness(cmd: list[str], stdin_bytes: bytes) -> bytes:
    r = subprocess.run(cmd, input=stdin_bytes, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"{cmd[0]} returned {r.returncode}: "
                           f"{r.stderr.decode(errors='replace')}")
    return r.stdout


def hex_first(buf: bytes, n: int = 16) -> str:
    return " ".join(f"{b:02x}" for b in buf[:n])


def diff_stats(a: bytes, b: bytes) -> tuple[int, int, list[int]]:
    n = min(len(a), len(b))
    match = sum(1 for i in range(n) if a[i] == b[i])
    diffs = [a[i] ^ b[i] for i in range(n)]
    return match, n, diffs


def analyse_pattern(a: bytes, b: bytes) -> str:
    """Look at the bit-by-bit diff stream and classify."""
    n = min(len(a), len(b))
    if n == 0:
        return "empty input"
    same = sum(1 for i in range(n) if a[i] == b[i])
    if same == n:
        return "bit-identical"
    if same == 0 and all(a[i] in (0, 1) and b[i] in (0, 1) and a[i] != b[i]
                         for i in range(n)):
        return "uniform inversion"
    # Run lengths of agreement / disagreement give an autocorrelation hint.
    boundaries = []
    cur = a[0] == b[0]
    run = 1
    for i in range(1, n):
        if (a[i] == b[i]) == cur:
            run += 1
        else:
            boundaries.append((cur, run))
            cur = a[i] == b[i]
            run = 1
    boundaries.append((cur, run))
    n_blocks = len(boundaries)
    max_run = max(r for _, r in boundaries)
    if max_run >= n // 4:
        return f"block-boundary structure (max run {max_run}/{n})"
    return f"no obvious pattern (boundaries={n_blocks}, max run={max_run})"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--spiral", type=Path,
                    default=Path("/tmp/viterbi_spiral_cli"))
    ap.add_argument("--dab-rs", type=Path,
                    default=Path(
                        "/Users/zobithecat/Documents/projects/etc_projects/"
                        "dab-rs/target/release/dab"))
    ap.add_argument("--dab-rs-arg", type=str, default="viterbi-cli")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    if not args.spiral.exists():
        print(f"error: {args.spiral} missing — build with:")
        print("  c++ ... docs/diag/viterbi_spiral_cli.cpp ...  (see file)")
        return 2

    # Build vectors.
    rng_state = args.seed
    def lcg():
        nonlocal rng_state
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFF_FFFF
        return rng_state

    msg_random = [(lcg() >> 16) & 1 for _ in range(FIC_OUT_BITS)]
    vectors = {
        "zeros":       b"\x00" * FIC_IN_BITS,
        "plus127":     b"\x7f" * FIC_IN_BITS,
        "minus127":    b"\x81" * FIC_IN_BITS,
        "alternating": (b"\x7f\x81" * (FIC_IN_BITS // 2)),
        "encoded_rng": make_encoded_vector(msg_random),
    }

    print("---- vector summary ----")
    for name, blob in vectors.items():
        first = " ".join(f"{b:02x}" for b in blob[:8])
        print(f"  {name:14s}  {len(blob)} bytes  first 8: {first}")

    print("\n---- harness outputs ----")
    spiral_cmd = [str(args.spiral)]
    dabrs_cmd  = [str(args.dab_rs), args.dab_rs_arg]

    all_match = True
    pattern_summary = []
    for name, blob in vectors.items():
        out_spiral = run_harness(spiral_cmd, blob)
        out_dabrs  = run_harness(dabrs_cmd,  blob)
        match, n, _ = diff_stats(out_spiral, out_dabrs)
        rate = match / n if n else 0.0
        pattern = analyse_pattern(out_spiral, out_dabrs)
        pattern_summary.append((name, rate, pattern))
        sum_o = sum(out_spiral)
        sum_d = sum(out_dabrs)
        print(f"  {name:14s} match {match}/{n} = {rate:.4f}  "
              f"sums O={sum_o:>4d} D={sum_d:>4d}  "
              f"pattern: {pattern}")
        if rate < 0.9999:
            all_match = False

    # ---- Encoded round-trip recovery test ----
    print("\n---- encoded vector round-trip ----")
    out_o = run_harness(spiral_cmd, vectors["encoded_rng"])
    out_d = run_harness(dabrs_cmd,  vectors["encoded_rng"])
    expected_first = bytes(msg_random[:32])
    matches_o = sum(1 for i in range(FIC_OUT_BITS) if out_o[i] == msg_random[i])
    matches_d = sum(1 for i in range(FIC_OUT_BITS) if out_d[i] == msg_random[i])
    print(f"  spiral output vs original message: "
          f"{matches_o}/{FIC_OUT_BITS} = {matches_o/FIC_OUT_BITS:.4f}")
    print(f"  dab-rs output vs original message: "
          f"{matches_d}/{FIC_OUT_BITS} = {matches_d/FIC_OUT_BITS:.4f}")

    # ---- Judgement ----
    print("\n========== JUDGEMENT ==========")
    if all_match:
        print("Result X — All 5 vectors produce bit-identical outputs.")
        print("  → dab-viterbi convention is correct; bug is upstream.")
        print("    Next slice: audit OFDM bit-ordering against EN 300 401 §14.6.")
    else:
        rates = [r for _, r, _ in pattern_summary]
        avg = sum(rates) / len(rates)
        if avg < 0.55:
            print("Result Z — Outputs are near-random; no systematic pattern.")
            print("  → Deeper bisection needed (try shorter inputs).")
        else:
            print("Result Y — Outputs differ systematically.")
            for n, r, p in pattern_summary:
                print(f"    {n:14s} {r:.4f}  {p}")
            print("  → Investigate the pattern; map to convention difference:")
            print("    uniform inversion       → soft-bit polarity")
            print("    bit-reverse per byte    → packing endianness")
            print("    sliding shift           → polynomial order")
            print("    block-boundary structure → trellis termination")
    print("==================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
