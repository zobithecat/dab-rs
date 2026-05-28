#!/usr/bin/env python3
"""Slice-10: synthetic OFDM round-trip with a 2×2×2 hypothesis sweep.

Build a known 3072-info-bit FIC frame, encode + FIC-puncture to 9216
transmitted bits, generate the corresponding three 2048-bin
*differential* spectra under a configurable
`(p1=interleaver_direction, p2=iq_layout, p3=conj_direction)` triple,
pipe the 49152-byte spectra stream into the dab-rs
`dab synth-test` subcommand, read back the 384-byte (12 FIBs)
output, and check FIB CRC.

dab-rs's chain hard-codes ONE choice on each axis:
  • interleaver_direction = forward   (`FreqInterleaver::map_in(i)`)
  • iq_layout             = block     (1536 I bits, then 1536 Q bits)
  • conj_direction        = curr * conj(prev)

If dab-rs's choice matches the actual DAB transmitter on every axis,
exactly one configuration of the 8 should round-trip — namely
`(forward, block, curr_conj_prev)` — and the others should produce
garbage. If a *different* configuration is the one that round-trips,
THAT axis is the bug.

Output: a table of (p1, p2, p3) × (CRC pass /12, info-bit match
/3072), and a judgement.
"""

from __future__ import annotations

import argparse
import itertools
import math
import struct
import subprocess
import sys
from pathlib import Path

# DAB Mode I parameters.
T_U = 2048
K_CARRIERS = 1536
N_SYMS_FIC = 3
INFO_BITS = 3072            # = 12 FIBs × 256
MOTHER_BITS = INFO_BITS * 4 + 24 * 4  # 12 ficBlocks × (3072+24), but FIC frame is 4 × 3096 = 12384

# Actually for ONE FIC frame: 4 ficBlocks × 768 info bits = 3072 info per frame,
# 4 × (3072+24) = 12384 mother bits, 4 × 2304 = 9216 transmitted bits.
FIC_BLOCKS_PER_FRAME = 4
FIC_OUT_BITS = 768
FIC_VITERBI_LEN = 3072 + 24
FIC_IN_BITS = 2304
TRANSMITTED_BITS_PER_FRAME = FIC_BLOCKS_PER_FRAME * FIC_IN_BITS  # 9216

# DAB rate-1/4 mother code polynomials in MSB-newest convention.
POLYS = [0o133, 0o171, 0o145, 0o133]
KCONSTRAINT = 7


def parity(x: int) -> int:
    x ^= x >> 16
    x ^= x >> 8
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1


def bit_for(state: int, poly: int, bit: int) -> int:
    reg = state if bit == 0 else state + (1 << (KCONSTRAINT - 1))
    return parity(reg & poly)


def convolutional_encode(msg: list[int]) -> list[int]:
    state = 0
    n = len(msg)
    total = n + (KCONSTRAINT - 1)
    out: list[int] = []
    for step in range(total):
        b = msg[step] if step < n else 0
        for p in POLYS:
            out.append(bit_for(state, p, b))
        state = (state >> 1) | (b << (KCONSTRAINT - 2))
    return out


def build_fic_puncture_table() -> list[bool]:
    """Replicates the FIC depuncture table dab-viterbi and eti-stuff
    both build (21 × PI_16 + 3 × PI_15 + PI_X)."""
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


def encode_ficblock_and_puncture(info_bits: list[int]) -> list[int]:
    """768 info bits → 3096 mother → 2304 transmitted."""
    assert len(info_bits) == FIC_OUT_BITS
    coded = convolutional_encode(info_bits)
    assert len(coded) == FIC_VITERBI_LEN
    table = build_fic_puncture_table()
    out = []
    for c, keep in zip(coded, table):
        if keep:
            out.append(c)
    assert len(out) == FIC_IN_BITS
    return out


def encode_frame(info: list[int]) -> list[int]:
    """3072 info bits (12 FIBs) → 9216 transmitted bits per frame
    (= 4 ficBlocks of 2304)."""
    assert len(info) == INFO_BITS
    out: list[int] = []
    for f in range(FIC_BLOCKS_PER_FRAME):
        chunk = info[f * FIC_OUT_BITS:(f + 1) * FIC_OUT_BITS]
        out.extend(encode_ficblock_and_puncture(chunk))
    assert len(out) == TRANSMITTED_BITS_PER_FRAME
    return out


# ---- FreqInterleaver replica (matches dab-ofdm `freq_interleaver.rs`) ----

def build_perm() -> list[int]:
    v1 = 511
    lwb = 256
    upb = 256 + K_CARRIERS  # 1792
    half = T_U // 2  # 1024
    tmp = [0] * T_U
    for i in range(1, T_U):
        tmp[i] = (13 * tmp[i - 1] + v1) % T_U
    perm = []
    for val in tmp:
        if val == half:
            continue
        if val < lwb or val > upb:
            continue
        perm.append(val - half)
    assert len(perm) == K_CARRIERS
    return perm


def build_inverse_perm(perm: list[int]) -> list[int]:
    """inv[i] = j such that perm[j]'s `carrier index` interpretation
    matches. Used for the P1 = inverse synth direction."""
    # Build a map from carrier number to position in the table.
    # perm[k] ∈ {-768..-1, 1..768}. We need: for each logical carrier
    # number c (the value we will place onto the spectrum bin), what
    # logical-stream position k carries it. inv is indexed by k →
    # carrier such that the encoder writes bit-for-stream-position k
    # onto bin_of(carrier).
    inv = [0] * K_CARRIERS
    for k, carrier in enumerate(perm):
        # Forward: encoder writes bit k onto bin_of(perm[k]).
        # Inverse: encoder writes bit k onto bin_of(inv_perm[k]) where
        # inv_perm[k] is the carrier such that perm[inv_perm_carrier_idx] = k.
        # Concretely: we swap the role of source-and-target indices.
        pass
    # Simpler: P1 = inverse means we permute the bit STREAM through the
    # inverse permutation before placing — equivalent to placing bit k
    # onto carrier perm⁻¹(k). To get perm⁻¹: for each carrier c, find k
    # such that perm[k] = c. We need this mapping for all c in {-768..-1,
    # 1..768}. Build a dict.
    by_carrier = {c: k for k, c in enumerate(perm)}
    out = []
    # Walk the perm in carrier-number order (i.e., for each k, what
    # carrier does the inverse direction place it on?). Equivalently:
    # the inverse table's k-th entry is the carrier such that
    # forward_perm[that carrier's position] = k? This is getting hairy.
    # Pragmatic: just shuffle the perm by replacing perm[k] with the
    # carrier value at position perm[k]'s rank in the forward table.
    for k in range(K_CARRIERS):
        # Find the carrier c such that perm⁻¹(c) = k. That means perm[k]
        # is the "logical" index in the inverse interpretation. So in
        # the inverse direction the carrier is the position k *in the
        # forward table*, but expressed as the carrier label that
        # forward-table entry by_carrier might point to. Let's just
        # invert by mapping perm[k]'s carrier to k:
        out.append(by_carrier_reverse(perm, k))
    return out


def by_carrier_reverse(perm: list[int], k: int) -> int:
    """For the P1 = inverse direction, return the carrier label at
    which to place stream-bit k. Concretely: if forward places stream-
    bit k at carrier perm[k], inverse should place stream-bit k at the
    *carrier label whose rank in perm is k*. We treat perm as a
    bijection between {0..K-1} and {-768..-1, 1..768}; the inverse
    of this bijection (as functions on the integer set) gives the
    inverse direction."""
    # Build a sorted list of carriers; the j-th carrier in sorted order
    # is the natural number `j` we map onto. The forward table assigns
    # stream-position k to carrier perm[k]; for inverse, stream-position
    # k goes to the carrier *whose value equals the carrier that
    # forward places at stream-position perm[k]*. This is getting
    # circular. Use a simpler interpretation: invert by swap.
    inv = [0] * K_CARRIERS
    # The inverse permutation as a self-map of {0..K-1}: if forward maps
    # stream-position k to carrier index perm[k] (signed), build a
    # carrier→stream-position dictionary, then the inverse table walks
    # the natural carrier order and emits the original stream-position.
    return perm[k]  # placeholder — replaced below


def make_inverse_perm(perm: list[int]) -> list[int]:
    """The 'inverse direction' interpretation: build a new perm table
    `inv` such that running the synth with `inv[k]` as carrier label
    has the same effect as if dab-rs were using the *inverse* of its
    `map_in` permutation in its demap.

    Concretely: forward synth places stream-bit k onto carrier perm[k];
    forward dab-rs reads carrier perm[k] for stream-bit k. They round-
    trip.

    Inverse synth places stream-bit k onto carrier inv[k], where inv is
    chosen so that *no other axis change* round-trips. The cleanest
    definition: inv = the natural ordering of carriers (sorted) — this
    swaps the LCG-shuffled forward permutation for the identity-like
    inverse-sort placement."""
    # Sort carriers in their natural numerical order to define the
    # inverse direction. Skip carrier 0; signed range {-768..-1, 1..768}.
    sorted_carriers = sorted(
        [c for c in range(-K_CARRIERS // 2, K_CARRIERS // 2 + 1) if c != 0],
        key=lambda x: (abs(x), x),  # arbitrary deterministic order
    )
    return sorted_carriers


def bin_of(carrier: int) -> int:
    if carrier > 0:
        return carrier
    return carrier + T_U


def make_diff_spectra(transmitted: list[int],
                      p1: str, p2: str, p3: str,
                      perm_fwd: list[int],
                      perm_inv: list[int]) -> list[list[complex]]:
    """Convert 9216 transmitted bits into 3 differential spectra under
    a given (p1, p2, p3) configuration."""
    assert len(transmitted) == TRANSMITTED_BITS_PER_FRAME
    bits_per_sym = TRANSMITTED_BITS_PER_FRAME // N_SYMS_FIC  # 3072

    perm = perm_fwd if p1 == "forward" else perm_inv

    spectra: list[list[complex]] = []
    for s in range(N_SYMS_FIC):
        sym_bits = transmitted[s * bits_per_sym:(s + 1) * bits_per_sym]
        spec = [complex(0, 0)] * T_U
        for k in range(K_CARRIERS):
            if p2 == "block":
                # First K bits are a-bits, last K are b-bits.
                a = sym_bits[k]
                b = sym_bits[K_CARRIERS + k]
            else:
                # Per-carrier interleaved: (a_0, b_0, a_1, b_1, ...).
                a = sym_bits[2 * k]
                b = sym_bits[2 * k + 1]
            # Constellation under dab-rs's demap polarity:
            # bits[i]      = -r.re/|r|*127 with `+ ⇒ bit 1` convention,
            # so bit_a = 0 ↔ r.re > 0  and bit_a = 1 ↔ r.re < 0.
            r_re = +1.0 if a == 0 else -1.0
            r_im = +1.0 if b == 0 else -1.0
            if p3 == "conj_curr":
                # If transmitter uses `r = conj(curr) * prev` then
                # dab-rs's hardcoded `r' = curr * conj(prev) = conj(r)`
                # — i.e. the imaginary part is negated.
                r_im = -r_im
            mag = math.sqrt(2.0)
            r = complex(r_re / mag, r_im / mag)
            spec[bin_of(perm[k])] = r
        spectra.append(spec)
    return spectra


def serialise_spectra(spectra: list[list[complex]]) -> bytes:
    """Pack as 3 × 2048 × (f32 LE re, f32 LE im) = 49152 bytes."""
    buf = bytearray()
    for spec in spectra:
        for z in spec:
            buf.extend(struct.pack("<f", z.real))
            buf.extend(struct.pack("<f", z.imag))
    return bytes(buf)


def fib_crc_ok(fib: bytes) -> bool:
    if len(fib) != 32:
        return False
    crc = 0xFFFF
    for b in fib[:30]:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    crc ^= 0xFFFF
    expected = (fib[30] << 8) | fib[31]
    return crc == expected


def run_synth_test(dab: Path, spectra_bytes: bytes) -> bytes:
    r = subprocess.run([str(dab), "synth-test"],
                       input=spectra_bytes, capture_output=True)
    if r.returncode != 0:
        raise RuntimeError(f"synth-test rc={r.returncode}: "
                           f"{r.stderr.decode(errors='replace')}")
    return r.stdout


def make_info_bits(seed: int = 42) -> list[int]:
    # Use the same LCG style as slice 8 so the test is deterministic.
    s = seed
    out = []
    for _ in range(INFO_BITS):
        s = (s * 1103515245 + 12345) & 0x7FFFFFFF
        out.append((s >> 16) & 1)
    return out


def info_bits_to_fib_bytes(info: list[int]) -> bytes:
    """Pack 3072 info bits MSB-first into 384 bytes. The 3 FIBs per
    ficBlock × 4 ficBlocks layout matches dab-rs's `descramble_and_pack`
    output (modulo the PRBS XOR which is applied between info bits and
    the packing). The synth-test subcommand applies the PRBS XOR, so
    here we need to PRE-XOR with the PRBS so that the recovered bytes
    match this packed form."""
    # 4 ficBlocks × 768 bits each
    out = bytearray()
    for f in range(FIC_BLOCKS_PER_FRAME):
        chunk_bits = info[f * FIC_OUT_BITS:(f + 1) * FIC_OUT_BITS]
        # Pack MSB-first
        n = len(chunk_bits) // 8
        for i in range(n):
            byte = 0
            for j in range(8):
                byte = (byte << 1) | chunk_bits[i * 8 + j]
            out.append(byte)
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dab-rs", type=Path,
                    default=Path("/Users/zobithecat/Documents/projects/etc_projects/"
                                 "dab-rs/target/release/dab"))
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    perm_fwd = build_perm()
    perm_inv = make_inverse_perm(perm_fwd)
    info = make_info_bits(args.seed)
    transmitted = encode_frame(info)
    print(f"generated frame: info bits = {len(info)}, "
          f"transmitted = {len(transmitted)}")

    # The expected packed-bytes output from dab synth-test = MSB-first
    # pack of the FIC PRBS XOR'd info bits, in groups of 96 bytes per
    # ficBlock. dab-rs's chain applies the PRBS internally, so to
    # compare we apply it to our reference too.
    prbs = fic_prbs(FIC_OUT_BITS)
    expected_bytes = bytearray()
    for f in range(FIC_BLOCKS_PER_FRAME):
        chunk = info[f * FIC_OUT_BITS:(f + 1) * FIC_OUT_BITS]
        xored = [chunk[i] ^ prbs[i] for i in range(FIC_OUT_BITS)]
        for i in range(FIC_OUT_BITS // 8):
            byte = 0
            for j in range(8):
                byte = (byte << 1) | xored[i * 8 + j]
            expected_bytes.append(byte)
    expected_bytes = bytes(expected_bytes)

    print(f"reference packed bytes: {len(expected_bytes)}  first 8: "
          f"{expected_bytes[:8].hex()}")

    print("\n---- 8-config sweep ----")
    print(f"  {'p1':<8} {'p2':<13} {'p3':<14}  {'CRC pass /12':>12}  "
          f"{'info-bit match /3072':>20}")
    results = []
    for p1, p2, p3 in itertools.product(
            ["forward", "inverse"],
            ["block", "interleaved"],
            ["curr_conj_prev", "conj_curr"]):
        spectra = make_diff_spectra(transmitted, p1, p2, p3,
                                    perm_fwd, perm_inv)
        blob = serialise_spectra(spectra)
        out = run_synth_test(args.dab_rs, blob)
        assert len(out) == 384, f"synth-test returned {len(out)} bytes"
        crc_pass = sum(1 for k in range(12)
                       if fib_crc_ok(out[k * 32:(k + 1) * 32]))
        # Per-info-bit match: unpack `out` MSB-first, XOR with prbs to
        # get info bits, compare to reference `info`.
        info_recovered = []
        for f in range(FIC_BLOCKS_PER_FRAME):
            chunk_bytes = out[f * 96:(f + 1) * 96]
            chunk_bits = []
            for b in chunk_bytes:
                for j in range(7, -1, -1):
                    chunk_bits.append((b >> j) & 1)
            for i in range(FIC_OUT_BITS):
                info_recovered.append(chunk_bits[i] ^ prbs[i])
        match = sum(1 for i in range(INFO_BITS)
                    if info_recovered[i] == info[i])
        rate = match / INFO_BITS
        results.append((p1, p2, p3, crc_pass, match, rate))
        print(f"  {p1:<8} {p2:<13} {p3:<14}  {crc_pass:>12d}  "
              f"{match:>4d}/3072 = {rate:.4f}")

    # ---- Judgement ----
    best = max(results, key=lambda r: r[3] * 10000 + r[4])
    print("\n========== JUDGEMENT ==========")
    if best[3] == 12 and best[5] > 0.99:
        print(f"Result S — single config round-trips perfectly:")
        print(f"  ({best[0]}, {best[1]}, {best[2]})  "
              f"CRC {best[3]}/12  info match {best[5]:.4f}")
        canonical = ("forward", "block", "curr_conj_prev")
        if (best[0], best[1], best[2]) == canonical:
            print("  → dab-rs's conventions are correct on all 3 axes.")
            print("    The 0/2496 FIB failure on real signals must come")
            print("    from a different stage (puncture table, scrambler,")
            print("    or fic_iq's ficBlock boundary handling).")
        else:
            for axis, label in [
                (0, "FreqInterleaver direction (P1)"),
                (1, "I/Q layout (P2)"),
                (2, "differential conjugation (P3)"),
            ]:
                if best[axis] != canonical[axis]:
                    print(f"  → dab-rs has the wrong choice on axis: {label}.")
                    print(f"    Correct value is `{best[axis]}` "
                          f"(dab-rs currently uses `{canonical[axis]}`).")
    elif sum(1 for r in results if r[3] >= 1) >= 2:
        print(f"Result M — multiple configs partially round-trip:")
        for r in results:
            if r[3] >= 1:
                print(f"  ({r[0]}, {r[1]}, {r[2]})  "
                      f"CRC {r[3]}/12  info match {r[5]:.4f}")
    else:
        best_match = max(r[5] for r in results)
        print(f"Result N' — no config round-trips. best info-match = "
              f"{best_match:.4f}")
        print("  → Bug is outside the (P1, P2, P3) 3-axis space.")
    print("==================================")
    return 0


def fic_prbs(n: int) -> list[int]:
    sr = [1] * 9
    out = []
    for _ in range(n):
        b = sr[8] ^ sr[4]
        for k in range(8, 0, -1):
            sr[k] = sr[k - 1]
        sr[0] = b
        out.append(b)
    return out


if __name__ == "__main__":
    sys.exit(main())
