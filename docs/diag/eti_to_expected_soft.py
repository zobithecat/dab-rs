#!/usr/bin/env python3
"""Slice-14 Path A: Back-derive expected OFDM soft bits from a LIVE ETI(NI).

Approach: the LIVE ETI(NI) file (k8b_v4.eti) is byte-identical ground truth
produced by working eti-stuff in airspy mode. Each 6144-byte frame contains
12 FIBs (CRC-validated at capture time). For each ficBlock (3 FIBs = 96
bytes), we can:

  1. CRC-check the 3 FIBs to know which are valid
  2. Unpack 96 bytes → 768 descrambled bits (MSB-first per byte)
  3. Reverse PRBS XOR to get the 768 pre-PRBS info bits (= what Viterbi output)
  4. Convolutional re-encode with the dab-rs polys/convention → 3096 bits
  5. Apply the dab-rs puncture table → 2304 transmitted bits
  6. Map 1→+127, 0→-127 → 2304 i8 soft bits

The result is the SOFT BITS DAB-RS'S DEMAP SHOULD HAVE PRODUCED if the chain
were correct. Compare with the actual DAB_RS_DUMP_DEMAP_OUT to find the
first stage where dab-rs diverges from ground truth.

Output format (per frame, only frames where ≥ 1 ficBlock has all 3 FIBs OK):
    u32 LE frame_idx (1-based, matches dab-rs DAB_RS_DUMP_DEMAP_OUT)
    u8     valid_mask (bit b = ficBlock b has all 3 FIBs CRC-valid)
    u8     padding × 3
    i8     expected[4 * 2304] = 9216 bytes (sentinel 0 in invalid ficBlocks)

ETI(NI) frame layout per ETSI ETS 300 799 §6.1, Mode I:
    ERR(1) + FSYNC(3) + FCT(1) + (FICF<<7|NST)(1) + (FP<<5|MID<<3|FL_hi)(1)
    + FL_lo(1) + STC(NST*4) + MNSC(2) + HCRC(2) + FIC(12*32=384) + MST(...) + EOF/TIST
    FIC starts at offset 12 + NST*4 (4 sync/FC + NST*4 STC + 4 EOH).
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

# -----------------------------------------------------------------------------
# Static DAB tables (verbatim ports of dab-rs/eti-stuff)
# -----------------------------------------------------------------------------

# Viterbi polys, K=7 — same as dab-rs scalar viterbi-handler convention
# (new bit shifted into bit position K-1 = 6, taps read from poly LSB upward).
POLYS = [0o133, 0o171, 0o145, 0o133]
K = 7
NUM_STATES = 1 << (K - 1)  # 64

# Puncture tables P_CODES[14] = PI_15, P_CODES[15] = PI_16, P_CODES[7] = PI_X
# (verbatim from protTables.cpp row labels 1-indexed; dab-rs/tables.rs id).
# Row 16 (PI_16, 24 ones in 32): 8 × {1,1,1,0}.
# Row 15 (PI_15, 23 ones in 32): 7 × {1,1,1,0} + {1,1,0,0}.
# Row 8  (PI_X,  16 ones in 32): 8 × {1,1,0,0}; only first 24 used → 12 ones.
PI_16 = [1,1,1,0] * 8
PI_15 = [1,1,1,0] * 7 + [1,1,0,0]
PI_X  = [1,1,0,0] * 8

assert sum(PI_16) == 24
assert sum(PI_15) == 23
assert sum(PI_X) == 16
assert sum(PI_X[:24]) == 12

# -----------------------------------------------------------------------------
# Reusable primitives
# -----------------------------------------------------------------------------

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (poly 0x1021, init 0xFFFF, final XOR 0xFFFF) — same as
    dab-fic::crc16_ccitt and eti-stuff check_crc_bytes/check_CRC_bits."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc ^ 0xFFFF


def fib_ok(fib32: bytes) -> bool:
    if len(fib32) != 32:
        return False
    expected = (fib32[30] << 8) | fib32[31]
    return crc16_ccitt(fib32[:30]) == expected


def prbs_sequence(n: int) -> list[int]:
    """DAB FIC energy-dispersal PRBS: x⁹ + x⁵ + 1, init all-ones."""
    sr = [1] * 9
    out = []
    for _ in range(n):
        b = sr[8] ^ sr[4]
        sr = [b] + sr[:8]
        out.append(b)
    return out


def bit_for(state: int, poly: int, bit: int) -> int:
    reg = state if bit == 0 else state + NUM_STATES
    reg &= poly
    res = 0
    for _ in range(K + 1):
        res ^= reg & 1
        reg >>= 1
    return res


def convolutional_encode(msg_bits: list[int]) -> list[int]:
    """Verbatim port of dab-viterbi::convolutional_encode (conv A, polys 0o133…).
    Appends 6 zero tail bits → 4·(len+6) coded bits."""
    state = 0
    out = []
    total = len(msg_bits) + 6
    for step in range(total):
        bit = msg_bits[step] if step < len(msg_bits) else 0
        for poly in POLYS:
            out.append(bit_for(state, poly, bit))
        state = (state >> 1) | (bit << (K - 2))
    return out


def make_puncture_table() -> list[bool]:
    """Identical to dab-viterbi::FicProtection::new() and eti-stuff
    fic-handler.cpp ficHandler constructor."""
    table: list[bool] = []
    for _ in range(21):
        for k in range(32 * 4):
            table.append(PI_16[k % 32] != 0)
    for _ in range(3):
        for k in range(32 * 4):
            table.append(PI_15[k % 32] != 0)
    for k in range(24):
        table.append(PI_X[k] != 0)  # uses first 24 entries of PI_X
    assert len(table) == 3072 + 24
    assert sum(table) == 2304
    return table


# -----------------------------------------------------------------------------
# Main back-derivation
# -----------------------------------------------------------------------------

FRAME_SIZE = 6144
# CRITICAL: each ETI(NI) frame contains 1 ficBlock (3 FIBs = 96 bytes),
# NOT 4 ficBlocks per frame. eti-generator.cpp:293 `memcpy(&theVector[off],
# fibVector[index_Out], 96)` copies a single ring-buffer slot per emit.
# So one DAB OFDM frame (4 ficBlocks) is split across 4 ETI(NI) frames.
FIBS_PER_ETI_FRAME = 3
FIC_BYTES = FIBS_PER_ETI_FRAME * 32  # 96
FICBLOCKS_PER_DAB_FRAME = 4
SOFT_BITS_PER_FICBLOCK = 2304
SOFT_BITS_PER_DAB_FRAME = FICBLOCKS_PER_DAB_FRAME * SOFT_BITS_PER_FICBLOCK  # 9216


def process_eti(eti_path: Path, out_path: Path, max_frames: int | None,
                phase: int = 0) -> None:
    eti = eti_path.read_bytes()
    n_frames = len(eti) // FRAME_SIZE
    if max_frames is not None:
        n_frames = min(n_frames, max_frames)
    print(f"ETI: {len(eti)} bytes → {n_frames} frames")

    prbs = prbs_sequence(768)
    punc = make_puncture_table()

    # Pass 1: extract per-ETI-frame ficBlock — 1 ficBlock per ETI frame.
    # Re-group into DAB frames (4 consecutive ETI frames = 1 DAB frame).
    eti_ficblocks: list[tuple[int, bytes, bool]] = []  # (eti_idx, 96 bytes, all_3_FIBs_valid)
    total_fibs = 0
    fib_pass = 0

    for frame_idx_zero in range(n_frames):
        frame = eti[frame_idx_zero * FRAME_SIZE : (frame_idx_zero + 1) * FRAME_SIZE]
        if len(frame) < FRAME_SIZE:
            break
        if frame[0] != 0xFF:
            eti_ficblocks.append((frame_idx_zero, b"", False))
            continue
        nst = frame[5] & 0x7F
        fic_off = 12 + nst * 4  # SYNC(4) + FC(4) + STC(NST*4) + EOH(4)
        if fic_off + FIC_BYTES > FRAME_SIZE:
            eti_ficblocks.append((frame_idx_zero, b"", False))
            continue
        ficblock_bytes = frame[fic_off : fic_off + FIC_BYTES]  # 96 bytes = 3 FIBs

        all_ok = True
        for fi in range(FIBS_PER_ETI_FRAME):
            fib = ficblock_bytes[fi * 32 : (fi + 1) * 32]
            total_fibs += 1
            if fib_ok(fib):
                fib_pass += 1
            else:
                all_ok = False
        eti_ficblocks.append((frame_idx_zero, ficblock_bytes, all_ok))

    print(f"FIB CRC pass rate (LIVE ground truth): {fib_pass}/{total_fibs} = "
          f"{100*fib_pass/total_fibs:.1f}%")

    # Pass 2: group 4 consecutive ETI frames → 1 DAB frame's 4 ficBlocks.
    # `phase` lets the user shift the boundary if eti-cmdline-airspy started
    # mid-OFDM-frame (so eti_ficblocks[0] is actually ficBlock `phase` of
    # some OFDM frame, not ficBlock 0). The first `(4-phase) % 4` ETI frames
    # are then dropped to align on a clean OFDM-frame boundary.
    skip = (FICBLOCKS_PER_DAB_FRAME - phase) % FICBLOCKS_PER_DAB_FRAME
    eti_ficblocks = eti_ficblocks[skip:]
    n_dab_frames = len(eti_ficblocks) // FICBLOCKS_PER_DAB_FRAME
    print(f"phase={phase} → skipped {skip} ETI frames, grouping {n_dab_frames} DAB frames")
    frames_emitted = 0
    ficblocks_emitted = 0

    with out_path.open("wb") as out:
        for dab_frame in range(n_dab_frames):
            valid_mask = 0
            expected = bytearray(SOFT_BITS_PER_DAB_FRAME)

            for ficblock in range(FICBLOCKS_PER_DAB_FRAME):
                eti_idx, ficblock_bytes, ok = eti_ficblocks[
                    dab_frame * FICBLOCKS_PER_DAB_FRAME + ficblock
                ]
                if not ok or len(ficblock_bytes) != 96:
                    continue

                # Unpack 96 bytes → 768 descrambled bits, MSB-first per byte.
                desc = []
                for byte in ficblock_bytes:
                    for b in range(8):
                        desc.append((byte >> (7 - b)) & 1)

                # Reverse PRBS XOR → pre-PRBS info bits (= Viterbi output).
                info = [desc[i] ^ prbs[i] for i in range(768)]

                mother = convolutional_encode(info)
                tx = [mother[i] for i in range(3096) if punc[i]]
                assert len(tx) == 2304

                soft = bytes((127 if b else (-127 & 0xFF)) for b in tx)
                expected[ficblock * SOFT_BITS_PER_FICBLOCK
                         : (ficblock + 1) * SOFT_BITS_PER_FICBLOCK] = soft

                valid_mask |= 1 << ficblock
                ficblocks_emitted += 1

            if valid_mask == 0:
                continue

            dab_frame_idx_1based = dab_frame + 1  # match dab-rs frame_idx convention
            out.write(struct.pack("<I", dab_frame_idx_1based))
            out.write(bytes([valid_mask, 0, 0, 0]))
            out.write(bytes(expected))
            frames_emitted += 1

    print(f"DAB frames emitted (≥ 1 valid ficBlock): {frames_emitted}/{n_dab_frames}")
    print(f"FicBlocks emitted (all 3 FIBs valid): {ficblocks_emitted}/"
          f"{FICBLOCKS_PER_DAB_FRAME * n_dab_frames}")
    print(f"Output: {out_path}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("eti", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--phase", type=int, default=0,
                    help="ficBlock starting phase: which of the 4 ficBlock slots "
                         "the first ETI frame represents (0..3). Useful if "
                         "eti-cmdline-airspy started mid-OFDM-frame.")
    args = ap.parse_args()
    process_eti(args.eti, args.out, args.max_frames, args.phase)
    return 0


if __name__ == "__main__":
    sys.exit(main())
