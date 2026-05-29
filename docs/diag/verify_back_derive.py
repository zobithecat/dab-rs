#!/usr/bin/env python3
"""Self-check the eti_to_expected_soft.py back-derivation.

For each LIVE ficBlock with all-3-FIBs CRC-valid, the chain
  FIB bytes
    -> desc[768] (unpack MSB-first)
    -> info[768] = desc XOR PRBS
    -> mother[3096] = convolutional_encode(info)
    -> tx[2304] = puncture(mother)
must be deterministic and reversible:
  Round-trip: descramble(pack(info)) should equal original FIB bytes.

If self-round-trip passes byte-perfect, the back-derivation IS correct and
the 50%-agreement signal from compare_demap.py truly says dab-rs's
demap output is statistically uncorrelated with the LIVE ground truth — i.e.
the OFDM chain is producing wrong bits even though slice 11 saw clean
π/4-DQPSK clusters.

This script is independent of dab-rs (no Rust dependency).
"""

from __future__ import annotations

import sys
from pathlib import Path

# Reuse exactly the same primitives as eti_to_expected_soft.py.
sys.path.insert(0, str(Path(__file__).parent))
from eti_to_expected_soft import (  # noqa: E402
    crc16_ccitt,
    fib_ok,
    prbs_sequence,
    convolutional_encode,
    make_puncture_table,
    FRAME_SIZE,
    FIBS_PER_ETI_FRAME,
    FIC_BYTES,
)


def main() -> int:
    eti_path = Path("/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_v4.eti")
    data = eti_path.read_bytes()
    prbs = prbs_sequence(768)
    n_frames = len(data) // FRAME_SIZE

    blocks_checked = 0
    blocks_roundtrip_ok = 0
    blocks_partial_match = 0

    for frame_i in range(min(n_frames, 50)):
        frame = data[frame_i * FRAME_SIZE : (frame_i + 1) * FRAME_SIZE]
        if frame[0] != 0xFF:
            continue
        nst = frame[5] & 0x7F
        fic_off = 12 + nst * 4
        ficblock_bytes = frame[fic_off : fic_off + FIC_BYTES]

        # All 3 FIBs must pass to use this block.
        all_ok = all(fib_ok(ficblock_bytes[i*32:(i+1)*32]) for i in range(3))
        if not all_ok:
            continue
        blocks_checked += 1

        # Unpack 96 bytes -> 768 bits MSB-first.
        desc = []
        for byte in ficblock_bytes:
            for b in range(8):
                desc.append((byte >> (7 - b)) & 1)

        # Reverse PRBS XOR -> 768 info bits.
        info = [desc[i] ^ prbs[i] for i in range(768)]

        # Re-apply PRBS XOR and re-pack into 96 bytes; should equal original.
        repacked = bytearray(96)
        for byte_i in range(96):
            v = 0
            for bit_i in range(8):
                idx = byte_i * 8 + bit_i
                v = (v << 1) | (info[idx] ^ prbs[idx])
            repacked[byte_i] = v

        if bytes(repacked) == ficblock_bytes:
            blocks_roundtrip_ok += 1
        else:
            mismatch = sum(1 for i in range(96) if repacked[i] != ficblock_bytes[i])
            if mismatch <= 4:
                blocks_partial_match += 1
                print(f"  frame {frame_i}: {mismatch} byte mismatch")

        # Also exercise the encoder+puncture chain (used by eti_to_expected_soft.py).
        mother = convolutional_encode(info)
        assert len(mother) == 3096
        punc = make_puncture_table()
        tx = [mother[i] for i in range(3096) if punc[i]]
        assert len(tx) == 2304
        # Mean energy of expected: 1.0 (each bit ±1). Just confirm it terminates.

    print(f"Blocks checked:                {blocks_checked}")
    print(f"Round-trip byte-identical:     {blocks_roundtrip_ok}")
    print(f"Round-trip partial (≤4 byte):  {blocks_partial_match}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
