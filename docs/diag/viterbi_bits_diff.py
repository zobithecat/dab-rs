#!/usr/bin/env python3
"""Bit-by-bit Viterbi-output / descrambled-bits diff: dab-rs vs live ETI.

Driven by slice 6 of the Week 3e investigation. dab-rs's full chain
produces 0 / 2496 valid FIBs while the live ETI shows 75.0 % (7517 / 10024)
on the same K8B capture. The OFDM Stage 1–7 is already known to be
functionally correct (208 / 208 frames decoded, healthy soft-bit stats);
the 100-point gap must be downstream — in dab-viterbi, dab-descramble, or
the FIB packing. This script localises the bug by comparing each chain
intermediate to the live ETI's FIB bytes.

Inputs
------
- ``--eti``: live ETI capture (`k8b_v4.eti`).
- ``--viterbi``: dab-rs's pre-descramble Viterbi-output dump produced by
  ``DAB_RS_DUMP_VITERBI_OUT=path dab fic-iq``. Per frame:
  ``u32 LE frame_idx, 3072 bytes (bit-per-byte: 4 ficBlocks × 768 bits)``.
- ``--descrambled``: dab-rs's post-descramble dump, same layout.

ETI(NI) Mode I frame layout (slice 6 Part A finding)
----------------------------------------------------
24 ms ETI(NI) frame contains 4 FIB slots; slots 0–2 are real, slot 3 is
always-fail padding. 4 ETI frames per 96 ms DAB frame ⇒ 4 × 3 = 12 real
FIBs per DAB frame ⇒ matches dab-rs's 12-FIB-per-frame output exactly.

Mapping: dab-rs DAB frame M, FIB[k] for k = 0..11 ↔
         live ETI frame [base_offset + M*4 + k // 3], slot [k % 3].

The script sweeps a small ``base_offset`` search to absorb sync-acquisition
skew (oracle and dab-rs typically lock on different first frames).

Output
------
For each of {viterbi_out, descrambled}:
- best base_offset and exact-match rate at that offset
- diff histogram over each FIB's 256 bits
- per-bit-position diff rate across all aligned FIBs (highlights
  systematic offsets like off-by-one, polarity flip on every Nth bit, etc.)

The fix follows the divergence:
- viterbi_out matches but descrambled doesn't → descrambler PRBS bug
- neither matches at any offset → likely a Viterbi convention mismatch
  (gotcha #7's primary suspicion)
- both match modulo a uniform sign flip → soft-bit polarity
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

FRAME_SIZE = 6144
FIB_SIZE = 32
FIC_FIBS = 4
FIB_BITS = 256
FIBS_PER_DAB = 12          # 4 ETI frames × 3 real slots
ETI_FRAMES_PER_DAB = 4
DAB_RS_HEADER = 4
DAB_RS_BITS_PER_FRAME = 4 * 768   # 3072 bit-per-byte
DAB_RS_FRAME_BYTES = DAB_RS_HEADER + DAB_RS_BITS_PER_FRAME


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


def bytes_to_bits(buf: bytes) -> list[int]:
    """Unpack MSB-first into a list of 0/1 bits — matches dab-descramble's
    ``descramble_and_pack``."""
    out: list[int] = []
    for b in buf:
        for k in range(7, -1, -1):
            out.append((b >> k) & 1)
    return out


def read_live_eti_fibs(eti_path: Path) -> list[list[int]]:
    """Returns a flat list of 256-bit arrays, one per *real* FIB
    (skipping slot 3 in every ETI frame). The list length divided by 3
    is the number of ETI frames; divided by 12 is the number of DAB
    frames."""
    fibs: list[list[int]] = []
    with eti_path.open("rb") as f:
        while True:
            frame = f.read(FRAME_SIZE)
            if len(frame) < FRAME_SIZE:
                break
            ficf = (frame[5] >> 7) & 1
            nst = frame[5] & 0x7F
            if not ficf:
                continue
            off = 4 + 4 + 4 * nst + 4  # SYNC + FC + STC + EOH
            for slot in range(FIC_FIBS - 1):  # 0, 1, 2 — skip the padding slot
                fib = frame[off + slot * FIB_SIZE : off + (slot + 1) * FIB_SIZE]
                fibs.append(bytes_to_bits(fib))
    return fibs


def read_dab_rs_frames(dump_path: Path) -> list[tuple[int, list[int]]]:
    """Returns ``[(frame_idx, [bit; 3072]), ...]`` — one tuple per DAB
    frame in the dump. Each inner list spans 4 ficBlocks × 768 bits."""
    out: list[tuple[int, list[int]]] = []
    raw = dump_path.read_bytes()
    n_frames = len(raw) // DAB_RS_FRAME_BYTES
    for i in range(n_frames):
        off = i * DAB_RS_FRAME_BYTES
        frame_idx = struct.unpack_from("<I", raw, off)[0]
        bits = list(raw[off + DAB_RS_HEADER : off + DAB_RS_FRAME_BYTES])
        out.append((frame_idx, bits))
    return out


def slice_dab_rs_fib(frame_bits: list[int], k: int) -> list[int]:
    """Extract FIB k (0..11) from a DAB frame's 3072-bit payload, in the
    same order dab-rs's ``descramble_and_pack`` writes them: ficBlock
    K = k // 3 occupies bits [K*768 .. (K+1)*768); FIB j = k % 3 within
    that ficBlock occupies bits [j*256 .. (j+1)*256)."""
    fic_block = k // 3
    j = k % 3
    start = fic_block * 768 + j * FIB_BITS
    return frame_bits[start : start + FIB_BITS]


def diff_count(a: list[int], b: list[int]) -> int:
    return sum(1 for x, y in zip(a, b) if x != y)


def compare(
    eti_fibs: list[list[int]],
    dab_rs_frames: list[tuple[int, list[int]]],
    label: str,
    offset_range: int = 20,
) -> None:
    """Sweep `base_offset` in [-offset_range, +offset_range] DAB frames
    and report the alignment that maximises exact-bit matches across
    all comparable FIBs."""
    n_dab = len(dab_rs_frames)
    if n_dab == 0 or not eti_fibs:
        print(f"[{label}] empty input (eti={len(eti_fibs)} fibs, dab_rs={n_dab} frames)")
        return

    best_offset = 0
    best_match = 0
    best_total = 0
    for offset in range(-offset_range, offset_range + 1):
        match = 0
        total = 0
        for m in range(n_dab):
            base = (m + offset) * 3 * ETI_FRAMES_PER_DAB
            if base < 0 or base + 12 > len(eti_fibs):
                continue
            _, dab_bits = dab_rs_frames[m]
            for k in range(FIBS_PER_DAB):
                ref = eti_fibs[base + k]
                got = slice_dab_rs_fib(dab_bits, k)
                match += sum(1 for x, y in zip(got, ref) if x == y)
                total += FIB_BITS
        if match > best_match:
            best_match = match
            best_offset = offset
            best_total = total

    if best_total == 0:
        print(f"[{label}] no overlap at any offset in [{-offset_range}, {offset_range}]")
        return

    rate = best_match / best_total
    print(
        f"[{label}] best_offset={best_offset} (in DAB frames)  "
        f"bit_match={best_match}/{best_total} = {rate:.4f}  "
        f"baseline=0.5 (random)"
    )

    # Per-bit-position diff rate at the winning offset.
    counts = [0] * FIB_BITS
    n_fibs = 0
    for m in range(n_dab):
        base = (m + best_offset) * 3 * ETI_FRAMES_PER_DAB
        if base < 0 or base + 12 > len(eti_fibs):
            continue
        _, dab_bits = dab_rs_frames[m]
        for k in range(FIBS_PER_DAB):
            ref = eti_fibs[base + k]
            got = slice_dab_rs_fib(dab_bits, k)
            for i in range(FIB_BITS):
                if got[i] != ref[i]:
                    counts[i] += 1
            n_fibs += 1

    print(f"[{label}] per-bit-position diff rate over {n_fibs} aligned FIBs:")
    print("    bit-position bucket  diff-rate")
    bucket = 32
    for start in range(0, FIB_BITS, bucket):
        end = start + bucket
        d = sum(counts[start:end])
        rate = d / (n_fibs * bucket)
        bar = "#" * int(rate * 40)
        print(f"      {start:3d}..{end-1:3d}  {rate:6.4f}  {bar}")

    # Spot-check: show first FIB side-by-side, hex.
    if n_dab > 0 and len(eti_fibs) > 12 + best_offset * 12:
        base = best_offset * 3 * ETI_FRAMES_PER_DAB if best_offset >= 0 else 0
        if 0 <= base and base + 1 <= len(eti_fibs):
            _, dab_bits = dab_rs_frames[max(0, -best_offset)]
            got = slice_dab_rs_fib(dab_bits, 0)
            ref = eti_fibs[base]
            print(f"[{label}] first aligned FIB (DAB frame {-best_offset if best_offset < 0 else 0}, "
                  f"ETI fib_idx {base}):")
            print(f"    dab_rs: {''.join(map(str, got[:64]))}...")
            print(f"    live:   {''.join(map(str, ref[:64]))}...")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--eti", type=Path, required=True, help="live ETI capture")
    ap.add_argument("--viterbi", type=Path, help="dab-rs viterbi-out dump (pre-descramble)")
    ap.add_argument("--descrambled", type=Path, help="dab-rs descrambled-bits dump")
    ap.add_argument("--offset-range", type=int, default=20)
    args = ap.parse_args()

    print(f"reading live ETI {args.eti} …")
    eti_fibs = read_live_eti_fibs(args.eti)
    print(f"  found {len(eti_fibs)} real FIBs "
          f"({len(eti_fibs) // 3} ETI frames × 3 real slots)")

    if args.viterbi:
        print(f"\nreading dab-rs viterbi-out {args.viterbi} …")
        dab_rs = read_dab_rs_frames(args.viterbi)
        print(f"  found {len(dab_rs)} DAB frames")
        compare(eti_fibs, dab_rs, "viterbi_out", args.offset_range)

    if args.descrambled:
        print(f"\nreading dab-rs descrambled {args.descrambled} …")
        dab_rs = read_dab_rs_frames(args.descrambled)
        print(f"  found {len(dab_rs)} DAB frames")
        compare(eti_fibs, dab_rs, "descrambled", args.offset_range)

    return 0


if __name__ == "__main__":
    sys.exit(main())
