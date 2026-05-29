#!/usr/bin/env python3
"""Slice-20 Part A: dab-rs FFT pre-NCO samples vs cf32 file at the same
useful_start. If they match byte-for-byte (modulo float noise), the cf32
reader is correct and the bug is in NCO / FFT / spectral processing.
If they differ, the reader is buggy — that's the root cause.

Record (DAB_RS_DUMP_FFT_INPUT):
    u32 frame, u32 sym, u64 useful_start,
    2048 × (f32 re, f32 im) pre-NCO,
    2048 × (f32 re, f32 im) post-NCO.
"""
from __future__ import annotations
import math, struct, sys
from pathlib import Path

T_U = 2048
REC_SZ = 4 + 4 + 8 + T_U * 8 + T_U * 8  # 32784

def main():
    cf32_path  = '/tmp/sim5_resampled.cf32'
    dump_path  = '/tmp/sim5_fftinput_dab.bin'
    cf32 = open(cf32_path, 'rb').read()
    dump = open(dump_path, 'rb').read()
    n_rec = len(dump) // REC_SZ
    print(f'records: {n_rec}, file: {len(cf32)} bytes ({len(cf32)//8} samples)')

    print('\n=== PRE-NCO vs cf32 file (byte/numeric equality) ===')
    print(f'{"rec":>4}  {"frame":>5} {"sym":>4}  {"useful":>10}  '
          f'{"first_diff":>11}  {"max_abs_diff":>13}  {"mean_abs_diff":>14}')
    for i in range(min(n_rec, 40)):
        off = i * REC_SZ
        frame, sym, useful = struct.unpack_from('<IIQ', dump, off)
        pre = dump[off + 16 : off + 16 + T_U * 8]
        # cf32 file's 2048 samples starting at useful_start
        file_off = useful * 8
        if file_off + T_U * 8 > len(cf32):
            continue
        file_samples = cf32[file_off : file_off + T_U * 8]
        # Compare as f32 sequences
        pre_f = struct.unpack(f'<{2*T_U}f', pre)
        file_f = struct.unpack(f'<{2*T_U}f', file_samples)
        # Find first diff
        first_diff = -1
        max_diff = 0.0
        sum_diff = 0.0
        for j in range(2 * T_U):
            d = abs(pre_f[j] - file_f[j])
            if d > max_diff:
                max_diff = d
            sum_diff += d
            if first_diff < 0 and d > 1e-9:
                first_diff = j
        mean_diff = sum_diff / (2 * T_U)
        print(f'{i:>4}  {frame:>5} {sym:>4}  {useful:>10}  '
              f'{first_diff:>11}  {max_diff:>13.2e}  {mean_diff:>14.2e}')

    print('\n=== POST-NCO vs PRE-NCO ratio (should be smooth exp(-j·2π·f·n/fs)) ===')
    # Take one record and compute angle(post[n]/pre[n]) over n
    for i in [3]:  # frame 1 sym 4 typically
        off = i * REC_SZ
        frame, sym, useful = struct.unpack_from('<IIQ', dump, off)
        pre = struct.unpack(f'<{2*T_U}f', dump[off + 16 : off + 16 + T_U * 8])
        post = struct.unpack(f'<{2*T_U}f', dump[off + 16 + T_U*8 : off + 16 + 2*T_U*8])
        # Compute angles at a few sample indices
        print(f'rec {i} frame {frame} sym {sym} useful={useful}')
        # angle(post/pre) ≈ -2π·cfo·n/fs accumulated phase
        prev_phase = None
        slopes = []
        for n in [0, 100, 500, 1000, 1500, 2000]:
            pre_z = complex(pre[2*n], pre[2*n+1])
            post_z = complex(post[2*n], post[2*n+1])
            if abs(pre_z) < 1e-9 or abs(post_z) < 1e-9:
                continue
            ratio = post_z / pre_z
            ph = math.atan2(ratio.imag, ratio.real)
            print(f'  n={n}: |ratio|={abs(ratio):.4f}  arg={ph:+.4f} rad')

if __name__ == '__main__':
    sys.exit(main())
