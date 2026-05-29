#!/usr/bin/env python3
"""Slice-23 Part A: independently compute CP autocorr on cf32 stream to
verify dab-rs cp.estimate_cfo_hz output and find where the 2x factor lives.

Math:
    For OFDM symbol at stream position d:
        a = iq[d + k]            (CP sample, k ∈ [0, T_g))
        b = iq[d + T_u + k]      (last T_g samples of useful = CP twin)
        r += conj(a) * b
    arg(r) = 2π · X · T_u / fs   →   X = arg(r) / (2π) · carrier_diff
"""
import math
import struct
import sys

T_G = 504
T_U = 2048
T_S = T_G + T_U  # 2552
FS = 2_048_000.0

def read_cf32(path):
    raw = open(path, 'rb').read()
    n = len(raw) // 8
    samples = struct.unpack(f'<{2*n}f', raw)
    return [complex(samples[2*i], samples[2*i+1]) for i in range(n)]

def cp_autocorr(iq, s0, n_syms):
    acc = 0j
    for j in range(n_syms):
        d = s0 + j * T_S
        if d + T_U + T_G > len(iq): break
        for k in range(T_G):
            a = iq[d + k]
            b = iq[d + T_U + k]
            acc += a.conjugate() * b
    arg = math.atan2(acc.imag, acc.real)
    return arg, abs(acc), arg / (2*math.pi) * 1000.0

# Also: alternative form (eti-stuff style — sum over useful tail)
# Per eti-stuff main loop:
#   for i in T_u..T_s:
#     FreqCorr += ofdmBuffer[i] * conj(ofdmBuffer[i - T_u])
# i.e., late * conj(early). dab-rs uses conj(early) * late = same.
def cp_autocorr_eti_style(iq, s0, n_syms):
    acc = 0j
    for j in range(n_syms):
        d = s0 + j * T_S
        if d + T_S > len(iq): break
        for i in range(T_U, T_S):
            a = iq[d + i]            # ofdmBuffer[i] — index i in symbol
            b = iq[d + i - T_U]      # ofdmBuffer[i - T_u]
            acc += a * b.conjugate()
    arg = math.atan2(acc.imag, acc.real)
    return arg, abs(acc), arg / (2*math.pi) * 1000.0

if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sim5_resampled.cf32'
    print(f'reading {path} ...')
    iq = read_cf32(path)
    print(f'  {len(iq)} samples = {len(iq)/FS:.2f}s')

    # Use the known prs_start of dab-rs's first stable lock: frame 3 in sim5
    # was prs_start = 387242 - 504 = 386738 (cp_start). Or just try several s0.
    for s0 in [386738, 387000, 1000000, 2000000, 5000000]:
        if s0 + 50 * T_S > len(iq):
            continue
        arg_d, mag_d, x_d = cp_autocorr(iq, s0, 50)
        arg_e, mag_e, x_e = cp_autocorr_eti_style(iq, s0, 50)
        print(f's0={s0:>8}:')
        print(f'  dab-rs form (conj(CP) * useful_tail): X = {x_d:+.2f} Hz '
              f'(arg={arg_d:+.4f}, mag={mag_d:.0f})')
        print(f'  eti-stuff form (useful_tail * conj(CP)): X = {x_e:+.2f} Hz '
              f'(arg={arg_e:+.4f}, mag={mag_e:.0f})')

    # Now: what if we measure CFO using only PRS region + first FIC sym,
    # i.e., compute autocorrelation between two specific symbols T_u apart?
    print('\n--- single-symbol CP autocorr at sim5 frame 3 PRS (prs_start ≈ 386738) ---')
    s0 = 386738
    for j in [0, 1, 2, 3]:
        d = s0 + j * T_S
        acc = 0j
        for k in range(T_G):
            a = iq[d + k]
            b = iq[d + T_U + k]
            acc += a.conjugate() * b
        arg = math.atan2(acc.imag, acc.real)
        X = arg / (2*math.pi) * 1000.0
        print(f'  sym j={j}: X = {X:+.2f} Hz   |acc|={abs(acc):.0f}')

    # Sanity: also compute mean over MANY symbols at known stable locks
    # (skip first 5 frames to avoid airspy startup transient).
    print('\n--- per-frame X over the full file (every 100k samples) ---')
    print(f'{"s0":>10}  {"X dab-form":>12}  {"X eti-form":>12}')
    for s0 in range(2_000_000, min(28_000_000, len(iq) - 50*T_S), 1_000_000):
        _, _, x_d = cp_autocorr(iq, s0, 50)
        _, _, x_e = cp_autocorr_eti_style(iq, s0, 50)
        print(f'  {s0:>8}  {x_d:>+12.2f}  {x_e:>+12.2f}')
