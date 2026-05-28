# dab-rs ↔ eti-stuff cross-validation diagnostics

This directory documents the manual workflow for diagnosing where `dab-rs`'s
OFDM chain diverges from the reference C++ implementation
[`eti-stuff`](https://github.com/JvanKatwijk/eti-stuff). It is the
infrastructure used to investigate Stage B/C correctness — see the project
README's *Discovered subtleties* gotcha #7 (`dab-viterbi` self-consistency
without real-signal validation) and the open Week 3e investigation.

## What the diagnostic captures

`eti-stuff` runs `ofdmProcessor::processBlock` once per OFDM data symbol; its
output is a length-`2*K = 3072` `int16_t ibits[]` buffer (the soft bits sent
to the Viterbi decoder). The patch in
[`eti-stuff-ibits-dump.patch`](./eti-stuff-ibits-dump.patch) adds an
env-var-gated dump at exactly that point. `dab-cli` ships matching
subcommands that drive the comparison from the dab-rs side.

## End-to-end workflow

```bash
# 1. Apply the patch to a local checkout of eti-stuff:
cd /path/to/airspy-mini-dmb/eti-stuff
git apply /path/to/dab-rs/docs/diag/eti-stuff-ibits-dump.patch
# (No CMake change needed — the patch is env-gated, off by default.)

# 2. Build the rawfiles variant:
cd eti-cmdline/build-rawfiles
cmake ..  -DRAWFILES=ON      # only needed on first build
make

# 3a. Resample to either CU8 @ 2.048 MSPS (--out cu8) or 16-bit PCM
#     stereo WAV @ 2.048 MSPS (--out wav, default). WAV preserves the
#     full source precision; CU8 is more compact but lossy at low SNR.
cd /path/to/dab-rs
./target/release/dab convert-iq           \
    /path/to/k8b_v4.iq                    \
    /tmp/k8b_v4_2048k.wav                 \
    --out wav     # or --out cu8

# 3b. (CU8 path) Run eti-cmdline-rawfiles with one or more dump env-vars:
DAB_RS_DIAG_DUMP=/tmp/oracle_ibits.bin                                 \
DAB_RS_DIAG_DUMP_FFT=/tmp/oracle_fft.bin                               \
DAB_RS_DIAG_DUMP_PREFFT=/tmp/oracle_prefft.bin                         \
DAB_RS_DIAG_DUMP_META=/tmp/oracle_meta.bin                             \
  /path/to/eti-stuff/eti-cmdline/build-rawfiles/eti-cmdline-rawfiles   \
  -F /tmp/k8b_v4_2048k.cu8                                             \
  -O /tmp/oracle_k8b_v4.eti                                            \
  -D 15 -t 22

# 3c. (WAV path) Same dump env-vars, but the WAVFILES build:
#     (build first with: cd build-wavfiles && cmake -DWAVFILES=ON \
#      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. && make)
DAB_RS_DIAG_DUMP=/tmp/oracle_wav_ibits.bin                             \
DAB_RS_DIAG_DUMP_FFT=/tmp/oracle_wav_fft.bin                           \
DAB_RS_DIAG_DUMP_PREFFT=/tmp/oracle_wav_prefft.bin                     \
DAB_RS_DIAG_DUMP_META=/tmp/oracle_wav_meta.bin                         \
  /path/to/eti-stuff/eti-cmdline/build-wavfiles/eti-cmdline-wavfiles   \
  -F /tmp/k8b_v4_2048k.wav                                             \
  -O /tmp/oracle_wav.eti                                               \
  -D 15 -t 22

# 5. Cross-validate with dab-rs:
./target/release/dab diag-ibits     \
    /tmp/k8b_v4_2048k.cu8           \
    /tmp/oracle_ibits.bin           \
    --ingest cu8 --frames 30

# 6. Drill into a specific (frame, symbol) record:
./target/release/dab diag-pair      \
    /tmp/k8b_v4_2048k.cu8           \
    /tmp/oracle_ibits.bin           \
    --ingest cu8 --frame 5 --symbol 5 --show 16
```

## Dump format

The patch adds four independent env-var-gated dump channels. They share
the same per-symbol record cadence and `(frame_idx, ofdm_symbol_idx)`
header but carry different payloads. Enable as many or as few as you
need by setting the matching env var to a writable file path before
running `eti-cmdline-rawfiles`.

| Env var                       | Channel | Payload (after the 8-byte header)                 | Bytes/record | When written |
| ----------------------------- | ------- | ------------------------------------------------- | ------------ | ------------ |
| `DAB_RS_DIAG_DUMP`            | ibits   | `i16[3072]` — Stage 7 demap soft bits (±127).     | 6 152        | every data symbol (`ofdmSymbolCount` 2..76) |
| `DAB_RS_DIAG_DUMP_FFT`        | fft     | `complex<float>[2048]` — `fft_buffer` *after* `do_FFT`, *before* the differential. | 16 392 | every data symbol |
| `DAB_RS_DIAG_DUMP_PREFFT`     | prefft  | `complex<float>[2048]` — `ofdmBuffer[T_g..T_g+T_u]`, the time-domain useful part going into the FFT. | 16 392 | every data symbol |
| `DAB_RS_DIAG_DUMP_META`       | meta    | `u32 frame_idx, i16 fineCorrector, i16 _pad, i32 coarseCorrector` (12 bytes, no `ofdm_symbol_idx`). | 12 | once per frame (when `ofdmSymbolCount==2`) |

All multi-byte fields are little-endian. The meta channel is `fflush`'d
each frame so it's visible to readers mid-run; the per-symbol channels
let libc buffer them and rely on `fclose` at process exit.

`fft_buffer` is *not* mutated by `processBlock`'s per-carrier demap
loop (`r1 = fft_buffer[idx] * conj(referenceFase[idx])` only reads
`fft_buffer`), so dumping immediately after `processBlock` returns
captures the pre-differential FFT output verbatim. The header
`frame_idx` increments on every `ofdmSymbolCount==2`, so all four
channels share the same frame numbering.

## First-iteration findings (2026-05-28)

Running the workflow on `k8b_v4.iq` (the K8B oracle capture):

- **eti-stuff failed to lock** on the CU8-quantised input (12 dB SNR
  capture + ~−48 dB quantisation noise was apparently below the binary's
  FIB-CRC threshold) — `oracle_k8b_v4.eti` is 0 bytes despite 11 607
  per-symbol records being dumped.
- The `dab fic-iq` pipeline also produces 0 valid FIBs on the same
  capture (the Week 3e starting point).
- **dab-rs vs oracle ibits match rate = 0.42 %** = random baseline
  (1/255). Same input, totally uncorrelated outputs.
- **Multiset equality on a single (frame, symbol) record: false.** The
  values themselves are different — this is not a freq-de-interleaver
  permutation, an I/Q swap, a sign flip, or a small bin shift. All of
  those hypotheses tested at ≈ 0.4 % match.
- **Statistical distributions match exactly**: oracle and dab-rs both
  give mean `|b|` = 63/127 and max `|b|` = 126 across many frames. The
  chains are producing the *same kind* of soft bits, just different
  values at the same positions.

That combination — identical distributions, uncorrelated values — points
upstream of the demap. Most likely candidates: fractional CFO
estimation, PRS start-position sub-sample alignment, or the integer-CFO
decision (which `detect_integer_cfo` makes with a peak/runner-up
confidence guard, while eti-stuff's `phaseSynchronizer::estimateOffset`
uses adjacent-carrier phase differences). A small persistent residual
CFO rotates every consecutive-symbol differential by a fixed angle,
which produces exactly this "right distribution, wrong values"
signature.

## Second-iteration findings (2026-05-28, P3 + P1 + P4 dumps added)

With the meta + fft + prefft channels enabled and the same K8B
`k8b_v4_2048k.cu8` input, the META dump reveals **why** the ibits
comparison gave random-baseline match rate: *the oracle never reaches a
stable CFO lock on the CU8-quantised input.* `coarseCorrector` (per
ETSI §, applied via NCO in time domain, units = Hz) jumps wildly
frame-to-frame and is repeatedly reset by the
`abs(coarseCorrector) > Khz(35)` guard:

| frame | fineCorrector (Hz) | coarseCorrector (Hz) |
| ----- | ------------------ | -------------------- |
| 1     | 0                  | −9 000               |
| 2     | 47                 | +17 000              |
| 3     | 52                 | 0 (reset)            |
| 5     | 43                 | −10 000              |
| 8     | −47                | +20 000              |
| 15    | −121               | −25 000              |
| 28    | −265               | −33 000              |
| 30    | −354               | +25 000              |
| 151   | −192               | −23 000              |
| 155   | −235               | +10 000              |

`fineCorrector` (the fractional residual) drifts in a *somewhat* sane
way — settles around −200 Hz after ~30 frames — but `coarseCorrector`
implies the oracle thinks the integer-carrier CFO swings by ±25 carriers
every other frame. Each such swing reads a different set of FFT bins for
the same logical carrier indices, so the post-differential soft bits get
scrambled differently on every frame.

That explains the diag-ibits histogram: *both pipelines see the same
input, but the oracle is processing it from a different bin offset on
every frame*. dab-rs's `detect_integer_cfo` confidence guard
(`peak > 1.5 × runner_up`) keeps it at `δ = 0` when the correlation is
noisy, which on this input is most of the time. The two pipelines are
effectively reading carriers at different bin positions and producing
uncorrelated ibits as a consequence.

Equally important: the **oracle's coarse-CFO loop is in trouble**, not
dab-rs's. Stage A above also failed (0-byte ETI), which is consistent
with the unstable coarseCorrector. So this particular comparison oracle
is **not a reliable byte-identical reference for the cu8 path**. Two
follow-up paths to consider:

1. **Give the oracle a better input.** The CU8 quantisation drops the
   bottom 8 bits of dynamic range. At our marginal 12 dB SNR that may
   sit right at the lock threshold. Options: pre-scale the f32 samples
   before quantising so the per-sample peak fills more of the byte
   range, or switch to a libsndfile-readable 16-bit WAV (eti-stuff's
   `WAVFILES=ON` build) so we keep the full Cs16 precision.
2. **Pick a different comparison surface.** Within a single frame,
   coarse + fine corrector are constant, so the *within-frame*
   differential demap is still informative — just don't expect the
   *between-frame* state (and therefore the absolute soft-bit values
   placed onto specific carriers) to match. A comparator that aligns
   on band-energy structure rather than per-position equality would be
   more robust against this kind of oracle instability.

The dab-rs `dab fic-iq` pipeline produces 0 / 2496 valid FIBs on the
*same* `k8b_v4.iq` capture even when its OFDM black-box test passes —
so dab-rs may have its own bugs too. But the oracle dump no longer
serves as a clean ground truth on this input, and the next slice has
to decide which fork to take above before more comparison work is
useful.

## Third-iteration findings (2026-05-28, WAV path + scale audit)

The cu8 path's failure motivated trying eti-stuff's `WAVFILES=ON` build
so the source `Cs16Le` precision could survive the round-trip through
libsndfile (16-bit PCM @ 2.048 MSPS). `dab convert-iq --out wav`
produces the file, `eti-cmdline-wavfiles` reads it. End result on
`k8b_v4.iq`:

- WAV file generated: 164 MiB, valid RIFF/WAVE header verified by `xxd`.
- `eti-cmdline-wavfiles` *still* fails to lock — 0-byte ETI, same
  `coarseCorrector` instability signature as the cu8 path
  (`coarseCorrector` stdev ≈ 17 000, range [−33 k, +33 k]).

That ruled out cu8 quantisation as the root cause. The real problem
sits one level deeper: **input-amplitude scale mismatch** between
eti-stuff's online and offline input handlers.

```
eti-cmdline-airspy:    airspy-handler.cpp:337
                       sample / 2048   (12-bit raw → ~[−1, +1])
eti-cmdline-rawfiles:  rawfile-handler.cpp:98
                       (sample - 128) / 128   (CU8 mid-zero → [−1, +1])
eti-cmdline-wavfiles:  wavfile-handler.cpp via sf_readf_float
                       libsndfile divides 16-bit PCM by 32768  → ~[−1/16, +1/16] on our data
```

The K8B capture's `Cs16Le` values measured: `p50=1807, p99=7018,
max=12940`, i.e. **~21 %** of the 16-bit range. That sits awkwardly
in the gap between airspy-handler's `/2048` (would produce ~±6.3,
heavy saturation) and libsndfile's `/32768` (produces ~±0.21, 16× too
small relative to airspy-handler-equivalent OFDM input). The live
`eti-cmdline-airspy` run that produced `k8b_v4.eti` saw airspy-handler
scaling; both offline handlers feed the OFDM processor at a *different*
amplitude. If even one early-sync stage in eti-stuff has an absolute
amplitude threshold (the null detector is the obvious suspect), this
single-stage mismatch can cascade into the coarseCorrector instability
we see in every meta dump.

**Bit-usage measurement** (Step 1 of the slice-3 plan), confirming the
21 % figure:

```
$ python3  # over 2 M IQ half-samples from k8b_v4.iq, cs16le @ 3 MSPS
mean   2150.9    rms   2707.1
p50    1807      p90   4463
p99    7018      max  12940
```

(Same script reused — runs in 5 s.)

**Resampler spec** (Step 2 of the v2 plan): the dab-rs polyphase
resampler is `L = 256, M = 375` (exact 256/375 = 2.048M / 3.000M),
**16 taps per phase**, **Blackman-windowed sinc** prototype of length
`16·256 + 1 = 4097`, cutoff `1/M` (cycles/sample at upsampled rate to
honour the anti-aliasing edge). Blackman stops at ~−58 dB sidelobes,
which is weaker than scipy's default Kaiser β=14 (−126 dB) but
comfortably above the K8B 12 dB SNR floor. Linear-phase, group delay
`(4097 − 1)/2 = 2048` upsampled samples ≈ 5.46 input samples.

The resampler is run *once* by `dab convert-iq` and its output is
consumed identically by both pipelines via the WAV file, so it cannot
itself be a source of dab-rs vs oracle divergence.

## Next-slice fork

Three concrete options, in order of preference:

1. **32-bit float WAV with airspy-handler-equivalent scaling.** Write
   `SF_FORMAT_WAV | SF_FORMAT_FLOAT` with samples pre-scaled to match
   airspy-handler's `/2048` output (no 16-bit-PCM clipping issue since
   floats can carry values > 1). Most likely to make the offline oracle
   behave the same as the live one.
2. **Patch `wavfile-handler.cpp` to multiply by 16 after libsndfile**
   reads, instead of changing the writer. Equivalent in effect, larger
   change to eti-stuff.
3. **Use `dab fic-iq` on the WAV input directly.** Skip the oracle
   altogether for the byte-identical step; instead validate
   dab-rs against the live `k8b_v4.eti` by feeding the same WAV through
   our pipeline and comparing the FIB CRC pass rate or ensemble model
   to the live capture's metadata.

The infrastructure (patched eti-stuff with four dump channels,
`dab convert-iq --out wav`, `dab diag-ibits`, `dab diag-pair`) is all
in place — the next slice picks one of the three forks and exercises it.

## Closing the patch

The patch is intentionally minimal so it stays a small set of hunks
against a clean eti-stuff tree. Add additional dump hooks alongside
the existing ones as needed for follow-up investigations.
