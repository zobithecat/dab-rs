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

# 3. Resample + quantise an existing Cs16Le @ 3 MSPS capture into the CU8
#    @ 2.048 MSPS format the rawfiles binary consumes:
cd /path/to/dab-rs
./target/release/dab convert-iq  \
    /path/to/k8b_v4.iq           \
    /tmp/k8b_v4_2048k.cu8

# 4. Run eti-cmdline-rawfiles with the env-var pointing at a dump path:
DAB_RS_DIAG_DUMP=/tmp/oracle_ibits.bin                                 \
  /path/to/eti-stuff/eti-cmdline/build-rawfiles/eti-cmdline-rawfiles   \
  -F /tmp/k8b_v4_2048k.cu8                                             \
  -O /tmp/oracle_k8b_v4.eti                                            \
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

Per OFDM data symbol, one record of `8 + 2 * 3072 = 6152` bytes:

| Field             | Size                 | Description                                        |
| ----------------- | -------------------- | -------------------------------------------------- |
| `frame_idx`       | `u32` (little-endian) | 1-based; increments on every `ofdmSymbolCount==2`. |
| `ofdm_symbol_idx` | `u32` (little-endian) | 2..=76 (Mode I: 75 data symbols per frame).        |
| `ibits[3072]`     | `i16` × 3072 (LE)     | 1536 I-bits at `[0..1536)`, 1536 Q-bits at `[1536..3072)`, range ±127. |

The eti-stuff side writes one record per `processBlock` invocation. Each
record is the raw oracle soft-bit output that ordinarily feeds
`viterbiSpiral` via the FIC / EEP / UEP protection front-ends.

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

## Next steps

To localise the divergence further, additional dump points are needed:

- Per-frame: PRS start sample index, fractional CFO estimate (in Hz),
  integer CFO offset (in carriers).
- Per data symbol: the FFT output (`fft_buffer[T_u]`) before the
  differential, so dab-rs's `SymbolFft::fft_symbol` output can be
  compared bin-for-bin.

The patch is intentionally minimal so it stays a single hunk against a
clean eti-stuff tree. Add additional dump hooks alongside the existing
one as needed for follow-up investigations.
