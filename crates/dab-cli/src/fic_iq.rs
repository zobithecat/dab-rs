//! `dab fic-iq` pipeline — raw I/Q → OFDM demod → FIC bits → Ensemble.
//!
//! This is the first end-to-end orchestrator in `dab-rs`. It wires Stages
//! 1–7 of `dab-ofdm` together with `dab-viterbi::FicProtection`,
//! `dab-descramble`, and `dab-fic::FicAccumulator` so the FIC path is
//! exercised against raw I/Q without going through a recorded ETI file.
//!
//! # Per-frame algorithm
//!
//! For every DAB Mode I frame located in the resampled stream:
//!
//! 1. Locate the null symbol's start (already done globally by
//!    [`NullDetector`](dab_ofdm::NullDetector)).
//! 2. Refine the PRS start with the CP autocorrelator
//!    ([`CpSync::fine_time`](dab_ofdm::CpSync::fine_time)).
//! 3. Estimate the fractional carrier-frequency offset
//!    ([`CpSync::estimate_cfo_hz`](dab_ofdm::CpSync::estimate_cfo_hz)).
//! 4. FFT the PRS with fractional-CFO removal; detect the integer carrier
//!    offset `δ`
//!    ([`detect_integer_cfo`](dab_ofdm::detect_integer_cfo)).
//! 5. For each OFDM symbol of the frame:
//!    - Strip CP and FFT.
//!    - Rotate the spectrum left by `δ` to undo the integer-carrier shift.
//!    - PRS: seed the
//!      [`DifferentialReference`](dab_ofdm::DifferentialReference).
//!    - Data symbols 1..=3 (the FIC region): step the differential and
//!      demap to 3072 soft bits per symbol via
//!      [`DqpskDemap`](dab_ofdm::DqpskDemap), appending to a per-frame
//!      `soft_bits` buffer.
//! 6. Split the 9216-element FIC `soft_bits` buffer into 4 ficBlocks of
//!    2304 soft bits each; for each block:
//!    - [`FicProtection::deconvolve`](dab_viterbi::FicProtection::deconvolve)
//!      → 768 information bits.
//!    - [`descramble_and_pack`](dab_descramble::descramble_and_pack) →
//!      96 bytes (3 FIBs).
//! 7. Concatenate 4 × 96 = 384 bytes (12 FIBs) and feed to
//!    [`FicAccumulator::feed_fic`](dab_fic::FicAccumulator::feed_fic),
//!    which validates each 32-byte FIB's CRC-16 and dispatches FIGs into
//!    the ensemble model.
//!
//! The integer-CFO correction in step 5 is critical: even a 1-carrier
//! mismatch corrupts the frequency-de-interleaver output for every logical
//! index, which would in turn break every FIB CRC downstream. The Stage 6/7
//! integration test on `k8b_v4.iq` reports `int_cfo = 1` for that capture.

use std::path::Path;

use anyhow::{anyhow, Result};
use num_complex::Complex;

use dab_descramble::descramble_and_pack;
use dab_fic::FicAccumulator;
use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::{
    estimate_offset_eti, CpSync, DifferentialReference, DqpskDemap, LinearResampler, Nco,
    NullDetector, SymbolFft,
};
use dab_viterbi::{FicProtection, FIC_IN_BITS, FIC_OUT_BITS};

/// Mode I per-symbol length (`T_g + T_u = 504 + 2048`) at the internal
/// 2.048 MSPS rate.
const TS: usize = 2552;
/// Mode I null-symbol length in samples.
const NULL_LEN: usize = 2656;
/// Number of OFDM symbols carrying FIC per frame (Mode I).
const FIC_SYMBOLS: usize = 3;
/// Soft bits per OFDM symbol = `2 * K = 2 * 1536`.
const BITS_PER_SYMBOL: usize = 3072;
/// Soft bits per frame's FIC region.
const FIC_SOFT_BITS_PER_FRAME: usize = FIC_SYMBOLS * BITS_PER_SYMBOL; // 9216
/// FicBlocks per frame.
const FIC_BLOCKS_PER_FRAME: usize = FIC_SOFT_BITS_PER_FRAME / FIC_IN_BITS; // 4
/// Decoded FIC bytes per frame: 4 blocks × 96 bytes = 384 bytes = 12 FIBs.
const FIC_BYTES_PER_FRAME: usize = FIC_BLOCKS_PER_FRAME * (FIC_OUT_BITS / 8); // 384
/// Internal sample rate after Stage 1 resample.
pub const FS_INTERNAL: f64 = 2_048_000.0;

/// Aggregate result of running the I/Q → FIC pipeline on a capture.
#[derive(Debug, Default)]
pub struct FicIqResult {
    /// Ensemble model assembled from successful FIB CRCs.
    pub ensemble: dab_fic::Ensemble,
    /// Total FIBs fed to the accumulator (32-byte chunks of all frame outputs).
    pub fib_total: usize,
    /// FIBs that passed the CRC-16 check.
    pub fib_ok: usize,
    /// Number of DAB frames the pipeline produced FIC bytes for.
    pub frames_decoded: usize,
    /// Frames that were skipped (sync failed, fractional CFO out of range,
    /// or insufficient samples remaining after the null).
    pub frames_skipped: usize,
    /// Total resampled samples actually processed.
    pub resampled_samples: usize,
    /// Number of null dips detected across the whole capture.
    pub null_dips: usize,
    /// Best-frame band ratio (active vs. guard band, dB) — for reporting.
    pub best_band_ratio_db: f64,
}

/// Run the full FIC pipeline on an I/Q capture file.
///
/// `iq_path` points at a raw I/Q file; `input_format` and `input_sample_rate_hz`
/// describe its on-disk layout (the in-tree `airspy-mini-dmb` captures are
/// `Cs16Le` at `3_000_000`). The output is an aggregate `FicIqResult` with
/// the populated ensemble and accumulated FIB statistics.
///
/// Diagnostic dumps are opt-in via env vars (same pattern as the eti-stuff
/// patches in `docs/diag/eti-stuff-ibits-dump.patch`):
///
/// - `DAB_RS_DUMP_VITERBI_OUT` — per-frame pre-descramble Viterbi output
///   (3072 bytes, bit-per-byte; preceded by a 4-byte LE `frame_idx`).
/// - `DAB_RS_DUMP_DESCRAMBLED` — per-frame post-descramble bits, same
///   layout. XOR of the two gives the frame's PRBS (proves the descrambler
///   is doing what's documented).
pub fn process_iq_to_fic(
    iq_path: &Path,
    input_format: IqFormat,
    input_sample_rate_hz: u32,
) -> Result<FicIqResult> {
    // ---- Stage 1: read + resample to 2.048 MSPS ----
    //
    // Slice-17 bypass: if the input is already at the internal rate
    // (`Cf32Le @ 2.048 MSPS`), skip the polyphase resampler so dab-rs
    // consumes the EXACT post-resample stream eti-stuff produced via its
    // own linear-interpolation rate converter. This isolates the sync /
    // FFT-framing / differential stages from the resampler step.
    let bypass_resampler =
        input_format == IqFormat::Cf32Le && input_sample_rate_hz == 2_048_000;
    let mut reader = IqFileReader::open(iq_path, input_format, input_sample_rate_hz)?;
    // SLICE-22: use eti-stuff's linear-interpolation resampler (airspy-handler.cpp:157-162)
    // instead of the polyphase FIR. Slice-17 measured the FIR losing ~2.5 dB
    // SNR vs linear-interp on K8B (marginal SNR ~11 dB), pushing the chain
    // below decoding threshold for the cs16le-with-native-resampler path.
    let mut resampler = LinearResampler::new(input_sample_rate_hz);
    let mut resampled: Vec<Complex<f32>> = Vec::with_capacity(41_000_000);
    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        if bypass_resampler {
            resampled.extend_from_slice(&buf[..n]);
        } else {
            resampled.extend_from_slice(&resampler.process(&buf[..n]));
        }
    }
    if resampled.len() < 2_000_000 {
        return Err(anyhow!(
            "resampled stream too short ({} samples) — capture appears truncated",
            resampled.len()
        ));
    }

    // ---- Stage 2: null detection across the whole stream ----
    let nulls = NullDetector::new(2_048_000).detect(&resampled);
    if nulls.positions.is_empty() {
        return Err(anyhow!("no null symbols detected in the capture"));
    }

    let cp = CpSync::mode_i();
    let mut sfft = SymbolFft::mode_i();
    let demap = DqpskDemap::mode_i();
    let mut acc = FicAccumulator::new();

    let mut result = FicIqResult {
        resampled_samples: resampled.len(),
        null_dips: nulls.positions.len(),
        ..FicIqResult::default()
    };

    // ---- env-gated diagnostic dumps (see docstring) ----
    use std::fs::File;
    use std::io::{BufWriter, Write};
    // SLICE-19 absolute-sample sync position dump. Same coordinate as
    // eti-stuff's DAB_RS_DUMP_SYNC_POS (counted from getSamples consumption
    // of the resampled stream). Record layout:
    //     u32 LE frame_idx (1-based)
    //     u32 LE ofdmSymbolCount (1 for PRS, 2..76 for data)
    //     u64 LE abs_useful_start (sample index in the resampled stream
    //                              where the FFT useful-part starts)
    let mut sync_pos_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_SYNC_POS")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));

    // SLICE-20 FFT input dump. Per FIC symbol writes:
    //     u32 frame_idx, u32 sym, u64 useful_start,
    //     2048 × complex<f32> pre-NCO useful samples,
    //     2048 × complex<f32> post-NCO useful samples.
    // Same useful_start coordinate as SYNC_POS so the PRE-mix samples
    // can be byte-compared to cf32 file[useful_start..useful_start+T_u].
    let mut fft_input_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_FFT_INPUT")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    let mut viterbi_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_VITERBI_OUT")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    let mut descrambled_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_DESCRAMBLED")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    // Slice-7 cross-check: per-frame Viterbi *input* (4 ficBlocks × 3096
    // i16). Header: u32 LE frame_idx. Payload: `4 * 3096 * 2 = 24768`
    // bytes. Pairs with the patched eti-stuff `DAB_RS_ORACLE_VITERBI_IN`
    // (which writes per ficBlock; the comparator aggregates 4 oracle
    // records into one frame to match this layout).
    let mut viterbi_in_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_VITERBI_IN")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    // Slice-9 bisection: the *pre-depuncture* OFDM demap output, 9216
    // i8 soft bits per frame (4 ficBlocks × 2304). The demap emits
    // values in `[-127, +127]` (slice-2 `dqpsk_demap` spec), so an
    // i8 cast is lossless. Each frame: u32 LE frame_idx + 9216 i8.
    let mut demap_out_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_DEMAP_OUT")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    // Slice-11 arg(r) heatmap: per-frame 3 differential spectra
    // (post-rotation by integer-CFO δ, post-`DifferentialReference::step`).
    // Header: u32 frame_idx, payload: 3 × 2048 complex<f32> LE pairs
    // (49152 bytes). The arg of each active-carrier r value gives the
    // π/4-DQPSK constellation phase + any residual upstream rotation.
    let mut diff_spec_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_DIFF_SPEC")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));

    // SLICE-16: pre-differential FFT bins, matching eti-stuff's `fft_buffer`
    // dump (DAB_RS_DIAG_DUMP_FFT). Per-symbol record: u32 frame_idx, u32
    // ofdmSymbolCount (eti-stuff convention: dab-rs FIC symbol s ↔ count s+1),
    // then 2048 complex<f32> (re, im) LE bins in natural FFT order.
    let mut fft_pre_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_FFT_PRE")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));

    let p = |z: &Complex<f32>| (z.re as f64).powi(2) + (z.im as f64).powi(2);

    // SLICE-15 Pair 4: NCO continuous phase across frame boundaries.
    // eti-stuff's `localPhase` is a class member that accumulates -phase per
    // sample across the entire stream (ofdm-processor.cpp:168, 217). When
    // re-instantiated per-frame, the start-of-frame phase resets to 0 and any
    // residual CFO produces a discontinuity at every frame boundary that
    // corrupts the next frame's PRS reference vs the eti-stuff convention.
    let mut frame_nco = Nco::new(FS_INTERNAL);

    // SLICE-15 Pair 3: cumulative fractional-CFO integrator. eti-stuff
    // accumulates `fineCorrector += 0.1 * arg(FreqCorr) / M_PI *
    // (carrierDiff/2)` per frame (ofdm-processor.cpp:526). The per-frame
    // absolute estimate from CP autocorr equals (cumulative + residual)
    // when samples are pre-corrected by cumulative, so an equivalent
    // exponentially-weighted MA is `cumulative = 0.9 * cumulative + 0.1 *
    // absolute`. Start at 0; converges over the first ~10 frames.
    let mut cumulative_cfo_hz: f64 = 0.0;

    // ---- Per-frame loop ----
    for &null_pos in &nulls.positions {
        let prs_guess = match null_pos.checked_add(NULL_LEN) {
            Some(v) => v,
            None => {
                result.frames_skipped += 1;
                continue;
            }
        };

        // Need PRS + 3 FIC symbols of samples available.
        let frame_end_min = prs_guess + (1 + FIC_SYMBOLS) * TS;
        if frame_end_min > resampled.len() {
            result.frames_skipped += 1;
            continue;
        }

        // SLICE-19 narrow fine_time search: half = T_g/4 = 126 samples.
        // The wide TS=2552 half makes CP autocorr peak-pick onto adjacent
        // OFDM symbols' CP (autocorr peaks every T_s samples), and the
        // SYNC_POS comparison showed dab-rs's prs_start oscillating by
        // ±T_s = ±2552 between frames vs eti-stuff's stable lock. eti-stuff
        // uses a frequency-domain PRS cross-correlation (findIndex) that is
        // uniquely peaked on PRS only; until that's ported, bounding the
        // CP-autocorr search to a quarter CP keeps us within the correct
        // symbol's CP window.
        let prs_start = cp.fine_time(&resampled, prs_guess, 126);
        if prs_start + (1 + FIC_SYMBOLS) * TS > resampled.len() {
            result.frames_skipped += 1;
            continue;
        }
        // SLICE-19 sync-position dump. PRS useful_start = prs_start + T_g
        // (start of the CP of the PRS symbol + T_g samples). Data symbol
        // s=1..3 useful_start = prs_start + s*TS + T_g.
        {
            const T_G: usize = 504;
            let frame_idx_now = (result.frames_decoded as u32) + 1;
            if let Some(fp) = sync_pos_fp.as_mut() {
                let _ = fp.write_all(&frame_idx_now.to_le_bytes());
                let _ = fp.write_all(&1u32.to_le_bytes());
                let prs_useful = (prs_start + T_G) as u64;
                let _ = fp.write_all(&prs_useful.to_le_bytes());
                for s in 1..=FIC_SYMBOLS {
                    let _ = fp.write_all(&frame_idx_now.to_le_bytes());
                    let sym = (s as u32) + 1;
                    let _ = fp.write_all(&sym.to_le_bytes());
                    let useful = (prs_start + s * TS + T_G) as u64;
                    let _ = fp.write_all(&useful.to_le_bytes());
                }
            }
        }

        // Absolute fractional CFO estimate from CP autocorrelation of raw
        // samples. Used as the *input* to the integrator below; the NCO
        // actually mixes with `cumulative_cfo_hz`.
        let abs_cfo_hz = cp.estimate_cfo_hz(&resampled, prs_start, 50) as f64;
        if !abs_cfo_hz.is_finite() || abs_cfo_hz.abs() > 600.0 {
            // Wider than the 500 Hz integration-test threshold to tolerate
            // occasional outliers; a frame with CFO > 600 Hz is almost
            // certainly a mis-locked sync attempt rather than real drift.
            result.frames_skipped += 1;
            continue;
        }
        // SLICE-25 fudge-free CFO chain.
        //
        // Slice 21 used `abs_cfo_hz / 2` as an empirical fix that gave 89% FIB
        // but had no mathematical justification. Slice 24 narrowed the 2× to a
        // post-FFT step. Slice 25 found the actual mechanism:
        //
        //   eti-stuff applies the FULL CFO (integer carrier + fractional Hz)
        //   in time-domain NCO via `getSamples(coarse + fine)`. The integer
        //   part `coarseCorrector` contributes per-symbol NCO phase advance
        //   `2π · δ · 1000 · T_s / fs = 2π · δ · T_s/T_u`, which the rotate-
        //   based dab-rs approach (rotate_spectrum after FFT) DOES NOT apply.
        //   Missing that ~7.83 rad / sym phase advance for δ=1 mis-aligns the
        //   differential demap against the π/4-DQPSK constellation, hence the
        //   4% pass at α=1.0.
        //
        //   Slice-21's /2 fudge compensated this by halving the NCO mix freq,
        //   which happened to land the differential rotation near a workable
        //   point on the constellation (still wrong by structure, but more
        //   forgiving for the Viterbi soft-bit envelope). That's why α=0.5
        //   + rotate worked at 89%.
        //
        //   The fix: include integer CFO in the time-domain NCO mix (matching
        //   eti-stuff's `coarseCorrector + fineCorrector`), then skip
        //   rotate_spectrum for data symbols. PRS keeps the rotate path
        //   because we need PRS FFT first to detect δ. Within-frame phase
        //   continuity then matches eti-stuff bit-for-bit and the full
        //   fractional estimate (α=1.0) is used directly.
        //
        //   Validated: sim5 1704/1884 (90.4%), sim4 1637/1908 (85.8%) —
        //   both at fudge-free α=1.0 + integer-in-NCO.
        if result.frames_decoded == 0 {
            cumulative_cfo_hz = abs_cfo_hz;
        } else {
            cumulative_cfo_hz = 0.9 * cumulative_cfo_hz + 0.1 * abs_cfo_hz;
        }
        let cfo_hz = cumulative_cfo_hz;

        // ---- Stage 4/5: FFT the PRS, with fractional-CFO removal ----
        //
        // The `Nco` is hoisted outside the per-frame loop (slice-15 Pair 4)
        // so its phase accumulator is continuous across the entire stream,
        // matching eti-stuff's class-member `localPhase` that runs over all
        // samples without reset (ofdm-processor.cpp `getSample`/`getSamples`).
        let prs_spec_raw =
            fft_symbol_corrected_with_dump(
                &resampled, prs_start, cfo_hz, &mut sfft, &mut frame_nco,
                fft_input_fp.as_mut(),
                (result.frames_decoded as u32) + 1,
                1u32,
            );

        // ---- Stage 4b: integer CFO detection on the PRS spectrum ----
        //
        // Slice-13 Pair 2 fix: use the eti-stuff `estimateOffset` algorithm
        // (adjacent-carrier phase differences) instead of the magnitude-
        // correlation `detect_integer_cfo`. The phase-difference form is
        // timing-invariant — sub-sample timing residuals introduce a
        // per-carrier phase ramp that cancels in adjacent differences — so
        // it recovers the correct δ even when `cp.fine_time` is one sample
        // off the true symbol start. Search range is ±35 carriers (verbatim
        // from `eti-stuff/src/ofdm/phasereference.cpp::estimateOffset`).
        // SLICE-20 Result-shift fix: per_bin_phase.py measured |cc|=0.9986 at
        // m=+1 between dab-rs and eti-stuff FFT bins on the same input window
        // (sample-aligned via SYNC_POS). The dab-rs spectrum was right-shifted
        // by 1 carrier vs eti-stuff because slice-15's `delta=0` forced this
        // to drop integer-CFO correction. rotate_spectrum(spec, +1) does
        // `out[i] = spec[i+1]` — a left shift — undoing the +1 spectral
        // mismatch so de-interleaver mapIn(i) reads the right carrier.
        let delta = estimate_offset_eti(&prs_spec_raw);
        // SLICE-25 fix: integer-CFO via time-domain NCO for data symbols
        // (matches eti-stuff `coarseCorrector` contribution to NCO).
        // PRS keeps rotate-based correction since we need its FFT first
        // to detect δ; PRS phase trajectory is rebased once we know δ via
        // the cf32 stream's frame_nco continuation onto data symbols.
        let nco_extra_hz = (delta as f64) * 1000.0;

        // Track best band ratio (post-rotation) just for the reporting line.
        // PRS: keep the rotate_spectrum correction so the seed spectrum is
        // at correct carrier alignment. Data symbols below use NCO time-
        // domain integer correction instead.
        let prs_spec = rotate_spectrum(&prs_spec_raw, delta);
        let active_e: f64 =
            (1..=768).chain(1280..=2047).map(|i| p(&prs_spec[i])).sum();
        let guard_e: f64 = (769..=1279).map(|i| p(&prs_spec[i])).sum();
        if guard_e > 0.0 {
            let active_per_bin = active_e / 1536.0;
            let guard_per_bin = guard_e / 511.0;
            let band_ratio_db = 10.0 * (active_per_bin / guard_per_bin).log10();
            if band_ratio_db > result.best_band_ratio_db {
                result.best_band_ratio_db = band_ratio_db;
            }
        }

        // ---- Stage 6: seed the differential reference with the post-rotation PRS spec ----
        let mut diff_ref = DifferentialReference::new();
        diff_ref.seed_prs(&prs_spec);

        // ---- Demap the 3 FIC OFDM symbols ----
        let mut frame_soft: Vec<i16> = Vec::with_capacity(FIC_SOFT_BITS_PER_FRAME);
        let mut diff_specs_for_dump: Vec<Vec<Complex<f32>>> = Vec::with_capacity(FIC_SYMBOLS);
        let mut ok = true;
        for s in 1..=FIC_SYMBOLS {
            let cp_start = prs_start + s * TS;
            // SLICE-25: total CFO = fractional cfo_hz + integer δ·carrier_diff
            // applied via time-domain NCO. No rotate_spectrum needed for data
            // symbols because the NCO handles both fractional and integer.
            let data_cfo = cfo_hz + nco_extra_hz;
            let spec_raw = fft_symbol_corrected_with_dump(
                &resampled, cp_start, data_cfo, &mut sfft, &mut frame_nco,
                fft_input_fp.as_mut(),
                (result.frames_decoded as u32) + 1, (s as u32) + 1);
            let spec = spec_raw;
            // SLICE-16: dump pre-differential FFT bins (eti-stuff fft_buffer match)
            if let Some(fp) = fft_pre_fp.as_mut() {
                let frame_idx = (result.frames_decoded as u32) + 1;
                let ofdm_symbol_count = (s as u32) + 1; // dab-rs s ↔ eti-stuff count s+1
                let _ = fp.write_all(&frame_idx.to_le_bytes());
                let _ = fp.write_all(&ofdm_symbol_count.to_le_bytes());
                let mut buf = Vec::with_capacity(2048 * 8);
                for z in &spec {
                    buf.extend_from_slice(&z.re.to_le_bytes());
                    buf.extend_from_slice(&z.im.to_le_bytes());
                }
                let _ = fp.write_all(&buf);
            }
            let diff = diff_ref.step(&spec);
            if diff_spec_fp.is_some() {
                diff_specs_for_dump.push(diff.clone());
            }
            let bits = demap.demap(&diff);
            if bits.len() != BITS_PER_SYMBOL {
                ok = false;
                break;
            }
            frame_soft.extend_from_slice(&bits);
        }
        if !ok || frame_soft.len() != FIC_SOFT_BITS_PER_FRAME {
            result.frames_skipped += 1;
            continue;
        }

        // ---- Pre-depuncture OFDM demap dump (slice-9 bisection input) ----
        let dab_frame_idx_pre = (result.frames_decoded as u32) + 1;
        if let Some(fp) = demap_out_fp.as_mut() {
            let _ = fp.write_all(&dab_frame_idx_pre.to_le_bytes());
            // Cast each i16 demap sample to i8 — the demap guarantees the
            // value is in [-127, +127] so truncation is lossless.
            let bytes: Vec<u8> = frame_soft
                .iter()
                .map(|&v| (v as i8) as u8)
                .collect();
            let _ = fp.write_all(&bytes);
        }

        // ---- Decode 4 ficBlocks → 384 bytes ----
        let mut dumps = DecodeDumps::default();
        let frame_bytes = decode_fic_soft_bits_with_dumps(&frame_soft, &mut dumps);
        debug_assert_eq!(frame_bytes.len(), FIC_BYTES_PER_FRAME);

        // Persist optional intermediate-stage dumps. Header per frame:
        // u32 LE frame_idx (1-based, matches the oracle dump convention),
        // then 3072 bytes of bit-per-byte (4 ficBlocks × 768 info bits).
        let dab_frame_idx = (result.frames_decoded as u32) + 1;
        if let Some(fp) = viterbi_fp.as_mut() {
            let _ = fp.write_all(&dab_frame_idx.to_le_bytes());
            let _ = fp.write_all(&dumps.viterbi_out);
        }
        if let Some(fp) = descrambled_fp.as_mut() {
            let _ = fp.write_all(&dab_frame_idx.to_le_bytes());
            let _ = fp.write_all(&dumps.descrambled);
        }
        if let Some(fp) = viterbi_in_fp.as_mut() {
            let _ = fp.write_all(&dab_frame_idx.to_le_bytes());
            // Serialise i16 LE — matches the eti-stuff fic-handler dump.
            let mut buf = Vec::with_capacity(dumps.viterbi_in.len() * 2);
            for v in &dumps.viterbi_in {
                buf.extend_from_slice(&v.to_le_bytes());
            }
            let _ = fp.write_all(&buf);
        }
        if let Some(fp) = diff_spec_fp.as_mut() {
            if diff_specs_for_dump.len() == FIC_SYMBOLS {
                let _ = fp.write_all(&dab_frame_idx.to_le_bytes());
                let mut buf = Vec::with_capacity(FIC_SYMBOLS * 2048 * 8);
                for spec in &diff_specs_for_dump {
                    for z in spec {
                        buf.extend_from_slice(&z.re.to_le_bytes());
                        buf.extend_from_slice(&z.im.to_le_bytes());
                    }
                }
                let _ = fp.write_all(&buf);
            }
        }

        // ---- Feed to the accumulator ----
        let prior_total = acc.fib_total;
        let prior_ok = acc.fib_ok;
        acc.feed_fic(&frame_bytes);
        result.fib_total += acc.fib_total - prior_total;
        result.fib_ok += acc.fib_ok - prior_ok;
        result.frames_decoded += 1;
    }

    if let Some(mut fp) = viterbi_fp {
        let _ = fp.flush();
    }
    if let Some(mut fp) = descrambled_fp {
        let _ = fp.flush();
    }
    if let Some(mut fp) = viterbi_in_fp {
        let _ = fp.flush();
    }
    if let Some(mut fp) = demap_out_fp {
        let _ = fp.flush();
    }
    if let Some(mut fp) = diff_spec_fp {
        let _ = fp.flush();
    }

    result.ensemble = acc.ensemble;
    Ok(result)
}

/// FFT one symbol with fractional-CFO removal — copy `TS` samples at
/// `cp_start`, mix to cancel `cfo_hz`, FFT the useful part, return the
/// 2048-bin natural-order spectrum.
#[allow(dead_code)]
pub fn fft_symbol_corrected(
    resampled: &[Complex<f32>],
    cp_start: usize,
    cfo_hz: f64,
    sfft: &mut SymbolFft,
    nco: &mut Nco,
) -> Vec<Complex<f32>> {
    let mut region = resampled[cp_start..cp_start + TS].to_vec();
    nco.mix(&mut region, -cfo_hz);
    sfft.fft_symbol(&region)
}

/// SLICE-20 instrumented variant: optionally dumps PRE/POST-NCO useful
/// samples to `dump` with a (frame, sym, useful_start) header.
fn fft_symbol_corrected_with_dump(
    resampled: &[Complex<f32>],
    cp_start: usize,
    cfo_hz: f64,
    sfft: &mut SymbolFft,
    nco: &mut Nco,
    mut dump: Option<&mut std::io::BufWriter<std::fs::File>>,
    frame_idx: u32,
    sym: u32,
) -> Vec<Complex<f32>> {
    const T_G: usize = 504;
    const T_U: usize = 2048;
    use std::io::Write;
    let mut region = resampled[cp_start..cp_start + TS].to_vec();
    let useful_start_u64 = (cp_start + T_G) as u64;
    if let Some(fp) = dump.as_deref_mut() {
        let _ = fp.write_all(&frame_idx.to_le_bytes());
        let _ = fp.write_all(&sym.to_le_bytes());
        let _ = fp.write_all(&useful_start_u64.to_le_bytes());
        // pre-NCO useful samples
        for z in &region[T_G..T_G + T_U] {
            let _ = fp.write_all(&z.re.to_le_bytes());
            let _ = fp.write_all(&z.im.to_le_bytes());
        }
    }
    nco.mix(&mut region, -cfo_hz);
    if let Some(fp) = dump.as_deref_mut() {
        // post-NCO useful samples
        for z in &region[T_G..T_G + T_U] {
            let _ = fp.write_all(&z.re.to_le_bytes());
            let _ = fp.write_all(&z.im.to_le_bytes());
        }
    }
    sfft.fft_symbol(&region)
}

/// Rotate a natural-order spectrum by `delta` FFT bins to undo an integer
/// carrier-frequency offset of `+delta` carriers (received carrier `k` lands
/// at bin `k + delta`, so the corrected spectrum reads bin `i + delta` when
/// the de-interleaver asks for bin `i`).
///
/// Equivalent to `out[i] = spec[(i + delta) mod T_u]`. Returns a fresh vector
/// to keep the function side-effect-free.
pub fn rotate_spectrum(spec: &[Complex<f32>], delta: i32) -> Vec<Complex<f32>> {
    let n = spec.len();
    if delta == 0 {
        return spec.to_vec();
    }
    let shift = (delta.rem_euclid(n as i32)) as usize;
    // shift bins so that out[0] = spec[shift], out[1] = spec[shift+1], ...
    let mut out = Vec::with_capacity(n);
    out.extend_from_slice(&spec[shift..]);
    out.extend_from_slice(&spec[..shift]);
    out
}

/// Run the 4 ficBlocks through depuncture + Viterbi + descramble + pack and
/// concatenate to a single 384-byte (12 FIB) per-frame buffer. Exposed for
/// tests that want to feed synthetic soft bits directly.
pub fn decode_fic_soft_bits_to_bytes(frame_soft: &[i16]) -> Vec<u8> {
    decode_fic_soft_bits_with_dumps(frame_soft, &mut DecodeDumps::default())
}

/// Optional intermediate-stage dump buffers for the FIC decode pipeline.
///
/// Each field, if non-empty, accumulates one bit-per-`u8` record per ficBlock
/// (`FIC_OUT_BITS = 768` entries each). Used by the bit-by-bit diff lane
/// (`dab diag-viterbi-bits`) to localise where the dab-rs chain diverges
/// from a live ETI's FIB bytes — separately checking the Viterbi output
/// before energy descramble (gotcha #7 prime suspect), after descramble
/// (PRBS polarity / seed), and the final packed bytes (bit-ordering).
#[derive(Debug, Default)]
pub struct DecodeDumps {
    /// Per-ficBlock depunctured Viterbi *input* (3096 i16 soft bits each).
    /// Full frame length = 4 * 3096 = 12384 entries. Matches what the
    /// patched eti-stuff `ficHandler::process_ficInput` writes to its
    /// `DAB_RS_ORACLE_VITERBI_IN` dump bit-for-bit (modulo the OFDM-side
    /// soft-bit divergence which slice 7 is trying to measure).
    pub viterbi_in: Vec<i16>,
    /// Pre-descramble Viterbi output, bit-per-byte (`0` / `1`). Length
    /// after a full frame: 4 * 768 = 3072 entries.
    pub viterbi_out: Vec<u8>,
    /// Post-descramble bits, bit-per-byte. Same length as `viterbi_out`.
    pub descrambled: Vec<u8>,
}

/// Same as [`decode_fic_soft_bits_to_bytes`] but also fills the provided
/// [`DecodeDumps`] with per-ficBlock intermediate bit streams.
pub fn decode_fic_soft_bits_with_dumps(
    frame_soft: &[i16],
    dumps: &mut DecodeDumps,
) -> Vec<u8> {
    assert_eq!(
        frame_soft.len(),
        FIC_SOFT_BITS_PER_FRAME,
        "frame_soft must be exactly {} soft bits",
        FIC_SOFT_BITS_PER_FRAME
    );
    let mut out = Vec::with_capacity(FIC_BYTES_PER_FRAME);
    let mut fic = FicProtection::new();
    for chunk in frame_soft.chunks_exact(FIC_IN_BITS) {
        // Capture the depunctured Viterbi input by replaying the depuncture
        // here against the public puncture table — this is what FicProtection
        // builds internally before calling its scalar Viterbi. Slice-7
        // cross-check uses this to verify dab-rs and the oracle hand the
        // same 3096-soft-bit codeword to the Viterbi decoder.
        let table = fic.index_table();
        debug_assert_eq!(table.len(), dab_viterbi::FIC_VITERBI_LEN);
        let mut viterbi_block = vec![0_i16; dab_viterbi::FIC_VITERBI_LEN];
        let mut ic = 0_usize;
        for i in 0..dab_viterbi::FIC_VITERBI_LEN {
            if table[i] {
                viterbi_block[i] = chunk[ic];
                ic += 1;
            }
        }
        debug_assert_eq!(ic, FIC_IN_BITS);
        dumps.viterbi_in.extend_from_slice(&viterbi_block);

        // Stage A: depuncture + Viterbi decode → 768 info bits, bit-per-byte.
        let info_bits = fic.deconvolve(chunk);
        debug_assert_eq!(info_bits.len(), FIC_OUT_BITS);
        dumps.viterbi_out.extend_from_slice(&info_bits);

        // Stage B: energy descramble (XOR with FIC PRBS).
        //
        // `descramble_and_pack` re-applies the PRBS internally and packs
        // MSB-first. To dump the bit-level descrambled stream separately
        // we recompute it here in bit form. The packed `bytes` below uses
        // the same XOR so the two are guaranteed consistent.
        let prbs = dab_descramble::prbs_sequence(info_bits.len());
        let descrambled_bits: Vec<u8> = info_bits
            .iter()
            .zip(prbs.iter())
            .map(|(b, p)| b ^ p)
            .collect();
        dumps.descrambled.extend_from_slice(&descrambled_bits);

        // Stage C: pack MSB-first into 96 bytes (= 3 FIBs).
        let bytes = descramble_and_pack(&info_bits);
        debug_assert_eq!(bytes.len(), FIC_OUT_BITS / 8);
        out.extend_from_slice(&bytes);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const T_U: usize = 2048;

    #[test]
    fn rotate_spectrum_zero_delta_is_identity() {
        let spec: Vec<Complex<f32>> = (0..T_U)
            .map(|i| Complex::new(i as f32, -(i as f32)))
            .collect();
        let r = rotate_spectrum(&spec, 0);
        assert_eq!(r, spec);
    }

    #[test]
    fn rotate_spectrum_positive_delta_shifts_window_up() {
        // delta = +1: out[i] = spec[i+1], out[T_u-1] = spec[0].
        let mut spec = vec![Complex::new(0.0_f32, 0.0); T_U];
        spec[5] = Complex::new(1.0, 0.0);
        spec[6] = Complex::new(2.0, 0.0);
        let r = rotate_spectrum(&spec, 1);
        assert_eq!(r[4], Complex::new(1.0, 0.0));
        assert_eq!(r[5], Complex::new(2.0, 0.0));
        assert_eq!(r[6], Complex::new(0.0, 0.0));
    }

    #[test]
    fn rotate_spectrum_negative_delta_shifts_window_down() {
        // delta = -1: out[0] = spec[T_u - 1], out[1] = spec[0], …
        let mut spec = vec![Complex::new(0.0_f32, 0.0); T_U];
        spec[0] = Complex::new(7.0, 0.0);
        spec[T_U - 1] = Complex::new(9.0, 0.0);
        let r = rotate_spectrum(&spec, -1);
        assert_eq!(r[0], Complex::new(9.0, 0.0));
        assert_eq!(r[1], Complex::new(7.0, 0.0));
    }

    #[test]
    fn frame_byte_geometry_checks_out() {
        assert_eq!(FIC_SOFT_BITS_PER_FRAME, 9216);
        assert_eq!(FIC_BLOCKS_PER_FRAME, 4);
        assert_eq!(FIC_BYTES_PER_FRAME, 384);
        assert_eq!(FIC_OUT_BITS, 768);
        assert_eq!(FIC_IN_BITS, 2304);
    }
}
