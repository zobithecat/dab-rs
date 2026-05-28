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
    detect_integer_cfo, CpSync, DifferentialReference, DqpskDemap, Nco, NullDetector, Resampler,
    SymbolFft,
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
const FS_INTERNAL: f64 = 2_048_000.0;

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
    let mut reader = IqFileReader::open(iq_path, input_format, input_sample_rate_hz)?;
    let mut resampler = Resampler::new_3m_to_2048k();
    let mut resampled: Vec<Complex<f32>> = Vec::with_capacity(41_000_000);
    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        resampled.extend_from_slice(&resampler.process(&buf[..n]));
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
    let mut viterbi_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_VITERBI_OUT")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));
    let mut descrambled_fp: Option<BufWriter<File>> = std::env::var("DAB_RS_DUMP_DESCRAMBLED")
        .ok()
        .and_then(|p| File::create(&p).ok().map(BufWriter::new));

    let p = |z: &Complex<f32>| (z.re as f64).powi(2) + (z.im as f64).powi(2);

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

        let prs_start = cp.fine_time(&resampled, prs_guess, TS);
        if prs_start + (1 + FIC_SYMBOLS) * TS > resampled.len() {
            result.frames_skipped += 1;
            continue;
        }

        let cfo_hz = cp.estimate_cfo_hz(&resampled, prs_start, 50) as f64;
        if !cfo_hz.is_finite() || cfo_hz.abs() > 600.0 {
            // Wider than the 500 Hz integration-test threshold to tolerate
            // occasional outliers; a frame with CFO > 600 Hz is almost
            // certainly a mis-locked sync attempt rather than real drift.
            result.frames_skipped += 1;
            continue;
        }

        // ---- Stage 4/5: FFT the PRS, with fractional-CFO removal ----
        let prs_spec_raw = fft_symbol_corrected(&resampled, prs_start, cfo_hz, &mut sfft);

        // ---- Stage 4b: integer CFO detection on the PRS spectrum ----
        //
        // `detect_integer_cfo` is sensitive to timing residuals on captured
        // signals (coherent correlation breaks down under per-carrier phase
        // ramps left by sub-sample timing offsets). Only apply the detected
        // δ when the peak is *meaningfully* above the runner-up; otherwise
        // default to δ = 0. eti-stuff's `estimateOffset` uses adjacent-
        // carrier phase *differences* which would be timing-invariant — that
        // is a planned follow-up; see notes in `dab-ofdm::integer_cfo`.
        let icfo_raw = detect_integer_cfo(&prs_spec_raw, 5);
        let delta = if icfo_raw.peak > 1.5 * icfo_raw.runner_up {
            icfo_raw.offset
        } else {
            0
        };

        // Track best band ratio (post-rotation) just for the reporting line.
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
        let mut ok = true;
        for s in 1..=FIC_SYMBOLS {
            let cp_start = prs_start + s * TS;
            let spec_raw = fft_symbol_corrected(&resampled, cp_start, cfo_hz, &mut sfft);
            let spec = rotate_spectrum(&spec_raw, delta);
            let diff = diff_ref.step(&spec);
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

    result.ensemble = acc.ensemble;
    Ok(result)
}

/// FFT one symbol with fractional-CFO removal — copy `TS` samples at
/// `cp_start`, mix to cancel `cfo_hz`, FFT the useful part, return the
/// 2048-bin natural-order spectrum.
fn fft_symbol_corrected(
    resampled: &[Complex<f32>],
    cp_start: usize,
    cfo_hz: f64,
    sfft: &mut SymbolFft,
) -> Vec<Complex<f32>> {
    let mut region = resampled[cp_start..cp_start + TS].to_vec();
    Nco::new(FS_INTERNAL).mix(&mut region, -cfo_hz);
    sfft.fft_symbol(&region)
}

/// Rotate a natural-order spectrum by `delta` FFT bins to undo an integer
/// carrier-frequency offset of `+delta` carriers (received carrier `k` lands
/// at bin `k + delta`, so the corrected spectrum reads bin `i + delta` when
/// the de-interleaver asks for bin `i`).
///
/// Equivalent to `out[i] = spec[(i + delta) mod T_u]`. Returns a fresh vector
/// to keep the function side-effect-free.
fn rotate_spectrum(spec: &[Complex<f32>], delta: i32) -> Vec<Complex<f32>> {
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
