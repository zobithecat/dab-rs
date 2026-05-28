//! Week 3c–3d golden integration test: full OFDM sync/demod chain
//! (Stages 1–7) on the new best K8B oracle capture `k8b_v4.iq`.
//!
//! Pipeline: dab-iq (INT16_IQ @ 3 MSPS) → Stage 1 resample 3→2.048
//! MSPS → Stage 2 null detection → Stage 3 CP fine-time + fractional CFO
//! → Stage 4 NCO frequency correction + integer carrier offset → Stage 5
//! per-symbol FFT framing → Stage 6 differential per-carrier reference →
//! Stage 7 π/4-DQPSK demap + frequency de-interleaving → soft bits.
//!
//! The cross-validation oracle is `airspy-mini-dmb/data/captures/k8b_v4.eti`
//! (15.4 MB live ETI from stock `eti-cmdline-airspy` on the same capture,
//! 2506 frames). Full byte-identical compare requires the dab-cli pipeline
//! orchestrator (dab-viterbi → dab-descramble → dab-fec → ETI(NI) framing)
//! which is the next slice. This test is **black-box** validation of the
//! OFDM chain in isolation:
//!
//! - Resampled stream length matches the 60M-sample (20s) capture.
//! - Null-symbol detector finds ≈ 208 dips at the 96 ms frame cadence.
//! - PRS sync converges (fractional CFO bounded, FFT band-ratio ≫ 1).
//! - Integer CFO is small for a centred capture.
//! - Stage 6 differential consumes the PRS and produces 75 data symbols per
//!   frame without NaNs.
//! - Stage 7 soft bits land in `[-127, +127]`, have non-trivial energy, and
//!   are roughly balanced (≈ half negative, half positive) — confirms the
//!   demap is producing real information, not a stuck pattern.
//!
//! The 240 MB capture is Git-LFS / not committed here. Provide a local copy:
//!
//! ```sh
//! export DAB_RS_K8B_V4_IQ=/path/to/k8b_v4.iq
//! cargo test -p dab-ofdm --test k8b_v4_ofdm_chain -- --include-ignored --nocapture
//! ```

use num_complex::Complex;

use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::{
    detect_integer_cfo, CpSync, DifferentialReference, DqpskDemap, Nco, NullDetector, Resampler,
    SymbolFft,
};

const DEFAULT_IQ: &str =
    "/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_v4.iq";

const TS: usize = 2552; // full OFDM symbol @ 2.048 MSPS (T_g + T_u = 504 + 2048)
const NULL_LEN: usize = 2656; // null-symbol length in samples
const SYMBOLS_PER_FRAME: usize = 76; // 1 PRS + 75 data symbols per DAB frame
const FS: f64 = 2_048_000.0;

fn capture_path() -> std::path::PathBuf {
    std::env::var("DAB_RS_K8B_V4_IQ")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from(DEFAULT_IQ))
}

/// Copy `TS` samples at `cp_start`, remove the fractional CFO with the NCO,
/// FFT the symbol. Returns the natural-order 2048-bin spectrum.
fn fft_symbol_corrected(
    resampled: &[Complex<f32>],
    cp_start: usize,
    cfo_hz: f64,
    sfft: &mut SymbolFft,
) -> Vec<Complex<f32>> {
    let mut region = resampled[cp_start..cp_start + TS].to_vec();
    // mix(-cfo) removes a +cfo offset (NCO sign convention; see nco.rs).
    Nco::new(FS).mix(&mut region, -cfo_hz);
    sfft.fft_symbol(&region)
}

#[test]
#[ignore = "requires the k8b_v4.iq oracle capture; run with --include-ignored"]
fn k8b_v4_full_chain_stages_1_through_7() {
    let path = capture_path();
    if !path.exists() {
        eprintln!("skipping: capture not found at {}", path.display());
        return;
    }

    // ---- Stage 1: read + resample 3 MSPS → 2.048 MSPS ----
    let mut reader =
        IqFileReader::open(&path, IqFormat::Cs16Le, 3_000_000).expect("open capture");
    let mut resampler = Resampler::new_3m_to_2048k();
    let mut resampled: Vec<Complex<f32>> = Vec::with_capacity(41_000_000);
    let mut buf = vec![Complex::new(0.0f32, 0.0f32); 1 << 20];
    loop {
        let n = reader.read_samples(&mut buf).expect("read");
        if n == 0 {
            break;
        }
        resampled.extend_from_slice(&resampler.process(&buf[..n]));
    }
    // 20 s × 2.048 MSPS ≈ 40.96 M samples (LFS file is exactly 60 M @ 3 MSPS).
    assert!(
        resampled.len() > 40_000_000,
        "resampled len {} too small",
        resampled.len()
    );

    // ---- Stage 2: null detection ----
    let nulls = NullDetector::new(2_048_000).detect(&resampled);
    // 20 s / 96 ms ≈ 208 frames → expect ≈ 208 null dips.
    assert!(
        nulls.positions.len() >= 180 && nulls.positions.len() <= 230,
        "unexpected null count {}",
        nulls.positions.len()
    );

    // ---- Stage 3 + band-ratio scan: pick the best-aligned PRS across the
    // middle third of the capture. Marginal indoor SNR (~12 dB) gives
    // frame-to-frame variation; the textbook mid-capture pick can land on a
    // fading dip with active/guard ratio dipping near 9 dB. Scanning a few
    // candidate nulls and keeping the best ratio is the small amount of
    // robustness this black-box test needs to stay reliable. (The full
    // pipeline, when wired up, will instead track sync continuously rather
    // than pick a single frame.) ----
    let cp = CpSync::mode_i();
    let mut sfft = SymbolFft::mode_i();
    let p = |z: &Complex<f32>| (z.re as f64) * (z.re as f64) + (z.im as f64) * (z.im as f64);

    let lo = nulls.positions.len() / 3;
    let hi = (2 * nulls.positions.len() / 3).max(lo + 1);
    let mut best_prs_start = 0_usize;
    let mut best_cfo_hz = 0.0_f64;
    let mut best_band_ratio_db = f64::NEG_INFINITY;
    let mut best_prs_spec: Vec<Complex<f32>> = Vec::new();

    for &null_pos in &nulls.positions[lo..hi] {
        let prs_start = cp.fine_time(&resampled, null_pos + NULL_LEN, TS);
        if prs_start + TS > resampled.len() {
            continue;
        }
        let cfo_hz = cp.estimate_cfo_hz(&resampled, prs_start, 50) as f64;
        if cfo_hz.abs() > 500.0 {
            continue;
        }
        let prs_spec = fft_symbol_corrected(&resampled, prs_start, cfo_hz, &mut sfft);
        let active_e: f64 =
            (1..=768).chain(1280..=2047).map(|i| p(&prs_spec[i])).sum();
        let guard_e: f64 = (769..=1279).map(|i| p(&prs_spec[i])).sum();
        if guard_e <= 0.0 {
            continue;
        }
        let active_per_bin = active_e / 1536.0;
        let guard_per_bin = guard_e / 511.0;
        let band_ratio_db = 10.0 * (active_per_bin / guard_per_bin).log10();
        if band_ratio_db > best_band_ratio_db {
            best_band_ratio_db = band_ratio_db;
            best_prs_start = prs_start;
            best_cfo_hz = cfo_hz;
            best_prs_spec = prs_spec;
        }
    }

    assert!(
        !best_prs_spec.is_empty(),
        "no usable PRS frame in the middle third of the capture"
    );
    let prs_start = best_prs_start;
    let cfo_hz = best_cfo_hz;
    let band_ratio_db = best_band_ratio_db;
    let prs_spec = best_prs_spec;

    // Active-band energy must dominate the guard band by a wide margin; a
    // chain bug (mis-resampled, wrong CP offset, wrong FFT size) collapses
    // this to ~0 dB, while real K8B reception consistently produces > 7 dB
    // on at least one frame in any 7-second window. The threshold is set
    // well above the ~3 dB chance level for an unsynced FFT but loose
    // enough to accept the marginal indoor SNR of `k8b_v4`.
    assert!(
        band_ratio_db > 7.0,
        "PRS active/guard band ratio {band_ratio_db:.1} dB too low (sync/framing wrong?)"
    );
    assert_eq!(prs_spec.len(), 2048);

    let icfo = detect_integer_cfo(&prs_spec, 5);
    // For a centred capture the integer offset should be small; we don't
    // pin it to exactly 0 because the local oscillator is not perfectly
    // centred on the channel.
    assert!(
        icfo.offset.abs() <= 2,
        "unexpected integer CFO {} (peak {:.3} runner {:.3})",
        icfo.offset,
        icfo.peak,
        icfo.runner_up
    );

    // ---- Stage 6: seed the differential reference with the PRS ----
    let mut diff_ref = DifferentialReference::new();
    diff_ref.seed_prs(&prs_spec);
    assert!(diff_ref.is_seeded());

    // ---- Stages 5+6+7: extract 75 data symbols → soft bits ----
    let demap = DqpskDemap::mode_i();
    assert_eq!(demap.out_size(), 3072);

    let mut total_soft_bits = 0_usize;
    let mut zero_bits = 0_usize;
    let mut neg_bits = 0_i64;
    let mut pos_bits = 0_i64;
    let mut abs_sum: i64 = 0;
    let mut max_abs: i16 = 0;
    let mut data_syms_done = 0_usize;

    for s in 1..SYMBOLS_PER_FRAME {
        let cp_start = prs_start + s * TS;
        if cp_start + TS > resampled.len() {
            break;
        }
        let spec = fft_symbol_corrected(&resampled, cp_start, cfo_hz, &mut sfft);
        let diff = diff_ref.step(&spec);
        assert_eq!(diff.len(), 2048);
        // No NaNs allowed in the differential output.
        assert!(diff.iter().all(|z| z.re.is_finite() && z.im.is_finite()));

        let bits = demap.demap(&diff);
        assert_eq!(bits.len(), 3072);

        for &b in &bits {
            assert!((-127..=127).contains(&b), "soft bit {b} out of range");
            total_soft_bits += 1;
            if b == 0 {
                zero_bits += 1;
            } else if b < 0 {
                neg_bits += 1;
            } else {
                pos_bits += 1;
            }
            let a = b.unsigned_abs() as i64;
            abs_sum += a;
            if (a as i16) > max_abs {
                max_abs = a as i16;
            }
        }
        data_syms_done += 1;
    }

    assert!(
        data_syms_done >= SYMBOLS_PER_FRAME - 1,
        "should demap all 75 data symbols of the frame, got {data_syms_done}"
    );
    assert_eq!(total_soft_bits, 75 * 3072);

    // Sanity: at SNR 12 dB on K8B, most soft bits are far from zero. The mean
    // absolute soft-bit value over all 230_400 bits in a frame should sit
    // comfortably above 60 (out of a max of 127) — a stuck/inverted chain or
    // a freq-de-interleaver bug would tank this.
    let mean_abs = abs_sum as f64 / total_soft_bits as f64;
    assert!(
        mean_abs > 60.0,
        "demap mean(|soft|) {mean_abs:.2} too low — chain likely broken"
    );

    // Sanity: balance — neither sign should dominate (would indicate stuck DC).
    let pos_frac = pos_bits as f64 / total_soft_bits as f64;
    let neg_frac = neg_bits as f64 / total_soft_bits as f64;
    assert!(
        pos_frac > 0.30 && pos_frac < 0.70,
        "positive-bit fraction {pos_frac:.3} skewed (neg {neg_frac:.3})"
    );

    eprintln!(
        "OK: resampled={}  nulls={}  prs_start={prs_start}  cfo={cfo_hz:.1}Hz  \
         band_ratio={band_ratio_db:.1}dB  int_cfo={} (peak {:.3} runner {:.3})  \
         data_syms={data_syms_done}  soft_bits={total_soft_bits}  \
         mean(|b|)={mean_abs:.2}  max(|b|)={max_abs}  \
         pos_frac={pos_frac:.3}  neg_frac={neg_frac:.3}  zero_frac={:.3}",
        resampled.len(),
        nulls.positions.len(),
        icfo.offset,
        icfo.peak,
        icfo.runner_up,
        zero_bits as f64 / total_soft_bits as f64,
    );
}
