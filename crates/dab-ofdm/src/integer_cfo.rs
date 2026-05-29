//! Stage 4b of the OFDM sync chain — integer carrier-frequency offset (CFO).
//!
//! After the *fractional* CFO is removed ([`crate::nco`]) and the
//! Phase-Reference Symbol (PRS) is transformed to the frequency domain
//! ([`crate::symbol_fft`]), any residual offset is an integer number `δ` of
//! carrier spacings. An integer CFO of `δ` carriers shifts the whole received
//! spectrum toward higher bins by `δ`, so:
//!
//! ```text
//! R[bin(k)] ≈ ref[bin(k − δ)]
//! ```
//!
//! where `ref = phase_reference()` is the known unit-magnitude PRS spectrum
//! (zero on DC and the guard band). We detect `δ` by coherently correlating the
//! received PRS spectrum against the reference over the active carriers, for
//! each candidate shift in `[−range, +range]`, and pick the `δ` that maximizes
//! the correlation magnitude:
//!
//! ```text
//! C(δ) = | Σ_{active k} R[bin(k)] · conj(ref[bin(k − δ)]) |
//! ```
//!
//! normalized by `Σ |R|` over the terms that contributed. Bins that fall on DC
//! or out of range are skipped.
//!
//! The eti-stuff oracle (`phasereference.cpp::estimateOffset`) computes an
//! equivalent quantity via adjacent-carrier phase differences around carrier 0,
//! returning `index − T_u`; this magnitude-correlation form is simpler and more
//! robust to noise.
//!
//! # Sign convention
//!
//! `δ > 0` means the received spectrum is shifted toward **higher** FFT bins.
//! The integer carrier frequency offset in Hz is `δ × carrier_diff`
//! (`carrier_diff = 1000 Hz` for Mode I).

use num_complex::Complex;

use crate::params::DabParams;
use crate::phasereference::phase_reference;

/// Result of integer-CFO detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntegerCfoResult {
    /// Detected integer carrier offset `δ` (positive = spectrum shifted high).
    pub offset: i32,
    /// Correlation magnitude at the winning `δ` (normalized).
    pub peak: f32,
    /// Correlation magnitude of the second-best `δ` (normalized).
    pub runner_up: f32,
}

/// Detect the integer carrier offset of a fractionally-corrected PRS symbol.
///
/// `prs_spectrum` is the 2048-bin natural-order FFT of the PRS symbol (after
/// fractional CFO removal). `range` bounds the search to `δ ∈ [−range, range]`.
/// Returns the best `δ`, its normalized correlation peak, and the runner-up.
///
/// # Panics
/// Panics if `prs_spectrum.len()` does not equal the FFT size (`T_u = 2048`).
pub fn detect_integer_cfo(prs_spectrum: &[Complex<f32>], range: i32) -> IntegerCfoResult {
    let p = DabParams::mode_i();
    let t_u = p.t_u as usize;
    assert_eq!(prs_spectrum.len(), t_u, "spectrum length must be T_u");
    let half = (p.carriers / 2) as i32; // 768
    let n = t_u as i32;

    let ref_table = phase_reference();

    // Active-carrier indices k ∈ {−768..=−1, +1..=+768} mapped to FFT bins.
    // carrier k>0 -> bin k ; carrier k<0 -> bin n + k.
    let bin_of = |k: i32| -> usize {
        if k > 0 {
            k as usize
        } else {
            (n + k) as usize
        }
    };

    let active: Vec<i32> = (-half..=half).filter(|&k| k != 0).collect();

    let mut best = IntegerCfoResult {
        offset: 0,
        peak: f32::NEG_INFINITY,
        runner_up: f32::NEG_INFINITY,
    };

    for delta in -range..=range {
        let mut acc = Complex::new(0.0_f32, 0.0);
        let mut norm = 0.0_f32;
        for &k in &active {
            // Reference carrier index is k − δ; skip if it leaves the active
            // band or lands on DC.
            let kr = k - delta;
            if kr == 0 || kr < -half || kr > half {
                continue;
            }
            let r = prs_spectrum[bin_of(k)];
            let rf = ref_table[bin_of(kr)];
            acc += r * rf.conj();
            norm += r.norm();
        }
        let mag = if norm > 0.0 { acc.norm() / norm } else { 0.0 };

        if mag > best.peak {
            best.runner_up = best.peak;
            best.peak = mag;
            best.offset = delta;
        } else if mag > best.runner_up {
            best.runner_up = mag;
        }
    }

    // If nothing beat NEG_INFINITY for runner-up (range == 0), clamp to 0.
    if !best.runner_up.is_finite() {
        best.runner_up = 0.0;
    }
    best
}

/// Adjacent-carrier phase-difference integer CFO estimator —
/// `phaseReference::estimateOffset` from eti-stuff (`phasereference.cpp:114`)
/// ported verbatim.
///
/// Searches `δ ∈ [−SEARCH_RANGE/2, SEARCH_RANGE/2)` (= [−35, 35)) and returns
/// the δ that minimizes the L1 phase residual between received and reference
/// adjacent-carrier differences. This estimator is **timing-invariant**:
/// adjacent-carrier phase differences cancel any per-carrier phase ramp from
/// sub-sample timing offsets, so it works correctly even when CP-autocorrelation
/// fine-time picks a sample one off the true symbol start.
///
/// In contrast, [`detect_integer_cfo`] uses absolute magnitude correlation
/// against the PRS reference table, which is sensitive to timing residuals.
///
/// # Convention
///
/// `δ > 0` means the received spectrum is shifted toward **higher** FFT bins.
/// Apply the correction by rotating the spectrum left by δ bins
/// (`rotate_spectrum`).
pub fn estimate_offset_eti(prs_spectrum: &[Complex<f32>]) -> i32 {
    const DIFF_LENGTH: usize = 50;
    const SEARCH_RANGE_HALF: i32 = 35;

    let t_u = prs_spectrum.len() as i32;
    let ref_table = phase_reference();

    // Reference adjacent-carrier phase differences around carrier 0
    // (i.e. low positive carriers 1..51 of the ideal PRS).
    // pd_ref[j] = ref[bin(j+1)] * conj(ref[bin(j+2)])
    let phase_diffs: Vec<Complex<f32>> = (0..DIFF_LENGTH)
        .map(|j| {
            let j = j as i32;
            let ind1 = ((t_u + j + 1).rem_euclid(t_u)) as usize;
            let ind2 = ((t_u + j + 2).rem_euclid(t_u)) as usize;
            ref_table[ind1] * ref_table[ind2].conj()
        })
        .collect();

    let mut best_diff = f32::INFINITY;
    let mut best_index = t_u; // default i = T_u → δ = 0

    // Candidate i ∈ [T_u - 35, T_u + 35); returned δ = i − T_u ∈ [−35, 35).
    for i in (t_u - SEARCH_RANGE_HALF)..(t_u + SEARCH_RANGE_HALF) {
        let mut diff = 0.0_f32;
        for j in 0..DIFF_LENGTH {
            let j = j as i32;
            let ind1 = ((i + j + 1).rem_euclid(t_u)) as usize;
            let ind2 = ((i + j + 2).rem_euclid(t_u)) as usize;
            let pd = prs_spectrum[ind1] * prs_spectrum[ind2].conj();
            diff += (pd * phase_diffs[j as usize].conj()).arg().abs();
        }
        if diff < best_diff {
            best_diff = diff;
            best_index = i;
        }
    }

    best_index - t_u
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::Fft;
    use crate::symbol_fft::SymbolFft;

    /// Minimal in-test xorshift32 PRNG.
    struct XorShift32(u32);
    impl XorShift32 {
        fn next_f32(&mut self) -> f32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            (x as f32 / u32::MAX as f32) * 2.0 - 1.0
        }
    }

    /// Build a PRS time-domain symbol (with CP) from a 2048-bin spectrum.
    fn prs_symbol_from(spectrum: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let p = DabParams::mode_i();
        let t_u = p.t_u as usize;
        let t_g = p.t_g as usize;
        let fft = Fft::new(t_u);
        let mut useful = spectrum.to_vec();
        fft.inverse(&mut useful);
        let mut sym = Vec::with_capacity(t_g + t_u);
        sym.extend_from_slice(&useful[t_u - t_g..]);
        sym.extend_from_slice(&useful);
        sym
    }

    /// Shift the active-carrier spectrum by δ bins (δ>0 = toward higher bins),
    /// reproducing a received integer CFO of +δ carriers.
    fn shift_spectrum(spectrum: &[Complex<f32>], delta: i32) -> Vec<Complex<f32>> {
        let p = DabParams::mode_i();
        let n = p.t_u as i32;
        let half = (p.carriers / 2) as i32;
        let bin_of = |k: i32| -> usize {
            if k > 0 {
                k as usize
            } else {
                (n + k) as usize
            }
        };
        let mut out = vec![Complex::new(0.0_f32, 0.0); n as usize];
        // received[bin(k)] = ref[bin(k − δ)]
        for k in (-half..=half).filter(|&k| k != 0) {
            let kr = k - delta;
            if kr == 0 || kr < -half || kr > half {
                continue;
            }
            out[bin_of(k)] = spectrum[bin_of(kr)];
        }
        out
    }

    #[test]
    fn zero_offset() {
        let prs = phase_reference();
        let sym = prs_symbol_from(&prs);
        let mut sf = SymbolFft::mode_i();
        let spec = sf.fft_symbol(&sym);

        let res = detect_integer_cfo(&spec, 5);
        assert_eq!(res.offset, 0);
        assert!(
            res.peak > 2.0 * res.runner_up,
            "peak {} not dominant vs runner_up {}",
            res.peak,
            res.runner_up
        );
    }

    #[test]
    fn injected_positive_offset() {
        let prs = phase_reference();
        let shifted = shift_spectrum(&prs, 2);
        let sym = prs_symbol_from(&shifted);
        let mut sf = SymbolFft::mode_i();
        let spec = sf.fft_symbol(&sym);

        let res = detect_integer_cfo(&spec, 5);
        assert_eq!(res.offset, 2);
    }

    #[test]
    fn injected_negative_offset() {
        let prs = phase_reference();
        let shifted = shift_spectrum(&prs, -3);
        let sym = prs_symbol_from(&shifted);
        let mut sf = SymbolFft::mode_i();
        let spec = sf.fft_symbol(&sym);

        let res = detect_integer_cfo(&spec, 5);
        assert_eq!(res.offset, -3);
    }

    #[test]
    fn offset_recovered_under_noise() {
        let prs = phase_reference();
        let shifted = shift_spectrum(&prs, 1);
        let mut sym = prs_symbol_from(&shifted);

        // Light additive complex noise.
        let mut rng = XorShift32(0xDEAD_BEEF);
        for z in sym.iter_mut() {
            z.re += 0.05 * rng.next_f32();
            z.im += 0.05 * rng.next_f32();
        }

        let mut sf = SymbolFft::mode_i();
        let spec = sf.fft_symbol(&sym);
        let res = detect_integer_cfo(&spec, 5);
        assert_eq!(res.offset, 1);
    }
}
