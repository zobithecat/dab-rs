//! Stage 6 of the OFDM sync chain — differential per-carrier reference.
//!
//! DAB Mode I uses π/4-DQPSK: each data symbol's per-carrier phase is encoded
//! *relative to the previous symbol on the same carrier*. The slow channel
//! response `H[k]` cancels in that conjugate product, so an explicit per-carrier
//! division by an estimated `Ĥ[k]` is unnecessary. The *previous symbol's
//! spectrum is itself the per-carrier reference*.
//!
//! This mirrors the eti-stuff oracle exactly
//! (`eti-cmdline/src/ofdm/ofdm-processor.cpp` lines 462–495):
//!
//! ```text
//!   // processBlock_0 (the PRS) seeds the reference:
//!   referenceFase[bin] = fft_buffer[bin];
//!
//!   // processBlock (each subsequent data symbol):
//!   r1                  = fft_buffer[bin] * conj(referenceFase[bin]);
//!   referenceFase[bin]  = fft_buffer[bin];   // pre-multiply value
//! ```
//!
//! `r1` is the per-carrier differential output. Stage 7 ([`crate::dqpsk_demap`])
//! consumes it to emit soft bits.
//!
//! # Bin order
//!
//! Inputs and outputs are length `T_u = 2048` in natural FFT order
//! (bin 0 = DC, positive carriers at bins `1..=768`, negative carriers wrapped
//! at bins `T_u-768 ..= T_u-1`). Inactive bins (DC, guard band) are kept in the
//! buffer but downstream demap (Stage 7) only reads the 1536 active bins.

use num_complex::Complex;

use crate::params::DabParams;

/// Differential reference manager for π/4-DQPSK demodulation.
///
/// Holds the previous symbol's full `T_u`-bin spectrum and produces per-carrier
/// differential outputs on each call to [`step`](Self::step).
pub struct DifferentialReference {
    /// Previous-symbol spectrum, indexed by FFT bin (length `T_u`).
    prev: Vec<Complex<f32>>,
    /// `true` once a PRS has seeded `prev`. Used by [`step`] to refuse running
    /// before a reference exists (calling on an all-zero `prev` would yield an
    /// all-zero differential, masking the bug).
    seeded: bool,
}

impl DifferentialReference {
    /// Construct an empty reference. Call [`seed_prs`](Self::seed_prs) with the
    /// FFT of the Phase-Reference Symbol before the first [`step`](Self::step).
    pub fn new() -> Self {
        let t_u = DabParams::mode_i().t_u as usize;
        DifferentialReference {
            prev: vec![Complex::new(0.0_f32, 0.0); t_u],
            seeded: false,
        }
    }

    /// Seed the reference with the PRS spectrum (mirrors `processBlock_0`).
    ///
    /// `prs_spectrum` is the FFT of the Phase-Reference Symbol in natural order
    /// (length `T_u = 2048`). After this call, [`step`](Self::step) will demap
    /// the first data symbol against the PRS.
    ///
    /// # Panics
    /// Panics if `prs_spectrum.len() != T_u`.
    pub fn seed_prs(&mut self, prs_spectrum: &[Complex<f32>]) {
        assert_eq!(
            prs_spectrum.len(),
            self.prev.len(),
            "PRS spectrum length must be T_u"
        );
        self.prev.copy_from_slice(prs_spectrum);
        self.seeded = true;
    }

    /// Demap one data symbol against the running reference.
    ///
    /// Returns the per-bin differential `r[bin] = current[bin] * conj(prev[bin])`
    /// (length `T_u`) and updates the internal reference to `current` (the
    /// pre-multiply value, exactly as the oracle does).
    ///
    /// # Panics
    /// Panics if not seeded (call [`seed_prs`](Self::seed_prs) first) or if
    /// `current.len() != T_u`.
    pub fn step(&mut self, current: &[Complex<f32>]) -> Vec<Complex<f32>> {
        assert!(
            self.seeded,
            "DifferentialReference::step before seed_prs"
        );
        assert_eq!(
            current.len(),
            self.prev.len(),
            "data symbol length must be T_u"
        );
        let mut out = Vec::with_capacity(current.len());
        for (c, p) in current.iter().zip(self.prev.iter()) {
            out.push(c * p.conj());
        }
        // Update reference to the *pre-multiply* current spectrum.
        self.prev.copy_from_slice(current);
        out
    }

    /// Reset the reference to "unseeded". Useful between independent frames or
    /// after a re-sync.
    pub fn reset(&mut self) {
        for z in self.prev.iter_mut() {
            *z = Complex::new(0.0, 0.0);
        }
        self.seeded = false;
    }

    /// `true` once a PRS has been ingested.
    pub fn is_seeded(&self) -> bool {
        self.seeded
    }
}

impl Default for DifferentialReference {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_4;

    const T_U: usize = 2048;

    fn unit_vec(n: usize, fill: Complex<f32>) -> Vec<Complex<f32>> {
        vec![fill; n]
    }

    #[test]
    fn step_before_seed_panics() {
        let mut r = DifferentialReference::new();
        let sym = unit_vec(T_U, Complex::new(1.0, 0.0));
        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| r.step(&sym)));
        assert!(res.is_err(), "step before seed must panic");
    }

    #[test]
    fn seeded_with_identical_symbol_yields_squared_magnitude_real() {
        let mut r = DifferentialReference::new();
        let prs = unit_vec(T_U, Complex::new(0.6, 0.8)); // |z|^2 = 1.0
        let data = prs.clone();
        r.seed_prs(&prs);
        let out = r.step(&data);
        assert_eq!(out.len(), T_U);
        for z in &out {
            assert!((z.re - 1.0).abs() < 1e-6, "re {} ≠ 1", z.re);
            assert!(z.im.abs() < 1e-6, "im {} ≠ 0", z.im);
        }
    }

    #[test]
    fn differential_carries_only_phase_increment() {
        // Build prev with magnitude m and phase θ0; current with magnitude m and phase θ0+Δ.
        // The differential r = current * conj(prev) should have magnitude m² and phase Δ.
        let m: f32 = 1.7;
        let theta0: f32 = 0.3;
        let delta: f32 = FRAC_PI_4; // expected differential phase

        let prev = unit_vec(
            T_U,
            Complex::new(m * theta0.cos(), m * theta0.sin()),
        );
        let curr = unit_vec(
            T_U,
            Complex::new(m * (theta0 + delta).cos(), m * (theta0 + delta).sin()),
        );

        let mut r = DifferentialReference::new();
        r.seed_prs(&prev);
        let out = r.step(&curr);

        let expect_mag = m * m;
        for z in &out {
            assert!(
                (z.norm() - expect_mag).abs() < 1e-4,
                "|r| {} ≠ m² {}",
                z.norm(),
                expect_mag
            );
            assert!(
                (z.arg() - delta).abs() < 1e-4,
                "arg(r) {} ≠ Δ {}",
                z.arg(),
                delta
            );
        }
    }

    #[test]
    fn reference_advances_to_current() {
        // After step(s1), prev == s1. After step(s2), the differential carries
        // s2 * conj(s1), and a follow-up step(s3) gives s3 * conj(s2). The
        // sequence of references is exactly the sequence of inputs.
        let s1 = unit_vec(T_U, Complex::new(1.0, 0.0));
        let s2 = unit_vec(T_U, Complex::new(0.0, 1.0));
        let s3 = unit_vec(T_U, Complex::new(-1.0, 0.0));

        let mut r = DifferentialReference::new();
        // Seed with the "zeroth" symbol (acts like the PRS here).
        r.seed_prs(&s1);

        // step(s2): r = s2 * conj(s1) = (0,1)*(1,0) = (0,1).
        let out = r.step(&s2);
        for z in &out {
            assert!((z.re - 0.0).abs() < 1e-6 && (z.im - 1.0).abs() < 1e-6);
        }

        // step(s3): r = s3 * conj(s2) = (-1,0)*(0,-1) = (0,1).
        let out = r.step(&s3);
        for z in &out {
            assert!((z.re - 0.0).abs() < 1e-6 && (z.im - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn reset_clears_state() {
        let mut r = DifferentialReference::new();
        let prs = unit_vec(T_U, Complex::new(1.0, 0.0));
        r.seed_prs(&prs);
        assert!(r.is_seeded());
        r.reset();
        assert!(!r.is_seeded());
    }
}
