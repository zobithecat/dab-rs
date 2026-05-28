//! Stage 7 of the OFDM sync chain — π/4-DQPSK soft-bit demap with frequency
//! de-interleaving.
//!
//! Given the per-bin differential output from Stage 6
//! ([`crate::channel_eq::DifferentialReference`]), produces `2K = 3072` soft
//! bits in **logical (de-interleaved) order**, matching the eti-stuff oracle
//! `ofdm-processor.cpp::processBlock` exactly (lines 472–485):
//!
//! ```text
//! for i in 0..K {
//!     bin     = freq_interleaver.map_in(i) as FFT bin;  // signed carrier; wrap negatives by +T_u
//!     r       = differential[bin];
//!     a       = jan_abs(r) = |r.re| + |r.im|;
//!     ibits[i]     = -r.re / a * 127;
//!     ibits[K+i]   = -r.im / a * 127;
//! }
//! ```
//!
//! Output type is `i16` to match the downstream `dab-viterbi` API
//! (`EepProtection::deconvolve(soft: &[i16])`); values lie in `[-127, +127]`.
//!
//! # Polarity convention (`+ ⇒ bit 1`)
//!
//! The leading minus in `-r.re/a*127` flips the sign so that a positive
//! decision metric maps to bit value 1 in the Viterbi decoder, per the
//! convention documented in the project README *Discovered subtleties*
//! ("Viterbi soft-bit polarity convention"). Inverting this — i.e. dropping the
//! minus — silently produces byte-flipped Viterbi output downstream.
//!
//! # Layout
//!
//! ```text
//! soft_bits[0..K)        // I-channel soft bits (de-interleaved)
//! soft_bits[K..2K)       // Q-channel soft bits (de-interleaved)
//! ```
//!
//! with `K = 1536` for Mode I.

use num_complex::Complex;

use crate::freq_interleaver::FreqInterleaver;
use crate::params::DabParams;

/// `jan_abs(z) = |re| + |im|` — the L1-norm magnitude approximation used by the
/// eti-stuff oracle (`dab-constants.h::jan_abs`). Chosen over the true
/// magnitude (`sqrt(re² + im²)`) for byte-identical correspondence with the
/// oracle, not for performance reasons.
#[inline]
pub fn jan_abs(z: Complex<f32>) -> f32 {
    z.re.abs() + z.im.abs()
}

/// Mode I π/4-DQPSK soft-bit demapper with frequency de-interleaving.
pub struct DqpskDemap {
    interleaver: FreqInterleaver,
    t_u: usize,
    carriers: usize,
}

impl DqpskDemap {
    /// Build the Mode I demapper (`T_u = 2048`, `K = 1536`, ETSI frequency
    /// permutation).
    pub fn mode_i() -> Self {
        let p = DabParams::mode_i();
        DqpskDemap {
            interleaver: FreqInterleaver::mode_i(),
            t_u: p.t_u as usize,
            carriers: p.carriers as usize,
        }
    }

    /// Number of soft bits produced per symbol (`2 · K = 3072` for Mode I).
    pub fn out_size(&self) -> usize {
        2 * self.carriers
    }

    /// Demap one symbol's differential spectrum to soft bits.
    ///
    /// `diff_spec` is the natural-order per-FFT-bin differential output from
    /// Stage 6 (length `T_u = 2048`). Returns a length-`3072` vector with
    /// I bits at `[0, K)` and Q bits at `[K, 2K)`, in de-interleaved order
    /// ready for the Viterbi decoder.
    ///
    /// # Panics
    /// Panics if `diff_spec.len() != T_u`.
    pub fn demap(&self, diff_spec: &[Complex<f32>]) -> Vec<i16> {
        assert_eq!(
            diff_spec.len(),
            self.t_u,
            "differential spectrum must be length T_u"
        );
        let k = self.carriers;
        let t_u = self.t_u as i32;
        let mut bits = vec![0_i16; 2 * k];
        for i in 0..k {
            let carrier = self.interleaver.map_in(i) as i32;
            // mapIn returns a signed carrier index in {-768..=-1, 1..=768};
            // wrap negatives into the FFT natural-order bin range.
            let bin = if carrier > 0 {
                carrier as usize
            } else {
                (carrier + t_u) as usize
            };
            let r = diff_spec[bin];
            let a = jan_abs(r);
            if a == 0.0 {
                // Pathological all-zero bin — emit hard zero (matches the
                // oracle's behaviour: a zero numerator stays zero, and the
                // oracle's lack of div-by-zero guard relies on FFT-bin energy
                // always being non-zero on active carriers in real streams).
                bits[i] = 0;
                bits[k + i] = 0;
            } else {
                // Match the oracle's int16_t truncation toward zero. Rust's
                // `as i16` cast from f32 truncates toward zero and saturates
                // out-of-range; our values are in [-127, +127] so saturation
                // never triggers.
                bits[i] = (-r.re / a * 127.0) as i16;
                bits[k + i] = (-r.im / a * 127.0) as i16;
            }
        }
        bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jan_abs_is_l1_norm() {
        assert_eq!(jan_abs(Complex::new(3.0, 4.0)), 7.0);
        assert_eq!(jan_abs(Complex::new(-3.0, 4.0)), 7.0);
        assert_eq!(jan_abs(Complex::new(0.0, 0.0)), 0.0);
        assert_eq!(jan_abs(Complex::new(1.5, -2.5)), 4.0);
    }

    #[test]
    fn out_size_matches_mode_i() {
        assert_eq!(DqpskDemap::mode_i().out_size(), 3072);
    }

    #[test]
    fn pure_real_positive_yields_minus_127_i_zero_q() {
        // r = (1, 0) → I bit = -1/1 * 127 = -127, Q bit = 0/1 * 127 = 0.
        let demap = DqpskDemap::mode_i();
        let t_u = DabParams::mode_i().t_u as usize;
        let mut spec = vec![Complex::new(0.0_f32, 0.0); t_u];
        // Drive every active bin with (1, 0).
        let il = FreqInterleaver::mode_i();
        for i in 0..1536 {
            let c = il.map_in(i) as i32;
            let bin = if c > 0 { c as usize } else { (c + t_u as i32) as usize };
            spec[bin] = Complex::new(1.0, 0.0);
        }
        let bits = demap.demap(&spec);
        assert_eq!(bits.len(), 3072);
        for &b in &bits[..1536] {
            assert_eq!(b, -127, "I bit must be -127 for real-positive input");
        }
        for &b in &bits[1536..] {
            assert_eq!(b, 0, "Q bit must be 0 for real-positive input");
        }
    }

    #[test]
    fn pure_imag_positive_yields_zero_i_minus_127_q() {
        // r = (0, 1) → I bit = 0, Q bit = -1/1 * 127 = -127.
        let demap = DqpskDemap::mode_i();
        let t_u = DabParams::mode_i().t_u as usize;
        let mut spec = vec![Complex::new(0.0_f32, 0.0); t_u];
        let il = FreqInterleaver::mode_i();
        for i in 0..1536 {
            let c = il.map_in(i) as i32;
            let bin = if c > 0 { c as usize } else { (c + t_u as i32) as usize };
            spec[bin] = Complex::new(0.0, 1.0);
        }
        let bits = demap.demap(&spec);
        for &b in &bits[..1536] {
            assert_eq!(b, 0, "I bit must be 0 for pure imag input");
        }
        for &b in &bits[1536..] {
            assert_eq!(b, -127, "Q bit must be -127 for imag-positive input");
        }
    }

    #[test]
    fn negative_real_input_gives_positive_bit_per_polarity_convention() {
        // r = (-1, 0) → I bit = -(-1)/1 * 127 = +127.
        // Polarity reminder: positive soft bit ⇒ decoded bit value 1 (README).
        let demap = DqpskDemap::mode_i();
        let t_u = DabParams::mode_i().t_u as usize;
        let mut spec = vec![Complex::new(0.0_f32, 0.0); t_u];
        let il = FreqInterleaver::mode_i();
        // Just probe one carrier to keep the test focused.
        let c = il.map_in(0) as i32;
        let bin = if c > 0 { c as usize } else { (c + t_u as i32) as usize };
        spec[bin] = Complex::new(-1.0, 0.0);

        let bits = demap.demap(&spec);
        assert_eq!(bits[0], 127, "I bit for real-negative must be +127");
    }

    #[test]
    fn deinterleaving_routes_bins_to_logical_indices() {
        // For each logical i, demap reads the spectrum at the bin corresponding
        // to carrier = map_in(i). Place a unique value at one specific bin and
        // verify it lands at the matching logical index only.
        let demap = DqpskDemap::mode_i();
        let il = FreqInterleaver::mode_i();
        let t_u = DabParams::mode_i().t_u as usize;
        let mut spec = vec![Complex::new(0.0_f32, 0.0); t_u];

        let probe_i = 42_usize;
        let carrier = il.map_in(probe_i) as i32;
        let bin = if carrier > 0 {
            carrier as usize
        } else {
            (carrier + t_u as i32) as usize
        };
        spec[bin] = Complex::new(1.0, 0.0); // I=-127, Q=0 at this carrier

        let bits = demap.demap(&spec);
        assert_eq!(bits[probe_i], -127, "energy must land at logical index {probe_i}");
        // All other logical indices stay at 0 (their bins are still zero).
        for (j, &b) in bits[..1536].iter().enumerate() {
            if j != probe_i {
                assert_eq!(b, 0, "spurious I energy at index {j}");
            }
        }
        for &b in &bits[1536..] {
            assert_eq!(b, 0, "spurious Q energy");
        }
    }

    #[test]
    fn zero_input_emits_zero_safely() {
        // No NaN from division — the all-zero bin guard kicks in.
        let demap = DqpskDemap::mode_i();
        let t_u = DabParams::mode_i().t_u as usize;
        let spec = vec![Complex::new(0.0_f32, 0.0); t_u];
        let bits = demap.demap(&spec);
        assert_eq!(bits.len(), 3072);
        for &b in &bits {
            assert_eq!(b, 0);
        }
    }
}
