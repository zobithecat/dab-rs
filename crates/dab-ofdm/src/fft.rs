//! Thin `rustfft` wrapper over `Complex<f32>`.
//!
//! Provides forward and inverse transforms for the OFDM useful-part size
//! (`T_u = 2048`), generic over any FFT length.
//!
//! ## Normalization convention
//!
//! `rustfft` performs an **unnormalized** transform in both directions: a
//! forward followed by an inverse scales the data by `N`. To recover a true
//! round-trip identity, [`Fft::inverse`] divides its output by `N`, so that
//! `inverse(forward(x)) == x` (up to floating-point tolerance). The forward
//! transform is left unscaled, matching the oracle's `do_FFT` semantics.

use std::sync::Arc;

use num_complex::Complex;
use rustfft::{Fft as RustFft, FftPlanner};

/// Planned forward/inverse FFT pair for a fixed size.
pub struct Fft {
    size: usize,
    forward: Arc<dyn RustFft<f32>>,
    inverse: Arc<dyn RustFft<f32>>,
    inv_scale: f32,
}

impl Fft {
    /// Plan an FFT of length `size`.
    pub fn new(size: usize) -> Self {
        let mut planner = FftPlanner::<f32>::new();
        let forward = planner.plan_fft_forward(size);
        let inverse = planner.plan_fft_inverse(size);
        Fft {
            size,
            forward,
            inverse,
            inv_scale: 1.0 / size as f32,
        }
    }

    /// Plan the OFDM useful-part FFT (`T_u = 2048`).
    pub fn mode_i() -> Self {
        Self::new(2048)
    }

    /// The FFT length.
    pub fn size(&self) -> usize {
        self.size
    }

    /// Forward (unnormalized) FFT, in place.
    ///
    /// # Panics
    /// Panics if `buf.len() != self.size()`.
    pub fn forward(&self, buf: &mut [Complex<f32>]) {
        assert_eq!(buf.len(), self.size, "fft buffer length mismatch");
        self.forward.process(buf);
    }

    /// Inverse FFT, in place, normalized by `1/N` for a true round-trip identity.
    ///
    /// # Panics
    /// Panics if `buf.len() != self.size()`.
    pub fn inverse(&self, buf: &mut [Complex<f32>]) {
        assert_eq!(buf.len(), self.size, "fft buffer length mismatch");
        self.inverse.process(buf);
        for z in buf.iter_mut() {
            *z *= self.inv_scale;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal in-test xorshift32 PRNG (no external rng crate).
    struct XorShift32(u32);
    impl XorShift32 {
        fn next_f32(&mut self) -> f32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            // Map to [-1, 1).
            (x as f32 / u32::MAX as f32) * 2.0 - 1.0
        }
    }

    #[test]
    fn forward_inverse_round_trip() {
        let fft = Fft::mode_i();
        let n = fft.size();

        let mut rng = XorShift32(0x1234_5678);
        let original: Vec<Complex<f32>> = (0..n)
            .map(|_| Complex::new(rng.next_f32(), rng.next_f32()))
            .collect();

        let mut buf = original.clone();
        fft.forward(&mut buf);
        fft.inverse(&mut buf);

        for (a, b) in original.iter().zip(buf.iter()) {
            assert!((a.re - b.re).abs() < 1e-4, "re mismatch: {a:?} vs {b:?}");
            assert!((a.im - b.im).abs() < 1e-4, "im mismatch: {a:?} vs {b:?}");
        }
    }
}
