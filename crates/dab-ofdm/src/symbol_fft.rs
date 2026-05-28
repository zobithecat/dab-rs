//! Stage 5 of the OFDM sync chain — per-symbol FFT framing.
//!
//! With symbol timing and frequency recovered (Stages 1–4), each OFDM symbol is
//! transformed to the frequency domain. A DAB Mode I symbol is `T_s = 2552`
//! samples: a `T_g = 504`-sample cyclic-prefix guard followed by the
//! `T_u = 2048`-sample useful part. We discard the guard and FFT the useful
//! part, exactly as the oracle does in `eti-stuff/src/ofdm/ofdm-processor.cpp`
//! (`processBlock` runs `do_FFT` over `&inv[T_g]`, i.e. the buffer offset past
//! the guard interval).
//!
//! # Carrier ↔ bin convention (ETSI EN 300 401 §14.3)
//!
//! The 2048-point FFT is in natural order with bin 0 = DC. DAB places `K = 1536`
//! active carriers symmetrically about DC, skipping DC itself:
//!
//! ```text
//! carrier k > 0  ->  FFT bin k             (k = 1..=768)
//! carrier k < 0  ->  FFT bin 2048 + k      (bins 1280..=2047)
//! ```
//!
//! The active-carrier vector ([`active_carriers`](SymbolFft::active_carriers))
//! has length 1536 with the *negative* carriers first:
//!
//! ```text
//! index 0    -> carrier −768  (bin 1280)
//! index 767  -> carrier  −1   (bin 2047)
//! index 768  -> carrier  +1   (bin 1)
//! index 1535 -> carrier +768  (bin 768)
//! ```

use num_complex::Complex;

use crate::fft::Fft;
use crate::params::DabParams;

/// Per-symbol FFT framer for DAB Mode I.
pub struct SymbolFft {
    fft: Fft,
    /// Useful-part length / FFT size (`T_u = 2048`).
    t_u: usize,
    /// Guard-interval length (`T_g = 504`).
    t_g: usize,
    /// Number of active carriers (`K = 1536`).
    carriers: usize,
}

impl SymbolFft {
    /// Construct the DAB Mode I framer (`T_u = 2048`, `T_g = 504`, `K = 1536`).
    pub fn mode_i() -> Self {
        let p = DabParams::mode_i();
        SymbolFft {
            fft: Fft::new(p.t_u as usize),
            t_u: p.t_u as usize,
            t_g: p.t_g as usize,
            carriers: p.carriers as usize,
        }
    }

    /// FFT one OFDM symbol.
    ///
    /// `sym` must contain at least `T_s = T_u + T_g` samples starting at the
    /// cyclic prefix. The guard (`T_g`) is skipped and the `T_u` useful samples
    /// are transformed, returning the 2048-bin spectrum in natural order
    /// (bin 0 = DC). The transform is **not** normalized (matches the oracle).
    ///
    /// # Panics
    /// Panics if `sym.len() < T_g + T_u`.
    pub fn fft_symbol(&mut self, sym: &[Complex<f32>]) -> Vec<Complex<f32>> {
        assert!(
            sym.len() >= self.t_g + self.t_u,
            "symbol buffer too short: {} < {}",
            sym.len(),
            self.t_g + self.t_u
        );
        let mut buf: Vec<Complex<f32>> = sym[self.t_g..self.t_g + self.t_u].to_vec();
        self.fft.forward(&mut buf);
        buf
    }

    /// Extract the 1536 active carriers from a 2048-bin natural-order spectrum.
    ///
    /// Returns a length-1536 vector ordered negative-carriers-first per the
    /// module convention (index 0 = carrier −768 … index 1535 = carrier +768).
    ///
    /// # Panics
    /// Panics if `spectrum.len() != T_u`.
    pub fn active_carriers(&self, spectrum: &[Complex<f32>]) -> Vec<Complex<f32>> {
        assert_eq!(spectrum.len(), self.t_u, "spectrum length must be T_u");
        let half = self.carriers / 2; // 768
        let mut out = Vec::with_capacity(self.carriers);
        // Negative carriers −768..=−1  ->  bins 1280..=2047.
        for k in (1..=half).rev() {
            // carrier −k -> bin T_u − k
            out.push(spectrum[self.t_u - k]);
        }
        // Positive carriers +1..=+768  ->  bins 1..=768.
        for k in 1..=half {
            out.push(spectrum[k]);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::DabParams;

    /// Build a time-domain symbol (with CP) from a 2048-bin spectrum via IFFT.
    fn symbol_from_spectrum(spectrum: &[Complex<f32>]) -> Vec<Complex<f32>> {
        let p = DabParams::mode_i();
        let t_u = p.t_u as usize;
        let t_g = p.t_g as usize;
        let fft = Fft::new(t_u);
        let mut useful = spectrum.to_vec();
        fft.inverse(&mut useful);
        // Prepend the cyclic prefix: the last T_g samples of the useful part.
        let mut sym = Vec::with_capacity(t_g + t_u);
        sym.extend_from_slice(&useful[t_u - t_g..]);
        sym.extend_from_slice(&useful);
        sym
    }

    #[test]
    fn single_carrier_peak_and_mapping() {
        let p = DabParams::mode_i();
        let t_u = p.t_u as usize;

        // Spectrum with a single active carrier +100 -> bin 100.
        let carrier = 100_usize;
        let mut spectrum = vec![Complex::new(0.0_f32, 0.0); t_u];
        spectrum[carrier] = Complex::new(1.0, 0.0);

        let sym = symbol_from_spectrum(&spectrum);

        let mut sf = SymbolFft::mode_i();
        let out = sf.fft_symbol(&sym);

        // Peak at bin 100.
        let peak_bin = out
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.norm().partial_cmp(&b.norm()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak_bin, carrier);

        let peak = out[carrier].norm();
        let max_other = out
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != carrier)
            .map(|(_, z)| z.norm())
            .fold(0.0_f32, f32::max);
        // < −40 dB elsewhere.
        assert!(
            max_other < peak * 0.01,
            "leakage too high: {max_other} vs peak {peak}"
        );

        // active_carriers puts energy at index for carrier +100: 768 + 100 − 1 = 867.
        let ac = sf.active_carriers(&out);
        let expect_idx = 768 + carrier - 1;
        let ac_peak = ac
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.norm().partial_cmp(&b.norm()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(ac_peak, expect_idx);
    }

    #[test]
    fn active_carrier_mapping() {
        let p = DabParams::mode_i();
        let t_u = p.t_u as usize;
        let mut spectrum = vec![Complex::new(0.0_f32, 0.0); t_u];

        // Known values at the four boundary bins.
        spectrum[1] = Complex::new(1.0, 0.0); // carrier +1   -> index 768
        spectrum[768] = Complex::new(2.0, 0.0); // carrier +768 -> index 1535
        spectrum[1280] = Complex::new(3.0, 0.0); // carrier −768 -> index 0
        spectrum[2047] = Complex::new(4.0, 0.0); // carrier −1   -> index 767

        let sf = SymbolFft::mode_i();
        let ac = sf.active_carriers(&spectrum);

        assert_eq!(ac.len(), 1536);
        assert_eq!(ac[768], Complex::new(1.0, 0.0)); // +1
        assert_eq!(ac[1535], Complex::new(2.0, 0.0)); // +768
        assert_eq!(ac[0], Complex::new(3.0, 0.0)); // −768
        assert_eq!(ac[767], Complex::new(4.0, 0.0)); // −1
    }
}
