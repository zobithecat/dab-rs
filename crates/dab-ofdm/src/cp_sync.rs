//! Stage 3 of the OFDM sync chain — cyclic-prefix (CP) autocorrelation.
//!
//! Fine **symbol-timing** recovery and **fractional carrier-frequency offset
//! (CFO)** estimation for DAB Mode I, derived from the redundancy of the
//! cyclic prefix (ETSI EN 300 401 §14.5). Each OFDM symbol prepends a copy of
//! the last `T_g = 504` samples of its useful part (`T_u = 2048`) as a guard
//! interval, giving the full symbol length `T_s = T_u + T_g = 2552` at
//! 2.048 MSPS. Because the guard is a verbatim copy of the tail, the samples at
//! the symbol start correlate strongly with those exactly `T_u` later:
//!
//! ```text
//! x[d + k] ≈ x[d + k + T_u]   for k in [0, T_g)
//! ```
//!
//! where `d` is the symbol-start offset (start of the CP).
//!
//! # Timing metric — `gate_cp_autocorr`
//!
//! The normalized CP correlation magnitude reproduces `gate_cp_autocorr` in
//! `airspy-mini-dmb/tools/iq_validate_dab.py`:
//!
//! ```text
//! r(d)   = Σ_{k=0}^{T_g-1} conj(x[d+k]) · x[d+k+T_u]
//! p1     = Σ_{k=0}^{T_g-1} |x[d+k]|²
//! p2     = Σ_{k=0}^{T_g-1} |x[d+k+T_u]|²
//! mag(d) = |r(d)| / sqrt(p1 · p2)            (in [0, 1])
//! ```
//!
//! At a true symbol start `mag → 1`; off the boundary it falls toward the
//! noise floor. ([`metric_at`](CpSync::metric_at), [`fine_time`](CpSync::fine_time),
//! [`lock`](CpSync::lock).)
//!
//! # Fractional CFO — eti-stuff `ofdm-processor.cpp`
//!
//! A residual carrier offset `Δf` rotates every sample by `exp(j·2π·Δf·n/fs)`,
//! so over the `T_u`-sample lag the CP correlation `r` accumulates a phase
//! `2π·Δf·T_u/fs`. With `carrier_diff = fs/T_u = 1000 Hz`:
//!
//! ```text
//! frac_cfo_hz = arg(r) / (2π) · carrier_diff
//! ```
//!
//! This is exactly the `eti-stuff` `FreqCorr`/`fineCorrector` update. There the
//! loop accumulates `FreqCorr += x[i]·conj(x[i-T_u])` over `i in [T_u, T_s)` of
//! each symbol — the conjugate-mirrored form of `r` above — and then advances
//! `fineCorrector += 0.1 · arg(FreqCorr)/π · (carrier_diff/2)`. Since
//! `arg(r)/(2π)·carrier_diff == arg(r)/π·(carrier_diff/2)`, the per-symbol
//! correction term matches term-for-term (the `0.1` is the loop gain of their
//! tracking integrator; [`estimate_cfo_hz`](CpSync::estimate_cfo_hz) reports the
//! raw single-shot estimate by summing the complex `r` across symbols and
//! taking the argument once).
//!
//! Only the **fractional** offset (`|Δf| < carrier_diff/2 = 500 Hz`) is
//! resolvable from the CP: a shift by a whole carrier spacing leaves the CP
//! phase unchanged. The integer-carrier part is recovered later from the
//! Phase-Reference Symbol (a separate, out-of-scope PRS stage).

use std::f32::consts::PI;

use num_complex::Complex;

/// CP autocorrelation result at one candidate symbol start.
#[derive(Debug, Clone, Copy)]
pub struct CpMetric {
    /// Normalized correlation magnitude `|r| / sqrt(p1·p2)`, in `[0, 1]`.
    pub mag: f32,
    /// Fractional carrier-frequency offset in Hz, `arg(r)/(2π)·carrier_diff`.
    pub frac_cfo_hz: f32,
}

/// Outcome of a [`CpSync::lock`] scan: the global timing peak and how many of
/// the following symbols re-locked at `+T_s` spacing.
#[derive(Debug, Clone)]
pub struct LockReport {
    /// Absolute offset of the best symbol start found in the scan span.
    pub peak_idx: usize,
    /// Normalized correlation magnitude at `peak_idx`.
    pub peak_mag: f32,
    /// Symbols (of `n_tested`) with `mag >= frac·peak_mag` at `+T_s` spacing.
    pub n_locked: usize,
    /// Number of follow-on symbols tested at `+T_s` spacing.
    pub n_tested: usize,
}

/// Cyclic-prefix synchronizer parameterized by the OFDM symbol geometry.
pub struct CpSync {
    tu: usize,
    tg: usize,
    ts: usize,
    carrier_diff: f32,
}

impl CpSync {
    /// DAB Mode I geometry at 2.048 MSPS: `Tu=2048`, `Tg=504`, `Ts=2552`,
    /// `carrier_diff = fs/Tu = 1000 Hz`.
    pub fn mode_i() -> Self {
        Self::new(2048, 504, 2552, 1000.0)
    }

    /// Build a synchronizer for an arbitrary OFDM geometry.
    pub fn new(tu: usize, tg: usize, ts: usize, carrier_diff: f32) -> Self {
        CpSync { tu, tg, ts, carrier_diff }
    }

    /// CP metric at absolute symbol-start offset `d`.
    ///
    /// Returns `None` when the correlation window `[d, d + T_u + T_g)` would
    /// read past the end of `iq`. Mirrors `gate_cp_autocorr`'s inner loop body.
    pub fn metric_at(&self, iq: &[Complex<f32>], d: usize) -> Option<CpMetric> {
        // Need x[d .. d+Tg) and x[d+Tu .. d+Tu+Tg).
        let end = d.checked_add(self.tu)?.checked_add(self.tg)?;
        if end > iq.len() {
            return None;
        }

        let mut r = Complex::<f32>::new(0.0, 0.0);
        let mut p1 = 0.0f32;
        let mut p2 = 0.0f32;
        for k in 0..self.tg {
            let a = iq[d + k]; // CP sample
            let b = iq[d + self.tu + k]; // its useful-part twin, Tu later
            // r += conj(a) · b  (gate_cp_autocorr: np.vdot(x[k:k+Tg], x[k+Tu:...]))
            r += a.conj() * b;
            p1 += a.norm_sqr();
            p2 += b.norm_sqr();
        }

        let mag = if p1 > 1e-12 && p2 > 1e-12 {
            r.norm() / (p1 * p2).sqrt()
        } else {
            0.0
        };

        // arg(r) is the phase advance over Tu samples = 2π·Δf·Tu/fs, so
        // Δf = arg(r)/(2π) · (fs/Tu) = arg(r)/(2π) · carrier_diff.
        let frac_cfo_hz = r.arg() / (2.0 * PI) * self.carrier_diff;

        Some(CpMetric { mag, frac_cfo_hz })
    }

    /// Search offsets `[center - half, center + half]` for the maximum CP
    /// correlation magnitude and return the best absolute offset.
    ///
    /// Candidates whose window runs off the end are skipped. If none are valid
    /// the clamped `center` is returned unchanged.
    pub fn fine_time(&self, iq: &[Complex<f32>], center: usize, half: usize) -> usize {
        let lo = center.saturating_sub(half);
        let hi = center.saturating_add(half);
        let mut best_idx = center;
        let mut best_mag = -1.0f32;
        for d in lo..=hi {
            if let Some(m) = self.metric_at(iq, d) {
                if m.mag > best_mag {
                    best_mag = m.mag;
                    best_idx = d;
                }
            }
        }
        best_idx
    }

    /// Scan a `2·T_s` span from `scan_start` for the global timing peak, then
    /// test the next `n` symbols at `+T_s` spacing, counting those that re-lock
    /// with `mag >= frac·peak`. Faithful to `gate_cp_autocorr`'s peak-then-lock
    /// logic (which uses `frac = 0.5`, `n = 50`).
    pub fn lock(&self, iq: &[Complex<f32>], scan_start: usize, n: usize, frac: f32) -> LockReport {
        let span = self.ts * 2;
        let mut peak_idx = scan_start;
        let mut peak_mag = -1.0f32;
        for off in 0..span {
            let d = scan_start + off;
            if let Some(m) = self.metric_at(iq, d) {
                if m.mag > peak_mag {
                    peak_mag = m.mag;
                    peak_idx = d;
                }
            }
        }
        if peak_mag < 0.0 {
            peak_mag = 0.0;
        }

        let mut n_locked = 0usize;
        for j in 1..=n {
            let idx = peak_idx + j * self.ts;
            if let Some(m) = self.metric_at(iq, idx) {
                if m.mag >= frac * peak_mag {
                    n_locked += 1;
                }
            }
        }

        LockReport { peak_idx, peak_mag, n_locked, n_tested: n }
    }

    /// Average fractional CFO (Hz) over `n` symbols starting at symbol-start
    /// `s0`.
    ///
    /// Reproduces the eti-stuff `FreqCorr` accumulation: the complex CP
    /// correlation `r` is summed across symbols (coherent integration) and the
    /// argument of that single sum is taken once, then scaled to Hz exactly as
    /// `fineCorrector`'s `arg/π · (carrier_diff/2)` update. Returns `0.0` when
    /// no symbol window is valid.
    pub fn estimate_cfo_hz(&self, iq: &[Complex<f32>], s0: usize, n: usize) -> f32 {
        let mut acc = Complex::<f32>::new(0.0, 0.0);
        for j in 0..n {
            let d = s0 + j * self.ts;
            let end = match d.checked_add(self.tu).and_then(|v| v.checked_add(self.tg)) {
                Some(v) => v,
                None => break,
            };
            if end > iq.len() {
                break;
            }
            for k in 0..self.tg {
                let a = iq[d + k];
                let b = iq[d + self.tu + k];
                acc += a.conj() * b;
            }
        }
        if acc.norm_sqr() < 1e-20 {
            return 0.0;
        }
        acc.arg() / (2.0 * PI) * self.carrier_diff
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::Fft;

    /// Minimal in-test xorshift32 PRNG (no external rng crate).
    struct XorShift32(u32);
    impl XorShift32 {
        fn new(seed: u32) -> Self {
            XorShift32(seed | 1)
        }
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        /// Uniform in [-1, 1).
        fn next_f32(&mut self) -> f32 {
            (self.next_u32() as f32 / u32::MAX as f32) * 2.0 - 1.0
        }
        /// Crude zero-mean Gaussian-ish via averaging 4 uniforms.
        fn next_gauss(&mut self) -> f32 {
            (self.next_f32() + self.next_f32() + self.next_f32() + self.next_f32()) * 0.5
        }
        /// Random QPSK symbol on the unit circle (±1 ± j)/√2.
        fn next_qpsk(&mut self) -> Complex<f32> {
            let r = self.next_u32();
            let re = if r & 1 == 0 { 1.0 } else { -1.0 };
            let im = if r & 2 == 0 { 1.0 } else { -1.0 };
            Complex::new(re, im) * std::f32::consts::FRAC_1_SQRT_2
        }
    }

    const TU: usize = 2048;
    const TG: usize = 504;
    const TS: usize = 2552;
    const FS: f32 = 2_048_000.0;

    /// Build `n_syms` synthetic DAB-like OFDM symbols.
    ///
    /// Each symbol: random QPSK on 1536 active carriers (centered layout, DC
    /// nulled), inverse-FFT to `Tu = 2048` time samples, then prepend the last
    /// `Tg = 504` samples as the cyclic prefix → `Ts = 2552` samples. The
    /// returned stream is the concatenation; the first symbol starts at index 0.
    fn synth_stream(n_syms: usize, seed: u32) -> Vec<Complex<f32>> {
        let fft = Fft::new(TU);
        let mut rng = XorShift32::new(seed);
        let active = 1536usize;
        let half = active / 2; // 768 carriers each side of DC
        let mut out = Vec::with_capacity(n_syms * TS);

        for _ in 0..n_syms {
            // Frequency-domain symbol: zero everywhere except the active band.
            let mut spec = vec![Complex::<f32>::new(0.0, 0.0); TU];
            // Positive carriers: bins 1..=half. Negative carriers: bins TU-half..TU.
            for k in 1..=half {
                spec[k] = rng.next_qpsk();
            }
            for k in 1..=half {
                spec[TU - k] = rng.next_qpsk();
            }
            // DC (bin 0) left at zero.

            // Inverse FFT -> time-domain useful part.
            fft.inverse(&mut spec);
            let useful = spec; // length Tu

            // Prepend the last Tg samples as the cyclic prefix.
            out.extend_from_slice(&useful[TU - TG..TU]);
            out.extend_from_slice(&useful);
        }
        out
    }

    /// Apply a complex carrier rotation exp(j·2π·Δf·n/fs) in place.
    fn apply_cfo(iq: &mut [Complex<f32>], df_hz: f32) {
        for (n, z) in iq.iter_mut().enumerate() {
            let phase = 2.0 * PI * df_hz * n as f32 / FS;
            *z *= Complex::from_polar(1.0, phase);
        }
    }

    #[test]
    fn perfect_alignment_metric() {
        let iq = synth_stream(5, 0xC0FFEE);
        let sync = CpSync::mode_i();

        // True symbol start of symbol #2 is at index 2*Ts.
        let d = 2 * TS;
        let m = sync.metric_at(&iq, d).expect("in range");
        assert!(m.mag > 0.95, "aligned mag {} should be ~1", m.mag);

        // Half a symbol off the boundary: clearly lower.
        let off = sync.metric_at(&iq, d + TS / 2).expect("in range");
        assert!(
            off.mag < m.mag - 0.3,
            "off-boundary mag {} not clearly below aligned {}",
            off.mag,
            m.mag
        );
    }

    #[test]
    fn fine_time_recovers_start() {
        let iq = synth_stream(6, 0xBADF00D);
        let sync = CpSync::mode_i();
        let true_start = 3 * TS;
        // Center the search a bit off (within ±Tg) and recover the true start.
        let center = true_start + 77;
        let found = sync.fine_time(&iq, center, TG);
        assert!(
            (found as i64 - true_start as i64).abs() <= 1,
            "fine_time found {} expected {}",
            found,
            true_start
        );
    }

    #[test]
    fn fractional_cfo_recovery() {
        let mut iq = synth_stream(25, 0x5EED);
        let df = 137.0f32; // within ±500 Hz
        apply_cfo(&mut iq, df);
        let sync = CpSync::mode_i();
        // Symbol start 0 is a true boundary; estimate over ~20 symbols.
        let est = sync.estimate_cfo_hz(&iq, 0, 20);
        assert!(
            (est - df).abs() < 5.0,
            "CFO estimate {} not within a few Hz of {}",
            est,
            df
        );
    }

    #[test]
    fn lock_full_on_clean_stream() {
        let iq = synth_stream(60, 0x1357);
        let sync = CpSync::mode_i();
        let report = sync.lock(&iq, 0, 50, 0.5);
        assert!(report.peak_mag > 0.95, "peak_mag {}", report.peak_mag);
        assert_eq!(
            report.n_locked, report.n_tested,
            "expected 100% lock, got {}/{}",
            report.n_locked, report.n_tested
        );
    }

    #[test]
    fn lock_and_cfo_robust_to_noise() {
        let mut iq = synth_stream(60, 0x2468);
        let df = -211.0f32; // within ±500 Hz
        apply_cfo(&mut iq, df);

        // Add light complex Gaussian-ish noise. Signal RMS per component ~0.5
        // after the centered-IFFT scaling; keep noise well below that.
        let mut rng = XorShift32::new(0x9999);
        let noise_sigma = 0.05f32;
        for z in iq.iter_mut() {
            *z += Complex::new(rng.next_gauss() * noise_sigma, rng.next_gauss() * noise_sigma);
        }

        let sync = CpSync::mode_i();
        let report = sync.lock(&iq, 0, 50, 0.5);
        let pct = 100.0 * report.n_locked as f32 / report.n_tested as f32;
        assert!(pct >= 80.0, "noisy lock only {}% ({}/{})", pct, report.n_locked, report.n_tested);

        let est = sync.estimate_cfo_hz(&iq, report.peak_idx, 20);
        assert!(
            (est - df).abs() < 15.0,
            "noisy CFO estimate {} not within tolerance of {}",
            est,
            df
        );
    }
}
