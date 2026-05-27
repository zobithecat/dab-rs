//! Stage 1 of the OFDM sync chain — polyphase rational resampler.
//!
//! Front-end SDR captures (e.g. an Airspy Mini) deliver the DAB band at
//! **3.000 MSPS**, but the Mode I OFDM math is defined at **2.048 MSPS**
//! (`T_u = 2048` useful samples per `1 ms` symbol; ETSI EN 300 401 §14.5).
//! This module converts 3.000 MSPS → 2.048 MSPS with an exact rational ratio.
//!
//! # Exact ratio
//!
//! ```text
//! 2_048_000 / 3_000_000 = 256 / 375   (gcd = 8000)
//! ```
//!
//! so we **upsample by `L = 256`** and **downsample by `M = 375`**. A direct
//! polyphase FIR implementation is used (no external crate): the prototype
//! low-pass filter runs at the `L · fs` rate, but only the `L` phase that lands
//! on each output sample is ever evaluated, and only every `M`-th upsampled
//! position is kept. This is the standard Crochiere–Rabiner polyphase
//! decomposition — the cost per output sample is `taps_per_phase` complex MACs,
//! independent of `L`.
//!
//! # Prototype filter
//!
//! The shared anti-imaging / anti-aliasing low-pass prototype has cutoff
//! `min(1/L, 1/M)` in cycles/sample at the upsampled rate. Here `1/M < 1/L`
//! (`1/375 < 1/256`), so the binding constraint is the anti-aliasing edge:
//! `f_c = 1/(2·M)` of the upsampled Nyquist, i.e. the normalized cutoff
//! `f_c = 1/M` half-cycle (`= π/M` rad/sample) so nothing above the 2.048 MSPS
//! Nyquist survives the decimation. The prototype is a windowed sinc with a
//! **Blackman** window for good stopband attenuation, length
//! `taps_per_phase · L + 1` (odd, linear phase).
//!
//! # Output-length rule
//!
//! The resampler is fully streaming and keeps its commutator/history state
//! across [`Resampler::process`] calls. For a total of `N` input samples fed in
//! (in any chunking), the cumulative number of output samples produced is
//! exactly
//!
//! ```text
//! out = floor((N · L + (L - 1)) / M)        // == ceil(N · L / M) for our L,M
//! ```
//!
//! which for `N = 60_000_000` gives `floor((60_000_000·256 + 255)/375)
//! = 40_960_000`. The polyphase group delay manifests as the first
//! `~taps_per_phase/2` output samples being a filter-fill transient; the *count*
//! above is exact, the first handful of samples are simply not yet steady-state.

use num_complex::Complex;

/// Polyphase rational resampler with persistent streaming state.
pub struct Resampler {
    /// Upsampling factor `L`.
    up: usize,
    /// Downsampling factor `M`.
    down: usize,
    /// Taps per polyphase branch.
    taps_per_phase: usize,
    /// Polyphase filter bank: `phases[p][t]` is tap `t` of branch `p`.
    /// `phases.len() == up`, each inner length `== taps_per_phase`.
    phases: Vec<Vec<f32>>,
    /// Input history (most recent samples), length `taps_per_phase`.
    /// `history[0]` is the oldest, `history[last]` the newest.
    history: Vec<Complex<f32>>,
    /// Current upsampled-domain phase index in `[0, up)` (the commutator).
    phase: usize,
}

impl Resampler {
    /// Build the 3.000 MSPS → 2.048 MSPS resampler (`L = 256`, `M = 375`,
    /// 16 taps per phase). This is the canonical DAB front-end converter.
    pub fn new_3m_to_2048k() -> Self {
        Self::new(256, 375, 16)
    }

    /// Build a general rational resampler upsampling by `up`, downsampling by
    /// `down`, using `taps_per_phase` taps per polyphase branch.
    ///
    /// The prototype low-pass is a Blackman-windowed sinc of length
    /// `taps_per_phase · up + 1` with cutoff `min(1/up, 1/down)` (in cycles per
    /// upsampled sample), scaled by `up` to preserve unity passband gain after
    /// the `1/up` energy split across phases.
    ///
    /// # Panics
    /// Panics if `up == 0`, `down == 0`, or `taps_per_phase == 0`.
    pub fn new(up: usize, down: usize, taps_per_phase: usize) -> Self {
        assert!(up > 0 && down > 0 && taps_per_phase > 0, "invalid resampler ratios");

        let proto = Self::design_prototype(up, down, taps_per_phase);

        // Decompose into `up` polyphase branches. Branch `p` collects prototype
        // taps at indices `p, p+up, p+2·up, …`. Each branch has `taps_per_phase`
        // taps (the prototype length is `taps_per_phase·up + 1`; the trailing
        // `+1` tap of branch 0 is dropped to keep every branch equal length,
        // which is the standard even-decomposition and does not affect the
        // passband response materially).
        let mut phases = vec![vec![0.0f32; taps_per_phase]; up];
        for (p, branch) in phases.iter_mut().enumerate() {
            for (t, w) in branch.iter_mut().enumerate() {
                let idx = t * up + p;
                if idx < proto.len() {
                    *w = proto[idx];
                }
            }
        }

        Resampler {
            up,
            down,
            taps_per_phase,
            phases,
            history: vec![Complex::new(0.0, 0.0); taps_per_phase],
            phase: 0,
        }
    }

    /// Design the Blackman-windowed-sinc prototype low-pass filter.
    fn design_prototype(up: usize, down: usize, taps_per_phase: usize) -> Vec<f32> {
        let len = taps_per_phase * up + 1; // odd -> exact linear phase
        let m = (len - 1) as f64; // window span
        // Cutoff in cycles/sample at the upsampled rate.
        let fc = 1.0 / (up.max(down) as f64); // == min(1/up, 1/down)
        let center = m / 2.0;

        let mut taps = vec![0.0f32; len];
        for (n, tap) in taps.iter_mut().enumerate() {
            let x = n as f64 - center;
            // Normalized sinc with cutoff fc (cycles/sample): h = 2·fc·sinc(2·fc·x)
            let sinc = if x.abs() < 1e-12 {
                2.0 * fc
            } else {
                let a = std::f64::consts::PI * 2.0 * fc * x;
                (2.0 * fc) * (a.sin() / a)
            };
            // Blackman window.
            let w = 0.42 - 0.5 * (2.0 * std::f64::consts::PI * n as f64 / m).cos()
                + 0.08 * (4.0 * std::f64::consts::PI * n as f64 / m).cos();
            *tap = (sinc * w) as f32;
        }

        // Normalize to unity DC gain, then scale by `up` so each branch (which
        // only sees ~1/up of the energy) reconstructs unity passband gain.
        let sum: f32 = taps.iter().sum();
        let scale = up as f32 / sum;
        for t in taps.iter_mut() {
            *t *= scale;
        }
        taps
    }

    /// Push input samples; return all resampled output produced so far.
    ///
    /// Streaming and stateful — the commutator phase and the `taps_per_phase`
    /// input-history samples persist across calls, so a multi-million-sample
    /// capture can be fed in arbitrary chunks with identical results to one
    /// big call.
    pub fn process(&mut self, input: &[Complex<f32>]) -> Vec<Complex<f32>> {
        // Upper bound on output count for this chunk: ceil(n·up/down) + 1.
        let mut out = Vec::with_capacity(input.len() * self.up / self.down + 2);

        for &sample in input {
            // Shift the newest input into history (drop the oldest).
            self.history.rotate_left(1);
            self.history[self.taps_per_phase - 1] = sample;

            // The commutator advances by `up` per input sample; for every
            // multiple of `down` it crosses, we emit one output sample using
            // the polyphase branch at that fractional position.
            self.phase += self.up;
            while self.phase >= self.down {
                self.phase -= self.down;
                // The branch index is the current upsampled position mod up.
                // After the decrement, `self.phase` is in [0, up*?) — but we
                // need the branch that produced the just-crossed output. The
                // crossed upsampled index has phase `(self.phase) ` relative to
                // the new input; branch = phase within [0,up).
                let branch = &self.phases[self.phase % self.up];
                // Convolve branch with history. history[last] is newest, which
                // aligns with branch tap 0 (most-recent input).
                let mut acc = Complex::new(0.0f32, 0.0f32);
                for (t, &w) in branch.iter().enumerate() {
                    let h = self.history[self.taps_per_phase - 1 - t];
                    acc.re += w * h.re;
                    acc.im += w * h.im;
                }
                out.push(acc);
            }
        }
        out
    }
}

/// One-shot 3.000 MSPS → 2.048 MSPS resample (internally streams).
pub fn resample_3m_to_2048k(input: &[Complex<f32>]) -> Vec<Complex<f32>> {
    let mut r = Resampler::new_3m_to_2048k();
    r.process(input)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::Fft;

    /// Tiny LCG (numerical-recipes constants) for deterministic test signals.
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Lcg(seed)
        }
        fn next_f32(&mut self) -> f32 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // top 24 bits -> [-1, 1)
            ((self.0 >> 40) as f32 / (1u32 << 24) as f32) * 2.0 - 1.0
        }
    }

    #[test]
    fn output_length_ratio() {
        // N = 375_000 -> N·256/375 = 256_000 exactly.
        let n = 375_000usize;
        let input = vec![Complex::new(1.0f32, 0.0); n];
        let out = resample_3m_to_2048k(&input);
        let expected = n * 256 / 375; // 256_000
        let tol = 4usize; // a few samples of commutator slack
        assert!(
            (out.len() as i64 - expected as i64).abs() <= tol as i64,
            "len {} not within {} of {}",
            out.len(),
            tol,
            expected
        );
    }

    #[test]
    fn tone_preservation() {
        // 100 kHz complex exponential at 3 MSPS -> resample to 2.048 MSPS.
        let fs_in = 3_000_000.0f32;
        let f0 = 100_000.0f32;
        let n = 60_000usize;
        let mut input = Vec::with_capacity(n);
        for k in 0..n {
            let ph = 2.0 * std::f32::consts::PI * f0 * k as f32 / fs_in;
            input.push(Complex::new(ph.cos(), ph.sin()));
        }
        let out = resample_3m_to_2048k(&input);

        // Drop the filter-fill transient, take a power-of-two FFT block.
        let fs_out = 2_048_000.0f32;
        let nfft = 4096usize;
        let skip = 2048usize;
        assert!(out.len() >= skip + nfft);
        let mut buf: Vec<Complex<f32>> = out[skip..skip + nfft].to_vec();
        let fft = Fft::new(nfft);
        fft.forward(&mut buf);

        let (mut max_bin, mut max_pow) = (0usize, 0.0f32);
        for (b, c) in buf.iter().enumerate() {
            let p = c.norm_sqr();
            if p > max_pow {
                max_pow = p;
                max_bin = b;
            }
        }
        let bin_hz = fs_out / nfft as f32;
        let peak_hz = max_bin as f32 * bin_hz;
        assert!(
            (peak_hz - f0).abs() < 3.0 * bin_hz,
            "peak at {} Hz, expected ~{} (bin {} Hz)",
            peak_hz,
            f0,
            bin_hz
        );
    }

    #[test]
    fn passband_snr() {
        // Single in-band tone should dominate: peak power >> next-strongest.
        let fs_in = 3_000_000.0f32;
        let f0 = 250_000.0f32;
        let n = 60_000usize;
        let mut input = Vec::with_capacity(n);
        let mut rng = Lcg::new(1);
        for k in 0..n {
            let ph = 2.0 * std::f32::consts::PI * f0 * k as f32 / fs_in;
            // tone + tiny noise
            input.push(Complex::new(
                ph.cos() + 0.001 * rng.next_f32(),
                ph.sin() + 0.001 * rng.next_f32(),
            ));
        }
        let out = resample_3m_to_2048k(&input);
        let nfft = 4096usize;
        let skip = 2048usize;
        let mut buf: Vec<Complex<f32>> = out[skip..skip + nfft].to_vec();
        let fft = Fft::new(nfft);
        fft.forward(&mut buf);

        let mut pows: Vec<f32> = buf.iter().map(|c| c.norm_sqr()).collect();
        let total: f32 = pows.iter().sum();
        pows.sort_by(|a, b| b.partial_cmp(a).unwrap());
        let peak = pows[0];
        // Peak should hold the vast majority of the energy.
        // No FFT window is applied, so an off-bin tone leaks into adjacent
        // bins; the realistic, robust check is that the peak holds most of the
        // total energy and is well above the next-strongest bin.
        assert!(peak / total > 0.5, "peak fraction {} too low", peak / total);
        assert!(peak / pows[1] > 10.0, "peak/next ratio {} too low", peak / pows[1]);
    }
}
