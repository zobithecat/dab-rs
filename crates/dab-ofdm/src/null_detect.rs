//! Stage 2 of the OFDM sync chain — null-symbol envelope detector.
//!
//! Coarse time synchronization for DAB Mode I. Every transmission frame
//! (`T_F = 196_608` samples ≈ 96 ms at 2.048 MSPS; ETSI EN 300 401 §14.5)
//! begins with a **null symbol** (`T_NULL = 2656` samples) during which the
//! transmitter is silent. Detecting the periodic energy dips in the signal
//! envelope yields the frame boundaries.
//!
//! # Why an *adaptive* threshold (the SFN gotcha)
//!
//! DAB is broadcast as a **Single Frequency Network (SFN)**: many synchronized
//! transmitters share one frequency. Their signals arrive with different delays
//! (multipath), and during one transmitter's null symbol the *other*
//! transmitters can still be radiating useful symbols, partially filling the
//! null. The dip can therefore be far shallower than the textbook
//! "envelope → 0" — empirically only ~25 % below the active level
//! (`min/mu ≈ 0.75`). A **fixed** `μ`-multiple threshold (e.g. `0.5·μ`) misses
//! these shallow dips. Instead we set the threshold as a fraction of the
//! observed envelope *range* (1st…99th percentile), which adapts to whatever
//! depth the SFN multipath leaves.
//!
//! This is a faithful port of `gate_null` in
//! `airspy-mini-dmb/tools/iq_validate_dab.py`. The reference decimates the
//! input when `fs > 2 MSPS` (`dec = fs // 2_000_000`); for the post-resampler
//! 2.048 MSPS DAB stream `dec == 1`, so no decimation occurs here.

use num_complex::Complex;

/// Result of null-symbol detection.
pub struct NullDetectResult {
    /// Start index (in samples) of each qualifying null-dip run.
    pub positions: Vec<usize>,
    /// Median inter-dip spacing in samples, restricted to the 88…104 ms
    /// window, when at least 3 such gaps were observed; else `None`.
    pub frame_period: Option<usize>,
}

/// Adaptive null-symbol detector operating on a magnitude envelope.
pub struct NullDetector {
    sample_rate: u32,
}

impl NullDetector {
    /// Build a detector for a stream at `sample_rate` Hz (2_048_000 for the
    /// resampled DAB Mode I stream).
    pub fn new(sample_rate: u32) -> Self {
        NullDetector { sample_rate }
    }

    /// Detect periodic null dips in the IQ stream.
    ///
    /// Mirrors `gate_null` step-for-step: magnitude → moving-average smoothing
    /// → adaptive percentile threshold → masked-run collection with a minimum
    /// length → median gap within the 96 ms window.
    pub fn detect(&self, iq: &[Complex<f32>]) -> NullDetectResult {
        let fs = self.sample_rate as f64;

        // 1) Magnitude envelope.
        let env: Vec<f32> = iq.iter().map(|c| c.norm()).collect();
        let n = env.len();
        if n == 0 {
            return NullDetectResult { positions: Vec::new(), frame_period: None };
        }

        // 2) Moving-average smoothing, window W = round(0.5 ms · fs), 'same'
        //    centering (NumPy `np.convolve(..., mode="same")` semantics).
        let w = ((0.5e-3 * fs).round() as usize).max(64);
        let sm = moving_average_same(&env, w);

        // 3) Adaptive threshold from the 1st / 99th percentiles.
        let p1 = percentile(&sm, 1.0);
        let p99 = percentile(&sm, 99.0);
        let thresh = p1 + 0.30 * (p99 - p1);

        // 4) Collect masked runs (sm < thresh) longer than 0.3 ms.
        let min_run = (0.3e-3 * fs).round() as usize;
        let mut positions = Vec::new();
        let mut in_dip = false;
        let mut rstart = 0usize;
        for (i, &s) in sm.iter().enumerate() {
            let m = s < thresh;
            if m && !in_dip {
                rstart = i;
                in_dip = true;
            } else if !m && in_dip {
                if i - rstart > min_run {
                    positions.push(rstart);
                }
                in_dip = false;
            }
        }
        // A dip still open at end-of-buffer is left unclosed, matching the
        // Python loop which only records a run on its falling edge.

        // 6) Frame-period estimate: gaps within the 88…104 ms window.
        let lo = (0.088 * fs).round() as i64;
        let hi = (0.104 * fs).round() as i64;
        let mut near96: Vec<usize> = Vec::new();
        for w2 in positions.windows(2) {
            let gap = w2[1] as i64 - w2[0] as i64;
            if gap > lo && gap < hi {
                near96.push(gap as usize);
            }
        }
        let frame_period = if near96.len() >= 3 {
            near96.sort_unstable();
            Some(median_sorted(&near96))
        } else {
            None
        };

        NullDetectResult { positions, frame_period }
    }
}

/// Moving average with NumPy `mode="same"` centering: output length equals
/// input length, output[i] is the mean of the window of `w` taps centered on
/// `i` (the convolution of the signal with `ones(w)/w`, cropped to the center).
fn moving_average_same(x: &[f32], w: usize) -> Vec<f32> {
    let n = x.len();
    if w <= 1 {
        return x.to_vec();
    }
    // Prefix sums for O(n) windowed means.
    let mut prefix = vec![0.0f64; n + 1];
    for i in 0..n {
        prefix[i + 1] = prefix[i] + x[i] as f64;
    }
    let inv_w = 1.0 / w as f64;
    // 'same': full convolution length is n + w - 1, starting at offset
    // (w - 1)/2 (floor) for the kept center region. For output index i the
    // full-convolution index is i + (w - 1)/2; that sum covers input indices
    // [i + (w-1)/2 - (w-1) .. i + (w-1)/2], clipped to [0, n).
    let half = (w - 1) / 2;
    let mut out = vec![0.0f32; n];
    for i in 0..n {
        let center = i as i64 + half as i64; // index into full convolution
        let start = center - (w as i64 - 1);
        let lo = start.max(0) as usize;
        let hi = (center + 1).min(n as i64) as usize; // exclusive
        let sum = prefix[hi] - prefix[lo];
        // Divide by the full window `w` (not the clipped count) — matches
        // NumPy zero-padded convolution at the edges.
        out[i] = (sum * inv_w) as f32;
    }
    out
}

/// Linear-interpolated percentile (NumPy default) of an unsorted slice.
fn percentile(x: &[f32], q: f32) -> f32 {
    let mut v = x.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let n = v.len();
    if n == 1 {
        return v[0];
    }
    let rank = (q / 100.0) * (n as f32 - 1.0);
    let lo = rank.floor() as usize;
    let hi = rank.ceil() as usize;
    let frac = rank - lo as f32;
    v[lo] + (v[hi] - v[lo]) * frac
}

fn median_sorted(v: &[usize]) -> usize {
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic xorshift RNG for synthetic noise.
    struct Xorshift(u64);
    impl Xorshift {
        fn new(seed: u64) -> Self {
            Xorshift(seed.max(1))
        }
        /// Uniform in [-1, 1).
        fn next_f32(&mut self) -> f32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            ((x >> 40) as f32 / (1u32 << 24) as f32) * 2.0 - 1.0
        }
    }

    /// Build `frames` frames of [null @ `null_amp`][active @ 1.0], each frame
    /// `T_F = 196608` samples with a `null_len = 2656`-sample null at the start.
    fn synth(frames: usize, null_amp: f32, seed: u64) -> Vec<Complex<f32>> {
        let t_f = 196_608usize;
        let null_len = 2656usize;
        let mut rng = Xorshift::new(seed);
        let mut out = Vec::with_capacity(frames * t_f);
        for _ in 0..frames {
            for i in 0..t_f {
                let amp = if i < null_len { null_amp } else { 1.0 };
                out.push(Complex::new(amp * rng.next_f32(), amp * rng.next_f32()));
            }
        }
        out
    }

    #[test]
    fn detects_deep_nulls() {
        let frames = 10;
        let iq = synth(frames, 0.05, 12345);
        let det = NullDetector::new(2_048_000);
        let res = det.detect(&iq);
        // ~one dip per frame (first null may be partially clipped by 'same'
        // edge smoothing, last may be unclosed -> allow a small slack).
        assert!(
            res.positions.len() >= frames - 2 && res.positions.len() <= frames + 1,
            "found {} dips, expected ~{}",
            res.positions.len(),
            frames
        );
        let fp = res.frame_period.expect("frame period should be estimated");
        assert!(
            (fp as i64 - 196_608).abs() <= 64,
            "frame period {} not ~196608",
            fp
        );
    }

    #[test]
    fn detects_shallow_sfn_nulls() {
        // SFN fill: null only ~25% below active (min/mu ~0.75). A fixed
        // 0.5·mu threshold would miss these; the adaptive threshold catches
        // them.
        let frames = 10;
        let iq = synth(frames, 0.75, 999);
        let det = NullDetector::new(2_048_000);
        let res = det.detect(&iq);
        assert!(
            res.positions.len() >= frames - 2,
            "shallow nulls: found only {} dips",
            res.positions.len()
        );
        let fp = res.frame_period.expect("shallow-null frame period should be estimated");
        assert!(
            (fp as i64 - 196_608).abs() <= 64,
            "shallow-null frame period {} not ~196608",
            fp
        );
    }

    #[test]
    fn empty_input_is_safe() {
        let det = NullDetector::new(2_048_000);
        let res = det.detect(&[]);
        assert!(res.positions.is_empty());
        assert!(res.frame_period.is_none());
    }
}
