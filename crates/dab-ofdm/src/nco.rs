//! Stage 4a of the OFDM sync chain — numerically-controlled oscillator (NCO).
//!
//! Once the fractional carrier-frequency offset (CFO) is estimated (Stage 3,
//! [`crate::cp_sync`]), the residual offset is removed by multiplying every
//! sample by a complex exponential that rotates in the opposite direction:
//!
//! ```text
//! y[n] = x[n] · exp(+j·2π·f_hz·n / fs)
//! ```
//!
//! The sign is chosen so that passing the **negated** offset removes it: a
//! tone `exp(+j·2π·f0·n/fs)` mixed with `f_hz = −f0` rotates downward to DC.
//! This mirrors the oracle's `getSample`/`getSamples` NCO in
//! `eti-stuff/src/ofdm/ofdm-processor.cpp`, which mixes the incoming stream
//! against a precomputed `oscillatorTable` indexed by a wrapping phase
//! accumulator. We accumulate phase in `f64` to keep long streams precise.
//!
//! Use a **negative** `f_hz` to *remove* a measured `+f_hz` offset.

use num_complex::Complex;

use std::f64::consts::PI;

/// Streaming numerically-controlled oscillator / frequency mixer.
///
/// Keeps a phase accumulator across [`mix`](Nco::mix) calls so a long stream can
/// be processed in arbitrary-sized chunks without phase discontinuities at the
/// chunk boundaries.
pub struct Nco {
    /// Running phase in radians (advances by `2π·f_hz·n/fs` across samples).
    phase: f64,
    /// Sample rate in Hz (`fs`).
    fs: f64,
}

impl Nco {
    /// Create an NCO for sample rate `fs` (DAB Mode I: `2_048_000.0`).
    pub fn new(fs: f64) -> Self {
        Nco { phase: 0.0, fs }
    }

    /// Mix `iq` in place by `f_hz`, advancing the phase accumulator.
    ///
    /// Each sample `n` (continuing the running index) is multiplied by
    /// `exp(−j·2π·f_hz·n/fs)`. Pass a negative `f_hz` to cancel a positive
    /// frequency offset. The accumulated phase persists for the next call.
    pub fn mix(&mut self, iq: &mut [Complex<f32>], f_hz: f64) {
        // Phase increment per sample (radians). With this sign, passing the
        // *negated* offset removes it: a +f0 tone exp(+j2πf0n/fs) mixed with
        // f_hz = −f0 rotates by exp(−j2πf0n/fs), cancelling it to DC.
        let dphi = 2.0 * PI * f_hz / self.fs;
        for z in iq.iter_mut() {
            let (s, c) = self.phase.sin_cos();
            let rot = Complex::new(c as f32, s as f32);
            *z *= rot;
            self.phase += dphi;
            // Keep the accumulator bounded to preserve precision over long
            // streams. Wrap to [−π, π).
            if self.phase >= PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }
    }

    /// Reset the phase accumulator to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

/// One-shot frequency mix: rotate `iq` in place by `f_hz` at sample rate `fs`,
/// with the phase starting at zero.
///
/// Equivalent to `Nco::new(fs).mix(iq, f_hz)`. Convenience for correcting a
/// single isolated buffer.
pub fn mix_frequency(iq: &mut [Complex<f32>], f_hz: f64, fs: f64) {
    Nco::new(fs).mix(iq, f_hz);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fft::Fft;

    const FS: f64 = 2_048_000.0;

    /// Build a pure complex tone exp(+j·2π·f·n/fs) of length `n`.
    fn tone(f_hz: f64, n: usize) -> Vec<Complex<f32>> {
        (0..n)
            .map(|k| {
                let ph = 2.0 * PI * f_hz * k as f64 / FS;
                Complex::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect()
    }

    #[test]
    fn round_trip_mix_recovers_original() {
        let f0 = 1234.0;
        let n = 4096;
        let original = tone(f0, n);

        let mut buf = original.clone();
        // Mix up by +f0 then back down by −f0 (across two NCO calls to also
        // exercise phase continuity).
        let mut nco = Nco::new(FS);
        let (a, b) = buf.split_at_mut(n / 2);
        nco.mix(a, f0);
        nco.mix(b, f0);
        let mut nco2 = Nco::new(FS);
        nco2.mix(&mut buf, -f0);

        for (o, r) in original.iter().zip(buf.iter()) {
            assert!((o.re - r.re).abs() < 1e-3, "re mismatch: {o:?} vs {r:?}");
            assert!((o.im - r.im).abs() < 1e-3, "im mismatch: {o:?} vs {r:?}");
        }
    }

    #[test]
    fn mixing_tone_to_dc() {
        // A +f0 tone exp(+j·2π·f0·n/fs) is removed (brought to DC, bin 0) by
        // mixing with exp(−j·2π·f0·n/fs), i.e. mix(.., +f0). This matches the
        // contract "use negative f_hz to remove a +f_hz offset": removing a
        // positive offset means a downward rotation by exactly that frequency.
        let n = 2048;
        // Pick a frequency that is an exact FFT bin: f0 = bin * fs/N.
        let bin = 37.0;
        let f0 = bin * FS / n as f64;
        let mut buf = tone(f0, n);

        // "Mixing a tone at f0 by −f0 yields DC" (spec test 1): removing the
        // +f0 offset by passing the negated frequency.
        mix_frequency(&mut buf, -f0, FS);

        let fft = Fft::new(n);
        fft.forward(&mut buf);

        // Energy should concentrate at bin 0.
        let dc = buf[0].norm();
        let max_other = buf
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != 0)
            .map(|(_, z)| z.norm())
            .fold(0.0_f32, f32::max);
        assert!(
            dc > 100.0 * max_other,
            "DC ({dc}) not dominant vs max_other ({max_other})"
        );
    }
}
