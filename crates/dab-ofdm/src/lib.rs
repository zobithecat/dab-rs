//! `dab-ofdm` — DAB Mode I OFDM demodulator.
//!
//! Faithful Rust port of the eti-stuff C++ oracle, validated stage-by-stage
//! against ETSI EN 300 401. The crate is split into a static foundation
//! (input-independent reference data) and a 7-stage runtime sync/demod chain.
//!
//! # Static foundation
//!
//! - [`params`]         — DAB Mode I transmission parameters (§14.5).
//! - [`phasetable`]     — Mode I reference-phase function `get_phi(k)` (§14.3.2).
//! - [`phasereference`] — the frequency-domain Phase-Reference Symbol (§14.3.2).
//! - [`freq_interleaver`] — the Mode I frequency (de-)interleaving table (§14.6).
//! - [`fft`]            — a thin `rustfft` wrapper (forward + inverse).
//!
//! # 7-stage sync/demod chain
//!
//! Each stage consumes the output of the previous one and exposes types listed
//! in [`prelude`-style re-exports](#reexports) below:
//!
//! 1. Resample 3 → 2.048 MSPS (polyphase, L/M = 256/375) — [`Resampler`].
//! 2. Coarse time sync (null-symbol envelope dip, adaptive threshold) — [`NullDetector`].
//! 3. Fine time + fractional frequency offset (cyclic-prefix autocorrelation) — [`CpSync`].
//! 4. NCO frequency correction + integer carrier offset via PRS — [`Nco`], [`detect_integer_cfo`].
//! 5. Per-symbol FFT framing (strip guard, transform useful part) — [`SymbolFft`].
//! 6. Differential per-carrier reference (`current * conj(prev)`) — [`DifferentialReference`].
//! 7. π/4-DQPSK demap + frequency de-interleave → soft bits — [`DqpskDemap`].
//!
//! Stage 6 is *not* a true `Ĥ[k]`-estimating equalizer: DAB's π/4-DQPSK is
//! intrinsically differential, so the previous symbol's spectrum is the
//! per-carrier reference and the channel cancels in the conjugate product
//! (matches `eti-stuff/src/ofdm/ofdm-processor.cpp::processBlock`). Stage 7's
//! soft-bit polarity follows the `+ ⇒ bit 1` convention — see the project
//! README *Discovered subtleties*.

#![forbid(unsafe_code)]

pub mod channel_eq;
pub mod cp_sync;
pub mod dqpsk_demap;
pub mod fft;
pub mod freq_interleaver;
pub mod integer_cfo;
pub mod nco;
pub mod null_detect;
pub mod params;
pub mod phasereference;
pub mod phasetable;
pub mod resampler;
pub mod symbol_fft;

pub use channel_eq::DifferentialReference;
pub use cp_sync::{CpMetric, CpSync, LockReport};
pub use dqpsk_demap::{jan_abs, DqpskDemap};
pub use fft::Fft;
pub use freq_interleaver::FreqInterleaver;
pub use integer_cfo::{detect_integer_cfo, estimate_offset_eti, IntegerCfoResult};
pub use nco::{mix_frequency, Nco};
pub use null_detect::{NullDetectResult, NullDetector};
pub use params::DabParams;
pub use phasereference::phase_reference;
pub use resampler::{resample_3m_to_2048k, LinearResampler, Resampler};
pub use symbol_fft::SymbolFft;
