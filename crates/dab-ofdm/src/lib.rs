//! `dab-ofdm` — DAB Mode I OFDM demodulator.
//!
//! This crate is built in two slices. The current slice is the **static
//! foundation**: the input-independent, deterministic reference data plus an
//! FFT wrapper that every later stage depends on. It is a faithful port of the
//! `eti-stuff` C++ oracle and of ETSI EN 300 401:
//!
//! - [`params`]        — DAB Mode I transmission parameters (§14.5).
//! - [`phasetable`]    — Mode I reference-phase function `get_phi(k)` (§14.3.2).
//! - [`phasereference`]— the frequency-domain Phase-Reference Symbol (§14.3.2).
//! - [`freq_interleaver`] — the Mode I frequency (de-)interleaving table (§14.6).
//! - [`fft`]           — a thin `rustfft` wrapper (forward + inverse).
//!
//! # Deferred: the 7-stage sync chain (Week 3-5)
//!
//! The runtime, input-dependent demodulation pipeline is NOT implemented yet.
//! It will build on this static foundation in a later slice:
//!
//! 1. Null-symbol detection (frame start / energy dip).
//! 2. Coarse time synchronization (PRS correlation, `findIndex`).
//! 3. Coarse frequency synchronization (`estimateOffset` via phase differences).
//! 4. Fine time/frequency synchronization (cyclic-prefix tracking).
//! 5. Per-symbol FFT (strip guard interval, transform the useful part).
//! 6. Differential demodulation (DQPSK against the previous symbol).
//! 7. Frequency de-interleaving (apply [`freq_interleaver`]) to recover soft bits.
//
// TODO (Week 3-5 sync chain): implement stages 1-7 above. No stub functions are
// provided here on purpose — the foundation must not pretend to demodulate.

#![forbid(unsafe_code)]

pub mod fft;
pub mod freq_interleaver;
pub mod params;
pub mod phasereference;
pub mod phasetable;

pub use fft::Fft;
pub use freq_interleaver::FreqInterleaver;
pub use params::DabParams;
pub use phasereference::phase_reference;
pub use phasetable::get_phi;
