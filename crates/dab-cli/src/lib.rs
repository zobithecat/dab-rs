//! `dab-cli` library — pipeline orchestrators for the `dab` CLI binary.
//!
//! The CLI's `main.rs` is a thin clap front-end. The actual stage-spanning
//! orchestration logic (e.g. raw I/Q → FIC ensemble) lives here so it can be
//! exercised from integration tests and reused from external callers.

#![forbid(unsafe_code)]

pub mod fic_iq;
