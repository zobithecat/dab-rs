//! I/Q sample format definitions.
//!
//! Supported formats:
//!
//! | Variant           | Bytes/sample | Conversion to `Complex<f32>`                   |
//! |-------------------|-------------|------------------------------------------------|
//! | `Cs8`             | 2           | `(byte as f32 - 127.5) / 127.5`               |
//! | `Cs16Le`          | 4           | `i16_le as f32 / 32768.0`                      |
//! | `Cf32Le`          | 8           | identity (native `f32` LE pair)                |
//! | `AirspyPacked12`  | —           | **Deferred — Week 4+ via libairspy**           |
//!
//! ## Cs8 convention
//! RTL-SDR and many cheap SDRs output unsigned 8-bit I/Q bytes whose zero-signal centre
//! is 127 or 128. The convention used here — `(byte as f32 - 127.5) / 127.5` — places
//! the centred signal at exactly 0.0 and scales the full 0-255 range to roughly ±1.0.
//!
//! ## Cs16Le convention
//! The Airspy Mini outputs signed 12-bit samples packed into 16-bit words. When read as
//! `INT16_IQ` (the mode used in K8B captures) the range is actually limited to ±2048,
//! but we normalise by 32768.0 to be consistent with software that treats the stream as
//! full-range 16-bit. **This matches the Python reference** `tools/iq_validate_dab.py`:
//! ```python
//! samples = np.fromfile(path, dtype="<i2") / 32768.0
//! ```
//! Keeping the divisor identical ensures numeric agreement between the Rust pipeline and
//! the Python validation oracle.

/// Raw sample encoding on disk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IqFormat {
    /// Interleaved unsigned 8-bit I/Q (RTL-SDR style).
    /// Centred at 127.5: `(byte as f32 - 127.5) / 127.5`.
    Cs8,
    /// Interleaved signed little-endian i16 I/Q (Airspy `INT16_IQ`).
    /// `i16_le as f32 / 32768.0` — matches the Python reference.
    Cs16Le,
    /// Interleaved little-endian f32 complex (gqrx default).
    /// Read directly as native `Complex<f32>`.
    Cf32Le,
    /// Airspy packed 12-bit I/Q. **Deferred to Week 4+** (requires libairspy).
    /// `IqFileReader::open` and `read_samples` return [`IqError::UnsupportedFormat`].
    AirspyPacked12,
}

impl IqFormat {
    /// Number of raw bytes consumed per complex sample, or `None` for unsupported formats.
    pub fn bytes_per_sample(self) -> Option<usize> {
        match self {
            IqFormat::Cs8 => Some(2),
            IqFormat::Cs16Le => Some(4),
            IqFormat::Cf32Le => Some(8),
            IqFormat::AirspyPacked12 => None,
        }
    }

    /// Try to parse the `rf.sample_format` string from a JSON sidecar.
    ///
    /// Recognised strings (case-insensitive):
    /// - `"INT16_IQ"` → [`IqFormat::Cs16Le`]
    /// - `"CS8"` / `"UINT8"` → [`IqFormat::Cs8`]
    /// - `"FLOAT32_IQ"` / `"CF32"` → [`IqFormat::Cf32Le`]
    pub fn from_sidecar_str(s: &str) -> Option<Self> {
        match s.to_uppercase().as_str() {
            "INT16_IQ" => Some(IqFormat::Cs16Le),
            "CS8" | "UINT8" => Some(IqFormat::Cs8),
            "FLOAT32_IQ" | "CF32" => Some(IqFormat::Cf32Le),
            _ => None,
        }
    }
}
