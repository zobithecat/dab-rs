//! JSON sidecar metadata.
//!
//! The sidecar file (`<stem>.json`) lives next to the `.iq` capture file. Fields are
//! nested under an `"rf"` object. Unknown top-level and `"rf"`-level keys are silently
//! ignored by `serde`.

use serde::Deserialize;

/// The `"rf"` sub-object parsed from the sidecar JSON.
///
/// Unknown fields are silently ignored so that new sidecar keys added in future
/// captures (or by other tools) do not break the reader.
#[derive(Debug, Clone, Deserialize)]
pub struct RfSection {
    /// Sample rate in Hz (e.g. 3_000_000).
    pub sample_rate: u32,
    /// Centre frequency in Hz.
    pub frequency_hz: u64,
    /// Sample encoding string, e.g. `"INT16_IQ"`.
    pub sample_format: String,
    /// RF channel label (e.g. `"K8B"`). Optional.
    #[serde(default)]
    pub channel: Option<String>,
    /// Operator / ensemble label. Optional.
    #[serde(default)]
    pub operator: Option<String>,
}

/// Parsed subset of the JSON sidecar.
///
/// Fields that are optional or frequently absent carry `Option<T>` types so that
/// older or third-party sidecars are accepted without errors.
#[derive(Debug, Clone, Deserialize)]
pub struct IqMetadata {
    /// Sample rate in Hz from `rf.sample_rate`.
    pub sample_rate: u32,
    /// Centre frequency in Hz from `rf.frequency_hz`.
    pub frequency_hz: u64,
    /// Encoding string from `rf.sample_format` (e.g. `"INT16_IQ"`).
    pub sample_format: String,
    /// RF channel label from `rf.channel`. Optional.
    #[serde(default)]
    pub channel: Option<String>,
    /// Operator / ensemble label from `rf.operator`. Optional.
    #[serde(default)]
    pub operator: Option<String>,
}

/// Top-level sidecar wrapper — only the `"rf"` section is required.
#[derive(Deserialize)]
pub(crate) struct SidecarFile {
    pub rf: RfSection,
}

impl From<RfSection> for IqMetadata {
    fn from(rf: RfSection) -> Self {
        IqMetadata {
            sample_rate: rf.sample_rate,
            frequency_hz: rf.frequency_hz,
            sample_format: rf.sample_format,
            channel: rf.channel,
            operator: rf.operator,
        }
    }
}
