//! Error type for the `dab-iq` crate.

use crate::format::IqFormat;

/// Errors that can occur while opening or reading an I/Q file.
#[derive(thiserror::Error, Debug)]
pub enum IqError {
    /// An underlying I/O error (file not found, permission denied, …).
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// The requested format is not yet supported.
    ///
    /// [`IqFormat::AirspyPacked12`] is deferred to Week 4+ when the libairspy
    /// FFI layer is wired up. Attempting to open a file with this format returns
    /// this error immediately.
    #[error("unsupported I/Q format {0:?} — AirspyPacked12 is deferred to Week 4+")]
    UnsupportedFormat(IqFormat),

    /// The JSON sidecar file could not be parsed.
    #[error("bad sidecar JSON: {0}")]
    BadSidecar(#[from] serde_json::Error),

    /// The JSON sidecar was parsed but lacked a required field or had an
    /// unrecognised `sample_format` value.
    #[error("sidecar field error: {0}")]
    SidecarField(String),
}
