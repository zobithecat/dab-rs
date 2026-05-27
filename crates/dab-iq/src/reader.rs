//! File-based I/Q reader with per-format sample conversion.
//!
//! # Buffer handling
//!
//! Each sample format has a fixed number of bytes per complex sample (2, 4, or 8). The
//! reader maintains a small internal `remainder` buffer — at most
//! `bytes_per_sample - 1` bytes — so that partial reads at chunk boundaries never lose
//! data. This means `read_samples` is safe to call with any buffer size, including 1.

use std::{
    fs::File,
    io::{BufReader, Read},
    path::Path,
};

use num_complex::Complex;

use crate::{
    error::IqError,
    format::IqFormat,
    metadata::{IqMetadata, SidecarFile},
};

/// Chunk size for buffered file reads (128 KiB of raw bytes).
const READ_CHUNK: usize = 128 * 1024;

/// File-based raw I/Q reader.
///
/// Supports [`IqFormat::Cs8`], [`IqFormat::Cs16Le`], and [`IqFormat::Cf32Le`].
/// [`IqFormat::AirspyPacked12`] is deferred — `open` returns
/// [`IqError::UnsupportedFormat`] immediately.
#[derive(Debug)]
pub struct IqFileReader {
    inner: BufReader<File>,
    format: IqFormat,
    sample_rate: u32,
    metadata: Option<IqMetadata>,
    /// Incomplete bytes from the last read that did not fill a complete sample tuple.
    remainder: Vec<u8>,
}

impl IqFileReader {
    /// Open a raw I/Q file with an explicit format and sample rate.
    ///
    /// Returns [`IqError::UnsupportedFormat`] for [`IqFormat::AirspyPacked12`].
    pub fn open(path: &Path, format: IqFormat, sample_rate: u32) -> Result<Self, IqError> {
        if format.bytes_per_sample().is_none() {
            return Err(IqError::UnsupportedFormat(format));
        }
        let file = File::open(path)?;
        Ok(IqFileReader {
            inner: BufReader::new(file),
            format,
            sample_rate,
            metadata: None,
            remainder: Vec::new(),
        })
    }

    /// Open using a JSON sidecar (`<stem>.json`) to pick format and sample rate.
    ///
    /// The sidecar must reside next to the `.iq` file and share the same file stem.
    /// The `rf.sample_format` field is mapped to [`IqFormat`] via
    /// [`IqFormat::from_sidecar_str`]; unknown values produce
    /// [`IqError::SidecarField`].
    pub fn open_with_sidecar(path: &Path) -> Result<Self, IqError> {
        // Build sidecar path: replace extension with ".json"
        let sidecar_path = path.with_extension("json");
        let sidecar_bytes = std::fs::read(&sidecar_path)?;
        let sidecar: SidecarFile = serde_json::from_slice(&sidecar_bytes)?;

        let rf = sidecar.rf;
        let format = IqFormat::from_sidecar_str(&rf.sample_format).ok_or_else(|| {
            IqError::SidecarField(format!(
                "unrecognised sample_format {:?} in sidecar {:?}",
                rf.sample_format, sidecar_path
            ))
        })?;

        if format.bytes_per_sample().is_none() {
            return Err(IqError::UnsupportedFormat(format));
        }

        let metadata: IqMetadata = rf.into();
        let sample_rate = metadata.sample_rate;

        let file = File::open(path)?;
        Ok(IqFileReader {
            inner: BufReader::new(file),
            format,
            sample_rate,
            metadata: Some(metadata),
            remainder: Vec::new(),
        })
    }

    /// Fill `buf` with up to `buf.len()` complex samples.
    ///
    /// Returns the number of samples actually written (0 at EOF).
    /// Partial reads across chunk boundaries are handled transparently via the
    /// internal `remainder` buffer — at most `bytes_per_sample - 1` leftover bytes
    /// are carried between calls.
    pub fn read_samples(&mut self, buf: &mut [Complex<f32>]) -> Result<usize, IqError> {
        let bps = match self.format.bytes_per_sample() {
            Some(n) => n,
            None => return Err(IqError::UnsupportedFormat(self.format)),
        };

        if buf.is_empty() {
            return Ok(0);
        }

        let want_bytes = buf.len() * bps;
        // Allocate a raw byte buffer pre-seeded with any leftover bytes.
        let mut raw: Vec<u8> = Vec::with_capacity(self.remainder.len() + want_bytes);
        raw.extend_from_slice(&self.remainder);
        self.remainder.clear();

        // Read in chunks until we have enough bytes or hit EOF.
        let still_needed = want_bytes.saturating_sub(raw.len());
        let to_read = still_needed.min(READ_CHUNK).max(bps);
        let mut chunk = vec![0u8; to_read];
        loop {
            if raw.len() >= want_bytes {
                break;
            }
            let available = want_bytes - raw.len();
            let read_size = available.min(READ_CHUNK);
            if chunk.len() != read_size {
                chunk.resize(read_size, 0);
            }
            let n = self.inner.read(&mut chunk)?;
            if n == 0 {
                break; // EOF
            }
            raw.extend_from_slice(&chunk[..n]);
        }

        // How many complete samples do we have?
        let complete = raw.len() / bps;
        let used = complete.min(buf.len());
        let bytes_used = used * bps;

        // Save leftover bytes for the next call.
        self.remainder.extend_from_slice(&raw[bytes_used..]);

        // Convert bytes to Complex<f32>.
        let sample_bytes = &raw[..bytes_used];
        match self.format {
            IqFormat::Cs8 => convert_cs8(sample_bytes, &mut buf[..used]),
            IqFormat::Cs16Le => convert_cs16le(sample_bytes, &mut buf[..used]),
            IqFormat::Cf32Le => convert_cf32le(sample_bytes, &mut buf[..used]),
            IqFormat::AirspyPacked12 => unreachable!("checked above"),
        }

        Ok(used)
    }

    /// The format this reader was opened with.
    pub fn format(&self) -> IqFormat {
        self.format
    }

    /// Sample rate in Hz.
    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Sidecar metadata, if the file was opened via [`Self::open_with_sidecar`].
    pub fn metadata(&self) -> Option<&IqMetadata> {
        self.metadata.as_ref()
    }
}

// ---------------------------------------------------------------------------
// Per-format converters
// ---------------------------------------------------------------------------

/// Cs8: interleaved unsigned 8-bit. `(byte as f32 - 127.5) / 127.5`.
fn convert_cs8(bytes: &[u8], out: &mut [Complex<f32>]) {
    for (i, s) in out.iter_mut().enumerate() {
        let re = (bytes[2 * i] as f32 - 127.5) / 127.5;
        let im = (bytes[2 * i + 1] as f32 - 127.5) / 127.5;
        *s = Complex::new(re, im);
    }
}

/// Cs16Le: interleaved signed LE i16. `i16_le as f32 / 32768.0`.
fn convert_cs16le(bytes: &[u8], out: &mut [Complex<f32>]) {
    for (i, s) in out.iter_mut().enumerate() {
        let re_raw = i16::from_le_bytes([bytes[4 * i], bytes[4 * i + 1]]);
        let im_raw = i16::from_le_bytes([bytes[4 * i + 2], bytes[4 * i + 3]]);
        *s = Complex::new(re_raw as f32 / 32768.0, im_raw as f32 / 32768.0);
    }
}

/// Cf32Le: interleaved little-endian f32. Read directly.
fn convert_cf32le(bytes: &[u8], out: &mut [Complex<f32>]) {
    for (i, s) in out.iter_mut().enumerate() {
        let re = f32::from_le_bytes([
            bytes[8 * i],
            bytes[8 * i + 1],
            bytes[8 * i + 2],
            bytes[8 * i + 3],
        ]);
        let im = f32::from_le_bytes([
            bytes[8 * i + 4],
            bytes[8 * i + 5],
            bytes[8 * i + 6],
            bytes[8 * i + 7],
        ]);
        *s = Complex::new(re, im);
    }
}
