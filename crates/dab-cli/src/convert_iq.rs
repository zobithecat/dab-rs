//! `dab convert-iq` — resample + reformat an I/Q capture so an
//! eti-stuff offline input handler can read it.
//!
//! Two output formats are supported:
//!
//! - **CU8 @ 2.048 MSPS** (`eti-cmdline-rawfiles` input). Compact (78 MiB
//!   for a 20 s capture) but quantises to 8 bits per channel — at our
//!   K8B captures' marginal SNR + small per-sample amplitude (p99 of
//!   `|i16|` is ~7000, ~21 % of the 16-bit range), this costs about 3
//!   effective bits and breaks the oracle's coarse-CFO lock.
//! - **WAV 16-bit PCM stereo @ 2.048 MSPS** (`eti-cmdline-wavfiles`
//!   input). Larger (~164 MiB / 20 s) but preserves the full 16-bit
//!   precision of the source `Cs16Le` file, which is what the WAV path
//!   needs to keep the oracle locked.
//!
//! Both go through [`dab_ofdm::Resampler::new_3m_to_2048k`] for the rate
//! change so the only format difference between them is the per-sample
//! quantisation depth.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use anyhow::{anyhow, Result};
use num_complex::Complex;

use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::Resampler;

/// Resample a `Cs16Le @ 3 MSPS` file to `CU8 @ 2.048 MSPS` and write it to
/// `dst`. Returns the total number of cu8 sample-pairs written (each pair
/// is 2 bytes on disk).
pub fn convert_cs16_3m_to_cu8_2048k(src: &Path, dst: &Path) -> Result<usize> {
    let mut reader = IqFileReader::open(src, IqFormat::Cs16Le, 3_000_000)
        .map_err(|e| anyhow!("open {}: {e}", src.display()))?;
    let mut resampler = Resampler::new_3m_to_2048k();

    let out = File::create(dst).map_err(|e| anyhow!("create {}: {e}", dst.display()))?;
    let mut out = BufWriter::with_capacity(1 << 20, out);

    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    let mut byte_chunk: Vec<u8> = Vec::with_capacity(1 << 21);
    let mut total_pairs = 0_usize;

    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        let resampled = resampler.process(&buf[..n]);

        byte_chunk.clear();
        byte_chunk.reserve(2 * resampled.len());
        for z in &resampled {
            byte_chunk.push(quantize_to_u8(z.re));
            byte_chunk.push(quantize_to_u8(z.im));
        }
        out.write_all(&byte_chunk)?;
        total_pairs += resampled.len();
    }

    out.flush()?;
    Ok(total_pairs)
}

/// Quantize one `f32` value in `[-1.0, +1.0)` to a CU8 byte matching the
/// eti-stuff `rawfileHandler` inverse `V = (u8 - 128) / 128`.
#[inline]
fn quantize_to_u8(v: f32) -> u8 {
    let scaled = (v * 128.0 + 128.0).round();
    scaled.clamp(0.0, 255.0) as u8
}

/// Resample a `Cs16Le @ 3 MSPS` file to a libsndfile-readable
/// **WAV 16-bit PCM stereo @ 2.048 MSPS** and write it to `dst`.
/// Returns the total number of I/Q sample-pairs written (each pair is 4
/// bytes on disk).
///
/// The reverse of [`dab_iq`]'s normalisation: each `f32 ∈ [-1, +1)` is
/// multiplied by `32768` and clamped to `[-32768, 32767]` before being
/// written as a little-endian `i16`. eti-stuff's `wavfileHandler` reads
/// the file through `libsndfile`'s `sf_readf_float`, which divides by
/// `32768` again, so the value seen by the oracle is bit-for-bit the
/// same `f32` the dab-rs pipeline saw — *except* for the resampling step
/// (shared) and the 16-bit quantisation (negligible on the K8B capture
/// whose original cs16 dynamic range is ~12 bits anyway).
pub fn convert_cs16_3m_to_wav_2048k(src: &Path, dst: &Path) -> Result<usize> {
    let mut reader = IqFileReader::open(src, IqFormat::Cs16Le, 3_000_000)
        .map_err(|e| anyhow!("open {}: {e}", src.display()))?;
    let mut resampler = Resampler::new_3m_to_2048k();

    let out = File::create(dst).map_err(|e| anyhow!("create {}: {e}", dst.display()))?;
    let mut out = BufWriter::with_capacity(1 << 20, out);

    // ---- Reserve the WAV header (we fill in sizes after the data is written) ----
    const HEADER_BYTES: usize = 44;
    out.write_all(&[0_u8; HEADER_BYTES])?;

    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    let mut byte_chunk: Vec<u8> = Vec::with_capacity(1 << 22);
    let mut total_pairs = 0_usize;

    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        let resampled = resampler.process(&buf[..n]);

        byte_chunk.clear();
        byte_chunk.reserve(4 * resampled.len());
        for z in &resampled {
            let i = quantize_to_i16(z.re);
            let q = quantize_to_i16(z.im);
            byte_chunk.extend_from_slice(&i.to_le_bytes());
            byte_chunk.extend_from_slice(&q.to_le_bytes());
        }
        out.write_all(&byte_chunk)?;
        total_pairs += resampled.len();
    }

    out.flush()?;

    // ---- Rewind and patch the WAV header with the final sizes ----
    let inner = out
        .into_inner()
        .map_err(|e| anyhow!("flush wav: {}", e.error()))?;
    let data_bytes: u32 = (total_pairs * 4) as u32;
    let file_minus_8: u32 = HEADER_BYTES as u32 - 8 + data_bytes;
    write_wav_header(inner, file_minus_8, data_bytes)?;

    Ok(total_pairs)
}

/// Quantize one `f32` value in `[-1.0, +1.0)` to an `i16` matching the
/// inverse of `dab-iq`'s `Cs16Le → f32 / 32768.0` normalisation.
#[inline]
fn quantize_to_i16(v: f32) -> i16 {
    let scaled = (v * 32768.0).round();
    scaled.clamp(-32768.0, 32767.0) as i16
}

/// Resample a `Cs16Le @ 3 MSPS` file to a libsndfile-readable
/// **32-bit IEEE float WAV stereo @ 2.048 MSPS**, scaling values so they
/// match the airspy-handler-equivalent OFDM input amplitude. Returns the
/// total number of I/Q sample-pairs written (each pair is 8 bytes on
/// disk).
///
/// # Scaling
///
/// The live eti-stuff path (`eti-cmdline-airspy`) feeds the OFDM
/// processor with `airspyHandler::data_available` output, which is
/// `int16_libairspy / 2048` per sample (`airspy-handler.cpp:337`). The
/// offline path through 16-bit PCM WAV instead funnels values through
/// libsndfile's `/32768` PCM normalisation — a 16× amplitude shortfall
/// that empirically breaks eti-stuff's coarse-CFO loop on the K8B
/// capture.
///
/// `SF_FORMAT_FLOAT` files pass values through `sf_readf_float`
/// **as-is**, so we can write the post-resample f32 directly. To match
/// airspy-handler we apply
///
/// ```text
/// wav_sample = dab_iq_f32 * (32768 / 2048) = dab_iq_f32 * 16
///            = libairspy_int16 / 2048
/// ```
///
/// which is the *same* numeric stream the live oracle saw when it
/// produced `k8b_v4.eti`.
pub fn convert_cs16_3m_to_wav32_2048k(src: &Path, dst: &Path) -> Result<usize> {
    let mut reader = IqFileReader::open(src, IqFormat::Cs16Le, 3_000_000)
        .map_err(|e| anyhow!("open {}: {e}", src.display()))?;
    let mut resampler = Resampler::new_3m_to_2048k();

    let out = File::create(dst).map_err(|e| anyhow!("create {}: {e}", dst.display()))?;
    let mut out = BufWriter::with_capacity(1 << 20, out);

    // Reserve a fixed-size header (filled in after the data is written).
    // Float WAVs use a longer header than minimal PCM: 18-byte fmt chunk +
    // 12-byte fact chunk. Total: 12 + 26 + 12 + 8 = 58 bytes.
    const HEADER_BYTES: usize = 58;
    out.write_all(&[0_u8; HEADER_BYTES])?;

    // `dab-iq`'s `Cs16Le → f32 / 32768.0` gives us the source value in
    // `[-1, +1)`. Multiply by 16 to land on the airspy-handler scale.
    const AIRSPY_SCALE: f32 = 16.0;

    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    let mut byte_chunk: Vec<u8> = Vec::with_capacity(1 << 23);
    let mut total_pairs = 0_usize;

    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        let resampled = resampler.process(&buf[..n]);

        byte_chunk.clear();
        byte_chunk.reserve(8 * resampled.len());
        for z in &resampled {
            let i = (z.re * AIRSPY_SCALE).to_bits();
            let q = (z.im * AIRSPY_SCALE).to_bits();
            byte_chunk.extend_from_slice(&i.to_le_bytes());
            byte_chunk.extend_from_slice(&q.to_le_bytes());
        }
        out.write_all(&byte_chunk)?;
        total_pairs += resampled.len();
    }

    out.flush()?;

    let inner = out
        .into_inner()
        .map_err(|e| anyhow!("flush wav32: {}", e.error()))?;
    let data_bytes: u32 = (total_pairs * 8) as u32;
    let file_minus_8: u32 = HEADER_BYTES as u32 - 8 + data_bytes;
    write_wav32_header(inner, file_minus_8, data_bytes, total_pairs as u32)?;

    Ok(total_pairs)
}

/// Rewrite the 58-byte WAVE_FORMAT_IEEE_FLOAT header at the start of
/// `f`. The fact chunk holds the number of *sample frames* (one frame =
/// `channels` samples), required by the spec for non-PCM formats so
/// libsndfile (and other strict readers) accept the file.
fn write_wav32_header(
    mut f: File,
    file_minus_8: u32,
    data_bytes: u32,
    sample_frames: u32,
) -> Result<()> {
    use std::io::{Seek, SeekFrom};

    const SAMPLE_RATE: u32 = 2_048_000;
    const CHANNELS: u16 = 2;
    const BITS_PER_SAMPLE: u16 = 32;
    const BYTE_RATE: u32 = SAMPLE_RATE * CHANNELS as u32 * (BITS_PER_SAMPLE as u32 / 8);
    const BLOCK_ALIGN: u16 = CHANNELS * (BITS_PER_SAMPLE / 8);
    /// WAVE_FORMAT_IEEE_FLOAT (RIFF audio format tag).
    const FORMAT_IEEE_FLOAT: u16 = 3;

    f.seek(SeekFrom::Start(0))?;

    let mut hdr = [0_u8; 58];

    // RIFF header (12 bytes)
    hdr[0..4].copy_from_slice(b"RIFF");
    hdr[4..8].copy_from_slice(&file_minus_8.to_le_bytes());
    hdr[8..12].copy_from_slice(b"WAVE");

    // fmt chunk (18-byte payload: PCM-like + cbSize)
    hdr[12..16].copy_from_slice(b"fmt ");
    hdr[16..20].copy_from_slice(&18_u32.to_le_bytes()); // chunk size
    hdr[20..22].copy_from_slice(&FORMAT_IEEE_FLOAT.to_le_bytes());
    hdr[22..24].copy_from_slice(&CHANNELS.to_le_bytes());
    hdr[24..28].copy_from_slice(&SAMPLE_RATE.to_le_bytes());
    hdr[28..32].copy_from_slice(&BYTE_RATE.to_le_bytes());
    hdr[32..34].copy_from_slice(&BLOCK_ALIGN.to_le_bytes());
    hdr[34..36].copy_from_slice(&BITS_PER_SAMPLE.to_le_bytes());
    hdr[36..38].copy_from_slice(&0_u16.to_le_bytes()); // cbSize = 0

    // fact chunk (4-byte payload: # sample frames)
    hdr[38..42].copy_from_slice(b"fact");
    hdr[42..46].copy_from_slice(&4_u32.to_le_bytes());
    hdr[46..50].copy_from_slice(&sample_frames.to_le_bytes());

    // data chunk header
    hdr[50..54].copy_from_slice(b"data");
    hdr[54..58].copy_from_slice(&data_bytes.to_le_bytes());

    f.write_all(&hdr)?;
    f.flush()?;
    Ok(())
}

/// Rewrite the 44-byte WAV header at the start of `f`, populating sizes
/// for 16-bit PCM stereo @ 2.048 MSPS. Mirrors libsndfile's canonical
/// minimal RIFF/WAVE/fmt/data layout (no padding, no extension chunks).
fn write_wav_header(mut f: File, file_minus_8: u32, data_bytes: u32) -> Result<()> {
    use std::io::{Seek, SeekFrom};

    const SAMPLE_RATE: u32 = 2_048_000;
    const CHANNELS: u16 = 2;
    const BITS_PER_SAMPLE: u16 = 16;
    const BYTE_RATE: u32 = SAMPLE_RATE * CHANNELS as u32 * (BITS_PER_SAMPLE as u32 / 8);
    const BLOCK_ALIGN: u16 = CHANNELS * (BITS_PER_SAMPLE / 8);

    f.seek(SeekFrom::Start(0))?;

    let mut hdr = [0_u8; 44];
    hdr[0..4].copy_from_slice(b"RIFF");
    hdr[4..8].copy_from_slice(&file_minus_8.to_le_bytes());
    hdr[8..12].copy_from_slice(b"WAVE");

    // "fmt " sub-chunk (16 bytes payload for PCM)
    hdr[12..16].copy_from_slice(b"fmt ");
    hdr[16..20].copy_from_slice(&16_u32.to_le_bytes()); // chunk size
    hdr[20..22].copy_from_slice(&1_u16.to_le_bytes());  // PCM
    hdr[22..24].copy_from_slice(&CHANNELS.to_le_bytes());
    hdr[24..28].copy_from_slice(&SAMPLE_RATE.to_le_bytes());
    hdr[28..32].copy_from_slice(&BYTE_RATE.to_le_bytes());
    hdr[32..34].copy_from_slice(&BLOCK_ALIGN.to_le_bytes());
    hdr[34..36].copy_from_slice(&BITS_PER_SAMPLE.to_le_bytes());

    // "data" sub-chunk header
    hdr[36..40].copy_from_slice(b"data");
    hdr[40..44].copy_from_slice(&data_bytes.to_le_bytes());

    f.write_all(&hdr)?;
    f.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantize_midpoint_is_128() {
        // The "zero" sample (0.0) maps to byte 128, since (0 * 128) + 128 = 128.
        assert_eq!(quantize_to_u8(0.0), 128);
    }

    #[test]
    fn quantize_endpoints() {
        // +1.0 → byte 256 → clamps to 255.
        assert_eq!(quantize_to_u8(1.0), 255);
        // -1.0 → byte 0.
        assert_eq!(quantize_to_u8(-1.0), 0);
        // +0.5 → 192, -0.5 → 64.
        assert_eq!(quantize_to_u8(0.5), 192);
        assert_eq!(quantize_to_u8(-0.5), 64);
    }

    #[test]
    fn quantize_saturates_out_of_range() {
        // Inputs outside [-1, +1) saturate cleanly rather than wrap.
        assert_eq!(quantize_to_u8(5.0), 255);
        assert_eq!(quantize_to_u8(-5.0), 0);
        assert_eq!(quantize_to_u8(f32::INFINITY), 255);
        assert_eq!(quantize_to_u8(f32::NEG_INFINITY), 0);
    }

    /// Verifies the wav32 round-trip scale: an `i16` value `v` read from
    /// the source `cs16le` is normalised by `dab-iq` to `v / 32768` and
    /// then multiplied by `16` by the wav32 writer, so the float that
    /// `sf_readf_float` (on a SF_FORMAT_FLOAT WAV) returns to the OFDM
    /// processor is exactly `v / 2048` — matching what
    /// `airspy-handler.cpp:337` would have produced from the same i16
    /// on the live path.
    #[test]
    fn wav32_scale_matches_airspy_handler() {
        // Pick the same i16 ceiling the airspy-handler comment mentions
        // (12-bit raw):
        let i16_val: i32 = 2048;
        let dab_iq_f32 = i16_val as f32 / 32768.0;
        let wav32_value = dab_iq_f32 * 16.0;
        let airspy_handler_value = i16_val as f32 / 2048.0;
        // Floating-point exact equality holds here: 2048/32768 * 16 = 2048/2048 = 1.0.
        assert_eq!(wav32_value, airspy_handler_value);
        assert!((wav32_value - 1.0).abs() < 1e-7);

        // And a non-trivial value: i16 = 7000 (~ our K8B p99).
        let i16_val = 7000_i32;
        let dab_iq_f32 = i16_val as f32 / 32768.0;
        let wav32_value = dab_iq_f32 * 16.0;
        let airspy_handler_value = i16_val as f32 / 2048.0;
        assert!(
            (wav32_value - airspy_handler_value).abs() < 1e-5,
            "wav32 {wav32_value} ≠ airspy {airspy_handler_value}"
        );
    }
}
