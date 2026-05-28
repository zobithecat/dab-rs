//! `dab convert-iq` — resample + quantize an I/Q capture so the oracle's
//! rawfiles input handler can read it.
//!
//! eti-stuff's `eti-cmdline-rawfiles` expects samples in the CU8 layout
//! `(uint8 I, uint8 Q)` at 2.048 MSPS (the internal Mode I rate). Our K8B
//! captures are `Cs16Le` at 3 MSPS. This converter reuses
//! [`dab_ofdm::Resampler::new_3m_to_2048k`] to do the rate change, then
//! quantizes each `Complex<f32>` sample to a pair of `u8` bytes:
//!
//! ```text
//! byte = clamp(round(value * 128.0 + 128.0), 0, 255) as u8
//! ```
//!
//! The mapping matches the inverse of `rawfileHandler::getSamples`
//! (`V = (u8 - 128) / 128`), so feeding the cu8 file back through
//! `eti-cmdline-rawfiles` recovers a stream within ±1/128 of the input.

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
}
