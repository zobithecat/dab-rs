//! `dab-iq` — File-based raw I/Q reader with JSON sidecar metadata.
//!
//! Supported sample formats:
//!
//! | Format              | Encoding                      | Source                        |
//! |---------------------|-------------------------------|-------------------------------|
//! | [`IqFormat::Cs8`]   | Unsigned 8-bit (centred 127.5)| RTL-SDR                       |
//! | [`IqFormat::Cs16Le`]| Signed LE i16 / 32768.0       | Airspy Mini `INT16_IQ`        |
//! | [`IqFormat::Cf32Le`]| LE f32 pair                   | gqrx, SDR#                    |
//! | [`IqFormat::AirspyPacked12`] | Deferred — Week 4+   | libairspy native 12-bit       |
//!
//! # Quick start
//!
//! ```no_run
//! use dab_iq::{IqFileReader, IqFormat};
//! use num_complex::Complex;
//! use std::path::Path;
//!
//! // Explicit format
//! let mut reader = IqFileReader::open(
//!     Path::new("capture.iq"),
//!     IqFormat::Cs16Le,
//!     3_000_000,
//! ).unwrap();
//!
//! let mut buf = vec![Complex::new(0f32, 0f32); 4096];
//! let n = reader.read_samples(&mut buf).unwrap();
//! println!("Read {n} samples");
//!
//! // Auto-detect via sidecar
//! let mut reader2 = IqFileReader::open_with_sidecar(Path::new("capture.iq")).unwrap();
//! let meta = reader2.metadata().unwrap();
//! println!("Channel: {:?}, rate: {} Hz", meta.channel, meta.sample_rate);
//! ```
#![forbid(unsafe_code)]

mod error;
mod format;
mod metadata;
mod reader;

pub use error::IqError;
pub use format::IqFormat;
pub use metadata::IqMetadata;
pub use reader::IqFileReader;

#[cfg(test)]
mod tests {
    use super::*;
    use num_complex::Complex;
    use std::io::Write;
    use std::path::PathBuf;

    /// Write bytes to a unique temp file and return the path.
    fn temp_file(stem: &str, ext: &str, data: &[u8]) -> PathBuf {
        let mut path = std::env::temp_dir();
        // Use process-id + stem for uniqueness between parallel test runs.
        path.push(format!("dab_iq_test_{}_{}.{}", std::process::id(), stem, ext));
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(data).unwrap();
        path
    }

    // ------------------------------------------------------------------
    // 1. Round-trip Cs16Le — various buffer sizes
    // ------------------------------------------------------------------
    #[test]
    fn roundtrip_cs16le_various_buf_sizes() {
        // Build synthetic samples with a range of values.
        let original: Vec<Complex<f32>> = (0..256)
            .map(|i| {
                let angle = (i as f32) * std::f32::consts::TAU / 256.0;
                Complex::new(angle.cos() * 0.9, angle.sin() * 0.9)
            })
            .collect();

        // Encode to Cs16Le bytes.
        let mut bytes: Vec<u8> = Vec::with_capacity(original.len() * 4);
        for s in &original {
            let re = (s.re * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
            let im = (s.im * 32768.0).round().clamp(-32768.0, 32767.0) as i16;
            bytes.extend_from_slice(&re.to_le_bytes());
            bytes.extend_from_slice(&im.to_le_bytes());
        }

        let path = temp_file("cs16le", "iq", &bytes);

        // Tolerance for i16 quantisation: 1 / 32768 ≈ 3e-5.
        let tol = 1.0f32 / 32768.0 + 1e-6;

        for &buf_size in &[1usize, 7, 64, 1024] {
            let mut reader =
                IqFileReader::open(&path, IqFormat::Cs16Le, 3_000_000).unwrap();
            let mut buf = vec![Complex::new(0f32, 0f32); buf_size];
            let mut recovered = Vec::with_capacity(original.len());
            loop {
                let n = reader.read_samples(&mut buf).unwrap();
                if n == 0 {
                    break;
                }
                recovered.extend_from_slice(&buf[..n]);
            }
            assert_eq!(
                recovered.len(),
                original.len(),
                "buf_size={buf_size}: wrong sample count"
            );
            for (idx, (got, exp)) in recovered.iter().zip(original.iter()).enumerate() {
                assert!(
                    (got.re - exp.re).abs() <= tol,
                    "buf_size={buf_size} sample[{idx}] re: got {} exp {}",
                    got.re,
                    exp.re
                );
                assert!(
                    (got.im - exp.im).abs() <= tol,
                    "buf_size={buf_size} sample[{idx}] im: got {} exp {}",
                    got.im,
                    exp.im
                );
            }
        }
        let _ = std::fs::remove_file(&path);
    }

    // ------------------------------------------------------------------
    // 2. Round-trip Cf32Le
    // ------------------------------------------------------------------
    #[test]
    fn roundtrip_cf32le() {
        let original: Vec<Complex<f32>> = (0..128)
            .map(|i| Complex::new(i as f32 * 0.007 - 0.44, -(i as f32) * 0.005 + 0.30))
            .collect();

        let mut bytes: Vec<u8> = Vec::with_capacity(original.len() * 8);
        for s in &original {
            bytes.extend_from_slice(&s.re.to_le_bytes());
            bytes.extend_from_slice(&s.im.to_le_bytes());
        }

        let path = temp_file("cf32le", "iq", &bytes);

        for &buf_size in &[1usize, 7, 1024] {
            let mut reader =
                IqFileReader::open(&path, IqFormat::Cf32Le, 2_048_000).unwrap();
            let mut buf = vec![Complex::new(0f32, 0f32); buf_size];
            let mut recovered = Vec::with_capacity(original.len());
            loop {
                let n = reader.read_samples(&mut buf).unwrap();
                if n == 0 {
                    break;
                }
                recovered.extend_from_slice(&buf[..n]);
            }
            assert_eq!(recovered.len(), original.len(), "buf_size={buf_size}");
            for (idx, (got, exp)) in recovered.iter().zip(original.iter()).enumerate() {
                // f32 round-trip should be bit-exact.
                assert_eq!(
                    got.re, exp.re,
                    "buf_size={buf_size} sample[{idx}] re mismatch"
                );
                assert_eq!(
                    got.im, exp.im,
                    "buf_size={buf_size} sample[{idx}] im mismatch"
                );
            }
        }
        let _ = std::fs::remove_file(&path);
    }

    // ------------------------------------------------------------------
    // 3. Cs8 hand-built conversion
    // ------------------------------------------------------------------
    #[test]
    fn cs8_conversion() {
        // Byte 127 -> (127 - 127.5) / 127.5 ≈ -0.003921…
        // Byte 128 -> (128 - 127.5) / 127.5 ≈ +0.003921…
        // Byte 0   -> (0   - 127.5) / 127.5  = -1.0
        // Byte 255 -> (255 - 127.5) / 127.5  = +1.0
        let bytes: Vec<u8> = vec![
            0, 0,     // sample 0: re=-1, im=-1
            255, 255, // sample 1: re=+1, im=+1
            127, 128, // sample 2: re≈-0.00392, im≈+0.00392
        ];
        let path = temp_file("cs8", "iq", &bytes);
        let mut reader = IqFileReader::open(&path, IqFormat::Cs8, 2_400_000).unwrap();
        let mut buf = vec![Complex::new(0f32, 0f32); 3];
        let n = reader.read_samples(&mut buf).unwrap();
        assert_eq!(n, 3);

        let tol = 1e-5f32;
        assert!((buf[0].re - (-1.0)).abs() < tol, "s0 re={}", buf[0].re);
        assert!((buf[0].im - (-1.0)).abs() < tol, "s0 im={}", buf[0].im);
        assert!((buf[1].re - 1.0).abs() < tol, "s1 re={}", buf[1].re);
        assert!((buf[1].im - 1.0).abs() < tol, "s1 im={}", buf[1].im);
        // (127 - 127.5) / 127.5
        let exp2_re = (127.0f32 - 127.5) / 127.5;
        let exp2_im = (128.0f32 - 127.5) / 127.5;
        assert!((buf[2].re - exp2_re).abs() < tol, "s2 re={}", buf[2].re);
        assert!((buf[2].im - exp2_im).abs() < tol, "s2 im={}", buf[2].im);

        let _ = std::fs::remove_file(&path);
    }

    // ------------------------------------------------------------------
    // 4. Sidecar parse: INT16_IQ -> Cs16Le
    // ------------------------------------------------------------------
    #[test]
    fn sidecar_parse() {
        let json = r#"{
            "schema_version": 1,
            "tag": "test",
            "rf": {
                "channel": "K8B",
                "frequency_hz": 183008000,
                "sample_rate": 3000000,
                "sample_format": "INT16_IQ",
                "operator": "TestOp"
            },
            "some_other_section": { "ignored": true }
        }"#;

        // Empty IQ data (zero samples).
        let iq_bytes: Vec<u8> = Vec::new();
        let iq_path = temp_file("sidecar_test", "iq", &iq_bytes);
        // Write sidecar next to it.
        let json_path = iq_path.with_extension("json");
        std::fs::write(&json_path, json.as_bytes()).unwrap();

        let reader = IqFileReader::open_with_sidecar(&iq_path).unwrap();
        assert_eq!(reader.format(), IqFormat::Cs16Le, "format mismatch");
        assert_eq!(reader.sample_rate(), 3_000_000, "sample rate mismatch");

        let meta = reader.metadata().expect("metadata should be Some");
        assert_eq!(meta.sample_rate, 3_000_000);
        assert_eq!(meta.frequency_hz, 183_008_000);
        assert_eq!(meta.sample_format, "INT16_IQ");
        assert_eq!(meta.channel.as_deref(), Some("K8B"));
        assert_eq!(meta.operator.as_deref(), Some("TestOp"));

        let _ = std::fs::remove_file(&iq_path);
        let _ = std::fs::remove_file(&json_path);
    }

    // ------------------------------------------------------------------
    // 5. AirspyPacked12 -> UnsupportedFormat
    // ------------------------------------------------------------------
    #[test]
    fn airspy_packed12_unsupported() {
        let path = temp_file("packed12", "iq", &[]);
        let err = IqFileReader::open(&path, IqFormat::AirspyPacked12, 10_000_000)
            .expect_err("should fail");
        assert!(
            matches!(err, IqError::UnsupportedFormat(IqFormat::AirspyPacked12)),
            "unexpected error: {err}"
        );
        let _ = std::fs::remove_file(&path);
    }
}
