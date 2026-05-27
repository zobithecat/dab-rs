//! Week 3a golden integration test: OFDM sync chain stages 1-2 on the real
//! K8B raw I/Q capture.
//!
//! Pipeline: dab-iq reads `k8b_rust.iq` (INT16_IQ @ 3 MSPS) -> Stage 1
//! polyphase resample 3 -> 2.048 MSPS -> Stage 2 adaptive null-symbol
//! detection. Cross-checked against the Python reference
//! `airspy-mini-dmb/tools/iq_validate_dab.py`, whose `gate_null` found
//! 209 dips / 207 at the 96.00 ms DAB Mode I null cadence on the same file.
//!
//! The 240 MB capture is Git-LFS / not committed here. Provide a local copy:
//!   export DAB_RS_K8B_IQ=/path/to/k8b_rust.iq
//!   cargo test -p dab-ofdm --test k8b_rust_ofdm_stage12 -- --include-ignored
//! Absent the file the test is skipped (it is `#[ignore]`d).

use num_complex::Complex;

use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::{NullDetector, Resampler};

const DEFAULT_IQ: &str =
    "/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_rust.iq";

const CAPTURE_SAMPLES: usize = 60_000_000; // 20.06 s @ 3 MSPS
const RESAMPLED_SAMPLES: usize = 40_960_000; // 60e6 * 256/375, exact
const FRAME_2048K: usize = 196_608; // 96 ms @ 2.048 MSPS

fn capture_path() -> std::path::PathBuf {
    std::env::var("DAB_RS_K8B_IQ")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| std::path::PathBuf::from(DEFAULT_IQ))
}

#[test]
#[ignore = "requires the K8B raw I/Q capture; run with --include-ignored"]
fn k8b_rust_resample_and_null_detect() {
    let path = capture_path();
    if !path.exists() {
        eprintln!("skipping: capture not found at {}", path.display());
        return;
    }

    // Stage 0: read raw I/Q (INT16_IQ @ 3 MSPS) and stream through Stage 1.
    let mut reader =
        IqFileReader::open(&path, IqFormat::Cs16Le, 3_000_000).expect("open capture");
    let mut resampler = Resampler::new_3m_to_2048k();

    let mut in_count = 0usize;
    let mut resampled: Vec<Complex<f32>> = Vec::with_capacity(RESAMPLED_SAMPLES + 1024);
    let mut buf = vec![Complex::new(0.0f32, 0.0f32); 1 << 20]; // 1 Msample chunks
    loop {
        let n = reader.read_samples(&mut buf).expect("read");
        if n == 0 {
            break;
        }
        in_count += n;
        resampled.extend_from_slice(&resampler.process(&buf[..n]));
    }

    // Stage 1 invariants.
    assert_eq!(in_count, CAPTURE_SAMPLES, "input sample count");
    let delta = resampled.len() as i64 - RESAMPLED_SAMPLES as i64;
    assert!(
        delta.abs() <= 8,
        "resampled length {} != {} (±8); delta={delta}",
        resampled.len(),
        RESAMPLED_SAMPLES
    );

    // Stage 2: adaptive null detection on the 2.048 MSPS stream.
    let det = NullDetector::new(2_048_000);
    let res = det.detect(&resampled);

    // 20.06 s / 96 ms ≈ 209 frames; the Python reference recovered 207 at the
    // 96 ms cadence. Require a solid majority of frames detected.
    assert!(
        res.positions.len() >= 200,
        "expected >= 200 null dips, got {} (Python reference: 207)",
        res.positions.len()
    );

    let fp = res.frame_period.expect("frame period should lock");
    let fp_err = fp as i64 - FRAME_2048K as i64;
    assert!(
        fp_err.abs() <= 64,
        "frame_period {fp} != {FRAME_2048K} (±64); err={fp_err}"
    );

    eprintln!(
        "OK: in={in_count} resampled={} dips={} frame_period={fp} (target {FRAME_2048K})",
        resampled.len(),
        res.positions.len(),
    );
}
