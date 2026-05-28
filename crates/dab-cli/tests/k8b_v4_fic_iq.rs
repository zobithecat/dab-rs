//! End-to-end FIC integration test: raw I/Q → ensemble decode.
//!
//! Drives the full OFDM-to-FIC pipeline on the K8B oracle capture `k8b_v4.iq`
//! and asserts the ensemble configuration matches the live ground truth
//! recorded into `k8b_v4.eti` by stock `eti-cmdline-airspy` (per
//! `airspy-mini-dmb/data/captures/k8b_v4.json`):
//!
//! - EId = 0xE040 (YTN DMB)
//! - Ensemble label contains "YTN"
//! - At least 4 services discovered (oracle: 5 — mYTN, HD mYTN, 4DRIVE,
//!   LOTTE Homeshop, YTN EWS)
//! - FIB CRC pass rate ≥ 40 % of total (oracle's fibquality is 100 with the
//!   raw count ≈ 75 %; we accept a margin to allow occasional frame
//!   mis-syncs through marginal SNR)
//!
//! # Current status: FAILING — the pipeline is wired but produces 0/2496 valid
//! FIBs on the K8B oracle.
//!
//! The OFDM chain (Stages 1–7, validated by `dab-ofdm`'s
//! `k8b_v4_ofdm_chain.rs`) produces healthy soft bits (mean |b| = 63/127,
//! balanced pos/neg), and `FicProtection` round-trips synthetic data
//! correctly. The chain breaks somewhere between the soft bits and the FIB
//! bytes. Most likely cause: `dab-viterbi`'s scalar decoder has only ever
//! been validated against its own `convolutional_encode` round-trip — never
//! against a real DAB stream. The eti-stuff oracle uses `viterbiSpiral` in
//! the FIC and EEP paths (the scalar `viterbiHandler::deconvolve` call sites
//! are commented out in `eep-protection.cpp` and `uep-protection.cpp`); the
//! spiral works in a bit-reversed polynomial representation, and it is
//! plausible that the scalar port's convention is not the inverse of the
//! actual DAB transmit encoder. Tracking this in the README as discovered
//! subtlety #7; resolution is the next slice and requires either porting
//! `viterbiSpiral` or instrumenting `eti-stuff` with `HAVE_DUMPING` to
//! cross-check Viterbi input/output bits frame-by-frame against this
//! pipeline.
//!
//! The 240 MB capture is Git-LFS / not committed here:
//!
//! ```sh
//! export DAB_RS_K8B_V4_IQ=/path/to/k8b_v4.iq
//! cargo test -p dab-cli --test k8b_v4_fic_iq -- --include-ignored --nocapture
//! ```

use std::path::PathBuf;

use dab_cli::fic_iq::process_iq_to_fic;
use dab_iq::IqFormat;

const DEFAULT_IQ: &str =
    "/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_v4.iq";

fn capture_path() -> PathBuf {
    std::env::var("DAB_RS_K8B_V4_IQ")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(DEFAULT_IQ))
}

#[test]
#[ignore = "requires the k8b_v4.iq oracle capture; run with --include-ignored"]
fn k8b_v4_fic_iq_pipeline_recovers_ensemble() {
    let path = capture_path();
    if !path.exists() {
        eprintln!("skipping: capture not found at {}", path.display());
        return;
    }

    let res = process_iq_to_fic(&path, IqFormat::Cs16Le, 3_000_000)
        .expect("FIC pipeline must run end-to-end without error");

    eprintln!(
        "resampled={} nulls={} frames_decoded={} frames_skipped={} best_band_ratio={:.1}dB",
        res.resampled_samples,
        res.null_dips,
        res.frames_decoded,
        res.frames_skipped,
        res.best_band_ratio_db,
    );
    eprintln!("fib_ok={}/{}", res.fib_ok, res.fib_total);
    if let Some(eid) = res.ensemble.eid {
        eprintln!("EId=0x{eid:04X} label={:?}", res.ensemble.label);
    } else {
        eprintln!("EId=(none) label={:?}", res.ensemble.label);
    }
    eprintln!("services={}", res.ensemble.services.len());
    eprintln!("sub_channels={}", res.ensemble.sub_channels.len());

    // ---- Coarse sanity ----
    assert!(
        res.resampled_samples > 40_000_000,
        "resampled stream too short: {}",
        res.resampled_samples
    );
    assert!(
        res.null_dips >= 180 && res.null_dips <= 230,
        "unexpected null count {}",
        res.null_dips
    );
    assert!(
        res.frames_decoded >= 150,
        "should decode >= 150 frames out of ~208, got {}",
        res.frames_decoded
    );

    // ---- Ensemble identity (oracle ground truth from k8b_v4.json) ----
    assert_eq!(res.ensemble.eid, Some(0xE040), "EId must match YTN DMB");

    let label = &res.ensemble.label;
    assert!(
        label.to_uppercase().contains("YTN"),
        "ensemble label should mention YTN, got {label:?}"
    );

    assert!(
        res.ensemble.services.len() >= 4,
        "expected >= 4 services in the YTN DMB ensemble (oracle 5), got {}",
        res.ensemble.services.len()
    );

    // ---- FIB CRC pass rate ----
    assert!(res.fib_total > 0, "no FIBs were ever fed to the accumulator");
    let pass_rate = res.fib_ok as f64 / res.fib_total as f64;
    assert!(
        pass_rate >= 0.40,
        "FIB CRC pass rate {pass_rate:.3} too low — chain likely broken \
         (oracle is ~0.75; threshold is set well below to absorb marginal-SNR \
         frame mis-syncs)"
    );

    eprintln!("OK: fib pass rate = {pass_rate:.3} (oracle ~0.75)");
}
