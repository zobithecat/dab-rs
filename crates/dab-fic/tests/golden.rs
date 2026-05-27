//! Golden integration test: reproduce the validated Python receiver's FIC
//! ensemble decode on the K8B reference capture, byte-for-byte.
//!
//! Values verified against the Python reference (`tdmb/eti/fic.py`) on
//! `k8b_100pct.eti`: fib_total=20064, fib_ok=15042, EId=0xE040,
//! label="YTN DMB", four sub-channels, five services.
//!
//! The 30 MB capture is NOT committed. Provide a local copy via the
//! `DAB_RS_K8B_ETI` env var; otherwise the test falls back to the known local
//! path. If the file is absent the test is skipped (it is `#[ignore]`d so the
//! default `cargo test` never fails on a machine without the capture):
//!
//!   cargo test -p dab-fic --test golden -- --include-ignored

use std::path::PathBuf;

use dab_eti::FrameReader;
use dab_fic::FicAccumulator;

const DEFAULT_ETI: &str =
    "/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_100pct.eti";

fn capture_path() -> PathBuf {
    std::env::var("DAB_RS_K8B_ETI")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(DEFAULT_ETI))
}

#[test]
#[ignore = "requires the K8B reference capture; run with --include-ignored"]
fn k8b_fic_reproduces_python_reference() {
    let path = capture_path();
    let bytes = match std::fs::read(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skipping golden test: cannot read {}: {e}", path.display());
            return;
        }
    };

    let mut acc = FicAccumulator::new();
    for frame in FrameReader::new(&bytes) {
        let Ok(frame) = frame else { continue };
        if !frame.fic.is_empty() {
            acc.feed_fic(&frame.fic);
        }
    }

    // --- FIB counts ---
    assert_eq!(acc.fib_total, 20_064, "fib_total");
    assert_eq!(acc.fib_ok, 15_042, "fib_ok");

    let ens = &acc.ensemble;

    // --- Ensemble identity ---
    assert_eq!(ens.eid, Some(0xE040), "EId");
    assert_eq!(ens.label, "YTN DMB", "ensemble label");

    // --- Sub-channels: exactly {1,3,6,9} ---
    let keys: Vec<u8> = ens.sub_channels.keys().copied().collect();
    assert_eq!(keys, vec![1, 3, 6, 9], "sub-channel ids");

    let s1 = &ens.sub_channels[&1];
    assert_eq!(s1.start_addr, 0);
    assert_eq!(s1.size_cu, 264);
    assert_eq!(s1.protection, "EEP-3A");
    assert_eq!(s1.bitrate_kbps, 352);
    assert!(s1.is_long_form);

    let s3 = &ens.sub_channels[&3];
    assert_eq!(s3.start_addr, 750);
    assert_eq!(s3.size_cu, 114);
    assert_eq!(s3.protection, "EEP-3A");
    assert_eq!(s3.bitrate_kbps, 152);

    let s6 = &ens.sub_channels[&6];
    assert_eq!(s6.start_addr, 264);
    assert_eq!(s6.size_cu, 270);
    assert_eq!(s6.protection, "EEP-3B");
    assert_eq!(s6.bitrate_kbps, 480);

    let s9 = &ens.sub_channels[&9];
    assert_eq!(s9.start_addr, 534);
    assert_eq!(s9.size_cu, 216);
    assert_eq!(s9.protection, "EEP-3B");
    assert_eq!(s9.bitrate_kbps, 384);

    // --- Services: exactly the five expected SIds ---
    let svc_keys: Vec<u32> = ens.services.keys().copied().collect();
    assert_eq!(
        svc_keys,
        vec![0xF1E00400, 0xF1E00402, 0xF1E00404, 0xF1E00408, 0xF1E77404],
        "service ids"
    );

    // All services are data services.
    for svc in ens.services.values() {
        assert!(svc.is_data, "service 0x{:08X} should be data", svc.sid);
    }

    // 0xF1E00400 "mYTN": stream-data, sub 1, primary.
    let svc = &ens.services[&0xF1E00400];
    assert_eq!(svc.label, "mYTN");
    let c = svc
        .components
        .iter()
        .find(|c| c.transport == "stream-data")
        .expect("stream-data component");
    assert_eq!(c.sub_ch_id, Some(1));
    assert!(c.is_primary);

    // 0xF1E00402 "HD mYTN": stream-data sub 6.
    let svc = &ens.services[&0xF1E00402];
    assert_eq!(svc.label, "HD mYTN");
    let c = svc
        .components
        .iter()
        .find(|c| c.transport == "stream-data")
        .expect("stream-data component");
    assert_eq!(c.sub_ch_id, Some(6));

    // 0xF1E00404 "4DRIVE": packet, sub_ch_id None, sc_id_s=3.
    let svc = &ens.services[&0xF1E00404];
    assert_eq!(svc.label, "4DRIVE");
    let c = svc
        .components
        .iter()
        .find(|c| c.transport == "packet")
        .expect("packet component");
    assert_eq!(c.sub_ch_id, None);
    assert_eq!(c.sc_id_s, 3);

    // 0xF1E00408 "LOTTE Homeshop": stream-data sub 9.
    let svc = &ens.services[&0xF1E00408];
    assert_eq!(svc.label, "LOTTE Homeshop");
    let c = svc
        .components
        .iter()
        .find(|c| c.transport == "stream-data")
        .expect("stream-data component");
    assert_eq!(c.sub_ch_id, Some(9));

    // 0xF1E77404 "YTN EWS": fidc sub 58.
    let svc = &ens.services[&0xF1E77404];
    assert_eq!(svc.label, "YTN EWS");
    let c = svc
        .components
        .iter()
        .find(|c| c.transport == "fidc")
        .expect("fidc component");
    assert_eq!(c.sub_ch_id, Some(58));
}
