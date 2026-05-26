//! Golden integration test: reproduce the validated Python receiver's
//! outer-FEC result on the K8B reference capture, byte-for-byte.
//!
//! Reference (airspy-mini-dmb `tools/verify_outer_fec.py`, SubCh 1):
//!   aligned=true, phase_offset=160, rs_total=25953,
//!   rs_ok+corrected=22652 (87.3%), rs_failed=3301,
//!   RS bytes corrected=142822, top PIDs: 0x0113=16397, 0x0114=4916.
//! See `tests/golden/k8b_100pct_rs_result.json` for the pinned values.
//!
//! The 30 MB capture is NOT committed. Provide a local copy via the
//! `DAB_RS_K8B_ETI` env var; otherwise the test falls back to the known
//! local path. If the file is absent the test is skipped (it is `#[ignore]`d
//! so default `cargo test` never fails on a machine without the capture):
//!
//!   cargo test -p dab-fec --test golden -- --include-ignored

use std::collections::HashMap;
use std::path::PathBuf;

use dab_eti::FrameReader;
use dab_fec::KoreanTDmbOuterFec;
use dab_msc::extract_subchannel;

const DEFAULT_ETI: &str =
    "/Users/zobithecat/Documents/projects/etc_projects/airspy-mini-dmb/data/captures/k8b_100pct.eti";
const SUBCH: u8 = 1;

fn capture_path() -> PathBuf {
    std::env::var("DAB_RS_K8B_ETI")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(DEFAULT_ETI))
}

#[test]
#[ignore = "requires the K8B reference capture; run with --include-ignored"]
fn k8b_outer_fec_reproduces_python_reference() {
    let path = capture_path();
    let bytes = match std::fs::read(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("skipping golden test: cannot read {}: {e}", path.display());
            return;
        }
    };

    let mut fec = KoreanTDmbOuterFec::new();
    let mut packets = 0usize;
    let mut ok = 0usize;
    let mut bytes_corrected = 0i64;
    let mut pids: HashMap<u16, u32> = HashMap::new();

    for frame in FrameReader::new(&bytes) {
        let Ok(frame) = frame else { continue };
        if let Some(chunk) = extract_subchannel(&frame, SUBCH) {
            for ts in fec.feed(chunk) {
                packets += 1;
                if ts.rs_errors >= 0 {
                    ok += 1;
                    if ts.rs_errors > 0 {
                        bytes_corrected += ts.rs_errors as i64;
                    }
                    if ts.data.first() == Some(&0x47) {
                        let pid = (((ts.data[1] & 0x1f) as u16) << 8) | ts.data[2] as u16;
                        *pids.entry(pid).or_default() += 1;
                    }
                }
            }
        }
    }

    let stats = fec.stats();

    // --- Hard byte-identical assertions (see golden JSON) ---
    assert!(stats.aligned, "expected sync alignment to lock");
    assert_eq!(stats.phase_offset, 160, "0x47 phase offset");
    assert_eq!(stats.bytes_in, 5_296_896, "total MSC bytes fed");
    assert_eq!(stats.rs_total, 25_953, "total RS blocks");
    assert_eq!(ok, 22_652, "RS ok+corrected packets");
    assert_eq!(stats.rs_failed, 3_301, "RS uncorrectable blocks");
    assert_eq!(bytes_corrected, 142_822, "total RS bytes corrected");

    // 87.3% success rate
    let rate = ok as f64 * 100.0 / packets as f64;
    assert!((rate - 87.3).abs() < 0.05, "success rate {rate:.1}% != 87.3%");

    // Dominant PIDs after RS (video + audio of the mYTN service).
    assert_eq!(pids.get(&0x0113), Some(&16_397), "PID 0x0113 count");
    assert_eq!(pids.get(&0x0114), Some(&4_916), "PID 0x0114 count");
}
