//! Dump `convolutional_encode` output for a fixed test vector. Used by
//! slice-14 Path A to confirm Python's eti_to_expected_soft.py emits
//! identical mother-codeword bits to the Rust encoder dab-rs decodes against.

use dab_viterbi::convolutional_encode;

fn main() {
    // Two distinctive 768-bit inputs.
    let all_zero = vec![0u8; 768];
    let alternating: Vec<u8> = (0..768).map(|i| (i & 1) as u8).collect();

    for (label, msg) in [("all_zero", &all_zero), ("alternating", &alternating)] {
        let coded = convolutional_encode(msg);
        // Print first 32 coded bits, then last 32 — enough to detect any divergence.
        let first: String = coded[..32].iter().map(|b| char::from(b'0' + *b)).collect();
        let last: String = coded[coded.len() - 32..].iter()
            .map(|b| char::from(b'0' + *b))
            .collect();
        println!("{}: len={} first32={} last32={}", label, coded.len(), first, last);
    }
}
