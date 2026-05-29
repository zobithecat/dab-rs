//! Feed Python's `expected_soft.bin` (slice-14 Path A back-derivation)
//! through dab-rs's FicProtection + descrambler + CRC and check if the
//! resulting FIB bytes match the LIVE ETI ground truth.
//!
//! If the back-derive is correct AND dab-rs's encoder/decoder are
//! bit-equivalent, every valid ficBlock should produce 3 FIBs that all
//! pass CRC.

use std::env;
use std::fs::File;
use std::io::Read;
use std::process::ExitCode;

use dab_viterbi::{FicProtection, FIC_IN_BITS, FIC_OUT_BITS};

fn prbs_seq(n: usize) -> Vec<u8> {
    let mut sr = [1u8; 9];
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let b = sr[8] ^ sr[4];
        for k in (1..9).rev() {
            sr[k] = sr[k - 1];
        }
        sr[0] = b;
        out.push(b);
    }
    out
}

fn descramble_and_pack(info: &[u8]) -> Vec<u8> {
    let prbs = prbs_seq(info.len());
    let n_bytes = info.len() / 8;
    let mut out = Vec::with_capacity(n_bytes);
    for byte_i in 0..n_bytes {
        let mut v = 0u8;
        for bit_i in 0..8 {
            let idx = byte_i * 8 + bit_i;
            v = (v << 1) | ((info[idx] ^ prbs[idx]) & 1);
        }
        out.push(v);
    }
    out
}

const SOFT_BITS_PER_FICBLOCK: usize = 2304;
const FICBLOCKS_PER_FRAME: usize = 4;
const SOFT_BITS_PER_FRAME: usize = SOFT_BITS_PER_FICBLOCK * FICBLOCKS_PER_FRAME;
const RECORD_SIZE: usize = 4 + 4 + SOFT_BITS_PER_FRAME; // u32 idx + u8 mask + 3 pad + 9216 i8

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc ^ 0xFFFF
}

fn fib_ok(fib: &[u8]) -> bool {
    if fib.len() != 32 {
        return false;
    }
    let stored = u16::from_be_bytes([fib[30], fib[31]]);
    crc16_ccitt(&fib[..30]) == stored
}

fn main() -> ExitCode {
    let path = env::args()
        .nth(1)
        .unwrap_or_else(|| "/tmp/expected_soft.bin".to_string());
    let mut f = File::open(&path).expect("open expected_soft.bin");
    let mut buf = Vec::new();
    f.read_to_end(&mut buf).expect("read");
    let n_records = buf.len() / RECORD_SIZE;
    println!("records: {}", n_records);

    let mut fic = FicProtection::new();

    let mut total_blocks = 0;
    let mut blocks_all_3_ok = 0;
    let mut fib_passed = 0;
    let mut fib_total = 0;

    for rec_i in 0..n_records {
        let rec = &buf[rec_i * RECORD_SIZE..(rec_i + 1) * RECORD_SIZE];
        let frame_idx = u32::from_le_bytes([rec[0], rec[1], rec[2], rec[3]]);
        let mask = rec[4];
        let soft = &rec[8..8 + SOFT_BITS_PER_FRAME];

        for fb in 0..FICBLOCKS_PER_FRAME {
            if mask & (1 << fb) == 0 {
                continue;
            }
            total_blocks += 1;
            let off = fb * SOFT_BITS_PER_FICBLOCK;
            // Cast i8 -> i16 (sign-extending).
            let soft_i16: Vec<i16> = soft[off..off + SOFT_BITS_PER_FICBLOCK]
                .iter()
                .map(|&b| (b as i8) as i16)
                .collect();
            assert_eq!(soft_i16.len(), FIC_IN_BITS);

            let info_bits = fic.deconvolve(&soft_i16);
            assert_eq!(info_bits.len(), FIC_OUT_BITS);

            // Descramble + pack -> 96 bytes = 3 FIBs.
            let bytes = descramble_and_pack(&info_bits);
            assert_eq!(bytes.len(), 96);

            let mut block_ok = true;
            for fi in 0..3 {
                fib_total += 1;
                if fib_ok(&bytes[fi * 32..(fi + 1) * 32]) {
                    fib_passed += 1;
                } else {
                    block_ok = false;
                }
            }
            if block_ok {
                blocks_all_3_ok += 1;
            } else if rec_i < 3 {
                println!("  frame {} fb {}: at least one FIB CRC fail", frame_idx, fb);
            }
        }
    }

    println!("ficBlocks processed: {}", total_blocks);
    println!("ficBlocks where all 3 FIBs pass CRC: {}", blocks_all_3_ok);
    println!("FIB CRC pass: {}/{} ({:.1}%)", fib_passed, fib_total,
        100.0 * fib_passed as f64 / fib_total as f64);

    if blocks_all_3_ok == total_blocks && total_blocks > 0 {
        println!("✓ Back-derivation round-trips PERFECTLY through dab-rs FicProtection.");
        ExitCode::SUCCESS
    } else {
        println!("✗ Back-derivation does NOT round-trip — Python ≠ dab-rs convention somewhere.");
        ExitCode::FAILURE
    }
}
