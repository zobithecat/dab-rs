//! `dab-fic` — FIC: FIB CRC-16, FIG 0/x and 1/x dispatch -> Ensemble.
//!
//! Operates on already-decoded FIC bytes (no Viterbi). A FIC block is 96 bytes
//! per ETI frame in Mode I (4 FIBs x 32 bytes). This is a faithful Rust port of
//! the validated Python reference and reproduces its ensemble byte-for-byte.
//!
//! See ETSI EN 300 401 §5.2 (FIB/FIG) and §6 (MCI / labels).
#![forbid(unsafe_code)]

mod crc;
mod ensemble;
mod fig;
mod tables;

pub use crc::{crc16_ccitt, fib_ok};
pub use ensemble::{
    decode_label, Ensemble, FicAccumulator, Service, ServiceComponent, SubChannel,
};
pub use fig::{iter_figs, Fig};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_vector() {
        // This FIB variant uses poly 0x1021, init 0xFFFF AND a final XOR of
        // 0xFFFF (= CRC-16/GENIBUS), so the standard CCITT/FALSE check value
        // 0x29B1 appears inverted: 0x29B1 ^ 0xFFFF == 0xD64E.
        assert_eq!(crc16_ccitt(b"123456789"), 0xD64E);
        assert_eq!(crc16_ccitt(b"123456789") ^ 0xFFFF, 0x29B1);
    }

    #[test]
    fn fib_ok_roundtrip() {
        let mut fib = vec![0u8; 32];
        // Build a payload, compute CRC over first 30 bytes, append big-endian.
        for (i, b) in fib.iter_mut().take(30).enumerate() {
            *b = (i as u8).wrapping_mul(7).wrapping_add(3);
        }
        let crc = crc16_ccitt(&fib[..30]);
        fib[30] = (crc >> 8) as u8;
        fib[31] = (crc & 0xFF) as u8;
        assert!(fib_ok(&fib));

        // Flip a payload byte: CRC must now fail.
        fib[0] ^= 0x01;
        assert!(!fib_ok(&fib));

        // Wrong length must fail.
        assert!(!fib_ok(&fib[..31]));
    }

    #[test]
    fn iter_figs_splits_and_stops() {
        // FIG #1: type 0, len 3, data [0x00, 0x11, 0x22]; header = (0<<5)|3 = 0x03.
        // FIG #2: type 1, len 2, data [0xAA, 0xBB]; header = (1<<5)|2 = 0x22.
        // Then 0xFF end marker; trailing garbage must be ignored.
        let mut payload = vec![0x03, 0x00, 0x11, 0x22, 0x22, 0xAA, 0xBB, 0xFF];
        payload.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);

        let figs = iter_figs(&payload);
        assert_eq!(figs.len(), 2);
        assert_eq!(figs[0].ftype, 0);
        assert_eq!(figs[0].data, vec![0x00, 0x11, 0x22]);
        assert_eq!(figs[1].ftype, 1);
        assert_eq!(figs[1].data, vec![0xAA, 0xBB]);
    }

    #[test]
    fn decode_label_trims_padding() {
        let mut buf = b"YTN DMB".to_vec();
        buf.extend_from_slice(&[b' ', b' ', 0x00, 0x00]);
        assert_eq!(decode_label(&buf), "YTN DMB");

        // Trailing NULs and spaces both stripped.
        assert_eq!(decode_label(b"abc\x00 \x00"), "abc");
    }
}
