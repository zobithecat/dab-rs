//! CRC-16-CCITT used by FIBs and FIC.
//!
//! Polynomial 0x1021, initial value 0xFFFF, output XOR 0xFFFF
//! (ETSI EN 300 401 §5.2.1, FIB CRC). Faithful port of the Python reference
//! `tdmb/eti/crc.py`.

/// CRC-16-CCITT (poly 0x1021, init 0xFFFF, final XOR 0xFFFF).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc ^ 0xFFFF
}

/// A FIB is 32 bytes: 30 payload + 2-byte big-endian CRC.
///
/// Returns true iff the length is 32 and the CRC over bytes `[0..30]` equals
/// the big-endian 16-bit value in `fib[30..32]`.
pub fn fib_ok(fib: &[u8]) -> bool {
    if fib.len() != 32 {
        return false;
    }
    let expected = ((fib[30] as u16) << 8) | fib[31] as u16;
    crc16_ccitt(&fib[..30]) == expected
}
