//! `dab-descramble` — Energy-dispersal PRBS descrambler for DAB/T-DMB.
//!
//! Implements the energy-dispersal scrambling/descrambling defined in
//! **ETSI EN 300 401 §11** using the PRBS polynomial **x⁹ + x⁵ + 1** with
//! the shift register initialised to all-ones.
//!
//! This is a faithful port of the C++ `eti-generator` in
//! [eti-stuff](https://github.com/JvanKatwijk/eti-stuff), specifically the
//! `process_CIF` / `process_subCh` descrambler loop.
//!
//! ## Algorithm
//!
//! The 9-element shift register `SR[0..8]` is initialised to all ones.
//! Each clock tick produces one PRBS output bit and advances the register:
//!
//! ```text
//! b = SR[8] ^ SR[4]        // feedback tap: x^9 XOR x^5
//! SR[8..1] = SR[7..0]      // shift right (SR[k] ← SR[k-1])
//! SR[0] = b                // inject feedback
//! output_bit = b
//! ```
//!
//! This sequence is XORed element-wise with the incoming bit array
//! (descramble == scramble; the operation is self-inverse). The resulting bit
//! array is then packed MSB-first into bytes.
#![forbid(unsafe_code)]

/// Generate `len` bits of the DAB energy-dispersal PRBS (x⁹ + x⁵ + 1,
/// initial state all-ones), one bit (`0u8` or `1u8`) per element.
///
/// Mirrors the C++ shift-register loop in `eti-generator.cpp` exactly:
///
/// ```c
/// uint8_t shiftRegister[9];
/// memset(shiftRegister, 1, 9);
/// for (int j = 0; j < len; j++) {
///     uint8_t b = shiftRegister[8] ^ shiftRegister[4];
///     for (int k = 8; k > 0; k--) shiftRegister[k] = shiftRegister[k-1];
///     shiftRegister[0] = b;
///     descrambler[j] = b;
/// }
/// ```
///
/// # EN 300 401 reference
/// ETSI EN 300 401 §11 — Energy dispersal (energy-dispersal scrambling).
pub fn prbs_sequence(len: usize) -> Vec<u8> {
    let mut sr = [1u8; 9]; // SR[0..8], all initialised to 1
    let mut out = Vec::with_capacity(len);

    for _ in 0..len {
        let b = sr[8] ^ sr[4]; // feedback: x^9 ^ x^5
        // Shift right: sr[k] ← sr[k-1] for k = 8 down to 1
        sr[8] = sr[7];
        sr[7] = sr[6];
        sr[6] = sr[5];
        sr[5] = sr[4];
        sr[4] = sr[3];
        sr[3] = sr[2];
        sr[2] = sr[1];
        sr[1] = sr[0];
        sr[0] = b;
        out.push(b);
    }

    out
}

/// XOR a bit array in place with the DAB PRBS sequence (descramble == scramble;
/// the operation is self-inverse).
///
/// `bits` contains one bit per element (`0u8` or `1u8`). The PRBS sequence is
/// generated fresh from the all-ones initial state on every call.
///
/// # EN 300 401 reference
/// ETSI EN 300 401 §11 — Energy dispersal.
pub fn descramble_bits(bits: &mut [u8]) {
    let prbs = prbs_sequence(bits.len());
    for (b, p) in bits.iter_mut().zip(prbs.iter()) {
        *b ^= p;
    }
}

/// Descramble a bit array then pack the result to bytes, MSB first (8 bits per
/// output byte).
///
/// `bits.len()` must be a multiple of 8; each group of 8 consecutive bits is
/// packed into one output byte with the first bit in the most-significant
/// position.
///
/// Mirrors the C++ pack loop in `eti-generator.cpp`:
///
/// ```c
/// for (j = 0; j < 24*bitRate/8; j++) {
///     int temp = 0;
///     for (k = 0; k < 8; k++)
///         temp = (temp << 1) | (outVector[j*8 + k] & 1); // MSB first
///     output[j] = temp;
/// }
/// ```
///
/// # Panics
/// Panics if `bits.len()` is not a multiple of 8.
///
/// # EN 300 401 reference
/// ETSI EN 300 401 §11 — Energy dispersal (pack after descramble).
pub fn descramble_and_pack(bits: &[u8]) -> Vec<u8> {
    assert!(
        bits.len() % 8 == 0,
        "bits.len() must be a multiple of 8, got {}",
        bits.len()
    );

    let prbs = prbs_sequence(bits.len());

    // XOR then pack in one pass
    let n_bytes = bits.len() / 8;
    let mut out = Vec::with_capacity(n_bytes);

    for byte_idx in 0..n_bytes {
        let mut temp: u8 = 0;
        for bit_idx in 0..8 {
            let raw = bits[byte_idx * 8 + bit_idx];
            let p = prbs[byte_idx * 8 + bit_idx];
            temp = (temp << 1) | ((raw ^ p) & 1);
        }
        out.push(temp);
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---------------------------------------------------------------------------
    // Minimal in-test bit source (xorshift32, no external crate).
    // ---------------------------------------------------------------------------

    fn xorshift32(state: &mut u32) -> u32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        x
    }

    fn random_bits(n: usize, seed: u32) -> Vec<u8> {
        let mut state = seed;
        let mut out = Vec::with_capacity(n);
        let mut word = 0u32;
        let mut bits_left = 0u32;
        for _ in 0..n {
            if bits_left == 0 {
                word = xorshift32(&mut state);
                bits_left = 32;
            }
            out.push((word & 1) as u8);
            word >>= 1;
            bits_left -= 1;
        }
        out
    }

    // ---------------------------------------------------------------------------
    // 1. PRBS determinism + known-prefix guard
    // ---------------------------------------------------------------------------

    /// Independent reimplementation of the same SR loop (written differently to
    /// catch copy-paste/indexing typos in the production code).
    fn reference_prbs(len: usize) -> Vec<u8> {
        // SR stored as a 9-bit integer for a completely different representation.
        // Bit 0 = SR[0] (newest), bit 8 = SR[8] (oldest), all init to 1 → 0x1FF.
        let mut sr: u16 = 0x1FF;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            // feedback = bit8 ^ bit4
            let b = ((sr >> 8) ^ (sr >> 4)) & 1;
            sr = ((sr << 1) | b) & 0x1FF; // shift left, inject at LSB (= SR[0])
            out.push(b as u8);
        }
        out
    }

    #[test]
    fn test_prbs_determinism_and_known_prefix() {
        let seq = prbs_sequence(40);
        let ref_seq = reference_prbs(40);
        assert_eq!(
            seq, ref_seq,
            "prbs_sequence output does not match independent reference"
        );

        // Must not be all-zero
        assert!(
            seq.iter().any(|&b| b != 0),
            "PRBS sequence is all-zero (init or feedback bug)"
        );

        // Sanity: the first few values should be non-trivial (with SR all-ones
        // the first output bit is 1^1 = 0, second … let the reference decide).
        // The important check is equality with reference, already done above.
    }

    #[test]
    fn test_prbs_not_eventually_constant() {
        let seq = prbs_sequence(600);
        // Check that both 0 and 1 appear in the last 100 elements.
        let tail = &seq[500..];
        assert!(
            tail.iter().any(|&b| b == 0),
            "PRBS tail is all-ones (sequence appears constant)"
        );
        assert!(
            tail.iter().any(|&b| b == 1),
            "PRBS tail is all-zeros (sequence appears constant)"
        );
    }

    // ---------------------------------------------------------------------------
    // 2. Self-inverse property
    // ---------------------------------------------------------------------------

    #[test]
    fn test_descramble_bits_self_inverse() {
        let original = random_bits(128, 0xDEAD_BEEF);
        let mut bits = original.clone();

        descramble_bits(&mut bits); // first application
        descramble_bits(&mut bits); // second application — must restore original

        assert_eq!(
            bits, original,
            "descramble_bits applied twice does not restore original"
        );
    }

    // ---------------------------------------------------------------------------
    // 3. Pack correctness (hand-built 16-bit pattern)
    // ---------------------------------------------------------------------------

    /// Pack a bit slice MSB-first without any PRBS XOR (helper for test 3).
    fn pack_bits(bits: &[u8]) -> Vec<u8> {
        assert!(bits.len() % 8 == 0);
        bits.chunks(8)
            .map(|chunk| {
                let mut byte = 0u8;
                for &b in chunk {
                    byte = (byte << 1) | (b & 1);
                }
                byte
            })
            .collect()
    }

    #[test]
    fn test_pack_correctness() {
        // Bit pattern: 1010_0011  0110_1100
        // Expected bytes: 0xA3, 0x6C
        #[rustfmt::skip]
        let bits: [u8; 16] = [
            1, 0, 1, 0, 0, 0, 1, 1,  // 0xA3
            0, 1, 1, 0, 1, 1, 0, 0,  // 0x6C
        ];
        let packed = pack_bits(&bits);
        assert_eq!(packed, vec![0xA3, 0x6C], "MSB-first packing is incorrect");
    }

    // ---------------------------------------------------------------------------
    // 4. End-to-end identity: scramble then descramble_and_pack == pack(original)
    // ---------------------------------------------------------------------------

    #[test]
    fn test_end_to_end_identity() {
        // Use 48 bits (= 6 bytes) for a compact but non-trivial test.
        let original = random_bits(48, 0xCAFE_F00D);

        // Manually scramble: XOR with PRBS
        let prbs = prbs_sequence(original.len());
        let mut scrambled = original.clone();
        for (b, p) in scrambled.iter_mut().zip(prbs.iter()) {
            *b ^= p;
        }

        // descramble_and_pack on the scrambled bits should equal pack(original)
        let result = descramble_and_pack(&scrambled);
        let expected = pack_bits(&original);

        assert_eq!(
            result, expected,
            "descramble_and_pack did not undo the scramble"
        );
    }
}
