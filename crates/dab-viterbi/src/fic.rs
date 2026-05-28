//! FIC-specific depuncturing + Viterbi decode.
//!
//! Faithful port of the eti-stuff oracle
//! (`eti-cmdline/src/eti-handling/fic-handler.cpp` lines 81-112). DAB's
//! Fast Information Channel uses a fixed protection profile, not the EEP
//! tables — see ETSI EN 300 401 §11.2:
//!
//! ```text
//! mother-code length  = 3072 + 24 = 3096 bits (rate-1/4, 768 info bits +
//!                                              6 tail bits × 4 = 24)
//! transmitted block   = 2304 soft bits per ficBlock
//! decoded output      = 768 information bits per ficBlock
//! ```
//!
//! The 3072-bit motherword is split into 24 sub-blocks of 128 bits:
//! - sub-blocks  0..20 (21 blocks) are punctured per `PI_16` = `p_codes(15)`;
//! - sub-blocks 21..23 ( 3 blocks) are punctured per `PI_15` = `p_codes(14)`;
//! - the trailing 24 tail bits are punctured per `PI_X` (first 24 entries of
//!   `p_codes(7)`).
//!
//! For Mode I, each frame's 3 FIC OFDM symbols deliver 3 × 3072 = 9216 soft
//! bits, which split into 4 ficBlocks of 2304 soft bits → 4 × 768 = 3072
//! decoded information bits per frame (12 FIBs of 256 bits each, after
//! energy descrambling).

use crate::tables::p_codes;
use crate::viterbi::Viterbi;

/// Decoded information bits emitted per ficBlock (DAB-standard fixed value).
pub const FIC_OUT_BITS: usize = 768;
/// Transmitted soft bits expected per ficBlock (sum of `index_table`).
pub const FIC_IN_BITS: usize = 2304;
/// Length of the rate-1/4 motherword including the 24 tail bits.
pub const FIC_VITERBI_LEN: usize = 3072 + 24;

/// FIC depuncturing + Viterbi decoder front-end.
///
/// Mirrors `ficHandler` in `fic-handler.cpp`: holds a 3096-entry boolean
/// `index_table` and routes the 2304 received soft bits into a 3096-position
/// mother-code buffer with zeros at punctured positions, then deconvolves
/// to 768 information bits.
pub struct FicProtection {
    /// `index_table[i] == true` iff mother-code bit `i` was transmitted.
    /// Length is [`FIC_VITERBI_LEN`].
    index_table: Vec<bool>,
    viterbi: Viterbi,
}

impl FicProtection {
    /// Build the FIC depuncturer with the fixed standard puncturing table.
    pub fn new() -> Self {
        let pi_16 = p_codes(16 - 1);
        let pi_15 = p_codes(15 - 1);
        let pi_x = p_codes(8 - 1);

        let mut index_table = vec![false; FIC_VITERBI_LEN];
        let mut local = 0usize;

        // First 21 sub-blocks: 128 bits each, depunctured per PI_16.
        // Each 128-bit block is divided into 4 × 32-bit chunks; the same
        // 32-bit pattern is applied to each chunk.
        for _block in 0..21 {
            for k in 0..(32 * 4) {
                if pi_16[k % 32] != 0 {
                    index_table[local] = true;
                }
                local += 1;
            }
        }

        // Next 3 sub-blocks: 128 bits each, depunctured per PI_15.
        for _block in 0..3 {
            for k in 0..(32 * 4) {
                if pi_15[k % 32] != 0 {
                    index_table[local] = true;
                }
                local += 1;
            }
        }

        // Trailing 24 tail bits: depunctured per PI_X (the first 24 entries of
        // `p_codes(7)`).
        for k in 0..24 {
            if pi_x[k] != 0 {
                index_table[local] = true;
            }
            local += 1;
        }

        debug_assert_eq!(local, FIC_VITERBI_LEN);

        FicProtection {
            index_table,
            viterbi: Viterbi::new(FIC_OUT_BITS),
        }
    }

    /// Number of decoded output bits per ficBlock (= [`FIC_OUT_BITS`]).
    pub fn out_size(&self) -> usize {
        FIC_OUT_BITS
    }

    /// The depuncturing `index_table` (`true` = mother-code bit transmitted).
    /// Length is [`FIC_VITERBI_LEN`]. Exposed for tests and pipeline glue.
    pub fn index_table(&self) -> &[bool] {
        &self.index_table
    }

    /// Number of soft bits this depuncturer expects per call to
    /// [`deconvolve`](Self::deconvolve). Equals the count of `true` entries in
    /// [`index_table`](Self::index_table); for DAB this is [`FIC_IN_BITS`].
    pub fn in_size(&self) -> usize {
        self.index_table.iter().filter(|&&b| b).count()
    }

    /// Depuncture `soft` (the 2304 received punctured soft bits, in
    /// transmission order) by scattering them into a zeroed mother-code block
    /// per `index_table`, then Viterbi-decode into 768 information bits.
    ///
    /// Verbatim port of `ficHandler::process_ficInput`'s depuncture step
    /// (plus the final `deconvolve` call). Note that this function does *not*
    /// apply the FIC PRBS energy descrambling — feed the returned bit vector
    /// to `dab-descramble::descramble_and_pack` to obtain the 96-byte (3 FIBs)
    /// output the FIC accumulator expects.
    ///
    /// # Panics
    /// Panics if `soft.len() != FIC_IN_BITS`.
    pub fn deconvolve(&mut self, soft: &[i16]) -> Vec<u8> {
        assert_eq!(
            soft.len(),
            FIC_IN_BITS,
            "FIC deconvolve expects {} soft bits, got {}",
            FIC_IN_BITS,
            soft.len()
        );

        let mut viterbi_block = vec![0i16; FIC_VITERBI_LEN];
        let mut input_counter = 0usize;
        for i in 0..FIC_VITERBI_LEN {
            if self.index_table[i] {
                viterbi_block[i] = soft[input_counter];
                input_counter += 1;
            }
        }
        debug_assert_eq!(input_counter, FIC_IN_BITS);

        self.viterbi.deconvolve(&viterbi_block)
    }
}

impl Default for FicProtection {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viterbi::convolutional_encode;

    /// Map a coded bit (0/1) to a soft sample, using the same `1 → +255,
    /// 0 → −255` mapping that exercises the decoder's metric branches.
    fn bit_to_soft(b: u8) -> i16 {
        if b == 0 {
            -255
        } else {
            255
        }
    }

    /// Tiny seeded xorshift32 RNG so tests need no external crate.
    struct XorShift32(u32);
    impl XorShift32 {
        fn new(seed: u32) -> Self {
            XorShift32(if seed == 0 { 0x1234_5678 } else { seed })
        }
        fn next_u32(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        fn next_bit(&mut self) -> u8 {
            (self.next_u32() & 1) as u8
        }
    }

    #[test]
    fn puncture_table_geometry_matches_standard() {
        let f = FicProtection::new();
        let table = f.index_table();
        assert_eq!(table.len(), FIC_VITERBI_LEN);
        assert_eq!(table.len(), 3096);

        let kept = table.iter().filter(|&&b| b).count();
        assert_eq!(
            kept, FIC_IN_BITS,
            "expected {} transmitted bits per ficBlock, got {}",
            FIC_IN_BITS, kept
        );
        assert_eq!(f.in_size(), FIC_IN_BITS);
        assert_eq!(f.out_size(), FIC_OUT_BITS);
    }

    #[test]
    fn fic_roundtrip_clean_channel() {
        // Encode 768 random info bits at rate 1/4 → 3096 mother-code bits.
        // Apply the FIC puncturing pattern → 2304 transmitted bits. Soft-map,
        // decode, and require exact recovery.
        let f = FicProtection::new();
        let mut rng = XorShift32::new(0xF1C0_0001);
        let message: Vec<u8> = (0..FIC_OUT_BITS).map(|_| rng.next_bit()).collect();

        let coded = convolutional_encode(&message);
        assert_eq!(coded.len(), FIC_VITERBI_LEN);

        // Puncture: keep only bits flagged in the index_table.
        let mut tx_soft: Vec<i16> = Vec::with_capacity(FIC_IN_BITS);
        for (&bit, &keep) in coded.iter().zip(f.index_table().iter()) {
            if keep {
                tx_soft.push(bit_to_soft(bit));
            }
        }
        assert_eq!(tx_soft.len(), FIC_IN_BITS);

        let mut f2 = FicProtection::new();
        let decoded = f2.deconvolve(&tx_soft);
        assert_eq!(decoded.len(), FIC_OUT_BITS);
        assert_eq!(decoded, message, "FIC clean round-trip failed");
    }

    #[test]
    fn fic_roundtrip_tolerates_isolated_errors() {
        // A handful of flipped soft bits across a 2304-bit transmission still
        // decode cleanly through the rate-1/4 K=7 Viterbi.
        let mut f = FicProtection::new();
        let mut rng = XorShift32::new(0xF1C0_BEEF);
        let message: Vec<u8> = (0..FIC_OUT_BITS).map(|_| rng.next_bit()).collect();
        let coded = convolutional_encode(&message);

        let mut tx_soft: Vec<i16> = coded
            .iter()
            .zip(f.index_table().iter())
            .filter_map(|(&b, &keep)| if keep { Some(bit_to_soft(b)) } else { None })
            .collect();
        assert_eq!(tx_soft.len(), FIC_IN_BITS);

        // Flip 6 widely-spaced soft bits.
        for &idx in &[17usize, 233, 559, 1024, 1611, 2103] {
            tx_soft[idx] = -tx_soft[idx];
        }

        let decoded = f.deconvolve(&tx_soft);
        assert_eq!(decoded, message);
    }

    #[test]
    #[should_panic(expected = "FIC deconvolve expects")]
    fn wrong_input_size_panics() {
        let mut f = FicProtection::new();
        let bad = vec![0i16; FIC_IN_BITS - 1];
        let _ = f.deconvolve(&bad);
    }
}
