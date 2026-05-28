//! `dab-viterbi` — rate-1/4 punctured convolutional (Viterbi) inner decoder.
//!
//! Faithful Rust port of the scalar Viterbi decoder and EEP depuncturing from
//! the `eti-stuff` reference implementation (the validation oracle):
//! `eti-cmdline/src/eti-handling/{viterbi-handler,eep-protection,protection,
//! protTables}.cpp`. Correctness against that oracle is the design goal.
//!
//! Code parameters (EN 300 401 clause 11): DAB mother code with constraint
//! length `K = 7`, rate 1/4, generator polynomials (octal) 0o133, 0o171,
//! 0o145, 0o133, giving `numStates = 1 << (K - 1) = 64`.
//!
//! Soft-bit convention (from the oracle): `i16`, where `-255 => bit 1` and
//! `+255 => bit 0`.
//!
//! Scope: scalar Viterbi (`viterbiHandler`), Equal Error Protection (EEP)
//! depuncturing, and FIC-specific depuncturing are ported. The SIMD
//! `viterbiSpiral` variant and UEP (Unequal Error Protection) are
//! intentionally deferred.
#![forbid(unsafe_code)]

mod eep;
mod fic;
mod tables;
mod viterbi;

pub use eep::EepProtection;
pub use fic::{FicProtection, FIC_IN_BITS, FIC_OUT_BITS, FIC_VITERBI_LEN};
pub use tables::{p_codes, P_CODES};
pub use viterbi::{bit_for, convolutional_encode, Viterbi, K, POLYS};

#[cfg(test)]
mod tests {
    use super::*;

    /// Tiny seeded xorshift32 RNG so the tests need no external crate.
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

    /// Map a coded bit (0/1) to a soft sample.
    ///
    /// The oracle decoder negates each input symbol (`sym = -sym`) and then
    /// `computeCostTable` adds `+sym_j` for an output-1 branch / `-sym_j` for an
    /// output-0 branch before taking the path minimum. Working that through:
    /// a branch whose coded output is `1` is favoured (lower cost) when the
    /// soft sample is positive. So to feed the decoder a clean channel where it
    /// reconstructs coded bit `c`, we map `1 => +255` and `0 => -255`.
    ///
    /// (The "−255 ⇒ bit 1, +255 ⇒ bit 0" wording in the oracle comment is the
    /// hardware demodulator's labelling; this encoder→decoder round-trip uses
    /// the mapping that the decoder's metric actually selects.)
    fn bit_to_soft(b: u8) -> i16 {
        if b == 0 {
            -255
        } else {
            255
        }
    }

    #[test]
    fn bit_for_hand_checked() {
        // bitFor(state, poly, bit): register = (bit==0?state:state+64) & poly,
        // then XOR-reduce the low K+1 = 8 bits.
        //
        // state = 0, bit = 0: register = 0 -> parity 0 for any poly.
        assert_eq!(bit_for(0, POLYS[0], 0), 0);
        assert_eq!(bit_for(0, POLYS[1], 0), 0);

        // state = 0, bit = 1, poly = 0o133 = 0b1011011.
        // register = (0 + 64) & 0o133 = 64 & 91 = 64 = 0b1000000.
        // XOR of its set bits = 1 (single bit set).
        assert_eq!(bit_for(0, 0o133, 1), 1);

        // poly = 0o171 = 0b1111001. register = 64 & 121 = 64 -> parity 1.
        assert_eq!(bit_for(0, 0o171, 1), 1);

        // state = 1, bit = 0, poly = 0o133: register = 1 & 0o133 = 1 -> parity 1.
        assert_eq!(bit_for(1, 0o133, 0), 1);

        // state = 3 (0b11), poly = 0o133 (0b1011011), bit = 0:
        // register = 3 & 0o133 = 0b11 = 2 set bits -> parity 0.
        assert_eq!(bit_for(3, 0o133, 0), 0);
    }

    #[test]
    fn p_codes_known_rows() {
        // Row 1 (idx 0) from protTables.cpp.
        assert_eq!(
            *p_codes(0),
            [
                1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0,
                0, 1, 0, 0, 0
            ]
        );
        // Row 24 (idx 23) is all ones.
        assert_eq!(*p_codes(23), [1i8; 32]);
        // Row 8 (idx 7) from protTables.cpp.
        assert_eq!(
            *p_codes(7),
            [
                1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0,
                0, 1, 1, 0, 0
            ]
        );
    }

    #[test]
    fn viterbi_roundtrip_clean() {
        let mut rng = XorShift32::new(0xACE1);
        let n = 200usize;
        let message: Vec<u8> = (0..n).map(|_| rng.next_bit()).collect();

        let coded = convolutional_encode(&message);
        assert_eq!(coded.len(), 4 * (n + 6));

        let soft: Vec<i16> = coded.iter().map(|&b| bit_to_soft(b)).collect();
        assert_eq!(soft.len(), 4 * n + 24);

        let mut v = Viterbi::new(n);
        let decoded = v.deconvolve(&soft);
        assert_eq!(decoded, message);
    }

    #[test]
    fn viterbi_error_tolerance() {
        let mut rng = XorShift32::new(0xBEEF);
        let n = 300usize;
        let message: Vec<u8> = (0..n).map(|_| rng.next_bit()).collect();

        let coded = convolutional_encode(&message);
        let mut soft: Vec<i16> = coded.iter().map(|&b| bit_to_soft(b)).collect();

        // Flip a few soft bits (sign). Rate 1/4, K=7 tolerates a handful of
        // isolated errors. Keep them sparse.
        for &idx in &[5usize, 53, 211, 480, 900] {
            soft[idx] = -soft[idx];
        }

        let mut v = Viterbi::new(n);
        let decoded = v.deconvolve(&soft);
        assert_eq!(decoded, message);
    }

    /// Puncture an encoded mother-code stream using an EEP index_table, decode,
    /// and assert exact recovery on a clean channel.
    fn eep_roundtrip_case(bit_rate: i16, prot_level: i16, seed: u32) {
        let mut eep = EepProtection::new(bit_rate, prot_level);
        let out_size = eep.out_size();

        let mut rng = XorShift32::new(seed);
        let message: Vec<u8> = (0..out_size).map(|_| rng.next_bit()).collect();

        // Mother-code encode: out_size bits -> 4*(out_size+6) = out_size*4+24 bits.
        let coded = convolutional_encode(&message);
        let table_len = out_size * 4 + 24;
        assert_eq!(coded.len(), table_len);
        assert_eq!(eep.index_table().len(), table_len);

        // Puncture: keep only positions the index_table marks as transmitted,
        // mapping each kept bit to soft.
        let punctured_soft: Vec<i16> = coded
            .iter()
            .zip(eep.index_table().iter())
            .filter_map(|(&b, &keep)| if keep { Some(bit_to_soft(b)) } else { None })
            .collect();

        let decoded = eep.deconvolve(&punctured_soft);
        assert_eq!(decoded.len(), out_size);
        assert_eq!(decoded, message, "EEP roundtrip failed for bit_rate={bit_rate} prot_level={prot_level}");
    }

    #[test]
    fn eep_roundtrip_profiles() {
        // EEP-1A at the smallest rate (bit_rate=8 => 32 kbps), prot_level=0.
        // Exercises the special bitRate==8 path indirectly (that path is
        // level 2; level 1 here still validates the small-rate fill loop).
        eep_roundtrip_case(8, 0, 0x0001);
        // EEP-2A special bitRate==8 case (prot_level & 0o3 == 1, A series).
        eep_roundtrip_case(8, 1, 0x0002);
        // EEP-3A: A series, level 3 (prot_level = 2).
        eep_roundtrip_case(12, 2, 0x0003);
        // EEP-3B: B series, level 3 (prot_level = (1<<2) | 2 = 6).
        eep_roundtrip_case(12, 6, 0x0004);
        // EEP-1B: B series, level 1 (prot_level = (1<<2) | 0 = 4).
        eep_roundtrip_case(16, 4, 0x0005);
    }
}
