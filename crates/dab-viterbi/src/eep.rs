//! Equal Error Protection (EEP) depuncturing + Viterbi decode.
//!
//! Faithful port of the eti-stuff oracle
//! (`eti-cmdline/src/eti-handling/eep-protection.cpp` and its base
//! `protection.cpp`). EN 300 401 clause 11.3.2 defines the EEP puncturing
//! profiles; the (L1, PI1, L2, PI2) selection here reproduces the oracle's
//! `eep_protection::eep_protection` constructor exactly, including the special
//! `bitRate == 8`, A-level-2 case.
//!
//! UEP (Unequal Error Protection) and FIC-specific puncturing are intentionally
//! deferred — only EEP is ported here.

use crate::tables::p_codes;
use crate::viterbi::Viterbi;

/// EEP depuncturing + scatter front-end to the [`Viterbi`] decoder.
///
/// Mirrors `eep_protection` (deriving from `protection`): `out_size = 24 *
/// bit_rate` decoded bits; the depuncturing `index_table` has length
/// `out_size * 4 + 24`.
pub struct EepProtection {
    out_size: usize,
    /// `indexTable` (oracle): `true` where the mother-code bit is transmitted.
    index_table: Vec<bool>,
    viterbi: Viterbi,
}

impl EepProtection {
    /// Builds an EEP depuncturer for the given `bit_rate` (eti-stuff sense, so
    /// `out_size = 24 * bit_rate`) and `prot_level`.
    ///
    /// Profile selection (verbatim from `eep-protection.cpp`):
    /// - bit `(1 << 2)` of `prot_level` selects A (clear) vs B (set) series;
    /// - `prot_level & 0o3` selects level 1..4.
    pub fn new(bit_rate: i16, prot_level: i16) -> Self {
        let out_size = 24 * bit_rate as usize;
        let table_len = out_size * 4 + 24;

        let l1: i32;
        let l2: i32;
        let pi1: &'static [i8; 32];
        let pi2: &'static [i8; 32];

        let br = bit_rate as i32;

        if (prot_level & (1 << 2)) == 0 {
            // A profiles.
            match prot_level & 0o3 {
                0 => {
                    // level 1
                    l1 = 6 * br / 8 - 3;
                    l2 = 3;
                    pi1 = p_codes(24 - 1);
                    pi2 = p_codes(23 - 1);
                }
                1 => {
                    // level 2
                    if bit_rate == 8 {
                        l1 = 5;
                        l2 = 1;
                        pi1 = p_codes(13 - 1);
                        pi2 = p_codes(12 - 1);
                    } else {
                        l1 = 2 * br / 8 - 3;
                        l2 = 4 * br / 8 + 3;
                        pi1 = p_codes(14 - 1);
                        pi2 = p_codes(13 - 1);
                    }
                }
                2 => {
                    // level 3
                    l1 = 6 * br / 8 - 3;
                    l2 = 3;
                    pi1 = p_codes(8 - 1);
                    pi2 = p_codes(7 - 1);
                }
                _ => {
                    // level 4 (case 3)
                    l1 = 4 * br / 8 - 3;
                    l2 = 2 * br / 8 + 3;
                    pi1 = p_codes(3 - 1);
                    pi2 = p_codes(2 - 1);
                }
            }
        } else {
            // B series.
            match prot_level & 0o3 {
                3 => {
                    // level 4
                    l1 = 24 * br / 32 - 3;
                    l2 = 3;
                    pi1 = p_codes(2 - 1);
                    pi2 = p_codes(1 - 1);
                }
                2 => {
                    // level 3
                    l1 = 24 * br / 32 - 3;
                    l2 = 3;
                    pi1 = p_codes(4 - 1);
                    pi2 = p_codes(3 - 1);
                }
                1 => {
                    // level 2
                    l1 = 24 * br / 32 - 3;
                    l2 = 3;
                    pi1 = p_codes(6 - 1);
                    pi2 = p_codes(5 - 1);
                }
                _ => {
                    // level 1 (case 0)
                    l1 = 24 * br / 32 - 3;
                    l2 = 3;
                    pi1 = p_codes(10 - 1);
                    pi2 = p_codes(9 - 1);
                }
            }
        }

        let pi_x = p_codes(8 - 1);

        let mut index_table = vec![false; table_len];
        let mut ctr: usize = 0;

        // (L1, PI1) then (L2, PI2): each "row" is 128 mother-code positions,
        // PI indexed modulo 32.
        for _ in 0..l1 {
            for j in 0..128 {
                if pi1[j % 32] != 0 {
                    index_table[ctr] = true;
                }
                ctr += 1;
            }
        }
        for _ in 0..l2 {
            for j in 0..128 {
                if pi2[j % 32] != 0 {
                    index_table[ctr] = true;
                }
                ctr += 1;
            }
        }
        // Final 24-bit block (the 6 * 4 register tail) punctured by PI_X.
        for i in 0..24 {
            if pi_x[i] != 0 {
                index_table[ctr] = true;
            }
            ctr += 1;
        }

        EepProtection {
            out_size,
            index_table,
            viterbi: Viterbi::new(out_size),
        }
    }

    /// Number of decoded output bits (`out_size = 24 * bit_rate`).
    pub fn out_size(&self) -> usize {
        self.out_size
    }

    /// The depuncturing `index_table` (`true` = mother-code bit transmitted).
    /// Length is `out_size * 4 + 24`. Exposed for tests that need to puncture
    /// an encoded stream with the identical pattern.
    pub fn index_table(&self) -> &[bool] {
        &self.index_table
    }

    /// Depunctures `soft` (the received punctured soft bits, in transmission
    /// order) by scattering them into a zeroed mother-code block per
    /// `index_table`, then Viterbi-decodes into `out_size` bits.
    ///
    /// Verbatim port of `eep_protection::deconvolve`.
    pub fn deconvolve(&mut self, soft: &[i16]) -> Vec<u8> {
        let len = self.out_size * 4 + 24;
        let mut viterbi_block = vec![0i16; len];
        let mut input_counter = 0usize;
        for i in 0..len {
            if self.index_table[i] {
                viterbi_block[i] = soft[input_counter];
                input_counter += 1;
            }
        }
        self.viterbi.deconvolve(&viterbi_block)
    }
}
