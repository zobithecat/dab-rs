//! Mode I frequency (de-)interleaving permutation.
//!
//! ETSI EN 300 401 §14.6 (frequency interleaving). Ported from `createMapper`
//! in the `eti-stuff` oracle (`src/ofdm/freq-interleaver.cpp`).
//!
//! A pseudo-random sequence `tmp[i] = (13 * tmp[i-1] + V1) mod T_u` (a full-period
//! LCG for the Mode I parameters) is filtered to the active carrier window and
//! re-centred around DC, yielding a bijection from bin order index `n` (0..1535)
//! onto the carriers `{-768..=-1} ∪ {1..=768}`.

use crate::params::DabParams;

/// Frequency (de-)interleaver permutation table for DAB Mode I.
pub struct FreqInterleaver {
    perm: Vec<i16>,
}

impl FreqInterleaver {
    /// Build the Mode I permutation table.
    ///
    /// Equivalent to the oracle's `createMapper(T_u=2048, V1=511, lwb=256,
    /// upb=256+1536=1792)`.
    pub fn mode_i() -> Self {
        let p = DabParams::mode_i();
        let t_u = p.t_u as i32;
        let v1: i32 = 511;
        let lwb: i32 = 256;
        let upb: i32 = 256 + p.carriers as i32; // 1792
        let half = t_u / 2; // 1024

        // Generate the LCG sequence.
        let mut tmp = vec![0_i32; t_u as usize];
        for i in 1..t_u as usize {
            tmp[i] = (13 * tmp[i - 1] + v1) % t_u;
        }

        // Filter to the active window and re-centre around DC.
        let mut perm = Vec::with_capacity(p.carriers as usize);
        for &val in &tmp {
            if val == half {
                continue;
            }
            if val < lwb || val > upb {
                continue;
            }
            perm.push((val - half) as i16);
        }

        FreqInterleaver { perm }
    }

    /// Carrier index for bin order index `n`. Mirrors `interLeaver::mapIn`.
    ///
    /// # Panics
    /// Panics if `n` is out of range (`>= 1536`).
    pub fn map_in(&self, n: usize) -> i16 {
        self.perm[n]
    }

    /// The full permutation table (length 1536 for Mode I).
    pub fn table(&self) -> &[i16] {
        &self.perm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn permutation_is_bijection_onto_carrier_set() {
        let il = FreqInterleaver::mode_i();
        let table = il.table();

        // Table length equals K = 1536.
        assert_eq!(table.len(), 1536);

        // No zero, all within [-768, 768].
        for &c in table {
            assert_ne!(c, 0, "carrier 0 must be excluded");
            assert!((-768..=768).contains(&c), "carrier {c} out of range");
        }

        // Sorted values must be exactly {-768..=-1} ∪ {1..=768}: proves a
        // duplicate-free bijection (and full-period LCG mod 2048).
        let mut sorted: Vec<i16> = table.to_vec();
        sorted.sort_unstable();

        let mut expected: Vec<i16> = Vec::with_capacity(1536);
        expected.extend(-768..=-1);
        expected.extend(1..=768);

        assert_eq!(sorted, expected);

        // map_in agrees with the raw table.
        assert_eq!(il.map_in(0), table[0]);
        assert_eq!(il.map_in(1535), table[1535]);
    }
}
