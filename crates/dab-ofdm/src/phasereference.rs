//! Phase-Reference Symbol (PRS) generation.
//!
//! ETSI EN 300 401 §14.3.2. Ported from the `phaseReference` constructor in
//! the `eti-stuff` oracle (`src/ofdm/phasereference.cpp`).
//!
//! The PRS is the frequency-domain reference symbol used by the sync chain for
//! correlation-based frame detection. Each active carrier carries a unit-modulus
//! phasor `exp(j*Phi(k))`; the DC bin and the unused carriers stay zero.

use num_complex::Complex;

use crate::params::DabParams;
use crate::phasetable::get_phi;

/// Build the frequency-domain Phase-Reference Symbol for DAB Mode I.
///
/// Returns a `T_u`-length (2048) vector. Bin layout matches the FFT natural
/// order used by the oracle: positive carriers `1..=768` map to bins `1..=768`,
/// negative carriers `-1..=-768` wrap to bins `T_u-1 ..= T_u-768`. Bin 0 (DC)
/// is the null carrier and stays zero.
pub fn phase_reference() -> Vec<Complex<f32>> {
    let p = DabParams::mode_i();
    let t_u = p.t_u as usize;
    let half = (p.carriers / 2) as i32; // 768

    let mut ref_table = vec![Complex::new(0.0_f32, 0.0_f32); t_u];

    for i in 1..=half {
        let phi_pos = get_phi(i);
        ref_table[i as usize] = Complex::new(phi_pos.cos(), phi_pos.sin());

        let phi_neg = get_phi(-i);
        ref_table[t_u - i as usize] = Complex::new(phi_neg.cos(), phi_neg.sin());
    }

    ref_table
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prs_shape_and_unit_magnitude() {
        let prs = phase_reference();

        // Length is T_u.
        assert_eq!(prs.len(), 2048);

        // DC bin is the null carrier.
        assert_eq!(prs[0], Complex::new(0.0, 0.0));

        // Exactly 1536 active carriers (K).
        let nonzero: Vec<&Complex<f32>> =
            prs.iter().filter(|z| z.norm_sqr() != 0.0).collect();
        assert_eq!(nonzero.len(), 1536);

        // Every active carrier is unit modulus.
        for z in nonzero {
            assert!((z.norm() - 1.0).abs() < 1e-5, "non-unit magnitude: {z:?}");
        }
    }
}
