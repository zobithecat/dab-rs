//! Mode I reference-phase table and `get_phi(k)`.
//!
//! ETSI EN 300 401 §14.3.2 (phase reference symbol). Ported verbatim from the
//! `eti-stuff` oracle (`src/ofdm/phasetable.cpp`, `includes/ofdm/phasetable.h`).
//!
//! The phase for carrier `k` is `pi/2 * (h_table(i, k - kmin) + n)` where the
//! `(kmin, kmax, i, n)` quad is the table row whose range contains `k`.

use std::f32::consts::PI;

/// One row of the Mode I phase table: `[kmin, kmax, i, n]`.
struct PhaseElement {
    kmin: i32,
    kmax: i32,
    i: i32,
    n: i32,
}

/// Mode I phase table, copied verbatim from `modeI_table` in `phasetable.cpp`.
///
/// NOTE: the `{97, 128, 1, 1}` row carries the 2014-09-03 Jorgen Scott bug-fix
/// (`i = 1`, not `2`); it is reproduced exactly as the oracle ships it.
static MODE_I_TABLE: &[PhaseElement] = &[
    PhaseElement { kmin: -768, kmax: -737, i: 0, n: 1 },
    PhaseElement { kmin: -736, kmax: -705, i: 1, n: 2 },
    PhaseElement { kmin: -704, kmax: -673, i: 2, n: 0 },
    PhaseElement { kmin: -672, kmax: -641, i: 3, n: 1 },
    PhaseElement { kmin: -640, kmax: -609, i: 0, n: 3 },
    PhaseElement { kmin: -608, kmax: -577, i: 1, n: 2 },
    PhaseElement { kmin: -576, kmax: -545, i: 2, n: 2 },
    PhaseElement { kmin: -544, kmax: -513, i: 3, n: 3 },
    PhaseElement { kmin: -512, kmax: -481, i: 0, n: 2 },
    PhaseElement { kmin: -480, kmax: -449, i: 1, n: 1 },
    PhaseElement { kmin: -448, kmax: -417, i: 2, n: 2 },
    PhaseElement { kmin: -416, kmax: -385, i: 3, n: 3 },
    PhaseElement { kmin: -384, kmax: -353, i: 0, n: 1 },
    PhaseElement { kmin: -352, kmax: -321, i: 1, n: 2 },
    PhaseElement { kmin: -320, kmax: -289, i: 2, n: 3 },
    PhaseElement { kmin: -288, kmax: -257, i: 3, n: 3 },
    PhaseElement { kmin: -256, kmax: -225, i: 0, n: 2 },
    PhaseElement { kmin: -224, kmax: -193, i: 1, n: 2 },
    PhaseElement { kmin: -192, kmax: -161, i: 2, n: 2 },
    PhaseElement { kmin: -160, kmax: -129, i: 3, n: 1 },
    PhaseElement { kmin: -128, kmax: -97, i: 0, n: 1 },
    PhaseElement { kmin: -96, kmax: -65, i: 1, n: 3 },
    PhaseElement { kmin: -64, kmax: -33, i: 2, n: 1 },
    PhaseElement { kmin: -32, kmax: -1, i: 3, n: 2 },
    PhaseElement { kmin: 1, kmax: 32, i: 0, n: 3 },
    PhaseElement { kmin: 33, kmax: 64, i: 3, n: 1 },
    PhaseElement { kmin: 65, kmax: 96, i: 2, n: 1 },
    // { 97, 128, 2, 1 } — original; superseded by the bug-fix below.
    PhaseElement { kmin: 97, kmax: 128, i: 1, n: 1 },
    PhaseElement { kmin: 129, kmax: 160, i: 0, n: 2 },
    PhaseElement { kmin: 161, kmax: 192, i: 3, n: 2 },
    PhaseElement { kmin: 193, kmax: 224, i: 2, n: 1 },
    PhaseElement { kmin: 225, kmax: 256, i: 1, n: 0 },
    PhaseElement { kmin: 257, kmax: 288, i: 0, n: 2 },
    PhaseElement { kmin: 289, kmax: 320, i: 3, n: 2 },
    PhaseElement { kmin: 321, kmax: 352, i: 2, n: 3 },
    PhaseElement { kmin: 353, kmax: 384, i: 1, n: 3 },
    PhaseElement { kmin: 385, kmax: 416, i: 0, n: 0 },
    PhaseElement { kmin: 417, kmax: 448, i: 3, n: 2 },
    PhaseElement { kmin: 449, kmax: 480, i: 2, n: 1 },
    PhaseElement { kmin: 481, kmax: 512, i: 1, n: 3 },
    PhaseElement { kmin: 513, kmax: 544, i: 0, n: 3 },
    PhaseElement { kmin: 545, kmax: 576, i: 3, n: 3 },
    PhaseElement { kmin: 577, kmax: 608, i: 2, n: 3 },
    PhaseElement { kmin: 609, kmax: 640, i: 1, n: 0 },
    PhaseElement { kmin: 641, kmax: 672, i: 0, n: 3 },
    PhaseElement { kmin: 673, kmax: 704, i: 3, n: 0 },
    PhaseElement { kmin: 705, kmax: 736, i: 2, n: 1 },
    PhaseElement { kmin: 737, kmax: 768, i: 1, n: 1 },
];

// The h-tables: each is a 16-entry pattern repeated twice (32 entries).
#[rustfmt::skip]
static H0: [i32; 32] = [
    0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1,
    0, 2, 0, 0, 0, 0, 1, 1, 2, 0, 0, 0, 2, 2, 1, 1,
];
#[rustfmt::skip]
static H1: [i32; 32] = [
    0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0,
    0, 3, 2, 3, 0, 1, 3, 0, 2, 1, 2, 3, 2, 3, 3, 0,
];
#[rustfmt::skip]
static H2: [i32; 32] = [
    0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3,
    0, 0, 0, 2, 0, 2, 1, 3, 2, 2, 0, 2, 2, 0, 1, 3,
];
#[rustfmt::skip]
static H3: [i32; 32] = [
    0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2,
    0, 1, 2, 1, 0, 3, 3, 2, 2, 3, 2, 1, 2, 1, 3, 2,
];

/// Select h-table `i` (0..=3) and index it by `j`, matching `phaseTable::h_table`.
fn h_table(i: i32, j: i32) -> i32 {
    let table = match i {
        0 => &H0,
        1 => &H1,
        2 => &H2,
        _ => &H3,
    };
    table[j as usize]
}

/// Reference phase `Phi(k)` for carrier index `k`, in radians.
///
/// Mirrors `phaseTable::get_Phi`. The C++ path uses `std::complex<float>`, so
/// `f32` precision is used here for bit-faithful parity.
///
/// # Panics
/// Panics if `k` falls outside the Mode I carrier range `[-768, 768] \ {0}`.
pub fn get_phi(k: i32) -> f32 {
    for row in MODE_I_TABLE {
        if row.kmin <= k && k <= row.kmax {
            return PI / 2.0 * (h_table(row.i, k - row.kmin) + row.n) as f32;
        }
    }
    panic!("get_phi: carrier {k} out of Mode I range");
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-5;

    #[test]
    fn get_phi_hand_checked() {
        // k = 1: row {1, 32, 0, 3}, j = 0, h0[0] = 0 -> pi/2 * (0 + 3) = 3*pi/2.
        assert!((get_phi(1) - 3.0 * PI / 2.0).abs() < TOL);

        // k = -1: row {-32, -1, 3, 2}, j = 31, h3[31] = 2 -> pi/2 * (2 + 2) = 2*pi.
        assert!((get_phi(-1) - 2.0 * PI).abs() < TOL);

        // k = 33: row {33, 64, 3, 1}, j = 0, h3[0] = 0 -> pi/2 * (0 + 1) = pi/2.
        assert!((get_phi(33) - PI / 2.0).abs() < TOL);

        // k = 98: row {97, 128, 1, 1} (bug-fixed i=1), j = 1, h1[1] = 3
        //         -> pi/2 * (3 + 1) = 2*pi.
        assert!((get_phi(98) - 2.0 * PI).abs() < TOL);

        // k = -768: row {-768, -737, 0, 1}, j = 0, h0[0] = 0 -> pi/2 * (0 + 1) = pi/2.
        assert!((get_phi(-768) - PI / 2.0).abs() < TOL);

        // k = 768: row {737, 768, 1, 1}, j = 31, h1[31] = 0 -> pi/2 * (0 + 1) = pi/2.
        assert!((get_phi(768) - PI / 2.0).abs() < TOL);
    }
}
