//! DAB transmission-mode parameters.
//!
//! Mode I constants per ETSI EN 300 401 §14.5 (transmission frame structure)
//! ported verbatim from the `eti-stuff` oracle (`src/support/dab-params.cpp`).

/// Static DAB transmission parameters for a given mode.
///
/// All fields are sample- or carrier-counts that drive the OFDM front end.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DabParams {
    /// Transmission mode identifier (1 for Mode I).
    pub mode: u8,
    /// Number of OFDM symbols (blocks) per transmission frame.
    pub l: u16,
    /// Number of active carriers `K`.
    pub carriers: u16,
    /// Samples per transmission frame `T_F`.
    pub t_f: u32,
    /// Null-symbol length in samples `T_NULL`.
    pub t_null: u32,
    /// OFDM symbol (block) length in samples `T_s` (= `T_u + T_g`).
    pub t_s: u32,
    /// Useful symbol part in samples `T_u` (the FFT size).
    pub t_u: u32,
    /// Guard interval (cyclic prefix) length in samples `T_g`.
    pub t_g: u32,
    /// Carrier spacing in Hz.
    pub carrier_diff: u32,
}

impl DabParams {
    /// DAB Mode I parameters (the only mode this crate targets).
    pub const fn mode_i() -> Self {
        DabParams {
            mode: 1,
            l: 76,
            carriers: 1536,
            t_f: 196_608,
            t_null: 2656,
            t_s: 2552,
            t_u: 2048,
            t_g: 504,
            carrier_diff: 1000,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_i_constants() {
        let p = DabParams::mode_i();
        assert_eq!(p.mode, 1);
        assert_eq!(p.l, 76);
        assert_eq!(p.carriers, 1536);
        assert_eq!(p.t_f, 196_608);
        assert_eq!(p.t_null, 2656);
        assert_eq!(p.t_s, 2552);
        assert_eq!(p.t_u, 2048);
        assert_eq!(p.t_g, 504);
        assert_eq!(p.carrier_diff, 1000);
        // T_s must equal T_u + T_g.
        assert_eq!(p.t_s, p.t_u + p.t_g);
    }
}
