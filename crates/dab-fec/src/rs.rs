//! Reed-Solomon (204, 188, T=8) decoder for MPEG-2 TS / Korean T-DMB outer
//! FEC.
//!
//! Per ETSI TS 102 427 (§7.1), identical to DVB-T:
//! ```text
//!   Field GF(2^8) with primitive poly 0x11D
//!   Generator g(x) = ∏_{i=0}^{15} (x - α^i),  α = 2
//!   fcr = 0,  prim = 1
//! ```
//!
//! The Python reference delegated to the `reedsolo` library — a direct port of
//! the Wikiversity "Reed-Solomon for coders" tutorial. We use the Rust
//! `reed-solomon` crate (mersinvald, 0.2), which is a port of the SAME
//! tutorial. We verified its field/generator parameters match DVB exactly:
//!
//! * `gf::EXP[1] == 0x02` ⇒ α = 2, and the field tables are built for the
//!   `0x11D` primitive polynomial (see the crate's `gf/mod.rs`).
//! * `generator_poly` builds roots `gf::pow(2, i)` for `i in 0..ecclen`, i.e.
//!   roots α^0 .. α^15 ⇒ fcr = 0, generator = 2.
//! * `correct_err_count` returns the corrected-error count and reports
//!   `DecoderError::TooManyErrors` for uncorrectable blocks, mirroring
//!   reedsolo's `len(errata_pos)` / `ReedSolomonError`.
//!
//! Because the crate is the same tutorial port with identical DVB parameters,
//! we did NOT hand-roll a decoder.

use reed_solomon::{Decoder, Encoder};

/// Codeword length (bytes).
pub const N: usize = 204;
/// Message length (bytes).
pub const K: usize = 188;
/// Number of parity (ECC) symbols.
pub const NSYM: usize = N - K; // 16

/// Result of decoding one 204-byte RS block.
pub struct RsResult {
    /// 188 recovered message bytes (the TS packet).
    pub data: Vec<u8>,
    /// Number of byte errors corrected; `-1` if uncorrectable.
    pub errors: i32,
    /// `true` if decode succeeded (with 0..=8 corrections).
    pub ok: bool,
}

/// Decode 204-byte RS-coded blocks back to 188-byte TS packets, tracking
/// frame statistics.
pub struct RsDecoder {
    decoder: Decoder,
    pub frames_total: usize,
    pub frames_ok: usize,
    pub frames_corrected: usize,
    pub frames_failed: usize,
}

impl RsDecoder {
    pub fn new() -> Self {
        RsDecoder {
            decoder: Decoder::new(NSYM),
            frames_total: 0,
            frames_ok: 0,
            frames_corrected: 0,
            frames_failed: 0,
        }
    }

    /// Decode one 204-byte block. Panics if `block.len() != N`.
    pub fn decode(&mut self, block: &[u8]) -> RsResult {
        assert_eq!(block.len(), N, "need {N}-byte RS block, got {}", block.len());
        self.frames_total += 1;
        match self.decoder.correct_err_count(block, None) {
            Ok((buffer, n_err)) => {
                let data = buffer.data().to_vec();
                if n_err == 0 {
                    self.frames_ok += 1;
                } else {
                    self.frames_corrected += 1;
                }
                RsResult {
                    data,
                    errors: n_err as i32,
                    ok: true,
                }
            }
            Err(_) => {
                self.frames_failed += 1;
                RsResult {
                    data: block[..K].to_vec(),
                    errors: -1,
                    ok: false,
                }
            }
        }
    }
}

impl Default for RsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// RS(204,188) encoder exposed for round-trip tests. Encodes 188 message
/// bytes into a 204-byte codeword (message followed by 16 parity bytes).
pub struct RsEncoder {
    encoder: Encoder,
}

impl RsEncoder {
    pub fn new() -> Self {
        RsEncoder {
            encoder: Encoder::new(NSYM),
        }
    }

    /// Encode a 188-byte message into a 204-byte codeword. Panics if
    /// `data.len() != K`.
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        assert_eq!(data.len(), K, "need {K}-byte message, got {}", data.len());
        let encoded = self.encoder.encode(data);
        encoded[..].to_vec()
    }
}

impl Default for RsEncoder {
    fn default() -> Self {
        Self::new()
    }
}
