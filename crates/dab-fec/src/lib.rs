//! Korean T-DMB outer FEC: sync-aligned Forney deinterleaver + RS(204,188)
//! (ETSI TS 102 427).
//!
//! Faithful Rust translation of the validated Python reference
//! (`tdmb/fec/{outer,interleaver,rs}.py`). The pipeline aligns the incoming
//! MSC byte stream on the `0x47` TS sync cadence, runs a 12-branch Forney
//! convolutional deinterleaver, drops the 2244-byte settling transient, then
//! RS(204,188)-decodes each block into a 188-byte MPEG-2 TS packet.

mod interleaver;
mod outer;
mod rs;

pub use interleaver::{
    ConvolutionalDeinterleaver, ConvolutionalInterleaver, M_DEPTH, N_BRANCHES,
};
pub use outer::{
    KoreanTDmbOuterFec, Stats, TsPacket, DEINTERLEAVER_LATENCY, PRESYNC_PROBE_BYTES,
    RS_BLOCK_SIZE, TS_PACKET_SIZE,
};
pub use rs::{RsDecoder, RsEncoder, RsResult};

#[cfg(test)]
mod tests {
    use super::*;

    /// Tiny deterministic LCG so tests are reproducible without an RNG crate.
    struct Lcg(u64);
    impl Lcg {
        fn new(seed: u64) -> Self {
            Lcg(seed)
        }
        fn next_byte(&mut self) -> u8 {
            // Numerical Recipes LCG constants.
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 33) as u8
        }
        fn bytes(&mut self, n: usize) -> Vec<u8> {
            (0..n).map(|_| self.next_byte()).collect()
        }
    }

    // ---------------------------------------------------------------
    // 1. Interleaver ↔ deinterleaver identity + latency constant.
    // ---------------------------------------------------------------
    #[test]
    fn interleaver_deinterleaver_identity() {
        assert_eq!(DEINTERLEAVER_LATENCY, 2244);
        assert_eq!(DEINTERLEAVER_LATENCY, (N_BRANCHES - 1) * M_DEPTH * N_BRANCHES);

        let mut rng = Lcg::new(0xC0FFEE);
        let n = 20_000;
        let input = rng.bytes(n);

        let mut interleaver = ConvolutionalInterleaver::new();
        let mut deinterleaver = ConvolutionalDeinterleaver::new();

        let interleaved = interleaver.feed(&input);
        let output = deinterleaver.feed(&interleaved);

        // End-to-end latency: output[i + 2244] == input[i].
        let lat = DEINTERLEAVER_LATENCY;
        assert_eq!(deinterleaver.latency_bytes(), lat);
        for i in 0..(n - lat) {
            assert_eq!(
                output[i + lat],
                input[i],
                "mismatch at i={i} (output idx {})",
                i + lat
            );
        }
    }

    // ---------------------------------------------------------------
    // 2. RS round-trip: correct up to 8 errors, fail at 9+.
    // ---------------------------------------------------------------
    #[test]
    fn rs_roundtrip_correctable() {
        let mut rng = Lcg::new(42);
        let payload = rng.bytes(188);
        let enc = RsEncoder::new();
        let mut codeword = enc.encode(&payload);
        assert_eq!(codeword.len(), 204);

        // Corrupt 8 distinct byte positions (T=8 is the correction limit).
        let positions = [0usize, 13, 27, 50, 100, 150, 180, 203];
        for &p in &positions {
            codeword[p] ^= 0xFF;
        }

        let mut dec = RsDecoder::new();
        let res = dec.decode(&codeword);
        assert!(res.ok, "8 errors must be correctable");
        assert_eq!(res.errors, 8, "must report 8 corrected errors");
        assert_eq!(res.data, payload, "recovered payload must match original");
    }

    #[test]
    fn rs_roundtrip_uncorrectable() {
        let mut rng = Lcg::new(7);
        let payload = rng.bytes(188);
        let enc = RsEncoder::new();
        let mut codeword = enc.encode(&payload);

        // Corrupt 9 byte positions — beyond the T=8 correction limit.
        let positions = [0usize, 13, 27, 50, 100, 150, 175, 188, 203];
        for &p in &positions {
            codeword[p] ^= 0xAB;
        }

        let mut dec = RsDecoder::new();
        let res = dec.decode(&codeword);
        assert!(!res.ok, "9 errors must be uncorrectable");
        assert_eq!(res.errors, -1, "uncorrectable must report rs_errors == -1");
    }

    // ---------------------------------------------------------------
    // 3. Phase alignment at offset 160.
    // ---------------------------------------------------------------
    #[test]
    fn phase_alignment_offset_160() {
        let offset = 160usize;
        // Enough blocks to exceed PRESYNC_PROBE_BYTES and clear the
        // n_blocks >= 20 / first >= max(8, 0.4*n_blocks) thresholds.
        let n_blocks = 60usize;
        let total = n_blocks * RS_BLOCK_SIZE;

        // Random background, then plant 0x47 every 204 bytes at phase `offset`.
        let mut rng = Lcg::new(0xABCDEF);
        let mut stream = rng.bytes(total);
        let mut pos = offset;
        while pos < total {
            stream[pos] = 0x47;
            pos += RS_BLOCK_SIZE;
        }

        let mut fec = KoreanTDmbOuterFec::new();
        let _ = fec.feed(&stream);
        let stats = fec.stats();
        assert!(stats.aligned, "must lock alignment");
        assert_eq!(stats.phase_offset, offset as i32, "must lock phase 160");
    }
}
