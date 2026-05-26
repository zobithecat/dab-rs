//! Korean T-DMB outer FEC pipeline (ETSI TS 102 427).
//!
//! Chain:
//!   1. Convolutional time deinterleaver (Forney, 12 branches × 17 bytes)
//!   2. Reed-Solomon (204, 188) decoder
//!
//! Input:  raw MSC sub-channel byte stream from ETI (one CIF per ETI frame,
//!         concatenated). The stream contains 204-byte RS blocks that have
//!         been Forney-conv-interleaved at the transmitter.
//! Output: 188-byte MPEG-2 TS packets (sync byte `0x47` first).
//!
//! Per Eo & Bahk 2024 (§III-B) and analysis on `k8b_100pct.eti`:
//! "The deinterleaver requires the sync byte of a TS packet to be the first
//! in an input byte chunk."
//!
//! Why: the TX-side Forney interleaver routes byte `i` of an RS block through
//! branch `(i mod 12)` with `i*17` cycles of delay. Byte 0 (= `0x47`) goes
//! through branch 0 with no delay, so `0x47` lands at TX-output positions
//! 0, 204, 408, … But ETI captures don't start exactly at "byte 0 of RS
//! block #0" — we typically start somewhere in the middle (empirically
//! offset 160 within the 204-byte cycle). If we feed those bytes straight
//! into the RX deinterleaver, which always starts on branch 0, the branch
//! indices are off by `(160 mod 12) = 4` → RS blocks come out byte-scrambled
//! and RS decode fails on every block, even though the `0x47` sync pattern is
//! preserved at the right cadence.
//!
//! Fix: in [`KoreanTDmbOuterFec::feed`] we first locate the `0x47` alignment
//! by hit-counting at 204-byte stride across the raw input, then DISCARD the
//! bytes up to that offset and only feed the deinterleaver from the `0x47`
//! onwards. After the 2244-byte settling transient (= `(N-1)·M·N`) the
//! deinterleaver emits clean 204-byte RS codewords ready for systematic
//! RS(204,188) correction.

use crate::interleaver::{ConvolutionalDeinterleaver, M_DEPTH, N_BRANCHES};
use crate::rs::RsDecoder;

/// MPEG-2 TS packet size (bytes).
pub const TS_PACKET_SIZE: usize = 188;
/// RS-coded block size (bytes).
pub const RS_BLOCK_SIZE: usize = 204;
/// Conv-interleaver chain latency = `(N-1)·M·N = 11·17·12 = 2244` bytes.
pub const DEINTERLEAVER_LATENCY: usize = (N_BRANCHES - 1) * M_DEPTH * N_BRANCHES;
/// Min bytes needed to confidently pick the `0x47` phase before locking in.
/// 50 RS blocks ≈ 10 200 bytes ⇒ ~50 hits expected if the signal is real.
pub const PRESYNC_PROBE_BYTES: usize = RS_BLOCK_SIZE * 50;

/// A decoded MPEG-2 TS packet.
pub struct TsPacket {
    /// 188 bytes; `data[0] == 0x47` when sync recovered.
    pub data: Vec<u8>,
    /// Number of byte errors corrected; `-1` if RS uncorrectable.
    pub rs_errors: i32,
}

/// Pipeline statistics snapshot.
pub struct Stats {
    pub bytes_in: usize,
    pub aligned: bool,
    /// `-1` until locked.
    pub phase_offset: i32,
    pub rs_total: usize,
    /// Decoded with 0 corrections.
    pub rs_ok: usize,
    /// Decoded with >= 1 correction.
    pub rs_corrected: usize,
    /// Uncorrectable.
    pub rs_failed: usize,
    pub uncorrectable: usize,
    pub bytes_discarded_presync: usize,
}

/// Stream pipeline: align → deinterleave → RS decode → TS packets.
pub struct KoreanTDmbOuterFec {
    presync_buf: Vec<u8>, // raw bytes waiting for phase lock
    aligned: bool,        // did we lock the 0x47 phase?
    phase_offset: i32,    // selected 204-byte cycle phase

    di: Option<ConvolutionalDeinterleaver>,
    di_skip_remaining: usize, // transient to drop
    post_di_buf: Vec<u8>,
    rs: RsDecoder,

    bytes_in: usize,
    bytes_discarded_presync: usize,
    uncorrectable: usize,
}

impl KoreanTDmbOuterFec {
    pub fn new() -> Self {
        KoreanTDmbOuterFec {
            presync_buf: Vec::new(),
            aligned: false,
            phase_offset: -1,
            di: None,
            di_skip_remaining: DEINTERLEAVER_LATENCY,
            post_di_buf: Vec::new(),
            rs: RsDecoder::new(),
            bytes_in: 0,
            bytes_discarded_presync: 0,
            uncorrectable: 0,
        }
    }

    /// Push raw MSC sub-channel bytes; returns 0..N decoded TS packets.
    pub fn feed(&mut self, data: &[u8]) -> Vec<TsPacket> {
        if data.is_empty() {
            return Vec::new();
        }
        self.bytes_in += data.len();

        if !self.aligned {
            self.presync_buf.extend_from_slice(data);
            if self.presync_buf.len() < PRESYNC_PROBE_BYTES {
                return Vec::new();
            }
            self.try_align();
            if !self.aligned {
                // Keep buffer bounded if we never find sync.
                if self.presync_buf.len() > 4 * PRESYNC_PROBE_BYTES {
                    let drop = self.presync_buf.len() - PRESYNC_PROBE_BYTES;
                    self.presync_buf.drain(..drop);
                    self.bytes_discarded_presync += drop;
                }
                return Vec::new();
            }
            // Aligned: feed the survivor bytes through the deinterleaver.
            self.feed_aligned_initial()
        } else {
            // Steady-state.
            self.feed_aligned(data)
        }
    }

    /// Statistics snapshot.
    pub fn stats(&self) -> Stats {
        Stats {
            bytes_in: self.bytes_in,
            aligned: self.aligned,
            phase_offset: self.phase_offset,
            rs_total: self.rs.frames_total,
            rs_ok: self.rs.frames_ok,
            rs_corrected: self.rs.frames_corrected,
            rs_failed: self.rs.frames_failed,
            uncorrectable: self.uncorrectable,
            bytes_discarded_presync: self.bytes_discarded_presync,
        }
    }

    /// Score each of 204 phases by `0x47` hit-rate; lock if any phase clearly
    /// dominates (>40% of expected slots) and is far above the chance
    /// background (~0.4% for random bytes).
    fn try_align(&mut self) {
        let buf = &self.presync_buf;
        let n = buf.len();
        let n_blocks = n / RS_BLOCK_SIZE;
        if n_blocks < 20 {
            return;
        }
        let mut scores = [0usize; RS_BLOCK_SIZE];
        for k in 0..n_blocks {
            let base = k * RS_BLOCK_SIZE;
            // Tally 0x47 occurrences at the candidate phases (full sweep).
            for ph in 0..RS_BLOCK_SIZE {
                if buf[base + ph] == 0x47 {
                    scores[ph] += 1;
                }
            }
        }
        // `max(range(...), key=...)` in Python returns the FIRST index of the
        // maximum on ties; `Iterator::max_by_key` returns the LAST. Replicate
        // Python's first-index semantics.
        let mut best = 0usize;
        for ph in 1..RS_BLOCK_SIZE {
            if scores[ph] > scores[best] {
                best = ph;
            }
        }
        // Demand convincing dominance: best phase hit > 40% of blocks AND
        // > 5× the second-best (otherwise we're seeing random matches).
        let mut sorted_hits = scores;
        sorted_hits.sort_unstable_by(|a, b| b.cmp(a));
        let first = sorted_hits[0];
        let second = sorted_hits[1];
        if (first as f64) < (8.0_f64).max(n_blocks as f64 * 0.4) {
            return;
        }
        if first < second * 5 {
            return;
        }
        self.phase_offset = best as i32;
        self.aligned = true;
    }

    /// First call after alignment: trim presync buffer to the `0x47` position
    /// and start the deinterleaver from there.
    fn feed_aligned_initial(&mut self) -> Vec<TsPacket> {
        if self.phase_offset > 0 {
            let off = self.phase_offset as usize;
            self.presync_buf.drain(..off);
            self.bytes_discarded_presync += off;
        }
        self.di = Some(ConvolutionalDeinterleaver::new());
        let survivor = std::mem::take(&mut self.presync_buf);
        self.feed_aligned(&survivor)
    }

    fn feed_aligned(&mut self, data: &[u8]) -> Vec<TsPacket> {
        let di = self
            .di
            .as_mut()
            .expect("deinterleaver must exist once aligned");
        let mut deint = di.feed(data);
        if self.di_skip_remaining > 0 {
            if deint.len() <= self.di_skip_remaining {
                self.di_skip_remaining -= deint.len();
                return Vec::new();
            }
            deint.drain(..self.di_skip_remaining);
            self.di_skip_remaining = 0;
        }
        self.post_di_buf.extend_from_slice(&deint);
        self.drain_rs()
    }

    fn drain_rs(&mut self) -> Vec<TsPacket> {
        let mut packets = Vec::new();
        while self.post_di_buf.len() >= RS_BLOCK_SIZE {
            let block: Vec<u8> = self.post_di_buf.drain(..RS_BLOCK_SIZE).collect();
            let res = self.rs.decode(&block);
            if !res.ok {
                self.uncorrectable += 1;
                packets.push(TsPacket {
                    data: res.data[..TS_PACKET_SIZE].to_vec(),
                    rs_errors: -1,
                });
            } else {
                packets.push(TsPacket {
                    data: res.data[..TS_PACKET_SIZE].to_vec(),
                    rs_errors: res.errors,
                });
            }
        }
        packets
    }
}

impl Default for KoreanTDmbOuterFec {
    fn default() -> Self {
        Self::new()
    }
}
