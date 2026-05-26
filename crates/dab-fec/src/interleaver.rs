//! Forney convolutional byte (de)interleaver as used by Korean T-DMB
//! outer FEC (ETSI TS 102 427 §7.2).
//!
//! 12 branches, depth `M = 17` bytes per stage. Branch `i` on the encoder
//! side has `i*M` shift-register stages; the matching decoder uses
//! `(N-1-i)*M`, so the total chain delay is constant
//! `(N-1)*M*N = 11*17*12 = 2244` stream bytes for every branch.
//!
//! Faithful translation of `tdmb/fec/interleaver.py`.

/// Number of interleaver branches.
pub const N_BRANCHES: usize = 12;
/// Depth (bytes) of one shift-register stage per branch index.
pub const M_DEPTH: usize = 17;

/// Exact-latency shift register: `shift(b)` outputs the byte that was put in
/// `depth` cycles ago. The first `depth` outputs are zeros. A depth-0 register
/// is a passthrough.
struct ShiftReg {
    buf: Vec<u8>,
    head: usize,
}

impl ShiftReg {
    fn new(depth: usize) -> Self {
        ShiftReg {
            buf: vec![0u8; depth],
            head: 0,
        }
    }

    #[inline]
    fn shift(&mut self, b: u8) -> u8 {
        if self.buf.is_empty() {
            return b;
        }
        let out = self.buf[self.head];
        self.buf[self.head] = b;
        self.head = (self.head + 1) % self.buf.len();
        out
    }
}

/// Stream-mode deinterleaver. Branch `i` has `(N-1-i)*M` stages of delay.
///
/// Total end-to-end (interleaver + deinterleaver) latency is
/// `(N-1)*M*N = 2244` bytes — the first 2244 output bytes are garbage.
pub struct ConvolutionalDeinterleaver {
    regs: Vec<ShiftReg>,
    idx: usize,
}

impl ConvolutionalDeinterleaver {
    pub fn new() -> Self {
        let regs = (0..N_BRANCHES)
            .map(|i| ShiftReg::new((N_BRANCHES - 1 - i) * M_DEPTH))
            .collect();
        ConvolutionalDeinterleaver { regs, idx: 0 }
    }

    /// Push bytes through the deinterleaver; returns the same number of
    /// (delayed) output bytes.
    pub fn feed(&mut self, data: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; data.len()];
        let mut idx = self.idx;
        for (k, &b) in data.iter().enumerate() {
            out[k] = self.regs[idx].shift(b);
            idx = (idx + 1) % N_BRANCHES;
        }
        self.idx = idx;
        out
    }

    /// End-to-end (encoder + decoder) latency in bytes.
    pub fn latency_bytes(&self) -> usize {
        (N_BRANCHES - 1) * M_DEPTH * N_BRANCHES
    }
}

impl Default for ConvolutionalDeinterleaver {
    fn default() -> Self {
        Self::new()
    }
}

/// Encoder side, useful for tests. Branch `i` has `i*M` stages.
pub struct ConvolutionalInterleaver {
    regs: Vec<ShiftReg>,
    idx: usize,
}

impl ConvolutionalInterleaver {
    pub fn new() -> Self {
        let regs = (0..N_BRANCHES)
            .map(|i| ShiftReg::new(i * M_DEPTH))
            .collect();
        ConvolutionalInterleaver { regs, idx: 0 }
    }

    /// Push bytes through the interleaver; returns the same number of
    /// (delayed) output bytes.
    pub fn feed(&mut self, data: &[u8]) -> Vec<u8> {
        let mut out = vec![0u8; data.len()];
        let mut idx = self.idx;
        for (k, &b) in data.iter().enumerate() {
            out[k] = self.regs[idx].shift(b);
            idx = (idx + 1) % N_BRANCHES;
        }
        self.idx = idx;
        out
    }
}

impl Default for ConvolutionalInterleaver {
    fn default() -> Self {
        Self::new()
    }
}
