//! Scalar Viterbi decoder for the DAB rate-1/4, K=7 mother convolutional code.
//!
//! Faithful port of the scalar `viterbiHandler` from the eti-stuff oracle
//! (`eti-cmdline/src/eti-handling/viterbi-handler.cpp`). The SIMD `viterbiSpiral`
//! variant is intentionally NOT ported; the scalar path is the reference logic
//! and is what we validate against.
//!
//! Code parameters (EN 300 401, clause 11): constraint length `K = 7`,
//! rate 1/4, generator polynomials (octal) 0o133, 0o171, 0o145, 0o133, giving
//! `numStates = 1 << (K - 1) = 64`.
//!
//! Soft-bit convention (from the oracle): `i16`, where `-255 => bit 1` and
//! `+255 => bit 0`.

/// Constraint length of the DAB mother code.
pub const K: usize = 7;
/// Generator polynomials (octal) of the DAB rate-1/4 mother code.
pub const POLYS: [u32; 4] = [0o133, 0o171, 0o145, 0o133];

const NUM_STATES: usize = 1 << (K - 1); // 64

/// `bitFor(state, poly, bit)` — given the register state, a polynomial and the
/// bit being shifted in, returns the parity bit produced by the encoder.
///
/// Verbatim port of `viterbiHandler::bitFor`: the register after shifting `bit`
/// in is `state` (bit==0) or `state + NUM_STATES` (bit!=0), masked by `poly`,
/// then XOR-reduced over the low `K + 1` bits (the C++ loop runs `i = 0..=K`).
pub fn bit_for(state: u32, poly: u32, bit: u32) -> u8 {
    let mut the_register = if bit == 0 { state } else { state + NUM_STATES as u32 };
    the_register &= poly;
    let mut res_bit: u8 = 0;
    // C++: for (int i = 0; i <= K; i++) -> K + 1 iterations.
    for _ in 0..=K {
        res_bit ^= (the_register & 1) as u8;
        the_register >>= 1;
    }
    res_bit
}

/// Scalar Viterbi decoder, decoding `block_length` message bits per call.
///
/// Mirrors `viterbiHandler`: the input soft-bit array has length
/// `4 * block_length + 24` (the `+ 24` are the 6 four-symbol tail columns), and
/// `block_length` bits are produced.
pub struct Viterbi {
    block_length: usize,
    /// `indexTable[2 * NUM_STATES]`: maps (bit, state) to the 4-bit pattern of
    /// encoder outputs, used as a `cost_table` index.
    index_table: [i16; 2 * NUM_STATES],
    predecessor_for_0: [usize; NUM_STATES],
    predecessor_for_1: [usize; NUM_STATES],
    /// `transCosts[col][state]` — accumulated path costs.
    trans_costs: Vec<[i32; NUM_STATES]>,
    /// `history[col][state]` — best predecessor state.
    history: Vec<[usize; NUM_STATES]>,
    state_sequence: Vec<usize>,
}

impl Viterbi {
    /// Builds a decoder for `block_length` message bits.
    pub fn new(block_length: usize) -> Self {
        // C++ allocates blockLength + 6 columns of transCosts/history and
        // blockLength + 6 + 1 stateSequence entries, all zeroed.
        let cols = block_length + 6;
        let trans_costs = vec![[0i32; NUM_STATES]; cols];
        let history = vec![[0usize; NUM_STATES]; cols];
        let state_sequence = vec![0usize; block_length + 6 + 1];

        // Per-poly tables: poly_table[i * NUM_STATES + j] = bitFor(j, poly, i).
        let build = |poly: u32| -> [u8; 2 * NUM_STATES] {
            let mut t = [0u8; 2 * NUM_STATES];
            for i in 0..2 {
                for j in 0..NUM_STATES {
                    t[i * NUM_STATES + j] = bit_for(j as u32, poly, i as u32);
                }
            }
            t
        };
        let poly1 = build(POLYS[0]);
        let poly2 = build(POLYS[1]);
        let poly3 = build(POLYS[2]);
        let poly4 = build(POLYS[3]);

        // indexTable maps the four poly output bits to a 4-bit index.
        let mut index_table = [0i16; 2 * NUM_STATES];
        for i in 0..2 * NUM_STATES {
            index_table[i] = (if poly1[i] != 0 { 8 } else { 0 }
                + if poly2[i] != 0 { 4 } else { 0 }
                + if poly3[i] != 0 { 2 } else { 0 }
                + if poly4[i] != 0 { 1 } else { 0 }) as i16;
        }

        let mut predecessor_for_0 = [0usize; NUM_STATES];
        let mut predecessor_for_1 = [0usize; NUM_STATES];
        for i in 0..NUM_STATES {
            predecessor_for_0[i] = ((i << 1) + 0) & (NUM_STATES - 1);
            predecessor_for_1[i] = ((i << 1) + 1) & (NUM_STATES - 1);
        }

        Viterbi {
            block_length,
            index_table,
            predecessor_for_0,
            predecessor_for_1,
            trans_costs,
            history,
            state_sequence,
        }
    }

    /// `computeCostTable` — verbatim port. Builds the 16 branch costs from the
    /// four (already negated) soft symbols.
    fn compute_cost_table(sym_0: i32, sym_1: i32, sym_2: i32, sym_3: i32) -> [i32; 16] {
        [
            -sym_0 - sym_1 - sym_2 - sym_3,
            -sym_0 - sym_1 - sym_2 + sym_3,
            -sym_0 - sym_1 + sym_2 - sym_3,
            -sym_0 - sym_1 + sym_2 + sym_3,
            -sym_0 + sym_1 - sym_2 - sym_3,
            -sym_0 + sym_1 - sym_2 + sym_3,
            -sym_0 + sym_1 + sym_2 - sym_3,
            -sym_0 + sym_1 + sym_2 + sym_3,
            sym_0 - sym_1 - sym_2 - sym_3,
            sym_0 - sym_1 - sym_2 + sym_3,
            sym_0 - sym_1 + sym_2 - sym_3,
            sym_0 - sym_1 + sym_2 + sym_3,
            sym_0 + sym_1 - sym_2 - sym_3,
            sym_0 + sym_1 - sym_2 + sym_3,
            sym_0 + sym_1 + sym_2 - sym_3,
            sym_0 + sym_1 + sym_2 + sym_3,
        ]
    }

    /// Decodes `soft` (length `4 * block_length + 24`) into `block_length`
    /// bits (each `0u8` or `1u8`). Verbatim port of `viterbiHandler::deconvolve`.
    ///
    /// # Panics
    /// Panics if `soft.len() != 4 * block_length + 24`.
    pub fn deconvolve(&mut self, soft: &[i16]) -> Vec<u8> {
        assert_eq!(
            soft.len(),
            4 * self.block_length + 24,
            "soft length must be 4 * block_length + 24"
        );

        // Reset cost/history columns; column 0 stays at cost 0 (assumed start).
        for col in self.trans_costs.iter_mut() {
            *col = [0i32; NUM_STATES];
        }
        for col in self.history.iter_mut() {
            *col = [0usize; NUM_STATES];
        }

        let block_length = self.block_length;

        // Pump soft bits into the trellis, computing the cost matrix.
        for i in 1..block_length + 6 {
            // Note: the oracle negates each symbol here, and computeCostTable
            // negates again internally. Both negations are preserved exactly.
            let sym_0 = -(soft[4 * (i - 1) + 0] as i32);
            let sym_1 = -(soft[4 * (i - 1) + 1] as i32);
            let sym_2 = -(soft[4 * (i - 1) + 2] as i32);
            let sym_3 = -(soft[4 * (i - 1) + 3] as i32);

            let cost_table = Self::compute_cost_table(sym_0, sym_1, sym_2, sym_3);

            let prev_costs = self.trans_costs[i - 1];

            // First half: cState in 0..NUM_STATES/2 (entry bit 0).
            for c_state in 0..NUM_STATES / 2 {
                let prev_0 = self.predecessor_for_0[c_state];
                let prev_1 = self.predecessor_for_1[c_state];
                let costs_0 =
                    prev_costs[prev_0] + cost_table[self.index_table[prev_0] as usize];
                let costs_1 =
                    prev_costs[prev_1] + cost_table[self.index_table[prev_1] as usize];
                if costs_0 < costs_1 {
                    self.trans_costs[i][c_state] = costs_0;
                    self.history[i][c_state] = prev_0;
                } else {
                    self.trans_costs[i][c_state] = costs_1;
                    self.history[i][c_state] = prev_1;
                }
            }

            // Second half: cState in NUM_STATES/2..NUM_STATES (entry bit 1);
            // cost index uses prev_xx + NUM_STATES.
            for c_state in NUM_STATES / 2..NUM_STATES {
                let prev_0 = self.predecessor_for_0[c_state];
                let prev_1 = self.predecessor_for_1[c_state];
                let costs_0 = prev_costs[prev_0]
                    + cost_table[self.index_table[prev_0 + NUM_STATES] as usize];
                let costs_1 = prev_costs[prev_1]
                    + cost_table[self.index_table[prev_1 + NUM_STATES] as usize];
                if costs_0 < costs_1 {
                    self.trans_costs[i][c_state] = costs_0;
                    self.history[i][c_state] = prev_0;
                } else {
                    self.trans_costs[i][c_state] = costs_1;
                    self.history[i][c_state] = prev_1;
                }
            }
        }

        // Minimal-cost end state in column blockLength + 6 - 1.
        let last = block_length + 6 - 1;
        let mut minimal_costs = 1_000_000i32;
        let mut best_state = 0usize;
        for i in 0..NUM_STATES {
            if self.trans_costs[last][i] < minimal_costs {
                minimal_costs = self.trans_costs[last][i];
                best_state = i;
            }
        }

        self.state_sequence[last] = best_state;
        // Traceback to state 0.
        for i in (1..=last).rev() {
            self.state_sequence[i - 1] = self.history[i][self.state_sequence[i]];
        }

        let mut bits = vec![0u8; block_length];
        for i in 1..=block_length {
            bits[i - 1] = if self.state_sequence[i] >= NUM_STATES / 2 { 1 } else { 0 };
        }
        bits
    }
}

/// Rate-1/4 convolutional encoder using the four DAB polynomials, appending 6
/// zero tail bits (matching the K=7 trellis termination the decoder expects).
///
/// `n` message bits in → `4 * (n + 6)` coded bits (0/1) out. This is a
/// test/utility encoder, not part of the oracle; the bit order per step matches
/// the decoder's poly ordering so that round-trips recover the message.
pub fn convolutional_encode(message_bits: &[u8]) -> Vec<u8> {
    let mut state: u32 = 0;
    let total = message_bits.len() + 6;
    let mut out = Vec::with_capacity(4 * total);
    for step in 0..total {
        let bit: u32 = if step < message_bits.len() {
            (message_bits[step] & 1) as u32
        } else {
            0
        };
        // bitFor expects the *current* state and the bit being shifted in; the
        // decoder's index_table is built the same way, so emit in poly order.
        for &poly in POLYS.iter() {
            out.push(bit_for(state, poly, bit));
        }
        // State update must invert the decoder's trellis. The decoder uses
        // `predecessor = (cState << 1 | entryBit) & 63`, i.e.
        // `cState = (predecessor >> 1) | (entryBit << (K - 2))`, and recovers
        // the bit as `cState >= NUM_STATES / 2` (its high bit, position K-2).
        // So the encoder advances the register the same way.
        state = (state >> 1) | (bit << (K - 2));
    }
    out
}
