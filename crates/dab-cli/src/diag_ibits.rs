//! `dab diag-ibits` — cross-validate per-symbol soft bits against an
//! eti-stuff dump.
//!
//! The companion patch in `eti-cmdline/src/ofdm/ofdm-processor.cpp` (gated
//! on the env var `DAB_RS_DIAG_DUMP`) emits one record per OFDM data symbol:
//!
//! ```text
//! u32 frame_idx        // 1-based, increments on every ofdmSymbolCount==2
//! u32 ofdm_symbol_idx  // 2..=76 (Mode I: 75 data symbols per frame)
//! i16 ibits[3072]      // 1536 I-bits then 1536 Q-bits, ±127 range
//! ```
//!
//! This comparator runs the dab-rs OFDM chain on the same I/Q capture,
//! captures the equivalent ibits per data symbol, aligns the two streams by
//! sweeping small frame-index offsets, picks the offset that maximises
//! match-rate, and reports the first byte-level divergence within the best
//! alignment.
//!
//! A perfect chain produces match rate ≈ 1.0 (modulo low-bit noise from the
//! CU8 quantization round-trip that the oracle sees). A systematic
//! divergence — bit-reversed bytes, scaled magnitudes, freq-de-interleaver
//! permutation, sign flip — produces a low match rate with a recognisable
//! pattern that the report captures.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::Path;

use anyhow::{anyhow, Result};
use num_complex::Complex;

use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::{
    detect_integer_cfo, CpSync, DifferentialReference, DqpskDemap, Nco, NullDetector, Resampler,
    SymbolFft,
};

/// Strategy for ingesting the dab-rs side's I/Q samples.
///
/// `Cs16Le3MSPS` matches our native capture format and is the path dab-rs
/// normally runs; the oracle saw a CU8-quantised, resampled version of this.
/// `Cu82048k` reads the very same CU8 file the oracle consumed — bit-for-bit
/// identical input, eliminating quantisation as a source of divergence.
#[derive(Debug, Clone, Copy)]
pub enum IqIngest {
    Cs16Le3MSPS,
    Cu82048k,
}

const TS: usize = 2552;
const NULL_LEN: usize = 2656;
const SYMBOLS_PER_FRAME: usize = 76;
const BITS_PER_SYMBOL: usize = 3072;
const FS_INTERNAL: f64 = 2_048_000.0;

/// Header layout of one oracle dump record: `u32 frame_idx, u32 symbol_idx`.
const ORACLE_HEADER_BYTES: usize = 8;
const ORACLE_PAYLOAD_BYTES: usize = BITS_PER_SYMBOL * 2; // i16 * 3072
const ORACLE_RECORD_BYTES: usize = ORACLE_HEADER_BYTES + ORACLE_PAYLOAD_BYTES;

/// FFT-bin dump payload: T_u = 2048 complex<float> samples = 16384 bytes.
const FFT_PAYLOAD_BYTES: usize = 2048 * 8;
const FFT_RECORD_BYTES: usize = ORACLE_HEADER_BYTES + FFT_PAYLOAD_BYTES;

/// Key for indexing per-symbol records.
type SymKey = (u32, u32);

/// Aggregate outcome of one diagnostic run.
#[derive(Debug, Default)]
pub struct DiagResult {
    pub oracle_records: usize,
    pub dab_rs_records: usize,
    pub best_offset: i32,
    pub aligned_pairs: usize,
    pub overall_match_rate: f64,
    pub first_divergence: Option<Divergence>,
    /// Per-i16 sample diff histogram (binned at 0, ≤2, ≤8, ≤32, >32).
    pub diff_hist: [usize; 5],
}

#[derive(Debug, Clone)]
pub struct Divergence {
    pub dab_rs_frame: u32,
    pub oracle_frame: u32,
    pub symbol_idx: u32,
    pub bit_index: usize,
    pub oracle_value: i16,
    pub dab_rs_value: i16,
    pub i_or_q: char, // 'I' for [0..1536), 'Q' for [1536..3072)
    pub carrier_logical_index: usize, // bit_index % 1536
}

/// Read every record from the oracle dump into a `(frame, symbol) → ibits` map.
fn read_oracle_dump(path: &Path) -> Result<HashMap<SymKey, Vec<i16>>> {
    let f = File::open(path).map_err(|e| anyhow!("open {}: {e}", path.display()))?;
    let mut r = BufReader::with_capacity(1 << 20, f);

    let mut buf = vec![0u8; ORACLE_RECORD_BYTES];
    let mut map: HashMap<SymKey, Vec<i16>> = HashMap::new();
    loop {
        match r.read_exact(&mut buf) {
            Ok(()) => {}
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e.into()),
        }
        let frame_idx = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        let sym_idx = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        let mut ibits = vec![0_i16; BITS_PER_SYMBOL];
        for i in 0..BITS_PER_SYMBOL {
            let lo = ORACLE_HEADER_BYTES + 2 * i;
            ibits[i] = i16::from_le_bytes([buf[lo], buf[lo + 1]]);
        }
        map.insert((frame_idx, sym_idx), ibits);
    }
    Ok(map)
}

/// Run dab-rs's OFDM chain on the capture, yielding per-symbol ibits keyed by
/// dab-rs's own `(frame_idx, ofdm_symbol_idx)` (frame_idx starts at 1; symbol
/// indices match the oracle convention 2..=76). Returns at most `max_frames`
/// frames' worth of records.
fn read_cu8_2048k(path: &Path) -> Result<Vec<Complex<f32>>> {
    // CU8 layout: pairs of (u8 I, u8 Q). Convert each byte v to f32 via
    // (v - 128) / 128, matching eti-stuff's `rawfileHandler::getSamples`.
    let bytes = std::fs::read(path).map_err(|e| anyhow!("read {}: {e}", path.display()))?;
    if bytes.len() % 2 != 0 {
        return Err(anyhow!("cu8 file has odd byte count {}", bytes.len()));
    }
    let mut out = Vec::with_capacity(bytes.len() / 2);
    for pair in bytes.chunks_exact(2) {
        let i = (pair[0] as f32 - 128.0) / 128.0;
        let q = (pair[1] as f32 - 128.0) / 128.0;
        out.push(Complex::new(i, q));
    }
    Ok(out)
}

fn capture_dab_rs_ibits(
    iq_path: &Path,
    ingest: IqIngest,
    max_frames: usize,
) -> Result<HashMap<SymKey, Vec<i16>>> {
    let resampled: Vec<Complex<f32>> = match ingest {
        IqIngest::Cs16Le3MSPS => {
            let mut reader = IqFileReader::open(iq_path, IqFormat::Cs16Le, 3_000_000)?;
            let mut resampler = Resampler::new_3m_to_2048k();
            let mut out: Vec<Complex<f32>> = Vec::with_capacity(41_000_000);
            let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
            loop {
                let n = reader.read_samples(&mut buf)?;
                if n == 0 {
                    break;
                }
                out.extend_from_slice(&resampler.process(&buf[..n]));
            }
            out
        }
        IqIngest::Cu82048k => read_cu8_2048k(iq_path)?,
    };

    let nulls = NullDetector::new(2_048_000).detect(&resampled);
    let cp = CpSync::mode_i();
    let mut sfft = SymbolFft::mode_i();
    let demap = DqpskDemap::mode_i();

    let mut map: HashMap<SymKey, Vec<i16>> = HashMap::new();
    let mut frame_idx: u32 = 0;

    for &null_pos in &nulls.positions {
        if frame_idx as usize >= max_frames {
            break;
        }
        let prs_guess = match null_pos.checked_add(NULL_LEN) {
            Some(v) => v,
            None => continue,
        };
        if prs_guess + SYMBOLS_PER_FRAME * TS > resampled.len() {
            continue;
        }
        let prs_start = cp.fine_time(&resampled, prs_guess, TS);
        if prs_start + SYMBOLS_PER_FRAME * TS > resampled.len() {
            continue;
        }
        let cfo_hz = cp.estimate_cfo_hz(&resampled, prs_start, 50) as f64;
        if !cfo_hz.is_finite() || cfo_hz.abs() > 600.0 {
            continue;
        }
        let prs_spec_raw = fft_one(&resampled, prs_start, cfo_hz, &mut sfft);
        let icfo = detect_integer_cfo(&prs_spec_raw, 5);
        let delta = if icfo.peak > 1.5 * icfo.runner_up {
            icfo.offset
        } else {
            0
        };
        let prs_spec = rotate(&prs_spec_raw, delta);

        let mut diff_ref = DifferentialReference::new();
        diff_ref.seed_prs(&prs_spec);

        frame_idx += 1;
        // Symbol indices 2..=76 (75 data symbols), matching the oracle's
        // `ofdmSymbolCount` convention.
        for sym_idx in 2..=SYMBOLS_PER_FRAME as u32 {
            let s = (sym_idx - 1) as usize; // 1..=75
            let cp_start = prs_start + s * TS;
            let spec_raw = fft_one(&resampled, cp_start, cfo_hz, &mut sfft);
            let spec = rotate(&spec_raw, delta);
            let diff = diff_ref.step(&spec);
            let bits = demap.demap(&diff);
            debug_assert_eq!(bits.len(), BITS_PER_SYMBOL);
            map.insert((frame_idx, sym_idx), bits);
        }
    }

    Ok(map)
}

fn fft_one(
    resampled: &[Complex<f32>],
    cp_start: usize,
    cfo_hz: f64,
    sfft: &mut SymbolFft,
) -> Vec<Complex<f32>> {
    let mut region = resampled[cp_start..cp_start + TS].to_vec();
    Nco::new(FS_INTERNAL).mix(&mut region, -cfo_hz);
    sfft.fft_symbol(&region)
}

fn rotate(spec: &[Complex<f32>], delta: i32) -> Vec<Complex<f32>> {
    let n = spec.len();
    if delta == 0 {
        return spec.to_vec();
    }
    let shift = (delta.rem_euclid(n as i32)) as usize;
    let mut out = Vec::with_capacity(n);
    out.extend_from_slice(&spec[shift..]);
    out.extend_from_slice(&spec[..shift]);
    out
}

/// Compute the diff histogram for two same-length i16 slices.
fn update_hist(hist: &mut [usize; 5], a: &[i16], b: &[i16]) {
    for (x, y) in a.iter().zip(b.iter()) {
        let d = (x - y).unsigned_abs();
        if d == 0 {
            hist[0] += 1;
        } else if d <= 2 {
            hist[1] += 1;
        } else if d <= 8 {
            hist[2] += 1;
        } else if d <= 32 {
            hist[3] += 1;
        } else {
            hist[4] += 1;
        }
    }
}

/// Compare two (frame, symbol) maps. Tries oracle-frame offsets in
/// `[-5, +5]` and picks the offset that maximises exact-match count across
/// every symbol where both maps have a record.
pub fn compare(
    dab_rs: &HashMap<SymKey, Vec<i16>>,
    oracle: &HashMap<SymKey, Vec<i16>>,
) -> DiagResult {
    let mut result = DiagResult {
        oracle_records: oracle.len(),
        dab_rs_records: dab_rs.len(),
        ..DiagResult::default()
    };
    if dab_rs.is_empty() || oracle.is_empty() {
        return result;
    }

    // ---- Find best alignment offset (oracle_frame = dab_rs_frame + offset) ----
    // Sweep ±150 frames to cover the case where eti-stuff and dab-rs disagree
    // on which RF frame to call "frame 1" (different sync acquisition).
    let mut best_offset = 0_i32;
    let mut best_exact: usize = 0;
    for offset in -150_i32..=150 {
        let mut exact = 0_usize;
        for (&(f, s), a) in dab_rs.iter() {
            let of = (f as i32) + offset;
            if of <= 0 {
                continue;
            }
            if let Some(b) = oracle.get(&(of as u32, s)) {
                for (x, y) in a.iter().zip(b.iter()) {
                    if x == y {
                        exact += 1;
                    }
                }
            }
        }
        if exact > best_exact {
            best_exact = exact;
            best_offset = offset;
        }
    }
    result.best_offset = best_offset;

    // ---- Collect pairs at best alignment, fill histogram + first divergence ----
    let mut pairs: Vec<(SymKey, &Vec<i16>, &Vec<i16>)> = Vec::new();
    for (&(f, s), a) in dab_rs.iter() {
        let of = (f as i32) + best_offset;
        if of <= 0 {
            continue;
        }
        if let Some(b) = oracle.get(&(of as u32, s)) {
            pairs.push(((f, s), a, b));
        }
    }
    pairs.sort_by_key(|&(k, _, _)| k);

    let mut total_bits = 0_usize;
    let mut matching = 0_usize;
    for (_, a, b) in &pairs {
        update_hist(&mut result.diff_hist, a, b);
        total_bits += a.len();
        for (x, y) in a.iter().zip(b.iter()) {
            if x == y {
                matching += 1;
            }
        }
    }
    result.aligned_pairs = pairs.len();
    result.overall_match_rate = if total_bits > 0 {
        matching as f64 / total_bits as f64
    } else {
        0.0
    };

    // First divergence in lexicographic-order alignment.
    'outer: for ((f, s), a, b) in &pairs {
        for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
            if x != y {
                let i_or_q = if i < 1536 { 'I' } else { 'Q' };
                result.first_divergence = Some(Divergence {
                    dab_rs_frame: *f,
                    oracle_frame: ((*f as i32) + best_offset) as u32,
                    symbol_idx: *s,
                    bit_index: i,
                    oracle_value: *y,
                    dab_rs_value: *x,
                    i_or_q,
                    carrier_logical_index: i % 1536,
                });
                break 'outer;
            }
        }
    }

    result
}

/// CLI entry point: run the diagnostic and print a report.
pub fn run_diag(
    iq_path: &Path,
    ingest: IqIngest,
    oracle_dump_path: &Path,
    max_frames: usize,
) -> Result<DiagResult> {
    let oracle = read_oracle_dump(oracle_dump_path)?;
    let dab_rs = capture_dab_rs_ibits(iq_path, ingest, max_frames)?;
    Ok(compare(&dab_rs, &oracle))
}

/// Side-by-side dump of one (dab_rs_frame, symbol) record at the best
/// alignment offset. Prints first `n_show` `i16` values from oracle and
/// dab-rs, plus permutation diagnostics (do the sorted values match?).
pub fn dump_pair(
    iq_path: &Path,
    ingest: IqIngest,
    oracle_dump_path: &Path,
    dab_rs_frame: u32,
    symbol_idx: u32,
    n_show: usize,
) -> Result<()> {
    let oracle = read_oracle_dump(oracle_dump_path)?;
    let dab_rs = capture_dab_rs_ibits(iq_path, ingest, dab_rs_frame as usize + 4)?;
    let r = compare(&dab_rs, &oracle);
    println!("alignment best_offset={}", r.best_offset);
    let dab_v = match dab_rs.get(&(dab_rs_frame, symbol_idx)) {
        Some(v) => v,
        None => {
            println!("no dab-rs record for ({dab_rs_frame}, {symbol_idx})");
            return Ok(());
        }
    };
    let oracle_f = (dab_rs_frame as i32 + r.best_offset) as u32;
    let ora_v = match oracle.get(&(oracle_f, symbol_idx)) {
        Some(v) => v,
        None => {
            println!("no oracle record for ({oracle_f}, {symbol_idx})");
            return Ok(());
        }
    };
    println!(
        "comparing dab_rs ({dab_rs_frame}, {symbol_idx}) vs oracle ({oracle_f}, {symbol_idx})"
    );
    println!("  first {n_show} oracle: {:?}", &ora_v[..n_show.min(ora_v.len())]);
    println!("  first {n_show} dab_rs: {:?}", &dab_v[..n_show.min(dab_v.len())]);

    // Permutation hypothesis: do the multisets match?
    let mut a = ora_v.clone();
    let mut b = dab_v.clone();
    a.sort_unstable();
    b.sort_unstable();
    let multiset_match = a == b;
    println!("multiset equality (sorted lists match): {multiset_match}");

    // Permutation hypothesis stratified by I/Q halves.
    let mut a_i = ora_v[..1536].to_vec();
    let mut a_q = ora_v[1536..].to_vec();
    let mut b_i = dab_v[..1536].to_vec();
    let mut b_q = dab_v[1536..].to_vec();
    a_i.sort_unstable();
    a_q.sort_unstable();
    b_i.sort_unstable();
    b_q.sort_unstable();
    println!(
        "sorted equality (I half / Q half): {} / {}",
        a_i == b_i,
        a_q == b_q
    );

    // I↔Q swap hypothesis: does dab_rs[I] match oracle[Q] (and vice versa) when sorted?
    println!(
        "I/Q swap test (dab_rs.I == oracle.Q?): {}",
        b_i == a_q
    );

    // Sign-flip hypothesis: dab_rs ibits == -oracle ibits position-wise.
    let neg_match_count: usize = dab_v.iter().zip(ora_v.iter()).filter(|(a, b)| **a == -**b).count();
    println!(
        "sign-flip match count: {} / {} = {:.4}",
        neg_match_count,
        dab_v.len(),
        neg_match_count as f64 / dab_v.len() as f64,
    );

    // Bin-shift hypothesis (off-by-1 carrier): check positional match after shifting dab_rs by N positions.
    for shift in [-3_i32, -2, -1, 1, 2, 3] {
        let s = shift.rem_euclid(1536) as usize;
        let m: usize = (0..1536)
            .filter(|&i| dab_v[(i + s) % 1536] == ora_v[i])
            .count();
        println!("  I half shift {}: {}/1536 = {:.4}", shift, m, m as f64 / 1536.0);
    }
    Ok(())
}

/// Print a human-readable summary of a [`DiagResult`].
pub fn print_report(r: &DiagResult) {
    println!("oracle_records={}", r.oracle_records);
    println!("dab_rs_records={}", r.dab_rs_records);
    println!("best_oracle_offset={}", r.best_offset);
    println!("aligned_pairs={}", r.aligned_pairs);
    println!("overall_match_rate={:.4}", r.overall_match_rate);
    println!("diff_histogram (|oracle - dab_rs|):");
    println!("  =0       {}", r.diff_hist[0]);
    println!("  1..=2    {}", r.diff_hist[1]);
    println!("  3..=8    {}", r.diff_hist[2]);
    println!("  9..=32   {}", r.diff_hist[3]);
    println!("  >32      {}", r.diff_hist[4]);
    match &r.first_divergence {
        Some(d) => {
            println!(
                "first_divergence: dab_rs_frame={} oracle_frame={} sym={} i={} ({} carrier {}) \
                 oracle={} dab_rs={}",
                d.dab_rs_frame,
                d.oracle_frame,
                d.symbol_idx,
                d.bit_index,
                d.i_or_q,
                d.carrier_logical_index,
                d.oracle_value,
                d.dab_rs_value,
            );
        }
        None => println!("first_divergence: none (perfect match)"),
    }
}
