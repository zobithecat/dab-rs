//! `dab msc-iq` pipeline — raw I/Q → MSC sub-channel byte stream → MPEG-TS.
//!
//! Wires Stages 1-7 from `dab-ofdm` (same as `fic_iq`) but extends the data-
//! symbol demap loop from 3 FIC symbols to all 75 data symbols. After the
//! FIC chain decodes the ensemble, we look up the requested sub-channel and
//! decode its MSC bits per CIF.
//!
//! # MSC chain per eti-stuff `eti-generator.cpp`
//!
//! 1. Per DAB frame, demap 75 data symbols → 230400 soft bits.
//!    - First 9216 bits = FIC region (3 OFDM symbols).
//!    - Remaining 221184 bits = MSC region (72 OFDM symbols = 4 CIFs × 18 syms).
//! 2. Split MSC region into 4 CIFs × 55296 soft bits each.
//! 3. Apply 16-CIF time deinterleaver (Forney-like, bit-position based):
//!    `temp[i] = cifVector[(idx_out + interleaveMap[i & 0xF]) & 0xF][i]`
//!    with `interleaveMap = [0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15]`.
//!    The 16-slot ring buffer means we need 16 CIFs of warm-up before the
//!    first valid deinterleaved CIF emerges.
//! 4. Per sub-channel per CIF:
//!    a. Extract bits `cif[start_cu*64 .. (start_cu+size_cu)*64]`.
//!    b. `EepProtection::deconvolve` → 24*bit_rate decoded info bits.
//!    c. XOR with PRBS (x^9 + x^5 + 1, init all-ones), length 24*bit_rate.
//!    d. Pack MSB-first → 3*bit_rate bytes per CIF.
//! 5. Concatenate sub-channel bytes across CIFs → byte stream.
//! 6. Feed to `KoreanTDmbOuterFec::feed` → MPEG-TS packets.
//!
//! # SLICE-1 scope (current implementation)
//!
//! Stages 1-2 implemented (demap extension + CIF split). Stages 3-6 are
//! scaffolded but not yet wired — the time deinterleaver, EepProtection
//! integration, descramble, and outer-FEC path are deferred to a follow-up
//! slice. This module currently reports structural counts (CIFs assembled,
//! sub-channel bits per CIF) so the next slice can plug in the decode chain
//! without changing the demap-and-split foundation.

use std::path::Path;

use anyhow::Result;
use num_complex::Complex;

use dab_descramble::prbs_sequence;
use dab_fec::{KoreanTDmbOuterFec, TsPacket};
use dab_fic::{Ensemble, SubChannel};
use dab_iq::{IqFileReader, IqFormat};
use dab_ofdm::{
    estimate_offset_eti, CpSync, DifferentialReference, DqpskDemap, LinearResampler, Nco,
    NullDetector, SymbolFft,
};
use dab_viterbi::{EepProtection, FIC_IN_BITS, FIC_OUT_BITS};

use crate::fic_iq::{fft_symbol_corrected, rotate_spectrum, FS_INTERNAL};

/// Mode I OFDM per-symbol length (`T_g + T_u = 504 + 2048`) at 2.048 MSPS.
const TS: usize = 2552;
/// Mode I null-symbol length.
const NULL_LEN: usize = 2656;
/// Number of OFDM data symbols carrying FIC + MSC per frame (Mode I L=76,
/// minus the PRS at index 0 → 75 data symbols).
const DATA_SYMBOLS: usize = 75;
/// Soft bits per OFDM symbol.
const BITS_PER_SYMBOL: usize = 3072;
/// FIC region symbols (the first 3 data symbols).
const FIC_SYMBOLS: usize = 3;
/// FIC soft bits per frame (4 ficBlocks × 2304 = 9216).
const FIC_SOFT_BITS_PER_FRAME: usize = FIC_SYMBOLS * BITS_PER_SYMBOL;
/// FicBlocks per frame.
const FIC_BLOCKS_PER_FRAME: usize = FIC_SOFT_BITS_PER_FRAME / FIC_IN_BITS;
/// FIC bytes per frame (4 × 96 = 384 bytes = 12 FIBs).
#[allow(dead_code)]
const FIC_BYTES_PER_FRAME: usize = FIC_BLOCKS_PER_FRAME * (FIC_OUT_BITS / 8);
/// MSC region symbols per frame.
const MSC_SYMBOLS: usize = DATA_SYMBOLS - FIC_SYMBOLS; // 72
/// Number of OFDM data symbols per CIF (Mode I).
const SYMS_PER_CIF: usize = 18;
/// CIFs per DAB frame (Mode I = 4).
const CIFS_PER_FRAME: usize = MSC_SYMBOLS / SYMS_PER_CIF; // 4
/// Bits per CIF (18 × 3072 = 55296).
const BITS_PER_CIF: usize = SYMS_PER_CIF * BITS_PER_SYMBOL;
/// Capacity Unit size in bits (DAB spec).
const CU_SIZE_BITS: usize = 64;

/// `interleaveMap` from `eti-stuff/eti-generator.cpp:207`. Indexes the 16-CIF
/// ring buffer per bit position `i & 0xF` for the time deinterleaver.
const INTERLEAVE_MAP: [usize; 16] = [0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15];

/// Result of MSC sub-channel decode.
#[derive(Default)]
pub struct MscIqResult {
    /// FIC ensemble (same as `fic-iq` produces).
    pub ensemble: Ensemble,
    /// Sub-channel that was selected for decode.
    pub sub_channel: Option<SubChannel>,
    /// Frames demapped (entire MSC region).
    pub frames_decoded: usize,
    /// CIFs assembled (4 per frame).
    pub cifs_assembled: usize,
    /// Sub-channel bits accumulated (per-CIF extraction, before EepProtection).
    pub subch_bits_total: usize,
    /// Sub-channel bytes decoded (post-EepProtection + descramble + pack).
    pub subch_bytes_total: usize,
    /// MPEG-TS packets that passed RS(204, 188) outer FEC.
    pub ts_packets: Vec<TsPacket>,
}

/// Run the FIC chain to identify the ensemble + sub-channel, then re-process
/// the same I/Q decoding all 75 data symbols and routing the requested
/// sub-channel through EepProtection + descramble + outer FEC.
///
/// SLICE-1 status: demap extension and CIF assembly are implemented; the
/// time deinterleaver, EepProtection wiring, and outer-FEC integration are
/// scaffolded so the next slice can verify each stage against eti-stuff
/// dumps without re-touching the OFDM front-end.
pub fn process_iq_to_msc(
    iq_path: &Path,
    input_format: IqFormat,
    input_sample_rate_hz: u32,
    sub_ch_id: u8,
) -> Result<MscIqResult> {
    // ---- Pass 1: read + resample + run FIC chain to discover ensemble ----
    let fic_result = crate::fic_iq::process_iq_to_fic(iq_path, input_format, input_sample_rate_hz)?;
    let ensemble = fic_result.ensemble.clone();

    let sub_channel = ensemble.sub_channels.get(&sub_ch_id).cloned();
    if sub_channel.is_none() {
        return Ok(MscIqResult {
            ensemble,
            sub_channel: None,
            frames_decoded: 0,
            cifs_assembled: 0,
            subch_bits_total: 0,
            subch_bytes_total: 0,
            ts_packets: Vec::new(),
        });
    }
    let sub = sub_channel.unwrap();

    // ---- Pass 2: re-process IQ, demap all 75 data symbols per frame ----
    let bypass_resampler =
        input_format == IqFormat::Cf32Le && input_sample_rate_hz == 2_048_000;
    let mut reader = IqFileReader::open(iq_path, input_format, input_sample_rate_hz)?;
    let mut resampler = LinearResampler::new(input_sample_rate_hz);
    let mut resampled: Vec<Complex<f32>> = Vec::with_capacity(41_000_000);
    let mut buf = vec![Complex::new(0.0_f32, 0.0); 1 << 20];
    loop {
        let n = reader.read_samples(&mut buf)?;
        if n == 0 {
            break;
        }
        if bypass_resampler {
            resampled.extend_from_slice(&buf[..n]);
        } else {
            resampled.extend_from_slice(&resampler.process(&buf[..n]));
        }
    }
    let nulls = NullDetector::new(2_048_000).detect(&resampled);

    let cp = CpSync::mode_i();
    let mut sfft = SymbolFft::mode_i();
    let demap = DqpskDemap::mode_i();

    let mut frame_nco = Nco::new(FS_INTERNAL);
    let mut cumulative_cfo_hz: f64 = 0.0;

    // Time deinterleaver: 16-slot ring of one CIF each (i16 soft bits).
    let mut cif_ring: Vec<Vec<i16>> = vec![vec![0i16; BITS_PER_CIF]; 16];
    let mut ring_idx: usize = 0;
    let mut ring_filled: usize = 0; // counts CIFs written, capped at 16

    let mut result = MscIqResult {
        ensemble: ensemble.clone(),
        sub_channel: Some(sub.clone()),
        ..MscIqResult::default()
    };

    // Sub-channel decode state (built once we hit pass-2 settled output).
    let bit_rate = sub.bitrate_kbps as i16; // 352 for EEP-3A SubCh 1
    let prot_level: i16 = decode_prot_level(&sub.protection);
    let mut eep = EepProtection::new(bit_rate, prot_level);
    let subch_in_bits_per_cif = (sub.size_cu as usize) * CU_SIZE_BITS;
    let prbs_per_cif = prbs_sequence(eep.out_size());
    let mut outer = KoreanTDmbOuterFec::new();
    let mut decoded_bytes_total = 0usize;

    eprintln!(
        "msc-iq: scid={} start={}cu size={}cu prot={} {}kbps; subch_bits_per_cif={} eep_out_bits={}",
        sub.sub_ch_id,
        sub.start_addr,
        sub.size_cu,
        sub.protection,
        sub.bitrate_kbps,
        subch_in_bits_per_cif,
        eep.out_size(),
    );

    for &null_pos in &nulls.positions {
        let prs_guess = match null_pos.checked_add(NULL_LEN) {
            Some(v) => v,
            None => continue,
        };
        let frame_end_min = prs_guess + (1 + DATA_SYMBOLS) * TS;
        if frame_end_min > resampled.len() {
            continue;
        }
        let prs_start = cp.fine_time(&resampled, prs_guess, 126);
        if prs_start + (1 + DATA_SYMBOLS) * TS > resampled.len() {
            continue;
        }

        let abs_cfo_hz = cp.estimate_cfo_hz(&resampled, prs_start, 50) as f64;
        if !abs_cfo_hz.is_finite() || abs_cfo_hz.abs() > 600.0 {
            continue;
        }
        if result.frames_decoded == 0 {
            cumulative_cfo_hz = abs_cfo_hz;
        } else {
            cumulative_cfo_hz = 0.9 * cumulative_cfo_hz + 0.1 * abs_cfo_hz;
        }
        let cfo_hz = cumulative_cfo_hz;

        let prs_spec_raw =
            fft_symbol_corrected(&resampled, prs_start, cfo_hz, &mut sfft, &mut frame_nco);
        let delta = estimate_offset_eti(&prs_spec_raw);
        let nco_extra_hz = (delta as f64) * 1000.0;
        let prs_spec = rotate_spectrum(&prs_spec_raw, delta);

        let mut diff_ref = DifferentialReference::new();
        diff_ref.seed_prs(&prs_spec);

        // Demap all 75 data symbols.
        let mut frame_soft: Vec<i16> = Vec::with_capacity(DATA_SYMBOLS * BITS_PER_SYMBOL);
        let mut ok = true;
        for s in 1..=DATA_SYMBOLS {
            let cp_start = prs_start + s * TS;
            let data_cfo = cfo_hz + nco_extra_hz;
            let spec = fft_symbol_corrected(
                &resampled, cp_start, data_cfo, &mut sfft, &mut frame_nco);
            let diff = diff_ref.step(&spec);
            let bits = demap.demap(&diff);
            if bits.len() != BITS_PER_SYMBOL {
                ok = false;
                break;
            }
            frame_soft.extend_from_slice(&bits);
        }
        if !ok || frame_soft.len() != DATA_SYMBOLS * BITS_PER_SYMBOL {
            continue;
        }
        result.frames_decoded += 1;

        // Split: FIC (first 9216 bits) and MSC (next 221184 bits = 4 CIFs).
        // We don't need to redo the FIC accumulator (pass 1 did it); just take MSC.
        let msc_slice = &frame_soft[FIC_SOFT_BITS_PER_FRAME..];
        debug_assert_eq!(msc_slice.len(), CIFS_PER_FRAME * BITS_PER_CIF);

        for cif_idx in 0..CIFS_PER_FRAME {
            let cif_off = cif_idx * BITS_PER_CIF;
            let cif_in = &msc_slice[cif_off..cif_off + BITS_PER_CIF];

            // Time deinterleave: emit a freshly-deinterleaved CIF by reading
            // each bit position from the ring slot indexed via interleaveMap.
            // We write the *current* CIF into ring[ring_idx] BEFORE reading
            // (eti-stuff writes after read; we do the same below).
            let mut deint = vec![0i16; BITS_PER_CIF];
            for i in 0..BITS_PER_CIF {
                let slot = (ring_idx + INTERLEAVE_MAP[i & 0xF]) & 0xF;
                deint[i] = cif_ring[slot][i];
            }
            // Now write the current CIF input into ring[ring_idx] for future reads.
            cif_ring[ring_idx & 0xF].copy_from_slice(cif_in);
            ring_idx = (ring_idx + 1) & 0xF;

            result.cifs_assembled += 1;

            // Wait 15 CIFs of warmup; the 16th CIF emits valid output.
            // Verbatim from eti-stuff `eti-generator.cpp` (`if (amount < 15)`),
            // not 16 — the 16th read still uses one freshly-written slot
            // (the interleaveMap-=0 bit positions read from the current
            // write slot which gets overwritten in this same iteration).
            if ring_filled < 15 {
                ring_filled += 1;
                continue;
            }

            // Extract sub-channel bits from deinterleaved CIF.
            let sub_off = (sub.start_addr as usize) * CU_SIZE_BITS;
            let sub_end = sub_off + subch_in_bits_per_cif;
            if sub_end > deint.len() {
                continue;
            }
            let subch_soft: Vec<i16> = deint[sub_off..sub_end].to_vec();
            result.subch_bits_total += subch_soft.len();

            // EepProtection: depuncture + Viterbi → out_size bits.
            let info_bits = eep.deconvolve(&subch_soft);
            debug_assert_eq!(info_bits.len(), eep.out_size());

            // Descramble: XOR with PRBS (x^9+x^5+1, init=ones), 24*bit_rate bits.
            let mut descrambled: Vec<u8> = info_bits
                .iter()
                .zip(prbs_per_cif.iter())
                .map(|(b, p)| b ^ p)
                .collect();

            // Pack MSB-first → 3*bit_rate bytes per CIF.
            let n_bytes = descrambled.len() / 8;
            let mut bytes: Vec<u8> = Vec::with_capacity(n_bytes);
            for byte_i in 0..n_bytes {
                let mut v = 0u8;
                for bit_i in 0..8 {
                    v = (v << 1) | (descrambled[byte_i * 8 + bit_i] & 1);
                }
                bytes.push(v);
            }
            decoded_bytes_total += bytes.len();
            result.subch_bytes_total += bytes.len();

            // Feed outer FEC. Sync alignment + RS(204,188) handled internally.
            let ts = outer.feed(&bytes);
            result.ts_packets.extend(ts);

            // Suppress unused warning for now.
            let _ = &mut descrambled;
        }
    }

    let stats = outer.stats();
    eprintln!(
        "msc-iq: frames={} cifs={} subch_bits={} subch_bytes={} ts_packets={} \
         rs_corrected={} rs_failed={}",
        result.frames_decoded,
        result.cifs_assembled,
        result.subch_bits_total,
        result.subch_bytes_total,
        result.ts_packets.len(),
        stats.rs_corrected,
        stats.rs_failed,
    );
    let _ = decoded_bytes_total;

    Ok(result)
}

/// Decode protection label like "EEP-3A" → eti-stuff `prot_level` encoding.
/// Profile A: bit 2 clear; Profile B: bit 2 set. Level 1..4 → low 2 bits 0..3.
fn decode_prot_level(label: &str) -> i16 {
    // Examples we see: "EEP-1A", "EEP-2A", "EEP-3A", "EEP-4A", "EEP-1B"..."EEP-4B".
    let bytes = label.as_bytes();
    if bytes.len() < 6 {
        return 2; // sensible default
    }
    // Find the digit (level) and the trailing letter (A/B).
    let level_char = bytes[4] as char;
    let profile_char = bytes[5] as char;
    let level_idx = match level_char {
        '1' => 0,
        '2' => 1,
        '3' => 2,
        '4' => 3,
        _ => 2,
    };
    let profile_bit = if profile_char == 'B' { 1 << 2 } else { 0 };
    profile_bit | level_idx as i16
}
