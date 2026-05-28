//! dab-rs command-line front-end.
//!
//! - `dab fec <eti> --subch N`: T-DMB outer FEC on a recorded ETI(NI) capture.
//! - `dab fic <eti>`:           FIC ensemble decode on a recorded ETI capture.
//! - `dab fic-iq <iq>`:         FIC ensemble decode from raw I/Q via the full
//!                              OFDM chain (Stages 1–7 → Viterbi → descramble).

use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};

use dab_iq::IqFormat;

#[derive(Parser)]
#[command(name = "dab", version, about = "dab-rs: memory-safe DAB / T-DMB receiver")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the T-DMB outer FEC (sync-aligned Forney deinterleaver + RS) on an
    /// ETI(NI) capture sub-channel and print decode statistics.
    Fec {
        /// Path to an ETI(NI) capture (6144-byte frames).
        eti: PathBuf,
        /// MSC sub-channel id to extract (default: 1).
        #[arg(long, default_value_t = 1)]
        subch: u8,
    },
    /// Decode the FIC of an ETI(NI) capture and print the ensemble
    /// configuration (EId, label, sub-channels, services).
    Fic {
        /// Path to an ETI(NI) capture (6144-byte frames).
        eti: PathBuf,
    },
    /// Decode the FIC from a raw I/Q capture through the full OFDM chain
    /// (Stages 1–7 → FIC Viterbi → descramble → FIBs → Ensemble).
    FicIq {
        /// Path to a raw I/Q capture (default format: Cs16Le @ 3 MSPS).
        iq: PathBuf,
        /// On-disk sample format.
        #[arg(long, default_value = "cs16le")]
        format: String,
        /// On-disk sample rate (Hz).
        #[arg(long, default_value_t = 3_000_000)]
        rate: u32,
    },
    /// Resample a `Cs16Le @ 3 MSPS` capture to 2.048 MSPS and emit it in a
    /// format an eti-stuff offline input handler can consume.
    ///
    /// `--out cu8`    → `(uint8 I, uint8 Q)` for `eti-cmdline-rawfiles`.
    /// `--out wav`    → 16-bit PCM stereo WAV for `eti-cmdline-wavfiles`
    ///                  (still 16× too small in amplitude vs the live
    ///                  airspy-handler path — empirically breaks the
    ///                  oracle's coarse-CFO lock).
    /// `--out wav32`  → 32-bit IEEE float stereo WAV for
    ///                  `eti-cmdline-wavfiles`, samples pre-scaled by
    ///                  ×16 so the OFDM processor sees the exact same
    ///                  amplitude the live oracle saw — the path that
    ///                  matches the live `eti-cmdline-airspy` numerics
    ///                  bit-for-bit.
    ConvertIq {
        /// Input `Cs16Le @ 3 MSPS` capture.
        input: PathBuf,
        /// Output file.
        output: PathBuf,
        /// `cu8`, `wav` (16-bit PCM), or `wav32` (32-bit float,
        /// airspy-scaled). Default: `wav32`.
        #[arg(long, default_value = "wav32")]
        out: String,
    },
    /// Cross-validate dab-rs's per-symbol soft bits against an oracle dump
    /// produced by the patched `eti-cmdline-rawfiles` (env var
    /// `DAB_RS_DIAG_DUMP` set to a writable path).
    DiagIbits {
        /// Path to the I/Q capture (Cs16Le @ 3 MSPS by default; CU8 @
        /// 2.048 MSPS with `--ingest cu8` — use the same CU8 file the
        /// oracle consumed to make the inputs bit-identical).
        iq: PathBuf,
        /// Path to the oracle's binary dump.
        oracle: PathBuf,
        /// Maximum number of frames to capture on the dab-rs side.
        #[arg(long, default_value_t = 30)]
        frames: usize,
        /// `cs16le` (native, resampled) or `cu8` (read the same CU8 file
        /// the oracle saw — eliminates quantisation as a variable).
        #[arg(long, default_value = "cs16le")]
        ingest: String,
    },
    /// Dump one (frame, symbol) record side-by-side from dab-rs and the
    /// oracle, with permutation/sign-flip/bin-shift diagnostics. Useful
    /// for inspecting why `diag-ibits` reports a low match rate.
    DiagPair {
        iq: PathBuf,
        oracle: PathBuf,
        #[arg(long, default_value_t = 5)]
        frame: u32,
        #[arg(long, default_value_t = 5)]
        symbol: u32,
        #[arg(long, default_value_t = 12)]
        show: usize,
        #[arg(long, default_value = "cu8")]
        ingest: String,
    },
    /// Slice-8 standalone Viterbi harness. Reads 2304 signed bytes
    /// (one i8 soft bit each) from stdin, runs FIC depuncture +
    /// scalar Viterbi via `dab_viterbi::FicProtection`, writes 768
    /// hard bits (one bit per byte, value 0 or 1) to stdout.
    ///
    /// Same I/O protocol as `docs/diag/viterbi_spiral_cli.cpp` so a
    /// synthetic test vector can be piped to both binaries and the
    /// outputs bit-XOR'd by `docs/diag/viterbi_unit_diff.py`.
    ViterbiCli,
    /// Slice-10 synthetic OFDM round-trip harness. Reads 3 × 2048
    /// `Complex<f32>` differential spectra from stdin (49 152 bytes
    /// total — three 2048-bin natural-order FFT outputs as little-
    /// endian `re, im` f32 pairs), runs the *actual* dab-rs chain:
    ///
    ///   dqpsk_demap.demap(diff)               × 3 symbols
    ///   → 9216 i16 soft bits
    ///   → 4 × FicProtection::deconvolve       per ficBlock
    ///   → 3072 info bits
    ///   → descramble + MSB-first pack         → 384 bytes
    ///
    /// Writes 384 bytes (12 FIBs) to stdout. Python harness on the
    /// other side (`docs/diag/synth_ofdm.py`) generates the synthetic
    /// spectra under a configurable
    /// `(p1=interleaver_dir, p2=iq_layout, p3=conj_dir)` triple and
    /// checks the recovered FIB CRC.
    SynthTest,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse();
    match cli.command {
        Command::Fec { eti, subch } => run_fec(&eti, subch),
        Command::Fic { eti } => run_fic(&eti),
        Command::FicIq { iq, format, rate } => run_fic_iq(&iq, &format, rate),
        Command::ConvertIq { input, output, out } => run_convert_iq(&input, &output, &out),
        Command::DiagIbits { iq, oracle, frames, ingest } => {
            run_diag_ibits(&iq, &oracle, frames, &ingest)
        }
        Command::DiagPair { iq, oracle, frame, symbol, show, ingest } => {
            let m = parse_ingest(&ingest)?;
            dab_cli::diag_ibits::dump_pair(&iq, m, &oracle, frame, symbol, show)
        }
        Command::ViterbiCli => run_viterbi_cli(),
        Command::SynthTest => run_synth_test(),
    }
}

fn run_synth_test() -> Result<()> {
    use std::io::{Read, Write};
    use num_complex::Complex;
    const T_U: usize = 2048;
    const N_SYMS: usize = 3;
    const SPEC_BYTES: usize = T_U * 8; // 2 × f32 LE per bin
    let mut buf = vec![0_u8; N_SYMS * SPEC_BYTES];
    std::io::stdin()
        .read_exact(&mut buf)
        .map_err(|e| anyhow::anyhow!("stdin: expected {} bytes ({e})", N_SYMS * SPEC_BYTES))?;

    let demap = dab_ofdm::DqpskDemap::mode_i();
    let mut frame_soft: Vec<i16> = Vec::with_capacity(9216);
    for s in 0..N_SYMS {
        let mut spec: Vec<Complex<f32>> = Vec::with_capacity(T_U);
        let off = s * SPEC_BYTES;
        for k in 0..T_U {
            let re = f32::from_le_bytes(buf[off + k * 8..off + k * 8 + 4].try_into().unwrap());
            let im =
                f32::from_le_bytes(buf[off + k * 8 + 4..off + k * 8 + 8].try_into().unwrap());
            spec.push(Complex::new(re, im));
        }
        let bits = demap.demap(&spec);
        frame_soft.extend_from_slice(&bits);
    }
    assert_eq!(frame_soft.len(), 9216);
    let frame_bytes = dab_cli::fic_iq::decode_fic_soft_bits_to_bytes(&frame_soft);
    assert_eq!(frame_bytes.len(), 384);
    std::io::stdout()
        .write_all(&frame_bytes)
        .map_err(|e| anyhow::anyhow!("stdout: {e}"))?;
    Ok(())
}

fn run_viterbi_cli() -> Result<()> {
    use std::io::{Read, Write};
    const FIC_IN: usize = 2304;
    const FIC_OUT: usize = 768;
    let mut buf = [0_u8; FIC_IN];
    std::io::stdin()
        .read_exact(&mut buf)
        .map_err(|e| anyhow::anyhow!("stdin: expected {FIC_IN} bytes ({e})"))?;
    // Reinterpret bytes as i8 soft bits, widen to i16 for the decoder.
    let soft: Vec<i16> = buf.iter().map(|&b| (b as i8) as i16).collect();
    let mut fic = dab_viterbi::FicProtection::new();
    let bits = fic.deconvolve(&soft);
    assert_eq!(bits.len(), FIC_OUT);
    std::io::stdout()
        .write_all(&bits)
        .map_err(|e| anyhow::anyhow!("stdout: {e}"))?;
    Ok(())
}

fn parse_ingest(s: &str) -> Result<dab_cli::diag_ibits::IqIngest> {
    Ok(match s.to_lowercase().as_str() {
        "cs16le" => dab_cli::diag_ibits::IqIngest::Cs16Le3MSPS,
        "cu8" => dab_cli::diag_ibits::IqIngest::Cu82048k,
        other => anyhow::bail!("unsupported --ingest {other} (use cs16le or cu8)"),
    })
}

fn run_diag_ibits(
    iq: &std::path::Path,
    oracle: &std::path::Path,
    frames: usize,
    ingest: &str,
) -> Result<()> {
    let ingest_mode = match ingest.to_lowercase().as_str() {
        "cs16le" => dab_cli::diag_ibits::IqIngest::Cs16Le3MSPS,
        "cu8" => dab_cli::diag_ibits::IqIngest::Cu82048k,
        other => anyhow::bail!("unsupported --ingest {other} (use cs16le or cu8)"),
    };
    let res = dab_cli::diag_ibits::run_diag(iq, ingest_mode, oracle, frames)?;
    dab_cli::diag_ibits::print_report(&res);
    Ok(())
}

fn run_convert_iq(
    input: &std::path::Path,
    output: &std::path::Path,
    out_fmt: &str,
) -> Result<()> {
    let (pairs, bytes_per_pair, label) = match out_fmt.to_lowercase().as_str() {
        "cu8" => {
            let p = dab_cli::convert_iq::convert_cs16_3m_to_cu8_2048k(input, output)?;
            (p, 2_usize, "CU8")
        }
        "wav" => {
            let p = dab_cli::convert_iq::convert_cs16_3m_to_wav_2048k(input, output)?;
            (p, 4_usize, "WAV 16-bit PCM stereo")
        }
        "wav32" => {
            let p = dab_cli::convert_iq::convert_cs16_3m_to_wav32_2048k(input, output)?;
            (p, 8_usize, "WAV 32-bit float stereo (airspy-scaled)")
        }
        other => anyhow::bail!("unsupported --out {other} (use cu8, wav, or wav32)"),
    };
    let bytes = pairs * bytes_per_pair;
    let mb = bytes as f64 / (1024.0 * 1024.0);
    println!(
        "wrote {} sample pairs ({} bytes ≈ {:.1} MiB, {}) to {}",
        pairs,
        bytes,
        mb,
        label,
        output.display(),
    );
    Ok(())
}

fn run_fic_iq(iq: &std::path::Path, format: &str, rate: u32) -> Result<()> {
    let fmt = match format.to_lowercase().as_str() {
        "cs16le" => IqFormat::Cs16Le,
        "cs8" => IqFormat::Cs8,
        "cf32le" => IqFormat::Cf32Le,
        other => anyhow::bail!("unsupported I/Q format: {other} (use cs16le / cs8 / cf32le)"),
    };
    let res = dab_cli::fic_iq::process_iq_to_fic(iq, fmt, rate)?;

    println!(
        "resampled={} nulls={} frames_decoded={} frames_skipped={} best_band_ratio={:.1}dB",
        res.resampled_samples,
        res.null_dips,
        res.frames_decoded,
        res.frames_skipped,
        res.best_band_ratio_db,
    );
    println!("fib_ok={}/{}", res.fib_ok, res.fib_total);

    let ens = &res.ensemble;
    match ens.eid {
        Some(eid) => println!("EId=0x{eid:04X}"),
        None => println!("EId=(none)"),
    }
    println!("label={:?}", ens.label);
    println!("sub-channels ({}):", ens.sub_channels.len());
    for sc in ens.sub_channels.values() {
        println!(
            "  sub {:>2}: start={:>3} size={:>3}cu {} {}kbps {}",
            sc.sub_ch_id,
            sc.start_addr,
            sc.size_cu,
            sc.protection,
            sc.bitrate_kbps,
            if sc.is_long_form { "long" } else { "short" },
        );
    }
    println!("services ({}):", ens.services.len());
    for svc in ens.services.values() {
        println!(
            "  SId=0x{:08X} {} label={:?}",
            svc.sid,
            if svc.is_data { "data" } else { "prog" },
            svc.label,
        );
        for c in &svc.components {
            let sub = match c.sub_ch_id {
                Some(s) => format!("sub {s}"),
                None => format!("SCIdS {}", c.sc_id_s),
            };
            println!(
                "    [{}] {} ty={}{}",
                c.transport,
                sub,
                c.ascty_or_dscty,
                if c.is_primary { " primary" } else { "" },
            );
        }
    }
    Ok(())
}

fn run_fic(eti: &std::path::Path) -> Result<()> {
    let bytes = std::fs::read(eti)?;
    let mut acc = dab_fic::FicAccumulator::new();
    for frame in dab_eti::FrameReader::new(&bytes) {
        let frame = match frame {
            Ok(f) => f,
            Err(_) => continue,
        };
        if !frame.fic.is_empty() {
            acc.feed_fic(&frame.fic);
        }
    }

    println!("fib_ok={}/{}", acc.fib_ok, acc.fib_total);
    let ens = &acc.ensemble;
    match ens.eid {
        Some(eid) => println!("EId=0x{eid:04X}"),
        None => println!("EId=(none)"),
    }
    println!("label={:?}", ens.label);

    println!("sub-channels ({}):", ens.sub_channels.len());
    for sc in ens.sub_channels.values() {
        println!(
            "  sub {:>2}: start={:>3} size={:>3}cu {} {}kbps {}",
            sc.sub_ch_id,
            sc.start_addr,
            sc.size_cu,
            sc.protection,
            sc.bitrate_kbps,
            if sc.is_long_form { "long" } else { "short" },
        );
    }

    println!("services ({}):", ens.services.len());
    for svc in ens.services.values() {
        println!(
            "  SId=0x{:08X} {} label={:?}",
            svc.sid,
            if svc.is_data { "data" } else { "prog" },
            svc.label,
        );
        for c in &svc.components {
            let sub = match c.sub_ch_id {
                Some(s) => format!("sub {s}"),
                None => format!("SCIdS {}", c.sc_id_s),
            };
            println!(
                "    [{}] {} ty={}{}",
                c.transport,
                sub,
                c.ascty_or_dscty,
                if c.is_primary { " primary" } else { "" },
            );
        }
    }
    Ok(())
}

fn run_fec(eti: &std::path::Path, subch: u8) -> Result<()> {
    let bytes = std::fs::read(eti)?;
    let mut fec = dab_fec::KoreanTDmbOuterFec::new();
    let mut packets = 0usize;
    let mut ok = 0usize;
    for frame in dab_eti::FrameReader::new(&bytes) {
        let frame = match frame {
            Ok(f) => f,
            Err(_) => continue,
        };
        if let Some(chunk) = dab_msc::extract_subchannel(&frame, subch) {
            for ts in fec.feed(chunk) {
                packets += 1;
                if ts.rs_errors >= 0 {
                    ok += 1;
                }
            }
        }
    }
    let stats = fec.stats();
    println!("aligned={} phase_offset={}", stats.aligned, stats.phase_offset);
    println!("rs_total={} rs_ok+corrected={} rs_failed={}", stats.rs_total, ok, stats.rs_failed);
    if packets > 0 {
        println!("success_rate={:.1}%", ok as f64 * 100.0 / packets as f64);
    }
    Ok(())
}
