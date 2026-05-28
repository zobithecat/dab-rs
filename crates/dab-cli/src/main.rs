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
    /// Resample a `Cs16Le @ 3 MSPS` capture and quantize it to `CU8 @ 2.048
    /// MSPS` — the format eti-stuff's `eti-cmdline-rawfiles` expects.
    /// Lets the oracle binary process the same captures dab-rs handles
    /// natively, so the two pipelines can be cross-checked on identical
    /// resampled samples (the CU8 quantization adds ~−48 dB noise, which
    /// is well below any real chain-bug-sized divergence).
    ConvertIq {
        /// Input `Cs16Le @ 3 MSPS` capture.
        input: PathBuf,
        /// Output `CU8 @ 2.048 MSPS` file.
        output: PathBuf,
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
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse();
    match cli.command {
        Command::Fec { eti, subch } => run_fec(&eti, subch),
        Command::Fic { eti } => run_fic(&eti),
        Command::FicIq { iq, format, rate } => run_fic_iq(&iq, &format, rate),
        Command::ConvertIq { input, output } => run_convert_iq(&input, &output),
        Command::DiagIbits { iq, oracle, frames, ingest } => {
            run_diag_ibits(&iq, &oracle, frames, &ingest)
        }
        Command::DiagPair { iq, oracle, frame, symbol, show, ingest } => {
            let m = parse_ingest(&ingest)?;
            dab_cli::diag_ibits::dump_pair(&iq, m, &oracle, frame, symbol, show)
        }
    }
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

fn run_convert_iq(input: &std::path::Path, output: &std::path::Path) -> Result<()> {
    let pairs = dab_cli::convert_iq::convert_cs16_3m_to_cu8_2048k(input, output)?;
    let bytes = pairs * 2;
    let mb = bytes as f64 / (1024.0 * 1024.0);
    println!(
        "wrote {} CU8 sample pairs ({} bytes ≈ {:.1} MiB) to {}",
        pairs,
        bytes,
        mb,
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
