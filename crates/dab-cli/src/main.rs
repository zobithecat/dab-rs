//! dab-rs command-line front-end.
//!
//! Week 1 exposes the outer-FEC path: read an ETI(NI) capture, extract a
//! sub-channel, run the sync-aligned RS+TI pipeline, and report decode stats.

use std::path::PathBuf;

use anyhow::Result;
use clap::{Parser, Subcommand};

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
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse();
    match cli.command {
        Command::Fec { eti, subch } => run_fec(&eti, subch),
        Command::Fic { eti } => run_fic(&eti),
    }
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
