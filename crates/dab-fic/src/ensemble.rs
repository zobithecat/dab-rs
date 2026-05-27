//! Ensemble model accumulated across many ETI frames.
//!
//! Faithful port of the Python reference `tdmb/eti/fic.py`: FIG type 0
//! extensions 0/1/2 (EId, sub-channel organization, service organization) and
//! FIG type 1 extensions 0/1/5 (labels). See ETSI EN 300 401 §6.

use std::collections::BTreeMap;

use crate::crc::fib_ok;
use crate::fig::{iter_figs, Fig};
use crate::tables::{eep_size, uep_table};

/// Sub-channel description from FIG 0/1 (ETSI EN 300 401 §6.2.1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SubChannel {
    /// Sub-channel identifier (0..63).
    pub sub_ch_id: u8,
    /// CU start address in the CIF.
    pub start_addr: u16,
    /// Capacity units occupied.
    pub size_cu: u16,
    /// Human-readable protection label, e.g. "EEP-3A", "UEP-3".
    pub protection: String,
    /// Bit rate in kbit/s.
    pub bitrate_kbps: u16,
    /// True for long-form (EEP) coding, false for short-form (UEP).
    pub is_long_form: bool,
}

/// Service component from FIG 0/2 (ETSI EN 300 401 §6.3.1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceComponent {
    /// Sub-channel id for stream-mode components; `None` for packet mode.
    pub sub_ch_id: Option<u8>,
    /// Service component id within the service (packet mode SCIdS).
    pub sc_id_s: u16,
    /// Transport: "stream-audio", "stream-data", "fidc", or "packet".
    pub transport: String,
    /// Audio service type (ASCTy) or data service type (DSCTy).
    pub ascty_or_dscty: u8,
    /// True if this is the primary component of the service.
    pub is_primary: bool,
    /// Component label (currently unused; reserved for FIG 1/4).
    pub label: String,
}

/// Service from FIG 0/2 and FIG 1 labels (ETSI EN 300 401 §6.3).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Service {
    /// Service identifier.
    pub sid: u32,
    /// False = programme service, true = data service.
    pub is_data: bool,
    /// Service label.
    pub label: String,
    /// Service short label (currently unused; reserved for char-flag decode).
    pub short_label: String,
    /// Service components.
    pub components: Vec<ServiceComponent>,
}

impl Service {
    fn new(sid: u32, is_data: bool) -> Self {
        Service {
            sid,
            is_data,
            label: String::new(),
            short_label: String::new(),
            components: Vec::new(),
        }
    }
}

/// Ensemble state accumulated from the FIC.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Ensemble {
    /// Ensemble identifier from FIG 0/0.
    pub eid: Option<u16>,
    /// Ensemble label from FIG 1/0.
    pub label: String,
    /// Ensemble short label (reserved).
    pub short_label: String,
    /// Sub-channels keyed by id (sorted for deterministic iteration).
    pub sub_channels: BTreeMap<u8, SubChannel>,
    /// Services keyed by SId (sorted for deterministic iteration).
    pub services: BTreeMap<u32, Service>,
}

/// Decode a label field: best-effort UTF-8, else latin-1, then trim trailing
/// spaces and NULs. Mirrors `_decode_label` in the Python reference.
pub fn decode_label(payload: &[u8]) -> String {
    let raw = &payload[..payload.len().min(16)];
    let s = match std::str::from_utf8(raw) {
        Ok(s) => s.to_string(),
        // latin-1: every byte maps directly to the matching U+00xx code point.
        Err(_) => raw.iter().map(|&b| b as char).collect(),
    };
    s.trim_end_matches([' ', '\u{0}']).to_string()
}

fn handle_fig0_0(ens: &mut Ensemble, payload: &[u8]) {
    if payload.len() < 4 {
        return;
    }
    let eid = ((payload[1] as u16) << 8) | payload[2] as u16;
    ens.eid = Some(eid);
}

fn handle_fig0_1(ens: &mut Ensemble, payload: &[u8]) {
    let mut i = 1usize; // skip header byte
    while i + 2 <= payload.len() {
        let b0 = payload[i];
        let b1 = payload[i + 1];
        let sub_ch_id = (b0 >> 2) & 0x3F;
        let start_addr = (((b0 & 0x03) as u16) << 8) | b1 as u16;
        i += 2;
        if i >= payload.len() {
            break;
        }
        let b2 = payload[i];
        let long_form = (b2 >> 7) & 1;
        if long_form != 0 {
            if i + 2 > payload.len() {
                break;
            }
            let option = (b2 >> 4) & 0x07;
            let level = (b2 >> 2) & 0x03;
            let sub_size = (((b2 & 0x03) as u16) << 8) | payload[i + 1] as u16;
            i += 2;
            let (br, cu, lbl) = eep_size(option, level, sub_size);
            ens.sub_channels.insert(
                sub_ch_id,
                SubChannel {
                    sub_ch_id,
                    start_addr,
                    size_cu: if cu != 0 { cu } else { sub_size },
                    protection: lbl,
                    bitrate_kbps: br,
                    is_long_form: true,
                },
            );
        } else {
            let table_idx = b2 & 0x3F;
            i += 1;
            let (br, cu, lbl) = uep_table(table_idx)
                .map(|(b, c, l)| (b, c, l.to_string()))
                .unwrap_or((0, 0, format!("UEP?{table_idx}")));
            ens.sub_channels.insert(
                sub_ch_id,
                SubChannel {
                    sub_ch_id,
                    start_addr,
                    size_cu: cu,
                    protection: lbl,
                    bitrate_kbps: br,
                    is_long_form: false,
                },
            );
        }
    }
}

/// Idempotent insert of a service component on `(sub_ch_id, sc_id_s, transport)`.
fn add_component(svc: &mut Service, comp: ServiceComponent) {
    for c in &svc.components {
        if c.sub_ch_id == comp.sub_ch_id
            && c.sc_id_s == comp.sc_id_s
            && c.transport == comp.transport
        {
            return;
        }
    }
    svc.components.push(comp);
}

fn handle_fig0_2(ens: &mut Ensemble, payload: &[u8], p_d: u8) {
    let mut i = 1usize;
    while i < payload.len() {
        let sid: u32;
        if p_d == 0 {
            if i + 2 > payload.len() {
                return;
            }
            sid = ((payload[i] as u32) << 8) | payload[i + 1] as u32;
            i += 2;
        } else {
            if i + 4 > payload.len() {
                return;
            }
            sid = ((payload[i] as u32) << 24)
                | ((payload[i + 1] as u32) << 16)
                | ((payload[i + 2] as u32) << 8)
                | payload[i + 3] as u32;
            i += 4;
        }
        if i >= payload.len() {
            return;
        }
        let b = payload[i];
        i += 1;
        let ncomp = (b & 0x0F) as usize;
        let is_data = p_d == 1;
        let svc = ens
            .services
            .entry(sid)
            .or_insert_with(|| Service::new(sid, is_data));

        for _ in 0..ncomp {
            if i + 2 > payload.len() {
                return;
            }
            let w = ((payload[i] as u16) << 8) | payload[i + 1] as u16;
            i += 2;
            let tmid = (w >> 14) & 0x03;
            match tmid {
                0 => {
                    let ascty = ((w >> 8) & 0x3F) as u8;
                    let sub_ch_id = ((w >> 2) & 0x3F) as u8;
                    let primary = (w >> 1) & 1;
                    add_component(
                        svc,
                        ServiceComponent {
                            sub_ch_id: Some(sub_ch_id),
                            sc_id_s: 0,
                            transport: "stream-audio".to_string(),
                            ascty_or_dscty: ascty,
                            is_primary: primary != 0,
                            label: String::new(),
                        },
                    );
                }
                1 => {
                    let dscty = ((w >> 8) & 0x3F) as u8;
                    let sub_ch_id = ((w >> 2) & 0x3F) as u8;
                    let primary = (w >> 1) & 1;
                    add_component(
                        svc,
                        ServiceComponent {
                            sub_ch_id: Some(sub_ch_id),
                            sc_id_s: 0,
                            transport: "stream-data".to_string(),
                            ascty_or_dscty: dscty,
                            is_primary: primary != 0,
                            label: String::new(),
                        },
                    );
                }
                2 => {
                    let dscty = ((w >> 8) & 0x3F) as u8;
                    let sub_ch_id = ((w >> 2) & 0x3F) as u8;
                    let primary = (w >> 1) & 1;
                    add_component(
                        svc,
                        ServiceComponent {
                            sub_ch_id: Some(sub_ch_id),
                            sc_id_s: 0,
                            transport: "fidc".to_string(),
                            ascty_or_dscty: dscty,
                            is_primary: primary != 0,
                            label: String::new(),
                        },
                    );
                }
                _ => {
                    let scid = (w >> 2) & 0x0FFF;
                    let primary = (w >> 1) & 1;
                    add_component(
                        svc,
                        ServiceComponent {
                            sub_ch_id: None,
                            sc_id_s: scid,
                            transport: "packet".to_string(),
                            ascty_or_dscty: 0,
                            is_primary: primary != 0,
                            label: String::new(),
                        },
                    );
                }
            }
        }
    }
}

fn handle_fig1(ens: &mut Ensemble, payload: &[u8]) {
    if payload.is_empty() {
        return;
    }
    let hdr = payload[0];
    let ext = hdr & 0x07;
    let body = &payload[1..];
    match ext {
        0 => {
            // Ensemble label: [EId hi][EId lo][16B label][2B mask]
            if body.len() < 2 + 16 + 2 {
                return;
            }
            ens.label = decode_label(&body[2..2 + 16]);
        }
        1 => {
            // Programme service label (16-bit SId)
            if body.len() < 2 + 16 + 2 {
                return;
            }
            let sid = (((body[0] as u32) << 8) | body[1] as u32) as u32;
            let label = decode_label(&body[2..2 + 16]);
            ens.services
                .entry(sid)
                .or_insert_with(|| Service::new(sid, false))
                .label = label;
        }
        5 => {
            // Data service label (32-bit SId)
            if body.len() < 4 + 16 + 2 {
                return;
            }
            let sid = ((body[0] as u32) << 24)
                | ((body[1] as u32) << 16)
                | ((body[2] as u32) << 8)
                | body[3] as u32;
            let label = decode_label(&body[4..4 + 16]);
            ens.services
                .entry(sid)
                .or_insert_with(|| Service::new(sid, true))
                .label = label;
        }
        _ => {}
    }
}

/// Drives FIG dispatch and accumulates ensemble state across ETI frames.
#[derive(Debug, Default)]
pub struct FicAccumulator {
    /// Accumulated ensemble state.
    pub ensemble: Ensemble,
    /// Total FIBs seen (one per 32-byte slice of fed FIC blocks).
    pub fib_total: usize,
    /// FIBs that passed the CRC check.
    pub fib_ok: usize,
}

impl FicAccumulator {
    /// Create an empty accumulator.
    pub fn new() -> Self {
        FicAccumulator::default()
    }

    /// Feed a FIC block (96 bytes / 4 FIBs in Mode I). Each 32-byte FIB
    /// increments `fib_total`; passing the CRC increments `fib_ok` and
    /// dispatches its FIGs into the ensemble model.
    pub fn feed_fic(&mut self, fic: &[u8]) {
        let mut k = 0usize;
        while k < fic.len() {
            self.fib_total += 1;
            let end = (k + 32).min(fic.len());
            let fib = &fic[k..end];
            k += 32;
            if fib.len() < 32 {
                continue;
            }
            self.feed_fib(fib);
        }
    }

    fn feed_fib(&mut self, fib: &[u8]) {
        if !fib_ok(fib) {
            return;
        }
        self.fib_ok += 1;
        for fig in iter_figs(&fib[..30]) {
            self.dispatch(&fig);
        }
    }

    fn dispatch(&mut self, fig: &Fig) {
        match fig.ftype {
            0 => {
                if fig.data.is_empty() {
                    return;
                }
                let p_d = (fig.data[0] >> 5) & 1;
                let ext = fig.data[0] & 0x1F;
                match ext {
                    0 => handle_fig0_0(&mut self.ensemble, &fig.data),
                    1 => handle_fig0_1(&mut self.ensemble, &fig.data),
                    2 => handle_fig0_2(&mut self.ensemble, &fig.data, p_d),
                    _ => {}
                }
            }
            1 => handle_fig1(&mut self.ensemble, &fig.data),
            _ => {}
        }
    }
}
