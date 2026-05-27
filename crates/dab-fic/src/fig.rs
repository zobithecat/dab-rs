//! FIB -> FIG splitting.
//!
//! Mode I: 4 FIBs of 32 bytes per ETI frame. Each valid FIB carries a
//! sequence of FIGs terminated by 0xFF. The FIG header byte is
//! `TYPE[3] | LEN[5]` where LEN excludes the header (ETSI EN 300 401 §5.2).

/// A single Fast Information Group: its 3-bit type and payload bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Fig {
    /// FIG type (0..7) from the upper 3 bits of the header byte.
    pub ftype: u8,
    /// FIG payload (length taken from the lower 5 bits of the header byte).
    pub data: Vec<u8>,
}

/// Split a single 30-byte FIB payload into its FIGs, stopping at the `0xFF`
/// end marker or on a malformed length.
///
/// The caller is responsible for validating the FIB CRC before calling this.
pub fn iter_figs(payload: &[u8]) -> Vec<Fig> {
    let mut figs = Vec::new();
    let mut i = 0usize;
    while i < payload.len() {
        let hdr = payload[i];
        if hdr == 0xFF {
            break; // end marker
        }
        let ftype = (hdr >> 5) & 0x07;
        let flen = (hdr & 0x1F) as usize;
        i += 1;
        if i + flen > payload.len() {
            break; // malformed
        }
        figs.push(Fig {
            ftype,
            data: payload[i..i + flen].to_vec(),
        });
        i += flen;
    }
    figs
}
