//! ETI(NI, G.703) frame parser per ETSI EN 300 799.
//!
//! Each frame is 6144 bytes (= 24 ms @ 2.048 Mbit/s) with structure:
//!   SYNC (4)  ERR(1) + FSYNC(3, alternates 07-3A-B6 / F8-C5-49)
//!   FC   (4)  FCT[8] FICF[1] NST[7] FP[3] MID[2] FL[11]
//!   STC  (4*NST) per stream: SCID[6] SAD[10] TPL[6] STL[10]
//!   EOH  (4)  MNSC[16] CRC[16]
//!   MST  (variable) FIC + sub-channel CIFs
//!   EOF  (4)  CRC[16] RFU[16]
//!   TIST (4)
//!   + padding to 6144

pub const FRAME_SIZE: usize = 6144;
pub const FSYNC_EVEN: [u8; 3] = [0x07, 0x3a, 0xb6];
pub const FSYNC_ODD: [u8; 3] = [0xf8, 0xc5, 0x49];
pub const FIB_SIZE: usize = 32;
pub const FIC_FIBS_MODE_I: usize = 4;

/// Per-stream descriptor from the STC section of an ETI frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StreamChar {
    /// Sub-channel id (0..63)
    pub scid: u8,
    /// Start address in CIF (0..863)
    pub sad: u16,
    /// Protection level encoding
    pub tpl: u8,
    /// Stream length in 64-bit groups
    pub stl: u16,
}

impl StreamChar {
    /// Length of this sub-channel's data in bytes (stl * 8).
    pub fn length_bytes(&self) -> usize {
        self.stl as usize * 8
    }
}

/// A parsed ETI(NI) frame.
#[derive(Debug, Clone)]
pub struct Frame {
    /// ERR byte (0xFF = no error in ETI-NI)
    pub err: u8,
    /// Frame counter (0..249)
    pub fct: u8,
    /// True when FICF bit is set
    pub fic_present: bool,
    /// Number of streams in MST
    pub nst: u8,
    /// Frame phase
    pub fp: u8,
    /// Mode ID (1=TM-I, 2=TM-II, 3=TM-III, 0=TM-IV)
    pub mid: u8,
    /// Frame length in 32-bit words
    pub fl: u16,
    /// Stream descriptors
    pub streams: Vec<StreamChar>,
    /// Raw FIC bytes (includes CRCs); empty when FICF=0 or mid==3
    pub fic: Vec<u8>,
    /// Concatenated sub-channel data bytes
    pub msc: Vec<u8>,
}

/// Errors from ETI frame parsing.
#[derive(thiserror::Error, Debug)]
pub enum EtiError {
    #[error("short frame: {0} bytes")]
    ShortFrame(usize),
    #[error("bad fsync: {0:06x}")]
    BadFsync(u32),
}

/// Parse a single 6144-byte ETI(NI) frame buffer.
pub fn parse_frame(buf: &[u8]) -> Result<Frame, EtiError> {
    if buf.len() < FRAME_SIZE {
        return Err(EtiError::ShortFrame(buf.len()));
    }

    let fsync = [buf[1], buf[2], buf[3]];
    if fsync != FSYNC_EVEN && fsync != FSYNC_ODD {
        let fsync_val = ((buf[1] as u32) << 16) | ((buf[2] as u32) << 8) | buf[3] as u32;
        return Err(EtiError::BadFsync(fsync_val));
    }

    let err = buf[0];

    // FC: bytes 4-7
    let fct = buf[4];
    let ficf = (buf[5] >> 7) & 1;
    let nst = buf[5] & 0x7f;
    let fp = (buf[6] >> 5) & 0x07;
    let mid = (buf[6] >> 3) & 0x03;
    let fl = (((buf[6] & 0x07) as u16) << 8) | buf[7] as u16;

    // STC words: 4 bytes each, big-endian
    let mut streams = Vec::with_capacity(nst as usize);
    let mut off = 8usize;
    for _ in 0..nst {
        let w = ((buf[off] as u32) << 24)
            | ((buf[off + 1] as u32) << 16)
            | ((buf[off + 2] as u32) << 8)
            | buf[off + 3] as u32;
        let scid = ((w >> 26) & 0x3f) as u8;
        let sad = ((w >> 16) & 0x3ff) as u16;
        let tpl = ((w >> 10) & 0x3f) as u8;
        let stl = (w & 0x3ff) as u16;
        streams.push(StreamChar { scid, sad, tpl, stl });
        off += 4;
    }

    // EOH (4 bytes) — skip
    off += 4;

    // MST: FIC (if ficf && mid != 3), then sub-channel data
    let fic_len = if ficf != 0 && mid != 3 {
        FIB_SIZE * FIC_FIBS_MODE_I
    } else {
        0
    };
    let fic = if fic_len > 0 {
        buf[off..off + fic_len].to_vec()
    } else {
        Vec::new()
    };
    off += fic_len;

    let msc_total: usize = streams.iter().map(|s| s.length_bytes()).sum();
    let msc = buf[off..off + msc_total].to_vec();

    Ok(Frame {
        err,
        fct,
        fic_present: ficf != 0,
        nst,
        fp,
        mid,
        fl,
        streams,
        fic,
        msc,
    })
}

/// Find the next FSYNC position at or after `start` within `buf`.
/// Returns the byte index of the ERR byte (i.e. fsync starts at index+1),
/// or `None` if not found.
fn find_sync(buf: &[u8], start: usize) -> Option<usize> {
    // Need at least 4 bytes: ERR + FSYNC(3)
    if buf.len() < 4 {
        return None;
    }
    let n = buf.len() - 3; // last valid start of a 3-byte fsync is at index n-1
    let mut i = start;
    while i < n {
        let s = [buf[i + 1], buf[i + 2], buf[i + 3]];
        if s == FSYNC_EVEN || s == FSYNC_ODD {
            return Some(i);
        }
        i += 1;
    }
    None
}

/// Iterator over frames in a byte slice. On a parse failure, resyncs to the
/// next FSYNC and yields `Err` for the failed attempt before continuing.
pub struct FrameReader<'a> {
    data: &'a [u8],
}

impl<'a> FrameReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { data }
    }
}

impl<'a> Iterator for FrameReader<'a> {
    type Item = Result<Frame, EtiError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.data.len() < FRAME_SIZE {
                return None;
            }
            match parse_frame(&self.data[..FRAME_SIZE]) {
                Ok(frame) => {
                    self.data = &self.data[FRAME_SIZE..];
                    return Some(Ok(frame));
                }
                Err(e) => {
                    // Attempt to resync: scan from offset 1 for next FSYNC
                    match find_sync(self.data, 1) {
                        Some(pos) => {
                            self.data = &self.data[pos..];
                            // Yield the error for the failed attempt
                            return Some(Err(e));
                        }
                        None => {
                            // No sync found; keep last 3 bytes for partial match
                            let keep = if self.data.len() > 3 { self.data.len() - 3 } else { 0 };
                            self.data = &self.data[keep..];
                            return Some(Err(e));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid 6144-byte ETI frame with FSYNC_EVEN and one stream.
    fn make_frame(err: u8, fct: u8, scid: u8, sad: u16, tpl: u8, stl: u16) -> Vec<u8> {
        let mut buf = vec![0u8; FRAME_SIZE];
        // SYNC: ERR + FSYNC_EVEN
        buf[0] = err;
        buf[1] = FSYNC_EVEN[0];
        buf[2] = FSYNC_EVEN[1];
        buf[3] = FSYNC_EVEN[2];
        // FC
        buf[4] = fct;
        // FICF=1, NST=1
        buf[5] = (1 << 7) | 1u8;
        // FP=0, MID=1 (TM-I), FL_hi=0
        buf[6] = (0u8 << 5) | (1u8 << 3) | 0u8;
        buf[7] = 0; // FL_lo
        // STC word for stream 0
        let w: u32 = ((scid as u32 & 0x3f) << 26)
            | ((sad as u32 & 0x3ff) << 16)
            | ((tpl as u32 & 0x3f) << 10)
            | (stl as u32 & 0x3ff);
        buf[8] = (w >> 24) as u8;
        buf[9] = (w >> 16) as u8;
        buf[10] = (w >> 8) as u8;
        buf[11] = w as u8;
        // EOH at offset 12 (4 bytes, zeroed)
        // MST starts at offset 16
        // FIC: 4 FIBs * 32 bytes = 128 bytes (zeroed)
        // MSC: stl * 8 bytes (zeroed)
        // Write a recognisable pattern into MSC section
        let msc_off = 16 + FIB_SIZE * FIC_FIBS_MODE_I;
        let msc_len = stl as usize * 8;
        for i in 0..msc_len {
            buf[msc_off + i] = (i & 0xff) as u8;
        }
        buf
    }

    #[test]
    fn test_parse_frame_basic() {
        let stl: u16 = 6; // 6 * 8 = 48 bytes MSC
        let buf = make_frame(0xFF, 42, 1, 100, 0x0c, stl);
        let frame = parse_frame(&buf).expect("parse should succeed");

        assert_eq!(frame.err, 0xFF);
        assert_eq!(frame.fct, 42);
        assert!(frame.fic_present);
        assert_eq!(frame.nst, 1);
        assert_eq!(frame.mid, 1);
        assert_eq!(frame.streams.len(), 1);

        let s = &frame.streams[0];
        assert_eq!(s.scid, 1);
        assert_eq!(s.sad, 100);
        assert_eq!(s.tpl, 0x0c);
        assert_eq!(s.stl, stl);
        assert_eq!(s.length_bytes(), 48);

        assert_eq!(frame.fic.len(), FIB_SIZE * FIC_FIBS_MODE_I);
        assert_eq!(frame.msc.len(), 48);

        // Check MSC content matches what we wrote
        for i in 0..48usize {
            assert_eq!(frame.msc[i], (i & 0xff) as u8);
        }
    }

    #[test]
    fn test_parse_frame_bad_fsync() {
        let mut buf = make_frame(0xFF, 0, 1, 0, 0, 2);
        // Corrupt FSYNC
        buf[1] = 0xDE;
        buf[2] = 0xAD;
        buf[3] = 0xBE;
        let result = parse_frame(&buf);
        assert!(matches!(result, Err(EtiError::BadFsync(_))));
    }

    #[test]
    fn test_parse_frame_short() {
        let buf = vec![0u8; 100];
        let result = parse_frame(&buf);
        assert!(matches!(result, Err(EtiError::ShortFrame(100))));
    }

    #[test]
    fn test_frame_reader_resync() {
        // Build: 100 bytes of garbage, then a valid frame, then a valid frame
        let mut data = vec![0xAA_u8; 100];
        let valid = make_frame(0xFF, 7, 2, 50, 0, 4);
        data.extend_from_slice(&valid);
        data.extend_from_slice(&valid);

        let frames: Vec<_> = FrameReader::new(&data).collect();
        // The first 100 bytes cause sync errors; eventually we find valid frames.
        // Count successful frames
        let ok_count = frames.iter().filter(|r| r.is_ok()).count();
        assert!(ok_count >= 1, "expected at least one valid frame after resync");

        // The last successful frames should have fct=7
        let last_ok = frames.iter().filter(|r| r.is_ok()).last().unwrap().as_ref().unwrap();
        assert_eq!(last_ok.fct, 7);
    }

    #[test]
    fn test_frame_reader_consecutive() {
        // Two back-to-back valid frames, no garbage
        let f1 = make_frame(0xFF, 10, 1, 0, 0, 2);
        let f2 = make_frame(0xFE, 11, 3, 0, 0, 3);
        let mut data = Vec::new();
        data.extend_from_slice(&f1);
        data.extend_from_slice(&f2);

        let frames: Vec<_> = FrameReader::new(&data)
            .filter_map(|r| r.ok())
            .collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].fct, 10);
        assert_eq!(frames[1].fct, 11);
    }

    #[test]
    fn test_stream_char_length_bytes() {
        let s = StreamChar { scid: 0, sad: 0, tpl: 0, stl: 10 };
        assert_eq!(s.length_bytes(), 80);
    }
}
