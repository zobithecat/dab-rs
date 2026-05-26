//! MSC sub-channel byte extractor.
//!
//! ETI MST holds the concatenated sub-channel data (per stream described in STC).
//! For each ETI frame we have one CIF per sub-channel. Given a sub-channel id
//! we slice out the right portion of the MST.

use dab_eti::Frame;

/// Return the bytes belonging to `sub_ch_id` within this frame's MSC, or None.
///
/// Only valid when `frame.err == 0xFF` (no error in ETI-NI). Walks streams in
/// order accumulating an offset into `frame.msc`, returns the slice for the
/// matching scid.
pub fn extract_subchannel(frame: &Frame, sub_ch_id: u8) -> Option<&[u8]> {
    if frame.err != 0xFF {
        return None;
    }
    let mut offset = 0usize;
    for s in &frame.streams {
        let length = s.length_bytes();
        if s.scid == sub_ch_id {
            return Some(&frame.msc[offset..offset + length]);
        }
        offset += length;
    }
    None
}

/// Return the bytes-per-CIF for `sub_ch_id` in this frame, or None if not found.
pub fn find_subchannel_size(frame: &Frame, sub_ch_id: u8) -> Option<usize> {
    for s in &frame.streams {
        if s.scid == sub_ch_id {
            return Some(s.length_bytes());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use dab_eti::{Frame, StreamChar};

    fn make_frame(err: u8, streams: Vec<StreamChar>, msc: Vec<u8>) -> Frame {
        Frame {
            err,
            fct: 0,
            fic_present: true,
            nst: streams.len() as u8,
            fp: 0,
            mid: 1,
            fl: 0,
            streams,
            fic: vec![],
            msc,
        }
    }

    #[test]
    fn test_extract_subchannel_correct_slice() {
        // Stream 0: scid=1, stl=2 -> 16 bytes
        // Stream 1: scid=5, stl=3 -> 24 bytes
        let s0 = StreamChar { scid: 1, sad: 0, tpl: 0, stl: 2 };
        let s1 = StreamChar { scid: 5, sad: 0, tpl: 0, stl: 3 };
        let msc: Vec<u8> = (0u8..40).collect();

        let frame = make_frame(0xFF, vec![s0, s1], msc.clone());

        // extract scid=1 should be bytes 0..16
        let slice0 = extract_subchannel(&frame, 1).expect("should find scid=1");
        assert_eq!(slice0, &msc[0..16]);

        // extract scid=5 should be bytes 16..40
        let slice1 = extract_subchannel(&frame, 5).expect("should find scid=5");
        assert_eq!(slice1, &msc[16..40]);
    }

    #[test]
    fn test_extract_subchannel_not_found() {
        let s = StreamChar { scid: 1, sad: 0, tpl: 0, stl: 1 };
        let msc = vec![0u8; 8];
        let frame = make_frame(0xFF, vec![s], msc);

        assert!(extract_subchannel(&frame, 99).is_none());
    }

    #[test]
    fn test_extract_subchannel_err_not_0xff() {
        // err != 0xFF -> always None regardless of scid match
        let s = StreamChar { scid: 2, sad: 0, tpl: 0, stl: 2 };
        let msc = vec![0xABu8; 16];
        let frame = make_frame(0x00, vec![s], msc);

        assert!(extract_subchannel(&frame, 2).is_none());
    }

    #[test]
    fn test_find_subchannel_size() {
        let s0 = StreamChar { scid: 3, sad: 0, tpl: 0, stl: 5 };
        let s1 = StreamChar { scid: 7, sad: 0, tpl: 0, stl: 10 };
        let frame = make_frame(0xFF, vec![s0, s1], vec![0u8; 5 * 8 + 10 * 8]);

        assert_eq!(find_subchannel_size(&frame, 3), Some(40));
        assert_eq!(find_subchannel_size(&frame, 7), Some(80));
        assert_eq!(find_subchannel_size(&frame, 99), None);
    }
}
