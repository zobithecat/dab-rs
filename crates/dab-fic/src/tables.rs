//! Sub-channel sizing tables from ETSI EN 300 401 §6.2.1.
//!
//! UEP (Unequal Error Protection) short-form table and EEP (Equal Error
//! Protection) long-form sizing. Faithful port of the Python reference
//! `tdmb/eti/fic.py`.

/// UEP short-form table: index -> (bitrate_kbps, size_cu, protection label).
///
/// ETSI EN 300 401 §6.2.1, Table 8 (short form sub-channel organization).
pub fn uep_table(idx: u8) -> Option<(u16, u16, &'static str)> {
    Some(match idx {
        0 => (32, 16, "UEP-5"),
        1 => (32, 21, "UEP-4"),
        2 => (32, 24, "UEP-3"),
        3 => (32, 29, "UEP-2"),
        4 => (48, 24, "UEP-5"),
        5 => (48, 29, "UEP-4"),
        6 => (48, 35, "UEP-3"),
        7 => (48, 42, "UEP-2"),
        8 => (56, 29, "UEP-5"),
        9 => (56, 35, "UEP-4"),
        10 => (56, 42, "UEP-3"),
        11 => (56, 52, "UEP-2"),
        12 => (64, 32, "UEP-5"),
        13 => (64, 42, "UEP-4"),
        14 => (64, 48, "UEP-3"),
        15 => (64, 58, "UEP-2"),
        16 => (80, 40, "UEP-5"),
        17 => (80, 52, "UEP-4"),
        18 => (80, 58, "UEP-3"),
        19 => (80, 70, "UEP-2"),
        20 => (96, 48, "UEP-5"),
        21 => (96, 58, "UEP-4"),
        22 => (96, 70, "UEP-3"),
        23 => (96, 84, "UEP-2"),
        24 => (112, 58, "UEP-5"),
        25 => (112, 70, "UEP-4"),
        26 => (112, 84, "UEP-3"),
        27 => (112, 104, "UEP-2"),
        28 => (128, 64, "UEP-5"),
        29 => (128, 84, "UEP-4"),
        30 => (128, 96, "UEP-3"),
        31 => (128, 116, "UEP-2"),
        32 => (160, 80, "UEP-5"),
        33 => (160, 104, "UEP-4"),
        34 => (160, 116, "UEP-3"),
        35 => (160, 140, "UEP-2"),
        36 => (192, 96, "UEP-5"),
        37 => (192, 116, "UEP-4"),
        38 => (192, 140, "UEP-3"),
        39 => (192, 168, "UEP-2"),
        40 => (224, 116, "UEP-5"),
        41 => (224, 140, "UEP-4"),
        42 => (224, 168, "UEP-3"),
        43 => (224, 208, "UEP-2"),
        44 => (256, 128, "UEP-5"),
        45 => (256, 168, "UEP-4"),
        46 => (256, 192, "UEP-3"),
        47 => (256, 232, "UEP-2"),
        48 => (320, 168, "UEP-5"),
        49 => (320, 232, "UEP-2"),
        50 => (384, 208, "UEP-5"),
        51 => (384, 280, "UEP-2"),
        52 => (384, 280, "UEP-1"),
        53 => (32, 24, "UEP-1"),
        54 => (96, 48, "UEP-1"),
        55 => (128, 96, "UEP-1"),
        56 => (192, 116, "UEP-1"),
        57 => (256, 192, "UEP-1"),
        58 => (384, 280, "UEP-1"),
        59 => (32, 16, "UEP-5"),
        60 => (32, 16, "UEP-5"),
        61 => (32, 16, "UEP-5"),
        62 => (32, 16, "UEP-5"),
        63 => (32, 16, "UEP-5"),
        _ => return None,
    })
}

/// EEP option 0 (A-level): CUs per protection level (`level + 1` keyed).
fn eep_a(level_plus_1: u8) -> u16 {
    match level_plus_1 {
        1 => 12,
        2 => 8,
        3 => 6,
        4 => 4,
        _ => 0,
    }
}

/// EEP option 1 (B-level): CUs per protection level (`level + 1` keyed).
fn eep_b(level_plus_1: u8) -> u16 {
    match level_plus_1 {
        1 => 27,
        2 => 21,
        3 => 18,
        4 => 15,
        _ => 0,
    }
}

/// Return `(bitrate_kbps, size_cu, label)` for an EEP sub-channel given its
/// size in capacity units.
///
/// `opt == 0` selects EEP-A (bitrate = 8*n), `opt == 1` selects EEP-B
/// (bitrate = 32*n), per ETSI EN 300 401 §6.2.1 long-form.
pub fn eep_size(opt: u8, level: u8, sub_size_cu: u16) -> (u16, u16, String) {
    if opt == 0 {
        let m = eep_a(level + 1);
        if m == 0 {
            return (0, sub_size_cu, format!("EEP-{}A", level + 1));
        }
        let n = sub_size_cu / m;
        (8 * n, sub_size_cu, format!("EEP-{}A", level + 1))
    } else {
        let m = eep_b(level + 1);
        if m == 0 {
            return (0, sub_size_cu, format!("EEP-{}B", level + 1));
        }
        let n = sub_size_cu / m;
        (32 * n, sub_size_cu, format!("EEP-{}B", level + 1))
    }
}
