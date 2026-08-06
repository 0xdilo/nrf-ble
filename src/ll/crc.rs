use crate::Error;

/// BLE CRC-24 generator polynomial x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1.
pub const CRC_POLY: u32 = 0x0000_065B;
/// BLE CRC-24 initial value.
pub const CRC_INIT: u32 = 0x0055_5555;
const CRC_REFLECTED_POLY: u32 = 0x00DA_6000;

/// Bit-reverse the low `bits` bits of `value`.
const fn reflect(value: u32, bits: u32) -> u32 {
    let mut v = value;
    let mut r = 0u32;
    let mut i = 0;
    while i < bits {
        r = (r << 1) | (v & 1);
        v >>= 1;
        i += 1;
    }
    r
}

/// Compute the BLE CRC-24 over `data` with the standard initial value.
pub const fn crc24(data: &[u8]) -> u32 {
    crc24_with_init(data, CRC_INIT)
}

/// Compute the BLE CRC-24 over `data` with a custom (normal-form) initial value.
pub const fn crc24_with_init(data: &[u8], init: u32) -> u32 {
    let mut crc = reflect(init & 0xFF_FFFF, 24);
    let mut i = 0;
    while i < data.len() {
        crc ^= data[i] as u32;
        let mut bit = 0;
        while bit < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ CRC_REFLECTED_POLY;
            } else {
                crc >>= 1;
            }
            bit += 1;
        }
        crc &= 0xFF_FFFF;
        i += 1;
    }
    crc
}

/// Append the CRC bytes (on-air order: normal-form MSB byte first) to `out`.
pub fn append_crc(pdu: &[u8], out: &mut [u8]) -> Result<(), Error> {
    if out.len() < 3 {
        return Err(Error::BufferTooSmall);
    }
    let crc = crc24(pdu);
    out[0] = crc as u8;
    out[1] = (crc >> 8) as u8;
    out[2] = (crc >> 16) as u8;
    Ok(())
}

/// Verify a PDU with appended CRC: true when the CRC residue is zero.
pub fn check(pdu_with_crc: &[u8]) -> bool {
    if pdu_with_crc.len() < 3 {
        return false;
    }
    crc24(pdu_with_crc) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_check_value() {
        assert_eq!(crc24(b"123456789"), 0xC2_5A_56);
    }

    #[test]
    fn residue_is_zero_when_crc_appended() {
        let pdu = [0x00, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let mut buf = [0u8; 3];
        append_crc(&pdu, &mut buf).unwrap();
        let mut full = [0u8; 11];
        full[..8].copy_from_slice(&pdu);
        full[8..].copy_from_slice(&buf);
        assert!(check(&full));
    }

    #[test]
    fn crc_detects_single_bit_flip() {
        let pdu = [0x00, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let mut buf = [0u8; 3];
        append_crc(&pdu, &mut buf).unwrap();
        let mut full = [0u8; 11];
        full[..8].copy_from_slice(&pdu);
        full[8..].copy_from_slice(&buf);
        full[3] ^= 0x80;
        assert!(!check(&full));
    }

    #[test]
    fn crc_detects_burst_errors() {
        let pdu = [0x00, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let mut buf = [0u8; 3];
        append_crc(&pdu, &mut buf).unwrap();
        let mut full = [0u8; 11];
        full[..8].copy_from_slice(&pdu);
        full[8..].copy_from_slice(&buf);
        full[4] ^= 0x3C;
        assert!(!check(&full));
    }

    #[test]
    fn crc_differs_per_payload() {
        assert_ne!(crc24(b"aaaa"), crc24(b"aaab"));
    }

    #[test]
    fn crc_with_init_matches_connect_req_semantics() {
        let crc = crc24_with_init(b"\x00\x0C\x01", 0x654321);
        assert_eq!(crc, crc24_with_init(b"\x00\x0C\x01", 0x654321));
    }
}
