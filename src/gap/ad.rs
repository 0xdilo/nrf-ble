use crate::Error;

/// Flags.
pub const AD_TYPE_FLAGS: u8 = 0x01;
/// Incomplete list of 16-bit service UUIDs.
pub const AD_TYPE_UUID16_INCOMPLETE: u8 = 0x02;
/// Complete list of 16-bit service UUIDs.
pub const AD_TYPE_UUID16_COMPLETE: u8 = 0x03;
/// Incomplete list of 32-bit service UUIDs.
pub const AD_TYPE_UUID32_INCOMPLETE: u8 = 0x04;
/// Complete list of 32-bit service UUIDs.
pub const AD_TYPE_UUID32_COMPLETE: u8 = 0x05;
/// Incomplete list of 128-bit service UUIDs.
pub const AD_TYPE_UUID128_INCOMPLETE: u8 = 0x06;
/// Complete list of 128-bit service UUIDs.
pub const AD_TYPE_UUID128_COMPLETE: u8 = 0x07;
/// Shortened local name.
pub const AD_TYPE_SHORT_NAME: u8 = 0x08;
/// Complete local name.
pub const AD_TYPE_COMPLETE_NAME: u8 = 0x09;
/// TX power level (in dBm).
pub const AD_TYPE_TX_POWER_LEVEL: u8 = 0x0A;
/// Appearance.
pub const AD_TYPE_APPEARANCE: u8 = 0x19;
/// Manufacturer specific data.
pub const AD_TYPE_MANUFACTURER_SPECIFIC: u8 = 0xFF;

/// LE Limited Discoverable Mode flag bit.
pub const AD_FLAG_LE_LIMITED_DISCOVERABLE: u8 = 0x01;
/// LE General Discoverable Mode flag bit.
pub const AD_FLAG_LE_GENERAL_DISCOVERABLE: u8 = 0x02;
/// BR/EDR not supported flag bit.
pub const AD_FLAG_BR_EDR_NOT_SUPPORTED: u8 = 0x04;
/// LE and BR/EDR controller support flag bit.
pub const AD_FLAG_LE_BR_EDR_CONTROLLER: u8 = 0x08;
/// LE and BR/EDR host support flag bit.
pub const AD_FLAG_LE_BR_EDR_HOST: u8 = 0x10;

/// Maximum length of legacy advertising data and scan response data.
pub const LEGACY_AD_DATA_MAX_LEN: usize = 31;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// One AD structure: a length byte, an AD type byte and payload data.
pub struct AdEntry<'a> {
    /// AD type.
    pub ad_type: u8,
    /// AD payload.
    pub data: &'a [u8],
}

/// Iterator over the AD structures of a buffer.
pub struct AdIter<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> AdIter<'a> {
    /// Create an iterator over `buf`.
    pub const fn new(buf: &'a [u8]) -> Self {
        AdIter { buf, pos: 0 }
    }
}

impl<'a> Iterator for AdIter<'a> {
    type Item = Result<AdEntry<'a>, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.buf.len() {
            return None;
        }
        let len = self.buf[self.pos] as usize;
        if len == 0 {
            return Some(Err(Error::InvalidAd));
        }
        if self.pos + 1 + len > self.buf.len() {
            return Some(Err(Error::InvalidAd));
        }
        let entry = AdEntry {
            ad_type: self.buf[self.pos + 1],
            data: &self.buf[self.pos + 2..self.pos + 1 + len],
        };
        self.pos += 1 + len;
        Some(Ok(entry))
    }
}

/// Iterate the AD structures of `buf`.
pub fn iter(buf: &[u8]) -> AdIter<'_> {
    AdIter::new(buf)
}

/// Pack AD structures into `out`, returning the number of bytes written.
pub fn build(entries: &[AdEntry], out: &mut [u8]) -> Result<usize, Error> {
    let mut pos = 0usize;
    for entry in entries {
        let len = entry.data.len() + 1;
        if entry.data.len() > u8::MAX as usize - 1 {
            return Err(Error::InvalidLength);
        }
        if pos + 1 + len > out.len() {
            return Err(Error::BufferTooSmall);
        }
        out[pos] = len as u8;
        out[pos + 1] = entry.ad_type;
        out[pos + 2..pos + 1 + len].copy_from_slice(entry.data);
        pos += 1 + len;
    }
    Ok(pos)
}

/// Assemble the Flags AD payload value.
pub const fn ad_flags_value(
    general_discoverable: bool,
    limited_discoverable: bool,
    br_edr_not_supported: bool,
) -> u8 {
    let mut value = 0u8;
    if limited_discoverable {
        value |= AD_FLAG_LE_LIMITED_DISCOVERABLE;
    }
    if general_discoverable {
        value |= AD_FLAG_LE_GENERAL_DISCOVERABLE;
    }
    if br_edr_not_supported {
        value |= AD_FLAG_BR_EDR_NOT_SUPPORTED;
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_and_parse_roundtrip() {
        let entries = [
            AdEntry {
                ad_type: AD_TYPE_FLAGS,
                data: &[ad_flags_value(true, false, true)],
            },
            AdEntry {
                ad_type: AD_TYPE_TX_POWER_LEVEL,
                data: &[0],
            },
            AdEntry {
                ad_type: AD_TYPE_COMPLETE_NAME,
                data: b"nrf-ble",
            },
            AdEntry {
                ad_type: AD_TYPE_MANUFACTURER_SPECIFIC,
                data: &[0x59, 0x00, 0x01, 0x02],
            },
        ];
        let mut buf = [0u8; 31];
        let n = build(&entries, &mut buf).unwrap();
        assert!(n <= 31);
        let mut parsed: [Option<AdEntry>; 4] = [None; 4];
        let mut count = 0;
        for entry in iter(&buf[..n]) {
            parsed[count] = Some(entry.unwrap());
            count += 1;
        }
        assert_eq!(count, 4);
        let parsed0 = parsed[0].unwrap();
        let parsed2 = parsed[2].unwrap();
        assert_eq!(parsed0.ad_type, AD_TYPE_FLAGS);
        assert_eq!(parsed0.data, &[0x06]);
        assert_eq!(parsed2.ad_type, AD_TYPE_COMPLETE_NAME);
        assert_eq!(parsed2.data, b"nrf-ble");
    }

    #[test]
    fn truncated_entry_rejected() {
        let buf = [0x05, 0x09, b'n', b'r', b'f'];
        let mut it = iter(&buf);
        assert!(it.next().unwrap().is_err());
    }

    #[test]
    fn zero_length_entry_rejected() {
        let buf = [0x00, 0x01, 0x06];
        let mut it = iter(&buf);
        assert!(it.next().unwrap().is_err());
    }

    #[test]
    fn build_reports_buffer_too_small() {
        let entries = [AdEntry {
            ad_type: AD_TYPE_COMPLETE_NAME,
            data: b"a-very-long-name-that-does-not-fit",
        }];
        let mut buf = [0u8; 5];
        assert!(build(&entries, &mut buf).is_err());
    }

    #[test]
    fn empty_ad_is_valid() {
        let n = build(&[], &mut [0u8; 4]).unwrap();
        assert_eq!(n, 0);
    }

    #[test]
    fn uuid16_entries_pack_le() {
        let entries = [AdEntry {
            ad_type: AD_TYPE_UUID16_COMPLETE,
            data: &[0x0D, 0x18],
        }];
        let mut buf = [0u8; 8];
        let n = build(&entries, &mut buf).unwrap();
        assert_eq!(&buf[..n], &[0x03, 0x03, 0x0D, 0x18]);
    }
}
