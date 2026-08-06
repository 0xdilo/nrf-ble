use crate::Error;

/// Fixed BLE advertising access address.
pub const ADV_ACCESS_ADDRESS: u32 = 0x8E89_BED6;

/// Advertising physical channel indices (37, 38, 39).
pub const ADV_CHANNELS: [u8; 3] = [37, 38, 39];

/// Data channel hop increment sequence (Bluetooth Core Spec, Vol 6, Part B, 2.4.5.2).
pub const HOP_INCREMENTS: [u8; 20] = [
    5, 11, 19, 17, 9, 15, 6, 8, 13, 7, 10, 14, 12, 18, 3, 16, 4, 0, 1, 2,
];

/// RF frequency in MHz for a channel (0-39).
pub const fn channel_frequency(channel: u8) -> Result<u16, Error> {
    match channel {
        0..=36 => Ok(2404 + 2 * channel as u16),
        37 => Ok(2402),
        38 => Ok(2426),
        39 => Ok(2480),
        _ => Err(Error::InvalidChannel),
    }
}

/// Next data channel given the current channel and hop increment.
pub const fn next_data_channel(current: u8, hop: u8) -> u8 {
    (current + hop) % 37
}

/// Preamble byte for a given access address (nRF52 derives this from the AA first bit).
pub const fn preamble_for_access_address(aa: u32) -> u8 {
    if aa & 1 == 0 {
        0x55
    } else {
        0xAA
    }
}

/// Split an access address into the nRF52 BASE0 and PREFIX0.AP0 register values.
pub const fn access_address_to_base_prefix(aa: u32) -> (u32, u8) {
    let base = (aa & 0x00FF_FFFF) << 8;
    let prefix = (aa >> 24) as u8;
    (base, prefix)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adv_channel_frequencies() {
        assert_eq!(channel_frequency(37).unwrap(), 2402);
        assert_eq!(channel_frequency(38).unwrap(), 2426);
        assert_eq!(channel_frequency(39).unwrap(), 2480);
    }

    #[test]
    fn data_channel_frequencies() {
        assert_eq!(channel_frequency(0).unwrap(), 2404);
        assert_eq!(channel_frequency(36).unwrap(), 2476);
        assert_eq!(channel_frequency(10).unwrap(), 2424);
    }

    #[test]
    fn invalid_channel_rejected() {
        assert!(channel_frequency(40).is_err());
        assert!(channel_frequency(255).is_err());
    }

    #[test]
    fn fixed_hop_cycles_all_channels() {
        let mut seen = [false; 37];
        let mut ch = 0u8;
        for _ in 0..37 {
            ch = next_data_channel(ch, 13);
            seen[ch as usize] = true;
        }
        assert!(seen.iter().all(|&s| s));
    }

    #[test]
    fn hop_table_entries_are_valid() {
        assert_eq!(HOP_INCREMENTS.len(), 20);
        for &hop in &HOP_INCREMENTS {
            assert!(hop < 37);
        }
    }

    #[test]
    fn preamble_for_adv_access_address() {
        assert_eq!(preamble_for_access_address(ADV_ACCESS_ADDRESS), 0x55);
        assert_eq!(preamble_for_access_address(0x8E89BED7), 0xAA);
    }

    #[test]
    fn base_prefix_split_matches_nrf52_layout() {
        let (base, prefix) = access_address_to_base_prefix(ADV_ACCESS_ADDRESS);
        assert_eq!(base, 0x89BE_D600);
        assert_eq!(prefix, 0x8E);
    }
}
