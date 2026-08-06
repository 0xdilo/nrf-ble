/// BLE whitening LFSR initial value for a channel (0x40 + channel).
pub const fn whitening_init(channel: u8) -> u8 {
    0x40 | (channel & 0x3F)
}

/// Apply BLE data whitening to `buf` in place (involutive).
pub fn whiten(buf: &mut [u8], channel: u8) {
    let mut lfsr: u16 = whitening_init(channel) as u16;
    for byte in buf {
        let mut out = 0u8;
        let mut i = 0;
        while i < 8 {
            let data_bit = (*byte >> i) & 1;
            let key_bit = (lfsr >> 6) as u8 & 1;
            out |= (data_bit ^ key_bit) << i;
            let feedback = ((lfsr >> 6) ^ (lfsr >> 3)) & 1;
            lfsr = ((lfsr << 1) | feedback) & 0x7F;
            i += 1;
        }
        *byte = out;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_follows_ble_scheme() {
        assert_eq!(whitening_init(37), 0x65);
        assert_eq!(whitening_init(0), 0x40);
        assert_eq!(whitening_init(39), 0x67);
    }

    #[test]
    fn whiten_is_an_involution_for_all_adv_channels() {
        for ch in [0u8, 17, 36, 37, 38, 39] {
            let mut data = [0x00u8, 0x0C, 0xAA, 0x55, 0xFF, 0x01, 0x80, 0x7E];
            let original = data;
            whiten(&mut data, ch);
            whiten(&mut data, ch);
            assert_eq!(data, original);
        }
    }

    #[test]
    fn different_channels_produce_different_output() {
        let mut a = [0x42u8, 0x13, 0x00, 0xFF];
        let mut b = a;
        whiten(&mut a, 37);
        whiten(&mut b, 38);
        assert_ne!(a, b);
    }

    #[test]
    fn whitening_init_has_msb_set() {
        for ch in 0..40u8 {
            assert!(whitening_init(ch) & 0x40 != 0);
            assert!(whitening_init(ch) & 0x80 == 0);
        }
    }
}
