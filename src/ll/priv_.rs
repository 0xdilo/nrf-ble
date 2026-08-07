//! Privacy: IRK-based resolvable private address (RPA) generation and
//! resolution.

use aes::cipher::{BlockEncrypt, KeyInit};
use aes::Aes128;

/// Generate the 3-byte hash part of a resolvable private address.
///
/// `ah(irk, prand)` = AES-128(irk, prand || padding)[0..3] with the
/// `prand` in the 3 least significant octets of the block.
pub fn ah(irk: &[u8; 16], prand: [u8; 3]) -> [u8; 3] {
    let mut block = [0u8; 16];
    block[..3].copy_from_slice(&prand);
    let cipher = Aes128::new(irk.into());
    let mut out = block;
    cipher.encrypt_block((&mut out).into());
    let mut hash = [0u8; 3];
    hash.copy_from_slice(&out[..3]);
    hash
}

/// Build a resolvable private address from an IRK and a random 24-bit
/// value. The top two bits of the prand (and the address) are set to
/// `0b01`, and the hash is computed over the prand including those bits.
pub fn generate_rpa(irk: &[u8; 16], prand: u32) -> [u8; 6] {
    let prand = prand & 0x00FF_FFFF;
    let mut prand_bytes = [
        (prand & 0xFF) as u8,
        ((prand >> 8) & 0xFF) as u8,
        ((prand >> 16) & 0xFF) as u8,
    ];
    prand_bytes[2] = (prand_bytes[2] & 0x3F) | 0x40;
    let hash = ah(irk, prand_bytes);
    let mut addr = [0u8; 6];
    addr[..3].copy_from_slice(&hash);
    addr[3..6].copy_from_slice(&prand_bytes);
    addr
}

/// Try to resolve a resolvable private address with an IRK.
///
/// Returns `true` when the hash part matches.
pub fn resolve_rpa(irk: &[u8; 16], addr: &[u8; 6]) -> bool {
    if addr[5] >> 6 != 0b01 {
        return false;
    }
    let mut prand = [0u8; 3];
    prand.copy_from_slice(&addr[3..6]);
    let expected = ah(irk, prand);
    expected == addr[..3]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ah_matches_reference() {
        let irk = [0u8; 16];
        let prand = [0x01, 0x02, 0x03];
        let h = ah(&irk, prand);
        assert_eq!(h, [0x96, 0x3C, 0xCC]);
    }

    #[test]
    fn generate_and_resolve_roundtrip() {
        let irk = [0x11u8; 16];
        for prand in [0u32, 1, 0x123456, 0xFFFFFF] {
            let addr = generate_rpa(&irk, prand);
            assert!(resolve_rpa(&irk, &addr));
        }
    }

    #[test]
    fn wrong_irk_does_not_resolve() {
        let irk = [0x11u8; 16];
        let other = [0x22u8; 16];
        let addr = generate_rpa(&irk, 0xABCDEF);
        assert!(!resolve_rpa(&other, &addr));
    }

    #[test]
    fn rpa_has_random_type_bits() {
        let irk = [0x11u8; 16];
        let addr = generate_rpa(&irk, 42);
        assert_eq!(addr[5] >> 6, 0b01);
    }
}
