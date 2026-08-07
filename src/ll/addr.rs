#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Bluetooth device address type (GAP).
pub enum AddrType {
    /// Public address.
    Public,
    /// Random static address.
    RandomStatic,
    /// Random private resolvable address.
    RandomPrivateResolvable,
    /// Random private non-resolvable address.
    RandomPrivateNonResolvable,
}

impl AddrType {
    /// True when the address is a random address (TxAdd/RxAdd bit = 1).
    pub const fn is_random(&self) -> bool {
        !matches!(self, AddrType::Public)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A 6-byte Bluetooth device address.
pub struct BtAddr {
    /// Raw address bytes.
    pub addr: [u8; 6],
    /// Address type.
    pub addr_type: AddrType,
}

impl BtAddr {
    /// Construct an address with an explicit type.
    pub const fn from_bytes(addr: [u8; 6], addr_type: AddrType) -> Self {
        BtAddr { addr, addr_type }
    }

    /// Derive the random address subtype from the top two bits of the
    /// address (0b11 = static, 0b01 = private resolvable, 0b00 = private
    /// non-resolvable). Public addresses must be set explicitly with
    /// [`BtAddr::from_bytes`].
    pub const fn parse(addr: [u8; 6]) -> Self {
        let addr_type = match addr[5] >> 6 {
            0b11 => AddrType::RandomStatic,
            0b01 => AddrType::RandomPrivateResolvable,
            _ => AddrType::RandomPrivateNonResolvable,
        };
        BtAddr { addr, addr_type }
    }

    /// Raw address bytes.
    pub const fn as_bytes(&self) -> &[u8; 6] {
        &self.addr
    }

    /// The TxAdd bit used in advertising PDU headers.
    pub const fn tx_add_bit(&self) -> bool {
        !matches!(self.addr_type, AddrType::Public)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_private_non_resolvable() {
        let a = BtAddr::parse([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        assert_eq!(a.addr_type, AddrType::RandomPrivateNonResolvable);
    }

    #[test]
    fn parse_random_static() {
        let a = BtAddr::parse([0x01, 0x02, 0x03, 0x04, 0x05, 0xC1]);
        assert_eq!(a.addr_type, AddrType::RandomStatic);
    }

    #[test]
    fn parse_random_private_resolvable() {
        let a = BtAddr::parse([0x01, 0x02, 0x03, 0x04, 0x05, 0x40]);
        assert_eq!(a.addr_type, AddrType::RandomPrivateResolvable);
    }

    #[test]
    fn tx_add_bit_reflects_type() {
        assert!(!BtAddr::from_bytes([0; 6], AddrType::Public).tx_add_bit());
        assert!(BtAddr::from_bytes([0; 6], AddrType::RandomStatic).tx_add_bit());
    }
}
