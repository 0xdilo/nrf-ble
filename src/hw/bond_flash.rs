//! Flash-backed bond storage using the nRF52 NVMC peripheral.

use super::conn::{BondInfo, BondStore};
use super::pac;

/// Magic header of a programmed bond page.
pub const BOND_PAGE_MAGIC: u32 = 0x4E42_3144;
/// Flash page size used for the bond page.
pub const BOND_PAGE_SIZE: u32 = 4096;
pub const BOND_MAX: usize = 8;
pub const BOND_RECORD_LEN: usize = 44;

/// Serialize one bond record (44 bytes).
pub fn encode_record(bond: &BondInfo) -> [u8; BOND_RECORD_LEN] {
    let mut rec = [0u8; BOND_RECORD_LEN];
    rec[0] = bond.addr[0];
    rec[1] = bond.addr[1];
    rec[2] = bond.addr[2];
    rec[3] = bond.addr[3];
    rec[4] = bond.addr[4];
    rec[5] = bond.addr[5];
    rec[6] = 0;
    rec[7..23].copy_from_slice(&bond.ltk);
    rec[23] = bond.irk.is_some() as u8;
    if let Some(irk) = bond.irk {
        rec[24..40].copy_from_slice(&irk);
    }
    rec[40..44].copy_from_slice(&0xA5A5_5A5Au32.to_le_bytes());
    rec
}

/// Deserialize a bond record; `None` for empty or corrupt slots.
pub fn decode_record(rec: &[u8; BOND_RECORD_LEN]) -> Option<BondInfo> {
    if rec[40..44] != 0xA5A5_5A5Au32.to_le_bytes() {
        return None;
    }
    let mut addr = [0u8; 6];
    addr.copy_from_slice(&rec[..6]);
    let mut ltk = [0u8; 16];
    ltk.copy_from_slice(&rec[7..23]);
    let irk = if rec[23] != 0 {
        let mut irk = [0u8; 16];
        irk.copy_from_slice(&rec[24..40]);
        Some(irk)
    } else {
        None
    };
    Some(BondInfo { addr, ltk, irk })
}

/// Bond storage persisted in a reserved flash page (one 4 KB page,
/// erase-on-save).
pub struct FlashBondStore {
    /// Base address of the reserved page (must be page aligned).
    pub base: u32,
    nvmc: &'static pac::nvmc::RegisterBlock,
    cache: [Option<BondInfo>; BOND_MAX],
}

impl FlashBondStore {
    pub fn new(nvmc: &'static pac::nvmc::RegisterBlock, base: u32) -> Self {
        let mut store = FlashBondStore {
            base,
            nvmc,
            cache: [None; BOND_MAX],
        };
        store.load_page();
        store
    }

    fn read_ready(&self) {
        while self.nvmc.ready.read().bits() == 0 {}
    }

    fn load_page(&mut self) {
        let magic = unsafe { (self.base as *const u32).read_volatile() };
        if magic != BOND_PAGE_MAGIC {
            return;
        }
        for i in 0..BOND_MAX {
            let rec_addr = (self.base + 4 + (i as u32) * BOND_RECORD_LEN as u32) as *const u8;
            let mut rec = [0u8; BOND_RECORD_LEN];
            for (j, b) in rec.iter_mut().enumerate() {
                *b = unsafe { rec_addr.add(j).read_volatile() };
            }
            self.cache[i] = decode_record(&rec);
        }
    }

    fn flush(&mut self) {
        let r = self.nvmc;
        r.config.write(|w| w.wen().een());
        self.read_ready();
        r.erasepage()
            .write(|w| unsafe { w.erasepage().bits(self.base) });
        self.read_ready();
        r.config.write(|w| w.wen().wen());
        self.read_ready();
        let magic = BOND_PAGE_MAGIC;
        unsafe {
            (self.base as *mut u32).write_volatile(magic);
        }
        self.read_ready();
        for (i, bond) in self.cache.iter().enumerate() {
            if let Some(b) = bond {
                let rec = encode_record(b);
                let rec_addr = (self.base + 4 + (i as u32) * BOND_RECORD_LEN as u32) as *mut u8;
                for (j, byte) in rec.iter().enumerate() {
                    unsafe {
                        rec_addr.add(j).write_volatile(*byte);
                    }
                    self.read_ready();
                }
            }
        }
        r.config.write(|w| w.wen().ren());
        self.read_ready();
    }
}

impl BondStore for FlashBondStore {
    fn save(&mut self, peer: [u8; 6], ltk: [u8; 16], irk: Option<[u8; 16]>) {
        let info = BondInfo {
            addr: peer,
            ltk,
            irk,
        };
        for slot in self.cache.iter_mut() {
            if let Some(b) = slot {
                if b.addr == peer {
                    *slot = Some(info);
                    self.flush();
                    return;
                }
            }
        }
        for slot in self.cache.iter_mut() {
            if slot.is_none() {
                *slot = Some(info);
                self.flush();
                return;
            }
        }
    }

    fn find(&self, peer: &[u8; 6]) -> Option<BondInfo> {
        self.cache
            .iter()
            .flatten()
            .copied()
            .find(|b| &b.addr == peer)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_roundtrip() {
        let bond = BondInfo {
            addr: [0xAA; 6],
            ltk: [0x11; 16],
            irk: Some([0x22; 16]),
        };
        let rec = encode_record(&bond);
        assert_eq!(rec.len(), BOND_RECORD_LEN);
        assert_eq!(decode_record(&rec), Some(bond));
    }

    #[test]
    fn record_without_irk() {
        let bond = BondInfo {
            addr: [1, 2, 3, 4, 5, 6],
            ltk: [7; 16],
            irk: None,
        };
        let rec = encode_record(&bond);
        assert_eq!(decode_record(&rec), Some(bond));
    }

    #[test]
    fn empty_slot_decodes_to_none() {
        let rec = [0u8; BOND_RECORD_LEN];
        assert_eq!(decode_record(&rec), None);
    }

    #[test]
    fn corrupted_trailer_decodes_to_none() {
        let bond = BondInfo {
            addr: [0xAA; 6],
            ltk: [0x11; 16],
            irk: None,
        };
        let mut rec = encode_record(&bond);
        rec[40] ^= 0xFF;
        assert_eq!(decode_record(&rec), None);
    }
}
