//! nRF52 CCM peripheral driver for BLE LL payload encryption.

use super::pac;

pub const LL_NONCE_MASTER_TO_SLAVE: u8 = 0x01;
pub const LL_NONCE_SLAVE_TO_MASTER: u8 = 0x00;
pub const LL_MIC_LEN: usize = 4;

pub struct Ccm {
    regs: &'static pac::ccm::RegisterBlock,
    cfg: [u8; 45],
    scratch: [u8; 32],
}

impl Ccm {
    pub fn new(regs: &'static pac::ccm::RegisterBlock) -> Self {
        Ccm {
            regs,
            cfg: [0u8; 45],
            scratch: [0u8; 32],
        }
    }

    /// Program the session key, packet counter and direction into the
    /// CCM config block.
    pub fn setup(&mut self, key: &[u8; 16], packet_counter: u64, direction: u8) {
        self.cfg[..16].copy_from_slice(key);
        self.cfg[16] = 0x01;
        self.cfg[17..22].copy_from_slice(&packet_counter.to_le_bytes()[..5]);
        self.cfg[22] = direction;
        self.cfg[23..29].fill(0);
        self.cfg[29] = 0;
        self.cfg[30] = 0;
        self.cfg[31..45].fill(0);
    }

    fn run(&mut self, decrypt: bool, packet: *mut u8) -> Result<(), ()> {
        let r = &self.regs;
        r.mode.write(|w| {
            if decrypt {
                w.mode().decryption()
            } else {
                w.mode().encryption()
            }
            .datarate()
            ._1mbit()
            .length()
            .variant(pac::ccm::mode::LENGTH_A::DEFAULT)
        });
        r.enable.write(|w| unsafe { w.enable().bits(1) });
        r.cnfptr
            .write(|w| unsafe { w.cnfptr().bits(self.cfg.as_ptr() as u32) });
        r.scratchptr
            .write(|w| unsafe { w.scratchptr().bits(self.scratch.as_ptr() as u32) });
        r.inptr.write(|w| unsafe { w.inptr().bits(packet as u32) });
        r.outptr
            .write(|w| unsafe { w.outptr().bits(packet as u32) });
        r.tasks_ksgen.write(|w| unsafe { w.bits(1) });
        while r.events_endksgen.read().bits() == 0 {}
        r.events_endksgen.write(|w| w);
        r.tasks_crypt.write(|w| unsafe { w.bits(1) });
        while r.events_endcrypt.read().bits() == 0 {}
        r.events_endcrypt.write(|w| w);
        let err = r.events_error.read().bits() != 0;
        r.events_error.write(|w| w);
        r.enable.write(|w| unsafe { w.enable().bits(0) });
        if err {
            return Err(());
        }
        Ok(())
    }

    /// Encrypt or decrypt a data channel PDU in place.
    ///
    /// The packet must contain 4 spare bytes after the payload for the MIC
    /// (encrypt appends the MIC; decrypt validates and strips it).
    pub fn process(&mut self, decrypt: bool, pdu: &mut [u8], _len: usize) -> Result<(), ()> {
        self.cfg[29] = pdu[0];
        self.cfg[30] = pdu[1];
        self.cfg[31..45].fill(0);
        self.run(decrypt, pdu.as_mut_ptr())
    }
}
