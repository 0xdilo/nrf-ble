//! nRF52 RADIO peripheral driver in BLE 1 Mbit/s mode.
use crate::ll::channels;
use crate::Error;

use super::pac;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Radio TX power in dBm. Variants depend on the selected chip.
pub enum TxPower {
    /// -40 dBm
    Neg40Dbm,
    /// -30 dBm (nRF52811, nRF52832, nRF52833)
    #[cfg(any(feature = "nrf52811", feature = "nrf52832", feature = "nrf52833"))]
    Neg30Dbm,
    /// -20 dBm
    Neg20Dbm,
    /// -16 dBm
    Neg16Dbm,
    /// -12 dBm
    Neg12Dbm,
    /// -8 dBm
    Neg8Dbm,
    /// -4 dBm
    Neg4Dbm,
    /// 0 dBm
    ZeroDbm,
    /// +3 dBm
    Pos3Dbm,
    /// +4 dBm
    Pos4Dbm,
    /// +2 dBm (nRF52833, nRF52840)
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    Pos2Dbm,
    /// +5 dBm (nRF52833, nRF52840)
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    Pos5Dbm,
    /// +6 dBm (nRF52833, nRF52840)
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    Pos6Dbm,
    /// +7 dBm (nRF52833, nRF52840)
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    Pos7Dbm,
    /// +8 dBm (nRF52833, nRF52840)
    #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
    Pos8Dbm,
}

/// nRF52 RADIO driver in BLE 1 Mbit/s mode.
/// Radio PHY options.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phy {
    /// BLE 1 Mbit/s (LE 1M).
    Ble1Mbit,
    /// BLE 2 Mbit/s (LE 2M).
    Ble2Mbit,
}

/// nRF52 RADIO driver.
pub struct Radio {
    regs: pac::RADIO,
}

impl Radio {
    /// Wrap the RADIO peripheral.
    pub fn new(regs: pac::RADIO) -> Self {
        Radio { regs }
    }

    /// Select the PHY (1 Mbit/s or 2 Mbit/s).
    pub fn set_phy(&self, phy: Phy) {
        let r = &self.regs;
        match phy {
            Phy::Ble1Mbit => {
                r.mode.write(|w| w.mode().ble_1mbit());
                r.pcnf0
                    .modify(|_, w| unsafe { w.s1len().bits(0) }.plen()._8bit());
            }
            Phy::Ble2Mbit => {
                r.mode.write(|w| w.mode().ble_2mbit());
                r.pcnf0
                    .modify(|_, w| unsafe { w.s1len().bits(8) }.plen()._16bit());
            }
        }
    }

    /// Configure the radio for BLE 1 Mbit/s operation with the advertising
    /// access address, 24-bit CRC and data whitening.
    pub fn init(&self) {
        let r = &self.regs;
        r.mode.write(|w| w.mode().ble_1mbit());
        r.pcnf0.write(|w| {
            unsafe { w.lflen().bits(8).s0len().bit(true).s1len().bits(0) }
                .s1incl()
                .automatic()
                .plen()
                ._8bit()
        });
        r.pcnf1.write(|w| {
            unsafe { w.maxlen().bits(255).balen().bits(3).statlen().bits(0) }
                .endian()
                .little()
                .whiteen()
                .enabled()
        });
        r.crccnf.write(|w| w.len().three().skipaddr().skip());
        r.crcinit
            .write(|w| unsafe { w.crcinit().bits(0x0055_5555) });
        r.crcpoly
            .write(|w| unsafe { w.crcpoly().bits(0x0000_065B) });
        self.set_access_address(channels::ADV_ACCESS_ADDRESS);
        self.set_tx_power(TxPower::ZeroDbm);
        self.set_channel(channels::ADV_CHANNELS[0]).ok();
    }

    /// Set the access address (4 bytes) and enable logical address 0.
    pub fn set_access_address(&self, aa: u32) {
        let (base, prefix) = channels::access_address_to_base_prefix(aa);
        let r = &self.regs;
        r.base0.write(|w| unsafe { w.base0().bits(base) });
        r.prefix0.write(|w| unsafe { w.ap0().bits(prefix) });
        r.txaddress.write(|w| unsafe { w.txaddress().bits(0) });
        r.rxaddresses.write(|w| w.addr0().enabled());
    }

    /// Set the RF channel (0-39) and matching whitening initial value.
    pub fn set_channel(&self, channel: u8) -> Result<(), Error> {
        let freq = channels::channel_frequency(channel)? - 2400;
        let r = &self.regs;
        r.frequency
            .write(|w| unsafe { w.frequency().bits(freq as u8) });
        r.datawhiteiv
            .write(|w| unsafe { w.datawhiteiv().bits(0x40 | (channel & 0x3F)) });
        Ok(())
    }

    /// Set the CRC initial value (24-bit, normal form).
    pub fn set_crc_init(&self, init: u32) {
        self.regs
            .crcinit
            .write(|w| unsafe { w.crcinit().bits(init) });
    }

    /// Set the TX power.
    pub fn set_tx_power(&self, power: TxPower) {
        let r = &self.regs;
        r.txpower.write(|w| {
            let w = w.txpower();
            match power {
                TxPower::Neg40Dbm => w.neg40d_bm(),
                #[cfg(any(feature = "nrf52811", feature = "nrf52833"))]
                TxPower::Neg30Dbm => w.neg30d_bm(),
                #[cfg(feature = "nrf52832")]
                TxPower::Neg30Dbm => unsafe { w.bits(0xE2) },
                TxPower::Neg20Dbm => w.neg20d_bm(),
                TxPower::Neg16Dbm => w.neg16d_bm(),
                TxPower::Neg12Dbm => w.neg12d_bm(),
                TxPower::Neg8Dbm => w.neg8d_bm(),
                TxPower::Neg4Dbm => w.neg4d_bm(),
                TxPower::ZeroDbm => w._0d_bm(),
                TxPower::Pos3Dbm => w.pos3d_bm(),
                TxPower::Pos4Dbm => w.pos4d_bm(),
                #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
                TxPower::Pos2Dbm => w.pos2d_bm(),
                #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
                TxPower::Pos5Dbm => w.pos5d_bm(),
                #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
                TxPower::Pos6Dbm => w.pos6d_bm(),
                #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
                TxPower::Pos7Dbm => w.pos7d_bm(),
                #[cfg(any(feature = "nrf52833", feature = "nrf52840"))]
                TxPower::Pos8Dbm => w.pos8d_bm(),
            }
        });
    }

    /// Transmit one PDU, blocking until the radio is disabled.
    pub fn transmit(&self, pdu: &[u8]) {
        let r = &self.regs;
        r.packetptr
            .write(|w| unsafe { w.packetptr().bits(pdu.as_ptr() as u32) });
        r.tasks_txen.write(|w| unsafe { w.bits(1) });
        while r.events_ready.read().bits() == 0 {}
        r.events_ready.write(|w| w);
        r.tasks_start.write(|w| unsafe { w.bits(1) });
        while r.events_end.read().bits() == 0 {}
        r.events_end.write(|w| w);
        r.tasks_disable.write(|w| unsafe { w.bits(1) });
        while r.events_disabled.read().bits() == 0 {}
        r.events_disabled.write(|w| w);
    }

    /// Start reception into `buf`, blocking until the radio ramps up.
    pub fn receive_start(&self, buf: &mut [u8]) {
        let r = &self.regs;
        r.packetptr
            .write(|w| unsafe { w.packetptr().bits(buf.as_ptr() as u32) });
        r.tasks_rxen.write(|w| unsafe { w.bits(1) });
        while r.events_ready.read().bits() == 0 {}
        r.events_ready.write(|w| w);
        r.tasks_start.write(|w| unsafe { w.bits(1) });
    }

    /// Poll for packet end. Returns the PDU length when a valid packet was
    /// received, `Ok(None)` while still listening, or an error when the CRC
    /// check failed.
    pub fn receive_poll(&self, buf: &[u8]) -> Result<Option<usize>, Error> {
        let r = &self.regs;
        if r.events_end.read().bits() == 0 {
            return Ok(None);
        }
        let crc_ok = r.events_crcerror.read().bits() == 0;
        let pdu_len = 2 + (((buf[1] >> 1) & 0x3F) as usize);
        r.events_end.write(|w| w);
        r.events_crcerror.write(|w| w);
        r.tasks_disable.write(|w| unsafe { w.bits(1) });
        while r.events_disabled.read().bits() == 0 {}
        r.events_disabled.write(|w| w);
        if !crc_ok {
            return Err(Error::CrcMismatch);
        }
        Ok(Some(pdu_len))
    }

    /// Abort reception and disable the radio.
    pub fn receive_cancel(&self) {
        let r = &self.regs;
        r.tasks_disable.write(|w| unsafe { w.bits(1) });
        while r.events_disabled.read().bits() == 0 {}
        r.events_disabled.write(|w| w);
    }

    /// RSSI of the last received packet in dBm.
    pub fn rssi(&self) -> i8 {
        self.regs.rssisample.read().rssisample().bits() as i8
    }
}
