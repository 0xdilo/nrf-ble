//! Host-side virtual radio for testing the stack without hardware.
//!
//! Emulates the nRF52 RADIO peripheral's BLE-mode data path: the hardware
//! appends the 24-bit CRC and applies data whitening on transmit, and
//! de-whitens and validates the CRC on receive. A wrong channel or a
//! corrupted packet yields a [`Error::CrcMismatch`], exactly like the
//! hardware's CRC error event.
//! (see module doc in lib.rs)
use crate::ll::crc;
use crate::ll::whiten;
use crate::Error;

/// Maximum on-air packet length (PDU + 2-byte header + 3-byte CRC).
pub const MAX_PACKET_LEN: usize = 255 + 2 + 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// An on-air packet as produced by the emulated transmitter.
pub struct Packet {
    /// Channel the packet was transmitted on.
    pub channel: u8,
    /// Whitened bytes including the CRC.
    pub bytes: [u8; MAX_PACKET_LEN],
    /// Number of valid bytes.
    pub len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A received packet after de-whitening and CRC validation.
pub struct RxPacket {
    /// The PDU bytes (header + payload, CRC stripped).
    pub pdu: [u8; MAX_PACKET_LEN],
    /// PDU length.
    pub len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A virtual nRF52 radio on a single channel.
pub struct VirtualRadio {
    /// The channel this radio listens on.
    pub channel: u8,
}

impl VirtualRadio {
    /// Create a radio on `channel`.
    pub const fn new(channel: u8) -> Self {
        VirtualRadio { channel }
    }

    /// Emulate a transmit: append CRC and whiten, as the hardware would.
    pub fn transmit(&self, pdu: &[u8]) -> Result<Packet, Error> {
        if pdu.len() + 3 > MAX_PACKET_LEN {
            return Err(Error::BufferTooSmall);
        }
        let mut packet = Packet {
            channel: self.channel,
            bytes: [0u8; MAX_PACKET_LEN],
            len: 0,
        };
        packet.bytes[..pdu.len()].copy_from_slice(pdu);
        crc::append_crc(pdu, &mut packet.bytes[pdu.len()..])?;
        let total = pdu.len() + 3;
        packet.len = total;
        whiten::whiten(&mut packet.bytes[..total], self.channel);
        Ok(packet)
    }

    /// Emulate a receive: de-whiten with this radio's channel and validate
    /// the CRC.
    pub fn receive(&self, packet: &Packet) -> Result<RxPacket, Error> {
        if packet.len < 3 {
            return Err(Error::InvalidLength);
        }
        let mut buf = [0u8; MAX_PACKET_LEN];
        buf[..packet.len].copy_from_slice(&packet.bytes[..packet.len]);
        whiten::whiten(&mut buf[..packet.len], self.channel);
        if !crc::check(&buf[..packet.len]) {
            return Err(Error::CrcMismatch);
        }
        Ok(RxPacket {
            pdu: buf,
            len: packet.len - 3,
        })
    }
}

/// Full transmit/receive loopback of a PDU on one channel.
pub fn loopback(pdu: &[u8], channel: u8) -> Result<RxPacket, Error> {
    let radio = VirtualRadio::new(channel);
    let packet = radio.transmit(pdu)?;
    radio.receive(&packet)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_roundtrip_preserves_pdu() {
        let pdu = [
            0x00u8, 0x12, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
        ];
        let rx = loopback(&pdu, 37).unwrap();
        assert_eq!(&rx.pdu[..rx.len], &pdu);
    }

    #[test]
    fn wrong_channel_fails_crc() {
        let pdu = [0x00u8, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let radio_tx = VirtualRadio::new(37);
        let packet = radio_tx.transmit(&pdu).unwrap();
        let radio_rx = VirtualRadio::new(38);
        assert_eq!(radio_rx.receive(&packet), Err(Error::CrcMismatch));
    }

    #[test]
    fn bit_flip_in_air_fails_crc() {
        let pdu = [0x00u8, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let radio = VirtualRadio::new(39);
        let mut packet = radio.transmit(&pdu).unwrap();
        packet.bytes[2] ^= 0x01;
        assert!(radio.receive(&packet).is_err());
    }
}
