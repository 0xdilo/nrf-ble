//! Link Layer: CRC-24, data whitening, PDU codecs, addresses, channels.
//!
//! All code in this module is pure and fully unit-tested on the host.

/// BLE addresses and address types.
pub mod addr;
/// Physical channels, frequencies and the hop sequence.
pub mod channels;
/// BLE CRC-24.
pub mod crc;
/// Advertising and data channel PDU codecs.
pub mod pdu;
/// BLE data whitening.
pub mod whiten;
