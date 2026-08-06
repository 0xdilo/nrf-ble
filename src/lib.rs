//! An open-source Bluetooth Low Energy stack for Nordic nRF52 devices.
//!
//! `nrf-ble` reimplements the functionality of Nordic's proprietary
//! SoftDevice BLE stack as a clean-room implementation, based solely on the
//! public Bluetooth Core Specification and the public nRF52 Product
//! Specification. It is a `no_std` crate usable with just the official
//! device PAC (no RTOS, no HAL required).
//!
//! # Layout
//!
//! - [`ll`] - Link Layer: BLE CRC-24, data whitening, advertising and data
//!   channel PDU codecs, addresses, channel mapping.
//! - [`gap`] - Generic Access Profile: advertising data (AD) structure
//!   codec.
//! - [`hw`] - Hardware layer (enabled with the `nrf52832` or `nrf52840`
//!   feature): RADIO/TIMER0 drivers and the high-level [`hw::Ble`] API in
//!   the style of the SoftDevice (`gap_adv_start`, `gap_adv_stop`, ...).
//! - [`sim`] - Host-side virtual radio used for loopback testing without
//!   hardware (enabled with the `sim` feature).
//!
//! # Example
//!
//! ```no_run
//! use nrf_ble::hw::{AdvParams, Ble, BleEvent};
//! use nrf_ble::ll::addr::AddrType;
//!
//! let p = nrf52832_pac::Peripherals::take().unwrap();
//! let mut ble = Ble::new(p.RADIO, p.TIMER0);
//!
//! ble.gap_address_set([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF], AddrType::Public);
//! let adv_data = [
//!     0x02, 0x01, 0x06,               // Flags: LE General Discoverable
//!     0x03, 0x03, 0x0D, 0x18,         // Complete list of 16-bit UUIDs: 0x180D
//!     0x09, 0x6E, 0x72, 0x66, 0x2D, 0x62, 0x6C, 0x65, // Complete name "nrf-ble"
//! ];
//! ble.gap_adv_set_configure(&adv_data, &[]).unwrap();
//! ble.gap_adv_start(&AdvParams::default()).unwrap();
//!
//! ble.adv_forever(|ble, evt| match evt {
//!     BleEvent::ScanReqReceived { addr } => {
//!         // ...
//!     }
//!     _ => {}
//! });
//! ```

#![no_std]
#![warn(missing_docs)]

pub mod error;
pub mod gap;
#[cfg(any(feature = "nrf52832", feature = "nrf52840"))]
pub mod hw;
pub mod ll;
#[cfg(feature = "sim")]
pub mod sim;

pub use error::Error;
