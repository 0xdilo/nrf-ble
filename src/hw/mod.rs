//! Hardware layer: nRF52 RADIO/TIMER0 drivers and the high-level [`Ble`]
//! API.
//!
//! Requires the `nrf52832` or `nrf52840` feature.
#[cfg(all(feature = "nrf52832", feature = "nrf52840"))]
compile_error!("features `nrf52832` and `nrf52840` are mutually exclusive");

#[cfg(feature = "nrf52832")]
pub(crate) use nrf52832_pac as pac;
#[cfg(feature = "nrf52840")]
pub(crate) use nrf52840_pac as pac;

mod ble;
mod radio;
mod timers;

pub use ble::{AdvParams, AdvType, Ble, BleEvent, FilterPolicy, ScanParams};
pub use radio::{Radio, TxPower};
