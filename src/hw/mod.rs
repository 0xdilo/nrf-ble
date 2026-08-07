//! Hardware layer: nRF52 RADIO/TIMER0 drivers and the high-level [`Ble`]
//! API.
//!
//! Requires one of the `nrf52811`, `nrf52832`, `nrf52833` or `nrf52840` features.
#[cfg(not(any(
    feature = "nrf52811",
    feature = "nrf52832",
    feature = "nrf52833",
    feature = "nrf52840"
)))]
compile_error!(
    "one of the `nrf52811`, `nrf52832`, `nrf52833` or `nrf52840` features must be enabled"
);

#[cfg(all(
    feature = "nrf52811",
    any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")
))]
compile_error!("chip features are mutually exclusive");

#[cfg(all(feature = "nrf52832", any(feature = "nrf52833", feature = "nrf52840")))]
compile_error!("chip features are mutually exclusive");

#[cfg(all(feature = "nrf52833", feature = "nrf52840"))]
compile_error!("chip features are mutually exclusive");

#[cfg(feature = "nrf52811")]
pub(crate) use nrf52811_pac as pac;
#[cfg(feature = "nrf52832")]
pub(crate) use nrf52832_pac as pac;
#[cfg(feature = "nrf52833")]
pub(crate) use nrf52833_pac as pac;
#[cfg(feature = "nrf52840")]
pub(crate) use nrf52840_pac as pac;

mod ble;
mod conn;
mod radio;
mod timers;

pub use ble::{AdvParams, AdvType, Ble, BleEvent, FilterPolicy, ScanParams};
pub use conn::{DisconnectReason, LL_CONTROL_FEATURE_REQ, LL_CONTROL_FEATURE_RSP};
pub use radio::{Radio, TxPower};
