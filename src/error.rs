//! Error type shared across the stack.
use core::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Errors produced by the BLE stack.
pub enum Error {
    /// Output buffer too small for the operation.
    BufferTooSmall,
    /// Malformed or reserved PDU.
    InvalidPdu,
    /// PDU length field does not match the payload.
    InvalidLength,
    /// Radio channel outside the valid range (0-39).
    InvalidChannel,
    /// Invalid Bluetooth address.
    InvalidAddress,
    /// Parameter outside the valid range.
    InvalidParam,
    /// Operation is already running.
    AlreadyRunning,
    /// Operation is not running.
    NotRunning,
    /// Received packet failed the CRC check.
    CrcMismatch,
    /// Malformed advertising data.
    InvalidAd,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            Error::BufferTooSmall => "output buffer too small",
            Error::InvalidPdu => "invalid PDU",
            Error::InvalidLength => "invalid length",
            Error::InvalidChannel => "invalid channel",
            Error::InvalidAddress => "invalid address",
            Error::InvalidParam => "invalid parameter",
            Error::AlreadyRunning => "operation already running",
            Error::NotRunning => "operation not running",
            Error::CrcMismatch => "CRC check failed",
            Error::InvalidAd => "invalid advertising data",
        };
        f.write_str(s)
    }
}

impl core::error::Error for Error {}
