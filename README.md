# nrf-ble

An open-source Bluetooth Low Energy stack for Nordic nRF52 microcontrollers
(nRF52832, nRF52840), written from scratch in Rust as a **SoftDevice-free**
replacement for Nordic's proprietary BLE stack.

No Nordic SoftDevice binary, headers or code are used: everything is
implemented from the public Bluetooth Core Specification and the public
nRF52 Product Specification.

## Status

This is the foundation release. Working and tested (without hardware):

- **Link Layer**: BLE CRC-24 (validated against the published check value
  `0xC25A56`), data whitening, advertising and data channel PDU codecs,
  CONNECT_REQ LLData parsing, address handling, channel/frequency mapping
  and the hop sequence.
- **GAP**: advertising data (AD structure) codec.
- **Radio driver**: nRF52 RADIO peripheral configured for BLE 1 Mbit/s
  (packet framing, access address, 24-bit CRC, whitening), channel and TX
  power control, blocking TX / non-blocking RX.
- **Stack API**: SoftDevice-style API (`gap_adv_start`, `gap_adv_stop`,
  `gap_scan_start`, `gap_scan_stop`, events) with an advertising and
  scanning scheduler on TIMER0, including scan-request/scan-response
  exchange and CONNECT_REQ detection.
- **Simulator**: a host-side virtual radio (`sim` feature) that emulates
  the RADIO peripheral's data path (CRC append + whitening on TX,
  de-whitening + CRC validation on RX) so the whole stack can be tested on
  the host with `cargo test`.

Not yet implemented: connections (data channel scheduling, LL control
PDUs), GATT, L2CAP and pairing. The codec and parsing layers for these
already exist in `ll`.

## Usage

```toml
[dependencies]
nrf-ble = { git = "https://github.com/0xdilo/nrf-ble", features = ["nrf52832"] }
nrf52832-pac = "0.12"
```

```rust
use nrf_ble::hw::{AdvParams, Ble, BleEvent};
use nrf_ble::ll::addr::AddrType;

let p = nrf52832_pac::Peripherals::take().unwrap();
let mut ble = Ble::new(p.RADIO, p.TIMER0);

ble.gap_address_set([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF], AddrType::Public);
let adv_data = [
    0x02, 0x01, 0x06,                       // Flags: LE General Discoverable
    0x03, 0x03, 0x0D, 0x18,                 // 16-bit UUID: 0x180D
    0x09, b'n', b'r', b'f', b'-', b'b', b'l', b'e', // Name: "nrf-ble"
];
ble.gap_adv_set_configure(&adv_data, &[]).unwrap();
ble.gap_adv_start(&AdvParams::default()).unwrap();
ble.adv_forever(|_, evt| match evt {
    BleEvent::ConnectReqReceived { conn, .. } => { /* connection params parsed */ }
    _ => {}
});
```

The `Ble` API works with just the PAC (no HAL, no RTOS). For interrupt
driven use, call `adv_tick()`/`scan_tick()` from your TIMER0 handler and
drain events with `next_event()`.

## Testing without hardware

```sh
cargo test               # all codec + simulator tests on the host
cargo clippy --all-targets
cargo fmt --check
```

Cross-compile checks for the real target:

```sh
rustup target add thumbv7em-none-eabihf
cargo check --target thumbv7em-none-eabihf --features nrf52832
cargo check --target thumbv7em-none-eabihf --no-default-features --features nrf52840
```

## Hardware bring-up checklist

The driver register configuration follows the nRF52 Product Specification
and was cross-checked against Zephyr's production nRF52 BLE controller.
Untested-on-silicon items to verify on first hardware:

- PREAMBLE is derived by the nRF52 from the access address first bit
  (no PREAMBLE register exists on nRF52); PCNF0.PLEN selects 8-bit.
- Access address mapping: `PREFIX0.AP0 = AA[31:24]`,
  `BASE0 = (AA & 0x00FF_FFFF) << 8` (matches Zephyr).
- RADIO.FREQUENCY is an offset from 2400 MHz (e.g. 2 / 26 / 80 for the
  advertising channels).
- EVENTS are cleared by writing 0; TASKS are triggered by writing 1.

## Architecture

```
application
   │  Ble (gap_adv_start/stop, gap_scan_start/stop, next_event)
   ├── hw/radio   nRF52 RADIO driver (BLE 1 Mbit/s)
   ├── hw/timers  TIMER0 scheduler + interval tick math
   ├── ll/pdu     advertising & data channel PDU codecs
   ├── ll/crc     BLE CRC-24 (reflected, init 0x555555)
   ├── ll/whiten  BLE data whitening (LFSR, 0x40 + channel)
   ├── ll/addr    address types
   ├── ll/channels channel frequencies + hop sequence
   └── gap/ad     advertising data (AD) codec
sim/  host-only virtual radio for loopback tests (feature "sim")
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
