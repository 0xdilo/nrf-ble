#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52811_pac as pac;
use nrf_ble::hw::{AdvParams, AdvType, Ble, BleEvent};
use nrf_ble::ll::addr::AddrType;
use rtt_target::{rtt_init_print, rprintln};

fn adv_params() -> AdvParams {
    AdvParams {
        interval_min: 160,
        interval_max: 160,
        adv_type: AdvType::ConnectableUndirected,
        ..Default::default()
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    rtt_init_print!();
    rprintln!("nrf-ble hwtest: boot");

    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });

    let mut ble = Ble::new(p.RADIO, p.TIMER0, p.CCM);
    ble.gap_address_set([0x08, 0x25, 0x20, 0xEF, 0xBE, 0xC1], AddrType::RandomStatic);
    let adv_data = [
        0x02, 0x01, 0x06,
        0x08, 0x09, b'n', b'r', b'f', b'-', b'b', b'l', b'e',
    ];
    ble.gap_adv_set_configure(&adv_data, &[]).unwrap();

    loop {
        ble.gap_adv_start(&adv_params()).unwrap();
        rprintln!("nrf-ble hwtest: advertising, waiting for connection");

        ble.adv_forever(|ble, evt| {
            if let BleEvent::Connected { conn } = evt {
                rprintln!(
                    "CONNECTED: interval={} timeout={} aa={:08x} crc=0x{:06x}",
                    conn.interval, conn.timeout, conn.access_addr, conn.crc_init
                );
                ble.conn_send(b"hello from nrf-ble!").ok();
            }
        });

        rprintln!("nrf-ble hwtest: driving connection");
        ble.conn_forever(|ble, evt| match evt {
            BleEvent::ConnData => {
                let data = ble.conn_rx_data();
                let mut buf = [0u8; 20];
                let n = data.len();
                buf[..n].copy_from_slice(data);
                rprintln!("rx: {:?}", &buf[..n]);
                ble.conn_send(&buf[..n]).ok();
            }
            BleEvent::Disconnected { reason } => {
                rprintln!("DISCONNECTED: {:?}", reason);
            }
            other => rprintln!("conn evt: {:?}", other),
        });
        rprintln!("nrf-ble hwtest: connection closed, restarting");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
