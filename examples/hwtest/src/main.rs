#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52811_pac as pac;
use nrf_ble::hw::{Ble, BleEvent};
use nrf_ble::ll::addr::AddrType;
use rtt_target::{rtt_init_print, rprintln};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    rtt_init_print!();
    rprintln!("nrf-ble hwtest: boot");

    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });

    let mut ble = Ble::new(p.RADIO, p.TIMER0);
    ble.gap_address_set([0xC1, 0xBE, 0xEF, 0x20, 0x25, 0x08], AddrType::RandomStatic);

    let target = [0xDF, 0xCD, 0x01, 0xD5, 0xE5, 0xEF];
    rprintln!("nrf-ble hwtest: connecting to {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        target[0], target[1], target[2], target[3], target[4], target[5]);
    ble.gap_connect(target, AddrType::Public).unwrap();

    let mut ticks = 0u32;
    loop {
        while !ble.timer_compare_pending() {}
        ble.scan_tick();
        ticks += 1;
        if ticks % 200 == 0 {
            rprintln!("initiating: tick {}", ticks);
        }
        while let Some(evt) = ble.next_event() {
            match evt {
                BleEvent::Connected { conn } => {
                    rprintln!(
                        "CONNECTED (master): interval={} timeout={} aa={:08x} crc=0x{:06x}",
                        conn.interval, conn.timeout, conn.access_addr, conn.crc_init
                    );
                    rprintln!("driving connection as master");
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
                    rprintln!("connection closed");
                    loop {
                        cortex_m::asm::wfi();
                    }
                }
                other => rprintln!("evt: {:?}", other),
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
