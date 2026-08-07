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

    let mut ble = Ble::new(p.RADIO, p.TIMER0, p.CCM);
    ble.gap_address_set([0xC1, 0xBE, 0xEF, 0x20, 0x25, 0x08], AddrType::RandomStatic);

    let target = [0xDF, 0xCD, 0x01, 0xD5, 0xE5, 0xEF];
    rprintln!("connecting to {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        target[0], target[1], target[2], target[3], target[4], target[5]);
    ble.gap_connect(target, AddrType::Public).unwrap();

    loop {
        while !ble.timer_compare_pending() {}
        ble.scan_tick();
        if let Some(BleEvent::Connected { conn }) = ble.next_event() {
            rprintln!(
                "CONNECTED: interval={} timeout={} aa={:08x}",
                conn.interval, conn.timeout, conn.access_addr
            );
            break;
        }
    }

    rprintln!("connection ok, requesting pairing (peer has no L2CAP, expect silence)");
    ble.gap_pair().ok();

    let mut ticks = 0u32;
    loop {
        while !ble.timer_compare_pending() {}
        if !ble.conn_tick() {
            rprintln!("DISCONNECTED");
            loop {
                cortex_m::asm::wfi();
            }
        }
        ticks += 1;
        if ticks % 500 == 0 {
            rprintln!("conn tick {}", ticks);
        }
        while let Some(evt) = ble.next_event() {
            rprintln!("evt: {:?}", evt);
        }
        if let Some((op, buf, len)) = ble.gatt_poll() {
            rprintln!("gatt: op=0x{:02x} {:02x?}", op, &buf[..len.min(20)]);
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
