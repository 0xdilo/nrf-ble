#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52811_pac as pac;
use nrf_ble::hw::{Ble, BleEvent, ScanParams};
use rtt_target::{rtt_init_print, rprintln};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    rtt_init_print!();
    rprintln!("nrf-ble hwtest: boot");
    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    let mut ble = Ble::new(p.RADIO, p.TIMER0, p.CCM);
    ble.gap_scan_start(&ScanParams {
        interval: 300,
        window: 250,
        ..Default::default()
    })
    .unwrap();
    rprintln!("nrf-ble hwtest: scanning for pc adv");
    loop {
        while !ble.timer_compare_pending() {}
        ble.scan_tick();
        while let Some(evt) = ble.next_event() {
            if let BleEvent::ScanReport {
                addr,
                adv_type,
                data,
                data_len,
                ..
            } = evt
            {
                rprintln!(
                    "report: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} type={} data={:02x?}",
                    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], adv_type,
                    &data[..data_len]
                );
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
