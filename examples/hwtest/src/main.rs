#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nrf52811_pac as pac;
use nrf_ble::hw::{AdvParams, AdvType, Ble, BleEvent, ScanParams};
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
    let adv_data = [
        0x02, 0x01, 0x06,
        0x03, 0x03, 0x0D, 0x18,
        0x08, 0x09, b'n', b'r', b'f', b'-', b'b', b'l', b'e',
    ];
    ble.gap_adv_set_configure(&adv_data, &[]).unwrap();
    let params = AdvParams {
        interval_min: 160,
        interval_max: 160,
        adv_type: AdvType::ConnectableUndirected,
        ..Default::default()
    };
    ble.gap_adv_start(&params).unwrap();
    rprintln!("nrf-ble hwtest: advertising");

    let mut ticks = 0u32;
    'adv: loop {
        while !ble.timer_compare_pending() {}
        ble.adv_tick();
        ticks += 1;
        if ticks % 100 == 0 {
            rprintln!("adv tick {}", ticks);
        }
        if ticks == 3 {
            rprintln!("tx pdu: {:02x?}", ble.adv_pdu_snapshot());
        }
        while let Some(evt) = ble.next_event() {
            match evt {
                BleEvent::AdvStopped => break 'adv,
                _ => rprintln!("adv evt: {:?}", evt),
            }
        }
        if ticks == 150 {
            ble.gap_adv_stop().unwrap();
        }
    }
    rprintln!("nrf-ble hwtest: adv phase done");

    let scan_params = ScanParams {
        active: false,
        interval: 300,
        window: 250,
        ..Default::default()
    };
    ble.gap_scan_start(&scan_params).unwrap();
    rprintln!("nrf-ble hwtest: scanning");

    let mut st = 0u32;
    loop {
        while !ble.timer_compare_pending() {}
        ble.scan_tick();
        st += 1;
        if st % 200 == 0 {
            rprintln!("scan tick {}", st);
        }
        while let Some(evt) = ble.next_event() {
            match evt {
                BleEvent::ScanReport {
                    addr,
                    rssi,
                    adv_type,
                    data,
                    data_len,
                    ..
                } => {
                    rprintln!(
                        "report: addr={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} type=0x{:02x} rssi={} data={:02x?}",
                        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], adv_type, rssi,
                        &data[..data_len]
                    );
                }
                _ => {}
            }
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
