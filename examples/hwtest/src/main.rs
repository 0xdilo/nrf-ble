#![no_std]
#![no_main]

use core::ptr;

use cortex_m_rt::entry;
use nrf52811_pac as pac;
use nrf_ble::hw::{AdvParams, AdvType, Ble, BleEvent, ScanParams};
use nrf_ble::ll::addr::AddrType;
use rtt_target::{rtt_init_print, rprintln};

const RADIO_BASE: *const u32 = 0x4000_1000 as *const u32;

unsafe fn rd(off: usize) -> u32 {
    ptr::read_volatile(RADIO_BASE.add(off / 4))
}

fn dump_radio_config() {
    unsafe {
        rprintln!(
            "RADIO cfg: mode=0x{:08x} pcnf0=0x{:08x} pcnf1=0x{:08x} crccnf=0x{:08x}",
            rd(0x510),
            rd(0x514),
            rd(0x518),
            rd(0x534)
        );
        rprintln!(
            "RADIO cfg: crcinit=0x{:06x} crcpoly=0x{:06x} txaddr=0x{:02x} rxaddr=0x{:02x}",
            rd(0x53C),
            rd(0x538),
            rd(0x52C),
            rd(0x530)
        );
    }
}

fn dump_radio_live(tick: u32) {
    unsafe {
        rprintln!(
            "adv tick {}: state={} freq={} whiteiv=0x{:02x} txpower=0x{:02x}",
            tick,
            rd(0x550),
            unsafe { rd(0x508) },
            rd(0x554) & 0xFF,
            rd(0x50C) & 0xFF,
        );
    }
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    rtt_init_print!();
    rprintln!("nrf-ble hwtest: boot");

    p.CLOCK.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });

    let mut ble = Ble::new(p.RADIO, p.TIMER0);
    rprintln!("nrf-ble hwtest: ble init ok");
    dump_radio_config();

    ble.gap_address_set([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF], AddrType::Public);
    let adv_data = [
        0x02, 0x01, 0x06, // Flags: LE General Discoverable
        0x03, 0x03, 0x0D, 0x18, // 16-bit UUID: 0x180D
        0x09, b'n', b'r', b'f', b'-', b'b', b'l', b'e', // Name: "nrf-ble"
    ];
    ble.gap_adv_set_configure(&adv_data, &[]).unwrap();
    let params = AdvParams {
        interval_min: 160,
        interval_max: 160,
        adv_type: AdvType::ConnectableUndirected,
        ..Default::default()
    };
    ble.gap_adv_start(&params).unwrap();
    rprintln!("nrf-ble hwtest: advertising started");

    let mut ticks = 0u32;
    'adv: loop {
        while !ble.timer_compare_pending() {}
        ble.adv_tick();
        ticks += 1;
        if ticks % 5 == 0 {
            rprintln!(
                "adv tick {} t={} ch_freq={}",
                ticks,
                ble.timer_now(),
                unsafe { rd(0x508) },
            );
        }
        while let Some(evt) = ble.next_event() {
            match evt {
                BleEvent::AdvStopped => {
                    rprintln!("nrf-ble hwtest: adv stopped");
                    break 'adv;
                }
                other => rprintln!("adv evt: {:?}", other),
            }
        }
        if ticks == 50 {
            ble.gap_adv_stop().unwrap();
            rprintln!("nrf-ble hwtest: stop requested");
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
    let mut last_freq = 0u32;
    'scan: loop {
        while !ble.timer_compare_pending() {}
        ble.scan_tick();
        st += 1;
        let freq = unsafe { rd(0x508) };
        if freq != last_freq {
            rprintln!("scan tick {}: listening on freq {} ({} us)", st, freq, ble.timer_now() * 32);
            last_freq = freq;
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
                        "scan report: addr={:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} rssi={} type=0x{:02x} data={:02x?}",
                        addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], rssi, adv_type,
                        &data[..data_len]
                    );
                }
                BleEvent::ScanStopped => {
                    rprintln!("nrf-ble hwtest: scan stopped");
                    break 'scan;
                }
                other => rprintln!("scan evt: {:?}", other),
            }
        }
        if st == 90 {
            ble.gap_scan_stop().unwrap();
            rprintln!("nrf-ble hwtest: stop requested");
        }
        if st >= 120 {
            rprintln!("nrf-ble hwtest: scan did not stop, aborting");
            break 'scan;
        }
    }
    rprintln!("nrf-ble hwtest: done");
    loop {
        cortex_m::asm::wfi();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("PANIC: {}", info);
    loop {}
}
