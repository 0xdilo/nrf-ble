use nrf_ble::gap::ad::{build, iter, AdEntry, AD_TYPE_COMPLETE_NAME, AD_TYPE_FLAGS};
use nrf_ble::ll::addr::{AddrType, BtAddr};
use nrf_ble::ll::pdu::AdvPdu;
use nrf_ble::sim::{loopback, VirtualRadio};

const ADV_ADDR: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
const SCAN_ADDR: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];

fn make_advertising_data() -> ([u8; 31], usize) {
    let entries = [
        AdEntry {
            ad_type: AD_TYPE_FLAGS,
            data: &[0x06],
        },
        AdEntry {
            ad_type: AD_TYPE_COMPLETE_NAME,
            data: b"nrf-ble",
        },
    ];
    let mut buf = [0u8; 31];
    let n = build(&entries, &mut buf).unwrap();
    (buf, n)
}

#[test]
fn advertiser_to_scanner_adv_ind_loopback() {
    let (adv_data, adv_len) = make_advertising_data();
    let pdu = AdvPdu::AdvInd {
        adv_addr: &ADV_ADDR,
        data: &adv_data[..adv_len],
    };
    let mut tx = [0u8; 39];
    let n = pdu.encode(&mut tx).unwrap();

    let rx = loopback(&tx[..n], 37).unwrap();
    let decoded = AdvPdu::decode(&rx.pdu[..rx.len]).unwrap();
    match decoded {
        AdvPdu::AdvInd { adv_addr, data } => {
            assert_eq!(adv_addr, &ADV_ADDR);
            assert_eq!(data, &adv_data[..adv_len]);
            let mut entries = 0;
            for ad in iter(data) {
                let ad = ad.unwrap();
                entries += 1;
                match ad.ad_type {
                    AD_TYPE_FLAGS => assert_eq!(ad.data, &[0x06]),
                    AD_TYPE_COMPLETE_NAME => assert_eq!(ad.data, b"nrf-ble"),
                    _ => panic!("unexpected AD type"),
                }
            }
            assert_eq!(entries, 2);
        }
        other => panic!("expected AdvInd, got {:?}", other),
    }
}

#[test]
fn scan_req_scan_rsp_exchange() {
    let tx_radio = VirtualRadio::new(38);
    let rx_radio = VirtualRadio::new(38);

    let req = AdvPdu::ScanReq {
        scan_addr: &SCAN_ADDR,
        adv_addr: &ADV_ADDR,
    };
    let mut buf = [0u8; 39];
    let n = req.encode(&mut buf).unwrap();
    let packet = tx_radio.transmit(&buf[..n]).unwrap();
    let rx = rx_radio.receive(&packet).unwrap();
    match AdvPdu::decode(&rx.pdu[..rx.len]).unwrap() {
        AdvPdu::ScanReq {
            scan_addr,
            adv_addr,
        } => {
            assert_eq!(scan_addr, &SCAN_ADDR);
            assert_eq!(adv_addr, &ADV_ADDR);
        }
        other => panic!("expected ScanReq, got {:?}", other),
    }

    let rsp = AdvPdu::ScanRsp {
        adv_addr: &ADV_ADDR,
        data: b"rsp",
    };
    let n = rsp.encode(&mut buf).unwrap();
    let packet = tx_radio.transmit(&buf[..n]).unwrap();
    let rx = rx_radio.receive(&packet).unwrap();
    match AdvPdu::decode(&rx.pdu[..rx.len]).unwrap() {
        AdvPdu::ScanRsp { adv_addr, data } => {
            assert_eq!(adv_addr, &ADV_ADDR);
            assert_eq!(data, b"rsp");
        }
        other => panic!("expected ScanRsp, got {:?}", other),
    }
}

#[test]
fn pdu_type_and_address_derivation() {
    let addr = BtAddr::parse(ADV_ADDR);
    assert_eq!(addr.addr_type, AddrType::RandomStatic);
    assert!(addr.tx_add_bit());
}
