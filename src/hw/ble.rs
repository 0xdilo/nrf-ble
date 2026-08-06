//! High-level SoftDevice-style BLE API: advertising, scanning, events.
use crate::gap::ad::LEGACY_AD_DATA_MAX_LEN;
use crate::ll::addr::{AddrType, BtAddr};
use crate::ll::channels::ADV_CHANNELS;
use crate::ll::pdu::{self, AdvPdu, ConnectReqData};
use crate::Error;

use super::pac;
use super::radio::{Radio, TxPower};
use super::timers::{timeout_ticks, window_ticks, BtTimer, IntervalAccum};

const EVENT_QUEUE_LEN: usize = 8;
const RX_PDU_MAX: usize = 2 + 37;
const SCAN_REQ_WINDOW_MICROS: u32 = 300;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Advertising type (legacy advertising PDU).
pub enum AdvType {
    /// ADV_IND: connectable and scannable undirected.
    ConnectableUndirected,
    /// ADV_DIRECT_IND: connectable directed.
    ConnectableDirected,
    /// ADV_SCAN_IND: scannable, non-connectable.
    ScannableUndirected,
    /// ADV_NONCONN_IND: non-connectable, non-scannable.
    NonConnectableUndirected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Advertising/scanning filter policy.
pub enum FilterPolicy {
    /// Accept all devices (no accept-list filtering).
    AcceptAll,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Advertising parameters.
pub struct AdvParams {
    /// Minimum advertising interval in 0.625 ms units (32-16384).
    pub interval_min: u16,
    /// Maximum advertising interval in 0.625 ms units.
    pub interval_max: u16,
    /// Advertising type.
    pub adv_type: AdvType,
    /// Own address type.
    pub own_addr_type: AddrType,
    /// Peer address, required for directed advertising.
    pub peer_addr: Option<[u8; 6]>,
    /// Channel map: bit 0 = channel 37, bit 1 = 38, bit 2 = 39.
    pub channel_map: u8,
}

impl Default for AdvParams {
    fn default() -> Self {
        AdvParams {
            interval_min: 160,
            interval_max: 160,
            adv_type: AdvType::ConnectableUndirected,
            own_addr_type: AddrType::Public,
            peer_addr: None,
            channel_map: 0b111,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Scanning parameters.
pub struct ScanParams {
    /// Active scanning: send SCAN_REQ and report SCAN_RSP.
    pub active: bool,
    /// Scan interval in 0.625 ms units (4-16384).
    pub interval: u16,
    /// Scan window in 0.625 ms units (4-16384, <= interval).
    pub window: u16,
    /// Own address type.
    pub own_addr_type: AddrType,
    /// Filter policy.
    pub filter_policy: FilterPolicy,
}

impl Default for ScanParams {
    fn default() -> Self {
        ScanParams {
            active: false,
            interval: 80,
            window: 40,
            own_addr_type: AddrType::Public,
            filter_policy: FilterPolicy::AcceptAll,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Events reported by the stack to the application.
pub enum BleEvent {
    /// Advertising started.
    AdvStarted,
    /// Advertising stopped.
    AdvStopped,
    /// Scanning started.
    ScanStarted,
    /// Scanning stopped.
    ScanStopped,
    /// A scanner requested our scan response data.
    ScanReqReceived {
        /// Scanner address.
        addr: [u8; 6],
    },
    /// A connection request was received while advertising.
    ConnectReqReceived {
        /// Initiator address.
        init_addr: [u8; 6],
        /// Parsed connection parameters.
        conn: ConnectReqData,
    },
    /// An advertising or scan response packet was received while scanning.
    ScanReport {
        /// Advertiser address.
        addr: [u8; 6],
        /// Derived address type.
        addr_type: AddrType,
        /// PDU type of the received packet.
        adv_type: u8,
        /// Received signal strength in dBm.
        rssi: i8,
        /// Advertising data (31-byte max, legacy).
        data: [u8; 31],
        /// Number of valid data bytes.
        data_len: usize,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AdvState {
    Idle,
    Running,
    StopPending,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ScanState {
    Idle,
    Running,
    StopPending,
}

/// High-level BLE stack: owns the RADIO and TIMER0 peripherals.
pub struct Ble {
    radio: Radio,
    timer: BtTimer,
    adv_accum: IntervalAccum,
    scan_accum: IntervalAccum,
    adv_state: AdvState,
    scan_state: ScanState,
    adv_params: AdvParams,
    scan_params: ScanParams,
    adv_data: [u8; LEGACY_AD_DATA_MAX_LEN],
    adv_data_len: usize,
    scan_rsp: [u8; LEGACY_AD_DATA_MAX_LEN],
    scan_rsp_len: usize,
    own_addr: [u8; 6],
    own_addr_type: AddrType,
    adv_channel_idx: usize,
    scan_channel_idx: usize,
    tx_buf: [u8; RX_PDU_MAX],
    tx_pdu_len: usize,
    rx_buf: [u8; RX_PDU_MAX],
    rx_pdu_len: usize,
    events: [Option<BleEvent>; EVENT_QUEUE_LEN],
    event_head: usize,
    event_tail: usize,
}

impl Ble {
    /// Create a BLE stack instance, taking ownership of the RADIO and
    /// TIMER0 peripherals.
    pub fn new(radio: pac::RADIO, timer0: pac::TIMER0) -> Self {
        let ble = Ble {
            radio: Radio::new(radio),
            timer: BtTimer::new(timer0),
            adv_accum: IntervalAccum::new(),
            scan_accum: IntervalAccum::new(),
            adv_state: AdvState::Idle,
            scan_state: ScanState::Idle,
            adv_params: AdvParams::default(),
            scan_params: ScanParams::default(),
            adv_data: [0; LEGACY_AD_DATA_MAX_LEN],
            adv_data_len: 0,
            scan_rsp: [0; LEGACY_AD_DATA_MAX_LEN],
            scan_rsp_len: 0,
            own_addr: [0; 6],
            own_addr_type: AddrType::Public,
            adv_channel_idx: 0,
            scan_channel_idx: 0,
            tx_buf: [0; RX_PDU_MAX],
            tx_pdu_len: 0,
            rx_buf: [0; RX_PDU_MAX],
            rx_pdu_len: 0,
            events: [None; EVENT_QUEUE_LEN],
            event_head: 0,
            event_tail: 0,
        };
        ble.radio.init();
        ble.timer.init();
        ble
    }

    /// Set the device's own address.
    pub fn gap_address_set(&mut self, addr: [u8; 6], addr_type: AddrType) {
        self.own_addr = addr;
        self.own_addr_type = addr_type;
    }

    /// Configure advertising data and scan response data (31 bytes max each).
    pub fn gap_adv_set_configure(
        &mut self,
        adv_data: &[u8],
        scan_rsp_data: &[u8],
    ) -> Result<(), Error> {
        if adv_data.len() > LEGACY_AD_DATA_MAX_LEN || scan_rsp_data.len() > LEGACY_AD_DATA_MAX_LEN {
            return Err(Error::InvalidLength);
        }
        self.adv_data[..adv_data.len()].copy_from_slice(adv_data);
        self.adv_data_len = adv_data.len();
        self.scan_rsp[..scan_rsp_data.len()].copy_from_slice(scan_rsp_data);
        self.scan_rsp_len = scan_rsp_data.len();
        Ok(())
    }

    /// Start advertising.
    pub fn gap_adv_start(&mut self, params: &AdvParams) -> Result<(), Error> {
        if self.adv_state != AdvState::Idle {
            return Err(Error::AlreadyRunning);
        }
        if params.interval_min < 32 || params.interval_min > 16384 {
            return Err(Error::InvalidParam);
        }
        if params.interval_max < params.interval_min {
            return Err(Error::InvalidParam);
        }
        if params.channel_map & 0b111 == 0 {
            return Err(Error::InvalidParam);
        }
        if params.adv_type == AdvType::ConnectableDirected && params.peer_addr.is_none() {
            return Err(Error::InvalidParam);
        }
        self.adv_params = *params;
        self.own_addr_type = params.own_addr_type;
        self.adv_channel_idx = 0;
        self.adv_accum = IntervalAccum::new();
        let ticks = self.adv_accum.next(params.interval_min);
        let now = self.timer.now();
        self.timer.set_compare(now.wrapping_add(ticks));
        self.adv_state = AdvState::Running;
        self.push_event(BleEvent::AdvStarted);
        Ok(())
    }

    /// Request advertising to stop at the next event boundary.
    pub fn gap_adv_stop(&mut self) -> Result<(), Error> {
        match self.adv_state {
            AdvState::Running => {
                self.adv_state = AdvState::StopPending;
                Ok(())
            }
            AdvState::StopPending => Ok(()),
            AdvState::Idle => Err(Error::NotRunning),
        }
    }

    /// Advance the advertising state machine by one channel event.
    ///
    /// Call when the timer compare fires (polling or from an IRQ).
    pub fn adv_tick(&mut self) {
        if self.adv_state == AdvState::Idle {
            return;
        }
        self.timer.clear_compare();
        if self.adv_state == AdvState::StopPending {
            self.adv_state = AdvState::Idle;
            self.push_event(BleEvent::AdvStopped);
            return;
        }
        let mut ch_idx = self.adv_channel_idx;
        let mut channel = ADV_CHANNELS[ch_idx];
        for _ in 0..3 {
            if self.adv_params.channel_map & (1 << ch_idx) != 0 {
                break;
            }
            ch_idx = (ch_idx + 1) % 3;
            channel = ADV_CHANNELS[ch_idx];
        }
        self.adv_channel_idx = (ch_idx + 1) % 3;
        self.radio.set_channel(channel).ok();
        self.build_adv_pdu();
        self.radio.transmit(&self.tx_buf[..self.tx_pdu_len]);
        match self.adv_params.adv_type {
            AdvType::ConnectableUndirected | AdvType::ConnectableDirected => {
                self.listen_for_scan_and_connect();
            }
            AdvType::ScannableUndirected => {
                self.listen_for_scan_req();
            }
            AdvType::NonConnectableUndirected => {}
        }
        let ticks = self.adv_accum.next(self.adv_params.interval_min);
        let now = self.timer.now();
        self.timer.set_compare(now.wrapping_add(ticks));
    }

    /// Blocking advertising loop.
    ///
    /// Runs until [`Ble::gap_adv_stop`] is called from the event callback.
    pub fn adv_forever(&mut self, mut on_event: impl FnMut(&mut Self, BleEvent)) {
        loop {
            while !self.timer.compare_pending() {}
            self.adv_tick();
            self.drain_events(&mut |ble, evt| on_event(ble, evt));
            if self.adv_state == AdvState::Idle {
                break;
            }
        }
    }

    /// Start scanning.
    pub fn gap_scan_start(&mut self, params: &ScanParams) -> Result<(), Error> {
        if self.scan_state != ScanState::Idle {
            return Err(Error::AlreadyRunning);
        }
        if params.interval < 4 || params.interval > 16384 {
            return Err(Error::InvalidParam);
        }
        if params.window < 4 || params.window > 16384 {
            return Err(Error::InvalidParam);
        }
        if params.window > params.interval {
            return Err(Error::InvalidParam);
        }
        self.scan_params = *params;
        self.own_addr_type = params.own_addr_type;
        self.scan_channel_idx = 0;
        self.scan_accum = IntervalAccum::new();
        let ticks = self.scan_accum.next(params.interval);
        let now = self.timer.now();
        self.timer.set_compare(now.wrapping_add(ticks));
        self.scan_state = ScanState::Running;
        self.push_event(BleEvent::ScanStarted);
        Ok(())
    }

    /// Request scanning to stop at the next event boundary.
    pub fn gap_scan_stop(&mut self) -> Result<(), Error> {
        match self.scan_state {
            ScanState::Running => {
                self.scan_state = ScanState::StopPending;
                Ok(())
            }
            ScanState::StopPending => Ok(()),
            ScanState::Idle => Err(Error::NotRunning),
        }
    }

    /// Advance the scanning state machine by one channel listen.
    ///
    /// Call when the timer compare fires (polling or from an IRQ).
    pub fn scan_tick(&mut self) {
        if self.scan_state == ScanState::Idle {
            return;
        }
        self.timer.clear_compare();
        if self.scan_state == ScanState::StopPending {
            self.scan_state = ScanState::Idle;
            self.push_event(BleEvent::ScanStopped);
            return;
        }
        if self.scan_channel_idx >= 3 {
            self.scan_channel_idx = 0;
            let ticks = self.scan_accum.next(self.scan_params.interval);
            let now = self.timer.now();
            self.timer.set_compare(now.wrapping_add(ticks));
            return;
        }
        let channel = ADV_CHANNELS[self.scan_channel_idx];
        self.scan_channel_idx += 1;
        self.radio.set_channel(channel).ok();
        let per_channel = window_ticks(self.scan_params.window) / 3;
        let deadline = self.timer.now().wrapping_add(per_channel);
        self.radio.receive_start(&mut self.rx_buf);
        loop {
            match self.radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    self.rx_pdu_len = len;
                    self.handle_scan_pdu();
                    break;
                }
                Ok(None) => {
                    if self.timer.now() >= deadline {
                        self.radio.receive_cancel();
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    }

    /// Blocking scanning loop.
    ///
    /// Runs until [`Ble::gap_scan_stop`] is called from the event callback.
    pub fn scan_forever(&mut self, mut on_event: impl FnMut(&mut Self, BleEvent)) {
        loop {
            while !self.timer.compare_pending() {}
            self.scan_tick();
            self.drain_events(&mut |ble, evt| on_event(ble, evt));
            if self.scan_state == ScanState::Idle {
                break;
            }
        }
    }

    /// Pop the oldest pending event, if any.
    pub fn next_event(&mut self) -> Option<BleEvent> {
        let evt = self.events[self.event_tail].take();
        if evt.is_some() {
            self.event_tail = (self.event_tail + 1) % EVENT_QUEUE_LEN;
        }
        evt
    }

    /// Set the radio TX power.
    pub fn gap_tx_power_set(&mut self, power: TxPower) {
        self.radio.set_tx_power(power);
    }

    fn build_adv_pdu(&mut self) {
        let len = match self.adv_params.adv_type {
            AdvType::ConnectableDirected => {
                let peer = self.adv_params.peer_addr.unwrap();
                AdvPdu::AdvDirectInd {
                    adv_addr: &self.own_addr,
                    init_addr: &peer,
                }
                .encode(&mut self.tx_buf)
                .unwrap()
            }
            AdvType::ConnectableUndirected => AdvPdu::AdvInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode(&mut self.tx_buf)
            .unwrap(),
            AdvType::ScannableUndirected => AdvPdu::AdvScanInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode(&mut self.tx_buf)
            .unwrap(),
            AdvType::NonConnectableUndirected => AdvPdu::AdvNonconnInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode(&mut self.tx_buf)
            .unwrap(),
        };
        self.tx_pdu_len = len;
    }

    fn listen_for_scan_req(&mut self) {
        if self.radio_window(1 << pdu::PDU_SCAN_REQ) && self.scan_rsp_len > 0 {
            let len = AdvPdu::ScanRsp {
                adv_addr: &self.own_addr,
                data: &self.scan_rsp[..self.scan_rsp_len],
            }
            .encode(&mut self.tx_buf)
            .unwrap();
            self.radio.transmit(&self.tx_buf[..len]);
        }
    }

    fn listen_for_scan_and_connect(&mut self) {
        let accept = (1 << pdu::PDU_SCAN_REQ) | (1 << pdu::PDU_CONNECT_REQ);
        if !self.radio_window(accept) {
            return;
        }
        match AdvPdu::decode(&self.rx_buf[..self.rx_pdu_len]) {
            Ok(AdvPdu::ScanReq { adv_addr, .. }) => {
                if self.scan_rsp_len > 0 {
                    let len = AdvPdu::ScanRsp {
                        adv_addr,
                        data: &self.scan_rsp[..self.scan_rsp_len],
                    }
                    .encode(&mut self.tx_buf)
                    .unwrap();
                    self.radio.transmit(&self.tx_buf[..len]);
                }
            }
            Ok(AdvPdu::ConnectReq {
                init_addr, ll_data, ..
            }) => {
                if let Ok(conn) = ConnectReqData::decode(ll_data) {
                    self.push_event(BleEvent::ConnectReqReceived {
                        init_addr: *init_addr,
                        conn,
                    });
                    self.adv_state = AdvState::StopPending;
                }
            }
            _ => {}
        }
    }

    fn radio_window(&mut self, accept_mask: u32) -> bool {
        let deadline = self
            .timer
            .now()
            .wrapping_add(timeout_ticks(SCAN_REQ_WINDOW_MICROS));
        self.radio.receive_start(&mut self.rx_buf);
        loop {
            match self.radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    self.rx_pdu_len = len;
                    if self.rx_pdu_len >= 2 {
                        let pdu_type = self.rx_buf[0] & 0x0F;
                        return accept_mask & (1 << pdu_type) != 0;
                    }
                    return false;
                }
                Ok(None) => {
                    if self.timer.now() >= deadline {
                        self.radio.receive_cancel();
                        return false;
                    }
                }
                Err(_) => return false,
            }
        }
    }

    fn handle_scan_pdu(&mut self) {
        let rssi = self.radio.rssi();
        let mut report = None;
        let mut respond = false;
        if let Ok(pdu) = AdvPdu::decode(&self.rx_buf[..self.rx_pdu_len]) {
            match pdu {
                AdvPdu::AdvInd { adv_addr, data } | AdvPdu::AdvScanInd { adv_addr, data } => {
                    let addr = *adv_addr;
                    let mut buf = [0u8; 31];
                    let data_len = data.len().min(31);
                    buf[..data_len].copy_from_slice(&data[..data_len]);
                    report = Some((addr, pdu.pdu_type(), buf, data_len));
                    respond = self.scan_params.active;
                }
                _ => {}
            }
        }
        if let Some((addr, adv_type, data, data_len)) = report {
            self.push_event(BleEvent::ScanReport {
                addr,
                addr_type: BtAddr::parse(addr).addr_type,
                adv_type,
                rssi,
                data,
                data_len,
            });
        }
        if respond {
            self.respond_scan_req();
        }
    }

    fn respond_scan_req(&mut self) {
        let adv_addr: [u8; 6] = match self.rx_buf[2..8].try_into() {
            Ok(a) => a,
            Err(_) => return,
        };
        let len = match (AdvPdu::ScanReq {
            scan_addr: &self.own_addr,
            adv_addr: &adv_addr,
        })
        .encode(&mut self.tx_buf)
        {
            Ok(len) => len,
            Err(_) => return,
        };
        self.radio.transmit(&self.tx_buf[..len]);
        self.listen_for_scan_rsp();
    }

    fn listen_for_scan_rsp(&mut self) {
        let deadline = self
            .timer
            .now()
            .wrapping_add(timeout_ticks(SCAN_REQ_WINDOW_MICROS));
        self.radio.receive_start(&mut self.rx_buf);
        loop {
            match self.radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    if let Ok(AdvPdu::ScanRsp { adv_addr, data }) =
                        AdvPdu::decode(&self.rx_buf[..len])
                    {
                        let addr = *adv_addr;
                        let mut report = [0u8; 31];
                        let data_len = data.len().min(31);
                        report[..data_len].copy_from_slice(&data[..data_len]);
                        let rssi = self.radio.rssi();
                        self.push_event(BleEvent::ScanReport {
                            addr,
                            addr_type: BtAddr::parse(addr).addr_type,
                            adv_type: pdu::PDU_SCAN_RSP,
                            rssi,
                            data: report,
                            data_len,
                        });
                    }
                    break;
                }
                Ok(None) => {
                    if self.timer.now() >= deadline {
                        self.radio.receive_cancel();
                        break;
                    }
                }
                Err(_) => break,
            }
        }
    }

    fn push_event(&mut self, evt: BleEvent) {
        let next = (self.event_head + 1) % EVENT_QUEUE_LEN;
        if next == self.event_tail {
            self.event_tail = (self.event_tail + 1) % EVENT_QUEUE_LEN;
        }
        self.events[self.event_head] = Some(evt);
        self.event_head = next;
    }

    fn drain_events(&mut self, on_event: &mut impl FnMut(&mut Self, BleEvent)) {
        while let Some(evt) = self.next_event() {
            on_event(self, evt);
        }
    }
}
