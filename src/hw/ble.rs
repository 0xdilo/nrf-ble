//! High-level SoftDevice-style BLE API: advertising, scanning, events.
use crate::gap::ad::LEGACY_AD_DATA_MAX_LEN;
use crate::ll::addr::{AddrType, BtAddr};
use crate::ll::channels::ADV_CHANNELS;
use crate::ll::pdu::{self, AdvPdu, ConnectReqData, PDU_ADV_EXT_IND};
use crate::Error;

use super::conn::{BondInfo, BondStore, Conn, ConnEvent, ConnRole, DisconnectReason};
use super::pac;
use super::radio::{Radio, TxPower};
use super::timers::{timeout_ticks, window_ticks, BtTimer, IntervalAccum};

const EVENT_QUEUE_LEN: usize = 8;
const RX_PDU_MAX: usize = 2 + 255;
const SCAN_REQ_WINDOW_MICROS: u32 = 300;
const CONNECT_REQ_WINDOW_MICROS: u32 = 90_000;

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
    /// Only accept devices present in the accept list.
    AcceptList,
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
    /// Extended advertising: ADV_EXT_IND on the primary channels plus an
    /// auxiliary packet on a data channel carrying the full payload.
    pub extended: bool,
    /// Periodic advertising: advertise the periodic info in the extended
    /// header and send periodic packets on data channels.
    pub periodic: bool,
    /// Periodic advertising interval in 1.25 ms units.
    pub periodic_interval: u16,
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
            extended: false,
            periodic: false,
            periodic_interval: 100,
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
    /// A connection was established (peripheral role).
    Connected {
        /// Connection parameters from the CONNECT_REQ.
        conn: ConnectReqData,
    },
    /// The connection was closed.
    Disconnected {
        /// Why the connection ended.
        reason: DisconnectReason,
    },
    /// The peer wrote data to the NUS RX characteristic.
    ConnData,
    /// The peer sent an LL control PDU (opcode).
    LlControl(u8),
    /// The peer sent an L2CAP PDU on a non-ATT channel (channel ID).
    L2cap(u16),
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
    scan_cycle_start: u32,
    conn: Option<Conn>,
    connect_target: Option<[u8; 6]>,
    connect_target_type: AddrType,
    last_radio_channel: Option<u8>,
    accept_list: crate::ll::accept_list::AcceptList,
    irk: Option<[u8; 16]>,
    ccm: &'static pac::ccm::RegisterBlock,
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
    pub fn new(radio: pac::RADIO, timer0: pac::TIMER0, ccm: pac::CCM) -> Self {
        let ccm_regs: &'static pac::ccm::RegisterBlock = unsafe { &*pac::CCM::ptr() };
        let _ = ccm;
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
            scan_cycle_start: 0,
            conn: None,
            connect_target: None,
            connect_target_type: AddrType::Public,
            last_radio_channel: None,
            accept_list: crate::ll::accept_list::AcceptList::new(),
            irk: None,
            ccm: ccm_regs,
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
        if self.adv_params.extended {
            self.build_ext_adv_pdu(channel);
        } else {
            self.build_adv_pdu();
        }
        self.radio.transmit(&self.tx_buf[..self.tx_pdu_len]);
        if self.adv_params.extended {
            self.transmit_aux_adv();
            if self.adv_params.periodic {
                self.transmit_periodic_adv();
            }
        } else {
            match self.adv_params.adv_type {
                AdvType::ConnectableUndirected | AdvType::ConnectableDirected => {
                    self.listen_for_scan_and_connect(channel);
                }
                AdvType::ScannableUndirected => {
                    self.listen_for_scan_req();
                }
                AdvType::NonConnectableUndirected => {}
            }
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
            while !self.timer.compare_pending() {
                #[cfg(feature = "wfi")]
                cortex_m::asm::wfi();
            }
            self.adv_tick();
            self.drain_events(&mut |ble, evt| on_event(ble, evt));
            if self.adv_state == AdvState::Idle {
                break;
            }
        }
    }

    /// Start scanning.
    /// Connect to a specific advertiser (initiator role).
    pub fn gap_connect(&mut self, target: [u8; 6], target_type: AddrType) -> Result<(), Error> {
        if self.scan_state != ScanState::Idle {
            return Err(Error::AlreadyRunning);
        }
        self.connect_target = Some(target);
        self.connect_target_type = target_type;
        let params = ScanParams {
            active: false,
            interval: 100,
            window: 90,
            ..Default::default()
        };
        self.gap_scan_start(&params)
    }

    /// Save a bond (LTK/IRK keyed by peer address) to the store.
    pub fn gap_bond_save(&mut self, peer: [u8; 6], ltk: [u8; 16], irk: Option<[u8; 16]>) {
        if let Some(conn) = &mut self.conn {
            conn.bond_store.save(peer, ltk, irk);
        }
    }

    /// Look up a bond by peer address.
    pub fn gap_bond_find(&mut self, peer: &[u8; 6]) -> Option<BondInfo> {
        match &mut self.conn {
            Some(conn) => conn.bond_store.find(peer),
            None => None,
        }
    }

    /// Add a device to the accept list (max 8 entries).
    pub fn gap_accept_list_add(&mut self, addr: [u8; 6], addr_type: AddrType) -> bool {
        self.accept_list.add(addr, addr_type)
    }

    /// Set the IRK used for resolvable private address rotation.
    pub fn gap_privacy_set(&mut self, irk: [u8; 16]) {
        self.irk = Some(irk);
    }

    /// Rotate the own address to a fresh resolvable private address
    /// derived from the IRK (and the current timer value as the prand).
    ///
    /// Call periodically while advertising to keep the address rotating.
    pub fn gap_rotate_rpa(&mut self) -> bool {
        let Some(irk) = self.irk else {
            return false;
        };
        let prand = self.timer.now() & 0x00FF_FFFF;
        let rpa = crate::ll::priv_::generate_rpa(&irk, prand);
        self.own_addr = rpa;
        self.own_addr_type = AddrType::RandomPrivateResolvable;
        true
    }

    /// Clear the accept list.
    pub fn gap_accept_list_clear(&mut self) {
        self.accept_list.clear();
    }

    /// Number of entries in the accept list.
    pub fn gap_accept_list_len(&self) -> usize {
        self.accept_list.len()
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
            self.timer
                .set_compare(self.scan_cycle_start.wrapping_add(ticks));
            return;
        }
        if self.scan_channel_idx == 0 {
            self.scan_cycle_start = self.timer.now();
        }
        let channel = ADV_CHANNELS[self.scan_channel_idx];
        self.scan_channel_idx += 1;
        self.last_radio_channel = Some(channel);
        self.radio.set_channel(channel).ok();
        let per_channel = window_ticks(self.scan_params.window) / 3;
        let deadline = self.timer.now().wrapping_add(per_channel);
        self.timer.set_compare(deadline);
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
            while !self.timer.compare_pending() {
                #[cfg(feature = "wfi")]
                cortex_m::asm::wfi();
            }
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

    /// The last advertising PDU built for transmission.
    pub fn adv_pdu_snapshot(&self) -> &[u8] {
        &self.tx_buf[..self.tx_pdu_len]
    }

    /// Current BLE scheduler time in 32 us ticks.
    pub fn timer_now(&self) -> u32 {
        self.timer.now()
    }

    /// True when a scheduler compare event is pending (i.e. `adv_tick` or
    /// `scan_tick` is due).
    pub fn timer_compare_pending(&self) -> bool {
        self.timer.compare_pending()
    }

    fn build_adv_pdu(&mut self) {
        let tx_add = self.own_addr_type.is_random();
        let len = match self.adv_params.adv_type {
            AdvType::ConnectableDirected => {
                let peer = self.adv_params.peer_addr.unwrap();
                AdvPdu::AdvDirectInd {
                    adv_addr: &self.own_addr,
                    init_addr: &peer,
                }
                .encode_typed(&mut self.tx_buf, tx_add, false)
                .unwrap()
            }
            AdvType::ConnectableUndirected => AdvPdu::AdvInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap(),
            AdvType::ScannableUndirected => AdvPdu::AdvScanInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap(),
            AdvType::NonConnectableUndirected => AdvPdu::AdvNonconnInd {
                adv_addr: &self.own_addr,
                data: &self.adv_data[..self.adv_data_len],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap(),
        };
        self.tx_pdu_len = len;
    }

    fn build_ext_adv_pdu(&mut self, primary_channel: u8) {
        let aux = crate::ll::pdu::AuxPtr {
            channel: 0,
            offset: 5,
            phy: 0,
        };
        let mode = match self.adv_params.adv_type {
            AdvType::ConnectableUndirected => 0b001,
            AdvType::ScannableUndirected => 0b010,
            _ => 0,
        };
        let tx_add = self.own_addr_type.is_random();
        let len = AdvPdu::AdvExtInd {
            adv_addr: &self.own_addr,
            adv_mode: mode,
            adi: [0, 0],
            aux_ptr: aux,
        }
        .encode_typed(&mut self.tx_buf, tx_add, false)
        .unwrap();
        self.tx_pdu_len = len;
        let _ = primary_channel;
    }

    fn transmit_aux_adv(&mut self) {
        let tx_add = self.own_addr_type.is_random();
        let mut data = [0u8; 36];
        let n = self.adv_data_len.min(36);
        data[..n].copy_from_slice(&self.adv_data[..n]);
        let len = if self.adv_params.periodic {
            let info = crate::ll::pdu::PeriodicAdvInfo {
                adi: [0, 0],
                interval: self.adv_params.periodic_interval,
                adva_type: if self.own_addr_type.is_random() { 1 } else { 0 },
            };
            let mut ehs = [0u8; 40];
            ehs[0] = 0;
            ehs[1] = crate::ll::pdu::EXT_AD_PERIODIC;
            ehs[2..7].copy_from_slice(&info.encode());
            AdvPdu::AuxAdvExtIndWithEh {
                adv_addr: &self.own_addr,
                adi: [0, 0],
                ehs: &ehs[..7],
                data: &data[..n],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap()
        } else {
            AdvPdu::AuxAdvExtInd {
                adv_addr: &self.own_addr,
                adi: [0, 0],
                ext_type: 0x00,
                data: &data[..n],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap()
        };
        let deadline = self.timer.now().wrapping_add(timeout_ticks(200));
        self.timer.set_compare(deadline);
        while self.timer.now() < deadline {}
        self.timer.clear_compare();
        self.radio.set_channel(0).ok();
        self.radio.transmit(&self.tx_buf[..len]);
        self.radio
            .set_channel(crate::ll::channels::ADV_CHANNELS[0])
            .ok();
    }

    fn transmit_periodic_adv(&mut self) {
        let tx_add = self.own_addr_type.is_random();
        let len = AdvPdu::PeriodicAdv {
            adv_mode: 0,
            adi: [0, 0],
        }
        .encode_typed(&mut self.tx_buf, tx_add, false)
        .unwrap();
        let deadline = self.timer.now().wrapping_add(timeout_ticks(200));
        self.timer.set_compare(deadline);
        while self.timer.now() < deadline {}
        self.timer.clear_compare();
        self.radio.set_channel(0).ok();
        self.radio.transmit(&self.tx_buf[..len]);
        self.radio
            .set_channel(crate::ll::channels::ADV_CHANNELS[0])
            .ok();
    }

    fn listen_for_scan_req(&mut self) {
        if self.radio_window(1 << pdu::PDU_SCAN_REQ, SCAN_REQ_WINDOW_MICROS)
            && self.scan_rsp_len > 0
        {
            let tx_add = self.own_addr_type.is_random();
            let len = AdvPdu::ScanRsp {
                adv_addr: &self.own_addr,
                data: &self.scan_rsp[..self.scan_rsp_len],
            }
            .encode_typed(&mut self.tx_buf, tx_add, false)
            .unwrap();
            self.radio.transmit(&self.tx_buf[..len]);
        }
    }

    fn listen_for_scan_and_connect(&mut self, channel: u8) {
        let accept = (1 << pdu::PDU_SCAN_REQ) | (1 << pdu::PDU_CONNECT_REQ);
        if !self.radio_window(accept, CONNECT_REQ_WINDOW_MICROS) {
            return;
        }
        match AdvPdu::decode(&self.rx_buf[..self.rx_pdu_len]) {
            Ok(AdvPdu::ScanReq { adv_addr, .. }) => {
                if self.scan_rsp_len > 0 {
                    let tx_add = self.own_addr_type.is_random();
                    let len = AdvPdu::ScanRsp {
                        adv_addr,
                        data: &self.scan_rsp[..self.scan_rsp_len],
                    }
                    .encode_typed(&mut self.tx_buf, tx_add, false)
                    .unwrap();
                    self.radio.transmit(&self.tx_buf[..len]);
                }
            }
            Ok(AdvPdu::ConnectReq {
                init_addr, ll_data, ..
            }) => {
                if let Ok(params) = ConnectReqData::decode(ll_data) {
                    let init = *init_addr;
                    let now = self.timer.now();
                    self.conn = Some(Conn::new(&params, channel, now, ConnRole::Slave, self.ccm));
                    self.adv_state = AdvState::Idle;
                    self.push_event(BleEvent::Connected { conn: params });
                    self.push_event(BleEvent::ConnectReqReceived {
                        init_addr: init,
                        conn: params,
                    });
                }
            }
            _ => {}
        }
    }

    fn radio_window(&mut self, accept_mask: u32, window_micros: u32) -> bool {
        let deadline = self.timer.now().wrapping_add(timeout_ticks(window_micros));
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
                    if let Some(target) = self.connect_target {
                        if addr == target {
                            self.send_connect_req(true);
                            return;
                        }
                    }
                    if self.scan_params.filter_policy == FilterPolicy::AcceptList
                        && !self
                            .accept_list
                            .contains(addr, BtAddr::parse(addr).addr_type)
                    {
                        return;
                    }
                    let mut buf = [0u8; 31];
                    let data_len = data.len().min(31);
                    buf[..data_len].copy_from_slice(&data[..data_len]);
                    report = Some((addr, pdu.pdu_type(), buf, data_len));
                    respond = self.scan_params.active;
                }
                AdvPdu::AdvExtInd {
                    adv_addr, aux_ptr, ..
                } => {
                    let addr = *adv_addr;
                    let ptr = aux_ptr;
                    if let Some((aux_addr, aux_data, aux_len)) = self.follow_aux(&ptr) {
                        let mut buf = [0u8; 31];
                        let data_len = aux_len.min(31);
                        buf[..data_len].copy_from_slice(&aux_data[..data_len]);
                        report = Some((addr, PDU_ADV_EXT_IND, buf, data_len));
                        let _ = aux_addr;
                    }
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

    fn radio_last_channel(&self) -> Option<u8> {
        self.last_radio_channel
    }

    fn send_connect_req(&mut self, scannable: bool) {
        let target = match self.connect_target {
            Some(t) => t,
            None => return,
        };
        let access_addr = 0x8E89_BED6 ^ 0x12_34_56_78;
        let crc_init = 0x65_43_21;
        let ll = ConnectReqData {
            access_addr,
            crc_init,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        let mut llbuf = [0u8; 22];
        if ll.encode(&mut llbuf).is_err() {
            return;
        }
        let rx_add = self.connect_target_type.is_random();
        let len = match (AdvPdu::ConnectReq {
            init_addr: &self.own_addr,
            adv_addr: &target,
            ll_data: &llbuf,
        })
        .encode_typed(&mut self.tx_buf, self.own_addr_type.is_random(), rx_add)
        {
            Ok(len) => len,
            Err(_) => return,
        };
        self.radio.transmit(&self.tx_buf[..len]);
        self.connect_target = None;
        let now = self.timer.now();
        let channel = self.radio_last_channel().unwrap_or(37);
        self.conn = Some(Conn::new(&ll, channel, now, ConnRole::Master, self.ccm));
        self.scan_state = ScanState::Idle;
        self.push_event(BleEvent::Connected { conn: ll });
        let _ = scannable;
    }

    fn follow_aux(&mut self, ptr: &crate::ll::pdu::AuxPtr) -> Option<([u8; 6], [u8; 40], usize)> {
        self.radio.set_channel(ptr.channel).ok()?;
        let deadline = self.timer.now().wrapping_add(timeout_ticks(300));
        self.timer.set_compare(deadline);
        while self.timer.now() < deadline {}
        self.timer.clear_compare();
        self.radio.receive_start(&mut self.rx_buf);
        let mut spins = 0u32;
        loop {
            match self.radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    if let Ok(AdvPdu::AuxAdvExtInd { adv_addr, data, .. }) =
                        AdvPdu::decode(&self.rx_buf[..len])
                    {
                        let mut out = [0u8; 40];
                        let n = data.len().min(40);
                        out[..n].copy_from_slice(&data[..n]);
                        return Some((*adv_addr, out, n));
                    }
                    break;
                }
                Ok(None) => {
                    spins += 1;
                    if spins > 200_000 {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        self.radio.receive_cancel();
        self.radio
            .set_channel(crate::ll::channels::ADV_CHANNELS[0])
            .ok();
        None
    }

    fn respond_scan_req(&mut self) {
        let adv_addr: [u8; 6] = match self.rx_buf[2..8].try_into() {
            Ok(a) => a,
            Err(_) => return,
        };
        let tx_add = self.own_addr_type.is_random();
        let rx_add = BtAddr::parse(adv_addr).addr_type.is_random();
        let len = match (AdvPdu::ScanReq {
            scan_addr: &self.own_addr,
            adv_addr: &adv_addr,
        })
        .encode_typed(&mut self.tx_buf, tx_add, rx_add)
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

    /// Drive one connection event. Returns `false` once the link closed
    /// (a `Disconnected` event is queued).
    pub fn conn_tick(&mut self) -> bool {
        let (outcome, outcome_raw) = if let Some(conn) = &mut self.conn {
            let timeout = conn.timeout;
            let now = self.timer.now();
            let elapsed = now.wrapping_sub(conn.last_rx);
            let limit = u32::from(timeout) * 313;
            if elapsed >= limit {
                (Some(DisconnectReason::SupervisionTimeout), None)
            } else {
                match conn.event(&self.radio, &self.timer) {
                    Ok(ConnEvent::Disconnected(reason)) => (Some(reason), None),
                    Ok(other) => (None, Some(other)),
                    Err(_) => (Some(DisconnectReason::LocalError), None),
                }
            }
        } else {
            (Some(DisconnectReason::LocalError), None)
        };
        if let Some(reason) = outcome {
            self.conn = None;
            self.radio.init();
            self.push_event(BleEvent::Disconnected { reason });
            return false;
        }
        match outcome_raw {
            Some(ConnEvent::Control(op)) => {
                self.push_event(BleEvent::LlControl(op));
            }
            Some(ConnEvent::L2cap(cid)) => {
                self.push_event(BleEvent::L2cap(cid));
            }
            _ => {}
        }
        let has_data = self.conn.as_ref().is_some_and(|c| c.rx_data_len > 0);
        if has_data {
            if let Some(conn) = &mut self.conn {
                conn.rx_data_len = 0;
            }
            self.push_event(BleEvent::ConnData);
        }
        true
    }

    /// Drive the connection: run until the link is closed.
    ///
    /// The callback receives connection events; when `ConnData` is reported,
    /// the peer's write can be read via [`Ble::conn_rx_data`].
    pub fn conn_forever(&mut self, mut on_event: impl FnMut(&mut Self, BleEvent)) {
        while self.conn_tick() {
            #[cfg(feature = "wfi")]
            if !self.timer.compare_pending() {
                cortex_m::asm::wfi();
            }
            while let Some(evt) = self.next_event() {
                on_event(self, evt);
            }
        }
        while let Some(evt) = self.next_event() {
            on_event(self, evt);
        }
    }

    /// True when the current connection was initiated by us.
    pub fn conn_is_master(&self) -> bool {
        matches!(self.conn.as_ref().map(|c| c.role), Some(ConnRole::Master))
    }

    /// Data written by the peer to the NUS RX characteristic.
    pub fn conn_rx_data(&self) -> &[u8] {
        match &self.conn {
            Some(conn) => &conn.rx_data[..conn.rx_data_len],
            None => &[],
        }
    }

    /// Queue a notification on the NUS TX characteristic.
    pub fn conn_send(&mut self, data: &[u8]) -> Result<(), Error> {
        match &mut self.conn {
            Some(conn) => conn.queue_notify(data),
            None => Err(Error::NotRunning),
        }
    }

    /// L2CAP: open a credit-based connection-oriented channel (initiator).
    pub fn l2cap_connect(&mut self, psm: u16) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        conn.l2cap_connect(psm);
        Ok(())
    }

    /// L2CAP: send data on the open connection-oriented channel.
    pub fn l2cap_send(&mut self, data: &[u8]) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        conn.l2cap_send(data)
    }

    /// Start legacy Just Works pairing (master role: send the SMP pairing
    /// request; slave role: respond to the peer's request automatically).
    pub fn gap_pair(&mut self) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        if conn.role == ConnRole::Master {
            let mut req = [0u8; 7];
            req.copy_from_slice(&conn.smp.build_pairing_request());
            let mut l2 = [0u8; 11];
            l2[0..2].copy_from_slice(&7u16.to_le_bytes());
            l2[2..4].copy_from_slice(&super::smp::L2CAP_SMP_CID.to_le_bytes());
            l2[4..11].copy_from_slice(&req);
            conn.queue_l2cap(&l2, super::smp::L2CAP_SMP_CID);
        } else {
            conn.smp.state = super::smp::SmpState::Idle;
        }
        Ok(())
    }

    /// GATT client: discover primary services (READ_BY_GROUP_TYPE, 0x2800).
    pub fn gatt_discover_primary_services(&mut self) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        let mut req = [0u8; 7];
        req[0] = 0x10;
        req[1..3].copy_from_slice(&0x2800u16.to_le_bytes());
        req[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
        req[5..7].copy_from_slice(&0xFFFFu16.to_le_bytes());
        conn.gatt_send_request(&req);
        Ok(())
    }

    /// GATT client: discover characteristics within a handle range.
    pub fn gatt_discover_characteristics(&mut self, start: u16, end: u16) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        let mut req = [0u8; 7];
        req[0] = 0x08;
        req[1..3].copy_from_slice(&0x2803u16.to_le_bytes());
        req[3..5].copy_from_slice(&start.to_le_bytes());
        req[5..7].copy_from_slice(&end.to_le_bytes());
        conn.gatt_send_request(&req);
        Ok(())
    }

    /// GATT client: read an attribute value.
    pub fn gatt_read(&mut self, handle: u16) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        let mut req = [0u8; 3];
        req[0] = 0x0A;
        req[1..3].copy_from_slice(&handle.to_le_bytes());
        conn.gatt_send_request(&req);
        Ok(())
    }

    /// GATT client: write an attribute value (with response).
    pub fn gatt_write(&mut self, handle: u16, data: &[u8]) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        let mut req = [0u8; 23];
        req[0] = 0x12;
        req[1..3].copy_from_slice(&handle.to_le_bytes());
        let n = data.len().min(20);
        req[3..3 + n].copy_from_slice(&data[..n]);
        conn.gatt_send_request(&req[..3 + n]);
        Ok(())
    }

    /// GATT client: enable notifications on a CCCD (value 0x0001).
    pub fn gatt_subscribe(&mut self, cccd_handle: u16) -> Result<(), Error> {
        let conn = self.conn.as_mut().ok_or(Error::NotRunning)?;
        let mut req = [0u8; 5];
        req[0] = 0x12;
        req[1..3].copy_from_slice(&cccd_handle.to_le_bytes());
        req[3..5].copy_from_slice(&0x0001u16.to_le_bytes());
        conn.gatt_send_request(&req);
        Ok(())
    }

    /// GATT client: poll for a completed request response.
    ///
    /// Returns `(opcode, payload, length)` when a response arrived.
    pub fn gatt_poll(&mut self) -> Option<(u8, [u8; 64], usize)> {
        let conn = self.conn.as_mut()?;
        let (op, buf, len) = conn.gatt_take_result();
        if len > 0 {
            Some((op, buf, len))
        } else {
            None
        }
    }

    /// Request termination (LL_TERMINATE_IND) at the next connection event.
    pub fn gap_terminate(&mut self) -> Result<(), Error> {
        match &mut self.conn {
            Some(conn) => {
                conn.terminate();
                Ok(())
            }
            None => Err(Error::NotRunning),
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
