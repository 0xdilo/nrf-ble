//! LE connection (peripheral role), L2CAP-lite and a minimal GATT/ATT
//! server (Nordic UART Service).

use crate::ll::pdu::{
    ConnectReqData, DataPdu, LLID_CONTROL, LLID_DATA_COMPLETE, LLID_DATA_CONTINUATION_OR_COMPLETE,
    LLID_DATA_START,
};
use crate::Error;

use super::ccm::{Ccm, LL_MIC_LEN, LL_NONCE_MASTER_TO_SLAVE, LL_NONCE_SLAVE_TO_MASTER};
use super::pac;
use super::radio::Radio;
use super::smp::{
    Smp, L2CAP_SMP_CID, SMP_ENCRYPTION_INFORMATION, SMP_MASTER_IDENTIFICATION, SMP_PAIRING_CONFIRM,
    SMP_PAIRING_FAILED, SMP_PAIRING_RANDOM, SMP_PAIRING_REQUEST, SMP_PAIRING_RESPONSE,
    SMP_SECURITY_REQUEST,
};
use super::timers::{timeout_ticks, BtTimer, IntervalAccum};

/// LL Control PDU opcode: LL_CONNECTION_UPDATE_REQ.
pub const LL_CONTROL_CONNECTION_UPDATE_REQ: u8 = 0x00;
/// LL Control PDU opcode: LL_TERMINATE_IND.
pub const LL_CONTROL_TERMINATE_IND: u8 = 0x02;
/// LL Control PDU opcode: LL_ENC_REQ.
pub const LL_CONTROL_ENCRYPT_REQ: u8 = 0x03;
/// LL Control PDU opcode: LL_ENC_RSP.
pub const LL_CONTROL_ENCRYPT_RSP: u8 = 0x04;
/// LL Control PDU opcode: LL_START_ENC_REQ.
pub const LL_CONTROL_START_ENC_REQ: u8 = 0x05;
/// LL Control PDU opcode: LL_START_ENC_RSP.
pub const LL_CONTROL_START_ENC_RSP: u8 = 0x06;
/// LL Control PDU opcode: LL_UNKNOWN_RSP.
pub const LL_CONTROL_UNKNOWN_RSP: u8 = 0x07;
/// LL Control PDU opcode: LL_FEATURE_REQ.
pub const LL_CONTROL_FEATURE_REQ: u8 = 0x08;
/// LL Control PDU opcode: LL_FEATURE_RSP.
pub const LL_CONTROL_FEATURE_RSP: u8 = 0x09;
/// LL Control PDU opcode: LL_VERSION_IND.
pub const LL_CONTROL_VERSION_IND: u8 = 0x0C;
/// LL Control PDU opcode: LL_PING_REQ.
pub const LL_CONTROL_PING_REQ: u8 = 0x12;
/// LL Control PDU opcode: LL_PING_RSP.
pub const LL_CONTROL_PING_RSP: u8 = 0x13;
/// LL Control PDU opcode: LL_LENGTH_REQ.
pub const LL_CONTROL_LENGTH_REQ: u8 = 0x14;
/// LL Control PDU opcode: LL_LENGTH_RSP.
pub const LL_CONTROL_LENGTH_RSP: u8 = 0x15;
/// LL Control PDU opcode: LL_PHY_REQ.
pub const LL_CONTROL_PHY_REQ: u8 = 0x16;
/// LL Control PDU opcode: LL_PHY_RSP.
pub const LL_CONTROL_PHY_RSP: u8 = 0x17;

/// L2CAP channel ID for the ATT protocol.
pub const L2CAP_ATT_CID: u16 = 0x0004;
pub const L2CAP_SIGNALING_CID: u16 = 0x0005;
pub const L2CAP_SMP_CID_LOCAL: u16 = 0x0006;

pub const LE_CREDIT_BASED_CONNECTION_REQ: u8 = 0x14;
pub const LE_CREDIT_BASED_CONNECTION_RSP: u8 = 0x15;
pub const LE_FLOW_CONTROL_CREDIT: u8 = 0x16;
pub const LE_CREDIT_BASED_CONNECTION_END: u8 = 0x17;

/// Local source CID used for our connection-oriented channels.
pub const COC_LOCAL_CID: u16 = 0x0040;
pub const COC_CREDITS: u16 = 8;
pub const COC_MTU: u16 = 247;
pub const COC_MPS: u16 = 251;

/// ATT MTU of the server.
pub const ATT_MTU_DEFAULT: usize = 23;
/// Maximum supported ATT MTU.
pub const ATT_MTU_MAX: usize = 247;
/// Maximum ATT payload (MTU minus ATT header).
pub const ATT_PAYLOAD_MAX: usize = ATT_MTU_MAX - 3;
/// Maximum L2CAP payload (ATT + 4-byte L2CAP header).
pub const L2CAP_PAYLOAD_MAX: usize = ATT_MTU_MAX + 4;
/// Default maximum LL data PDU payload before DLE.
pub const LL_PDU_PAYLOAD_DEFAULT: usize = 27;
/// Maximum LL data PDU payload after DLE.
pub const LL_PDU_PAYLOAD_MAX: usize = 251;

pub const GATT_UUID_PRIMARY_SERVICE: u16 = 0x2800;
pub const GATT_UUID_CHARACTERISTIC: u16 = 0x2803;
pub const GATT_UUID_CLIENT_CHAR_CFG: u16 = 0x2902;

const NUS_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];
const NUS_TX_CHAR_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
];
const NUS_RX_CHAR_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
];

pub const HANDLE_PRIMARY_SERVICE: u16 = 0x0001;
pub const HANDLE_RX_CHAR_DECL: u16 = 0x0002;
pub const HANDLE_RX_VALUE: u16 = 0x0003;
pub const HANDLE_TX_CHAR_DECL: u16 = 0x0004;
pub const HANDLE_TX_VALUE: u16 = 0x0005;
pub const HANDLE_TX_CCCD: u16 = 0x0006;

/// A stored bond: the LTK (and optional IRK) for a peer address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BondInfo {
    /// Peer address.
    pub addr: [u8; 6],
    /// Long term key.
    pub ltk: [u8; 16],
    /// Optional identity resolving key.
    pub irk: Option<[u8; 16]>,
}

/// Persistent storage for bonds (LTK/IRK per peer address).
pub trait BondStore {
    /// Store or replace the bond for a peer address.
    fn save(&mut self, peer: [u8; 6], ltk: [u8; 16], irk: Option<[u8; 16]>);
    /// Look up a bond by peer address.
    fn find(&self, peer: &[u8; 6]) -> Option<BondInfo>;
}

/// Default in-RAM bond store (max 8 bonds).
#[derive(Debug, Clone, Copy)]
pub struct RamBondStore {
    bonds: [Option<BondInfo>; 8],
}

impl Default for RamBondStore {
    fn default() -> Self {
        Self::new()
    }
}

impl RamBondStore {
    /// Create an empty bond store.
    pub const fn new() -> Self {
        RamBondStore { bonds: [None; 8] }
    }
}

impl BondStore for RamBondStore {
    fn save(&mut self, peer: [u8; 6], ltk: [u8; 16], irk: Option<[u8; 16]>) {
        let info = BondInfo {
            addr: peer,
            ltk,
            irk,
        };
        for slot in self.bonds.iter_mut() {
            if let Some(b) = slot {
                if b.addr == peer {
                    *slot = Some(info);
                    return;
                }
            }
        }
        for slot in self.bonds.iter_mut() {
            if slot.is_none() {
                *slot = Some(info);
                return;
            }
        }
    }

    fn find(&self, peer: &[u8; 6]) -> Option<BondInfo> {
        self.bonds
            .iter()
            .flatten()
            .copied()
            .find(|b| &b.addr == peer)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A credit-based connection-oriented L2CAP channel.
pub struct Coc {
    /// Peer PSM.
    pub psm: u16,
    /// Local channel ID.
    pub local_cid: u16,
    /// Peer channel ID.
    pub peer_cid: u16,
    /// Credits we can use to send.
    pub credits: u16,
    /// Credits the peer has granted.
    pub peer_credits: u16,
    /// Channel MTU (SDU size).
    pub mtu: u16,
    /// Maximum PDU payload size.
    pub mps: u16,
    /// RX SDU buffer.
    pub rx_sdu: [u8; 256],
    /// RX SDU length so far.
    pub rx_sdu_len: usize,
    /// Expected SDU length.
    pub rx_sdu_total: usize,
    /// Peer SDU (the whole message) being sent.
    pub tx_sdu: [u8; 256],
    pub tx_sdu_len: usize,
    pub tx_sdu_offset: usize,
}

impl Coc {
    pub fn new(psm: u16) -> Coc {
        Coc {
            psm,
            local_cid: COC_LOCAL_CID,
            peer_cid: 0,
            credits: 0,
            peer_credits: 0,
            mtu: COC_MTU,
            mps: COC_MPS,
            rx_sdu: [0; 256],
            rx_sdu_len: 0,
            rx_sdu_total: 0,
            tx_sdu: [0; 256],
            tx_sdu_len: 0,
            tx_sdu_offset: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Role in the connection.
pub enum ConnRole {
    /// We initiated the connection.
    Master,
    /// We are the advertiser that accepted the connection.
    Slave,
}

pub struct Conn {
    pub role: ConnRole,
    pub access_addr: u32,
    pub crc_init: u32,
    pub interval: u16,
    pub timeout: u16,
    pub channel_map: [u8; 5],
    pub hop: u8,
    channel: u8,
    sn: bool,
    nesn: bool,
    anchor: u32,
    accum: IntervalAccum,
    rx_l2cap: [u8; 256],
    rx_l2cap_len: usize,
    rx_l2cap_msg_len: usize,
    rx_l2cap_cid: u16,
    tx_att: [u8; L2CAP_PAYLOAD_MAX],
    tx_att_len: usize,
    tx_frag_offset: usize,
    tx_pending: bool,
    tx_cid: u16,
    att_mtu: u16,
    tx_pdu_max: usize,
    rx_pdu_max: usize,
    pub coc: Option<Coc>,
    pub bond_store: RamBondStore,
    peer_version_ok: bool,
    pub last_rx: u32,
    terminate_pending: bool,
    pub nus_cccd: u8,
    pub rx_data: [u8; ATT_PAYLOAD_MAX],
    pub rx_data_len: usize,
    tx_buf: [u8; 64],
    rx_buf: [u8; 64],
    event_counter: u16,
    pending_update: Option<(u16, u16)>,
    pending_update_instant: u16,
    pending_request: bool,
    pub smp: Smp,
    pub encrypted: bool,
    packet_counter: u64,
    ccm: Ccm,
    want_encrypt: bool,
    pairing_in_progress: bool,
    pending_control: Option<[u8; 19]>,
    gatt_result: [u8; 64],
    gatt_result_len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Why a connection ended.
pub enum DisconnectReason {
    /// The peer sent LL_TERMINATE_IND.
    RemoteTerminate,
    /// We sent LL_TERMINATE_IND.
    HostTerminate,
    /// No packets within the supervision timeout.
    SupervisionTimeout,
    /// Internal error.
    LocalError,
}

impl Conn {
    pub fn new(
        params: &ConnectReqData,
        first_channel: u8,
        now: u32,
        role: ConnRole,
        ccm: &'static pac::ccm::RegisterBlock,
    ) -> Conn {
        let anchor = match role {
            ConnRole::Master => now.wrapping_add(timeout_ticks(200)),
            ConnRole::Slave => now,
        };
        Conn {
            role,
            access_addr: params.access_addr,
            crc_init: params.crc_init,
            interval: params.interval,
            timeout: params.timeout,
            channel_map: params.channel_map,
            hop: params.hop,
            channel: first_channel,
            sn: false,
            nesn: false,
            anchor,
            accum: IntervalAccum::new_125ms(),
            rx_l2cap: [0; 256],
            rx_l2cap_len: 0,
            rx_l2cap_msg_len: 0,
            rx_l2cap_cid: L2CAP_ATT_CID,
            tx_att: [0; L2CAP_PAYLOAD_MAX],
            tx_att_len: 0,
            tx_frag_offset: 0,
            tx_pending: false,
            tx_cid: L2CAP_ATT_CID,
            att_mtu: ATT_MTU_DEFAULT as u16,
            tx_pdu_max: LL_PDU_PAYLOAD_DEFAULT,
            rx_pdu_max: LL_PDU_PAYLOAD_DEFAULT,
            coc: None,
            bond_store: RamBondStore::new(),
            peer_version_ok: false,
            last_rx: now,
            terminate_pending: false,
            nus_cccd: 0,
            rx_data: [0; ATT_PAYLOAD_MAX],
            rx_data_len: 0,
            tx_buf: [0; 64],
            rx_buf: [0; 64],
            event_counter: 0,
            pending_update: None,
            pending_update_instant: 0,
            pending_request: false,
            smp: Smp::new(),
            encrypted: false,
            packet_counter: 0,
            ccm: Ccm::new(ccm),
            want_encrypt: false,
            pairing_in_progress: false,
            pending_control: None,
            gatt_result: [0; 64],
            gatt_result_len: 0,
        }
    }

    pub fn queue_notify(&mut self, data: &[u8]) -> Result<(), Error> {
        let len = data.len().min(ATT_PAYLOAD_MAX);
        self.tx_att[0] = 0x1B;
        self.tx_att[1..3].copy_from_slice(&HANDLE_TX_VALUE.to_le_bytes());
        self.tx_att[3..3 + len].copy_from_slice(&data[..len]);
        self.tx_att_len = 3 + len;
        self.tx_pending = true;
        Ok(())
    }

    /// Request termination at the next connection event.
    pub fn terminate(&mut self) {
        self.terminate_pending = true;
    }

    pub fn event(&mut self, radio: &Radio, timer: &BtTimer) -> Result<ConnEvent, Error> {
        self.event_counter = self.event_counter.wrapping_add(1);
        if self.encrypted {
            self.packet_counter = self.packet_counter.wrapping_add(1);
        }
        if let Some((interval, timeout)) = self.pending_update {
            if self.event_counter == self.pending_update_instant {
                self.interval = interval;
                self.timeout = timeout;
                self.accum = IntervalAccum::new_125ms();
            }
        }
        let ticks = self.accum.next(self.interval);
        self.anchor = self.anchor.wrapping_add(ticks);
        self.channel = next_channel(self.channel, self.hop, &self.channel_map);
        radio.set_access_address(self.access_addr);
        radio.set_crc_init(self.crc_init);
        radio.set_channel(self.channel)?;

        if self.role == ConnRole::Master {
            timer.set_compare(self.anchor);
            while !timer.compare_pending() {}
            timer.clear_compare();
            return self.event_master(radio, timer);
        }

        let listen_until = self.anchor.wrapping_add(timeout_ticks(2_000));
        let mut received = false;
        let mut ll = None;
        timer.set_compare(listen_until);
        radio.receive_start(&mut self.rx_buf);
        loop {
            match radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    if !self.rx(len) {
                        radio.receive_cancel();
                        break;
                    }
                    received = true;
                    match DataPdu::decode(&self.rx_buf[..len]) {
                        Ok(pdu) => {
                            ll = Some(pdu);
                        }
                        Err(_) => return Err(Error::InvalidPdu),
                    }
                    break;
                }
                Ok(None) => {
                    if timer.now() >= listen_until {
                        radio.receive_cancel();
                        break;
                    }
                }
                Err(_) => {
                    radio.receive_cancel();
                    break;
                }
            }
        }

        if !received {
            return Ok(ConnEvent::Idle);
        }
        self.last_rx = timer.now();

        let pdu = ll.ok_or(Error::InvalidPdu)?;
        let pdu_sn = pdu.sn;
        let pdu_llid = pdu.llid;
        let plen = pdu.payload.len().min(64);
        let mut payload = [0u8; 64];
        payload[..plen].copy_from_slice(&pdu.payload[..plen]);
        let mut result = ConnEvent::Idle;

        if plen == 0 {
            if pdu_sn != self.nesn {
                self.nesn = pdu_sn;
            }
        } else {
            match pdu_llid {
                LLID_CONTROL => {
                    if pdu_sn != self.nesn {
                        self.nesn = pdu_sn;
                    }
                    match self.handle_ll_control(&payload[..plen], radio, timer) {
                        Ok(()) => result = ConnEvent::Control(payload[0]),
                        Err(reason) => {
                            return Ok(ConnEvent::Disconnected(reason));
                        }
                    }
                }
                _ => {
                    let new_data = pdu_sn != self.nesn;
                    if new_data {
                        self.nesn = pdu_sn;
                    }
                    self.handle_l2cap(&payload[..plen], pdu_llid, &mut result);
                }
            }
        }

        if self.terminate_pending {
            self.tx_buf[0] =
                (LLID_CONTROL & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
            self.tx_buf[1] = 2;
            self.tx_buf[2] = LL_CONTROL_TERMINATE_IND;
            self.tx_buf[3] = 0x13;
            radio.transmit(&self.tx_buf[..4]);
            return Ok(ConnEvent::Disconnected(DisconnectReason::HostTerminate));
        }
        self.respond(radio, timer);
        Ok(result)
    }

    fn event_master(&mut self, radio: &Radio, timer: &BtTimer) -> Result<ConnEvent, Error> {
        if self.terminate_pending {
            self.tx_buf[0] =
                (LLID_CONTROL & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
            self.tx_buf[1] = 2;
            self.tx_buf[2] = LL_CONTROL_TERMINATE_IND;
            self.tx_buf[3] = 0x13;
            radio.transmit(&self.tx_buf[..4]);
            return Ok(ConnEvent::Disconnected(DisconnectReason::HostTerminate));
        }
        self.respond(radio, timer);

        let listen_until = timer.now().wrapping_add(timeout_ticks(2_000));
        timer.set_compare(listen_until);
        let mut result = ConnEvent::Idle;
        let mut payload = [0u8; 64];
        radio.receive_start(&mut self.rx_buf);
        loop {
            match radio.receive_poll(&self.rx_buf) {
                Ok(Some(len)) => {
                    if !self.rx(len) {
                        radio.receive_cancel();
                        break;
                    }
                    self.last_rx = timer.now();
                    if let Ok(pdu) = DataPdu::decode(&self.rx_buf[..len]) {
                        let plen = pdu.payload.len().min(64);
                        payload[..plen].copy_from_slice(&pdu.payload[..plen]);
                        if pdu.sn != self.nesn {
                            self.nesn = pdu.sn;
                        }
                        if plen > 0 {
                            if pdu.llid == LLID_CONTROL {
                                if pdu.payload.first() == Some(&LL_CONTROL_TERMINATE_IND) {
                                    return Ok(ConnEvent::Disconnected(
                                        DisconnectReason::RemoteTerminate,
                                    ));
                                }
                                result = ConnEvent::Control(payload[0]);
                            } else {
                                self.handle_l2cap(&payload[..plen], pdu.llid, &mut result);
                            }
                        }
                    }
                    break;
                }
                Ok(None) => {
                    if timer.now() >= listen_until {
                        radio.receive_cancel();
                        break;
                    }
                }
                Err(_) => {
                    radio.receive_cancel();
                    break;
                }
            }
        }
        Ok(result)
    }

    /// Build the next data channel fragment of the pending L2CAP PDU into
    /// `tx_buf`, returning the total packet length (2 + payload). Advances
    /// the fragment offset and clears the pending flag on the final
    /// fragment.
    pub fn tx_fragment(&mut self) -> usize {
        let att_len = self.tx_att_len;
        let remaining = att_len - self.tx_frag_offset;
        let first = self.tx_frag_offset == 0;
        let avail = if first {
            self.tx_pdu_max.saturating_sub(4)
        } else {
            self.tx_pdu_max
        };
        let chunk = remaining.min(avail);
        let complete = first && chunk == att_len;
        let llid = if complete {
            LLID_DATA_COMPLETE
        } else if first {
            LLID_DATA_START
        } else {
            LLID_DATA_CONTINUATION_OR_COMPLETE
        };
        let payload_len = if first { 4 + chunk } else { chunk };
        self.tx_buf[0] = (llid & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
        self.tx_buf[1] = payload_len as u8;
        if first {
            self.tx_buf[2..4].copy_from_slice(&(att_len as u16).to_le_bytes());
            self.tx_buf[4..6].copy_from_slice(&self.tx_cid.to_le_bytes());
            self.tx_buf[6..6 + chunk]
                .copy_from_slice(&self.tx_att[self.tx_frag_offset..self.tx_frag_offset + chunk]);
        } else {
            self.tx_buf[2..2 + chunk]
                .copy_from_slice(&self.tx_att[self.tx_frag_offset..self.tx_frag_offset + chunk]);
        }
        self.tx_frag_offset += chunk;
        if self.tx_frag_offset >= att_len {
            self.tx_pending = false;
            self.tx_frag_offset = 0;
        }
        2 + payload_len
    }

    fn tx(&mut self, radio: &Radio, len: usize) {
        if self.encrypted {
            let direction = match self.role {
                ConnRole::Master => LL_NONCE_MASTER_TO_SLAVE,
                ConnRole::Slave => LL_NONCE_SLAVE_TO_MASTER,
            };
            self.ccm
                .setup(&self.smp.stk, self.packet_counter, direction);
            if self.ccm.process(false, &mut self.tx_buf, len).is_err() {
                return;
            }
            radio.transmit(&self.tx_buf[..len + LL_MIC_LEN]);
        } else {
            radio.transmit(&self.tx_buf[..len]);
        }
    }

    fn rx(&mut self, len: usize) -> bool {
        if !self.encrypted {
            return true;
        }
        let direction = match self.role {
            ConnRole::Master => LL_NONCE_SLAVE_TO_MASTER,
            ConnRole::Slave => LL_NONCE_MASTER_TO_SLAVE,
        };
        self.ccm
            .setup(&self.smp.stk, self.packet_counter, direction);
        self.ccm.process(true, &mut self.rx_buf, len).is_ok()
    }

    fn respond(&mut self, radio: &Radio, timer: &BtTimer) {
        let header = |buf: &mut [u8; 64], llid: u8, nesn: bool, sn: bool, len: u8| {
            buf[0] = (llid & 0b11) | ((nesn as u8) << 2) | ((sn as u8) << 3);
            buf[1] = len;
        };
        if let Some(ctrl) = self.pending_control.take() {
            let plen: usize = 18;
            header(
                &mut self.tx_buf,
                LLID_CONTROL,
                !self.nesn,
                self.sn,
                plen as u8,
            );
            self.tx_buf[2..2 + plen].copy_from_slice(&ctrl[..plen]);
            let _ = timer;
            self.tx(radio, 2 + plen);
            return;
        }
        if self.tx_pending {
            let total = self.tx_fragment();
            self.sn = !self.sn;
            let _ = timer;
            self.tx(radio, total);
        } else {
            header(&mut self.tx_buf, LLID_DATA_COMPLETE, !self.nesn, self.sn, 0);
            self.tx(radio, 2);
        }
    }

    fn handle_le_signaling(&mut self, body: &[u8]) {
        if body.len() < 4 {
            return;
        }
        let code = body[0];
        match code {
            LE_CREDIT_BASED_CONNECTION_REQ => {
                if body.len() < 12 {
                    return;
                }
                let psm = u16::from_le_bytes([body[2], body[3]]);
                let peer_cid = u16::from_le_bytes([body[4], body[5]]);
                let mtu = u16::from_le_bytes([body[6], body[7]]);
                let mps = u16::from_le_bytes([body[8], body[9]]);
                let credits = u16::from_le_bytes([body[10], body[11]]);
                let mut coc = Coc::new(psm);
                coc.peer_cid = peer_cid;
                coc.mtu = mtu.min(COC_MTU);
                coc.mps = mps.min(COC_MPS);
                coc.credits = credits;
                self.coc = Some(coc);
                let mut rsp = [0u8; 16];
                rsp[0] = LE_CREDIT_BASED_CONNECTION_RSP;
                rsp[1] = 0;
                rsp[2..4].copy_from_slice(&peer_cid.to_le_bytes());
                rsp[4..6].copy_from_slice(&COC_MTU.to_le_bytes());
                rsp[6..8].copy_from_slice(&COC_MPS.to_le_bytes());
                rsp[8..10].copy_from_slice(&COC_CREDITS.to_le_bytes());
                rsp[10..12].copy_from_slice(&0u16.to_le_bytes());
                self.queue_signaling(&rsp[..12]);
            }
            LE_CREDIT_BASED_CONNECTION_RSP => {
                if let Some(coc) = &mut self.coc {
                    let peer_cid = u16::from_le_bytes([body[2], body[3]]);
                    let mtu = u16::from_le_bytes([body[4], body[5]]);
                    let mps = u16::from_le_bytes([body[6], body[7]]);
                    let credits = u16::from_le_bytes([body[8], body[9]]);
                    let result = u16::from_le_bytes([body[10], body[11]]);
                    if result == 0 {
                        coc.peer_cid = peer_cid;
                        coc.mtu = coc.mtu.min(mtu);
                        coc.mps = coc.mps.min(mps);
                        coc.peer_credits = credits;
                    } else {
                        self.coc = None;
                    }
                }
            }
            LE_FLOW_CONTROL_CREDIT => {
                if let Some(coc) = &mut self.coc {
                    if body.len() >= 6 {
                        let cid = u16::from_le_bytes([body[2], body[3]]);
                        let credits = u16::from_le_bytes([body[4], body[5]]);
                        if cid == coc.local_cid || cid == coc.peer_cid {
                            coc.credits = coc.credits.saturating_add(credits);
                        }
                    }
                }
            }
            LE_CREDIT_BASED_CONNECTION_END => {
                if let Some(coc) = &mut self.coc {
                    let cid = u16::from_le_bytes([body[2], body[3]]);
                    if cid == coc.local_cid || cid == coc.peer_cid {
                        self.coc = None;
                    }
                }
            }
            _ => {}
        }
    }

    fn queue_signaling(&mut self, msg: &[u8]) {
        let n = msg.len().min(self.tx_att.len() - 4);
        self.tx_att[..n].copy_from_slice(&msg[..n]);
        self.tx_att_len = n;
        self.tx_cid = L2CAP_SIGNALING_CID;
        self.tx_frag_offset = 0;
        self.tx_pending = true;
    }

    /// Queue a CoC open request for a PSM (initiator side).
    pub fn l2cap_connect(&mut self, psm: u16) {
        if self.coc.is_some() {
            return;
        }
        self.coc = Some(Coc::new(psm));
        let mut req = [0u8; 12];
        req[0] = LE_CREDIT_BASED_CONNECTION_REQ;
        req[1] = 0;
        req[2..4].copy_from_slice(&psm.to_le_bytes());
        req[4..6].copy_from_slice(&COC_LOCAL_CID.to_le_bytes());
        req[6..8].copy_from_slice(&COC_MTU.to_le_bytes());
        req[8..10].copy_from_slice(&COC_MPS.to_le_bytes());
        req[10..12].copy_from_slice(&COC_CREDITS.to_le_bytes());
        self.queue_signaling(&req);
    }

    /// Queue application data for the CoC channel. The SDU is framed with
    /// its length and sent (consuming one credit) via the normal
    /// fragmenter.
    pub fn l2cap_send(&mut self, data: &[u8]) -> Result<(), Error> {
        let coc = self.coc.as_mut().ok_or(Error::NotRunning)?;
        if data.len() > coc.mtu as usize || data.len() + 2 > self.tx_att.len() {
            return Err(Error::InvalidLength);
        }
        if coc.credits == 0 {
            return Err(Error::NotRunning);
        }
        coc.credits -= 1;
        let total = data.len() + 2;
        self.tx_att[..2].copy_from_slice(&(data.len() as u16).to_le_bytes());
        self.tx_att[2..total].copy_from_slice(data);
        self.tx_att_len = total;
        self.tx_cid = coc.local_cid;
        self.tx_frag_offset = 0;
        self.tx_pending = true;
        Ok(())
    }

    /// Assemble an incoming CoC SDU from a data frame; reports
    /// [`ConnEvent::Data`] when a complete SDU arrived (and returns the
    /// credit for it).
    fn rx_coc_sdu(&mut self, body: &[u8], result: &mut ConnEvent) {
        let mut complete = false;
        let mut total = 0usize;
        let mut sdu = [0u8; 256];
        let peer_cid;
        {
            let Some(coc) = &mut self.coc else {
                return;
            };
            if coc.rx_sdu_len == 0 {
                if body.len() < 2 {
                    return;
                }
                coc.rx_sdu_total = u16::from_le_bytes([body[0], body[1]]) as usize;
                coc.rx_sdu_len = 0;
                let chunk = &body[2..];
                let n = chunk.len().min(coc.rx_sdu.len() - coc.rx_sdu_len);
                coc.rx_sdu[coc.rx_sdu_len..coc.rx_sdu_len + n].copy_from_slice(&chunk[..n]);
                coc.rx_sdu_len += n;
            } else {
                let n = body.len().min(coc.rx_sdu.len() - coc.rx_sdu_len);
                coc.rx_sdu[coc.rx_sdu_len..coc.rx_sdu_len + n].copy_from_slice(&body[..n]);
                coc.rx_sdu_len += n;
            }
            peer_cid = coc.peer_cid;
            if coc.rx_sdu_len >= coc.rx_sdu_total {
                total = coc.rx_sdu_total.min(sdu.len());
                sdu[..total].copy_from_slice(&coc.rx_sdu[..total]);
                coc.rx_sdu_len = 0;
                coc.rx_sdu_total = 0;
                complete = true;
            }
        }
        if complete {
            let mut credit = [0u8; 6];
            credit[0] = LE_FLOW_CONTROL_CREDIT;
            credit[1] = 0;
            credit[2..4].copy_from_slice(&peer_cid.to_le_bytes());
            credit[4..6].copy_from_slice(&1u16.to_le_bytes());
            self.queue_signaling(&credit);
            let n = total.min(self.rx_data.len());
            self.rx_data[..n].copy_from_slice(&sdu[..n]);
            self.rx_data_len = n;
            *result = ConnEvent::Data;
        }
    }

    fn handle_smp(&mut self, body: &[u8], _result: &mut ConnEvent) {
        if body.is_empty() {
            return;
        }
        let op = body[0];
        let mut out = [0u8; 17];
        let mut out_len = 0;
        let mut complete = false;
        match op {
            SMP_PAIRING_FAILED => {
                self.pairing_in_progress = false;
                return;
            }
            SMP_ENCRYPTION_INFORMATION | SMP_MASTER_IDENTIFICATION => {
                if self.role == ConnRole::Slave {
                    let _ = op;
                }
            }
            SMP_SECURITY_REQUEST => {
                let mut req = [0u8; 7];
                req.copy_from_slice(&self.smp.build_pairing_request());
                let mut l2 = [0u8; 11];
                l2[0..2].copy_from_slice(&7u16.to_le_bytes());
                l2[2..4].copy_from_slice(&L2CAP_SMP_CID.to_le_bytes());
                l2[4..11].copy_from_slice(&req);
                self.queue_l2cap(&l2, L2CAP_SMP_CID);
                return;
            }
            SMP_PAIRING_REQUEST => {
                if let Ok(rsp) = self.smp.handle_pairing_request(&body[..body.len().min(7)]) {
                    out[..7].copy_from_slice(&rsp);
                    out_len = 7;
                    self.pairing_in_progress = true;
                }
            }
            SMP_PAIRING_RESPONSE => {
                if self
                    .smp
                    .handle_pairing_response(&body[..body.len().min(7)])
                    .is_ok()
                {
                    self.pairing_in_progress = true;
                }
            }
            SMP_PAIRING_CONFIRM => {
                if self.smp.handle_confirm(&body[..body.len().min(17)]).is_ok() {
                    let iat = self.smp.iat;
                    let ia = self.smp.ia;
                    let ra = self.smp.ra;
                    out = self.smp.build_confirm(iat, &ia, &ra);
                    out_len = 17;
                }
            }
            SMP_PAIRING_RANDOM => match self.smp.handle_random(&body[..body.len().min(17)]) {
                Ok(rnd) => {
                    out[..17].copy_from_slice(&rnd);
                    out_len = 17;
                    complete = true;
                }
                Err(_) => {
                    let reason = self.smp.pairing_failed;
                    let failed = self.smp.build_failed(reason);
                    let mut l2 = [0u8; 6];
                    l2[0..2].copy_from_slice(&2u16.to_le_bytes());
                    l2[2..4].copy_from_slice(&L2CAP_SMP_CID.to_le_bytes());
                    l2[4..6].copy_from_slice(&failed);
                    self.queue_l2cap(&l2, L2CAP_SMP_CID);
                    self.pairing_in_progress = false;
                    return;
                }
            },
            _ => {}
        }
        if out_len > 0 {
            let mut l2 = [0u8; 21];
            l2[0..2].copy_from_slice(&(out_len as u16).to_le_bytes());
            l2[2..4].copy_from_slice(&L2CAP_SMP_CID.to_le_bytes());
            l2[4..4 + out_len].copy_from_slice(&out[..out_len]);
            self.queue_l2cap(&l2[..4 + out_len], L2CAP_SMP_CID);
        }
        if complete {
            self.pairing_in_progress = false;
            if self.role == ConnRole::Master {
                self.queue_encrypt_req();
            }
        }
    }

    pub fn queue_l2cap(&mut self, l2: &[u8], cid: u16) {
        let n = l2.len().min(self.tx_att.len());
        self.tx_att[..n].copy_from_slice(&l2[..n]);
        self.tx_att_len = n;
        self.tx_cid = cid;
        self.tx_frag_offset = 0;
        self.tx_pending = true;
    }

    fn queue_encrypt_req(&mut self) {
        let mut ctrl = [0u8; 19];
        ctrl[0] = LL_CONTROL_ENCRYPT_REQ;
        ctrl[1..9].fill(0);
        ctrl[9..11].fill(0);
        ctrl[11..19].fill(0);
        self.pending_control = Some(ctrl);
        self.want_encrypt = true;
    }

    fn handle_l2cap(&mut self, payload: &[u8], llid: u8, result: &mut ConnEvent) {
        let is_first = llid != LLID_DATA_CONTINUATION_OR_COMPLETE || self.rx_l2cap_len == 0;
        if is_first {
            if payload.len() < 4 {
                return;
            }
            self.rx_l2cap_len = 0;
            self.rx_l2cap_msg_len = u16::from_le_bytes([payload[0], payload[1]]) as usize;
            self.rx_l2cap_cid = u16::from_le_bytes([payload[2], payload[3]]);
            let cid = self.rx_l2cap_cid;
            if cid == L2CAP_SMP_CID_LOCAL {
                let body = &payload[4..];
                self.handle_smp(body, result);
                self.rx_l2cap_len = 0;
                return;
            }
            if cid == L2CAP_SIGNALING_CID {
                let body = &payload[4..];
                self.handle_le_signaling(body);
                self.rx_l2cap_len = 0;
                return;
            }
            if cid != L2CAP_ATT_CID {
                let is_coc = self
                    .coc
                    .as_ref()
                    .is_some_and(|c| c.peer_cid == cid || c.local_cid == cid);
                if is_coc {
                    let body = &payload[4..];
                    self.rx_l2cap_len = 0;
                    self.rx_coc_sdu(body, result);
                    return;
                }
                *result = ConnEvent::L2cap(cid);
                self.rx_l2cap_len = 0;
                return;
            }
        }
        let body = if is_first { &payload[4..] } else { payload };
        let cap = self.rx_l2cap.len();
        let n = body.len().min(cap - self.rx_l2cap_len);
        self.rx_l2cap[self.rx_l2cap_len..self.rx_l2cap_len + n].copy_from_slice(&body[..n]);
        self.rx_l2cap_len += n;
        let msg_len = self.rx_l2cap_msg_len.min(cap);
        if self.rx_l2cap_len >= msg_len {
            let mut att = [0u8; 256];
            att[..msg_len].copy_from_slice(&self.rx_l2cap[..msg_len]);
            self.handle_att(&att[..msg_len], result);
            self.rx_l2cap_len = 0;
        }
    }

    fn handle_att(&mut self, att: &[u8], result: &mut ConnEvent) {
        if att.is_empty() {
            return;
        }
        let op = att[0];
        if self.pending_request
            && matches!(op, 0x01 | 0x03 | 0x09 | 0x0B | 0x11 | 0x13 | 0x15 | 0x17)
        {
            if op == 0x03 && att.len() >= 3 {
                let peer = u16::from_le_bytes([att[1], att[2]]);
                self.att_mtu = self.att_mtu.min(peer).max(ATT_MTU_DEFAULT as u16);
            }
            self.pending_request = false;
            let n = att.len().min(self.gatt_result.len());
            self.gatt_result[..n].copy_from_slice(&att[..n]);
            self.gatt_result_len = n;
            return;
        }
        if op == 0x1B {
            let mut buf = [0u8; 20];
            let handle = u16::from_le_bytes([att[1], att[2]]);
            let n = (att.len() - 3).min(20);
            buf[..n].copy_from_slice(&att[3..3 + n]);
            self.rx_data[..n].copy_from_slice(&buf[..n]);
            self.rx_data_len = n;
            let _ = handle;
            *result = ConnEvent::Data;
            return;
        }
        match op {
            0x02 => {
                let mut rsp = [0u8; 5];
                rsp[0] = 0x03;
                rsp[1..3].copy_from_slice(&self.att_mtu.to_le_bytes());
                self.queue_att(&rsp);
            }
            0x04 => {
                let mut rsp = [0u8; 64];
                rsp[0] = 0x05;
                let mut uuid16 = true;
                let start = u16::from_le_bytes([att[1], att[2]]);
                let end = u16::from_le_bytes([att[3], att[4]]);
                let (entries, count) = self.entries(start, end);
                for (_, uuid) in &entries[..count] {
                    if uuid.as_u16().is_none() {
                        uuid16 = false;
                    }
                }
                rsp[1] = if uuid16 { 0x01 } else { 0x02 };
                let mut pos = 2;
                for (handle, uuid) in entries[..count].iter().copied() {
                    let uuid16_val = uuid.as_u16();
                    let mut as128 = [0u8; 16];
                    if let Some(u) = uuid16_val {
                        if uuid16 {
                            if pos + 4 <= rsp.len() {
                                rsp[pos..pos + 2].copy_from_slice(&handle.to_le_bytes());
                                rsp[pos + 2..pos + 4].copy_from_slice(&u.to_le_bytes());
                                pos += 4;
                            }
                        } else {
                            as128[2..4].copy_from_slice(&u.to_le_bytes());
                            if pos + 18 <= rsp.len() {
                                rsp[pos..pos + 2].copy_from_slice(&handle.to_le_bytes());
                                rsp[pos + 2..pos + 18].copy_from_slice(&as128);
                                pos += 18;
                            }
                        }
                    } else if pos + 18 <= rsp.len() {
                        rsp[pos..pos + 2].copy_from_slice(&handle.to_le_bytes());
                        rsp[pos + 2..pos + 18].copy_from_slice(&uuid.to_128());
                        pos += 18;
                    }
                }
                self.queue_att(&rsp[..pos]);
            }
            0x08 => {
                let uuid = u16::from_le_bytes([att[1], att[2]]);
                let start = u16::from_le_bytes([att[3], att[4]]);
                let end = u16::from_le_bytes([att[5], att[6]]);
                let mut rsp = [0u8; 64];
                rsp[0] = 0x09;
                let mut pos = 2;
                let mut width = 0u8;
                let (entries, count) = self.by_type(start, end, uuid);
                for (handle, payload, plen) in entries[..count].iter().copied() {
                    if width == 0 {
                        width = 2 + plen as u8;
                        rsp[1] = width;
                    }
                    if pos + width as usize <= rsp.len() {
                        rsp[pos..pos + 2].copy_from_slice(&handle.to_le_bytes());
                        rsp[pos + 2..pos + 2 + plen].copy_from_slice(&payload[..plen]);
                        pos += width as usize;
                    }
                }
                if pos == 1 {
                    self.queue_att(&[0x01, 0x0A, 0x00, 0x00, 0x08]);
                } else {
                    self.queue_att(&rsp[..pos]);
                }
            }
            0x0A => {
                let handle = u16::from_le_bytes([att[1], att[2]]);
                let mut value = [0u8; 32];
                match self.read(handle, &mut value) {
                    Some(vlen) => {
                        let mut rsp = [0u8; 32];
                        rsp[0] = 0x0B;
                        rsp[1..1 + vlen].copy_from_slice(&value[..vlen]);
                        self.queue_att(&rsp[..1 + vlen]);
                    }
                    None => self.queue_att(&[0x01, 0x0A, 0x00, 0x00, 0x0A]),
                }
            }
            0x0C => {
                let handle = u16::from_le_bytes([att[1], att[2]]);
                let offset = u16::from_le_bytes([att[3], att[4]]) as usize;
                let mut value = [0u8; 32];
                match self.read(handle, &mut value) {
                    Some(vlen) if offset < vlen => {
                        let chunk = &value[offset..vlen];
                        let mut rsp = [0u8; 32];
                        rsp[0] = 0x0D;
                        rsp[1..1 + chunk.len()].copy_from_slice(chunk);
                        self.queue_att(&rsp[..1 + chunk.len()]);
                    }
                    _ => self.queue_att(&[0x01, 0x0C, 0x00, 0x00, 0x0A]),
                }
            }
            0x10 => {
                let uuid = u16::from_le_bytes([att[1], att[2]]);
                let start = u16::from_le_bytes([att[3], att[4]]);
                let end = u16::from_le_bytes([att[5], att[6]]);
                if uuid == GATT_UUID_PRIMARY_SERVICE {
                    let mut rsp = [0u8; 32];
                    rsp[0] = 0x11;
                    let mut pos = 2;
                    let mut width = 0u8;
                    let (services, count) = self.primary_services(start, end);
                    for (handle, svc) in services[..count].iter().copied() {
                        let payload: &[u8] = &svc;
                        if width == 0 {
                            width = 2 + payload.len() as u8;
                            rsp[1] = width;
                        }
                        if pos + width as usize <= rsp.len() {
                            rsp[pos..pos + 2].copy_from_slice(&handle.to_le_bytes());
                            rsp[pos + 2..pos + 2 + payload.len()].copy_from_slice(payload);
                            pos += width as usize;
                        }
                    }
                    if pos == 1 {
                        self.queue_att(&[0x01, 0x10, 0x00, 0x00, 0x10]);
                    } else {
                        self.queue_att(&rsp[..pos]);
                    }
                } else {
                    self.queue_att(&[0x01, 0x10, 0x00, 0x00, 0x10]);
                }
            }
            0x12 => {
                let handle = u16::from_le_bytes([att[1], att[2]]);
                let value = &att[3..];
                match self.write(handle, value, result) {
                    Ok(()) => {
                        let mut rsp = [0u8; 1];
                        rsp[0] = 0x13;
                        self.queue_att(&rsp);
                    }
                    Err(_) => self.queue_att(&[0x01, 0x12, 0x00, 0x00, 0x12]),
                }
            }
            _ => {
                self.queue_att(&[0x01, op, 0x00, 0x00, 0x06]);
            }
        }
    }

    pub fn gatt_send_request(&mut self, att: &[u8]) {
        self.queue_att(att);
        self.pending_request = true;
    }

    /// Parse the last (handle, ...) entry of a discovery response, for
    /// multi-range continuation. Returns `None` for malformed responses.
    #[cfg(test)]
    pub fn att_response_last_handle(buf: &[u8]) -> Option<u16> {
        if buf.len() < 2 {
            return None;
        }
        let width = buf[1] as usize;
        if width < 2 {
            return None;
        }
        let entries = (buf.len() - 2) / width;
        if entries == 0 {
            return None;
        }
        let pos = 2 + (entries - 1) * width;
        Some(u16::from_le_bytes([buf[pos], buf[pos + 1]]))
    }

    pub fn gatt_take_result(&mut self) -> (u8, [u8; 64], usize) {
        let op = self.gatt_result[0];
        let len = self.gatt_result_len;
        let mut buf = self.gatt_result;
        self.gatt_result_len = 0;
        self.gatt_result[0] = 0;
        buf[0] = op;
        (op, buf, len)
    }

    fn queue_att(&mut self, att: &[u8]) {
        let len = att.len().min(self.tx_att.len());
        self.tx_att[..len].copy_from_slice(&att[..len]);
        self.tx_att_len = len;
        self.tx_pending = true;
    }

    fn handle_ll_control(
        &mut self,
        payload: &[u8],
        radio: &Radio,
        timer: &BtTimer,
    ) -> Result<(), DisconnectReason> {
        if payload.is_empty() {
            return Ok(());
        }
        let op = payload[0];
        match op {
            LL_CONTROL_TERMINATE_IND => Err(DisconnectReason::RemoteTerminate),
            LL_CONTROL_VERSION_IND => {
                self.tx_buf[0] =
                    (LLID_CONTROL & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
                self.tx_buf[1] = 4;
                self.tx_buf[2] = LL_CONTROL_VERSION_IND;
                self.tx_buf[3] = 0x0C;
                self.tx_buf[4] = 0xFF;
                self.tx_buf[5] = 0xFF;
                self.tx_buf[6] = 0x00;
                self.tx_buf[7] = 0x00;
                radio.transmit(&self.tx_buf[..8]);
                let _ = timer;
                self.peer_version_ok = true;
                Ok(())
            }
            LL_CONTROL_FEATURE_REQ | LL_CONTROL_FEATURE_RSP => {
                self.tx_buf[0] =
                    (LLID_CONTROL & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
                self.tx_buf[1] = 9;
                self.tx_buf[2] = LL_CONTROL_FEATURE_RSP;
                self.tx_buf[3..11].fill(0);
                radio.transmit(&self.tx_buf[..11]);
                Ok(())
            }
            LL_CONTROL_ENCRYPT_REQ => {
                let mut rsp = [0u8; 19];
                rsp[0] = LL_CONTROL_ENCRYPT_RSP;
                rsp[1..9].fill(0);
                rsp[9..11].fill(0);
                rsp[11..19].fill(0);
                self.pending_control = Some(rsp);
                self.want_encrypt = true;
                Ok(())
            }
            LL_CONTROL_ENCRYPT_RSP => {
                let mut req = [0u8; 19];
                req[0] = LL_CONTROL_START_ENC_REQ;
                self.pending_control = Some(req);
                Ok(())
            }
            LL_CONTROL_START_ENC_REQ => {
                self.tx_buf[0] =
                    (LLID_CONTROL & 0b11) | ((!self.nesn as u8) << 2) | ((self.sn as u8) << 3);
                self.tx_buf[1] = 1;
                self.tx_buf[2] = LL_CONTROL_START_ENC_RSP;
                self.tx_buf[3] = 0;
                self.tx(radio, 3);
                self.encrypted = true;
                self.packet_counter = 0;
                Ok(())
            }
            LL_CONTROL_START_ENC_RSP => {
                self.encrypted = true;
                self.packet_counter = 0;
                Ok(())
            }
            LL_CONTROL_LENGTH_REQ => {
                let max_tx = u16::from_le_bytes([payload[1], payload[2]]) as usize;
                let max_rx = u16::from_le_bytes([payload[5], payload[6]]) as usize;
                let mut rsp = [0u8; 19];
                rsp[0] = LL_CONTROL_LENGTH_RSP;
                rsp[1..3].copy_from_slice(&(LL_PDU_PAYLOAD_MAX as u16).to_le_bytes());
                rsp[3..5].copy_from_slice(&2120u16.to_le_bytes());
                rsp[5..7].copy_from_slice(&(LL_PDU_PAYLOAD_MAX as u16).to_le_bytes());
                rsp[7..9].copy_from_slice(&2120u16.to_le_bytes());
                self.pending_control = Some(rsp);
                self.tx_pdu_max = max_tx.min(LL_PDU_PAYLOAD_MAX);
                self.rx_pdu_max = max_rx.min(LL_PDU_PAYLOAD_MAX);
                Ok(())
            }
            LL_CONTROL_LENGTH_RSP => {
                let max_tx = u16::from_le_bytes([payload[1], payload[2]]) as usize;
                let max_rx = u16::from_le_bytes([payload[5], payload[6]]) as usize;
                self.tx_pdu_max = max_tx.min(LL_PDU_PAYLOAD_MAX);
                self.rx_pdu_max = max_rx.min(LL_PDU_PAYLOAD_MAX);
                Ok(())
            }
            LL_CONTROL_PING_REQ => {
                let mut rsp = [0u8; 19];
                rsp[0] = LL_CONTROL_PING_RSP;
                self.pending_control = Some(rsp);
                Ok(())
            }
            LL_CONTROL_PHY_REQ => {
                let mut rsp = [0u8; 19];
                rsp[0] = LL_CONTROL_PHY_RSP;
                rsp[1] = 0;
                rsp[2] = 0x01;
                rsp[3] = 0x01;
                rsp[4] = 0x01;
                self.pending_control = Some(rsp);
                Ok(())
            }
            LL_CONTROL_CONNECTION_UPDATE_REQ => {
                if payload.len() >= 10 {
                    let interval = u16::from_le_bytes([payload[3], payload[4]]);
                    let timeout = u16::from_le_bytes([payload[7], payload[8]]);
                    let instant = u16::from_le_bytes([payload[9], payload[10]]);
                    self.pending_update = Some((interval, timeout));
                    self.pending_update_instant = instant;
                }
                Ok(())
            }
            _ => {
                let mut rsp = [0u8; 19];
                rsp[0] = LL_CONTROL_UNKNOWN_RSP;
                rsp[1] = payload[0];
                self.pending_control = Some(rsp);
                Ok(())
            }
        }
    }

    fn entries(&self, start: u16, end: u16) -> ([(u16, GattUuid); 6], usize) {
        let mut out = [(0u16, GattUuid::Uuid16(0)); 6];
        let all = [
            (HANDLE_PRIMARY_SERVICE, GattUuid::Uuid128(NUS_SERVICE_UUID)),
            (
                HANDLE_RX_CHAR_DECL,
                GattUuid::Uuid16(GATT_UUID_CHARACTERISTIC),
            ),
            (HANDLE_RX_VALUE, GattUuid::Uuid128(NUS_RX_CHAR_UUID)),
            (
                HANDLE_TX_CHAR_DECL,
                GattUuid::Uuid16(GATT_UUID_CHARACTERISTIC),
            ),
            (HANDLE_TX_VALUE, GattUuid::Uuid128(NUS_TX_CHAR_UUID)),
            (HANDLE_TX_CCCD, GattUuid::Uuid16(GATT_UUID_CLIENT_CHAR_CFG)),
        ];
        let mut n = 0;
        for (h, u) in all {
            if h >= start && h <= end {
                out[n] = (h, u);
                n += 1;
            }
        }
        let _ = n;
        (out, n)
    }

    fn by_type(&self, start: u16, end: u16, uuid: u16) -> ([(u16, [u8; 19], usize); 4], usize) {
        let mut out = [(0u16, [0u8; 19], 0usize); 4];
        let mut n = 0;
        if uuid == GATT_UUID_PRIMARY_SERVICE
            && HANDLE_PRIMARY_SERVICE >= start
            && HANDLE_PRIMARY_SERVICE <= end
        {
            out[n].0 = HANDLE_PRIMARY_SERVICE;
            out[n].1[..16].copy_from_slice(&NUS_SERVICE_UUID);
            out[n].2 = 16;
            n += 1;
        }
        if uuid == GATT_UUID_CHARACTERISTIC {
            if HANDLE_RX_CHAR_DECL >= start && HANDLE_RX_CHAR_DECL <= end {
                out[n].0 = HANDLE_RX_CHAR_DECL;
                out[n].1[0] = 0x08;
                out[n].1[1..3].copy_from_slice(&HANDLE_RX_VALUE.to_le_bytes());
                out[n].1[3..19].copy_from_slice(&NUS_RX_CHAR_UUID);
                out[n].2 = 19;
                n += 1;
            }
            if HANDLE_TX_CHAR_DECL >= start && HANDLE_TX_CHAR_DECL <= end {
                out[n].0 = HANDLE_TX_CHAR_DECL;
                out[n].1[0] = 0x20;
                out[n].1[1..3].copy_from_slice(&HANDLE_TX_VALUE.to_le_bytes());
                out[n].1[3..19].copy_from_slice(&NUS_TX_CHAR_UUID);
                out[n].2 = 19;
                n += 1;
            }
        }
        if uuid == GATT_UUID_CLIENT_CHAR_CFG && HANDLE_TX_CCCD >= start && HANDLE_TX_CCCD <= end {
            out[n].0 = HANDLE_TX_CCCD;
            out[n].2 = 2;
            n += 1;
        }
        (out, n)
    }

    fn primary_services(&self, start: u16, end: u16) -> ([(u16, [u8; 16]); 1], usize) {
        let mut out = [(0u16, [0u8; 16]); 1];
        let mut n = 0;
        if HANDLE_PRIMARY_SERVICE >= start && HANDLE_PRIMARY_SERVICE <= end {
            out[n] = (HANDLE_PRIMARY_SERVICE, NUS_SERVICE_UUID);
            n += 1;
        }
        (out, n)
    }

    fn read(&self, handle: u16, out: &mut [u8; 32]) -> Option<usize> {
        match handle {
            HANDLE_TX_VALUE => Some(0),
            HANDLE_TX_CCCD => {
                out[0] = self.nus_cccd;
                out[1] = 0;
                Some(2)
            }
            _ => None,
        }
    }

    fn write(&mut self, handle: u16, value: &[u8], result: &mut ConnEvent) -> Result<(), Error> {
        match handle {
            HANDLE_RX_VALUE => {
                let len = value.len().min(self.rx_data.len());
                self.rx_data[..len].copy_from_slice(&value[..len]);
                self.rx_data_len = len;
                *result = ConnEvent::Data;
                Ok(())
            }
            HANDLE_TX_CCCD => {
                self.nus_cccd = if value.len() >= 2 && value[0] & 0x01 != 0 {
                    0x01
                } else {
                    0
                };
                Ok(())
            }
            _ => Err(Error::InvalidPdu),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Outcome of one connection event.
pub enum ConnEvent {
    /// Nothing to report.
    Idle,
    /// The peer wrote data (see `rx_data`).
    Data,
    /// The peer sent an LL control PDU (opcode).
    Control(u8),
    /// The peer sent an L2CAP PDU on a non-ATT channel.
    L2cap(u16),
    /// The link ended.
    Disconnected(DisconnectReason),
}

pub fn next_channel(current: u8, hop: u8, map: &[u8; 5]) -> u8 {
    let mut ch = (current + hop) % 37;
    let mut guard = 0;
    while (map[ch as usize / 8] & (1 << (ch % 8))) == 0 && guard < 37 {
        ch = (ch + 1) % 37;
        guard += 1;
    }
    ch
}

#[derive(Debug, Clone, Copy)]
enum GattUuid {
    Uuid16(u16),
    Uuid128([u8; 16]),
}

impl GattUuid {
    fn as_u16(&self) -> Option<u16> {
        match self {
            GattUuid::Uuid16(u) => Some(*u),
            _ => None,
        }
    }
    fn to_128(self) -> [u8; 16] {
        match self {
            GattUuid::Uuid128(u) => u,
            _ => [0; 16],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn conn() -> Conn {
        let params = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        Conn::new(&params, 37, 0, ConnRole::Slave, ccm_regs())
    }

    fn ccm_regs() -> &'static pac::ccm::RegisterBlock {
        unsafe { &*pac::CCM::ptr() }
    }

    fn run_att(conn: &mut Conn, request: &[u8]) -> [u8; 256] {
        let mut result = ConnEvent::Idle;
        let mut out = [0u8; 256];
        let mut att = [0u8; 64];
        att[..request.len()].copy_from_slice(request);
        conn.handle_att(&att[..request.len()], &mut result);
        let n = conn.tx_att_len;
        out[..n].copy_from_slice(&conn.tx_att[..n]);
        out
    }

    #[test]
    fn exchange_mtu() {
        let mut c = conn();
        let rsp = run_att(&mut c, &[0x02, 0x27, 0x00]);
        assert_eq!(&rsp[..5], &[0x03, 23, 0, 0, 0]);
        assert_eq!(rsp[0], 0x03);
    }

    #[test]
    fn find_information_lists_handles() {
        let mut c = conn();
        let rsp = run_att(&mut c, &[0x04, 0x01, 0x00, 0xFF, 0xFF]);
        assert_eq!(rsp[0], 0x05);
        assert_eq!(rsp[1], 0x02);
        assert_eq!(&rsp[2..4], &[0x01, 0x00]);
        assert_eq!(&rsp[4..20], &NUS_SERVICE_UUID);
    }

    #[test]
    fn read_by_group_type_primary_service() {
        let mut c = conn();
        let rsp = run_att(&mut c, &[0x10, 0x00, 0x28, 0x01, 0x00, 0xFF, 0xFF]);
        assert_eq!(rsp[0], 0x11);
        assert_eq!(rsp[1], 18);
        assert_eq!(&rsp[2..4], &[0x01, 0x00]);
        assert_eq!(&rsp[4..20], &NUS_SERVICE_UUID);
    }

    #[test]
    fn read_by_type_characteristics() {
        let mut c = conn();
        let rsp = run_att(&mut c, &[0x08, 0x03, 0x28, 0x01, 0x00, 0xFF, 0xFF]);
        assert_eq!(rsp[0], 0x09);
        assert_eq!(rsp[1], 21);
        assert_eq!(&rsp[2..4], &[0x02, 0x00]);
        assert_eq!(rsp[4], 0x08);
        assert_eq!(&rsp[5..7], &[0x03, 0x00]);
        assert_eq!(&rsp[7..23], &NUS_RX_CHAR_UUID);
    }

    #[test]
    fn write_rx_characteristic_reports_data() {
        let mut c = conn();
        let mut result = ConnEvent::Idle;
        let mut att = [0u8; 64];
        let req = [0x12u8, 0x03, 0x00, b'h', b'i'];
        att[..req.len()].copy_from_slice(&req);
        c.handle_att(&att[..req.len()], &mut result);
        assert_eq!(result, ConnEvent::Data);
        assert_eq!(&c.rx_data[..2], b"hi");
        assert_eq!(c.tx_att[0], 0x13);
    }

    #[test]
    fn write_cccd_enables_notifications() {
        let mut c = conn();
        let mut result = ConnEvent::Idle;
        let mut att = [0u8; 64];
        let req = [0x12u8, 0x06, 0x00, 0x01, 0x00];
        att[..req.len()].copy_from_slice(&req);
        c.handle_att(&att[..req.len()], &mut result);
        assert_eq!(c.nus_cccd, 0x01);
    }

    #[test]
    fn read_cccd() {
        let mut c = conn();
        c.nus_cccd = 1;
        let rsp = run_att(&mut c, &[0x0A, 0x06, 0x00]);
        assert_eq!(&rsp[..3], &[0x0B, 0x01, 0x00]);
    }

    #[test]
    fn queue_notify_builds_att_pdu() {
        let mut c = conn();
        c.queue_notify(b"ping").unwrap();
        assert_eq!(c.tx_att[0], 0x1B);
        assert_eq!(&c.tx_att[1..3], &[0x05, 0x00]);
        assert_eq!(&c.tx_att[3..7], b"ping");
    }

    #[test]
    fn channel_hopping_skips_unused_channels() {
        let mut map = [0xFFu8; 5];
        map[0] = 0xFC;
        let ch = next_channel(0, 1, &map);
        assert_eq!(ch, 2);
    }

    #[test]
    fn terminate_flag_sends_control_pdu() {
        let mut c = conn();
        c.terminate();
        assert!(c.terminate_pending);
    }
}

#[cfg(test)]
mod gatt_client_tests {
    use super::*;

    fn conn() -> Conn {
        let params = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        Conn::new(&params, 37, 0, ConnRole::Master, ccm_regs())
    }

    fn ccm_regs() -> &'static pac::ccm::RegisterBlock {
        unsafe { &*pac::CCM::ptr() }
    }

    fn client_request(conn: &mut Conn, f: impl FnOnce(&mut Conn)) -> [u8; 32] {
        f(conn);
        let mut out = [0u8; 32];
        let n = conn.tx_att_len;
        out[..n].copy_from_slice(&conn.tx_att[..n]);
        out
    }

    fn feed_response(conn: &mut Conn, rsp: &[u8]) {
        let mut result = ConnEvent::Idle;
        conn.handle_att(rsp, &mut result);
    }

    #[test]
    fn discover_primary_services_request() {
        let mut c = conn();
        let req = client_request(&mut c, |c| {
            c.gatt_send_request(&[0x10, 0x00, 0x28, 0x01, 0x00, 0xFF, 0xFF])
        });
        assert_eq!(&req[..7], &[0x10, 0x00, 0x28, 0x01, 0x00, 0xFF, 0xFF]);
    }

    #[test]
    fn read_request_format() {
        let mut c = conn();
        let req = client_request(&mut c, |c| c.gatt_send_request(&[0x0A, 0x05, 0x00]));
        assert_eq!(&req[..3], &[0x0A, 0x05, 0x00]);
    }

    #[test]
    fn write_request_format() {
        let mut c = conn();
        let req = client_request(&mut c, |c| {
            let mut r = [0u8; 23];
            r[0] = 0x12;
            r[1..3].copy_from_slice(&0x0003u16.to_le_bytes());
            r[3..5].copy_from_slice(b"hi");
            c.gatt_send_request(&r[..5]);
        });
        assert_eq!(&req[..5], &[0x12, 0x03, 0x00, b'h', b'i']);
    }

    #[test]
    fn response_routing_stores_result() {
        let mut c = conn();
        c.gatt_send_request(&[0x10, 0x00, 0x28, 0x01, 0x00, 0xFF, 0xFF]);
        feed_response(&mut c, &[0x11, 18, 0x01, 0x00, 0x9E, 0xCA, 0xDC, 0x24]);
        let (op, buf, len) = c.gatt_take_result();
        assert_eq!(op, 0x11);
        assert_eq!(&buf[..len], &[0x11, 18, 0x01, 0x00, 0x9E, 0xCA, 0xDC, 0x24]);
    }

    #[test]
    fn notification_routed_as_data() {
        let mut c = conn();
        let mut result = ConnEvent::Idle;
        c.handle_att(&[0x1B, 0x05, 0x00, b'n', b'o', b't'], &mut result);
        assert_eq!(result, ConnEvent::Data);
        assert_eq!(&c.rx_data[..3], b"not");
    }

    #[test]
    fn response_clears_pending() {
        let mut c = conn();
        c.gatt_send_request(&[0x0A, 0x05, 0x00]);
        assert!(c.pending_request);
        feed_response(&mut c, &[0x0B, 0x01, 0x02, 0x03]);
        assert!(!c.pending_request);
    }

    #[test]
    fn discovery_last_handle_parsing() {
        let rsp = [
            0x11u8, 18, 0x01, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        ];
        assert_eq!(Conn::att_response_last_handle(&rsp), Some(0x0001));
        let rsp2 = [
            0x09u8, 7, 0x02, 0x00, 1, 2, 3, 4, 5, 0x05, 0x00, 6, 7, 8, 9, 10,
        ];
        assert_eq!(Conn::att_response_last_handle(&rsp2), Some(0x0005));
        assert_eq!(Conn::att_response_last_handle(&[0x09]), None);
    }

    #[test]
    fn error_response_routed() {
        let mut c = conn();
        c.gatt_send_request(&[0x0A, 0x05, 0x00]);
        feed_response(&mut c, &[0x01, 0x0A, 0x05, 0x00, 0x0A]);
        let (op, _, _) = c.gatt_take_result();
        assert_eq!(op, 0x01);
    }
}

#[cfg(test)]
mod data_path_tests {
    use super::*;

    fn conn() -> Conn {
        let params = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        Conn::new(&params, 37, 0, ConnRole::Slave, ccm_regs())
    }

    fn ccm_regs() -> &'static pac::ccm::RegisterBlock {
        unsafe { &*pac::CCM::ptr() }
    }

    #[test]
    fn mtu_exchange_negotiates_min() {
        let mut c = conn();
        c.gatt_send_request(&[0x02, 0x10, 0x00]);
        c.handle_att(&[0x03, 0x17, 0x00], &mut ConnEvent::Idle);
        assert_eq!(c.att_mtu, 23);
        c.att_mtu = 247;
        c.gatt_send_request(&[0x02, 0xF7, 0x00]);
        c.handle_att(&[0x03, 0x27, 0x00], &mut ConnEvent::Idle);
        assert_eq!(c.att_mtu, 39);
    }

    #[test]
    fn long_att_pdu_fragments_across_events() {
        let mut c = conn();
        let big = [0x42u8; 60];
        c.queue_att(&big);
        // first fragment: 27-4 = 23 ATT bytes with the L2CAP header
        c.tx_fragment();
        let buf = c.tx_buf;
        assert_eq!(buf[0] & 0b11, LLID_DATA_START);
        assert_eq!(buf[1] as usize, 4 + 23);
        assert!(c.tx_pending);
        // continuation: 27 bytes
        c.tx_fragment();
        let buf = c.tx_buf;
        assert_eq!(buf[0] & 0b11, LLID_DATA_CONTINUATION_OR_COMPLETE);
        assert_eq!(buf[1] as usize, 27);
        // final: 10 bytes
        c.tx_fragment();
        let buf = c.tx_buf;
        assert_eq!(buf[0] & 0b11, LLID_DATA_CONTINUATION_OR_COMPLETE);
        assert_eq!(buf[1] as usize, 10);
        assert!(!c.tx_pending);
    }

    #[test]
    fn single_pdu_uses_complete_llid() {
        let mut c = conn();
        c.queue_att(&[0x42u8; 10]);
        c.tx_fragment();
        let buf = c.tx_buf;
        assert_eq!(buf[0] & 0b11, LLID_DATA_COMPLETE);
        assert!(!c.tx_pending);
    }

    #[test]
    fn rx_reassembly_across_fragments() {
        let mut c = conn();
        let mut result = ConnEvent::Idle;
        // first fragment: L2CAP header (len 6, ATT cid) + 4 ATT bytes
        let mut f1 = [0u8; 7];
        f1[0..2].copy_from_slice(&5u16.to_le_bytes());
        f1[2..4].copy_from_slice(&L2CAP_ATT_CID.to_le_bytes());
        f1[4..7].copy_from_slice(&[0x0A, 0x06, 0x00]);
        c.handle_l2cap(&f1, LLID_DATA_START, &mut result);
        assert_eq!(c.rx_l2cap_len, 3);
        // continuation: remaining 2 ATT bytes
        let f2 = [0x01u8, 0x02];
        c.handle_l2cap(&f2, LLID_DATA_CONTINUATION_OR_COMPLETE, &mut result);
        assert_eq!(c.rx_l2cap_len, 0);
        assert!(c.tx_pending);
    }

    #[test]
    fn dle_negotiation_updates_pdu_max() {
        let mut c = conn();
        let mut req = [0u8; 9];
        req[0] = LL_CONTROL_LENGTH_REQ;
        req[1..3].copy_from_slice(&251u16.to_le_bytes());
        req[3..5].copy_from_slice(&2120u16.to_le_bytes());
        req[5..7].copy_from_slice(&251u16.to_le_bytes());
        req[7..9].copy_from_slice(&2120u16.to_le_bytes());
        let r = Radio::dummy();
        c.handle_ll_control(&req, &r, &BtTimer::dummy()).unwrap();
        assert_eq!(c.tx_pdu_max, 251);
        assert!(c.pending_control.is_some());
    }

    #[test]
    fn read_blob_serves_offset() {
        let mut c = conn();
        c.nus_cccd = 0;
        let mut out = [0u8; 256];
        let req = [0x0Cu8, 0x05, 0x00, 0x00, 0x00];
        let mut result = ConnEvent::Idle;
        c.handle_att(&req, &mut result);
        let n = c.tx_att_len;
        out[..n].copy_from_slice(&c.tx_att[..n]);
        assert_eq!(out[0], 0x01);
    }
}

#[cfg(test)]
impl Radio {
    fn dummy() -> Radio {
        Radio::new(unsafe { pac::Peripherals::steal() }.RADIO)
    }
}

#[cfg(test)]
impl BtTimer {
    fn dummy() -> BtTimer {
        BtTimer::new(unsafe { pac::Peripherals::steal() }.TIMER0)
    }
}

#[cfg(test)]
mod coc_tests {
    use super::*;

    fn conn() -> Conn {
        let params = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        Conn::new(&params, 37, 0, ConnRole::Slave, ccm_regs())
    }

    fn ccm_regs() -> &'static pac::ccm::RegisterBlock {
        unsafe { &*pac::CCM::ptr() }
    }

    #[test]
    fn open_request_builds_correct_message() {
        let mut c = conn();
        c.l2cap_connect(0x1234);
        // the signaling message is queued into tx_att
        let msg = &c.tx_att[..c.tx_att_len];
        assert_eq!(msg[0], LE_CREDIT_BASED_CONNECTION_REQ);
        assert_eq!(u16::from_le_bytes([msg[2], msg[3]]), 0x1234);
        assert_eq!(u16::from_le_bytes([msg[4], msg[5]]), COC_LOCAL_CID);
        assert_eq!(c.tx_cid, L2CAP_SIGNALING_CID);
    }

    #[test]
    fn peer_open_request_responds_and_opens() {
        let mut c = conn();
        let req = [
            0x14u8, 0x00, // code, id
            0x34, 0x12, // psm
            0x41, 0x00, // peer cid 0x0041
            0xF7, 0x00, // mtu 247
            0xFB, 0x00, // mps 251
            0x08, 0x00, // credits
        ];
        c.handle_le_signaling(&req);
        let coc = c.coc.unwrap();
        assert_eq!(coc.psm, 0x1234);
        assert_eq!(coc.peer_cid, 0x0041);
        assert_eq!(coc.credits, 8);
        let rsp = &c.tx_att[..c.tx_att_len];
        assert_eq!(rsp[0], LE_CREDIT_BASED_CONNECTION_RSP);
        assert_eq!(u16::from_le_bytes([rsp[4], rsp[5]]), COC_MTU);
    }

    #[test]
    fn credit_flow_limits_sending() {
        let mut c = conn();
        let req = [
            0x14u8, 0x00, 0x34, 0x12, 0x41, 0x00, 0xF7, 0x00, 0xFB, 0x00, 0x01, 0x00,
        ];
        c.handle_le_signaling(&req);
        assert!(c.l2cap_send(b"hello").is_ok());
        assert!(c.l2cap_send(b"world").is_err());
        // peer returns a credit
        let credit = [0x16u8, 0x00, 0x40, 0x00, 0x01, 0x00];
        c.handle_le_signaling(&credit);
        assert!(c.l2cap_send(b"again").is_ok());
    }

    #[test]
    fn sdu_framing_and_reassembly() {
        let mut c = conn();
        let req = [
            0x14u8, 0x00, 0x34, 0x12, 0x41, 0x00, 0xF7, 0x00, 0xFB, 0x00, 0x08, 0x00,
        ];
        c.handle_le_signaling(&req);
        let mut result = ConnEvent::Idle;
        // one frame carrying the whole SDU
        let mut frame = [0u8; 10];
        frame[0..2].copy_from_slice(&5u16.to_le_bytes());
        frame[2..7].copy_from_slice(b"hello");
        c.rx_coc_sdu(&frame, &mut result);
        assert_eq!(result, ConnEvent::Data);
        assert_eq!(&c.rx_data[..5], b"hello");
    }

    #[test]
    fn multi_frame_sdu_assembled() {
        let mut c = conn();
        let req = [
            0x14u8, 0x00, 0x34, 0x12, 0x41, 0x00, 0xF7, 0x00, 0xFB, 0x00, 0x08, 0x00,
        ];
        c.handle_le_signaling(&req);
        let mut result = ConnEvent::Idle;
        let mut frame = [0u8; 7];
        frame[0..2].copy_from_slice(&6u16.to_le_bytes());
        frame[2..7].copy_from_slice(b"hello");
        c.rx_coc_sdu(&frame, &mut result);
        assert_eq!(result, ConnEvent::Idle);
        c.rx_coc_sdu(b"!", &mut result);
        assert_eq!(result, ConnEvent::Data);
        assert_eq!(&c.rx_data[..6], b"hello!");
    }
}

#[cfg(test)]
mod opcode_tests {
    use super::*;

    fn conn() -> Conn {
        let params = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 1,
            win_offset: 0,
            interval: 24,
            latency: 0,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        Conn::new(&params, 37, 0, ConnRole::Slave, ccm_regs())
    }

    fn ccm_regs() -> &'static pac::ccm::RegisterBlock {
        unsafe { &*pac::CCM::ptr() }
    }

    #[test]
    fn ll_control_opcodes_match_spec() {
        assert_eq!(LL_CONTROL_CONNECTION_UPDATE_REQ, 0x00);
        assert_eq!(LL_CONTROL_TERMINATE_IND, 0x02);
        assert_eq!(LL_CONTROL_ENCRYPT_REQ, 0x03);
        assert_eq!(LL_CONTROL_ENCRYPT_RSP, 0x04);
        assert_eq!(LL_CONTROL_START_ENC_REQ, 0x05);
        assert_eq!(LL_CONTROL_START_ENC_RSP, 0x06);
        assert_eq!(LL_CONTROL_UNKNOWN_RSP, 0x07);
        assert_eq!(LL_CONTROL_FEATURE_REQ, 0x08);
        assert_eq!(LL_CONTROL_FEATURE_RSP, 0x09);
        assert_eq!(LL_CONTROL_VERSION_IND, 0x0C);
        assert_eq!(LL_CONTROL_PING_REQ, 0x12);
        assert_eq!(LL_CONTROL_PING_RSP, 0x13);
        assert_eq!(LL_CONTROL_LENGTH_REQ, 0x14);
        assert_eq!(LL_CONTROL_LENGTH_RSP, 0x15);
        assert_eq!(LL_CONTROL_PHY_REQ, 0x16);
        assert_eq!(LL_CONTROL_PHY_RSP, 0x17);
    }

    #[test]
    fn ping_is_answered_with_ping_rsp() {
        let mut c = conn();
        let payload = [LL_CONTROL_PING_REQ];
        let r = Radio::dummy();
        let t = BtTimer::dummy();
        c.handle_ll_control(&payload, &r, &t).unwrap();
        let rsp = c.pending_control.unwrap();
        assert_eq!(rsp[0], LL_CONTROL_PING_RSP);
    }

    #[test]
    fn unknown_opcode_gets_unknown_rsp() {
        let mut c = conn();
        let r = Radio::dummy();
        let t = BtTimer::dummy();
        c.handle_ll_control(&[0x7F], &r, &t).unwrap();
        let rsp = c.pending_control.unwrap();
        assert_eq!(rsp[0], LL_CONTROL_UNKNOWN_RSP);
        assert_eq!(rsp[1], 0x7F);
    }
}
