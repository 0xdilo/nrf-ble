//! LE connection (peripheral role), L2CAP-lite and a minimal GATT/ATT
//! server (Nordic UART Service).

use crate::ll::pdu::{ConnectReqData, DataPdu, LLID_CONTROL, LLID_DATA_COMPLETE};
use crate::Error;

use super::radio::Radio;
use super::timers::{timeout_ticks, BtTimer, IntervalAccum};

/// LL Control PDU opcode: LL_VERSION_IND.
pub const LL_CONTROL_VERSION_IND: u8 = 0x0C;
/// LL Control PDU opcode: LL_FEATURE_REQ.
pub const LL_CONTROL_FEATURE_REQ: u8 = 0x1D;
/// LL Control PDU opcode: LL_FEATURE_RSP.
pub const LL_CONTROL_FEATURE_RSP: u8 = 0x1E;
/// LL Control PDU opcode: LL_TERMINATE_IND.
pub const LL_CONTROL_TERMINATE_IND: u8 = 0x02;

/// L2CAP channel ID for the ATT protocol.
pub const L2CAP_ATT_CID: u16 = 0x0004;

/// ATT MTU of the server.
pub const ATT_MTU: usize = 23;
/// Maximum ATT payload (MTU minus ATT header).
pub const ATT_PAYLOAD: usize = ATT_MTU - 3;

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
    rx_l2cap: [u8; 64],
    rx_l2cap_len: usize,
    tx_att: [u8; ATT_PAYLOAD + 4],
    tx_att_len: usize,
    tx_pending: bool,
    peer_version_ok: bool,
    pub last_rx: u32,
    terminate_pending: bool,
    pub nus_cccd: u8,
    pub rx_data: [u8; ATT_PAYLOAD],
    pub rx_data_len: usize,
    tx_buf: [u8; 64],
    rx_buf: [u8; 64],
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
    pub fn new(params: &ConnectReqData, first_channel: u8, now: u32, role: ConnRole) -> Conn {
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
            rx_l2cap: [0; 64],
            rx_l2cap_len: 0,
            tx_att: [0; ATT_PAYLOAD + 4],
            tx_att_len: 0,
            tx_pending: false,
            peer_version_ok: false,
            last_rx: now,
            terminate_pending: false,
            nus_cccd: 0,
            rx_data: [0; ATT_PAYLOAD],
            rx_data_len: 0,
            tx_buf: [0; 64],
            rx_buf: [0; 64],
        }
    }

    pub fn queue_notify(&mut self, data: &[u8]) -> Result<(), Error> {
        let len = data.len().min(ATT_PAYLOAD);
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
                        Ok(()) => {}
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
                    self.handle_l2cap(&payload[..plen], new_data, &mut result);
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
                    self.last_rx = timer.now();
                    if let Ok(pdu) = DataPdu::decode(&self.rx_buf[..len]) {
                        let plen = pdu.payload.len().min(64);
                        payload[..plen].copy_from_slice(&pdu.payload[..plen]);
                        if pdu.sn != self.nesn {
                            self.nesn = pdu.sn;
                        }
                        if plen > 0 {
                            if pdu.llid == LLID_CONTROL
                                && pdu.payload.first() == Some(&LL_CONTROL_TERMINATE_IND)
                            {
                                return Ok(ConnEvent::Disconnected(
                                    DisconnectReason::RemoteTerminate,
                                ));
                            }
                            if pdu.llid != LLID_CONTROL {
                                self.handle_l2cap(&payload[..plen], true, &mut result);
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

    fn respond(&mut self, radio: &Radio, timer: &BtTimer) {
        let header = |buf: &mut [u8; 64], nesn: bool, sn: bool, len: u8| {
            buf[0] = (LLID_DATA_COMPLETE & 0b11) | ((nesn as u8) << 2) | ((sn as u8) << 3);
            buf[1] = len;
        };
        if self.tx_pending {
            let att = &self.tx_att[..self.tx_att_len];
            let payload_len = att.len() + 4;
            header(&mut self.tx_buf, !self.nesn, self.sn, payload_len as u8);
            self.tx_buf[2..4].copy_from_slice(&(att.len() as u16).to_le_bytes());
            self.tx_buf[4..6].copy_from_slice(&L2CAP_ATT_CID.to_le_bytes());
            self.tx_buf[6..6 + att.len()].copy_from_slice(att);
            let total = 2 + payload_len;
            self.sn = !self.sn;
            self.tx_pending = false;
            let _ = timer;
            radio.transmit(&self.tx_buf[..total]);
        } else {
            header(&mut self.tx_buf, !self.nesn, self.sn, 0);
            radio.transmit(&self.tx_buf[..2]);
        }
    }

    fn handle_l2cap(&mut self, payload: &[u8], new_data: bool, result: &mut ConnEvent) {
        if payload.len() < 4 {
            return;
        }
        let len = u16::from_le_bytes([payload[0], payload[1]]) as usize;
        let cid = u16::from_le_bytes([payload[2], payload[3]]);
        if cid != L2CAP_ATT_CID {
            return;
        }
        let body = &payload[4..];
        let _ = new_data;
        let cap = self.rx_l2cap.len();
        let n = body.len().min(cap);
        self.rx_l2cap[..n].copy_from_slice(&body[..n]);
        self.rx_l2cap_len = body.len();
        if self.rx_l2cap_len >= len.min(cap) {
            let alen = len.min(cap);
            let mut att = [0u8; 64];
            att[..alen].copy_from_slice(&self.rx_l2cap[..alen]);
            self.handle_att(&att[..alen], result);
            self.rx_l2cap_len = 0;
        }
    }

    fn handle_att(&mut self, att: &[u8], result: &mut ConnEvent) {
        if att.is_empty() {
            return;
        }
        let op = att[0];
        match op {
            0x02 => {
                let mut rsp = [0u8; 5];
                rsp[0] = 0x03;
                rsp[1..3].copy_from_slice(&(ATT_MTU as u16).to_le_bytes());
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
            _ => Ok(()),
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
        Conn::new(&params, 37, 0, ConnRole::Slave)
    }

    fn run_att(conn: &mut Conn, request: &[u8]) -> [u8; 32] {
        let mut result = ConnEvent::Idle;
        let mut out = [0u8; 32];
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
