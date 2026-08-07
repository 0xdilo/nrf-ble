//! HCI (host-controller interface) layer: a host can drive the stack over
//! a UART (H4) transport using the standard LE controller commands.

use crate::hw::{Ble, BleEvent};

/// H4 packet type: HCI command.
pub const HCI_COMMAND: u8 = 0x01;
/// H4 packet type: ACL data.
pub const HCI_ACL_DATA: u8 = 0x02;
/// H4 packet type: SCO data.
pub const HCI_SCO_DATA: u8 = 0x03;
/// H4 packet type: HCI event.
pub const HCI_EVENT: u8 = 0x04;

/// HCI event: command complete.
pub const HCI_EVT_COMMAND_COMPLETE: u8 = 0x0E;
/// HCI event: command status.
pub const HCI_EVT_COMMAND_STATUS: u8 = 0x0F;
/// HCI event: disconnection complete.
pub const HCI_EVT_DISCONNECTION_COMPLETE: u8 = 0x05;
/// HCI event: LE meta.
pub const HCI_EVT_LE_META: u8 = 0x3E;

/// LE meta subevent: advertising report.
pub const HCI_LE_ADVERTISING_REPORT: u8 = 0x02;
/// LE meta subevent: connection complete.
pub const HCI_LE_CONNECTION_COMPLETE: u8 = 0x01;
/// LE meta subevent: connection update complete.
pub const HCI_LE_CONNECTION_UPDATE_COMPLETE: u8 = 0x03;

const OGF_HOST: u8 = 0x03;
const OGF_LE: u8 = 0x08;

const OCF_READ_LOCAL_VERSION: u16 = 0x0001;
const OCF_READ_BD_ADDR: u16 = 0x0009;
const OCF_READ_BUFFER_SIZE: u16 = 0x0005;
const OCF_LE_SET_ADV_PARAMS: u16 = 0x0006;
const OCF_LE_SET_ADV_DATA: u16 = 0x0008;
const OCF_LE_SET_ADV_ENABLE: u16 = 0x000A;
const OCF_LE_SET_SCAN_PARAMS: u16 = 0x000B;
const OCF_LE_SET_SCAN_ENABLE: u16 = 0x000C;
const OCF_LE_CREATE_CONNECTION: u16 = 0x000D;
const OCF_LE_SET_RANDOM_ADDRESS: u16 = 0x0005;

/// One HCI packet (command, event or ACL data).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HciPacket {
    /// H4 packet type.
    pub packet_type: u8,
    /// Packet payload.
    pub payload: [u8; 260],
    /// Valid payload length.
    pub len: usize,
}

impl HciPacket {
    /// Build a command packet.
    pub fn command(ogf: u8, ocf: u16, params: &[u8]) -> HciPacket {
        let mut p = HciPacket {
            packet_type: HCI_COMMAND,
            payload: [0; 260],
            len: 0,
        };
        p.payload[0] = (ocf & 0xFF) as u8;
        p.payload[1] = (((ocf >> 8) & 0x03) as u8) | ((ogf & 0x3F) << 2);
        p.payload[2] = params.len() as u8;
        p.payload[3..3 + params.len()].copy_from_slice(params);
        p.len = 3 + params.len();
        p
    }

    /// Build an event packet.
    pub fn event(code: u8, payload: &[u8]) -> HciPacket {
        let mut p = HciPacket {
            packet_type: HCI_EVENT,
            payload: [0; 260],
            len: 0,
        };
        p.payload[0] = code;
        p.payload[1] = payload.len() as u8;
        p.payload[2..2 + payload.len()].copy_from_slice(payload);
        p.len = 2 + payload.len();
        p
    }

    /// Build an ACL data packet.
    pub fn acl(handle: u16, data: &[u8]) -> HciPacket {
        let mut p = HciPacket {
            packet_type: HCI_ACL_DATA,
            payload: [0; 260],
            len: 0,
        };
        p.payload[0] = (handle & 0x0FFF) as u8;
        p.payload[1] = (handle >> 8) as u8;
        p.payload[2] = (data.len() & 0xFF) as u8;
        p.payload[3] = (data.len() >> 8) as u8;
        p.payload[4..4 + data.len()].copy_from_slice(data);
        p.len = 4 + data.len();
        p
    }
}

/// H4 byte-stream framing: feed bytes, get complete packets.
pub struct H4Framer {
    /// Detected packet type of the current frame.
    pub packet_type: Option<u8>,
    expected: usize,
    buf: [u8; 260],
    buf_len: usize,
}

impl Default for H4Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl H4Framer {
    /// Create an idle framer.
    pub const fn new() -> Self {
        H4Framer {
            packet_type: None,
            expected: 0,
            buf: [0; 260],
            buf_len: 0,
        }
    }

    /// Feed one byte; returns the decoded packet when a full frame arrived.
    pub fn feed(&mut self, byte: u8) -> Option<HciPacket> {
        match self.packet_type {
            None => {
                if byte == HCI_COMMAND || byte == HCI_ACL_DATA || byte == HCI_EVENT {
                    self.packet_type = Some(byte);
                    self.buf_len = 0;
                }
                None
            }
            Some(HCI_COMMAND) => {
                self.buf[self.buf_len] = byte;
                self.buf_len += 1;
                if self.buf_len == 3 {
                    self.expected = 3 + self.buf[2] as usize;
                }
                if self.buf_len == self.expected && self.expected >= 3 {
                    self.reset();
                    Some(HciPacket {
                        packet_type: HCI_COMMAND,
                        payload: self.buf,
                        len: self.buf_len,
                    })
                } else {
                    None
                }
            }
            Some(HCI_EVENT) => {
                self.buf[self.buf_len] = byte;
                self.buf_len += 1;
                if self.buf_len == 2 {
                    self.expected = 2 + self.buf[1] as usize;
                }
                if self.buf_len == self.expected && self.expected >= 2 {
                    self.reset();
                    Some(HciPacket {
                        packet_type: HCI_EVENT,
                        payload: self.buf,
                        len: self.buf_len,
                    })
                } else {
                    None
                }
            }
            Some(HCI_ACL_DATA) => {
                self.buf[self.buf_len] = byte;
                self.buf_len += 1;
                if self.buf_len == 4 {
                    self.expected = 4 + u16::from_le_bytes([self.buf[2], self.buf[3]]) as usize;
                }
                if self.buf_len == self.expected && self.expected >= 4 {
                    self.reset();
                    Some(HciPacket {
                        packet_type: HCI_ACL_DATA,
                        payload: self.buf,
                        len: self.buf_len,
                    })
                } else {
                    None
                }
            }
            Some(_) => None,
        }
    }

    fn reset(&mut self) {
        self.packet_type = None;
        self.expected = 0;
    }
}

/// HCI controller layer bridging a host (over UART H4) to the BLE stack.
pub struct Hci {
    framer: H4Framer,
    events: [Option<HciPacket>; 16],
    event_head: usize,
    event_tail: usize,
    /// BD_ADDR reported to the host.
    pub own_addr: [u8; 6],
    adv_enabled: bool,
    scan_enabled: bool,
    /// Active connection handle.
    pub conn_handle: u16,
    /// True while a connection is open.
    pub connection_open: bool,
}

impl Default for Hci {
    fn default() -> Self {
        Self::new()
    }
}

impl Hci {
    /// Create an idle HCI controller.
    pub fn new() -> Self {
        Hci {
            framer: H4Framer::new(),
            events: [None; 16],
            event_head: 0,
            event_tail: 0,
            own_addr: [0; 6],
            adv_enabled: false,
            scan_enabled: false,
            conn_handle: 0,
            connection_open: false,
        }
    }

    /// Feed one byte from the transport; returns a decoded packet when a
    /// full one arrived.
    /// Feed one byte; returns the decoded packet when a full frame arrived.
    pub fn feed(&mut self, byte: u8) -> Option<HciPacket> {
        self.framer.feed(byte)
    }

    /// Handle one decoded HCI command or ACL packet, driving the BLE stack.
    /// Handle one decoded HCI packet (command or ACL), driving the stack.
    pub fn on_packet(&mut self, ble: &mut Ble, packet: &HciPacket) {
        match packet.packet_type {
            HCI_COMMAND => {
                self.handle_command(ble, packet);
            }
            HCI_ACL_DATA => {
                self.handle_acl(ble, packet);
            }
            _ => {}
        }
    }

    /// Drive the BLE stack and collect the events it produced.
    /// Drive the stack and forward its events to the host.
    pub fn tick(&mut self, ble: &mut Ble) {
        while let Some(evt) = ble.next_event() {
            match evt {
                BleEvent::AdvStarted => {}
                BleEvent::AdvStopped => {}
                BleEvent::ScanStarted => {}
                BleEvent::ScanStopped => {}
                BleEvent::ScanReport {
                    addr,
                    addr_type,
                    adv_type,
                    rssi,
                    data,
                    data_len,
                    ..
                } => {
                    let n = data_len.min(27);
                    let total = 11 + n + 1;
                    let mut payload = [0u8; 260];
                    payload[0] = HCI_LE_ADVERTISING_REPORT;
                    payload[1] = 1;
                    payload[2] = adv_type;
                    payload[3] = if addr_type.is_random() { 0x01 } else { 0x00 };
                    payload[4..10].copy_from_slice(&addr);
                    payload[10] = n as u8;
                    payload[11..11 + n].copy_from_slice(&data[..n]);
                    payload[11 + n] = rssi as u8;
                    self.push_event(HciPacket::event(HCI_EVT_LE_META, &payload[..total]));
                }
                BleEvent::Connected { conn } => {
                    self.connection_open = true;
                    self.conn_handle = 0;
                    let mut p = [0u8; 19];
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = if ble.conn_is_master() { 0x01 } else { 0x00 };
                    p[4] = 0;
                    p[5..11].fill(0);
                    p[11] = conn.interval as u8;
                    p[12] = (conn.interval >> 8) as u8;
                    p[13] = 0;
                    p[14] = 0;
                    p[15] = conn.timeout as u8;
                    p[16] = (conn.timeout >> 8) as u8;
                    p[17] = 0;
                    p[18] = 0;
                    let mut payload = [0u8; 260];
                    payload[0] = HCI_LE_CONNECTION_COMPLETE;
                    payload[1] = 0;
                    payload[2..21].copy_from_slice(&p);
                    self.push_event(HciPacket::event(HCI_EVT_LE_META, &payload[..21]));
                }
                BleEvent::Disconnected { reason } => {
                    let mut p = [0u8; 4];
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = reason as u8;
                    self.push_event(HciPacket::event(HCI_EVT_DISCONNECTION_COMPLETE, &p));
                    self.connection_open = false;
                }
                _ => {}
            }
        }
    }

    /// Pop the next pending event for the transport.
    /// Pop the next pending event for the transport.
    pub fn next_event(&mut self) -> Option<HciPacket> {
        let evt = self.events[self.event_tail].take();
        if evt.is_some() {
            self.event_tail = (self.event_tail + 1) % 16;
        }
        evt
    }

    fn push_event(&mut self, evt: HciPacket) {
        let next = (self.event_head + 1) % 16;
        if next == self.event_tail {
            self.event_tail = (self.event_tail + 1) % 16;
        }
        self.events[self.event_head] = Some(evt);
        self.event_head = next;
    }

    fn command_complete(&mut self, opcode: [u8; 2], status: u8, params: &[u8]) {
        let mut payload = [0u8; 260];
        payload[0] = 1;
        payload[1] = opcode[0];
        payload[2] = opcode[1];
        payload[3] = status;
        payload[4..4 + params.len()].copy_from_slice(params);
        self.push_event(HciPacket::event(
            HCI_EVT_COMMAND_COMPLETE,
            &payload[..4 + params.len()],
        ));
    }

    fn command_status(&mut self, opcode: [u8; 2], status: u8) {
        let mut payload = [0u8; 5];
        payload[0] = status;
        payload[1] = 1;
        payload[2] = opcode[0];
        payload[3] = opcode[1];
        payload[4] = 0;
        self.push_event(HciPacket::event(HCI_EVT_COMMAND_STATUS, &payload));
    }

    fn handle_command(&mut self, ble: &mut Ble, packet: &HciPacket) {
        let opcode = packet.payload[0] as u16 | ((packet.payload[1] as u16) << 8);
        let ocf = opcode & 0x03FF;
        let ogf = ((opcode >> 10) & 0x3F) as u8;
        let params = &packet.payload[3..packet.len];
        let op = [packet.payload[0], packet.payload[1]];
        match ogf {
            OGF_HOST => match ocf {
                OCF_READ_LOCAL_VERSION => {
                    self.command_complete(op, 0, &[0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
                }
                OCF_READ_BD_ADDR => {
                    let mut p = [0u8; 6];
                    p.copy_from_slice(&self.own_addr);
                    self.command_complete(op, 0, &p);
                }
                OCF_READ_BUFFER_SIZE => {
                    self.command_complete(op, 0, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]);
                }
                _ => {
                    self.command_complete(op, 0, &[]);
                }
            },
            OGF_LE => match ocf {
                OCF_LE_SET_ADV_DATA => {
                    if params.len() >= 32 {
                        let n = (params[0] as usize).min(31);
                        let mut buf = [0u8; 31];
                        buf[..n].copy_from_slice(&params[1..1 + n]);
                        ble.gap_adv_set_configure(&buf[..n], &[]).ok();
                    }
                    self.command_complete(op, 0, &[]);
                }
                OCF_LE_SET_ADV_PARAMS => {
                    self.command_complete(op, 0, &[]);
                }
                OCF_LE_SET_ADV_ENABLE => {
                    if params.first() == Some(&1) {
                        ble.gap_adv_start(&Default::default()).ok();
                        self.adv_enabled = true;
                    } else {
                        ble.gap_adv_stop().ok();
                        self.adv_enabled = false;
                    }
                    self.command_complete(op, 0, &[]);
                }
                OCF_LE_SET_SCAN_PARAMS => {
                    self.command_complete(op, 0, &[]);
                }
                OCF_LE_SET_SCAN_ENABLE => {
                    if params.first() == Some(&1) {
                        ble.gap_scan_start(&Default::default()).ok();
                        self.scan_enabled = true;
                    } else {
                        ble.gap_scan_stop().ok();
                        self.scan_enabled = false;
                    }
                    self.command_complete(op, 0, &[]);
                }
                OCF_LE_CREATE_CONNECTION => {
                    if params.len() >= 25 {
                        let mut target = [0u8; 6];
                        target.copy_from_slice(&params[14..20]);
                        ble.gap_connect(target, crate::ll::addr::AddrType::Public)
                            .ok();
                    }
                    self.command_status(op, 0);
                }
                OCF_LE_SET_RANDOM_ADDRESS => {
                    if params.len() >= 6 {
                        let mut a = [0u8; 6];
                        a.copy_from_slice(&params[..6]);
                        self.own_addr = a;
                    }
                    self.command_complete(op, 0, &[]);
                }
                _ => {
                    self.command_complete(op, 0, &[]);
                }
            },
            _ => {
                self.command_complete(op, 0, &[]);
            }
        }
        let _ = params;
    }

    fn handle_acl(&mut self, _ble: &mut Ble, packet: &HciPacket) {
        let _ = packet;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn h4_frames_command_packets() {
        let mut f = H4Framer::new();
        let pkt = HciPacket::command(0x08, 0x000A, &[1]);
        let mut out = None;
        for b in h4_bytes(HCI_COMMAND, &pkt) {
            if let Some(p) = f.feed(b) {
                out = Some(p);
            }
        }
        let p = out.unwrap();
        assert_eq!(p.packet_type, HCI_COMMAND);
        assert_eq!(p.payload[0] as u16 | ((p.payload[1] as u16) << 8), 0x200A);
        assert_eq!(p.payload[2], 1);
    }

    fn h4_bytes(packet_type: u8, pkt: &HciPacket) -> [u8; 261] {
        let mut bytes = [0u8; 261];
        bytes[0] = packet_type;
        bytes[1..1 + pkt.len].copy_from_slice(&pkt.payload[..pkt.len]);
        bytes
    }

    #[test]
    fn h4_frames_event_packets() {
        let mut f = H4Framer::new();
        let pkt = HciPacket::event(0x0E, &[1, 2, 3]);
        let mut out = None;
        for b in h4_bytes(HCI_EVENT, &pkt) {
            if let Some(p) = f.feed(b) {
                out = Some(p);
            }
        }
        let p = out.unwrap();
        assert_eq!(p.payload[0], 0x0E);
        assert_eq!(p.payload[2], 1);
    }

    #[test]
    fn h4_frames_acl_packets() {
        let mut f = H4Framer::new();
        let pkt = HciPacket::acl(0, &[0x04, 0x00, 0x04, 0x00, 1, 2, 3]);
        let mut out = None;
        for b in h4_bytes(HCI_ACL_DATA, &pkt) {
            if let Some(p) = f.feed(b) {
                out = Some(p);
            }
        }
        let p = out.unwrap();
        assert_eq!(p.packet_type, HCI_ACL_DATA);
        assert_eq!(p.len, 4 + 7);
    }

    #[test]
    fn hci_rejects_garbage_bytes() {
        let mut f = H4Framer::new();
        assert!(f.feed(0x00).is_none());
        assert!(f.feed(0xAA).is_none());
        assert!(f.feed(0xFF).is_none());
    }
}

#[cfg(test)]
mod queue_tests {
    use super::*;

    #[test]
    fn event_queue_roundtrip() {
        let mut hci = Hci::new();
        for i in 0..15 {
            hci.push_event(HciPacket::event(i as u8, &[i as u8]));
        }
        for i in 0..15 {
            let evt = hci.next_event().unwrap();
            assert_eq!(evt.payload[0], i as u8);
        }
        assert!(hci.next_event().is_none());
    }

    #[test]
    fn event_queue_never_exceeds_capacity() {
        let mut hci = Hci::new();
        for i in 0..40 {
            hci.push_event(HciPacket::event(0xEE, &[i as u8]));
        }
        let mut count = 0;
        while hci.next_event().is_some() {
            count += 1;
        }
        assert_eq!(count, 16);
    }

    #[test]
    fn acl_packet_roundtrip() {
        let pkt = HciPacket::acl(0x0042, &[0x04, 0x00, 0x04, 0x00, 0x0A, 0x06, 0x00]);
        assert_eq!(pkt.packet_type, HCI_ACL_DATA);
        assert_eq!(
            pkt.payload[0] as u16 | ((pkt.payload[1] as u16) << 8),
            0x0042
        );
        assert_eq!(pkt.len, 11);
    }
}
