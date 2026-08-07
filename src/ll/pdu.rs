use crate::Error;

/// Advertising indication (connectable undirected).
pub const PDU_ADV_IND: u8 = 0b0000;
/// Directed advertising indication (connectable).
pub const PDU_ADV_DIRECT_IND: u8 = 0b0001;
/// Non-connectable advertising indication.
pub const PDU_ADV_NONCONN_IND: u8 = 0b0010;
/// Scan request.
pub const PDU_SCAN_REQ: u8 = 0b0011;
/// Scan response.
pub const PDU_SCAN_RSP: u8 = 0b0100;
/// Connection request.
pub const PDU_CONNECT_REQ: u8 = 0b0101;
/// Scannable undirected advertising indication.
pub const PDU_ADV_SCAN_IND: u8 = 0b0110;

/// LLID: LL Data PDU, start of a new L2CAP PDU.
pub const LLID_DATA_START: u8 = 0b01;
/// LLID: LL Data PDU, continuation or complete L2CAP PDU.
pub const LLID_DATA_CONTINUATION_OR_COMPLETE: u8 = 0b00;
/// LLID: LL Data PDU, complete L2CAP PDU.
pub const LLID_DATA_COMPLETE: u8 = 0b11;
/// LLID: LL Control PDU.
pub const LLID_CONTROL: u8 = 0b10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Advertising physical channel PDU header (2 bytes).
pub struct AdvHdr {
    /// PDU type (4 bits).
    pub pdu_type: u8,
    /// ChSel bit (Bluetooth 5.0+).
    pub ch_sel: bool,
    /// TxAdd bit: 1 = transmitter address is random.
    pub tx_add: bool,
    /// RxAdd bit: 1 = receiver address is random.
    pub rx_add: bool,
    /// Payload length in bytes (6 bits).
    pub len: u8,
}

impl AdvHdr {
    /// Encode the header into two bytes.
    pub const fn encode(&self) -> [u8; 2] {
        let mut h = [0u8; 2];
        h[0] = (self.pdu_type & 0x0F) | ((self.ch_sel as u8) << 6) | ((self.tx_add as u8) << 7);
        h[1] = (self.rx_add as u8) | ((self.len & 0x3F) << 1);
        h
    }

    /// Decode a two-byte header.
    pub const fn decode(bytes: [u8; 2]) -> Result<Self, Error> {
        let pdu_type = bytes[0] & 0x0F;
        if pdu_type > 0b0111 {
            return Err(Error::InvalidPdu);
        }
        Ok(AdvHdr {
            pdu_type,
            ch_sel: bytes[0] & 0x40 != 0,
            tx_add: bytes[0] & 0x80 != 0,
            rx_add: bytes[1] & 0x01 != 0,
            len: (bytes[1] >> 1) & 0x3F,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A decoded advertising physical channel PDU.
pub enum AdvPdu<'a> {
    /// ADV_IND: connectable undirected.
    AdvInd {
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Advertising data.
        data: &'a [u8],
    },
    /// ADV_DIRECT_IND: connectable directed.
    AdvDirectInd {
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Target (initiator) address.
        init_addr: &'a [u8; 6],
    },
    /// ADV_NONCONN_IND: non-connectable.
    AdvNonconnInd {
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Advertising data.
        data: &'a [u8],
    },
    /// SCAN_REQ.
    ScanReq {
        /// Scanner address.
        scan_addr: &'a [u8; 6],
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
    },
    /// SCAN_RSP.
    ScanRsp {
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Scan response data.
        data: &'a [u8],
    },
    /// CONNECT_REQ.
    ConnectReq {
        /// Initiator address.
        init_addr: &'a [u8; 6],
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Raw LLData (see [`ConnectReqData`]).
        ll_data: &'a [u8],
    },
    /// ADV_SCAN_IND: scannable, non-connectable.
    AdvScanInd {
        /// Advertiser address.
        adv_addr: &'a [u8; 6],
        /// Advertising data.
        data: &'a [u8],
    },
}

impl<'a> AdvPdu<'a> {
    /// Decode a full advertising channel PDU (header + payload).
    pub fn decode(bytes: &'a [u8]) -> Result<AdvPdu<'a>, Error> {
        if bytes.len() < 2 {
            return Err(Error::InvalidPdu);
        }
        let header = AdvHdr::decode([bytes[0], bytes[1]])?;
        let payload = &bytes[2..];
        if payload.len() != header.len as usize {
            return Err(Error::InvalidLength);
        }
        let payload: &'a [u8] = payload;
        match header.pdu_type {
            PDU_ADV_IND => {
                let (addr, data) = split6(payload)?;
                Ok(AdvPdu::AdvInd {
                    adv_addr: addr,
                    data,
                })
            }
            PDU_ADV_DIRECT_IND => {
                let (a, b) = split6(payload)?;
                if b.len() != 6 {
                    return Err(Error::InvalidLength);
                }
                Ok(AdvPdu::AdvDirectInd {
                    adv_addr: a,
                    init_addr: b.try_into().map_err(|_| Error::InvalidLength)?,
                })
            }
            PDU_ADV_NONCONN_IND => {
                let (addr, data) = split6(payload)?;
                Ok(AdvPdu::AdvNonconnInd {
                    adv_addr: addr,
                    data,
                })
            }
            PDU_SCAN_REQ => {
                let (a, b) = split6(payload)?;
                if b.len() != 6 {
                    return Err(Error::InvalidLength);
                }
                Ok(AdvPdu::ScanReq {
                    scan_addr: a,
                    adv_addr: b.try_into().map_err(|_| Error::InvalidLength)?,
                })
            }
            PDU_SCAN_RSP => {
                let (addr, data) = split6(payload)?;
                Ok(AdvPdu::ScanRsp {
                    adv_addr: addr,
                    data,
                })
            }
            PDU_CONNECT_REQ => {
                let (a, b) = split6(payload)?;
                if b.len() != 28 {
                    return Err(Error::InvalidLength);
                }
                Ok(AdvPdu::ConnectReq {
                    init_addr: a,
                    adv_addr: b[..6].try_into().map_err(|_| Error::InvalidLength)?,
                    ll_data: &b[6..],
                })
            }
            PDU_ADV_SCAN_IND => {
                let (addr, data) = split6(payload)?;
                Ok(AdvPdu::AdvScanInd {
                    adv_addr: addr,
                    data,
                })
            }
            _ => Err(Error::InvalidPdu),
        }
    }

    /// Encode the PDU into `out`, returning the number of bytes written.
    ///
    /// The TxAdd/RxAdd header bits are cleared; use [`AdvPdu::encode_typed`]
    /// when the address types are known.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        self.encode_typed(out, false, false)
    }

    /// Encode the PDU with explicit TxAdd/RxAdd address-type header bits.
    pub fn encode_typed(&self, out: &mut [u8], tx_add: bool, rx_add: bool) -> Result<usize, Error> {
        if out.len() < 2 {
            return Err(Error::BufferTooSmall);
        }
        match *self {
            AdvPdu::AdvInd { adv_addr, data } => {
                Self::encode_with(out, PDU_ADV_IND, tx_add, false, 6 + data.len(), |o| {
                    o[..6].copy_from_slice(adv_addr);
                    o[6..].copy_from_slice(data);
                })
            }
            AdvPdu::AdvScanInd { adv_addr, data } => {
                Self::encode_with(out, PDU_ADV_SCAN_IND, tx_add, false, 6 + data.len(), |o| {
                    o[..6].copy_from_slice(adv_addr);
                    o[6..].copy_from_slice(data);
                })
            }
            AdvPdu::AdvNonconnInd { adv_addr, data } => Self::encode_with(
                out,
                PDU_ADV_NONCONN_IND,
                false,
                false,
                6 + data.len(),
                |o| {
                    o[..6].copy_from_slice(adv_addr);
                    o[6..].copy_from_slice(data);
                },
            ),
            AdvPdu::ScanRsp { adv_addr, data } => {
                Self::encode_with(out, PDU_SCAN_RSP, tx_add, false, 6 + data.len(), |o| {
                    o[..6].copy_from_slice(adv_addr);
                    o[6..].copy_from_slice(data);
                })
            }
            AdvPdu::AdvDirectInd {
                adv_addr,
                init_addr,
            } => Self::encode_with(out, PDU_ADV_DIRECT_IND, tx_add, false, 12, |o| {
                o[..6].copy_from_slice(adv_addr);
                o[6..12].copy_from_slice(init_addr);
            }),
            AdvPdu::ScanReq {
                scan_addr,
                adv_addr,
            } => Self::encode_with(out, PDU_SCAN_REQ, tx_add, rx_add, 12, |o| {
                o[..6].copy_from_slice(scan_addr);
                o[6..12].copy_from_slice(adv_addr);
            }),
            AdvPdu::ConnectReq {
                init_addr,
                adv_addr,
                ll_data,
            } => Self::encode_with(out, PDU_CONNECT_REQ, tx_add, rx_add, 34, |o| {
                o[..6].copy_from_slice(init_addr);
                o[6..12].copy_from_slice(adv_addr);
                o[12..34].copy_from_slice(ll_data);
            }),
        }
    }

    /// The PDU type of this packet.
    pub fn pdu_type(&self) -> u8 {
        match *self {
            AdvPdu::AdvInd { .. } => PDU_ADV_IND,
            AdvPdu::AdvDirectInd { .. } => PDU_ADV_DIRECT_IND,
            AdvPdu::AdvNonconnInd { .. } => PDU_ADV_NONCONN_IND,
            AdvPdu::ScanReq { .. } => PDU_SCAN_REQ,
            AdvPdu::ScanRsp { .. } => PDU_SCAN_RSP,
            AdvPdu::ConnectReq { .. } => PDU_CONNECT_REQ,
            AdvPdu::AdvScanInd { .. } => PDU_ADV_SCAN_IND,
        }
    }

    fn encode_with(
        out: &mut [u8],
        pdu_type: u8,
        tx_add: bool,
        rx_add: bool,
        payload_len: usize,
        write_payload: impl FnOnce(&mut [u8]),
    ) -> Result<usize, Error> {
        if payload_len > 0x3F {
            return Err(Error::InvalidLength);
        }
        let total = 2 + payload_len;
        if out.len() < total {
            return Err(Error::BufferTooSmall);
        }
        let header = AdvHdr {
            pdu_type,
            ch_sel: false,
            tx_add,
            rx_add,
            len: payload_len as u8,
        };
        out[..2].copy_from_slice(&header.encode());
        write_payload(&mut out[2..total]);
        Ok(total)
    }
}

fn split6(payload: &[u8]) -> Result<(&[u8; 6], &[u8]), Error> {
    if payload.len() < 6 {
        return Err(Error::InvalidLength);
    }
    Ok((
        payload[..6].try_into().map_err(|_| Error::InvalidLength)?,
        &payload[6..],
    ))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A decoded data physical channel PDU.
pub struct DataPdu<'a> {
    /// LLID field.
    pub llid: u8,
    /// Next Expected Sequence Number.
    pub nesn: bool,
    /// Sequence Number.
    pub sn: bool,
    /// More Data.
    pub md: bool,
    /// Payload.
    pub payload: &'a [u8],
}

impl<'a> DataPdu<'a> {
    /// Decode a full data channel PDU (header + payload).
    pub fn decode(bytes: &'a [u8]) -> Result<DataPdu<'a>, Error> {
        if bytes.len() < 2 {
            return Err(Error::InvalidPdu);
        }
        let llid = bytes[0] & 0b11;
        let nesn = bytes[0] & 0b100 != 0;
        let sn = bytes[0] & 0b1000 != 0;
        let md = bytes[0] & 0b10000 != 0;
        let len = (bytes[1] & 0b11_1111) as usize;
        let payload = &bytes[2..];
        if payload.len() != len {
            return Err(Error::InvalidLength);
        }
        Ok(DataPdu {
            llid,
            nesn,
            sn,
            md,
            payload,
        })
    }

    /// Encode the PDU into `out`, returning the number of bytes written.
    ///
    /// The TxAdd/RxAdd header bits are cleared; use [`AdvPdu::encode_typed`]
    /// when the address types are known.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        self.encode_typed(out, false, false)
    }

    /// Encode the PDU with explicit TxAdd/RxAdd address-type header bits.
    pub fn encode_typed(&self, out: &mut [u8], tx_add: bool, rx_add: bool) -> Result<usize, Error> {
        if self.payload.len() > 0x3F {
            return Err(Error::InvalidLength);
        }
        let total = 2 + self.payload.len();
        if out.len() < total {
            return Err(Error::BufferTooSmall);
        }
        out[0] = (self.llid & 0b11)
            | ((self.nesn as u8) << 2)
            | ((self.sn as u8) << 3)
            | ((self.md as u8) << 4);
        out[1] = (self.payload.len() as u8) & 0b11_1111;
        out[2..total].copy_from_slice(self.payload);
        Ok(total)
    }
}

/// Length of the CONNECT_REQ LLData field.
pub const CONNECT_LL_DATA_LEN: usize = 22;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Parsed CONNECT_REQ LLData (connection parameters).
pub struct ConnectReqData {
    /// Connection access address.
    pub access_addr: u32,
    /// Connection CRC initial value.
    pub crc_init: u32,
    /// Transmit window size in 1.25 ms units.
    pub win_size: u8,
    /// Transmit window offset in 1.25 ms units.
    pub win_offset: u16,
    /// Connection interval in 1.25 ms units.
    pub interval: u16,
    /// Slave latency in connection events.
    pub latency: u16,
    /// Supervision timeout in 10 ms units.
    pub timeout: u16,
    /// Data channel map (5 bytes, 37 channels).
    pub channel_map: [u8; 5],
    /// Hop increment (5 bits).
    pub hop: u8,
    /// Sleep clock accuracy (3 bits).
    pub sca: u8,
}

impl ConnectReqData {
    /// Decode a 22-byte LLData field.
    pub fn decode(ll_data: &[u8]) -> Result<ConnectReqData, Error> {
        if ll_data.len() != CONNECT_LL_DATA_LEN {
            return Err(Error::InvalidLength);
        }
        let access_addr = u32::from_le_bytes([ll_data[0], ll_data[1], ll_data[2], ll_data[3]]);
        let crc_init =
            u32::from(ll_data[4]) | (u32::from(ll_data[5]) << 8) | (u32::from(ll_data[6]) << 16);
        let win_size = ll_data[7];
        let win_offset = u16::from_le_bytes([ll_data[8], ll_data[9]]);
        let interval = u16::from_le_bytes([ll_data[10], ll_data[11]]);
        let latency = u16::from_le_bytes([ll_data[12], ll_data[13]]);
        let timeout = u16::from_le_bytes([ll_data[14], ll_data[15]]);
        let mut channel_map = [0u8; 5];
        channel_map.copy_from_slice(&ll_data[16..21]);
        let last = ll_data[21];
        Ok(ConnectReqData {
            access_addr,
            crc_init,
            win_size,
            win_offset,
            interval,
            latency,
            timeout,
            channel_map,
            hop: last & 0x1F,
            sca: (last >> 5) & 0x07,
        })
    }

    /// Encode the PDU into `out`, returning the number of bytes written.
    ///
    /// The TxAdd/RxAdd header bits are cleared; use [`AdvPdu::encode_typed`]
    /// when the address types are known.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, Error> {
        self.encode_typed(out, false, false)
    }

    /// Encode the PDU with explicit TxAdd/RxAdd address-type header bits.
    pub fn encode_typed(&self, out: &mut [u8], tx_add: bool, rx_add: bool) -> Result<usize, Error> {
        if out.len() < CONNECT_LL_DATA_LEN {
            return Err(Error::BufferTooSmall);
        }
        let aa = self.access_addr.to_le_bytes();
        out[0..4].copy_from_slice(&aa);
        out[4] = self.crc_init as u8;
        out[5] = (self.crc_init >> 8) as u8;
        out[6] = (self.crc_init >> 16) as u8;
        out[7] = self.win_size;
        out[8..10].copy_from_slice(&self.win_offset.to_le_bytes());
        out[10..12].copy_from_slice(&self.interval.to_le_bytes());
        out[12..14].copy_from_slice(&self.latency.to_le_bytes());
        out[14..16].copy_from_slice(&self.timeout.to_le_bytes());
        out[16..21].copy_from_slice(&self.channel_map);
        out[21] = (self.hop & 0x1F) | ((self.sca & 0x07) << 5);
        Ok(CONNECT_LL_DATA_LEN)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const A1: [u8; 6] = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
    const A2: [u8; 6] = [0x06, 0x05, 0x04, 0x03, 0x02, 0x01];
    const A3: [u8; 6] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
    const A4: [u8; 6] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66];
    const A5: [u8; 6] = [0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC];

    #[test]
    fn adv_header_bit_layout() {
        let h = AdvHdr {
            pdu_type: PDU_ADV_IND,
            ch_sel: false,
            tx_add: false,
            rx_add: false,
            len: 6,
        };
        assert_eq!(h.encode(), [0x00, 0x0C]);
        let h = AdvHdr {
            pdu_type: PDU_SCAN_REQ,
            ch_sel: false,
            tx_add: true,
            rx_add: false,
            len: 12,
        };
        assert_eq!(h.encode(), [0x83, 0x18]);
        assert_eq!(AdvHdr::decode([0x83, 0x18]).unwrap(), h);
    }

    #[test]
    fn adv_ind_roundtrip() {
        let data = [0x02, 0x01, 0x06, 0x03, 0x19, 0xC1, 0x03];
        let pdu = AdvPdu::AdvInd {
            adv_addr: &A1,
            data: &data,
        };
        let mut buf = [0u8; 64];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(n, 2 + 6 + 7);
        let decoded = AdvPdu::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, pdu);
    }

    #[test]
    fn adv_ind_canonical_bytes() {
        let pdu = AdvPdu::AdvInd {
            adv_addr: &A2,
            data: &[0x00, 0x01, 0x02],
        };
        let mut buf = [0u8; 64];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[0x00, 0x12, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, 0x01, 0x02]
        );
    }

    #[test]
    fn scan_req_roundtrip() {
        let pdu = AdvPdu::ScanReq {
            scan_addr: &A3,
            adv_addr: &A1,
        };
        let mut buf = [0u8; 64];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(n, 14);
        assert_eq!(AdvPdu::decode(&buf[..n]).unwrap(), pdu);
        assert_eq!(pdu.pdu_type(), PDU_SCAN_REQ);
    }

    #[test]
    fn connect_req_roundtrip() {
        let ll = ConnectReqData {
            access_addr: 0x1234_ABCD,
            crc_init: 0x654321,
            win_size: 2,
            win_offset: 1,
            interval: 24,
            latency: 4,
            timeout: 2000,
            channel_map: [0xFF, 0xFF, 0xFF, 0xFF, 0x1F],
            hop: 13,
            sca: 2,
        };
        let mut llbuf = [0u8; 22];
        ll.encode(&mut llbuf).unwrap();
        assert_eq!(ConnectReqData::decode(&llbuf).unwrap(), ll);
        let pdu = AdvPdu::ConnectReq {
            init_addr: &A4,
            adv_addr: &A5,
            ll_data: &llbuf,
        };
        let mut buf = [0u8; 64];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(n, 36);
        match AdvPdu::decode(&buf[..n]).unwrap() {
            AdvPdu::ConnectReq { ll_data, .. } => {
                assert_eq!(ConnectReqData::decode(ll_data).unwrap(), ll)
            }
            _ => panic!("wrong pdu type"),
        }
    }

    #[test]
    fn data_pdu_roundtrip() {
        let pdu = DataPdu {
            llid: LLID_CONTROL,
            nesn: true,
            sn: false,
            md: true,
            payload: &[0x01, 0x02, 0x03],
        };
        let mut buf = [0u8; 64];
        let n = pdu.encode(&mut buf).unwrap();
        assert_eq!(&buf[..n], &[0b0001_0110, 0b0000_0011, 0x01, 0x02, 0x03]);
        assert_eq!(DataPdu::decode(&buf[..n]).unwrap(), pdu);
    }

    #[test]
    fn truncated_pdu_rejected() {
        assert!(AdvPdu::decode(&[0x00]).is_err());
        assert!(AdvPdu::decode(&[0x00, 0x0C, 0x01]).is_err());
        assert!(DataPdu::decode(&[0x00]).is_err());
    }

    #[test]
    fn length_mismatch_rejected() {
        assert!(AdvPdu::decode(&[0x00, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05]).is_err());
    }

    #[test]
    fn reserved_pdu_type_rejected() {
        assert!(AdvPdu::decode(&[0x08, 0x00]).is_err());
    }
}
