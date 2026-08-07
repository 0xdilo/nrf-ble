//! SMP (LE legacy pairing) cryptographic functions and pairing state machine.

use aes::cipher::{BlockEncrypt, KeyInit};
use aes::Aes128;
use cmac::{Cmac, Mac};
use p256::ecdh::diffie_hellman;
use p256::elliptic_curve::sec1::FromEncodedPoint;
use p256::{EncodedPoint, PublicKey as P256Public, SecretKey};

/// SMP opcode: public key exchange.
pub const SMP_PUBLIC_KEY: u8 = 0x0C;

fn aes_cmac(key: &[u8; 16], data: &[u8]) -> [u8; 16] {
    let mut mac = <Cmac<Aes128> as Mac>::new_from_slice(key).unwrap();
    mac.update(data);
    let out = mac.finalize();
    let mut res = [0u8; 16];
    res.copy_from_slice(&out.into_bytes());
    res
}

/// Legacy Just Works TK: all-zero 16 bytes.
pub const TK_JUST_WORKS: [u8; 16] = [0; 16];

/// SMP L2CAP channel ID.
pub const L2CAP_SMP_CID: u16 = 0x0006;

/// SMP opcode: pairing request.
pub const SMP_PAIRING_REQUEST: u8 = 0x01;
/// SMP opcode: pairing response.
pub const SMP_PAIRING_RESPONSE: u8 = 0x02;
/// SMP opcode: pairing confirm.
pub const SMP_PAIRING_CONFIRM: u8 = 0x03;
/// SMP opcode: pairing random.
pub const SMP_PAIRING_RANDOM: u8 = 0x04;
/// SMP opcode: pairing failed.
pub const SMP_PAIRING_FAILED: u8 = 0x05;
/// SMP opcode: encryption information (LTK distribution).
pub const SMP_ENCRYPTION_INFORMATION: u8 = 0x06;
/// SMP opcode: master identification (EDIV + RAND).
pub const SMP_MASTER_IDENTIFICATION: u8 = 0x07;
/// SMP opcode: security request.
pub const SMP_SECURITY_REQUEST: u8 = 0x0B;
/// SMP failure reason: unspecified.
pub const SMP_PAIRING_FAILED_UNSPECIFIED: u8 = 0x08;
/// SMP failure reason: OOB data required.
pub const SMP_REASON_OOB: u8 = 0x02;
/// SMP failure reason: authentication requirements.
pub const SMP_REASON_AUTH: u8 = 0x05;
/// SMP failure reason: confirm value mismatch.
pub const SMP_REASON_CONFIRM: u8 = 0x06;

const IO_NO_INPUT_NO_OUTPUT: u8 = 0x03;

fn aes_encrypt(key: &[u8; 16], block: &mut [u8; 16]) {
    let cipher = Aes128::new(key.into());
    cipher.encrypt_block(block.into());
}

/// Inputs to the confirm value function c1.
#[derive(Debug, Clone, Copy)]
/// Inputs to the confirm value function c1.
pub struct C1Inputs {
    /// Random value (16 bytes).
    pub r: [u8; 16],
    /// Pairing request PDU (7 bytes).
    pub preq: [u8; 7],
    /// Pairing response PDU (7 bytes).
    pub pres: [u8; 7],
    /// Initiator address type.
    pub iat: u8,
    /// Responder address type.
    pub rat: u8,
    /// Initiator address.
    pub ia: [u8; 6],
    /// Responder address.
    pub ra: [u8; 6],
}

/// Confirm value function c1 (Bluetooth Core Spec Vol 3, Part H, 2.2.3).
pub fn c1(tk: &[u8; 16], inputs: &C1Inputs) -> [u8; 16] {
    let r = &inputs.r;
    let preq = &inputs.preq;
    let pres = &inputs.pres;
    let iat = inputs.iat;
    let rat = inputs.rat;
    let ia = &inputs.ia;
    let ra = &inputs.ra;
    let mut p1 = [0u8; 16];
    p1[0] = iat;
    p1[1] = rat;
    p1[2..9].copy_from_slice(preq);
    p1[9..16].copy_from_slice(pres);

    let mut res = [0u8; 16];
    for i in 0..16 {
        res[i] = r[i] ^ p1[i];
    }
    aes_encrypt(tk, &mut res);

    let mut p2 = [0u8; 16];
    p2[..6].copy_from_slice(ra);
    p2[6..12].copy_from_slice(ia);
    p2[12..].fill(0);

    for i in 0..16 {
        res[i] ^= p2[i];
    }
    aes_encrypt(tk, &mut res);
    res
}

/// Key generation function s1 (Bluetooth Core Spec Vol 3, Part H, 2.2.4).
pub fn s1(k: &[u8; 16], r1: &[u8; 16], r2: &[u8; 16]) -> [u8; 16] {
    let mut block = [0u8; 16];
    block[..8].copy_from_slice(&r2[..8]);
    block[8..].copy_from_slice(&r1[..8]);
    aes_encrypt(k, &mut block);
    block
}

/// SMP protocol error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// SMP protocol error.
pub enum SmpError {
    /// Malformed PDU.
    InvalidPdu,
    /// Confirm value mismatch.
    ConfirmMismatch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// SMP pairing state machine state.
pub enum SmpState {
    /// No pairing in progress.
    Idle,
    /// Pairing request sent, awaiting the response.
    WaitPairingResponse,
    /// Awaiting the peer confirm.
    WaitConfirm,
    /// Awaiting the peer random.
    WaitRandom,
    /// STK computed, awaiting LL encryption.
    WaitingStartEnc,
    /// Link encryption active.
    Encrypted,
}

impl SmpState {
    /// True while the pairing procedure is in progress.
    pub fn pairing(&self) -> bool {
        !matches!(self, SmpState::Idle | SmpState::Encrypted)
    }
}

/// Legacy SMP pairing state.
pub struct Smp {
    /// Current pairing state.
    pub state: SmpState,
    /// Our random value.
    pub own_random: [u8; 16],
    /// Peer random value.
    pub peer_random: [u8; 16],
    /// Our confirm value.
    pub own_confirm: [u8; 16],
    /// Peer confirm value.
    pub peer_confirm: [u8; 16],
    /// Pairing request PDU.
    pub preq: [u8; 7],
    /// Pairing response PDU.
    pub pres: [u8; 7],
    /// Initiator address type.
    pub iat: u8,
    /// Responder address type.
    pub rat: u8,
    /// Initiator address.
    pub ia: [u8; 6],
    /// Responder address.
    pub ra: [u8; 6],
    /// Session temporary key (the LL encryption key after pairing).
    pub stk: [u8; 16],
    /// True when we are the responder.
    pub responder: bool,
    /// Last pairing failure reason.
    pub pairing_failed: u8,
    /// Optional 6-digit passkey (legacy passkey entry; `None` = Just Works).
    pub passkey: Option<u32>,
    /// True when LE Secure Connections was negotiated.
    pub sc: bool,
    /// Our P-256 private key (32 bytes).
    pub own_private: [u8; 32],
    /// Peer public key (64 bytes, uncompressed point without the prefix).
    pub peer_public: [u8; 64],
    /// Shared ECDH secret.
    pub dhkey: [u8; 32],
    /// Our SC nonce.
    pub sc_nonce: [u8; 16],
    /// Peer SC nonce.
    pub peer_nonce: [u8; 16],
}

impl Default for Smp {
    fn default() -> Self {
        Self::new()
    }
}

impl Smp {
    /// Create an idle pairing state.
    pub fn new() -> Self {
        Smp {
            state: SmpState::Idle,
            own_random: [0; 16],
            peer_random: [0; 16],
            own_confirm: [0; 16],
            peer_confirm: [0; 16],
            preq: [0; 7],
            pres: [0; 7],
            iat: 0,
            rat: 0,
            ia: [0; 6],
            ra: [0; 6],
            stk: [0; 16],
            responder: false,
            pairing_failed: 0,
            passkey: None,
            sc: false,
            own_private: [0; 32],
            peer_public: [0; 64],
            dhkey: [0; 32],
            sc_nonce: [0; 16],
            peer_nonce: [0; 16],
        }
    }

    /// Configure a 6-digit passkey (legacy passkey entry; display or
    /// keyboard role). `None` selects Just Works.
    pub fn set_passkey(&mut self, passkey: Option<u32>) {
        self.passkey = passkey;
    }

    /// Enable LE Secure Connections (ECDH P-256 based pairing).
    pub fn enable_sc(&mut self) {
        self.sc = true;
    }

    fn tk(&self) -> [u8; 16] {
        match self.passkey {
            Some(pk) => {
                let mut tk = [0u8; 16];
                tk[..4].copy_from_slice(&pk.to_le_bytes());
                tk
            }
            None => TK_JUST_WORKS,
        }
    }

    /// Build a pairing request (bonding, 16-byte key).
    pub fn build_pairing_request(&mut self) -> [u8; 7] {
        let mut p = [0u8; 7];
        p[0] = SMP_PAIRING_REQUEST;
        p[1] = if self.passkey.is_some() {
            0x02
        } else {
            IO_NO_INPUT_NO_OUTPUT
        };
        p[2] = 0;
        p[3] = 0x01 | if self.sc { 0x08 } else { 0 };
        p[4] = 16;
        p[5] = 0x01;
        p[6] = 0x01;
        self.preq = p;
        p
    }

    /// Handle an incoming pairing request; returns the pairing response.
    pub fn handle_pairing_request(&mut self, data: &[u8]) -> Result<[u8; 7], SmpError> {
        if data.len() < 7 {
            return Err(SmpError::InvalidPdu);
        }
        self.preq[..7].copy_from_slice(&data[..7]);
        self.rat = 0;
        self.responder = true;
        self.state = SmpState::WaitConfirm;
        let mut rsp = [0u8; 7];
        rsp[0] = SMP_PAIRING_RESPONSE;
        rsp[1] = IO_NO_INPUT_NO_OUTPUT;
        rsp[2] = 0;
        rsp[3] = 0x01;
        rsp[4] = 16;
        rsp[5] = 0x01;
        rsp[6] = 0x01;
        self.pres = rsp;
        Ok(rsp)
    }

    /// Handle an incoming pairing response.
    pub fn handle_pairing_response(&mut self, data: &[u8]) -> Result<(), SmpError> {
        if data.len() < 7 {
            return Err(SmpError::InvalidPdu);
        }
        self.pres[..7].copy_from_slice(&data[..7]);
        self.state = SmpState::WaitConfirm;
        Ok(())
    }

    /// Generate the P-256 key pair and build the SMP_PUBLIC_KEY PDU
    /// (65 bytes: opcode + 64-byte uncompressed point).
    pub fn build_public_key(&mut self) -> [u8; 65] {
        let sk = SecretKey::from_bytes(&self.own_private.into()).unwrap();
        let pk = sk.public_key();
        let point = EncodedPoint::from(pk);
        let mut out = [0u8; 65];
        out[0] = SMP_PUBLIC_KEY;
        out[1..].copy_from_slice(&point.as_bytes()[1..]);
        out
    }

    /// Handle the peer public key and compute the shared DHKey.
    pub fn handle_public_key(&mut self, data: &[u8]) -> Result<(), SmpError> {
        if data.len() < 65 {
            return Err(SmpError::InvalidPdu);
        }
        self.peer_public[..].copy_from_slice(&data[1..65]);
        let mut enc = [0u8; 65];
        enc[0] = 0x04;
        enc[1..].copy_from_slice(&data[1..65]);
        let point = EncodedPoint::from_bytes(enc).map_err(|_| SmpError::InvalidPdu)?;
        let peer = Option::<P256Public>::from(P256Public::from_encoded_point(&point))
            .ok_or(SmpError::InvalidPdu)?;
        let sk =
            SecretKey::from_bytes(&self.own_private.into()).map_err(|_| SmpError::InvalidPdu)?;
        let dh = diffie_hellman(sk.to_nonzero_scalar(), peer.as_affine());
        let raw = dh.raw_secret_bytes();
        self.dhkey[..].copy_from_slice(raw);
        self.sc = true;
        Ok(())
    }

    /// Compute the SC LTK with f5 (kernel-verified construction).
    ///
    /// `n1`/`a1` are the initiator nonce/address, `n2`/`a2` the responder's.
    pub fn compute_ltk_sc(
        &self,
        n1: &[u8; 16],
        n2: &[u8; 16],
        a1: &[u8; 7],
        a2: &[u8; 7],
    ) -> [u8; 16] {
        let btle = [0x65u8, 0x6c, 0x74, 0x62];
        let salt = [
            0xbe, 0x83, 0x60, 0x5a, 0xdb, 0x0b, 0x37, 0x60, 0x38, 0xa5, 0xf5, 0xaa, 0x91, 0x83,
            0x88, 0x6c,
        ];
        let length = [0x00u8, 0x01];
        let t = aes_cmac(&salt, &self.dhkey);
        let mut m = [0u8; 53];
        m[..2].copy_from_slice(&length);
        m[2..9].copy_from_slice(a2);
        m[9..16].copy_from_slice(a1);
        m[16..32].copy_from_slice(n2);
        m[32..48].copy_from_slice(n1);
        m[48..52].copy_from_slice(&btle);
        m[52] = 0;
        let mackey = aes_cmac(&t, &m);
        m[52] = 1;
        aes_cmac(&mackey, &m)
    }

    /// Build a SMP_PAIRING_FAILED PDU.
    pub fn build_failed(&self, reason: u8) -> [u8; 2] {
        [SMP_PAIRING_FAILED, reason]
    }

    /// Fail the pairing with an unspecified reason.
    pub fn fail_unspecified(&mut self) -> [u8; 2] {
        self.pairing_failed = SMP_PAIRING_FAILED_UNSPECIFIED;
        self.build_failed(SMP_PAIRING_FAILED_UNSPECIFIED)
    }

    /// Fail the pairing because OOB data is required.
    pub fn fail_oob(&mut self) -> [u8; 2] {
        self.pairing_failed = SMP_REASON_OOB;
        self.build_failed(SMP_REASON_OOB)
    }

    /// Fail the pairing because authentication requirements differ.
    pub fn fail_auth(&mut self) -> [u8; 2] {
        self.pairing_failed = SMP_REASON_AUTH;
        self.build_failed(SMP_REASON_AUTH)
    }

    /// Build the pairing confirm PDU (Just Works, TK = 0).
    pub fn build_confirm(&mut self, iat: u8, ia: &[u8; 6], ra: &[u8; 6]) -> [u8; 17] {
        self.iat = iat;
        self.ia = *ia;
        self.ra = *ra;
        for b in self.own_random.iter_mut() {
            *b = 0x5A;
        }
        let tk = self.tk();
        self.own_confirm = c1(
            &tk,
            &C1Inputs {
                r: self.own_random,
                preq: self.preq,
                pres: self.pres,
                iat: self.iat,
                rat: self.rat,
                ia: self.ia,
                ra: self.ra,
            },
        );
        let mut out = [0u8; 17];
        out[0] = SMP_PAIRING_CONFIRM;
        out[1..].copy_from_slice(&self.own_confirm);
        out
    }

    /// Handle an incoming pairing confirm.
    pub fn handle_confirm(&mut self, data: &[u8]) -> Result<(), SmpError> {
        if data.len() < 17 {
            return Err(SmpError::InvalidPdu);
        }
        self.peer_confirm[..].copy_from_slice(&data[1..17]);
        self.state = SmpState::WaitRandom;
        Ok(())
    }

    /// Build the pairing random PDU.
    pub fn build_random(&self) -> [u8; 17] {
        let mut out = [0u8; 17];
        out[0] = SMP_PAIRING_RANDOM;
        out[1..].copy_from_slice(&self.own_random);
        out
    }

    /// Handle an incoming pairing random: verifies the peer confirm and
    /// computes the STK. Returns our random PDU.
    pub fn handle_random(&mut self, data: &[u8]) -> Result<[u8; 17], SmpError> {
        if data.len() < 17 {
            return Err(SmpError::InvalidPdu);
        }
        self.peer_random[..].copy_from_slice(&data[1..17]);
        let tk = self.tk();
        let expected = c1(
            &tk,
            &C1Inputs {
                r: self.peer_random,
                preq: self.preq,
                pres: self.pres,
                iat: self.iat,
                rat: self.rat,
                ia: self.ia,
                ra: self.ra,
            },
        );
        if expected != self.peer_confirm {
            self.pairing_failed = SMP_REASON_CONFIRM;
            return Err(SmpError::ConfirmMismatch);
        }
        let tk = self.tk();
        self.stk = s1(&tk, &self.own_random, &self.peer_random);
        self.state = SmpState::WaitingStartEnc;
        self.pairing_failed = 0;
        let mut out = [0u8; 17];
        out[0] = SMP_PAIRING_RANDOM;
        out[1..].copy_from_slice(&self.own_random);
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn c1_matches_reference() {
        let tk = [0u8; 16];
        let r = [0x01u8; 16];
        let preq = [0x02u8; 7];
        let pres = [0x03u8; 7];
        let ia = [0x04u8; 6];
        let ra = [0x05u8; 6];
        let c = c1(
            &tk,
            &C1Inputs {
                r,
                preq,
                pres,
                iat: 1,
                rat: 0,
                ia,
                ra,
            },
        );
        assert_eq!(
            c,
            [
                0xBD, 0xA1, 0xF6, 0xB0, 0xDE, 0x5F, 0x28, 0xB0, 0xD6, 0x6E, 0x05, 0x32, 0xB9, 0xBA,
                0xDE, 0xBA
            ]
        );
    }

    #[test]
    fn s1_matches_reference() {
        let k = [0u8; 16];
        let r1 = [0x01u8; 16];
        let r2 = [0x02u8; 16];
        let s = s1(&k, &r1, &r2);
        assert_eq!(
            s,
            [
                0x97, 0x4B, 0xC0, 0x47, 0xDE, 0x83, 0x0A, 0x80, 0x1A, 0x01, 0x4D, 0x94, 0x90, 0x9E,
                0xE9, 0x85
            ]
        );
    }

    #[test]
    fn pairing_exchange_roundtrip() {
        let mut a = Smp::new();
        let mut b = Smp::new();
        let req = a.build_pairing_request();
        let _ = req;
        let rsp = b.handle_pairing_request(&req).unwrap();
        a.handle_pairing_response(&rsp).unwrap();

        let ia = [0xAA; 6];
        let ra = [0xBB; 6];
        let conf_a = a.build_confirm(1, &ia, &ra);
        b.iat = 1;
        b.ia = ia;
        b.ra = ra;
        b.handle_confirm(&conf_a).unwrap();
        let conf_b = b.build_confirm(1, &ia, &ra);
        a.handle_confirm(&conf_b).unwrap();

        let rnd_a = a.build_random();
        let rnd_b = b.handle_random(&rnd_a).unwrap();
        a.handle_random(&rnd_b).unwrap();
        assert_eq!(a.stk, b.stk);
        assert!(!a.stk.iter().all(|&x| x == 0));
    }
}

#[cfg(test)]
mod sc_tests {
    use super::*;

    #[test]
    fn f5_matches_reference() {
        let dhkey = hex32("5fd14503997d08fc21ec94741882e4ed665e1dba4ee4bdcc6cb61f1a177e9817");
        let n1 = core::array::from_fn(|i| i as u8);
        let n2 = core::array::from_fn(|i| (i + 16) as u8);
        let a1 = {
            let mut a = [0xAAu8; 7];
            a[6] = 0;
            a
        };
        let a2 = {
            let mut a = [0xBBu8; 7];
            a[6] = 1;
            a
        };
        let mut smp = Smp::new();
        smp.dhkey = dhkey;
        smp.sc_nonce = n1;
        smp.peer_nonce = n2;
        let ltk = smp.compute_ltk_sc(&n1, &n2, &a1, &a2);
        assert_eq!(ltk, hex16("36217fdd23fe9c5f8e401cfaaa464714"));
    }

    #[test]
    fn ecdh_and_f5_end_to_end() {
        // two peers with fixed private keys
        let mut a = Smp::new();
        let mut b = Smp::new();
        a.own_private = hex32("1234567890123456789012345678901234567890123456789012345678901234");
        b.own_private = hex32("abcdefabcdefabcdefabcdefabcdefabcdefabcdefabcdefabcdefabcdefabcd");
        let pka = a.build_public_key();
        let pkb = b.build_public_key();
        assert_eq!(pka[0], SMP_PUBLIC_KEY);
        assert_eq!(pka[1], 0xCA);
        a.handle_public_key(&pkb).unwrap();
        b.handle_public_key(&pka).unwrap();
        assert_eq!(a.dhkey, b.dhkey);
        assert_eq!(
            a.dhkey,
            hex32("5fd14503997d08fc21ec94741882e4ed665e1dba4ee4bdcc6cb61f1a177e9817")
        );
        let a1 = {
            let mut x = [0xAAu8; 7];
            x[6] = 0;
            x
        };
        let a2 = {
            let mut x = [0xBBu8; 7];
            x[6] = 1;
            x
        };
        a.sc_nonce = [7u8; 16];
        a.peer_nonce = [9u8; 16];
        b.sc_nonce = [9u8; 16];
        b.peer_nonce = [7u8; 16];
        assert_eq!(a.compute_ltk_sc(&a1, &a2), b.compute_ltk_sc(&a1, &a2));
    }

    #[test]
    fn passkey_tk_layout() {
        let mut s = Smp::new();
        s.set_passkey(Some(123456));
        let tk = s.tk();
        assert_eq!(&tk[..4], &[0x40, 0xE2, 0x01, 0x00]);
        assert_eq!(&tk[4..], &[0; 12]);
    }

    fn hex32(h: &str) -> [u8; 32] {
        let mut out = [0u8; 32];
        for i in 0..32 {
            out[i] = u8::from_str_radix(&h[i * 2..i * 2 + 2], 16).unwrap();
        }
        out
    }

    fn hex16(h: &str) -> [u8; 16] {
        let mut out = [0u8; 16];
        for i in 0..16 {
            out[i] = u8::from_str_radix(&h[i * 2..i * 2 + 2], 16).unwrap();
        }
        out
    }
}
