//! SMP (LE legacy pairing) cryptographic functions and pairing state machine.

use aes::cipher::{BlockEncrypt, KeyInit};
use aes::Aes128;

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
        }
    }

    /// Build a legacy pairing request (Just Works, bonding, 16-byte key).
    pub fn build_pairing_request(&mut self) -> [u8; 7] {
        let mut p = [0u8; 7];
        p[0] = SMP_PAIRING_REQUEST;
        p[1] = IO_NO_INPUT_NO_OUTPUT;
        p[2] = 0;
        p[3] = 0x01;
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
        self.own_confirm = c1(
            &TK_JUST_WORKS,
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
        let expected = c1(
            &TK_JUST_WORKS,
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
        self.stk = s1(&TK_JUST_WORKS, &self.own_random, &self.peer_random);
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
