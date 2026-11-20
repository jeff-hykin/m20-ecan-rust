//! The Ethernet header around ECAN frames on the AOS board's eth1.

use crate::frame::FrameError;

/// The ECAN ethertype (bytes 0x44 0x52 on the wire).
pub const ETHERTYPE: u16 = 0x4452;
/// Ethernet header length.
pub const ETHERNET_HEADER_LEN: usize = 14;
/// rl_deploy sends every frame to this hardcoded destination (libecanmaster `SocketRaw::Send`).
pub const BOARD_MAC: [u8; 6] = [0x22, 0x33, 0x44, 0x55, 0x66, 0x77];

/// Destination, source, ethertype.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EthernetHeader {
    pub destination: [u8; 6],
    pub source: [u8; 6],
    pub ethertype: u16,
}

impl EthernetHeader {
    /// Split an Ethernet frame into its header and body; only ECAN frames are accepted.
    pub fn parse(frame: &[u8]) -> Result<(Self, &[u8]), FrameError> {
        if frame.len() < ETHERNET_HEADER_LEN {
            return Err(FrameError::Truncated { needed: ETHERNET_HEADER_LEN, got: frame.len() });
        }
        let header = EthernetHeader {
            destination: frame[0..6].try_into().unwrap(),
            source: frame[6..12].try_into().unwrap(),
            ethertype: u16::from_be_bytes([frame[12], frame[13]]),
        };
        if header.ethertype != ETHERTYPE {
            return Err(FrameError::NotEcan { ethertype: header.ethertype });
        }
        Ok((header, &frame[ETHERNET_HEADER_LEN..]))
    }

    pub fn write(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.destination);
        out.extend_from_slice(&self.source);
        out.extend_from_slice(&self.ethertype.to_be_bytes());
    }
}

/// The Ethernet frame rl_deploy puts on the wire for an ECAN body: to [`BOARD_MAC`] from `source`.
pub fn wrap(source: [u8; 6], body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(ETHERNET_HEADER_LEN + body.len());
    EthernetHeader { destination: BOARD_MAC, source, ethertype: ETHERTYPE }.write(&mut out);
    out.extend_from_slice(body);
    out
}

/// `aa:bb:cc:dd:ee:ff`.
pub fn format_mac(mac: &[u8; 6]) -> String {
    mac.iter().map(|byte| format!("{byte:02x}")).collect::<Vec<_>>().join(":")
}
