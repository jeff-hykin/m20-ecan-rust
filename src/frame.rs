//! The ECAN frame: a 10-byte little-endian header and a payload, carried after a 14-byte Ethernet header.
//!
//! ```text
//! 0  u16 magic 0x5244 ("DR")
//! 2  u8  sequence (the sender's counter)
//! 3  u8  flags (1 on joint commands, 6 on board replies, 0 on control requests)
//! 4  u8  code (0x12 joint command, 0x11 control request, 0x21 board reply)
//! 5  u8  frame type (see [`FrameType`])
//! 6  u16 payload length
//! 8  u16 checksum: the payload's bytes summed, mod 65536
//! 10 payload
//! ```

use std::fmt;

/// The header's first two bytes, little-endian ("DR").
pub const MAGIC: u16 = 0x5244;
/// Header length in bytes.
pub const HEADER_LEN: usize = 10;
/// `code` of the 1 kHz joint command rl_deploy sends.
pub const CODE_JOINT_COMMAND: u8 = 0x12;
/// `code` of one-shot control requests (charge enable, version request).
pub const CODE_CONTROL: u8 = 0x11;
/// `code` of every frame the ECAN board sends.
pub const CODE_BOARD_REPLY: u8 = 0x21;
/// `flags` of the 1 kHz joint command.
pub const FLAGS_JOINT_COMMAND: u8 = 1;
/// `flags` of board replies.
pub const FLAGS_BOARD_REPLY: u8 = 6;

/// The frame types rl_deploy's ECAN master sends or parses (libecanmaster `RunRoutine`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FrameType {
    /// 0x81, parsed by the master; not seen on this robot.
    Type81,
    /// 0x84, parsed by the master; not seen on this robot.
    Type84,
    /// 0x86, parsed by the master; not seen on this robot.
    Type86,
    /// 0x87, board -> host, 1 Hz: two 258-byte battery pack records.
    Battery,
    /// 0x89, both ways at 1 kHz: joint commands to the board, IMU + joint state back.
    Joints,
    /// 0x8a, board -> host: 256-byte ECAN status (hard stop, IMU status).
    EcanStatus,
    /// 0xa3, parsed by the master; not seen on this robot.
    TypeA3,
    /// 0xb1, host -> board version request, board -> host version strings.
    Version,
    /// 0xb3, host -> board: battery charge enable (empty payload).
    ChargeEnable,
    /// Anything else.
    Other(u8),
}

impl FrameType {
    pub fn from_byte(byte: u8) -> Self {
        match byte {
            0x81 => Self::Type81,
            0x84 => Self::Type84,
            0x86 => Self::Type86,
            0x87 => Self::Battery,
            0x89 => Self::Joints,
            0x8a => Self::EcanStatus,
            0xa3 => Self::TypeA3,
            0xb1 => Self::Version,
            0xb3 => Self::ChargeEnable,
            other => Self::Other(other),
        }
    }

    pub fn to_byte(self) -> u8 {
        match self {
            Self::Type81 => 0x81,
            Self::Type84 => 0x84,
            Self::Type86 => 0x86,
            Self::Battery => 0x87,
            Self::Joints => 0x89,
            Self::EcanStatus => 0x8a,
            Self::TypeA3 => 0xa3,
            Self::Version => 0xb1,
            Self::ChargeEnable => 0xb3,
            Self::Other(byte) => byte,
        }
    }
}

/// Why bytes are not an ECAN frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Fewer bytes than the header or than the header's payload length.
    Truncated { needed: usize, got: usize },
    /// The Ethernet type is not 0x4452.
    NotEcan { ethertype: u16 },
    /// A payload too long for the u16 length field.
    PayloadTooLong(usize),
    /// A payload whose length does not fit the frame type.
    BadLength { frame_type: FrameType, len: usize },
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated { needed, got } => write!(f, "truncated ECAN frame: need {needed} bytes, got {got}"),
            Self::NotEcan { ethertype } => write!(f, "ethertype {ethertype:#06x} is not ECAN (0x4452)"),
            Self::PayloadTooLong(len) => write!(f, "payload of {len} bytes does not fit a u16 length"),
            Self::BadLength { frame_type, len } => write!(f, "{frame_type:?} frame with a {len}-byte payload"),
        }
    }
}

impl std::error::Error for FrameError {}

/// The 10-byte header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// 0x5244 on every frame except the charge-enable request, which rl_deploy sends with 0.
    pub magic: u16,
    pub seq: u8,
    pub flags: u8,
    pub code: u8,
    pub frame_type: FrameType,
    pub payload_len: u16,
    pub checksum: u16,
}

/// A parsed frame, borrowing its payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    pub header: Header,
    pub payload: &'a [u8],
}

/// The checksum the board and rl_deploy use (`Motor::MotorImpl::CalCRC`): every payload byte summed, mod 65536.
pub fn checksum(payload: &[u8]) -> u16 {
    payload.iter().fold(0u16, |sum, &byte| sum.wrapping_add(byte as u16))
}

impl<'a> Frame<'a> {
    /// Parse the bytes after the Ethernet header; bytes past the payload (Ethernet padding) are ignored.
    pub fn parse(body: &'a [u8]) -> Result<Self, FrameError> {
        if body.len() < HEADER_LEN {
            return Err(FrameError::Truncated { needed: HEADER_LEN, got: body.len() });
        }
        let payload_len = u16::from_le_bytes([body[6], body[7]]);
        let end = HEADER_LEN + payload_len as usize;
        if body.len() < end {
            return Err(FrameError::Truncated { needed: end, got: body.len() });
        }
        Ok(Frame {
            header: Header {
                magic: u16::from_le_bytes([body[0], body[1]]),
                seq: body[2],
                flags: body[3],
                code: body[4],
                frame_type: FrameType::from_byte(body[5]),
                payload_len,
                checksum: u16::from_le_bytes([body[8], body[9]]),
            },
            payload: &body[HEADER_LEN..end],
        })
    }

    /// Parse a whole Ethernet frame (destination, source, ethertype, then the ECAN frame).
    pub fn parse_ethernet(ethernet: &'a [u8]) -> Result<Self, FrameError> {
        let (_, body) = crate::ethernet::EthernetHeader::parse(ethernet)?;
        Frame::parse(body)
    }

    /// Whether the header's checksum matches the payload.
    pub fn checksum_ok(&self) -> bool {
        checksum(self.payload) == self.header.checksum
    }
}

/// Build a frame body (header + payload) the way rl_deploy does, filling in the length and checksum.
pub fn encode(seq: u8, flags: u8, code: u8, frame_type: FrameType, payload: &[u8]) -> Result<Vec<u8>, FrameError> {
    encode_with_magic(MAGIC, seq, flags, code, frame_type, payload)
}

/// [`encode`] with an explicit magic (rl_deploy's charge-enable request carries 0).
pub fn encode_with_magic(
    magic: u16,
    seq: u8,
    flags: u8,
    code: u8,
    frame_type: FrameType,
    payload: &[u8],
) -> Result<Vec<u8>, FrameError> {
    let len = u16::try_from(payload.len()).map_err(|_| FrameError::PayloadTooLong(payload.len()))?;
    let mut body = Vec::with_capacity(HEADER_LEN + payload.len());
    body.extend_from_slice(&magic.to_le_bytes());
    body.push(seq);
    body.push(flags);
    body.push(code);
    body.push(frame_type.to_byte());
    body.extend_from_slice(&len.to_le_bytes());
    body.extend_from_slice(&checksum(payload).to_le_bytes());
    body.extend_from_slice(payload);
    Ok(body)
}

/// [`encode_with_magic`] for payloads this crate builds itself: one longer than a u16 length can say is cut to fit.
pub(crate) fn encode_fitted(
    magic: u16,
    seq: u8,
    flags: u8,
    code: u8,
    frame_type: FrameType,
    payload: &[u8],
) -> Vec<u8> {
    let payload = payload.get(..usize::from(u16::MAX)).unwrap_or(payload);
    encode_with_magic(magic, seq, flags, code, frame_type, payload).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let body = encode(7, FLAGS_JOINT_COMMAND, CODE_JOINT_COMMAND, FrameType::Joints, &[1, 2, 250]).unwrap();
        let frame = Frame::parse(&body).unwrap();
        assert_eq!(frame.header.seq, 7);
        assert_eq!(frame.header.magic, MAGIC);
        assert_eq!(frame.header.frame_type, FrameType::Joints);
        assert_eq!(frame.payload, &[1, 2, 250]);
        assert_eq!(frame.header.checksum, 253);
        assert!(frame.checksum_ok());
    }

    #[test]
    fn checksum_wraps() {
        assert_eq!(checksum(&vec![0xff; 300]), (300u32 * 255 % 65536) as u16);
    }

    #[test]
    fn truncated() {
        let body = encode(0, 0, 0, FrameType::Battery, &[0; 20]).unwrap();
        assert!(matches!(Frame::parse(&body[..15]), Err(FrameError::Truncated { needed: 30, got: 15 })));
        assert!(matches!(Frame::parse(&body[..5]), Err(FrameError::Truncated { .. })));
    }

    #[test]
    fn fitted_cuts_an_oversized_payload() {
        let body = encode_fitted(MAGIC, 0, 0, 0, FrameType::Joints, &vec![1; 70_000]);
        assert_eq!(Frame::parse(&body).unwrap().payload.len(), 65535);
    }

    #[test]
    fn frame_type_bytes() {
        for byte in 0..=255u8 {
            assert_eq!(FrameType::from_byte(byte).to_byte(), byte);
        }
    }
}
