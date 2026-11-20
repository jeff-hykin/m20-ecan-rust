//! A small pcapng reader/writer for ECAN captures: what the `ecan_tap.so` shim records inside rl_deploy (one
//! Ethernet interface, nanosecond timestamps, the `epb_flags` direction on every packet), and what Wireshark writes.

use std::fmt;
use std::io::{self, Write};
use std::path::Path;

use crate::message::Direction;

const SECTION_HEADER: u32 = 0x0A0D_0D0A;
const INTERFACE_DESCRIPTION: u32 = 1;
const ENHANCED_PACKET: u32 = 6;
const BYTE_ORDER_MAGIC: u32 = 0x1A2B_3C4D;
const OPTION_END: u16 = 0;
const OPTION_IF_TSRESOL: u16 = 9;
const OPTION_EPB_FLAGS: u16 = 2;
/// pcapng `LINKTYPE_ETHERNET`.
pub const LINKTYPE_ETHERNET: u16 = 1;

#[derive(Debug)]
pub enum PcapngError {
    Io(io::Error),
    /// Not a pcapng file, or big-endian (the shim and every host here write little-endian).
    NotPcapng,
    Malformed {
        offset: usize,
        what: &'static str,
    },
}

impl fmt::Display for PcapngError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(f, "{error}"),
            Self::NotPcapng => write!(f, "not a little-endian pcapng file"),
            Self::Malformed { offset, what } => write!(f, "malformed pcapng at byte {offset}: {what}"),
        }
    }
}

impl std::error::Error for PcapngError {}

impl From<io::Error> for PcapngError {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}

/// One captured packet.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Nanoseconds since the Unix epoch (the shim stamps CLOCK_REALTIME).
    pub timestamp_ns: u64,
    pub interface: u32,
    /// From `epb_flags`: inbound = received by the host = [`Direction::FromBoard`].
    pub direction: Option<Direction>,
    /// The whole Ethernet frame as captured.
    pub data: &'a [u8],
}

/// Iterates the enhanced packet blocks of a pcapng held in memory.
pub struct Reader<'a> {
    bytes: &'a [u8],
    offset: usize,
    /// Timestamp units per second, per interface.
    resolutions: Vec<u64>,
    failed: bool,
}

fn u16_at(bytes: &[u8], at: usize) -> Option<u16> {
    Some(u16::from_le_bytes(bytes.get(at..at + 2)?.try_into().ok()?))
}

fn u32_at(bytes: &[u8], at: usize) -> Option<u32> {
    Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?))
}

/// (code, value) pairs of an options area.
fn options(area: &[u8]) -> impl Iterator<Item = (u16, &[u8])> {
    let mut at = 0;
    std::iter::from_fn(move || {
        let code = u16_at(area, at)?;
        let len = u16_at(area, at + 2)? as usize;
        if code == OPTION_END {
            return None;
        }
        let value = area.get(at + 4..at + 4 + len)?;
        at += 4 + ((len + 3) & !3);
        Some((code, value))
    })
}

fn resolution(tsresol: u8) -> u64 {
    if tsresol & 0x80 != 0 {
        1u64 << (tsresol & 0x7f).min(63)
    } else {
        10u64.pow((tsresol as u32).min(19))
    }
}

impl<'a> Reader<'a> {
    pub fn new(bytes: &'a [u8]) -> Result<Self, PcapngError> {
        if u32_at(bytes, 0) != Some(SECTION_HEADER) || u32_at(bytes, 8) != Some(BYTE_ORDER_MAGIC) {
            return Err(PcapngError::NotPcapng);
        }
        Ok(Reader { bytes, offset: 0, resolutions: Vec::new(), failed: false })
    }

    fn malformed(&mut self, what: &'static str) -> Option<Result<Packet<'a>, PcapngError>> {
        self.failed = true;
        Some(Err(PcapngError::Malformed { offset: self.offset, what }))
    }
}

impl<'a> Iterator for Reader<'a> {
    type Item = Result<Packet<'a>, PcapngError>;

    fn next(&mut self) -> Option<Self::Item> {
        while !self.failed && self.offset + 12 <= self.bytes.len() {
            let start = self.offset;
            let block_type = u32_at(self.bytes, start)?;
            let block_len = u32_at(self.bytes, start + 4)? as usize;
            if block_len < 12 || !block_len.is_multiple_of(4) || start + block_len > self.bytes.len() {
                return self.malformed("bad block length");
            }
            let block = &self.bytes[start..start + block_len];
            self.offset += block_len;
            match block_type {
                SECTION_HEADER => self.resolutions.clear(),
                INTERFACE_DESCRIPTION => {
                    let area = block.get(16..block_len - 4).unwrap_or(&[]);
                    let tsresol = options(area).find(|(code, _)| *code == OPTION_IF_TSRESOL).map(|(_, v)| v[0]);
                    self.resolutions.push(tsresol.map_or(1_000_000, resolution));
                }
                ENHANCED_PACKET => {
                    if block_len < 32 {
                        return self.malformed("short enhanced packet block");
                    }
                    let interface = u32_at(block, 8)?;
                    let ticks = ((u32_at(block, 12)? as u64) << 32) | u32_at(block, 16)? as u64;
                    let captured = u32_at(block, 20)? as usize;
                    let Some(data) = block.get(28..28 + captured) else {
                        return self.malformed("packet longer than its block");
                    };
                    let area = block.get(28 + ((captured + 3) & !3)..block_len - 4).unwrap_or(&[]);
                    let direction = options(area)
                        .find(|(code, value)| *code == OPTION_EPB_FLAGS && value.len() >= 4)
                        .and_then(|(_, value)| match value[0] & 3 {
                            1 => Some(Direction::FromBoard),
                            2 => Some(Direction::ToBoard),
                            _ => None,
                        });
                    let per_second = self.resolutions.get(interface as usize).copied().unwrap_or(1_000_000);
                    let timestamp_ns = (ticks as u128 * 1_000_000_000 / per_second as u128) as u64;
                    return Some(Ok(Packet { timestamp_ns, interface, direction, data }));
                }
                _ => {}
            }
        }
        None
    }
}

/// Read a whole capture file into memory.
pub fn read_file(path: impl AsRef<Path>) -> Result<Vec<u8>, PcapngError> {
    Ok(std::fs::read(path)?)
}

/// Writes the same layout the shim does: one Ethernet interface with nanosecond timestamps, direction on every packet.
pub struct Writer<W: Write> {
    out: W,
}

impl<W: Write> Writer<W> {
    pub fn new(mut out: W, interface_name: &str) -> io::Result<Self> {
        let mut shb = Vec::new();
        shb.extend_from_slice(&SECTION_HEADER.to_le_bytes());
        shb.extend_from_slice(&28u32.to_le_bytes());
        shb.extend_from_slice(&BYTE_ORDER_MAGIC.to_le_bytes());
        shb.extend_from_slice(&1u16.to_le_bytes());
        shb.extend_from_slice(&0u16.to_le_bytes());
        shb.extend_from_slice(&u64::MAX.to_le_bytes());
        shb.extend_from_slice(&28u32.to_le_bytes());
        out.write_all(&shb)?;
        let name = interface_name.as_bytes();
        let mut area = Vec::new();
        area.extend_from_slice(&OPTION_IF_TSRESOL.to_le_bytes());
        area.extend_from_slice(&1u16.to_le_bytes());
        area.extend_from_slice(&[9, 0, 0, 0]);
        area.extend_from_slice(&2u16.to_le_bytes());
        area.extend_from_slice(&(name.len() as u16).to_le_bytes());
        area.extend_from_slice(name);
        area.resize((area.len() + 3) & !3, 0);
        area.extend_from_slice(&[0; 4]);
        let len = 20 + area.len() as u32;
        let mut idb = Vec::new();
        idb.extend_from_slice(&INTERFACE_DESCRIPTION.to_le_bytes());
        idb.extend_from_slice(&len.to_le_bytes());
        idb.extend_from_slice(&LINKTYPE_ETHERNET.to_le_bytes());
        idb.extend_from_slice(&0u16.to_le_bytes());
        idb.extend_from_slice(&65535u32.to_le_bytes());
        idb.extend_from_slice(&area);
        idb.extend_from_slice(&len.to_le_bytes());
        out.write_all(&idb)?;
        Ok(Writer { out })
    }

    pub fn write_packet(&mut self, timestamp_ns: u64, direction: Option<Direction>, data: &[u8]) -> io::Result<()> {
        let padded = (data.len() + 3) & !3;
        let flags = match direction {
            Some(Direction::FromBoard) => 1u32,
            Some(Direction::ToBoard) => 2,
            None => 0,
        };
        let len = (32 + padded + 12) as u32;
        let mut block = Vec::with_capacity(len as usize);
        block.extend_from_slice(&ENHANCED_PACKET.to_le_bytes());
        block.extend_from_slice(&len.to_le_bytes());
        block.extend_from_slice(&0u32.to_le_bytes());
        block.extend_from_slice(&((timestamp_ns >> 32) as u32).to_le_bytes());
        block.extend_from_slice(&(timestamp_ns as u32).to_le_bytes());
        block.extend_from_slice(&(data.len() as u32).to_le_bytes());
        block.extend_from_slice(&(data.len() as u32).to_le_bytes());
        block.extend_from_slice(data);
        block.resize(28 + padded, 0);
        block.extend_from_slice(&OPTION_EPB_FLAGS.to_le_bytes());
        block.extend_from_slice(&4u16.to_le_bytes());
        block.extend_from_slice(&flags.to_le_bytes());
        block.extend_from_slice(&[0; 4]);
        block.extend_from_slice(&len.to_le_bytes());
        self.out.write_all(&block)
    }

    pub fn into_inner(self) -> W {
        self.out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_then_read() {
        let mut writer = Writer::new(Vec::new(), "eth1").unwrap();
        writer.write_packet(1_790_000_000_123_456_789, Some(Direction::ToBoard), &[1, 2, 3, 4, 5]).unwrap();
        writer.write_packet(1_790_000_000_123_457_000, Some(Direction::FromBoard), &[9; 64]).unwrap();
        writer.write_packet(5, None, &[]).unwrap();
        let bytes = writer.into_inner();
        let packets: Vec<_> = Reader::new(&bytes).unwrap().collect::<Result<_, _>>().unwrap();
        assert_eq!(packets.len(), 3);
        assert_eq!(packets[0].timestamp_ns, 1_790_000_000_123_456_789);
        assert_eq!(packets[0].direction, Some(Direction::ToBoard));
        assert_eq!(packets[0].data, &[1, 2, 3, 4, 5]);
        assert_eq!(packets[1].direction, Some(Direction::FromBoard));
        assert_eq!(packets[1].data.len(), 64);
        assert_eq!(packets[2].direction, None);
    }

    #[test]
    fn rejects_other_files() {
        assert!(matches!(Reader::new(b"not a capture at all"), Err(PcapngError::NotPcapng)));
    }

    #[test]
    fn microsecond_default() {
        assert_eq!(resolution(6), 1_000_000);
        assert_eq!(resolution(9), 1_000_000_000);
        assert_eq!(resolution(0x80 | 10), 1024);
    }
}
