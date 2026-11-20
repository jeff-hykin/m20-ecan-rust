//! Passive taps: frames copied out of rl_deploy by the `ecan_tap.so` LD_PRELOAD shim, either recorded to pcapng or
//! streamed live to a unix datagram socket. Reading a tap sends nothing to the motor bus.
//!
//! Live stream: every datagram holds one or more records, each a 32-byte little-endian header and the frame.
//!
//! ```text
//! 0  [u8; 4] "ETAP"
//! 4  u16 version (1)
//! 6  u16 frame length (bytes after this header)
//! 8  u32 direction: 1 = received by rl_deploy (from the board), 2 = sent by rl_deploy (to the board)
//! 12 u32 frames the shim's ring dropped so far (rl_deploy's threads never wait; a full ring drops)
//! 16 u32 datagrams the shim could not send so far (no reader, or the reader's queue was full)
//! 20 u32 reserved (0)
//! 24 u64 CLOCK_REALTIME nanoseconds when rl_deploy's sendto/recvfrom returned
//! 32 the Ethernet frame
//! ```

use std::io;
#[cfg(unix)]
use std::path::{Path, PathBuf};

use crate::message::Direction;
use crate::pcapng::{Packet, PcapngError, Reader};

pub const RECORD_MAGIC: [u8; 4] = *b"ETAP";
pub const RECORD_VERSION: u16 = 1;
pub const RECORD_HEADER_LEN: usize = 32;
/// Where the shim streams to when `ECAN_TAP_SOCKET` is unset.
pub const DEFAULT_SOCKET_PATH: &str = "/var/opt/robot/data/ecan_capture/live.sock";

/// One frame the shim saw.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TapRecord {
    pub timestamp_ns: u64,
    pub direction: Direction,
    pub ring_dropped: u32,
    pub send_dropped: u32,
    /// The Ethernet frame.
    pub frame: Vec<u8>,
}

impl TapRecord {
    /// A frame longer than the u16 length field can say is cut to fit.
    pub fn encode(&self, out: &mut Vec<u8>) {
        let frame = self.frame.get(..usize::from(u16::MAX)).unwrap_or(&self.frame);
        out.extend_from_slice(&RECORD_MAGIC);
        out.extend_from_slice(&RECORD_VERSION.to_le_bytes());
        out.extend_from_slice(&u16::try_from(frame.len()).unwrap_or(u16::MAX).to_le_bytes());
        let direction: u32 = match self.direction {
            Direction::FromBoard => 1,
            Direction::ToBoard => 2,
        };
        out.extend_from_slice(&direction.to_le_bytes());
        out.extend_from_slice(&self.ring_dropped.to_le_bytes());
        out.extend_from_slice(&self.send_dropped.to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes());
        out.extend_from_slice(&self.timestamp_ns.to_le_bytes());
        out.extend_from_slice(frame);
    }

    /// Every record in a datagram; stops at the first malformed one.
    pub fn decode_datagram(datagram: &[u8]) -> Result<Vec<TapRecord>, io::Error> {
        let bad = |what: &str| io::Error::new(io::ErrorKind::InvalidData, format!("bad tap record: {what}"));
        let mut records = Vec::new();
        let mut at = 0;
        while at < datagram.len() {
            let header = datagram.get(at..at + RECORD_HEADER_LEN).ok_or_else(|| bad("short header"))?;
            if header[0..4] != RECORD_MAGIC {
                return Err(bad("magic"));
            }
            let version = u16::from_le_bytes([header[4], header[5]]);
            if version != RECORD_VERSION {
                return Err(bad("version"));
            }
            let len = u16::from_le_bytes([header[6], header[7]]) as usize;
            let word = |i: usize| u32::from_le_bytes([header[i], header[i + 1], header[i + 2], header[i + 3]]);
            let direction = match word(8) {
                1 => Direction::FromBoard,
                2 => Direction::ToBoard,
                _ => return Err(bad("direction")),
            };
            let start = at + RECORD_HEADER_LEN;
            let frame = datagram.get(start..start + len).ok_or_else(|| bad("short frame"))?;
            records.push(TapRecord {
                timestamp_ns: u64::from_le_bytes(std::array::from_fn(|k| header[24 + k])),
                direction,
                ring_dropped: word(12),
                send_dropped: word(16),
                frame: frame.to_vec(),
            });
            at = start + len;
        }
        Ok(records)
    }
}

/// A captured frame: CLOCK_REALTIME nanoseconds, direction, the Ethernet frame.
pub type CapturedFrame<'a> = (u64, Direction, &'a [u8]);

/// The frames of a shim pcapng, with their direction (packets without one are skipped).
pub fn read_capture(bytes: &[u8]) -> Result<Vec<CapturedFrame<'_>>, PcapngError> {
    let mut out = Vec::new();
    for packet in Reader::new(bytes)? {
        let Packet { timestamp_ns, direction, data, .. } = packet?;
        if let Some(direction) = direction {
            out.push((timestamp_ns, direction, data));
        }
    }
    Ok(out)
}

/// The reading end of the shim's live stream: binds the socket path the shim sends to.
#[cfg(unix)]
pub struct TapSocket {
    socket: std::os::unix::net::UnixDatagram,
    path: PathBuf,
    buffer: Vec<u8>,
}

#[cfg(unix)]
impl TapSocket {
    /// Bind `path`, replacing a stale socket file left by an earlier reader.
    pub fn bind(path: impl AsRef<Path>) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if std::fs::symlink_metadata(&path).is_ok() {
            std::fs::remove_file(&path)?;
        }
        let socket = std::os::unix::net::UnixDatagram::bind(&path)?;
        Ok(TapSocket { socket, path, buffer: vec![0; 1 << 16] })
    }

    /// Block until the shim sends a datagram (resuming after signals), and decode its records.
    pub fn recv(&mut self) -> io::Result<Vec<TapRecord>> {
        let len = loop {
            match self.socket.recv(&mut self.buffer) {
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                other => break other?,
            }
        };
        TapRecord::decode_datagram(self.buffer.get(..len).unwrap_or(&self.buffer))
    }

    pub fn set_read_timeout(&self, timeout: Option<std::time::Duration>) -> io::Result<()> {
        self.socket.set_read_timeout(timeout)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

#[cfg(unix)]
impl Drop for TapSocket {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}

/// The sending end of the live stream, for a bus owner other than rl_deploy (the Rust controller) to feed the same
/// readers the shim feeds. Non-blocking: a datagram the reader is not ready for is dropped and counted.
#[cfg(unix)]
pub struct TapSender {
    socket: std::os::unix::net::UnixDatagram,
    path: PathBuf,
    pub send_dropped: u32,
}

#[cfg(unix)]
impl TapSender {
    pub fn new(path: impl AsRef<Path>) -> io::Result<Self> {
        let socket = std::os::unix::net::UnixDatagram::unbound()?;
        socket.set_nonblocking(true)?;
        Ok(TapSender { socket, path: path.as_ref().to_path_buf(), send_dropped: 0 })
    }

    /// Send one frame as one record; false if no reader took it.
    pub fn send(&mut self, timestamp_ns: u64, direction: Direction, frame: &[u8]) -> bool {
        let mut datagram = Vec::with_capacity(RECORD_HEADER_LEN + frame.len());
        TapRecord { timestamp_ns, direction, ring_dropped: 0, send_dropped: self.send_dropped, frame: frame.to_vec() }
            .encode(&mut datagram);
        let ok = loop {
            match self.socket.send_to(&datagram, &self.path) {
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                result => break result.is_ok(),
            }
        };
        if !ok {
            self.send_dropped = self.send_dropped.wrapping_add(1);
        }
        ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(direction: Direction, len: usize) -> TapRecord {
        TapRecord {
            timestamp_ns: 1_790_000_000_000_000_001,
            direction,
            ring_dropped: 3,
            send_dropped: 7,
            frame: (0..len).map(|i| i as u8).collect(),
        }
    }

    #[test]
    fn datagram_round_trip() {
        let records =
            vec![record(Direction::ToBoard, 398), record(Direction::FromBoard, 438), record(Direction::FromBoard, 0)];
        let mut datagram = Vec::new();
        for r in &records {
            r.encode(&mut datagram);
        }
        assert_eq!(TapRecord::decode_datagram(&datagram).unwrap(), records);
        assert!(TapRecord::decode_datagram(&datagram[..datagram.len() - 1]).is_err());
        assert!(TapRecord::decode_datagram(b"XTAP").is_err());
    }

    #[test]
    fn oversized_frame_encodes_consistently() {
        let mut datagram = Vec::new();
        record(Direction::ToBoard, 70_000).encode(&mut datagram);
        let decoded = TapRecord::decode_datagram(&datagram).unwrap();
        assert_eq!(decoded[0].frame.len(), 65535);
    }

    #[cfg(unix)]
    #[test]
    fn sender_feeds_a_reader() {
        let dir = std::env::temp_dir().join(format!("m20-ecan-tap-sender-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("live.sock");
        let mut sender = TapSender::new(&path).unwrap();
        assert!(!sender.send(1, Direction::ToBoard, &[1, 2, 3]), "no reader yet");
        let mut tap = TapSocket::bind(&path).unwrap();
        assert!(sender.send(2, Direction::FromBoard, &[4, 5]));
        let got = tap.recv().unwrap();
        assert_eq!(
            (got[0].timestamp_ns, got[0].direction, got[0].frame.clone(), got[0].send_dropped),
            (2, Direction::FromBoard, vec![4, 5], 1)
        );
        drop(tap);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[cfg(unix)]
    #[test]
    fn socket_round_trip() {
        let dir = std::env::temp_dir().join(format!("m20-ecan-tap-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("live.sock");
        let mut tap = TapSocket::bind(&path).unwrap();
        let sender = std::os::unix::net::UnixDatagram::unbound().unwrap();
        let mut datagram = Vec::new();
        record(Direction::FromBoard, 100).encode(&mut datagram);
        sender.send_to(&datagram, &path).unwrap();
        assert_eq!(tap.recv().unwrap(), vec![record(Direction::FromBoard, 100)]);
        drop(tap);
        assert!(!path.exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
