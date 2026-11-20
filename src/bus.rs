//! The motor-bus owner on the AOS board (Linux): runs an [`Owner`] on the vendor raw socket. Sending happens on the
//! caller's thread at its pace; a receiver thread takes every frame the board sends (the vendor receive path blocks)
//! and hands it over with its CLOCK_REALTIME time. Refuses to open while rl_deploy runs: one owner per bus.

use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::joint::JointCommand;
use crate::owner::{Owner, OwnerConfig};
use crate::transport::{recv_address, RawSocket, MSG_YSC};

/// A frame seen on the bus: CLOCK_REALTIME ns, whether it was ours (sent) or the board's, the Ethernet frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BusFrame {
    pub timestamp_ns: u64,
    pub direction: crate::Direction,
    pub frame: Vec<u8>,
}

fn realtime_ns() -> u64 {
    std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0)
}

pub struct Bus {
    socket: Arc<RawSocket>,
    received: mpsc::Receiver<BusFrame>,
    pub owner: Owner,
    deadline: Option<Instant>,
}

/// Whether a process named rl_deploy is running.
pub fn rl_deploy_running() -> bool {
    std::fs::read_dir("/proc")
        .map(|entries| {
            entries.flatten().any(|e| {
                std::fs::read_to_string(e.path().join("comm")).map(|c| c.trim() == "rl_deploy").unwrap_or(false)
            })
        })
        .unwrap_or(false)
}

impl Bus {
    pub fn open(interface: &str, config: OwnerConfig) -> std::io::Result<Bus> {
        if rl_deploy_running() {
            return Err(std::io::Error::other("rl_deploy is running; it owns the motor bus"));
        }
        let socket = Arc::new(RawSocket::open(interface)?);
        socket.set_nonblocking(false)?;
        let (tx, received) = mpsc::channel();
        let reader = socket.clone();
        std::thread::Builder::new().name("m20-bus-recv".into()).spawn(move || {
            use std::os::fd::AsRawFd;
            let mut buf = [0u8; 1514];
            loop {
                let mut address = recv_address(reader.ifindex(), reader.mac());
                let mut len = std::mem::size_of::<libc::sockaddr_ll>() as libc::socklen_t;
                let n = unsafe {
                    libc::recvfrom(
                        reader.as_raw_fd(),
                        buf.as_mut_ptr().cast(),
                        buf.len(),
                        MSG_YSC,
                        (&mut address as *mut libc::sockaddr_ll).cast(),
                        &mut len,
                    )
                };
                if n <= 14 || buf[6..12] == reader.mac() || buf[12..14] != [0x44, 0x52] {
                    continue;
                }
                let frame = BusFrame {
                    timestamp_ns: realtime_ns(),
                    direction: crate::Direction::FromBoard,
                    frame: buf[..n as usize].to_vec(),
                };
                if tx.send(frame).is_err() {
                    return;
                }
            }
        })?;
        Ok(Bus { socket, received, owner: Owner::new(config), deadline: None })
    }

    /// Wait for the next tick's time, send its frame (commands as in [`Owner::next`]), and return what was sent.
    pub fn tick(&mut self, commands: Option<&[JointCommand]>) -> std::io::Result<Option<BusFrame>> {
        let tick = self.owner.next(commands);
        let deadline = self.deadline.map_or_else(Instant::now, |d| d + tick.delay);
        // late (a stall): restart the pace from now rather than sending a burst
        let deadline = deadline.max(Instant::now() - Duration::from_millis(1));
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
        self.deadline = Some(deadline);
        let Some(body) = tick.body else { return Ok(None) };
        self.socket.send(&body)?;
        Ok(Some(BusFrame {
            timestamp_ns: realtime_ns(),
            direction: crate::Direction::ToBoard,
            frame: crate::ethernet::wrap(self.socket.mac(), &body),
        }))
    }

    /// Every frame the board sent since the last call.
    pub fn drain(&self) -> Vec<BusFrame> {
        self.received.try_iter().collect()
    }

    /// The next frame from the board, waiting at most `timeout`.
    pub fn recv_timeout(&self, timeout: Duration) -> Option<BusFrame> {
        self.received.recv_timeout(timeout).ok()
    }
}
