//! The motor-bus owner on the AOS board (Linux): runs an [`Owner`] on the vendor raw socket. Sending happens on the
//! caller's thread at its pace; a receiver thread takes every frame the board sends (the vendor receive path blocks)
//! and hands it over with its CLOCK_REALTIME time. Refuses to open while rl_deploy runs: one owner per bus.

use std::io;
use std::sync::atomic::{AtomicI32, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::Duration;

use crate::ethernet::{ETHERNET_HEADER_LEN, ETHERTYPE};
use crate::joint::JointCommand;
use crate::owner::{Owner, OwnerConfig};
use crate::transport::{RawSocket, MAX_FRAME};

/// A frame seen on the bus: CLOCK_REALTIME ns, whether it was ours (sent) or the board's, the Ethernet frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BusFrame {
    pub timestamp_ns: u64,
    pub direction: crate::Direction,
    pub frame: Vec<u8>,
}

fn realtime_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| u64::try_from(d.as_nanos()).unwrap_or(u64::MAX))
        .unwrap_or(0)
}

/// CLOCK_MONOTONIC as a Duration since its epoch.
fn monotonic_now() -> Duration {
    let mut now = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut now) };
    Duration::new(u64::try_from(now.tv_sec).unwrap_or(0), u32::try_from(now.tv_nsec).unwrap_or(0))
}

/// Sleep until an absolute CLOCK_MONOTONIC time (no drift from the time spent computing it), resuming after signals.
fn sleep_until(deadline: Duration) {
    let target = libc::timespec {
        tv_sec: libc::time_t::try_from(deadline.as_secs()).unwrap_or(libc::time_t::MAX),
        tv_nsec: deadline.subsec_nanos().into(),
    };
    while unsafe { libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &target, std::ptr::null_mut()) }
        == libc::EINTR
    {}
}

/// Receive errors other than EINTR / EAGAIN the receiver thread has seen (it keeps receiving after them).
#[derive(Debug, Default)]
struct ReceiveErrors {
    count: AtomicU64,
    last_errno: AtomicI32,
}

pub struct Bus {
    socket: Arc<RawSocket>,
    received: mpsc::Receiver<BusFrame>,
    errors: Arc<ReceiveErrors>,
    pub owner: Owner,
    /// The last tick's CLOCK_MONOTONIC send time.
    deadline: Option<Duration>,
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
        let errors = Arc::new(ReceiveErrors::default());
        let seen = errors.clone();
        // blocks in recvfrom until the board answers; exits on the first frame after the Bus is dropped
        std::thread::Builder::new().name("m20-bus-recv".into()).spawn(move || {
            let mut buf = [0u8; MAX_FRAME];
            loop {
                let n = match reader.recv_ethernet(&mut buf) {
                    Ok(n) => n,
                    Err(error) => {
                        if error.kind() != io::ErrorKind::WouldBlock {
                            seen.count.fetch_add(1, Ordering::Relaxed);
                            seen.last_errno.store(error.raw_os_error().unwrap_or(0), Ordering::Relaxed);
                        }
                        continue;
                    }
                };
                let Some(frame) = buf.get(..n) else { continue };
                if !is_from_board(frame, &reader.mac()) {
                    continue;
                }
                let frame = BusFrame {
                    timestamp_ns: realtime_ns(),
                    direction: crate::Direction::FromBoard,
                    frame: frame.to_vec(),
                };
                if tx.send(frame).is_err() {
                    return;
                }
            }
        })?;
        Ok(Bus { socket, received, errors, owner: Owner::new(config), deadline: None })
    }

    /// Wait for the next tick's time, send its frame (commands as in [`Owner::next`]), and return what was sent.
    pub fn tick(&mut self, commands: Option<&[JointCommand]>) -> io::Result<Option<BusFrame>> {
        let tick = self.owner.next(commands);
        let now = monotonic_now();
        let deadline = self.deadline.map_or(now, |d| d + tick.delay);
        // late (a stall): restart the pace from now rather than sending a burst (at most one tick is caught up)
        let deadline = deadline.max(now.saturating_sub(crate::startup::TICK));
        sleep_until(deadline);
        self.deadline = Some(deadline);
        let Some(body) = tick.body else { return Ok(None) };
        self.socket.send(&body)?;
        Ok(Some(BusFrame {
            timestamp_ns: realtime_ns(),
            direction: crate::Direction::ToBoard,
            frame: crate::ethernet::wrap(self.socket.mac(), &body),
        }))
    }

    /// Receive errors (other than EINTR / EAGAIN) so far, and the last one's errno.
    pub fn receive_errors(&self) -> (u64, Option<io::Error>) {
        let count = self.errors.count.load(Ordering::Relaxed);
        let last = (count > 0).then(|| io::Error::from_raw_os_error(self.errors.last_errno.load(Ordering::Relaxed)));
        (count, last)
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

/// An ECAN frame someone other than this host sent (the board's replies; our own sends can loop back).
fn is_from_board(frame: &[u8], own_mac: &[u8; 6]) -> bool {
    frame.len() > ETHERNET_HEADER_LEN
        && frame.get(6..12) != Some(own_mac.as_slice())
        && frame.get(12..14) == Some(ETHERTYPE.to_be_bytes().as_slice())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_frames_only() {
        let own = [2, 0, 0, 0, 0, 1];
        let board = crate::ethernet::wrap([0x68, 0, 0, 0, 0xf0, 0x97], &[0; 10]);
        assert!(is_from_board(&board, &own));
        assert!(!is_from_board(&crate::ethernet::wrap(own, &[0; 10]), &own));
        assert!(!is_from_board(&board[..14], &own));
        let mut other = board.clone();
        other[12] = 0x08;
        assert!(!is_from_board(&other, &own));
    }

    #[test]
    fn sleep_until_is_absolute() {
        let start = monotonic_now();
        sleep_until(start + Duration::from_millis(3));
        assert!(monotonic_now() >= start + Duration::from_millis(3));
        sleep_until(start);
    }
}
