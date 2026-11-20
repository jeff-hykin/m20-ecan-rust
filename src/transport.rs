//! The raw-socket transport rl_deploy's ECAN master uses (libecanmaster `SocketRaw`), Linux only.
//!
//! The vendor kernel (Rockchip BSP 5.10.198 on the AOS board) has a private path for these frames: `sendto` and
//! `recvfrom` with flag `0x80000000` on an `AF_PACKET` socket go through `packet_snd_ysc` / `stmmac_ysc_xmit` and
//! `recv_ysc`, skipping the network stack (so tcpdump never sees them). This mirrors `SocketRaw` exactly:
//!
//! - `socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL))`, `SIOCGIFINDEX` and `SIOCGIFHWADDR` on the interface (eth1),
//!   non-blocking;
//! - send: Ethernet header (to 22:33:44:55:66:77, from the interface's MAC, ethertype 0x4452) + the ECAN body,
//!   `sendto(.., 0x80000000, sockaddr_ll { AF_PACKET, htons(ETH_P_ALL), ifindex }, 20)`;
//! - receive: `recvfrom(.., 1514, 0x80000000, sockaddr_ll { AF_PACKET, ifindex, halen 0, addr = own MAC }, 20)`,
//!   keeping frames with ethertype 0x4452 and returning the body after the Ethernet header.
//!
//! On the M20 the board only answers rl_deploy's 1 kHz commands; a second sender on the bus would fight rl_deploy
//! for the motors. Nothing in this crate sends unless [`RawSocket::send`] is called.

use std::ffi::CString;
use std::io;
use std::mem;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use crate::ethernet::{self, ETHERNET_HEADER_LEN, ETHERTYPE};

/// The flag that selects the vendor kernel's private ECAN path.
pub const MSG_YSC: libc::c_int = 0x8000_0000u32 as libc::c_int;
/// `recvfrom` length rl_deploy asks for (a full Ethernet frame without FCS).
pub const MAX_FRAME: usize = 1514;
const SOCKADDR_LL_LEN: libc::socklen_t = 20;

/// An open ECAN socket on one interface.
pub struct RawSocket {
    fd: OwnedFd,
    ifindex: i32,
    mac: [u8; 6],
}

fn last_error(what: &str) -> io::Error {
    let error = io::Error::last_os_error();
    io::Error::new(error.kind(), format!("{what}: {error}"))
}

fn ifreq_for(name: &str) -> io::Result<libc::ifreq> {
    let bytes = name.as_bytes();
    if bytes.is_empty() || bytes.len() >= libc::IFNAMSIZ {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "bad interface name"));
    }
    let mut ifreq: libc::ifreq = unsafe { mem::zeroed() };
    for (slot, &byte) in ifreq.ifr_name.iter_mut().zip(bytes) {
        *slot = byte as libc::c_char;
    }
    Ok(ifreq)
}

impl RawSocket {
    /// Open the socket on `interface` (eth1 on the AOS board). Needs CAP_NET_RAW.
    pub fn open(interface: &str) -> io::Result<Self> {
        let protocol = (libc::ETH_P_ALL as u16).to_be() as libc::c_int;
        let raw = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW, protocol) };
        if raw < 0 {
            return Err(last_error("socket(AF_PACKET)"));
        }
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let mut ifreq = ifreq_for(interface)?;
        if unsafe { libc::ioctl(raw, libc::SIOCGIFINDEX as _, &mut ifreq) } < 0 {
            return Err(last_error("SIOCGIFINDEX"));
        }
        let ifindex = unsafe { ifreq.ifr_ifru.ifru_ifindex };
        let mut ifreq = ifreq_for(interface)?;
        if unsafe { libc::ioctl(raw, libc::SIOCGIFHWADDR as _, &mut ifreq) } < 0 {
            return Err(last_error("SIOCGIFHWADDR"));
        }
        let hw = unsafe { ifreq.ifr_ifru.ifru_hwaddr.sa_data };
        #[allow(clippy::unnecessary_cast)] // c_char is i8 on x86_64
        let mac = [hw[0], hw[1], hw[2], hw[3], hw[4], hw[5]].map(|b| b as u8);
        let socket = RawSocket { fd, ifindex, mac };
        socket.set_nonblocking(true)?;
        Ok(socket)
    }

    /// rl_deploy keeps the socket non-blocking and polls it; blocking reads are available for tools.
    pub fn set_nonblocking(&self, nonblocking: bool) -> io::Result<()> {
        let raw = self.fd.as_raw_fd();
        let flags = unsafe { libc::fcntl(raw, libc::F_GETFL) };
        if flags < 0 {
            return Err(last_error("F_GETFL"));
        }
        let flags = if nonblocking { flags | libc::O_NONBLOCK } else { flags & !libc::O_NONBLOCK };
        if unsafe { libc::fcntl(raw, libc::F_SETFL, flags) } < 0 {
            return Err(last_error("F_SETFL"));
        }
        Ok(())
    }

    pub fn mac(&self) -> [u8; 6] {
        self.mac
    }

    pub fn ifindex(&self) -> i32 {
        self.ifindex
    }

    /// Send one ECAN body (header + payload) to the board. On the robot this drives the motors.
    pub fn send(&self, body: &[u8]) -> io::Result<usize> {
        let frame = ethernet::wrap(self.mac, body);
        let address = send_address(self.ifindex);
        let sent = unsafe {
            libc::sendto(
                self.fd.as_raw_fd(),
                frame.as_ptr().cast(),
                frame.len(),
                MSG_YSC,
                (&address as *const libc::sockaddr_ll).cast(),
                SOCKADDR_LL_LEN,
            )
        };
        if sent < 0 {
            return Err(last_error("sendto"));
        }
        Ok(sent as usize)
    }

    /// Receive one frame from the board into `body` (the bytes after the Ethernet header). `WouldBlock` when
    /// non-blocking and nothing is queued; frames that are not ECAN are skipped.
    pub fn recv(&self, body: &mut [u8]) -> io::Result<usize> {
        let mut frame = [0u8; MAX_FRAME];
        loop {
            let len = self.recv_ethernet(&mut frame)?;
            if len < ETHERNET_HEADER_LEN || u16::from_be_bytes([frame[12], frame[13]]) != ETHERTYPE {
                continue;
            }
            let body_len = len - ETHERNET_HEADER_LEN;
            if body_len > body.len() {
                return Err(io::Error::new(io::ErrorKind::InvalidInput, "buffer smaller than the frame"));
            }
            body[..body_len].copy_from_slice(&frame[ETHERNET_HEADER_LEN..len]);
            return Ok(body_len);
        }
    }

    /// Receive one whole Ethernet frame, whatever its type.
    pub fn recv_ethernet(&self, frame: &mut [u8]) -> io::Result<usize> {
        let mut address = recv_address(self.ifindex, self.mac);
        let mut address_len = SOCKADDR_LL_LEN;
        let len = unsafe {
            libc::recvfrom(
                self.fd.as_raw_fd(),
                frame.as_mut_ptr().cast(),
                frame.len().min(MAX_FRAME),
                MSG_YSC,
                (&mut address as *mut libc::sockaddr_ll).cast(),
                &mut address_len,
            )
        };
        if len < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(len as usize)
    }
}

impl AsRawFd for RawSocket {
    fn as_raw_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }
}

/// The `sockaddr_ll` `SocketRaw::Send` passes: family, protocol ETH_P_ALL, ifindex, the rest zero.
pub fn send_address(ifindex: i32) -> libc::sockaddr_ll {
    let mut address: libc::sockaddr_ll = unsafe { mem::zeroed() };
    address.sll_family = libc::AF_PACKET as u16;
    address.sll_protocol = (libc::ETH_P_ALL as u16).to_be();
    address.sll_ifindex = ifindex;
    address
}

/// The `sockaddr_ll` `SocketRaw::Receive` passes: family, ifindex, halen 0, addr = own MAC, protocol 0.
pub fn recv_address(ifindex: i32, mac: [u8; 6]) -> libc::sockaddr_ll {
    let mut address: libc::sockaddr_ll = unsafe { mem::zeroed() };
    address.sll_family = libc::AF_PACKET as u16;
    address.sll_ifindex = ifindex;
    address.sll_halen = 0;
    address.sll_addr[..6].copy_from_slice(&mac);
    address
}

/// The ifindex of an interface by name (`if_nametoindex`), as `SocketRaw::Receive` looks it up.
pub fn interface_index(name: &str) -> io::Result<i32> {
    let name = CString::new(name).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "NUL in name"))?;
    match unsafe { libc::if_nametoindex(name.as_ptr()) } {
        0 => Err(last_error("if_nametoindex")),
        index => Ok(index as i32),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn addresses_match_socket_raw() {
        assert_eq!(mem::size_of::<libc::sockaddr_ll>(), SOCKADDR_LL_LEN as usize);
        let send = send_address(5);
        let bytes: [u8; 20] = unsafe { mem::transmute(send) };
        // family 17, protocol 0x0300 as stored by `stp w5, w4` (w5 = 0x03000011), ifindex 5
        assert_eq!(&bytes[..8], &[0x11, 0x00, 0x00, 0x03, 5, 0, 0, 0]);
        assert!(bytes[8..].iter().all(|&b| b == 0));
        let recv = recv_address(5, [0xf2, 0x53, 0xbb, 0xa2, 0x93, 0xad]);
        let bytes: [u8; 20] = unsafe { mem::transmute(recv) };
        assert_eq!(&bytes[..8], &[0x11, 0, 0, 0, 5, 0, 0, 0]);
        assert_eq!(bytes[11], 0);
        assert_eq!(&bytes[12..18], &[0xf2, 0x53, 0xbb, 0xa2, 0x93, 0xad]);
    }

    #[test]
    fn flag_value() {
        assert_eq!(MSG_YSC as u32, 0x8000_0000);
    }

    #[test]
    fn loopback_lookup() {
        assert!(interface_index("lo").unwrap() > 0);
        assert!(interface_index("no-such-if0").is_err());
    }
}
