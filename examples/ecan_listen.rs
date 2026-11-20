//! Receive-only: print every frame the vendor kernel's private ECAN path delivers on an interface, for a while.
//! Sends nothing. Refuses to run while rl_deploy is running (a second reader could take frames rl_deploy needs).
//! usage: ecan_listen [interface (eth1)] [seconds (10)]

#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use std::time::{Duration, Instant};

    use m20_ecan::transport::RawSocket;
    use m20_ecan::{Direction, Frame, Message};

    let args: Vec<String> = std::env::args().collect();
    let interface = args.get(1).map(String::as_str).unwrap_or("eth1");
    let seconds: f64 = args.get(2).map(|s| s.parse()).transpose()?.unwrap_or(10.0);
    for entry in std::fs::read_dir("/proc")?.flatten() {
        if std::fs::read_to_string(entry.path().join("comm")).map(|comm| comm.trim() == "rl_deploy").unwrap_or(false) {
            return Err("rl_deploy is running; stop it first (only with the robot sitting or docked)".into());
        }
    }
    let socket = RawSocket::open(interface)?;
    socket.set_nonblocking(false)?;
    let timeout = libc::timeval { tv_sec: 0, tv_usec: 200_000 };
    unsafe {
        use std::os::fd::AsRawFd;
        libc::setsockopt(
            socket.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_RCVTIMEO,
            (&timeout as *const libc::timeval).cast(),
            std::mem::size_of::<libc::timeval>() as u32,
        );
    }
    println!(
        "listening on {interface} (ifindex {}, mac {}) for {seconds} s",
        socket.ifindex(),
        m20_ecan::ethernet::format_mac(&socket.mac())
    );
    let start = Instant::now();
    let mut frame = [0u8; 1514];
    let mut count = 0usize;
    while start.elapsed() < Duration::from_secs_f64(seconds) {
        let len = match socket.recv_ethernet(&mut frame) {
            Ok(len) => len,
            Err(error) if matches!(error.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) => {
                continue
            }
            Err(error) => return Err(error.into()),
        };
        count += 1;
        let ethernet = &frame[..len];
        let summary = match Frame::parse_ethernet(ethernet) {
            Ok(parsed) => match Message::decode(&parsed, Direction::FromBoard) {
                Ok(Message::JointStates { joints, .. }) => format!("joint states x{}", joints.len()),
                Ok(Message::Battery(packs)) => format!("battery x{}", packs.len()),
                Ok(other) => format!("{other:?}").chars().take(120).collect(),
                Err(error) => format!("{error}"),
            },
            Err(error) => format!("{error}"),
        };
        let hex: String = ethernet.iter().take(24).map(|b| format!("{b:02x}")).collect();
        println!("{:10.6} len {len:4} {hex} {summary}", start.elapsed().as_secs_f64());
    }
    println!("{count} frames in {seconds} s");
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("ecan_listen needs Linux (AF_PACKET)");
}
