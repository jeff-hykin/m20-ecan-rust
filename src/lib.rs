//! The Deep Robotics M20 (Lynx) motor bus ("ECAN"): the Ethernet protocol between the vendor walking controller
//! (rl_deploy, on the AOS board's eth1) and the ECAN board that drives the 16 joint motors and reads both batteries.
//!
//! - [`frame`]: the 10-byte header, checksum, frame types; [`ethernet`]: the Ethernet header around it.
//! - [`joint`]: joint command / state records, named by leg and joint, and the motor-to-URDF calibration.
//! - [`message`]: typed messages per direction (joint commands, joint states + board IMU, battery, charge enable, ...).
//! - [`tap`] and [`pcapng`]: passive readers for what the `ecan_tap.so` shim copies out of rl_deploy.
//! - `transport` (Linux): the vendor-kernel raw-socket path rl_deploy sends and receives through.
//! - [`startup`], [`owner`] and `bus` (Linux): owning the bus in rl_deploy's place: its startup, then one frame per
//!   1 ms tick, zero gains unless allowed.
//!
//! ```
//! use m20_ecan::{decode_ethernet, message::{Direction, Message}};
//! # let frame = m20_ecan::ethernet::wrap([2, 0, 0, 0, 0, 1], &Message::encode_charge_enable(3));
//! let (header, message) = decode_ethernet(&frame, Direction::ToBoard).unwrap();
//! assert_eq!(message, Message::ChargeEnable);
//! assert_eq!(header.seq, 3);
//! ```

#[cfg(target_os = "linux")]
pub mod bus;
pub mod ethernet;
pub mod frame;
pub mod joint;
pub mod message;
pub mod owner;
pub mod pcapng;
pub mod startup;
pub mod tap;
#[cfg(target_os = "linux")]
pub mod transport;

pub use frame::{Frame, FrameError, FrameType, Header};
pub use joint::{Calibration, JointCommand, JointId, JointKind, JointState, Leg};
pub use message::{Direction, Message};

/// Parse and decode a whole Ethernet frame (checksum not enforced; see [`Frame::checksum_ok`]).
pub fn decode_ethernet(ethernet: &[u8], direction: Direction) -> Result<(Header, Message), FrameError> {
    let frame = Frame::parse_ethernet(ethernet)?;
    Ok((frame.header, Message::decode(&frame, direction)?))
}
