//! Typed ECAN messages, by direction and frame type.

use crate::frame::{self, Frame, FrameError, FrameType};
use crate::joint::{JointCommand, JointState, JointUnit, JOINT_UNIT_LEN};

/// Which way a frame went.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    /// From the host (rl_deploy) to the ECAN board.
    ToBoard,
    /// From the ECAN board to the host.
    FromBoard,
}

/// Bytes of `ecan::ImuData` at the front of a joint-state payload.
pub const IMU_LEN: usize = 40;
/// Bytes of one `ecan::BatterySensorData` (`#pragma pack(2)`).
pub const BATTERY_PACK_LEN: usize = 258;
/// Bytes of `ecan::ECAN_STATUS`.
pub const ECAN_STATUS_LEN: usize = 256;

/// Joint records one frame's u16 payload length can hold.
const MAX_UNITS: usize = u16::MAX as usize / JOINT_UNIT_LEN;

/// `ecan::ImuData`: the ECAN board's own IMU (all zero on the M20, whose body IMU is the yesense on a serial port).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuData {
    pub timestamp: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub omega: [f32; 3],
    pub acc: [f32; 3],
}

impl ImuData {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let b = bytes.get(..IMU_LEN)?;
        let f = |i: usize| f32::from_le_bytes(std::array::from_fn(|k| b[4 + 4 * i + k]));
        Some(ImuData {
            timestamp: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            roll: f(0),
            pitch: f(1),
            yaw: f(2),
            omega: [f(3), f(4), f(5)],
            acc: [f(6), f(7), f(8)],
        })
    }

    pub fn write(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.timestamp.to_le_bytes());
        for value in [self.roll, self.pitch, self.yaw].iter().chain(&self.omega).chain(&self.acc) {
            out.extend_from_slice(&value.to_le_bytes());
        }
    }

    pub fn is_zero(&self) -> bool {
        *self == ImuData::default()
    }
}

/// `ecan::ECAN_STATUS`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EcanStatus {
    pub hard_stop: u8,
    pub imu_status: u8,
}

/// A decoded frame.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// 0x89 to the board: one command per joint.
    JointCommands(Vec<JointCommand>),
    /// 0x89 from the board: the board IMU and one state per joint.
    JointStates { imu: ImuData, joints: Vec<JointState> },
    /// 0x87 from the board: the two packs' raw records (left = index 0, right = index 1); decode them with the
    /// `m20-battery` crate.
    Battery(Vec<[u8; BATTERY_PACK_LEN]>),
    /// 0xb3 to the board.
    ChargeEnable,
    /// 0xb1 to the board.
    VersionRequest,
    /// 0xb1 from the board: NUL-separated version strings.
    Version(Vec<String>),
    /// 0x8a from the board.
    EcanStatus(EcanStatus),
    /// Any other type, or a known type in an unexpected direction.
    Other { frame_type: FrameType, payload: Vec<u8> },
}

impl Message {
    /// Decode a parsed frame. Joint records whose CAN channel / id are not an M20 joint are skipped.
    pub fn decode(frame: &Frame<'_>, direction: Direction) -> Result<Self, FrameError> {
        let payload = frame.payload;
        let frame_type = frame.header.frame_type;
        let bad = || FrameError::BadLength { frame_type, len: payload.len() };
        Ok(match (frame_type, direction) {
            (FrameType::Joints, Direction::ToBoard) => {
                if !payload.len().is_multiple_of(JOINT_UNIT_LEN) {
                    return Err(bad());
                }
                Message::JointCommands(
                    JointUnit::parse_all(payload).iter().filter_map(JointCommand::from_unit).collect(),
                )
            }
            (FrameType::Joints, Direction::FromBoard) => {
                if payload.len() < IMU_LEN || !(payload.len() - IMU_LEN).is_multiple_of(JOINT_UNIT_LEN) {
                    return Err(bad());
                }
                Message::JointStates {
                    imu: ImuData::parse(payload).ok_or_else(bad)?,
                    joints: JointUnit::parse_all(&payload[IMU_LEN..])
                        .iter()
                        .filter_map(JointState::from_unit)
                        .collect(),
                }
            }
            (FrameType::Battery, Direction::FromBoard) => {
                if !payload.len().is_multiple_of(BATTERY_PACK_LEN) {
                    return Err(bad());
                }
                Message::Battery(payload.as_chunks::<BATTERY_PACK_LEN>().0.to_vec())
            }
            (FrameType::ChargeEnable, Direction::ToBoard) => Message::ChargeEnable,
            (FrameType::Version, Direction::ToBoard) => Message::VersionRequest,
            (FrameType::Version, Direction::FromBoard) => Message::Version(
                payload
                    .split(|&b| b == 0)
                    .filter(|s| !s.is_empty())
                    .map(|s| String::from_utf8_lossy(s).into_owned())
                    .collect(),
            ),
            (FrameType::EcanStatus, Direction::FromBoard) => {
                if payload.len() < 2 {
                    return Err(bad());
                }
                Message::EcanStatus(EcanStatus { hard_stop: payload[0], imu_status: payload[1] })
            }
            _ => Message::Other { frame_type, payload: payload.to_vec() },
        })
    }

    /// The frame body (header + payload) rl_deploy would send for a command `seq`. Commands past the 2730 a u16
    /// length can hold are left out.
    pub fn encode_joint_commands(seq: u8, commands: &[JointCommand]) -> Vec<u8> {
        let commands = commands.get(..MAX_UNITS).unwrap_or(commands);
        let mut payload = Vec::with_capacity(commands.len() * JOINT_UNIT_LEN);
        for command in commands {
            command.to_unit().write(&mut payload);
        }
        frame::encode_fitted(
            frame::MAGIC,
            seq,
            frame::FLAGS_JOINT_COMMAND,
            frame::CODE_JOINT_COMMAND,
            FrameType::Joints,
            &payload,
        )
    }

    /// The charge-enable request exactly as rl_deploy sends it (header magic 0).
    pub fn encode_charge_enable(seq: u8) -> Vec<u8> {
        frame::encode_fitted(0, seq, 0, frame::CODE_CONTROL, FrameType::ChargeEnable, &[])
    }

    /// A board reply carrying joint states (what the ECAN board sends; for tests and simulators).
    pub fn encode_joint_states(seq: u8, imu: &ImuData, joints: &[JointState]) -> Vec<u8> {
        let mut payload = Vec::with_capacity(IMU_LEN + joints.len() * JOINT_UNIT_LEN);
        imu.write(&mut payload);
        for joint in joints {
            joint.to_unit().write(&mut payload);
        }
        frame::encode_fitted(
            frame::MAGIC,
            seq,
            frame::FLAGS_BOARD_REPLY,
            frame::CODE_BOARD_REPLY,
            FrameType::Joints,
            &payload,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joint::JointId;

    #[test]
    fn too_many_commands_do_not_panic() {
        let one = crate::startup::zero_commands(4)[0];
        let body = Message::encode_joint_commands(0, &vec![one; 3000]);
        let frame = Frame::parse(&body).unwrap();
        assert_eq!(frame.payload.len(), MAX_UNITS * JOINT_UNIT_LEN);
        assert!(frame.checksum_ok());
        assert_eq!(one.joint, JointId::from_index(0).unwrap());
    }
}
