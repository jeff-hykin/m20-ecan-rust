//! Joint records: the 24-byte `ecan::JointUnit` (vendor `data_structures.h`, `#pragma pack(2)`) in both directions,
//! named by leg and joint, and the vendor's motor-to-URDF calibration.
//!
//! ```text
//! 0  u8  board id (0x79 on this robot)
//! 1  u8  CAN channel = leg (0 FL, 1 FR, 2 HL, 3 HR)
//! 2  u16 CAN id = joint (2 hip abduction, 3 hip pitch, 4 knee, 5 wheel)
//! 4  f32 position (rad, motor side)   8 f32 velocity (rad/s)   12 f32 torque (N m)
//!    (wheels: position is the accumulated angle, many turns, not wrapped; it is the integral of the velocity)
//! 16 i16 command: kp x 10 | state: motor temperature x 100 (deg C)
//! 18 i16 command: kd x 10 | state: driver temperature x 100 (deg C)
//! 20 u16 command: control word (4 = motor control) | state: status word
//! 22 u16 data id (state: a per-joint counter, +1 per reply)
//! ```

use std::f32::consts::PI;

/// Bytes per joint record.
pub const JOINT_UNIT_LEN: usize = 24;
/// Joints on the M20 (4 legs x 4).
pub const JOINT_COUNT: usize = 16;
/// `control_word` of a normal 1 kHz joint command (`ecan::kIndexMotorControl`).
pub const CONTROL_MOTOR: u16 = 4;
/// `control_word` rl_deploy sends for one cycle, all gains zero, when it leaves passive / lying for a stand-up.
pub const CONTROL_ENABLE: u16 = 2;
/// `status_word` of every joint throughout both recorded sessions (sitting, stand-up, walking, docking, lying).
pub const STATUS_OK: u16 = 1;
/// Scale of the kp / kd fields on the wire (ECAN2DDS::Handler multiplies by 10 in f32 and truncates toward zero).
pub const GAIN_SCALE: f32 = 10.0;
/// Scale of the temperature fields on the wire (ECAN2DDS::handleMotorData divides by 100).
pub const TEMPERATURE_SCALE: f32 = 100.0;

/// The motor command words of `ecan::CommandIndex`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum CommandIndex {
    Disable = 1,
    Enable = 2,
    Calibration = 3,
    MotorControl = 4,
    MotorReset = 5,
    SetHome = 6,
    SetGear = 7,
    SetId = 8,
    SetCanTimeout = 9,
    SetBandwidth = 10,
    SetLimitCurrent = 11,
    SetUnderVoltage = 12,
    SetOverVoltage = 13,
    SetMotorTemperature = 14,
    SetDriverTemperature = 15,
    SaveConfig = 16,
    ErrorReset = 17,
    WriteAppBackStart = 18,
    WriteAppBack = 19,
    CheckAppBack = 20,
    DfuStart = 21,
    GetVersion = 22,
    GetStatusWord = 23,
    GetConfig = 24,
    OutEncoderCalibration = 25,
    GetOutEncoder = 26,
    CalibReport = 31,
}

/// A leg, numbered by its CAN channel. basic_server calls "left" the battery side and "back" the hard e-stop side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Leg {
    FrontLeft = 0,
    FrontRight = 1,
    HindLeft = 2,
    HindRight = 3,
}

impl Leg {
    pub const ALL: [Leg; 4] = [Leg::FrontLeft, Leg::FrontRight, Leg::HindLeft, Leg::HindRight];

    pub fn from_can_channel(channel: u8) -> Option<Self> {
        Self::ALL.get(channel as usize).copied()
    }

    /// The prefix the vendor URDF/MJCF uses: fl, fr, hl, hr.
    pub fn urdf_prefix(self) -> &'static str {
        match self {
            Leg::FrontLeft => "fl",
            Leg::FrontRight => "fr",
            Leg::HindLeft => "hl",
            Leg::HindRight => "hr",
        }
    }
}

/// A joint of a leg, numbered by CAN id - 2.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum JointKind {
    /// Hip abduction (CAN id 2).
    HipX = 0,
    /// Hip pitch (CAN id 3).
    HipY = 1,
    /// Knee (CAN id 4).
    Knee = 2,
    /// Wheel (CAN id 5).
    Wheel = 3,
}

impl JointKind {
    pub const ALL: [JointKind; 4] = [JointKind::HipX, JointKind::HipY, JointKind::Knee, JointKind::Wheel];

    pub fn from_can_id(can_id: u16) -> Option<Self> {
        can_id.checked_sub(2).and_then(|index| Self::ALL.get(index as usize).copied())
    }

    pub fn can_id(self) -> u16 {
        self as u16 + 2
    }

    pub fn urdf_suffix(self) -> &'static str {
        match self {
            JointKind::HipX => "hipx",
            JointKind::HipY => "hipy",
            JointKind::Knee => "knee",
            JointKind::Wheel => "wheel",
        }
    }
}

/// One of the 16 joints.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct JointId {
    pub leg: Leg,
    pub kind: JointKind,
}

impl JointId {
    pub fn from_can(channel: u8, can_id: u16) -> Option<Self> {
        Some(JointId { leg: Leg::from_can_channel(channel)?, kind: JointKind::from_can_id(can_id)? })
    }

    /// rl_deploy's joint index: leg x 4 + joint.
    pub fn index(self) -> usize {
        self.leg as usize * 4 + self.kind as usize
    }

    pub fn from_index(index: usize) -> Option<Self> {
        Some(JointId { leg: *Leg::ALL.get(index / 4)?, kind: *JointKind::ALL.get(index % 4)? })
    }

    /// All 16 in rl_deploy's order.
    pub fn all() -> impl Iterator<Item = JointId> {
        (0..JOINT_COUNT).filter_map(JointId::from_index)
    }

    /// The vendor URDF/MJCF joint name, e.g. `fl_hipx_joint`.
    pub fn urdf_name(self) -> String {
        format!("{}_{}_joint", self.leg.urdf_prefix(), self.kind.urdf_suffix())
    }
}

/// A raw 24-byte record, fields exactly as on the wire.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointUnit {
    pub board_id: u8,
    pub can_channel: u8,
    pub can_id: u16,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    /// kp x 10 (command) or motor temperature x 100 (state).
    pub word_a: i16,
    /// kd x 10 (command) or driver temperature x 100 (state).
    pub word_b: i16,
    /// Control word (command) or status word (state).
    pub control_or_status: u16,
    pub data_id: u16,
}

impl JointUnit {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let b = bytes.get(..JOINT_UNIT_LEN)?;
        let f = |at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        Some(JointUnit {
            board_id: b[0],
            can_channel: b[1],
            can_id: u16::from_le_bytes([b[2], b[3]]),
            position: f(4),
            velocity: f(8),
            torque: f(12),
            word_a: i16::from_le_bytes([b[16], b[17]]),
            word_b: i16::from_le_bytes([b[18], b[19]]),
            control_or_status: u16::from_le_bytes([b[20], b[21]]),
            data_id: u16::from_le_bytes([b[22], b[23]]),
        })
    }

    pub fn write(&self, out: &mut Vec<u8>) {
        out.push(self.board_id);
        out.push(self.can_channel);
        out.extend_from_slice(&self.can_id.to_le_bytes());
        out.extend_from_slice(&self.position.to_le_bytes());
        out.extend_from_slice(&self.velocity.to_le_bytes());
        out.extend_from_slice(&self.torque.to_le_bytes());
        out.extend_from_slice(&self.word_a.to_le_bytes());
        out.extend_from_slice(&self.word_b.to_le_bytes());
        out.extend_from_slice(&self.control_or_status.to_le_bytes());
        out.extend_from_slice(&self.data_id.to_le_bytes());
    }

    pub fn joint(&self) -> Option<JointId> {
        JointId::from_can(self.can_channel, self.can_id)
    }

    /// Every whole record in a payload.
    pub fn parse_all(payload: &[u8]) -> Vec<JointUnit> {
        payload.as_chunks::<JOINT_UNIT_LEN>().0.iter().filter_map(|unit| JointUnit::parse(unit)).collect()
    }
}

/// One joint's command (host -> board), in motor units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointCommand {
    pub board_id: u8,
    pub joint: JointId,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub kp: f32,
    pub kd: f32,
    pub control_word: u16,
    pub data_id: u16,
}

impl JointCommand {
    pub fn from_unit(unit: &JointUnit) -> Option<Self> {
        Some(JointCommand {
            board_id: unit.board_id,
            joint: unit.joint()?,
            position: unit.position,
            velocity: unit.velocity,
            torque: unit.torque,
            kp: unit.word_a as f32 / GAIN_SCALE,
            kd: unit.word_b as f32 / GAIN_SCALE,
            control_word: unit.control_or_status,
            data_id: unit.data_id,
        })
    }

    /// The wire record. kp and kd go to 0.1 steps the way rl_deploy does it: f32 times 10, truncated toward zero
    /// (`fcvtzs`), so 1.5 x 1.3 = 1.9499999 goes out as 19 (seen on the wire during NaviCharge's start-up gains).
    pub fn to_unit(&self) -> JointUnit {
        JointUnit {
            board_id: self.board_id,
            can_channel: self.joint.leg as u8,
            can_id: self.joint.kind.can_id(),
            position: self.position,
            velocity: self.velocity,
            torque: self.torque,
            word_a: (self.kp * GAIN_SCALE) as i16,
            word_b: (self.kd * GAIN_SCALE) as i16,
            control_or_status: self.control_word,
            data_id: self.data_id,
        }
    }
}

/// One joint's measured state (board -> host), in motor units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    pub board_id: u8,
    pub joint: JointId,
    /// rad, motor side (see [`Calibration`] for the URDF angle)
    pub position: f32,
    /// rad/s, motor side
    pub velocity: f32,
    /// N m, motor side
    pub torque: f32,
    pub motor_temperature_c: f32,
    pub driver_temperature_c: f32,
    pub status_word: u16,
    /// +1 per reply, per joint
    pub data_id: u16,
}

impl JointState {
    pub fn from_unit(unit: &JointUnit) -> Option<Self> {
        Some(JointState {
            board_id: unit.board_id,
            joint: unit.joint()?,
            position: unit.position,
            velocity: unit.velocity,
            torque: unit.torque,
            motor_temperature_c: unit.word_a as f32 / TEMPERATURE_SCALE,
            driver_temperature_c: unit.word_b as f32 / TEMPERATURE_SCALE,
            status_word: unit.control_or_status,
            data_id: unit.data_id,
        })
    }

    pub fn to_unit(&self) -> JointUnit {
        JointUnit {
            board_id: self.board_id,
            can_channel: self.joint.leg as u8,
            can_id: self.joint.kind.can_id(),
            position: self.position,
            velocity: self.velocity,
            torque: self.torque,
            word_a: (self.motor_temperature_c * TEMPERATURE_SCALE).round() as i16,
            word_b: (self.driver_temperature_c * TEMPERATURE_SCALE).round() as i16,
            control_or_status: self.status_word,
            data_id: self.data_id,
        }
    }
}

/// Checked on the robot (2026-11-20 moving session): with this calibration the stand-up target is exactly
/// (hipx 0, hipy -/+0.708, knee +/-1.415) on every leg and the standing pose puts all four wheels 0.40-0.44 m below
/// the body, under their hips; the measured torque follows `kp (q_cmd - q) + kd (dq_cmd - dq) + tau_ff` with slope
/// 0.93-1.0 (r 0.94-0.999) on all 16 joints, so torque has the same sign as position; a forward command turns all
/// four wheels negative (the MJCF wheel axis is -y), about -8.9 rad/s per commanded m/s (pure rolling at the
/// 0.09 m radius would be -11.1), and a left turn turns the right wheels more negative than the left.
///
/// Motor side to URDF side, per joint: `urdf = motor * direction + offset`. Velocity and torque take the direction
/// only. Hip pitch and knee encoders are single-turn, so their URDF angle is wrapped into (-pi, pi] (the vendor picks
/// the 360 degree branch from the pose at startup, which lands in the same range); wheels are not wrapped.
#[derive(Clone, Debug, PartialEq)]
pub struct Calibration {
    pub direction: [f32; JOINT_COUNT],
    pub offset_rad: [f32; JOINT_COUNT],
}

impl Calibration {
    /// The vendor's values (DeepRoboticsLab/sdk_deploy `M20_sdk_deploy/.../m20_interface.hpp`, `init_pos_offset` in
    /// degrees and `joint_dir`), which also match rl_deploy's DDS joint handler.
    pub fn m20() -> Self {
        const OFFSET_DEG: [f32; JOINT_COUNT] =
            [-25., -131., 160., 0., 25., -131., 160., 0., -25., 131., -160., 0., 25., 131., -160., 0.];
        const DIRECTION: [f32; JOINT_COUNT] = [1., 1., -1., 1., 1., -1., 1., -1., -1., 1., -1., 1., -1., -1., 1., -1.];
        Calibration { direction: DIRECTION, offset_rad: OFFSET_DEG.map(|deg| deg.to_radians()) }
    }

    /// Motor position to URDF position.
    pub fn urdf_position(&self, joint: JointId, motor: f32) -> f32 {
        let index = joint.index();
        let urdf = motor * self.direction[index] + self.offset_rad[index];
        if joint.kind == JointKind::Wheel {
            urdf
        } else {
            wrap_pi(urdf)
        }
    }

    /// URDF position to motor position (what a command carries).
    pub fn motor_position(&self, joint: JointId, urdf: f32) -> f32 {
        let index = joint.index();
        (urdf - self.offset_rad[index]) * self.direction[index]
    }

    /// Motor velocity or torque to the URDF side (and back: the direction is +-1).
    pub fn urdf_rate(&self, joint: JointId, motor: f32) -> f32 {
        motor * self.direction[joint.index()]
    }
}

/// Into (-pi, pi].
pub fn wrap_pi(angle: f32) -> f32 {
    let wrapped = (angle + PI).rem_euclid(2.0 * PI) - PI;
    if wrapped <= -PI {
        wrapped + 2.0 * PI
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_and_indices() {
        let names: Vec<String> = JointId::all().map(JointId::urdf_name).collect();
        assert_eq!(names[0], "fl_hipx_joint");
        assert_eq!(names[7], "fr_wheel_joint");
        assert_eq!(names[14], "hr_knee_joint");
        for (index, joint) in JointId::all().enumerate() {
            assert_eq!(joint.index(), index);
            assert_eq!(JointId::from_can(joint.leg as u8, joint.kind.can_id()), Some(joint));
        }
        assert_eq!(JointId::from_can(4, 2), None);
        assert_eq!(JointId::from_can(0, 6), None);
        assert_eq!(JointId::from_can(0, 1), None);
    }

    #[test]
    fn unit_round_trip() {
        let unit = JointUnit {
            board_id: 0x79,
            can_channel: 2,
            can_id: 4,
            position: -0.25,
            velocity: 1.5,
            torque: -3.0,
            word_a: 600,
            word_b: -15,
            control_or_status: 4,
            data_id: 65535,
        };
        let mut bytes = Vec::new();
        unit.write(&mut bytes);
        assert_eq!(bytes.len(), JOINT_UNIT_LEN);
        assert_eq!(JointUnit::parse(&bytes), Some(unit));
        let command = JointCommand::from_unit(&unit).unwrap();
        assert_eq!(command.kp, 60.0);
        assert_eq!(command.kd, -1.5);
        assert_eq!(command.to_unit(), unit);
    }

    #[test]
    fn every_wire_gain_survives_a_round_trip() {
        for raw in i16::MIN..=i16::MAX {
            let unit = JointUnit {
                board_id: 0,
                can_channel: 0,
                can_id: 2,
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                word_a: raw,
                word_b: raw,
                control_or_status: 4,
                data_id: 0,
            };
            assert_eq!(JointCommand::from_unit(&unit).unwrap().to_unit(), unit, "{raw}");
        }
    }

    #[test]
    fn gains_truncate_like_rl_deploy() {
        let mut command = JointCommand::from_unit(
            &JointUnit::parse(&[0; JOINT_UNIT_LEN])
                .map(|mut u| {
                    u.can_id = 2;
                    u
                })
                .unwrap(),
        )
        .unwrap();
        command.kp = 60.0f32 * 1.3;
        command.kd = 1.5f32 * 1.3;
        let unit = command.to_unit();
        assert_eq!((unit.word_a, unit.word_b), (780, 19));
    }

    #[test]
    fn calibration_round_trip() {
        let calibration = Calibration::m20();
        for joint in JointId::all() {
            for urdf in [-1.0f32, 0.0, 0.7, 2.5] {
                let motor = calibration.motor_position(joint, urdf);
                assert!((calibration.urdf_position(joint, motor) - urdf).abs() < 1e-5, "{joint:?} {urdf}");
            }
        }
    }

    #[test]
    fn wrap() {
        assert!((wrap_pi(3.5) - (3.5 - 2.0 * PI)).abs() < 1e-6);
        assert!((wrap_pi(-3.5) - (2.0 * PI - 3.5)).abs() < 1e-6);
        assert_eq!(wrap_pi(PI), PI);
        assert_eq!(wrap_pi(0.25), 0.25);
    }
}
