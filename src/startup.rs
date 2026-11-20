//! rl_deploy's startup on the motor bus, as frame bodies to send in order (renumber the sequence byte as you go), and
//! its zero-gain "sitting" command. Captured from rl_deploy v1.5.8 on 2026-11-20 and replayed on the robot: sent at
//! its pace (1 kHz joint frames) the board answers with status word 1; sent slower, the status reads 0x21 and stays
//! there until the startup runs again.
//!
//! What it is, in order: a version request (0xb1); 100 ms later a setup block (0xb2); 500 ms later six per-joint
//! frames (0x81, about 5 ms apart: ErrorReset, Disable, Enable, GetStatusWord, then SetMotorTemperature 1300 twice);
//! then zero-gain joint frames at 1 kHz with every position 0, among them 3 frames each of Disable, ErrorReset,
//! Enable and GetStatusWord (the handshake), and 4 Enable frames near the end; then the sitting command, which is what
//! a passive owner sends from then on. Three places in rl_deploy's capture are left out: two bursts of 3 charge-enable
//! frames (whoever owns the bus decides those) and 4 frames with kd 1.0 on every joint (they carry a gain), sent as
//! the sitting command instead.

use std::time::Duration;

use crate::frame::{self, FrameType};
use crate::joint::{Calibration, JointCommand, JointId, JointKind, CONTROL_MOTOR};
use crate::message::Message;

/// `code` of rl_deploy's per-joint setup frames (type 0x81).
pub const CODE_SETUP: u8 = 0x84;
/// rl_deploy's 0xb2 setup block.
pub const SETUP_BLOCK: [u8; 64] = {
    let mut block = [0u8; 64];
    block[0] = 1;
    block[1] = 1;
    block[2] = 1;
    block[3] = 5;
    block[6] = 1;
    block[7] = 1;
    block[8] = 1;
    block
};
/// The per-joint setup frames: (control word, value).
pub const SETUP_JOINT_WORDS: [(u16, u32); 6] = [(17, 0), (1, 0), (2, 0), (23, 0), (14, 1300), (14, 1300)];
/// (control word, frames) of the zero-gain 1 kHz part of the startup, as rl_deploy sends it.
pub const JOINT_PROGRAM: [(Step, usize); 12] = [
    (Step::Zero(4), 3099),
    (Step::Zero(1), 3),
    (Step::Zero(4), 8),
    (Step::Zero(17), 3),
    (Step::Zero(4), 7),
    (Step::Zero(2), 3),
    (Step::Zero(4), 7),
    (Step::Zero(23), 3),
    (Step::Zero(4), 1130),
    (Step::Pause, 3),
    (Step::Zero(4), 23),
    (Step::Sitting, 104),
];
/// After the program: 4 Enable frames, then the sitting command for good.
pub const FINAL_ENABLE_FRAMES: usize = 4;
/// The bit patterns rl_deploy writes for the URDF-zero motor positions of hip abduction, hip pitch and knee (f32
/// rounding of its own degree-to-radian conversion; the sign comes from the joint's offset and direction).
pub const SITTING_POSITION_BITS: [u32; 3] = [0x3edf66f4, 0x40125412, 0x4032b8c2];

/// One step of the 1 kHz part.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// A zero-gain joint frame with every position 0 and this control word.
    Zero(u16),
    /// rl_deploy's sitting command.
    Sitting,
    /// A 1 ms cycle with nothing sent (where rl_deploy sent charge-enable frames).
    Pause,
}

/// A frame body to send after waiting `delay` (from the previous one), or a pause.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Scheduled {
    pub delay: Duration,
    pub body: Option<Vec<u8>>,
}

/// `signed_zero`: velocity and torque as 0 x direction (-0.0 on the mirrored joints), as rl_deploy's sitting command
/// writes them; its startup frames carry plain zeros.
fn unit(joint: JointId, position: f32, word: u16, signed_zero: bool) -> JointCommand {
    let direction = if signed_zero { Calibration::m20().direction[joint.index()] } else { 1.0 };
    JointCommand {
        board_id: 0x79,
        joint,
        position,
        velocity: 0.0 * direction,
        torque: 0.0 * direction,
        kp: 0.0,
        kd: 0.0,
        control_word: word,
        data_id: 0,
    }
}

/// The zero-gain joint commands with every position 0 and this control word.
pub fn zero_commands(word: u16) -> Vec<JointCommand> {
    JointId::all().map(|joint| unit(joint, 0.0, word, false)).collect()
}

/// rl_deploy's sitting command: every gain 0, control word 4, each joint at the motor position of URDF zero.
pub fn sitting_commands() -> Vec<JointCommand> {
    let calibration = Calibration::m20();
    JointId::all()
        .map(|joint| {
            let index = joint.index();
            let position = match joint.kind {
                JointKind::Wheel => 0.0 * calibration.direction[index],
                kind => {
                    let magnitude = f32::from_bits(SITTING_POSITION_BITS[kind as usize]);
                    let sign = (-calibration.offset_rad[index] * calibration.direction[index]).signum();
                    magnitude.copysign(sign)
                }
            };
            unit(joint, position, CONTROL_MOTOR, true)
        })
        .collect()
}

/// A per-joint setup frame (0x81): 16 units carrying `word` and `value`, nothing else.
pub fn setup_joint_body(seq: u8, word: u16, value: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(16 * 24);
    for joint in JointId::all() {
        payload.push(0x79);
        payload.push(joint.leg as u8);
        payload.extend_from_slice(&joint.kind.can_id().to_le_bytes());
        payload.extend_from_slice(&value.to_le_bytes());
        payload.extend_from_slice(&[0; 12]);
        payload.extend_from_slice(&word.to_le_bytes());
        payload.extend_from_slice(&[0, 0]);
    }
    frame::encode(seq, frame::FLAGS_JOINT_COMMAND, CODE_SETUP, FrameType::Type81, &payload).expect("384 bytes")
}

/// The whole startup with sequence numbers from 0 (the caller may renumber).
pub fn sequence() -> Vec<Scheduled> {
    let mut out = Vec::new();
    let mut seq = 0u8;
    let push = |delay: Duration, body: Option<Vec<u8>>, out: &mut Vec<Scheduled>| out.push(Scheduled { delay, body });
    let next = |seq: &mut u8| {
        let s = *seq;
        *seq = seq.wrapping_add(1);
        s
    };
    let ms = Duration::from_millis;
    let s = next(&mut seq);
    push(ms(0), Some(frame::encode(s, 1, frame::CODE_CONTROL, FrameType::Version, &[]).unwrap()), &mut out);
    let s = next(&mut seq);
    push(
        ms(100),
        Some(frame::encode(s, 1, frame::CODE_CONTROL, FrameType::Other(0xb2), &SETUP_BLOCK).unwrap()),
        &mut out,
    );
    for (i, (word, value)) in SETUP_JOINT_WORDS.iter().enumerate() {
        let s = next(&mut seq);
        push(if i == 0 { ms(500) } else { ms(5) }, Some(setup_joint_body(s, *word, *value)), &mut out);
    }
    // rl_deploy's first charge-enable burst sat here: 3 cycles
    let mut delay = ms(6);
    let sitting = sitting_commands();
    for (step, count) in JOINT_PROGRAM {
        for _ in 0..count {
            let body = match step {
                Step::Zero(word) => Some(Message::encode_joint_commands(next(&mut seq), &zero_commands(word))),
                Step::Sitting => Some(Message::encode_joint_commands(next(&mut seq), &sitting)),
                Step::Pause => None,
            };
            push(delay, body, &mut out);
            delay = ms(1);
        }
    }
    for _ in 0..FINAL_ENABLE_FRAMES {
        let s = next(&mut seq);
        push(ms(1), Some(Message::encode_joint_commands(s, &zero_commands(2))), &mut out);
    }
    out
}

/// Whether a joint-command payload carries any gain (kp or kd) on any joint.
pub fn has_gain(commands: &[JointCommand]) -> bool {
    commands.iter().any(|c| c.kp != 0.0 || c.kd != 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_gain_anywhere() {
        for step in sequence() {
            let Some(body) = step.body else { continue };
            let frame = crate::Frame::parse(&body).unwrap();
            if let Ok(Message::JointCommands(commands)) = Message::decode(&frame, crate::Direction::ToBoard) {
                assert!(!has_gain(&commands));
                assert!(commands.iter().all(|c| c.torque == 0.0 && c.velocity == 0.0));
            }
            assert_ne!(frame.header.frame_type, FrameType::ChargeEnable);
        }
    }

    #[test]
    fn length_and_pace() {
        let steps = sequence();
        assert_eq!(steps.len(), 8 + JOINT_PROGRAM.iter().map(|(_, n)| n).sum::<usize>() + FINAL_ENABLE_FRAMES);
        assert!(steps[9..].iter().all(|s| s.delay == Duration::from_millis(1)));
    }
}
