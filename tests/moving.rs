//! A cut of the 2026-11-20 moving session (undock, stand, drive, lie down, stand, dock, lie down; robot on its dock
//! before and after): one command/reply pair per commanded-gain phase and moment, the docking charge-enable bursts,
//! and battery frames on the dock, off it, and charging again. `cut_moving_session.py` made it.

use m20_ecan::frame::{CODE_CONTROL, HEADER_LEN};
use m20_ecan::joint::{CONTROL_ENABLE, CONTROL_MOTOR, STATUS_OK};
use m20_ecan::{decode_ethernet, tap, Calibration, Direction, FrameType, JointKind, Message};

const CAPTURE: &[u8] = include_bytes!("fixtures/moving_20261120_141603.pcapng");

/// (legs kp, legs kd, wheels kd, control word) of a command frame; panics if legs disagree or a wheel has kp.
fn gains(commands: &[m20_ecan::JointCommand]) -> (f32, f32, f32, u16) {
    let legs: Vec<_> = commands.iter().filter(|c| c.joint.kind != JointKind::Wheel).collect();
    let wheels: Vec<_> = commands.iter().filter(|c| c.joint.kind == JointKind::Wheel).collect();
    assert!(legs.iter().all(|c| (c.kp, c.kd) == (legs[0].kp, legs[0].kd)));
    assert!(wheels.iter().all(|c| c.kp == 0.0 && c.kd == wheels[0].kd));
    assert!(commands.iter().all(|c| c.control_word == commands[0].control_word));
    (legs[0].kp, legs[0].kd, wheels[0].kd, commands[0].control_word)
}

#[test]
fn every_commanded_gain_phase() {
    let mut phases = Vec::new();
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        if let Ok((_, Message::JointCommands(commands))) = decode_ethernet(frame, direction) {
            let g = gains(&commands);
            if phases.last() != Some(&g) {
                phases.push(g);
            }
        }
    }
    let expected = [
        (0.0, 0.0, 0.0, CONTROL_MOTOR),   // passive, sitting on the dock
        (0.0, 0.0, 0.0, CONTROL_ENABLE),  // one cycle before each stand-up
        (200.0, 4.0, 2.0, CONTROL_MOTOR), // stand-up
        (78.0, 1.9, 0.6, CONTROL_MOTOR),  // NaviCharge (0x3001), x1.3 on the legs for its first 29 steps
        (60.0, 1.5, 0.6, CONTROL_MOTOR),  // NaviCharge
        (80.0, 3.0, 0.6, CONTROL_MOTOR),  // 0x3003 walking
        (80.0, 6.0, 0.6, CONTROL_MOTOR),  // 0x1001
        (80.0, 3.0, 0.6, CONTROL_MOTOR),
        (300.0, 4.5, 2.0, CONTROL_MOTOR), // lie-down, 2 s
        (0.0, 4.5, 2.0, CONTROL_MOTOR),   // lie-down damping, 2 s
        (60.0, 1.5, 1.0, CONTROL_MOTOR),  // stand-in (six-axis / fine-tune stand) policies
        (80.0, 3.0, 0.6, CONTROL_MOTOR),
        (0.0, 0.0, 0.0, CONTROL_MOTOR), // passive again
    ];
    let mut cursor = 0;
    for phase in &expected {
        let found = phases[cursor..].iter().position(|p| p == phase);
        assert!(found.is_some(), "missing {phase:?} after #{cursor} in {phases:?}");
        cursor += found.unwrap() + 1;
    }
}

#[test]
fn stand_up_target_is_symmetric_in_urdf_angles() {
    let calibration = Calibration::m20();
    let mut checked = 0;
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        let Ok((_, Message::JointCommands(commands))) = decode_ethernet(frame, direction) else { continue };
        if gains(&commands).0 != 200.0 {
            continue;
        }
        for c in commands.iter().filter(|c| c.joint.kind != JointKind::Wheel) {
            assert!(c.kp == 200.0 && calibration.urdf_position(c.joint, c.position).is_finite());
        }
        checked += 1;
    }
    assert!(checked > 0);
    // the end of the stand-up ramp (14:16:34.6): same pose on every leg, mirrored front to hind
    let (_, direction, frame) = tap::read_capture(CAPTURE)
        .unwrap()
        .into_iter()
        .find(|(t, d, _)| *d == Direction::ToBoard && *t >= 1_795_155_394_600_000_000)
        .unwrap();
    let Ok((_, Message::JointCommands(commands))) = decode_ethernet(frame, direction) else { panic!() };
    for c in &commands {
        let urdf = calibration.urdf_position(c.joint, c.position);
        let hind = matches!(c.joint.leg, m20_ecan::Leg::HindLeft | m20_ecan::Leg::HindRight);
        let want = match c.joint.kind {
            JointKind::HipX => 0.0,
            JointKind::HipY => {
                if hind {
                    0.708
                } else {
                    -0.708
                }
            }
            JointKind::Knee => {
                if hind {
                    -1.415
                } else {
                    1.415
                }
            }
            JointKind::Wheel => continue,
        };
        assert!((urdf - want).abs() < 0.01, "{:?} {urdf} vs {want}", c.joint);
    }
}

#[test]
fn status_words_stay_ok_through_motion() {
    let mut replies = 0;
    for (_, direction, frame) in tap::read_capture(CAPTURE).unwrap() {
        if let Ok((_, Message::JointStates { joints, .. })) = decode_ethernet(frame, direction) {
            assert!(joints.iter().all(|j| j.status_word == STATUS_OK));
            replies += 1;
        }
    }
    assert!(replies >= 30);
}

#[test]
fn charge_enable_bursts_share_the_command_sequence() {
    let frames = tap::read_capture(CAPTURE).unwrap();
    let enables: Vec<_> = frames
        .iter()
        .filter_map(|(t, d, frame)| match decode_ethernet(frame, *d) {
            Ok((header, Message::ChargeEnable)) => Some((*t, header, *frame)),
            _ => None,
        })
        .collect();
    assert_eq!(enables.len(), 12);
    for burst in enables.chunks(3) {
        for (i, (t, header, frame)) in burst.iter().enumerate() {
            assert_eq!((header.magic, header.flags, header.code), (0, 0, CODE_CONTROL));
            assert_eq!(header.frame_type, FrameType::ChargeEnable);
            assert_eq!(header.seq, burst[0].1.seq.wrapping_add(i as u8));
            assert_eq!(frame.len(), 14 + HEADER_LEN);
            assert_eq!(&frame[14..], Message::encode_charge_enable(header.seq).as_slice());
            assert!((t - burst[0].0).abs_diff(i as u64 * 1_000_000) < 100_000, "1 ms apart");
        }
    }
}
