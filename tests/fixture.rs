//! Decodes a real capture (the robot sitting on its dock, rl_deploy in joint damping, 2026-11-20 12:55:57 CST) and
//! checks it against rl_deploy's own once-a-second log of the same seconds (/var/opt/robot/log/system/*.csv).

use std::collections::HashMap;

use m20_ecan::frame::{CODE_BOARD_REPLY, CODE_JOINT_COMMAND, MAGIC};
use m20_ecan::joint::{CONTROL_MOTOR, JOINT_COUNT};
use m20_ecan::{decode_ethernet, tap, Calibration, Direction, Frame, FrameType, JointId, JointKind, Leg, Message};

const CAPTURE: &[u8] = include_bytes!("fixtures/sitting_docked_20261120_125557.pcapng");
const VENDOR_LOG: &str = include_str!("fixtures/vendor_log_20261120_125557.csv");

/// The vendor log's rows by their `Time s` stamp (local time, one per second), as column name -> value.
fn vendor_rows() -> HashMap<String, HashMap<String, f64>> {
    let mut lines = VENDOR_LOG.lines();
    let names: Vec<String> = lines.next().unwrap().split(',').map(|name| name.trim().to_string()).collect();
    lines
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            let row = names
                .iter()
                .zip(&fields)
                .skip(1)
                .filter_map(|(name, value)| Some((name.clone(), value.parse().ok()?)))
                .collect();
            (fields[0].to_string(), row)
        })
        .collect()
}

#[test]
fn every_frame_decodes_with_a_good_checksum() {
    let frames = tap::read_capture(CAPTURE).unwrap();
    let mut counts: HashMap<(Direction, FrameType), usize> = HashMap::new();
    for (_, direction, ethernet) in &frames {
        let frame = Frame::parse_ethernet(ethernet).unwrap();
        assert!(frame.checksum_ok(), "{:?}", frame.header);
        *counts.entry((*direction, frame.header.frame_type)).or_default() += 1;
        decode_ethernet(ethernet, *direction).unwrap();
    }
    assert_eq!(counts[&(Direction::ToBoard, FrameType::Joints)], 40);
    assert_eq!(counts[&(Direction::FromBoard, FrameType::Joints)], 40);
    assert_eq!(counts[&(Direction::FromBoard, FrameType::Battery)], 10);
    assert_eq!(counts[&(Direction::ToBoard, FrameType::ChargeEnable)], 3);
    assert_eq!(counts.len(), 4);
}

#[test]
fn headers() {
    for (_, direction, ethernet) in tap::read_capture(CAPTURE).unwrap() {
        let header = Frame::parse_ethernet(ethernet).unwrap().header;
        match (direction, header.frame_type) {
            (Direction::ToBoard, FrameType::Joints) => {
                assert_eq!((header.magic, header.code, header.flags), (MAGIC, CODE_JOINT_COMMAND, 1));
                assert_eq!(&ethernet[0..6], &m20_ecan::ethernet::BOARD_MAC);
            }
            (Direction::FromBoard, _) => assert_eq!((header.magic, header.code), (MAGIC, CODE_BOARD_REPLY)),
            (Direction::ToBoard, FrameType::ChargeEnable) => assert_eq!((header.magic, header.payload_len), (0, 0)),
            other => panic!("unexpected {other:?}"),
        }
    }
}

#[test]
fn damping_commands_hold_the_urdf_zero_pose_with_no_gain() {
    let calibration = Calibration::m20();
    let mut seen = 0;
    for (_, direction, ethernet) in tap::read_capture(CAPTURE).unwrap() {
        let Ok((_, Message::JointCommands(commands))) = decode_ethernet(ethernet, direction) else { continue };
        assert_eq!(commands.len(), JOINT_COUNT);
        for (index, command) in commands.iter().enumerate() {
            assert_eq!(command.joint, JointId::from_index(index).unwrap());
            assert_eq!(command.board_id, 0x79);
            assert_eq!(command.control_word, CONTROL_MOTOR);
            assert_eq!((command.kp, command.kd, command.torque, command.velocity), (0.0, 0.0, 0.0, 0.0));
            if command.joint.kind != JointKind::Wheel {
                let expected = calibration.motor_position(command.joint, 0.0);
                assert!(
                    (command.position - expected).abs() < 1e-3,
                    "{:?} {} vs {expected}",
                    command.joint,
                    command.position
                );
            }
        }
        seen += 1;
    }
    assert_eq!(seen, 40);
}

#[test]
fn joint_states_match_the_vendor_log_and_the_folded_pose() {
    let rows = vendor_rows();
    let row = &rows["20261120_125557"];
    let calibration = Calibration::m20();
    let mut seen = 0;
    let mut last_ids: Option<Vec<u16>> = None;
    for (timestamp_ns, direction, ethernet) in tap::read_capture(CAPTURE).unwrap() {
        let Ok((_, Message::JointStates { imu, joints })) = decode_ethernet(ethernet, direction) else { continue };
        assert!(timestamp_ns > 1_795_000_000_000_000_000);
        assert!(imu.is_zero(), "the M20's ECAN board IMU reads zero");
        assert_eq!(joints.len(), JOINT_COUNT);
        for (index, joint) in joints.iter().enumerate() {
            assert_eq!(joint.joint.index(), index);
            assert_eq!(joint.status_word, 1);
            // the log truncates to whole degrees
            assert_eq!(joint.motor_temperature_c.trunc() as f64, row[&format!("Joint {index} Motor Temperature")]);
            assert_eq!(joint.driver_temperature_c.trunc() as f64, row[&format!("Joint {index} Driver Temperature")]);
        }
        let ids: Vec<u16> = joints.iter().map(|j| j.data_id).collect();
        if let Some(last) = &last_ids {
            for (now, before) in ids.iter().zip(last) {
                assert_eq!(*now, before.wrapping_add(1), "data id counts replies");
            }
        }
        last_ids = Some(ids);
        // Folded legs: left/right hips mirror, front and hind knees fold opposite ways.
        let urdf = |leg: Leg, kind: JointKind| {
            let state = joints[JointId { leg, kind }.index()];
            calibration.urdf_position(state.joint, state.position)
        };
        for (leg, hipx_sign, fold_sign) in [
            (Leg::FrontLeft, 1.0, 1.0),
            (Leg::FrontRight, -1.0, 1.0),
            (Leg::HindLeft, 1.0, -1.0),
            (Leg::HindRight, -1.0, -1.0),
        ] {
            assert!(
                (urdf(leg, JointKind::HipX) - hipx_sign * 0.17).abs() < 0.05,
                "{leg:?} hipx {}",
                urdf(leg, JointKind::HipX)
            );
            assert!(
                (urdf(leg, JointKind::HipY) + fold_sign * 0.90).abs() < 0.05,
                "{leg:?} hipy {}",
                urdf(leg, JointKind::HipY)
            );
            assert!(
                (urdf(leg, JointKind::Knee) - fold_sign * 2.63).abs() < 0.05,
                "{leg:?} knee {}",
                urdf(leg, JointKind::Knee)
            );
        }
        seen += 1;
    }
    assert_eq!(seen, 40);
}

#[test]
fn state_replies_follow_commands() {
    let frames = tap::read_capture(CAPTURE).unwrap();
    let joints: Vec<_> = frames
        .iter()
        .filter(|(_, _, ethernet)| Frame::parse_ethernet(ethernet).unwrap().header.frame_type == FrameType::Joints)
        .collect();
    let mut gaps = Vec::new();
    for pair in joints.windows(2) {
        if pair[0].1 == Direction::ToBoard && pair[1].1 == Direction::FromBoard {
            gaps.push(pair[1].0 - pair[0].0);
        }
    }
    assert!(gaps.len() >= 39);
    assert!(gaps.iter().all(|&gap| (100_000..600_000).contains(&gap)), "{gaps:?}");
}

#[test]
fn battery_frames_carry_two_packs() {
    let mut seen = 0;
    for (_, direction, ethernet) in tap::read_capture(CAPTURE).unwrap() {
        if let Ok((_, Message::Battery(packs))) = decode_ethernet(ethernet, direction) {
            assert_eq!(packs.len(), 2);
            seen += 1;
        }
    }
    assert_eq!(seen, 10);
}

#[test]
fn re_encoding_reproduces_the_wire_bytes() {
    for (_, direction, ethernet) in tap::read_capture(CAPTURE).unwrap() {
        let frame = Frame::parse_ethernet(ethernet).unwrap();
        let body = &ethernet[14..14 + 10 + frame.payload.len()];
        let encoded = match decode_ethernet(ethernet, direction).unwrap().1 {
            Message::JointCommands(commands) => Message::encode_joint_commands(frame.header.seq, &commands),
            Message::JointStates { imu, joints } => Message::encode_joint_states(frame.header.seq, &imu, &joints),
            Message::ChargeEnable => Message::encode_charge_enable(frame.header.seq),
            _ => continue,
        };
        assert_eq!(encoded, body);
    }
}
