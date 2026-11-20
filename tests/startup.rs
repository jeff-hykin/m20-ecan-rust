//! The generated startup against rl_deploy's own (captured 2026-11-20 15:37 CST), byte for byte apart from the
//! sequence numbers, with its two charge-enable bursts left out and its 4 kd-1.0 frames sent as the sitting command.

use m20_ecan::startup::{self, sitting_commands};
use m20_ecan::Message;

const CAPTURE: &str = include_str!("fixtures/rl_deploy_startup_20261120_1537.txt");

fn hex(s: &str) -> Vec<u8> {
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

fn unseq(mut body: Vec<u8>) -> Vec<u8> {
    body[2] = 0;
    body
}

#[test]
fn matches_rl_deploys_startup() {
    let mut captured = Vec::new();
    for line in CAPTURE.lines().filter(|l| !l.starts_with('#')) {
        let parts: Vec<&str> = line.split_whitespace().collect();
        let (count, body) = (parts[1].parse::<usize>().unwrap(), hex(parts[3]));
        captured.extend(std::iter::repeat_n(body, count));
    }
    assert_eq!(captured.len(), 4420);
    let sitting = unseq(Message::encode_joint_commands(0, &sitting_commands()));
    let expected: Vec<Vec<u8>> = captured
        .iter()
        .filter(|b| b[5] != 0xb3)
        .map(|b| {
            let frame = m20_ecan::Frame::parse(b).unwrap();
            match Message::decode(&frame, m20_ecan::Direction::ToBoard) {
                Ok(Message::JointCommands(c)) if startup::has_gain(&c) => sitting.clone(),
                _ => b.clone(),
            }
        })
        .collect();
    let generated: Vec<Vec<u8>> = startup::sequence().into_iter().filter_map(|s| s.body).map(unseq).collect();
    assert_eq!(generated.len(), 4402);
    for (i, (ours, theirs)) in generated.iter().zip(&expected).enumerate() {
        assert_eq!(ours, theirs, "frame {i}");
    }
    // what follows in rl_deploy's capture is the sitting command, the passive owner's steady state
    assert!(expected[generated.len()..].iter().all(|b| *b == sitting));
    // the sitting command is the one rl_deploy sends while lying on the dock (the earlier capture)
    let docked = m20_ecan::tap::read_capture(include_bytes!("fixtures/sitting_docked_20261120_125557.pcapng")).unwrap();
    let (_, _, frame) = docked.iter().find(|(_, d, f)| *d == m20_ecan::Direction::ToBoard && f[19] == 0x89).unwrap();
    assert_eq!(unseq(frame[14..14 + 394].to_vec()), sitting);
}
