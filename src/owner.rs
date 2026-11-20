//! The motor-bus owner's decisions, without a socket: the startup, then one frame per 1 ms tick from whatever
//! commands the caller has (none = rl_deploy's sitting command), with gains and feed-forward torque refused unless
//! `allow_gains`, and charge-enable requests sent in place of passive commands the way rl_deploy sends them.

use std::collections::VecDeque;

use crate::joint::JointCommand;
use crate::message::Message;
use crate::startup::{self, is_active, is_complete, Scheduled};

/// Frames per charge-enable request. matches vendor: rl_deploy sends 3, one per cycle in place of joint commands
/// (`ecan::Motor::BatteryChargeEnable`, seen in every docked capture).
pub const CHARGE_ENABLE_FRAMES: usize = 3;

/// What the owner may send.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OwnerConfig {
    /// Let commands with kp / kd / feed-forward torque through. Off: any such command is replaced by the sitting
    /// command.
    pub allow_gains: bool,
}

/// One tick: wait `delay` after the previous one, then send `body` (none: nothing this tick).
pub type Tick = Scheduled;

pub struct Owner {
    config: OwnerConfig,
    startup: VecDeque<Scheduled>,
    pending: VecDeque<Vec<u8>>,
    seq: u8,
    /// Ticks whose commands were replaced by the sitting command: carrying a gain or torque while gains were not
    /// allowed, or not one command per joint.
    pub refused: u64,
    sitting: Vec<JointCommand>,
}

impl Owner {
    pub fn new(config: OwnerConfig) -> Self {
        Owner {
            config,
            startup: startup::sequence().into(),
            pending: VecDeque::new(),
            seq: 0,
            refused: 0,
            sitting: startup::sitting_commands(),
        }
    }

    pub fn in_startup(&self) -> bool {
        !self.startup.is_empty()
    }

    pub fn config(&self) -> OwnerConfig {
        self.config
    }

    /// Queue one charge-enable request (3 frames, one per tick in place of passive commands).
    pub fn queue_charge_enable(&mut self) {
        for _ in 0..CHARGE_ENABLE_FRAMES {
            self.pending.push_back(Message::encode_charge_enable(0));
        }
    }

    fn numbered(&mut self, mut body: Vec<u8>) -> Vec<u8> {
        if let Some(seq) = body.get_mut(2) {
            *seq = self.seq;
        }
        self.seq = self.seq.wrapping_add(1);
        body
    }

    /// The next tick. `commands` are ignored during the startup.
    pub fn next(&mut self, commands: Option<&[JointCommand]>) -> Tick {
        if let Some(step) = self.startup.pop_front() {
            let body = step.body.map(|b| self.numbered(b));
            return Tick { delay: step.delay, body };
        }
        let mut chosen = commands.filter(|c| !c.is_empty()).unwrap_or(&self.sitting).to_vec();
        if !is_complete(&chosen) || (is_active(&chosen) && !self.config.allow_gains) {
            self.refused += 1;
            chosen = self.sitting.clone();
        }
        let body = if is_active(&chosen) {
            // never swap out a command that holds the legs; the rule asks again on the next report
            self.pending.clear();
            Message::encode_joint_commands(0, &chosen)
        } else {
            match self.pending.pop_front() {
                Some(charge_enable) => charge_enable,
                None => Message::encode_joint_commands(0, &chosen),
            }
        };
        let body = self.numbered(body);
        Tick { delay: startup::TICK, body: Some(body) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::FrameType;
    use crate::startup::has_gain;
    use crate::{Direction, Frame};

    fn gains_in(body: &[u8]) -> bool {
        let frame = Frame::parse(body).unwrap();
        match Message::decode(&frame, Direction::ToBoard) {
            Ok(Message::JointCommands(c)) => has_gain(&c),
            _ => false,
        }
    }

    fn walking() -> Vec<JointCommand> {
        startup::sitting_commands()
            .into_iter()
            .map(|mut c| {
                c.kp = 60.0;
                c.kd = 1.5;
                c
            })
            .collect()
    }

    fn past_startup(owner: &mut Owner) {
        while owner.in_startup() {
            owner.next(Some(&walking()));
        }
    }

    #[test]
    fn never_sends_a_gain_unless_allowed() {
        let mut owner = Owner::new(OwnerConfig::default());
        let mut sent = 0;
        for _ in 0..(startup::sequence().len() + 2000) {
            if let Some(body) = owner.next(Some(&walking())).body {
                assert!(!gains_in(&body));
                sent += 1;
            }
        }
        assert!(sent > 6000);
        assert_eq!(owner.refused, 2000);
    }

    #[test]
    fn allowed_gains_pass_after_the_startup_only() {
        let mut owner = Owner::new(OwnerConfig { allow_gains: true });
        let startup_len = startup::sequence().len();
        for i in 0..startup_len + 10 {
            let body = owner.next(Some(&walking())).body;
            if let Some(body) = body {
                assert_eq!(gains_in(&body), i >= startup_len, "tick {i}");
            }
        }
        assert_eq!(owner.refused, 0);
    }

    #[test]
    fn charge_enable_replaces_passive_ticks_only() {
        let mut owner = Owner::new(OwnerConfig { allow_gains: true });
        past_startup(&mut owner);
        owner.queue_charge_enable();
        let types: Vec<FrameType> =
            (0..5).map(|_| Frame::parse(&owner.next(None).body.unwrap()).unwrap().header.frame_type).collect();
        assert_eq!(types[..3], [FrameType::ChargeEnable; 3]);
        assert_eq!(types[3..], [FrameType::Joints; 2]);
        owner.queue_charge_enable();
        assert!(gains_in(&owner.next(Some(&walking())).body.unwrap()));
        assert_eq!(Frame::parse(&owner.next(None).body.unwrap()).unwrap().header.frame_type, FrameType::Joints);
    }

    #[test]
    fn torque_without_gains_is_refused_unless_allowed() {
        let pushing: Vec<JointCommand> = startup::sitting_commands()
            .into_iter()
            .map(|mut c| {
                c.torque = 5.0;
                c
            })
            .collect();
        let mut owner = Owner::new(OwnerConfig::default());
        past_startup(&mut owner);
        let refused = owner.refused;
        let body = owner.next(Some(&pushing)).body.unwrap();
        let Ok(Message::JointCommands(sent)) = Message::decode(&Frame::parse(&body).unwrap(), Direction::ToBoard)
        else {
            panic!("not joint commands")
        };
        assert!(sent.iter().all(|c| c.torque == 0.0));
        assert_eq!(owner.refused, refused + 1);
        // allowed: it passes, and a queued charge-enable does not replace it
        let mut owner = Owner::new(OwnerConfig { allow_gains: true });
        past_startup(&mut owner);
        owner.queue_charge_enable();
        let body = owner.next(Some(&pushing)).body.unwrap();
        assert_eq!(Frame::parse(&body).unwrap().header.frame_type, FrameType::Joints);
        assert_eq!(owner.refused, 0);
    }

    #[test]
    fn incomplete_or_oversized_commands_are_refused() {
        let mut owner = Owner::new(OwnerConfig { allow_gains: true });
        past_startup(&mut owner);
        let refused = owner.refused;
        let sitting = Message::encode_joint_commands(0, &startup::sitting_commands());
        let short = &walking()[..15];
        let doubled: Vec<JointCommand> = walking().into_iter().chain(walking()).collect();
        let mut repeated = walking();
        repeated[1] = repeated[0];
        for commands in [short, &doubled[..], &vec![walking()[0]; 3000][..], &repeated[..]] {
            let body = owner.next(Some(commands)).body.unwrap();
            assert_eq!(body[3..], sitting[3..]);
        }
        assert_eq!(owner.refused, refused + 4);
    }

    #[test]
    fn sequence_numbers_run_on() {
        let mut owner = Owner::new(OwnerConfig::default());
        let seqs: Vec<u8> = (0..600).filter_map(|_| owner.next(None).body).map(|b| b[2]).collect();
        for pair in seqs.windows(2) {
            assert_eq!(pair[1], pair[0].wrapping_add(1));
        }
    }
}
