//! The motor-bus owner's decisions, without a socket: the startup, then one frame per 1 ms tick from whatever
//! commands the caller has (none = rl_deploy's sitting command), with gains refused unless `allow_gains`, and
//! charge-enable requests sent in place of passive commands the way rl_deploy sends them.

use std::collections::VecDeque;
use std::time::Duration;

use crate::joint::JointCommand;
use crate::message::Message;
use crate::startup::{self, has_gain, Scheduled};

/// What the owner may send.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OwnerConfig {
    /// Let commands with kp / kd through. Off: any command carrying a gain is replaced by the sitting command.
    pub allow_gains: bool,
}

/// One tick: wait `delay` after the previous one, then send `body` (none: nothing this tick).
pub type Tick = Scheduled;

pub struct Owner {
    config: OwnerConfig,
    startup: VecDeque<Scheduled>,
    pending: VecDeque<Vec<u8>>,
    seq: u8,
    /// Ticks whose commands carried a gain while gains were not allowed.
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
        for _ in 0..3 {
            self.pending.push_back(Message::encode_charge_enable(0));
        }
    }

    fn numbered(&mut self, mut body: Vec<u8>) -> Vec<u8> {
        body[2] = self.seq;
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
        if has_gain(&chosen) && !self.config.allow_gains {
            self.refused += 1;
            chosen = self.sitting.clone();
        }
        let body = match (self.pending.front(), has_gain(&chosen)) {
            (Some(_), false) => self.pending.pop_front().unwrap(),
            (Some(_), true) => {
                // never swap out a command that holds the legs; the rule asks again on the next report
                self.pending.clear();
                Message::encode_joint_commands(0, &chosen)
            }
            (None, _) => Message::encode_joint_commands(0, &chosen),
        };
        let body = self.numbered(body);
        Tick { delay: Duration::from_millis(1), body: Some(body) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::FrameType;
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
    fn sequence_numbers_run_on() {
        let mut owner = Owner::new(OwnerConfig::default());
        let seqs: Vec<u8> = (0..600).filter_map(|_| owner.next(None).body).map(|b| b[2]).collect();
        for pair in seqs.windows(2) {
            assert_eq!(pair[1], pair[0].wrapping_add(1));
        }
    }
}
