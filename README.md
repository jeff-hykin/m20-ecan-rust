# m20-ecan

The Deep Robotics M20 (Lynx) motor bus ("ECAN") in Rust: the Ethernet protocol (ethertype `0x4452`) between the
vendor walking controller `rl_deploy` on the AOS board's `eth1` and the ECAN board that drives the 16 joint motors
and reads both batteries.

- Frame codec: 10-byte header (`0x5244`, sequence, flags, code, type, length, byte-sum checksum), every frame type
  seen or parsed by the vendor master.
- Joint records: 1 kHz commands (position, velocity, torque, kp, kd, control word) and state replies (position,
  velocity, torque, motor/driver temperature, status word, counter), named by leg/joint (`fl_hipx_joint` ...), with
  the vendor's motor-to-URDF calibration.
- Battery frames (two 258-byte pack records); decode them with [`m20-battery`](https://crates.io/crates/m20-battery).
- The raw-socket transport rl_deploy uses (Linux): `AF_PACKET` with `sendto`/`recvfrom` flag `0x80000000`, the vendor
  kernel's private path that tcpdump cannot see.
- Bus owner: `startup` (rl_deploy's startup sequence, byte for byte, tested against its capture), `owner`
  (one frame per 1 ms tick; any command carrying a gain or feed-forward torque is replaced by the sitting command
  unless `allow_gains`, and so is one that is not exactly one command per joint;
  charge-enable requests in place of passive ticks) and `bus` (Linux: the owner on the vendor socket, refuses while
  rl_deploy runs).
- Passive taps: pcapng captures and the live unix-socket stream from the `ecan_tap.so` shim inside rl_deploy.

```rust
use m20_ecan::{decode_ethernet, tap, Calibration, Direction, Message};

let bytes = std::fs::read("capture.pcapng")?;
let calibration = Calibration::m20();
for (timestamp_ns, direction, frame) in tap::read_capture(&bytes)? {
    if let Ok((_, Message::JointStates { joints, .. })) = decode_ethernet(frame, direction) {
        for joint in joints {
            let angle = calibration.urdf_position(joint.joint, joint.position);
            println!("{timestamp_ns} {} {angle:.3} rad {:.1} C", joint.joint.urdf_name(), joint.motor_temperature_c);
        }
    }
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

What a moving session showed (2026-11-20): commanded gains per phase (passive 0/0; one enable cycle with control
word 2 before a stand-up; stand-up kp 200 / kd 4, wheels kd 2; NaviCharge 60/1.5 with x1.3 on the legs for its first
29 steps; walking 80/3 or 80/6, wheels kd 0.6; lie-down 300/4.5 then 0/4.5 for 2 s each), the calibration holding
through the stand-up, torque following the PD law with the position's sign, wheel velocity in rad/s with forward
negative, wheel position accumulated over many turns, status word 1 throughout. See `tests/moving.rs`.

Examples: `cargo run --example ecan_dump capture.pcapng joints.csv`; `ecan_listen eth1 10` (Linux, receive-only,
refuses to run while rl_deploy is up).

On the robot the board only answers rl_deploy's commands, and a second sender would fight rl_deploy for the motors:
nothing here sends unless you call `RawSocket::send`.

License: Apache-2.0
