//! Summarize a shim capture, optionally writing every joint record as CSV.
//! usage: ecan_dump capture.pcapng [joints.csv]

use std::collections::BTreeMap;
use std::io::Write;

use m20_ecan::{decode_ethernet, tap, Calibration, Direction, Frame, Message};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).ok_or("usage: ecan_dump capture.pcapng [joints.csv]")?;
    let bytes = std::fs::read(path)?;
    let frames = tap::read_capture(&bytes)?;
    let mut csv = match args.get(2) {
        Some(out) => {
            let mut file = std::io::BufWriter::new(std::fs::File::create(out)?);
            writeln!(
                file,
                "t_ns,dir,seq,joint,position,velocity,torque,kp_or_motor_c,kd_or_driver_c,word,data_id,urdf_position"
            )?;
            Some(file)
        }
        None => None,
    };
    let calibration = Calibration::m20();
    let mut counts: BTreeMap<(String, String), (usize, usize)> = BTreeMap::new();
    for (timestamp_ns, direction, ethernet) in &frames {
        let dir = match direction {
            Direction::ToBoard => "tx",
            Direction::FromBoard => "rx",
        };
        let frame = match Frame::parse_ethernet(ethernet) {
            Ok(frame) => frame,
            Err(error) => {
                counts.entry((dir.into(), format!("not ECAN ({error})"))).or_default().0 += 1;
                continue;
            }
        };
        let entry = counts.entry((dir.into(), format!("{:?}", frame.header.frame_type))).or_default();
        entry.0 += 1;
        if !frame.checksum_ok() {
            entry.1 += 1;
        }
        let Some(csv) = csv.as_mut() else { continue };
        let seq = frame.header.seq;
        match decode_ethernet(ethernet, *direction) {
            Ok((_, Message::JointCommands(commands))) => {
                for c in commands {
                    let urdf = calibration.urdf_position(c.joint, c.position);
                    writeln!(
                        csv,
                        "{timestamp_ns},tx,{seq},{},{},{},{},{},{},{},{},{urdf}",
                        c.joint.urdf_name(),
                        c.position,
                        c.velocity,
                        c.torque,
                        c.kp,
                        c.kd,
                        c.control_word,
                        c.data_id
                    )?;
                }
            }
            Ok((_, Message::JointStates { joints, .. })) => {
                for s in joints {
                    let urdf = calibration.urdf_position(s.joint, s.position);
                    writeln!(
                        csv,
                        "{timestamp_ns},rx,{seq},{},{},{},{},{},{},{},{},{urdf}",
                        s.joint.urdf_name(),
                        s.position,
                        s.velocity,
                        s.torque,
                        s.motor_temperature_c,
                        s.driver_temperature_c,
                        s.status_word,
                        s.data_id
                    )?;
                }
            }
            _ => {}
        }
    }
    let span = match (frames.first(), frames.last()) {
        (Some(first), Some(last)) => (last.0 - first.0) as f64 / 1e9,
        _ => 0.0,
    };
    println!("{} frames over {span:.3} s", frames.len());
    println!("{:3} {:28} {:>8} {:>8} {:>9}", "dir", "type", "count", "per_s", "bad_sum");
    for ((dir, kind), (count, bad)) in counts {
        println!("{dir:3} {kind:28} {count:8} {:8.1} {bad:9}", count as f64 / span.max(1e-9));
    }
    Ok(())
}
