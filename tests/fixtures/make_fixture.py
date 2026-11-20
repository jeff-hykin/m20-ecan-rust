#!/usr/bin/env python3
"""Cut a shim capture down to a test fixture: its header blocks, the first N joint frames each way, and every
battery / charge-enable / other frame. usage: make_fixture.py capture.pcapng out.pcapng [N]"""
import struct
import sys

source, target = sys.argv[1], sys.argv[2]
keep_joint_frames = int(sys.argv[3]) if len(sys.argv) > 3 else 40
data = open(source, "rb").read()
out = bytearray()
joint_frames = {}
offset = 0
while offset + 12 <= len(data):
    block_type, block_len = struct.unpack_from("<II", data, offset)
    block = data[offset : offset + block_len]
    offset += block_len
    if block_type != 6:
        out += block
        continue
    frame = block[28 : 28 + struct.unpack_from("<I", block, 20)[0]]
    frame_type = frame[19] if len(frame) > 19 else None
    if frame_type == 0x89:
        key = frame[0:6]
        joint_frames[key] = joint_frames.get(key, 0) + 1
        if joint_frames[key] > keep_joint_frames:
            continue
    out += block
open(target, "wb").write(out)
print(f"{len(out)} bytes")
