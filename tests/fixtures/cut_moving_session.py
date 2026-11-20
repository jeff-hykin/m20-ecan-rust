# Cut the moving session's ecan.pcapng to a fixture: for every distinct commanded (kp, kd, control word) signature the
# first 3 command/reply pairs, pairs at chosen moments, every charge-enable frame from 14:18:09 to 14:18:20 CST, and
# battery frames at chosen moments. usage: extract.py in.pcapng out.pcapng
import struct, sys, datetime
src, dst = sys.argv[1], sys.argv[2]
def cst(s): return int((datetime.datetime(2026, 11, 20, *map(int, s.split(':')[:2])) - datetime.datetime(1970,1,1)).total_seconds() - 8*3600 + float(s.split(':')[2])) * 10**9 if False else int(((datetime.datetime(2026,11,20,int(s[:2]),int(s[3:5])) - datetime.datetime(1970,1,1)).total_seconds() - 8*3600 + float(s[6:])) * 1e9)
moments = [cst(s) for s in ("14:16:20.000", "14:16:34.600", "14:17:00.000", "14:17:44.500", "14:18:30.000")]
battery_moments = [cst(s) for s in ("14:16:20", "14:17:00", "14:18:30")]
enable_window = (cst("14:18:09.000"), cst("14:18:20.000"))
f = open(src, "rb"); out = open(dst, "wb")
seen = {}; pending_pair = None; want_pairs = {}; got_moment = set(); got_battery = set(); n = 0
while True:
    head = f.read(8)
    if len(head) < 8: break
    kind, length = struct.unpack("<II", head)
    block = head + f.read(length - 8)
    if kind != 6:
        out.write(block); continue
    hi, lo, cap = struct.unpack_from("<III", block, 12)
    t = (hi << 32) | lo
    frame = block[28:28 + cap]
    ftype = frame[19] if len(frame) > 19 else None
    tx = frame[:6] == b"\x22\x33\x44\x55\x66\x77"
    keep = False
    if ftype == 0x89 and tx:
        units = [struct.unpack_from("<hhH", frame, 24 + 16 + 24 * i) for i in range(16)]
        sig = tuple(units)
        count = seen.get(sig, 0)
        if count < 3: seen[sig] = count + 1; keep = True
        for i, m in enumerate(moments):
            if i not in got_moment and t >= m: got_moment.add(i); keep = True
        pending_pair = keep
    elif ftype == 0x89 and not tx:
        keep = bool(pending_pair); pending_pair = None
    elif ftype == 0x87:
        for i, m in enumerate(battery_moments):
            if i not in got_battery and t >= m: got_battery.add(i); keep = True
    elif ftype == 0xB3:
        keep = enable_window[0] <= t <= enable_window[1]
    if keep:
        out.write(block); n += 1
print(n, "frames,", len(seen), "gain signatures")
