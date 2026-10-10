"""Minimal Feetech STS/SCS serial-bus protocol (half-duplex TTL, 1 Mbps) + a software fake bus for tests.

Packet:  FF FF ID LEN INST PARAMS... CHK     LEN = len(PARAMS)+2   CHK = ~(ID+LEN+INST+sum(PARAMS)) & 0xFF
Status:  FF FF ID LEN ERR  DATA...  CHK     LEN = len(DATA)+2
Register map is for the STS3215/STS3250 (little-endian words)."""
import struct, time
from dataclasses import dataclass

PING, READ, WRITE, REG_WRITE, ACTION, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x04, 0x05, 0x82, 0x83
BROADCAST = 0xFE

# name: (address, size)
REG = dict(
    id=(5, 1), baud=(6, 1), min_pos=(9, 2), max_pos=(11, 2), max_temp=(13, 1), max_volt=(14, 1), min_volt=(15, 1),
    max_torque=(16, 2), p_gain=(21, 1), d_gain=(22, 1), i_gain=(23, 1), homing_offset=(31, 2), mode=(33, 1),
    torque_enable=(40, 1), acceleration=(41, 1), goal_pos=(42, 2), goal_time=(44, 2), goal_speed=(46, 2), torque_limit=(48, 2),
    lock=(55, 1), present_pos=(56, 2), present_speed=(58, 2), present_load=(60, 2), present_volt=(62, 1), present_temp=(63, 1),
    moving=(66, 1), present_current=(69, 2),
)
TICKS = 4096          # per revolution
CENTER = 2048

def checksum(b):
    return (~sum(b)) & 0xFF

def packet(sid, inst, params=b""):
    body = bytes([sid, len(params) + 2, inst]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])

def encode_sign_magnitude(v, bit):
    return (abs(int(v)) | (1 << bit)) if v < 0 else int(v)

def decode_sign_magnitude(v, bit):
    return -(v & ~(1 << bit)) if v & (1 << bit) else v

def pack(size, value):
    return int(value).to_bytes(size, "little", signed=False)

def parse_status(buf):
    """Return (id, err, data) from the first valid status packet in buf, else None."""
    i = buf.find(b"\xff\xff")
    while i >= 0 and i + 4 <= len(buf):
        sid, ln = buf[i + 2], buf[i + 3]
        end = i + 4 + ln
        if end <= len(buf):
            body = buf[i + 2:end - 1]
            if checksum(body) == buf[end - 1]:
                return sid, buf[i + 4], bytes(buf[i + 5:end - 1])
        i = buf.find(b"\xff\xff", i + 1)
    return None

class ServoError(RuntimeError):
    pass

class Bus:
    """Real serial bus. One request/response at a time."""
    def __init__(self, port, baud=1_000_000, timeout=0.01):
        import serial
        self.s = serial.Serial(port, baud, timeout=timeout)
    def close(self): self.s.close()
    def _xfer(self, pkt, expect):
        self.s.reset_input_buffer(); self.s.write(pkt); self.s.flush()
        if expect == 0: return b""
        buf = self.s.read(expect)
        return buf
    def transact(self, sid, inst, params=b"", reply_len=0):
        raw = self._xfer(packet(sid, inst, params), 0 if sid == BROADCAST else 6 + reply_len)
        return raw

class FakeBus:
    """In-memory servo bank implementing PING/READ/WRITE/SYNC_WRITE/SYNC_READ against 256-byte memories. Used by tests and --dry-run."""
    def __init__(self, ids):
        self.mem = {i: bytearray(256) for i in ids}
        for i in ids:
            self.mem[i][5] = i; self.mem[i][62] = 120; self.mem[i][63] = 30      # volt 12.0 V, temp 30 C
            self.mem[i][56:58] = pack(2, CENTER); self.mem[i][42:44] = pack(2, CENTER)
    def close(self): pass
    def _status(self, sid, data=b"", err=0):
        return packet(sid, err, data)   # status packet has the same framing with ERR in the INST slot
    def transact(self, sid, inst, params=b"", reply_len=0):
        if inst == SYNC_WRITE:
            addr, n = params[0], params[1]; rest = params[2:]
            for k in range(0, len(rest), n + 1):
                i = rest[k]; 
                if i in self.mem: self._write(i, addr, rest[k + 1:k + 1 + n])
            return b""
        if inst == SYNC_READ:
            addr, n = params[0], params[1]
            out = b""
            for i in params[2:]:
                if i in self.mem: out += self._status(i, bytes(self.mem[i][addr:addr + n]))
            return out
        if sid == BROADCAST or sid not in self.mem: return b""
        if inst == PING: return self._status(sid)
        if inst == READ: return self._status(sid, bytes(self.mem[sid][params[0]:params[0] + params[1]]))
        if inst == WRITE: self._write(sid, params[0], params[1:]); return self._status(sid)
        return b""
    def _write(self, sid, addr, data):
        self.mem[sid][addr:addr + len(data)] = data
        if addr == 42 and len(data) >= 2:              # goal -> present (instant servo)
            self.mem[sid][56:58] = data[:2]
        if addr == 5: pass

class Servos:
    """High-level access to one bus."""
    def __init__(self, bus): self.bus = bus
    def ping(self, sid):
        r = self.bus.transact(sid, PING, b"", 0)
        return parse_status(r) is not None
    def read(self, sid, name):
        addr, size = REG[name]
        r = parse_status(self.bus.transact(sid, READ, bytes([addr, size]), size))
        if r is None: raise ServoError(f"no reply from servo {sid} reading {name}")
        if r[1]: raise ServoError(f"servo {sid} error flags 0x{r[1]:02x}")
        return int.from_bytes(r[2], "little")
    def write(self, sid, name, value):
        addr, size = REG[name]
        self.bus.transact(sid, WRITE, bytes([addr]) + pack(size, value), 0)
    def sync_write(self, name, values):
        """values: {id: int}"""
        addr, size = REG[name]
        params = bytes([addr, size]) + b"".join(bytes([i]) + pack(size, v) for i, v in values.items())
        self.bus.transact(BROADCAST, SYNC_WRITE, params, 0)
    def sync_read(self, name, ids):
        addr, size = REG[name]
        raw = self.bus.transact(BROADCAST, SYNC_READ, bytes([addr, size]) + bytes(ids), (size + 6) * len(ids))
        out, pos = {}, 0
        frame = size + 6
        while pos + frame <= len(raw):
            r = parse_status(raw[pos:pos + frame])
            if r: out[r[0]] = int.from_bytes(r[2], "little")
            pos += frame
        return out
    # --- convenience
    def torque(self, ids, on):
        self.sync_write("torque_enable", {i: int(on) for i in ids})
    def set_id(self, old, new):
        self.write(old, "lock", 0); self.write(old, "id", new); self.write(new, "lock", 1)
    def center(self, sid):
        """Declare the CURRENT physical position to be tick 2048 (used before mounting horns)."""
        self.write(sid, "torque_enable", 128)
