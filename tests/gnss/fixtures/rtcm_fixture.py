#!/usr/bin/env python3
"""rtcm_fixture.py - a synthetic RTCM3 stream for the GNSS stubs (SL-0233).

No real station data. Each frame is a valid RTCM3 transport frame:

    0xD3 | 000000 + 10-bit payload length | payload | CRC24Q (3 bytes)

The payload starts with a 12-bit message number (1005, 1077 ... as a
real base would send); the rest is deterministic filler from a seeded
generator. CRC24Q covers everything before the CRC (preamble included).

The default stream holds one frame each of 20, 180, 360, 720 and 1000
bytes (whole frame, header and CRC included): the sizes that decide how
de_mavlink fragments GPS_RTCM_DATA (180-byte data field, 4 fragments
at most), plus the near-maximum 1000-byte case.

    python3 rtcm_fixture.py [out.bin]     # write the default stream
"""

import os
import random
import sys

SIZES = (20, 180, 360, 720, 1000)
MSG_NUMBERS = (1005, 1077, 1087, 1097, 1230)
HEADER_LEN = 3
CRC_LEN = 3
PREAMBLE = 0xD3
CRC24Q_POLY = 0x1864CFB


def crc24q(data):
    """RTCM3 / Qualcomm CRC-24Q, init 0."""
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLY
    return crc & 0xFFFFFF


def make_frame(total_len, msg_number, rng):
    """One valid RTCM3 frame of exactly total_len bytes."""
    plen = total_len - HEADER_LEN - CRC_LEN
    if plen < 2 or plen > 1023:
        raise ValueError(f"frame length {total_len} has no valid payload")
    payload = bytearray(rng.getrandbits(8) for _ in range(plen))
    payload[0] = (msg_number >> 4) & 0xFF
    payload[1] = (payload[1] & 0x0F) | ((msg_number & 0x0F) << 4)
    head = bytes([PREAMBLE, (plen >> 8) & 0x03, plen & 0xFF])
    body = head + bytes(payload)
    return body + crc24q(body).to_bytes(3, "big")


def build_stream(sizes=SIZES, seed=1):
    rng = random.Random(seed)
    frames = [make_frame(n, MSG_NUMBERS[i % len(MSG_NUMBERS)], rng)
              for i, n in enumerate(sizes)]
    return b"".join(frames)


def split_frames(stream):
    """Cut a stream into frames. Raises ValueError on a bad frame, so a
    test sees a broken length or CRC at once. Returns the list of frame
    bytes."""
    out, i = [], 0
    while i < len(stream):
        if stream[i] != PREAMBLE or len(stream) - i < HEADER_LEN:
            raise ValueError(f"no RTCM3 preamble at byte {i}")
        plen = ((stream[i + 1] & 0x03) << 8) | stream[i + 2]
        end = i + HEADER_LEN + plen + CRC_LEN
        if end > len(stream):
            raise ValueError(f"frame at byte {i} runs past the stream")
        frame = stream[i:end]
        want = int.from_bytes(frame[-CRC_LEN:], "big")
        if crc24q(frame[:-CRC_LEN]) != want:
            raise ValueError(f"bad CRC24Q in the frame at byte {i}")
        out.append(frame)
        i = end
    return out


def default_path():
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "rtcm_fixture.bin")


def load_or_build(path=None):
    """The fixture bytes: the file when it exists, else the default
    stream (same bytes - the generator is seeded)."""
    if path and os.path.isfile(path):
        with open(path, "rb") as f:
            return f.read()
    return build_stream()


def main(argv):
    out = argv[0] if argv else default_path()
    data = build_stream()
    with open(out, "wb") as f:
        f.write(data)
    print(f"{out}: {len(data)} bytes, {len(split_frames(data))} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
