"""Seri protokol icin donanimsiz ve ROS'suz test.

Calistirma (repo kokunden):
    python3 jetson/test/test_protocol.py
"""

import os
import random
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src',
                                'stingray_mobility'))
from stingray_mobility.protocol import (  # noqa: E402
    SENSOR_PACKET_LEN, SensorFrameParser, encode_motor_command)
from stingray_mobility.utils import crc8  # noqa: E402

# Firmware tarafindaki teensy/test/test_frame_codec.cpp icinde de ayni
# cerceve dogrulaniyor: iki taraf ayni CRC ve bayt sirasini kullanmali.
GOLDEN_NEUTRAL = bytes([0xAA, 0x55, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC,
                        0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x09])


def sensor_frame(yaw=12345, pitch=-250, roll=90, depth_mm=1500, leak=0,
                 bat_cv=2310):
    payload = struct.pack('>HhhHBH', yaw, pitch, roll, depth_mm, leak,
                          bat_cv) + b'\x00'
    return b'\xBB\x66' + payload + bytes([crc8(payload), 0x00])


def test_crc8_check_value():
    assert crc8(b'123456789') == 0xF4


def test_motor_command_matches_firmware_golden_frame():
    assert encode_motor_command([1500] * 6, 1100, 1900) == GOLDEN_NEUTRAL


def test_motor_command_clamps_and_validates():
    frame = encode_motor_command([500, 2500, 1500, 1500, 1500, 1500],
                                 1100, 1900)
    assert struct.unpack('>H', frame[2:4])[0] == 1100
    assert struct.unpack('>H', frame[4:6])[0] == 1900
    try:
        encode_motor_command([1500] * 5, 1100, 1900)
    except ValueError:
        pass
    else:
        raise AssertionError('5 deger kabul edilmemeli')


def test_sensor_decode_units():
    (r,) = SensorFrameParser().feed(sensor_frame())
    assert abs(r.yaw - 123.45) < 1e-9
    assert abs(r.pitch + 2.50) < 1e-9
    assert abs(r.depth - 1.5) < 1e-9
    assert abs(r.battery_voltage - 23.10) < 1e-9
    assert r.leak_detected is False
    assert len(sensor_frame()) == SENSOR_PACKET_LEN


def test_header_split_across_reads_is_kept():
    # Eski uygulamada tampon tamamen temizleniyordu ve okumanin son bayti
    # olan 0xBB kayboluyordu; o cerceve dusuyordu.
    p = SensorFrameParser()
    f = sensor_frame()
    assert p.feed(b'\x01\x02' + f[:1]) == []
    assert len(p.feed(f[1:])) == 1


def test_bad_crc_then_good_frame():
    p = SensorFrameParser()
    bad = bytearray(sensor_frame())
    bad[8] ^= 0xFF
    out = p.feed(bytes(bad) + sensor_frame(depth_mm=2000))
    assert len(out) == 1 and abs(out[0].depth - 2.0) < 1e-9
    assert p.crc_errors == 1


def test_random_chunking_and_noise():
    rng = random.Random(7)
    stream = bytearray()
    for i in range(500):
        stream += bytes(rng.randrange(256) for _ in range(rng.randrange(10)))
        stream += sensor_frame(depth_mm=i)
    p = SensorFrameParser()
    got = []
    pos = 0
    while pos < len(stream):
        n = rng.randrange(1, 64)
        got += p.feed(bytes(stream[pos:pos + n]))
        pos += n
    depths = [round(r.depth * 1000) for r in got]
    assert depths == list(range(500))


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            fn()
            print(f'[ OK ] {name}')
    print('OK')
