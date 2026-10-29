"""Teensy <-> Jetson seri protokolu (ROS'tan bagimsiz).

Cerceve yapisi teensy/stingray_teensy/frame_codec.h icinde belgelenmistir.
Bu modul serial_bridge_node tarafindan kullanilir ve ROS kurulu olmadan
jetson/test/test_protocol.py ile test edilir.
"""

import struct
from dataclasses import dataclass
from typing import Iterable, List

from .utils import crc8

HEADER_CMD = bytes([0xAA, 0x55])
HEADER_SENSOR = bytes([0xBB, 0x66])
CMD_PACKET_LEN = 15
SENSOR_PACKET_LEN = 16
MOTOR_COUNT = 6


@dataclass(frozen=True)
class SensorReading:
    yaw: float              # derece
    pitch: float            # derece
    roll: float             # derece
    depth: float            # m
    leak_detected: bool
    battery_voltage: float  # V


def encode_motor_command(pwm_values: Iterable[float], pwm_min: int,
                         pwm_max: int) -> bytes:
    """6 PWM degerini sinirlayip komut cercevesine paketler."""
    values = list(pwm_values)
    if len(values) != MOTOR_COUNT:
        raise ValueError(f'{MOTOR_COUNT} PWM degeri bekleniyor, '
                         f'{len(values)} geldi')
    payload = bytearray()
    for pwm in values:
        clamped = int(max(pwm_min, min(pwm_max, pwm)))
        payload += struct.pack('>H', clamped)
    return HEADER_CMD + bytes(payload) + bytes([crc8(bytes(payload))])


def decode_sensor_payload(payload: bytes) -> SensorReading:
    yaw, pitch, roll, depth, leak, bat = struct.unpack('>HhhHBH', payload[:11])
    return SensorReading(
        yaw=yaw / 100.0,
        pitch=pitch / 100.0,
        roll=roll / 100.0,
        depth=depth / 1000.0,
        leak_detected=leak > 0,
        battery_voltage=bat / 100.0,   # firmware V x100 gonderir
    )


class SensorFrameParser:
    """Bayt akisini sensor cercevelerine ayirir.

    Bozuk CRC'de sadece baslik atilir; tamponun sonunda yarim kalmis bir
    baslik bayti (0xBB) bir sonraki okuma icin saklanir.
    """

    def __init__(self):
        self._buf = bytearray()
        self.frames_ok = 0
        self.crc_errors = 0

    def feed(self, chunk: bytes) -> List[SensorReading]:
        self._buf.extend(chunk)
        out: List[SensorReading] = []
        while True:
            idx = self._buf.find(HEADER_SENSOR)
            if idx < 0:
                # Son bayt bir sonraki cercevenin ilk baslik bayti olabilir.
                keep = 1 if self._buf[-1:] == HEADER_SENSOR[:1] else 0
                del self._buf[:len(self._buf) - keep]
                return out
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < SENSOR_PACKET_LEN:
                return out

            frame = bytes(self._buf[:SENSOR_PACKET_LEN])
            if crc8(frame[2:14]) != frame[14]:
                self.crc_errors += 1
                del self._buf[:1]
                continue

            out.append(decode_sensor_payload(frame[2:14]))
            self.frames_ok += 1
            del self._buf[:SENSOR_PACKET_LEN]
