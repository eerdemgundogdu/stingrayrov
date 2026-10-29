from . import config

def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def angle_diff(target: float, current: float) -> float:
    diff = target - current
    while diff > 180.0:
        diff -= 360.0
    while diff < -180.0:
        diff += 360.0
    return diff

def heading_reached(target: float, current: float, tol: float = None) -> bool:
    if tol is None:
        tol = config.HEADING_TOLERANCE
    return abs(angle_diff(target, current)) < tol

def depth_reached(target: float, current: float, tol: float = None) -> bool:
    if tol is None:
        tol = config.DEPTH_TOLERANCE
    return abs(target - current) < tol

def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))
