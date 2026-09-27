'''
Written by yufeng.wu0902@gmail.com

Binary framed wire protocol shared by the host, controller, and robot:

    [START=0xAA][MSG_TYPE][LENGTH][PAYLOAD...][CRC_LO][CRC_HI]

CRC-16/ARC (poly 0x8005, reflected 0xA001, init 0x0000) is computed over
MSG_TYPE + LENGTH + PAYLOAD. The C++ side (firmware/*/include/protocol.h)
must stay in sync with this module.
'''

import struct
from enum import IntEnum

START_BYTE = 0xAA
_HEADER_LEN = 2       # msg_type, length
_CRC_LEN = 2


class CmdType(IntEnum):
    """Host -> robot commands."""
    MOVE = 0x01
    TORQUE_SET = 0x02  # {mode: uint8} 0=off, 1=on (full stiffness), 2=damping
    BATTERY_REQUEST = 0x03
    RECORD_SEND = 0x04     # retired - recording removed; do not reuse this id
    JUMP = 0x05
    GAIT_RUN = 0x06   # {gait_id: uint8, direction_id: uint8} - run a local gait/direction autonomously
    GAIT_STOP = 0x07  # no payload - stop autonomous gait, hand control back to raw MOVE
    GREET = 0x08      # no payload - scripted greeting sequence, runs entirely on the robot
    HOST_CONNECTED = 0x09  # no payload - controller-local only, never forwarded to the robot;
                            # tells the controller firmware to switch from independent (gamepad) mode
                            # to relay mode
    LED_BRIGHTNESS = 0x0A  # {brightness: uint8, 0-255} - sets & persists the robot's RGB LED brightness
    TELEMETRY_REQUEST = 0x0B  # no payload - ask for one joint snapshot, answered with TELEMETRY_REPLY


class ReplyType(IntEnum):
    """Robot -> host replies."""
    BATTERY_REPLY = 0x81      # {percent: uint8, millivolts: uint16}
    RECORD_DATA_REPLY = 0x82  # retired alongside RECORD_SEND; do not reuse
    TELEMETRY_REPLY = 0x83    # one joint snapshot - see TELEMETRY_STRUCT


# Canonical direction ordering for GAIT_RUN's direction_id (0-11). Must match
# the order q8botone-firmware/robot/include/gaitTables.h's *_DIRECTIONS arrays
# use (see export_gait_tables.py) and the firmware protocol.h comment.
DIRECTION_ORDER = [
    'f', 'b', 'l', 'r',
    'fl_0.75', 'fl_0.5', 'fr_0.75', 'fr_0.5',
    'bl_0.75', 'bl_0.5', 'br_0.75', 'br_0.5',
]

# gait_id -> GAITS name, for GAIT_RUN. Slow/med/fast mirror TROT_FAST's own
# SPEED_TIER_CHAINS ordering, so native execution can blend across the same
# 3 tiers on-robot (see GaitManager.SPEED_TIER_CHAINS, operate.py).
GAIT_IDS = {
    0: 'TROT_SLOW',
    1: 'TROT_MED',
    2: 'TROT_FAST',
    3: 'TROT_HIGH',
    4: 'TROT_LOW',
    5: 'CRAWL',
    6: 'BOUND',
    7: 'TROT_INPLACE',
}

# Reverse lookups, for turning a resolved direction/gait name into the two
# bytes a GAIT_RUN payload wants.
DIRECTION_TO_ID = {name: i for i, name in enumerate(DIRECTION_ORDER)}
GAIT_NAME_TO_ID = {name: gait_id for gait_id, name in GAIT_IDS.items()}


# TELEMETRY_REPLY payload: all 8 joints, 56 bytes. Must match the
# TelemetryPayload struct in q8botone-firmware/robot/include/protocol.h.
#
# Position and current are read fresh on every request; temperature is
# whatever the robot last sampled (it refreshes on its own ~1s timer), since
# it sits outside the contiguous register block the main read covers and
# moves over seconds rather than milliseconds.
TELEMETRY_STRUCT = struct.Struct('<8i8h8b')
TELEMETRY_SIZE = TELEMETRY_STRUCT.size   # 56


def parse_battery(payload: bytes):
    """
    Unpack a BATTERY_REPLY payload.

    Returns:
        (percent, volts) - volts is None against firmware old enough to send
        only the percentage, so a host update without a reflash still reads
        the charge level instead of failing.
    """
    if not payload:
        return None, None
    percent = payload[0]
    if len(payload) >= 3:
        return percent, struct.unpack_from('<H', payload, 1)[0] / 1000.0
    return percent, None


def parse_telemetry(payload: bytes):
    """
    Unpack a TELEMETRY_REPLY payload.

    Returns:
        dict with 'position' (raw ticks), 'current' (mA, signed - reads near
        zero with torque off, since none is flowing) and 'temperature'
        (degrees C), each a tuple of 8. None if the payload is the wrong size.
    """
    if len(payload) != TELEMETRY_SIZE:
        return None
    values = TELEMETRY_STRUCT.unpack(payload)
    return {
        'position': values[0:8],
        'current': values[8:16],
        'temperature': values[16:24],
    }


# Matches q8Dynamixel.cpp's _deg2Dxl/_dxl2Deg exactly (_zeroOffset=4096,
# _gearRatio=1) - so a degree value round-trips through a raw tick and back
# to the same degree, and a tick read back from telemetry converts to
# whatever angle move_all() would need to return the servo to that exact
# position. That symmetry is what makes capturing a hand-posed pose and
# replaying it through move_all() work at all (see Q8Bot.capture_pose()).
ZERO_OFFSET = 4096
TICKS_PER_REV = 4096


def ticks_to_degrees(ticks):
    """Raw Dynamixel present-position ticks -> degrees, move_all()'s units."""
    return (ticks - ZERO_OFFSET) * 360.0 / TICKS_PER_REV


def crc16_arc(data: bytes, crc: int = 0x0000) -> int:
    """CRC-16/ARC over the given bytes."""
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(msg_type: int, payload: bytes) -> bytes:
    """Build a complete frame (start byte through CRC) for the given message type/payload."""
    if len(payload) > 255:
        raise ValueError(f"Payload too large for a single frame: {len(payload)} bytes")

    body = bytes([msg_type, len(payload)]) + payload
    crc = crc16_arc(body)
    return bytes([START_BYTE]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_frame(serial_handler, timeout=None):
    """
    Read one frame from a pyserial-like handler (must expose .read(n)).

    Scans for the start byte, reads the type/length header, then exactly
    that many payload bytes plus the CRC, and validates it. On any framing
    or CRC failure, returns None (caller should just try again next poll -
    a bad frame can never corrupt a subsequent one since we resume
    scanning from the very next byte).

    Returns:
        (msg_type, payload_bytes) or None if no valid frame was available.
    """
    start = serial_handler.read(1)
    if len(start) != 1 or start[0] != START_BYTE:
        return None

    header = serial_handler.read(_HEADER_LEN)
    if len(header) != _HEADER_LEN:
        return None
    msg_type, length = header[0], header[1]

    rest = serial_handler.read(length + _CRC_LEN)
    if len(rest) != length + _CRC_LEN:
        return None

    payload = rest[:length]
    crc_received = rest[length] | (rest[length + 1] << 8)
    crc_expected = crc16_arc(header + payload)

    if crc_received != crc_expected:
        return None

    return msg_type, payload
