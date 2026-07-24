'''
Written by yufeng.wu0902@gmail.com

Binary framed wire protocol shared by the host, controller, and robot:

    [START=0xAA][MSG_TYPE][LENGTH][PAYLOAD...][CRC_LO][CRC_HI]

CRC-16/ARC (poly 0x8005, reflected 0xA001, init 0x0000) is computed over
MSG_TYPE + LENGTH + PAYLOAD. The C++ side (firmware/*/include/protocol.h)
must stay in sync with this module.
'''

from enum import IntEnum

START_BYTE = 0xAA
_HEADER_LEN = 2       # msg_type, length
_CRC_LEN = 2


class CmdType(IntEnum):
    """Host -> robot commands."""
    MOVE = 0x01
    TORQUE_SET = 0x02
    BATTERY_REQUEST = 0x03
    RECORD_SEND = 0x04
    JUMP = 0x05


class ReplyType(IntEnum):
    """Robot -> host replies."""
    BATTERY_REPLY = 0x81
    RECORD_DATA_REPLY = 0x82


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
