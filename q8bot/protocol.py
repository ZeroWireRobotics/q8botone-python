'''
Written by yufeng.wu0902@gmail.com

Binary framed wire protocol shared by the host, controller, robot and camera:

    [START=0xAA][MSG_TYPE][LENGTH][PAYLOAD...][CRC_LO][CRC_HI]

CRC-16/ARC (poly 0x8005, reflected 0xA001, init 0x0000) is computed over
MSG_TYPE + LENGTH + PAYLOAD. The C++ side
(q8botone-firmware/{robot,controller,camera}/include/protocol.h, three
identical copies) must stay in sync with this module.
'''

import struct
from collections import namedtuple
from enum import IntEnum

# Wire protocol version, shared by host, controller and robot. Bump it for
# any change the other side has to know about: a new or changed command or
# reply, a payload layout, gait/direction IDs. Must equal protocol.h's
# PROTOCOL_VERSION. Not yet exchanged or checked on connect.
PROTOCOL_VERSION = 5  # 2: SENSOR_REQUEST / SENSOR_REPLY; 3: GAIT_RUN must be repeated;
                      # 4: TELEMETRY_REPLY adds servo voltage, hardware errors, torque restores (80 bytes),
                      #    POWER_RAIL, SERVO_REBOOT, LED_BRIGHTNESS's don't-save flag;
                      # 5: the camera - message ranges, CAM_STATUS_REQUEST, CAM_WIFI_SET, CAM_STATUS

START_BYTE = 0xAA
_HEADER_LEN = 2       # msg_type, length
_CRC_LEN = 2


# Who a message is for, by its type - the robot routes on this:
#   0x01-0x1F  commands for the robot   from the host over the radio, or from the camera
#   0x20-0x3F  commands for the camera  the robot passes them on to its camera link
#   0x81-0x9F  replies from the robot   back to whoever asked
#   0xA0-0xBF  replies from the camera  the robot passes them on to the host
def is_robot_command(msg_type):
    return 0x01 <= msg_type <= 0x1F


def is_camera_command(msg_type):
    return 0x20 <= msg_type <= 0x3F


def is_robot_reply(msg_type):
    return 0x81 <= msg_type <= 0x9F


def is_camera_reply(msg_type):
    return 0xA0 <= msg_type <= 0xBF


class CmdType(IntEnum):
    """Host -> robot commands."""
    MOVE = 0x01
    TORQUE_SET = 0x02  # {mode: uint8} 0=off, 1=on (full stiffness), 2=damping
    BATTERY_REQUEST = 0x03
    RECORD_SEND = 0x04     # retired - recording removed; do not reuse this id
    JUMP = 0x05
    GAIT_RUN = 0x06   # {gait_id: uint8, direction_id: uint8} - run a local gait/direction autonomously;
                      # repeat it while walking (NATIVE_GAIT_RESEND_INTERVAL) - the robot stops
                      # after 1s without one
    GAIT_STOP = 0x07  # no payload - stop autonomous gait, hand control back to raw MOVE
    GREET = 0x08      # no payload - scripted greeting sequence, runs entirely on the robot
    HOST_CONNECTED = 0x09  # no payload - controller-local only, never forwarded to the robot;
                            # tells the controller firmware to switch from independent (gamepad) mode
                            # to relay mode
    LED_BRIGHTNESS = 0x0A  # {brightness: uint8, 0-255[, flags: uint8]} - sets & persists the robot's
                           # RGB LED brightness; flags bit 0 = this once, don't persist it
    TELEMETRY_REQUEST = 0x0B  # no payload - ask for one joint snapshot, answered with TELEMETRY_REPLY
    SENSOR_REQUEST = 0x0C     # no payload - sample the robot's Qwiic sensors, answered with SENSOR_REPLY
    POWER_RAIL = 0x0D         # {rail: uint8 5|6, on: uint8} - switch a buck converter, for power
                              # measurements; 6V off cuts the servos' power, torque off until a restart
    SERVO_REBOOT = 0x0E       # {slot: uint8, 1-8} - restart one servo (DYNAMIXEL Reboot), clearing its
                              # hardware error; the robot puts back its profile, gain and torque

    # host or robot -> camera
    CAM_STATUS_REQUEST = 0x20  # no payload - answered with CAM_STATUS
    CAM_WIFI_SET = 0x21        # {ssid_len: uint8, ssid, password_len: uint8, password} - see
                               # build_camera_wifi_payload. The camera saves this network and joins it,
                               # then answers with CAM_STATUS. Taken only on the camera's own USB (see
                               # q8bot.camera): the radio isn't encrypted, so a password never crosses it


class ReplyType(IntEnum):
    """Robot or camera -> host replies."""
    BATTERY_REPLY = 0x81      # {percent: uint8, millivolts: uint16}
    RECORD_DATA_REPLY = 0x82  # retired alongside RECORD_SEND; do not reuse
    TELEMETRY_REPLY = 0x83    # one joint snapshot - see TELEMETRY_STRUCT
    SENSOR_REPLY = 0x84       # every detected Qwiic sensor's reading - see parse_sensors

    # camera -> host or robot
    CAM_STATUS = 0xA0         # the camera's Wi-Fi - see parse_camera_status


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
# 3 tiers on-robot (see GaitManager.SPEED_TIER_CHAINS).
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


# TELEMETRY_REPLY payload: all 8 joints, 72 bytes. Must match the
# TelemetryPayload struct in q8botone-firmware/robot/include/protocol.h.
#
# Position and current are read fresh on every request; temperature, supply
# voltage and hardware error status are whatever the robot last sampled (it
# refreshes them on its own ~1s timer), since they sit outside the
# contiguous register block the main read covers and move over seconds
# rather than milliseconds. The last 8 bytes count, per servo, the times the
# robot's torque watchdog turned it back on since it started. Protocol 3
# firmware sends only the first 56 bytes, the first protocol 4 builds 72.
TELEMETRY_STRUCT = struct.Struct('<8i8h8b8B8B8B')
TELEMETRY_SIZE = TELEMETRY_STRUCT.size   # 80
_TELEMETRY_72 = struct.Struct('<8i8h8b8B8B')
_TELEMETRY_V3 = struct.Struct('<8i8h8b')  # 56

# Hardware Error Status (register 70) bits on the XL330 - what a nonzero
# 'hw_error' means. A servo with any of them set has switched its own torque
# off, and keeps it off until it restarts.
SERVO_ERROR_BITS = {0: 'input voltage', 2: 'overheating', 4: 'electrical shock', 5: 'overload'}


def describe_servo_error(bits):
    """'input voltage, overload' for a Hardware Error Status value, '' for 0."""
    names = [name for bit, name in SERVO_ERROR_BITS.items() if bits & (1 << bit)]
    names += [f'bit {b}' for b in range(8) if bits & (1 << b) and b not in SERVO_ERROR_BITS]
    return ', '.join(names)


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
        zero with torque off, since none is flowing), 'temperature' (degrees
        C), 'voltage' (each servo's supply, V) and 'hw_error' (Hardware Error
        Status - see describe_servo_error) and 'restored' (how many times
        the robot found that servo's torque off with the motors on -
        restarted, most likely - and turned it back on, since it started;
        wraps at 255), each a tuple of 8. From protocol 3 firmware,
        'voltage', 'hw_error' and 'restored' are None; from the first
        protocol 4 builds, 'restored'. None if the payload is the wrong size.
    """
    restored = None
    if len(payload) in (TELEMETRY_SIZE, _TELEMETRY_72.size):
        full = len(payload) == TELEMETRY_SIZE
        values = (TELEMETRY_STRUCT if full else _TELEMETRY_72).unpack(payload)
        voltage = tuple(v / 10 for v in values[24:32])
        hw_error = values[32:40]
        restored = values[40:48] if full else None
    elif len(payload) == _TELEMETRY_V3.size:
        values = _TELEMETRY_V3.unpack(payload)
        voltage = hw_error = None
    else:
        return None
    return {
        'position': values[0:8],
        'current': values[8:16],
        'temperature': values[16:24],
        'voltage': voltage,
        'hw_error': hw_error,
        'restored': restored,
    }


# SENSOR_REPLY payload, little-endian. Must match the layout comment above
# SensorKind in q8botone-firmware/robot/include/protocol.h:
#
#   uint32  t_ms     robot millis() when sampled
#   uint8   count    sensor blocks that follow
#   count x block:
#     uint8    kind  sensor type - a key of SENSOR_KINDS
#     uint8    addr  its I2C address, telling two of the same kind apart
#     uint8    n     number of values that follow
#     float32  values[n] (NaN while the sensor isn't answering)
#
# n travels with every block, so a kind this table doesn't know still
# parses - adding a sensor type never changes the format.
_SENSOR_HEADER = struct.Struct('<IB')
_SENSOR_BLOCK_HEADER = struct.Struct('<BBB')

# What each kind's values are, in the order the robot sends them. Must match
# protocol.h's SensorKind.
#
#   values  ((name, unit), ...)
#   plots   how to chart them: ((title, (value index, ...)), ...) - values
#           sharing a plot share its axis, like an IMU's x/y/z. None plots
#           each value on its own.
SensorKind = namedtuple('SensorKind', 'name values plots', defaults=(None,))
SENSOR_KINDS = {
    1: SensorKind('INA237', (
        ('Bus voltage', 'V'),
        ('Shunt voltage', 'mV'),
        ('Current', 'mA'),
        ('Power', 'mW'),
        ('Die temperature', '°C'),
    )),
    2: SensorKind('MPU6050', (
        ('Accel X', 'm/s²'),
        ('Accel Y', 'm/s²'),
        ('Accel Z', 'm/s²'),
        ('Gyro X', 'rad/s'),
        ('Gyro Y', 'rad/s'),
        ('Gyro Z', 'rad/s'),
        ('Temperature', '°C'),
    ), plots=(
        ('Acceleration (m/s²)', (0, 1, 2)),
        ('Rotation rate (rad/s)', (3, 4, 5)),
        ('Temperature (°C)', (6,)),
    )),
}

# One sensor's values from one SENSOR_REPLY (a tuple of floats, NaN while it
# isn't answering), and the whole reply. A sensor is identified by
# (kind, address) - stable across replies, so it can key a plotted series.
SensorReading = namedtuple('SensorReading', 'kind address values')
SensorSnapshot = namedtuple('SensorSnapshot', 't_ms sensors')


def sensor_kind(kind, value_count):
    """
    SENSOR_KINDS[kind], or a generic stand-in for a kind this host doesn't
    know yet (newer firmware), with one unnamed entry per value.
    """
    known = SENSOR_KINDS.get(kind)
    if known is not None and len(known.values) == value_count:
        return known
    name = known.name if known is not None else f'Sensor kind {kind}'
    return SensorKind(name, tuple((f'Value {i + 1}', '') for i in range(value_count)))


def parse_sensors(payload: bytes):
    """
    Unpack a SENSOR_REPLY payload.

    Returns:
        SensorSnapshot(t_ms, sensors) - sensors is a tuple of SensorReading,
        empty when the robot has no sensor attached. None if the payload is
        malformed.
    """
    if len(payload) < _SENSOR_HEADER.size:
        return None
    t_ms, count = _SENSOR_HEADER.unpack_from(payload, 0)
    pos = _SENSOR_HEADER.size
    sensors = []
    for _ in range(count):
        if pos + _SENSOR_BLOCK_HEADER.size > len(payload):
            return None
        kind, address, n = _SENSOR_BLOCK_HEADER.unpack_from(payload, pos)
        pos += _SENSOR_BLOCK_HEADER.size
        if pos + 4 * n > len(payload):
            return None
        values = struct.unpack_from(f'<{n}f', payload, pos)
        pos += 4 * n
        sensors.append(SensorReading(kind, address, values))
    if pos != len(payload):
        return None
    return SensorSnapshot(t_ms, tuple(sensors))


class CameraWifi(IntEnum):
    """Where the camera is with its Wi-Fi network (CAM_STATUS). Must match protocol.h."""
    NOT_SET = 0      # no network saved yet
    CONNECTING = 1
    CONNECTED = 2
    FAILED = 3       # couldn't join - see CameraStatus.reason; it keeps trying


# Why joining failed: ESP-IDF's wifi_err_reason_t codes the camera reports,
# for the ones a person can act on. Others show as their number.
WIFI_FAILURES = {
    2: 'authentication expired',
    15: 'wrong password?',          # 4-way handshake timeout
    200: 'lost the signal',
    201: 'network not found',
    202: 'wrong password?',         # authentication failed
    203: 'the router turned it away',
    204: 'wrong password?',         # handshake timeout
    205: "couldn't connect",
}


def describe_wifi_failure(reason):
    """'network not found' for a CAM_STATUS reason, '' for 0."""
    if not reason:
        return ''
    return WIFI_FAILURES.get(reason, f'reason {reason}')


class CameraStatus(namedtuple('CameraStatus', 'protocol wifi reason rssi ip ssid')):
    """
    One CAM_STATUS: protocol (the camera's PROTOCOL_VERSION), wifi
    (CameraWifi), reason (why the last attempt to join failed, 0 if none -
    see describe_wifi_failure), rssi (dBm, None unless connected), ip
    ('10.0.0.101', None unless connected) and ssid (its network's name, ''
    if none is set - never the password).
    """
    __slots__ = ()

    @property
    def url(self):
        """The camera's web page, while it's on Wi-Fi - else None."""
        return f'http://{self.ip}/' if self.wifi == CameraWifi.CONNECTED and self.ip else None

    def describe(self):
        """One readable line, e.g. "Connected to home-wifi - http://10.0.0.101/"."""
        if self.wifi == CameraWifi.NOT_SET:
            return "No Wi-Fi network set"
        if self.wifi == CameraWifi.CONNECTED:
            return f"Connected to {self.ssid} - {self.url}"
        if self.wifi == CameraWifi.FAILED:
            return f"Can't join {self.ssid} yet ({describe_wifi_failure(self.reason)}) - still trying"
        return f"Joining {self.ssid}..."


# CAM_STATUS payload, little-endian. Must match protocol.h:
#   uint8 protocol, uint8 wifi, uint8 reason, int8 rssi, uint8 ip[4],
#   uint8 ssid_len, then that many bytes of SSID
_CAM_STATUS_HEADER = struct.Struct('<BBBb4sB')


def parse_camera_status(payload: bytes):
    """Unpack a CAM_STATUS payload into a CameraStatus - None if malformed."""
    if len(payload) < _CAM_STATUS_HEADER.size:
        return None
    protocol, wifi, reason, rssi, ip, ssid_len = _CAM_STATUS_HEADER.unpack_from(payload, 0)
    if len(payload) != _CAM_STATUS_HEADER.size + ssid_len:
        return None
    ssid = payload[_CAM_STATUS_HEADER.size:].decode('utf-8', errors='replace')
    try:
        wifi = CameraWifi(wifi)
    except ValueError:
        return None
    connected = wifi == CameraWifi.CONNECTED
    return CameraStatus(protocol, wifi, reason, rssi if connected else None,
                        '.'.join(map(str, ip)) if connected else None, ssid)


# What a Wi-Fi network takes: an SSID of 1-32 bytes, and a password that's
# empty (an open network), 8-63 characters, or a 64-digit hex key.
SSID_MAX_BYTES = 32
PASSWORD_MIN = 8
PASSWORD_MAX_BYTES = 63


def build_camera_wifi_payload(ssid: str, password: str) -> bytes:
    """
    CAM_WIFI_SET's payload: {ssid_len, ssid, password_len, password}.

    Raises:
        ValueError, saying what's wrong, for a network the camera couldn't
        join anyway - so it never swaps a working network for a bad one.
    """
    ssid_bytes = ssid.encode('utf-8')
    password_bytes = password.encode('utf-8')
    if not ssid_bytes:
        raise ValueError("Enter the network's name")
    if len(ssid_bytes) > SSID_MAX_BYTES:
        raise ValueError(f"A network name is at most {SSID_MAX_BYTES} bytes - this one is {len(ssid_bytes)}")
    if '\0' in ssid or '\0' in password:
        raise ValueError("The name and password can't contain a null character")
    hex_key = len(password) == 64 and all(c in '0123456789abcdefABCDEF' for c in password)
    if password_bytes and not hex_key:
        if len(password) < PASSWORD_MIN:
            raise ValueError(f"A Wi-Fi password has at least {PASSWORD_MIN} characters - "
                             "leave it empty for an open network")
        if len(password_bytes) > PASSWORD_MAX_BYTES:
            raise ValueError(f"A Wi-Fi password has at most {PASSWORD_MAX_BYTES} characters")
    return (bytes([len(ssid_bytes)]) + ssid_bytes
            + bytes([len(password_bytes)]) + password_bytes)


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
