
'''
Written by yufeng.wu0902@gmail.com

Python class that passes arguments to connected ESP32C3 controller, which
then sends commands wirelessly to the robot via ESPNow.

Wire format: see protocol.py - a binary framed protocol
([start][msg type][length][payload][CRC-16]), not raw ASCII.

Safe to use from more than one thread: every serial access goes through one
lock, so Q8Bot's background tick thread (see robot.py) can stream frames
while the caller's thread issues commands.
'''

import struct
import threading

import serial

from .protocol import CmdType, ReplyType, build_frame, read_frame

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

# Without a read timeout, pyserial blocks forever mid-frame if only part of a
# reply has arrived - which would hold the lock and stall the sender. Short
# enough not to stall, long enough for the rest of a frame already in flight;
# read_frame() returns None on a partial read and the caller retries.
DEFAULT_READ_TIMEOUT = 0.05


class q8_espnow:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 115200,
                 read_timeout = DEFAULT_READ_TIMEOUT):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.torque_on = False

        # Guards the serial port. Reentrant so a method built out of other
        # public methods (e.g. move_mirror -> move_all) doesn't deadlock.
        self._lock = threading.RLock()

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE,
                                           timeout=read_timeout)

        # Tell the controller a host is now driving it, so it switches out
        # of independent (gamepad) mode into relay mode - controller-local,
        # never forwarded to the robot (see CmdType.HOST_CONNECTED).
        self._send(CmdType.HOST_CONNECTED)

    #-------------------#
    # Connection        #
    #-------------------#

    @property
    def is_open(self):
        return self.serialHandler.is_open

    def close(self):
        """
        Close the serial port. Sends nothing first - stopping the robot (and
        dropping torque, if that's wanted) is the caller's decision; see
        Q8Bot.close() for the policy version.

        Safe to call more than once.
        """
        with self._lock:
            if self.serialHandler.is_open:
                self.serialHandler.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    #-------------------#
    # Commands          #
    #-------------------#

    def enable_torque(self):
        self._send(CmdType.TORQUE_SET, bytes([1]))
        self.torque_on = True
        return True

    def disable_torque(self):
        self._send(CmdType.TORQUE_SET, bytes([0]))
        self.torque_on = False
        return True

    def set_damping(self):
        """
        Keep torque on but at a much lower position gain (see firmware's
        q8Dynamixel::DAMPING_GAIN) - a joint pushed away from its last
        commanded goal yields instead of holding rigid. Q8Bot re-issues the
        current pose as that goal continuously while this is active (see
        Q8Bot.set_joint_mode()), so a hand-posed joint stays where it's put
        rather than easing back toward wherever damping started.
        """
        self._send(CmdType.TORQUE_SET, bytes([2]))
        self.torque_on = True
        return True

    def check_battery(self):
        return self._send(CmdType.BATTERY_REQUEST)

    def request_telemetry(self):
        """
        Ask for one joint snapshot. The robot answers with a TELEMETRY_REPLY
        (see protocol.parse_telemetry) - one request, one reply, so there is
        no streaming state on the robot to get stuck on.
        """
        return self._send(CmdType.TELEMETRY_REQUEST)

    def send_jump(self):
        return self._send(CmdType.JUMP)

    def send_greet(self):
        """
        Trigger the robot's own native greet() sequence (see q8Dynamixel.cpp)
        instead of routine_generator.py's host-streamed via-point version -
        mainly useful for testing the robot-side CmdType.GREET handler
        through the relay path.
        """
        return self._send(CmdType.GREET)

    def run_gait(self, gait_id, direction_id):
        """
        Tell the robot to autonomously run a local gait/direction (see
        protocol.GAIT_IDS/DIRECTION_ORDER) instead of streaming per-tick
        MOVE frames. A raw move_all()/move_mirror() call always stops this
        first on the robot side - no need to call stop_gait() before them.
        """
        return self._send(CmdType.GAIT_RUN, bytes([gait_id, direction_id]))

    def stop_gait(self):
        return self._send(CmdType.GAIT_STOP)

    def set_led_brightness(self, brightness):
        """
        Set (and have the robot persist to NVS) its RGB LED brightness.

        Args:
            brightness: int, 0-255, clamped
        """
        brightness = max(0, min(255, int(brightness)))
        return self._send(CmdType.LED_BRIGHTNESS, bytes([brightness]))

    def move_all(self, joints_pos, dur = 0):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        #
        # The trailing byte was a "record this frame" flag. Recording is gone,
        # but the byte stays in the layout so the controller's own MOVE
        # builder (firmware/q8bot_controller) doesn't have to change with it.
        try:
            angles_tenths = [round(a * 10) for a in joints_pos]
            payload = struct.pack('<8hHB', *angles_tenths, dur, 0)
        except Exception:
            return False
        return self._send(CmdType.MOVE, payload)

    def move_mirror(self, joint_pos, dur = 0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur)

    #-------------------#
    # Replies           #
    #-------------------#

    @property
    def bytes_waiting(self):
        """Bytes available to read - 0 if the port is closed."""
        with self._lock:
            if not self.serialHandler.is_open:
                return 0
            return self.serialHandler.in_waiting

    def read_reply(self):
        """
        Read one reply frame (e.g. a pending battery response).

        Returns:
            (ReplyType, payload_bytes) or None if nothing valid was read.
        """
        with self._lock:
            if not self.serialHandler.is_open:
                return None
            result = read_frame(self.serialHandler)

        if result is None:
            return None
        msg_type, payload = result
        try:
            return ReplyType(msg_type), payload
        except ValueError:
            return None

    #-------------------#
    # Private Functions #
    #-------------------#

    def _send(self, cmd, payload = b""):
        """Frame and write one command. False if the port is closed."""
        with self._lock:
            if not self.serialHandler.is_open:
                return False
            self.serialHandler.write(build_frame(cmd, payload))
        return True
