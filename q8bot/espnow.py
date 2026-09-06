
'''
Written by yufeng.wu0902@gmail.com

Python class that passes arguments to connected ESP32C3 controller, which
then sends commands wirelessly to the robot via ESPNow.

Wire format: see protocol.py - a binary framed protocol
([start][msg type][length][payload][CRC-16]), not raw ASCII.
'''

import struct
import serial

from protocol import CmdType, ReplyType, build_frame, read_frame

DEFAULT_JOINTLIST = [i + 11 for i in range(8)]

class q8_espnow:
    def __init__(self, port, joint_list = DEFAULT_JOINTLIST, baud = 115200):
        self.DEVICENAME = port
        self.BAUDRATE = baud
        self.JOINTS = joint_list
        self.torque_on = False

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE)

        # Tell the controller a host is now driving it, so it switches out
        # of independent (gamepad) mode into relay mode - controller-local,
        # never forwarded to the robot (see CmdType.HOST_CONNECTED).
        self.serialHandler.write(build_frame(CmdType.HOST_CONNECTED, b""))

    def enable_torque(self):
        self.serialHandler.write(build_frame(CmdType.TORQUE_SET, bytes([1])))
        self.torque_on = True
        return True

    def disable_torque(self):
        self.serialHandler.write(build_frame(CmdType.TORQUE_SET, bytes([0])))
        self.torque_on = False
        return True

    def check_battery(self):
        self.serialHandler.write(build_frame(CmdType.BATTERY_REQUEST, b""))
        return True

    def record_data(self):
        return self.move_all([0] * 8, 0, True)

    def finish_recording(self):
        self.serialHandler.write(build_frame(CmdType.RECORD_SEND, b""))
        return True

    def send_jump(self):
        self.serialHandler.write(build_frame(CmdType.JUMP, b""))
        return True

    def send_greet(self):
        """
        Trigger the robot's own native greet() sequence (see q8Dynamixel.cpp)
        instead of routine_generator.py's host-streamed via-point version -
        mainly useful for testing the robot-side CmdType.GREET handler
        through the relay path.
        """
        self.serialHandler.write(build_frame(CmdType.GREET, b""))
        return True

    def run_gait(self, gait_id, direction_id):
        """
        Tell the robot to autonomously run a local gait/direction (see
        protocol.GAIT_IDS/DIRECTION_ORDER) instead of streaming per-tick
        MOVE frames. A raw move_all()/move_mirror() call always stops this
        first on the robot side - no need to call stop_gait() before them.
        """
        self.serialHandler.write(build_frame(CmdType.GAIT_RUN, bytes([gait_id, direction_id])))
        return True

    def stop_gait(self):
        self.serialHandler.write(build_frame(CmdType.GAIT_STOP, b""))
        return True

    def set_led_brightness(self, brightness):
        """
        Set (and have the robot persist to NVS) its RGB LED brightness.

        Args:
            brightness: int, 0-255, clamped
        """
        brightness = max(0, min(255, int(brightness)))
        self.serialHandler.write(build_frame(CmdType.LED_BRIGHTNESS, bytes([brightness])))
        return True

    def read_reply(self, timeout=None):
        """
        Read one reply frame (e.g. a pending battery/recorded-data response).

        Returns:
            (ReplyType, payload_bytes) or None if nothing valid was read.
        """
        result = read_frame(self.serialHandler, timeout)
        if result is None:
            return None
        msg_type, payload = result
        try:
            return ReplyType(msg_type), payload
        except ValueError:
            return None

    def move_all(self, joints_pos, dur = 0, record = True):
        # Expects 8 positions in deg. For example: [0, 90, 0, 90, 0, 90, 0, 90]
        try:
            angles_tenths = [round(a * 10) for a in joints_pos]
            payload = struct.pack('<8hHB', *angles_tenths, dur, int(record))
            self.serialHandler.write(build_frame(CmdType.MOVE, payload))
        except Exception:
            return False
        return True

    def move_mirror(self, joint_pos, dur = 0):
        # Expects a pair of pos for one leg, which will be mirrored 4times.
        mirrored_pos = []
        for i in range(4):
            mirrored_pos.append(joint_pos[0])
            mirrored_pos.append(joint_pos[1])
        return self.move_all(mirrored_pos, dur, False)
