
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
        self.prev_pos = [90 for i in range(8)]
        self.prev_profile = 0
        self.torque_on = False

        # Initialize serial communication with ESP32-C3
        self.serialHandler = serial.Serial(self.DEVICENAME, self.BAUDRATE)

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
    
    def bulkread(self, addr, len = 4):
        value = [0 for i in range(8)]
        return value, True
    
    def joint_read4(self, joint, addr):
        value = 10
        return value
    
    def joint_read2(self, joint, addr):
        value = 10
        return value

    def check_voltage(self):
        voltage = 3.7
        return voltage

    def dxl2deg(self, angle_dxl):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_friendly = (angle_dxl - self.ZERO_OFFSET) * friendly_per_dxl
        return angle_friendly

    def deg2dxl(self, angle_friendly):
        # Dynamixel joint 0 to 360 deg is 0 to 4096
        friendly_per_dxl = 360.0 / 4096.0 / self.GEAR_RATIO
        angle_dxl = int(angle_friendly / friendly_per_dxl + 0.5) + \
                    self.ZERO_OFFSET
        return angle_dxl
    
    #-------------------#
    # Private Functions #
    #-------------------#
    
    def _set_profile(self, dur_ms):
        return