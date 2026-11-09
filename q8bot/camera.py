'''
Written by yufeng.wu0902@gmail.com

The camera board - a XIAO ESP32S3 running q8botone-firmware/camera - over
its own USB cable, which is where its Wi-Fi network is set:

    from q8bot.camera import Camera

    camera = Camera(Camera.find())
    camera.set_wifi('home-wifi', 'its password')
    print(camera.wait_for_wifi().describe())   # Connected to home-wifi - http://10.0.0.101/

The password only ever goes this way. The robot also passes camera messages
on between its radio and the camera, but that radio link isn't encrypted, so
the camera takes a network only over USB.
'''

import time
from concurrent.futures import ThreadPoolExecutor

import serial

from .helpers import XiaoPortFinder, open_serial
from .protocol import (
    CameraWifi,
    CmdType,
    ReplyType,
    build_camera_wifi_payload,
    build_frame,
    parse_camera_status,
    read_frame,
)

REPLY_TIMEOUT_S = 1.0   # the camera answers at once - this only covers USB
SET_TIMEOUT_S = 3.0     # saving a network writes the camera's flash: measured up to ~1 s
JOIN_TIMEOUT_S = 20.0   # time to join a network: usually 2-6 s, a retry adds 3


class Camera:
    """The camera on one USB port. Opens the port only while it's asking."""

    def __init__(self, port):
        self.port = port

    @staticmethod
    def find_all(timeout=1.5):
        """[(port, firmware version), ...] for every camera on USB."""
        ports = XiaoPortFinder.list_all()
        if not ports:
            return []
        with ThreadPoolExecutor(len(ports)) as pool:
            found = list(zip(ports, pool.map(lambda p: XiaoPortFinder.identify(p, timeout), ports)))
        return [(port, version) for port, (kind, version) in found if kind == 'camera']

    @classmethod
    def find(cls, timeout=1.5):
        """
        The port of the one camera on USB.

        Raises:
            RuntimeError if there's none, or more than one.
        """
        cameras = cls.find_all(timeout)
        if len(cameras) == 1:
            return cameras[0][0]
        if not cameras:
            raise RuntimeError("No camera found on USB - plug in the camera's own USB cable")
        raise RuntimeError(f"More than one camera plugged in ({', '.join(p for p, _ in cameras)}) - "
                           f"pass the one you want, e.g. Camera('{cameras[0][0]}')")

    def status(self, timeout=REPLY_TIMEOUT_S):
        """The camera's Wi-Fi now, as a CameraStatus. Raises TimeoutError if it doesn't answer."""
        return self._ask(build_frame(CmdType.CAM_STATUS_REQUEST, b''), timeout)

    def set_wifi(self, ssid, password, timeout=SET_TIMEOUT_S):
        """
        Saves a network on the camera - it keeps it through restarts - and
        has it join. Returns its status straight after, normally CONNECTING;
        wait_for_wifi() waits for the outcome.

        Raises:
            ValueError for a name or password no network could have (see
                protocol.build_camera_wifi_payload) - nothing is sent.
            TimeoutError if the camera doesn't answer.
            RuntimeError if it answered but kept its old network.
        """
        frame = build_frame(CmdType.CAM_WIFI_SET, build_camera_wifi_payload(ssid, password))
        status = self._ask(frame, timeout)
        if status.ssid != ssid:
            raise RuntimeError(f"The camera kept its old network ({status.ssid or 'none'}) - "
                               "its firmware may be too old for this")
        return status

    def wait_for_wifi(self, timeout=JOIN_TIMEOUT_S, poll=1.0):
        """
        Asks until the camera is on its network, or `timeout` has passed.
        Returns the last CameraStatus - CONNECTED, or how it's failing. A
        camera that's restarting (no answer, or its port gone for a moment)
        is asked again; TimeoutError only if it never answered at all.
        """
        deadline = time.monotonic() + timeout
        status = None
        while True:
            try:
                status = self.status()
            except (TimeoutError, serial.SerialException):
                pass
            if status is not None and (status.wifi == CameraWifi.CONNECTED or time.monotonic() >= deadline):
                return status
            if time.monotonic() >= deadline:
                raise TimeoutError(f"The camera on {self.port} didn't answer")
            time.sleep(poll)

    def _ask(self, frame, timeout):
        # The camera prints its status lines on this same port - read_frame()
        # skips them byte by byte until a frame's start byte turns up.
        with open_serial(self.port, 115200, 0.05) as link:
            link.reset_input_buffer()
            link.write(frame)
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                reply = read_frame(link)
                if reply and reply[0] == ReplyType.CAM_STATUS:
                    status = parse_camera_status(reply[1])
                    if status is not None:
                        return status
        raise TimeoutError(f"The camera on {self.port} didn't answer")
