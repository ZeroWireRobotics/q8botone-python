'''
Written by yufeng.wu0902@gmail.com

Logging, and finding the controller's serial port.
'''

import serial
import serial.tools.list_ports
import logging
import sys
from concurrent.futures import ThreadPoolExecutor


class Q8Logger:
    """
    Logging wrapper for Q8bot application.

    Provides consistent logging across all modules with configurable log levels.
    Default level is INFO, use debug mode for detailed output.

    Plain `logging` only - it draws nothing itself. A front end that wants to
    display messages attaches its own logging.Handler via add_handler(); see
    log_overlay.PygameLogOverlay for the one operate.py's window uses.
    """

    _logger = None

    def __init__(self, debug=False):
        """
        Initialize Q8Logger.

        Args:
            debug: bool, if True sets level to DEBUG, otherwise INFO
        """
        if Q8Logger._logger is None:
            Q8Logger._logger = logging.getLogger('q8bot')
            Q8Logger._logger.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create console handler
            handler = logging.StreamHandler(sys.stdout)
            handler.setLevel(logging.DEBUG if debug else logging.INFO)

            # Create formatter
            formatter = logging.Formatter('%(levelname)s: %(message)s')
            handler.setFormatter(formatter)

            # Add handler
            Q8Logger._logger.addHandler(handler)

    @staticmethod
    def get_logger():
        """Get the logger instance."""
        if Q8Logger._logger is None:
            Q8Logger()
        return Q8Logger._logger

    @staticmethod
    def add_handler(handler):
        """
        Attach an extra logging.Handler - e.g. a UI overlay that displays
        messages on screen (see log_overlay.PygameLogOverlay). Keeps the
        logger name in one place so front ends don't hardcode it.
        """
        Q8Logger.get_logger().addHandler(handler)

    @staticmethod
    def debug(msg):
        """Log debug message."""
        Q8Logger.get_logger().debug(msg)

    @staticmethod
    def info(msg):
        """Log info message."""
        Q8Logger.get_logger().info(msg)

    @staticmethod
    def warning(msg):
        """Log warning message."""
        Q8Logger.get_logger().warning(msg)

    @staticmethod
    def error(msg):
        """Log error message."""
        Q8Logger.get_logger().error(msg)


def open_serial(port, baudrate=115200, timeout=None):
    """
    Open `port` without touching the board's reset line.

    On the ESP32-C3's built-in USB port, DTR going low while RTS is high
    resets the chip - the trick flashing tools use on purpose. A port opened
    the default way has both lines high, and closing it drops them in exactly
    that order, so every close rebooted the board: a robot plugged in for
    charging restarted whenever it was asked what it was, and the controller
    restarted the moment a script disconnected - often before passing on the
    script's last stop command. Opened with both lines low, nothing ever
    resets, and the controller still relays normally.
    """
    link = serial.Serial()
    link.port = port
    link.baudrate = baudrate
    link.timeout = timeout
    link.dtr = False
    link.rts = False
    link.open()
    return link


class XiaoPortFinder:
    """
    Utility class for detecting and validating Seeed Studio XIAO ESP32C3 devices.

    The XIAO ESP32C3 is identified by its unique USB Vendor ID (VID) and Product ID (PID).
    """

    # Seeed Studio XIAO ESP32C3 USB identifiers
    VID = 0x303A
    PID = 0x1001

    @classmethod
    def find(cls):
        """
        Find the first connected XIAO ESP32C3 device - any of them: the
        robot and the controller look the same here. find_controller()
        asks which is which.

        Returns:
            str: COM port device name (e.g., "COM3" or "/dev/ttyUSB0"), or None if not found

        Example:
            >>> port = XiaoPortFinder.find()
            >>> if port:
            ...     print(f"Found XIAO at {port}")
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == cls.VID and port.pid == cls.PID:
                return port.device
        return None

    @classmethod
    def validate(cls, com_port):
        """
        Validate if a given COM port is a XIAO ESP32C3 device.

        Args:
            com_port (str): COM port device name to check

        Returns:
            bool: True if the port is a XIAO device, False otherwise

        Example:
            >>> if XiaoPortFinder.validate("COM3"):
            ...     print("COM3 is a valid XIAO device")
        """
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.device == com_port and port.vid == cls.VID and port.pid == cls.PID:
                return True
        return False

    @classmethod
    def list_all(cls):
        """
        List all connected XIAO ESP32C3 devices.

        Returns:
            list: List of COM port device names for all connected XIAO devices

        Example:
            >>> devices = XiaoPortFinder.list_all()
            >>> print(f"Found {len(devices)} XIAO device(s)")
        """
        ports = serial.tools.list_ports.comports()
        xiao_ports = []
        for port in ports:
            if port.vid == cls.VID and port.pid == cls.PID:
                xiao_ports.append(port.device)
        return xiao_ports

    # The USB ID above is Espressif's built-in USB port, shared by the robot,
    # the controller and nearly every other ESP32-C3/S3 board - so it can only
    # narrow the search. identify() asks a board what it is, the same way the
    # firmware flasher does: the 'i' serial command, harmless to both boards.
    # Connecting to the robot by mistake isn't harmless - it reads single
    # letters on USB as debug keys, and frame bytes can land on them.

    @classmethod
    def identify(cls, port, timeout=1.5):
        """
        What's on `port`.

        Returns:
            (kind, version): kind is 'controller', 'robot', 'other' (a board
            that says it's something else), 'silent' (no answer - firmware
            from before boards reported themselves, or not a Q8botOne board)
            or 'busy' (another program has the port open). version is the
            firmware version it reported, or None.
        """
        from .firmware_flasher import identify_device   # imported here: it needs nothing from helpers
        try:
            found = identify_device(port, timeout)
        except serial.SerialException:
            return 'busy', None
        if found is None:
            return 'silent', None
        if found.target in ('controller', 'robot'):
            return found.target, found.version
        return 'other', found.version

    @staticmethod
    def describe(kind, version=None):
        """A short, readable label for an identify() result."""
        text = {'controller': 'controller', 'robot': 'robot', 'other': 'not a Q8botOne board',
                'silent': 'no answer', 'busy': 'in use by another program'}[kind]
        return f"{text} {version}" if version and kind in ('controller', 'robot') else text

    @classmethod
    def find_controller(cls, timeout=1.5):
        """
        The port of the one Q8botOne controller plugged in, found by asking
        each XIAO ESP32-C3 what it is. A lone board that doesn't answer is
        taken to be a controller on older firmware, as it always was.

        Raises:
            RuntimeError, saying what's plugged in where, if there's no
            controller or more than one.
        """
        ports = cls.list_all()
        if not ports:
            raise RuntimeError("No controller found on USB - is it plugged in?")
        with ThreadPoolExecutor(len(ports)) as pool:
            found = dict(zip(ports, pool.map(lambda p: cls.identify(p, timeout), ports)))

        controllers = [p for p, (kind, _) in found.items() if kind == 'controller']
        if len(controllers) == 1:
            return controllers[0]
        if not controllers and len(ports) == 1 and found[ports[0]][0] == 'silent':
            Q8Logger.warning(f"{ports[0]} didn't say what it is (older firmware?) - "
                             "assuming it's the controller")
            return ports[0]

        plugged_in = ', '.join(f"{p}: {cls.describe(*found[p])}" for p in ports)
        if controllers:
            raise RuntimeError(f"More than one controller plugged in ({plugged_in}) - pass the one "
                               f"you want, e.g. Q8Bot.connect('{controllers[0]}')")
        raise RuntimeError(f"No controller found ({plugged_in}) - plug the controller in, "
                           f"or pass its port, e.g. Q8Bot.connect('COM5')")



