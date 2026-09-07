'''
Written by yufeng.wu0902@gmail.com

The Q8bot desktop control window.

Only the Connection panel is wired up so far - it owns a Q8Bot the same way
operate.py does, and the Debug Console is live alongside it so connecting
actually tells you something. Everything else is a placeholder (see
panels.py).

Nothing calls back from Q8Bot's tick thread: one QTimer polls bot.connected
and drains the log queue, which is why Q8Bot exposes state as properties
rather than signals.
'''

import logging

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QMainWindow,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from ..espnow import q8_espnow
from ..helpers import Q8Logger
from ..robot import Q8Bot
from .log_bridge import QueuedLogHandler
from .panels import (
    ConnectionPanel,
    DebugConsole,
    ManualControlPanel,
    RobotInfoPanel,
    ViewerPanel,
)

POLL_INTERVAL_MS = 50


class MainWindow(QMainWindow):
    def __init__(self, log_handler):
        super().__init__()
        self.setWindowTitle("Q8bot Control")
        self.resize(1180, 720)

        self._log_handler = log_handler
        self.bot = None
        self.joystick = None

        self.connection = ConnectionPanel(self._list_gamepads)
        self.console = DebugConsole()
        self.manual = ManualControlPanel()
        self.viewer = ViewerPanel()
        self.info = RobotInfoPanel()

        # Two independent columns rather than a shared grid: the panels don't
        # line up row-for-row in the mockup, and a grid forces them to.
        left = QVBoxLayout()
        left.addWidget(self.connection, 0)
        left.addWidget(self.console, 2)
        left.addWidget(self.manual, 2)

        right = QVBoxLayout()
        right.addWidget(self.viewer, 3)
        right.addWidget(self.info, 0)

        central = QWidget()
        columns = QHBoxLayout(central)
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar())
        self._set_status("Not connected")

        self.connection.robot_connect_requested.connect(self._connect_robot)
        self.connection.robot_disconnect_requested.connect(self._disconnect_robot)
        self.connection.gamepad_connect_requested.connect(self._connect_gamepad)
        self.connection.gamepad_disconnect_requested.connect(self._disconnect_gamepad)
        self.connection.mode_changed.connect(self.manual.set_available)
        self.manual.set_available(self.connection.pc_mode)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(POLL_INTERVAL_MS)

        Q8Logger.info("Q8bot control ready. Pick a port and hit Connect.")

    #-------------------#
    # Robot             #
    #-------------------#

    def _connect_robot(self, port):
        try:
            self.bot = Q8Bot(q8_espnow(port))
        except Exception as exc:
            Q8Logger.error(f"Could not connect on {port}: {exc}")
            return

        self.connection.set_robot_connected(True)
        self._set_status(f"Connected on {port} - {self.bot.gait}")
        Q8Logger.info(f"Connected on {port}. Gaits: {', '.join(self.bot.selectable)}")

    def _disconnect_robot(self):
        if self.bot is not None:
            self.bot.close()
            self.bot = None
        self.connection.set_robot_connected(False)
        self._set_status("Not connected")
        Q8Logger.info("Disconnected.")

    #-------------------#
    # Gamepad           #
    #-------------------#

    def _list_gamepads(self):
        from ..input_handler import list_available_joysticks
        try:
            return list_available_joysticks()
        except Exception as exc:
            Q8Logger.warning(f"Could not enumerate gamepads: {exc}")
            return []

    def _connect_gamepad(self, index):
        from ..input_handler import init_selected_joystick
        if index is None:
            Q8Logger.warning("No gamepad selected.")
            return

        used, joystick, mapping = init_selected_joystick(index)
        if not used:
            Q8Logger.error("That gamepad could not be initialised.")
            return

        self.joystick = joystick
        self.connection.set_gamepad_connected(True)
        known = "recognised" if mapping['recognized'] else "using the generic fallback"
        Q8Logger.info(f"Gamepad connected: {joystick.get_name()} ({known}).")

    def _disconnect_gamepad(self):
        if self.joystick is not None:
            self.joystick.quit()
            self.joystick = None
        self.connection.set_gamepad_connected(False)
        Q8Logger.info("Gamepad disconnected.")

    #-------------------#
    # Polling           #
    #-------------------#

    def _poll(self):
        for timestamp, level, message in self._log_handler.drain():
            self.console.append(timestamp, level, message)

        # Q8Bot latches itself disconnected if the link dies on its own; the
        # UI finds out here rather than through a callback off its thread.
        if self.bot is not None and not self.bot.connected:
            Q8Logger.error("Robot connection lost.")
            self.bot = None
            self.connection.set_robot_connected(False)
            self._set_status("Connection lost")

    def _set_status(self, text):
        self.statusBar().showMessage(text)

    def closeEvent(self, event):
        self._timer.stop()
        if self.bot is not None:
            self.bot.close()
            self.bot = None
        if self.joystick is not None:
            self.joystick.quit()
            self.joystick = None
        super().closeEvent(event)


def main(argv=None):
    """Launch the control window."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(description='Q8bot desktop control')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args(argv)

    Q8Logger(debug=args.debug)
    handler = QueuedLogHandler(level=logging.DEBUG if args.debug else logging.INFO)
    Q8Logger.add_handler(handler)

    app = QApplication(sys.argv[:1])
    window = MainWindow(handler)
    window.show()
    return app.exec()
