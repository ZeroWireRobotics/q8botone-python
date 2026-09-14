'''
Written by yufeng.wu0902@gmail.com

The Q8bot desktop control window.

Connection, the Debug Console, gamepad control and the Robot Information
readouts are live; it owns a Q8Bot the same way operate.py does and drives it
from the same InputHandler. The Manual Control sliders are still placeholders
(see panels.py).

One QTimer does everything: drains the log queue, polls bot.connected and
the battery, and reads the pad. Nothing calls back from Q8Bot's tick thread,
which is why Q8Bot exposes state as properties rather than signals.

Unlike operate.py's loop, nothing here may block: buttons fire on the press
edge rather than being debounced with time.sleep(), and the scripted
routines run on a worker so the window keeps painting.
'''

import logging
import threading
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QMainWindow,
    QStatusBar,
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
)
from .viewer import ViewerPanel
from .widgets import StatusDot

# Fast enough that a stick flick reaches the robot promptly. The gait itself
# is timed by Q8Bot's own thread, so this only has to carry changes.
POLL_INTERVAL_MS = 20

# Right-stick stance trim while idle, in mm at full deflection. Same values
# operate.py settled on by hand on hardware.
STANCE_TRIM_MAX_DX = 25.0
STANCE_TRIM_MAX_DY = 15.0

# Opening the COM port only proves the ESP32C3 dongle is plugged in - the
# robot is a separate device over ESP-NOW and the protocol has no ACK, so a
# port opens just as happily with the robot switched off. A battery reply is
# the cheapest thing that can only come back from a robot that is powered and
# paired, so the indicator is driven by that round trip instead.
BATTERY_POLL_SECONDS = 3.0
ROBOT_SILENT_SECONDS = 8.0   # ~2 missed replies before we stop claiming it's there

# Joint telemetry, on for as long as the window has a robot. Costs the servo
# bus about one percent (the read lands on 1 tick in 20, after the gait write,
# so it only uses slack) and ESP-NOW about 2 KB/s.
TELEMETRY_HZ = 10


class MainWindow(QMainWindow):
    def __init__(self, log_handler):
        super().__init__()
        self.setWindowTitle("Q8bot Control")
        self.resize(1180, 860)

        self._log_handler = log_handler
        self.bot = None
        self.joystick = None
        self.input = None          # InputHandler once a pad is connected
        self._buttons = {}         # last-seen state, for press-edge detection
        self._routine = None       # worker running a blocking routine
        self._port = None
        self._connected_at = 0.0
        self._last_request_at = 0.0
        self._last_reply_at = None   # None until the robot has ever answered

        self.connection = ConnectionPanel(self._list_gamepads)
        self.console = DebugConsole()
        self.manual = ManualControlPanel()
        self.viewer = ViewerPanel()
        self.info = RobotInfoPanel()

        # One grid rather than two independent columns: Manual Control and
        # Robot Information share row 2, which is what makes their Joint 1-8
        # rows line up across the gap (both grids also use the same row pitch,
        # see panels.PANEL_ROW_HEIGHT). The console and viewer take the slack.
        central = QWidget()
        grid = QGridLayout(central)
        grid.addWidget(self.connection, 0, 0)
        grid.addWidget(self.console, 1, 0)
        grid.addWidget(self.manual, 2, 0)
        grid.addWidget(self.viewer, 0, 1, 2, 1)
        grid.addWidget(self.info, 2, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(1, 1)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar())
        self._set_status("Not connected")

        self.connection.robot_connect_requested.connect(self._connect_robot)
        self.connection.robot_disconnect_requested.connect(self._disconnect_robot)
        self.connection.gamepad_connect_requested.connect(self._connect_gamepad)
        self.connection.gamepad_disconnect_requested.connect(self._disconnect_gamepad)
        self.connection.mode_changed.connect(self._on_mode_changed)
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
            self.connection.set_robot_status(
                StatusDot.ERROR, "Port failed", f"Could not open {port}: {exc}")
            return

        self._port = port
        self._last_reply_at = None
        self._connected_at = time.monotonic()
        self._last_request_at = 0.0  # so the first request goes out immediately
        self.connection.set_robot_connected(True)
        self.connection.set_robot_status(
            StatusDot.PENDING, "Waiting for robot...",
            f"{port} is open. Waiting for the robot to answer - is it powered on and paired?")
        self._set_status(f"{port} open - waiting for the robot")
        Q8Logger.info(f"{port} open. Gaits: {', '.join(self.bot.selectable)}")

        # Torque on and into the stance, as operate.py does on startup - the
        # robot has to be holding itself up before any of this can drive it.
        self.bot.set_torque(True)
        self.bot.stand(1000)
        Q8Logger.info("Torque on, moving to stance.")

        # Telemetry runs whenever this window holds a robot. It's
        # request/response, so closing the window simply stops asking - there
        # is no streaming state left running on the robot.
        self.bot.telemetry_hz = TELEMETRY_HZ

    def _disconnect_robot(self):
        if self.bot is not None:
            self.bot.close()
            self.bot = None
        self._last_reply_at = None
        self.connection.set_robot_connected(False)
        self.connection.set_robot_status(StatusDot.OFF, "Not connected")
        self.info.set_battery(None)
        self.info.set_telemetry(None)
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

        try:
            used, joystick, mapping = init_selected_joystick(index)
        except Exception as exc:
            # SDL can fail on a device that vanished between enumeration and
            # here. Never let that escape into Qt's slot dispatch.
            Q8Logger.error(f"Could not open that gamepad: {exc}")
            self.connection.set_gamepad_status(StatusDot.ERROR, "Failed to open", str(exc))
            return

        if not used:
            Q8Logger.error("That gamepad could not be initialised.")
            self.connection.set_gamepad_status(StatusDot.ERROR, "Failed to open")
            return

        from ..input_handler import InputHandler
        self.joystick = joystick
        self.input = InputHandler(True, joystick, mapping)
        self._buttons.clear()
        self.connection.set_gamepad_connected(True)
        # Switch to Xbox mode straight away - connecting a pad and then
        # finding it does nothing until you flip a toggle is a poor welcome.
        self.connection.mode_toggle.setChecked(False)
        # An unrecognised pad still works, just on guessed button positions -
        # worth flagging in the indicator rather than only in the log.
        if mapping['recognized']:
            self.connection.set_gamepad_status(
                StatusDot.LIVE, "Connected", f"{joystick.get_name()} - mapping recognised")
            Q8Logger.info(f"Gamepad connected: {joystick.get_name()}.")
        else:
            self.connection.set_gamepad_status(
                StatusDot.PENDING, "Generic mapping",
                f"{joystick.get_name()} is not in control_config.py - "
                f"using the generic fallback, so buttons may be wrong.")
            Q8Logger.warning(f"Gamepad connected: {joystick.get_name()} "
                             f"(unrecognised - using the generic fallback).")

    def _disconnect_gamepad(self):
        if self.bot is not None and self.bot.connected:
            self.bot.idle()  # don't leave it walking with nothing driving it
        self.input = None
        self._buttons.clear()
        if self.joystick is not None:
            self.joystick.quit()
            self.joystick = None
        self.connection.set_gamepad_connected(False)
        self.connection.set_gamepad_status(StatusDot.OFF, "Not connected")
        self.connection.mode_toggle.setChecked(True)
        Q8Logger.info("Gamepad disconnected.")

    def _on_mode_changed(self, pc_mode):
        self.manual.set_available(pc_mode)
        # Leaving Xbox mode drops control, so stop rather than coast.
        if pc_mode and self.bot is not None and self.bot.connected:
            self.bot.idle()

    #-------------------#
    # Polling           #
    #-------------------#

    def _poll(self):
        for timestamp, level, message in self._log_handler.drain():
            self.console.append(timestamp, level, message)

        if self.bot is None:
            return

        # Q8Bot latches itself disconnected if the link dies on its own; the
        # UI finds out here rather than through a callback off its thread.
        if not self.bot.connected:
            Q8Logger.error("Robot connection lost.")
            self.bot = None
            self._last_reply_at = None
            self.connection.set_robot_connected(False)
            self.connection.set_robot_status(
                StatusDot.ERROR, "Connection lost", "The link dropped - check the cable.")
            self.info.set_battery(None)
            self.info.set_telemetry(None)
            self._set_status("Connection lost")
            return

        self._poll_robot_liveness()
        self.info.set_telemetry(self.bot.telemetry)
        self._poll_gamepad()

    #-------------------#
    # Gamepad control   #
    #-------------------#

    def _poll_gamepad(self):
        """
        Turn gamepad state into Q8Bot commands - the same mapping operate.py
        uses, minus its time.sleep() debounces, which would freeze the window.
        Buttons fire on the press edge instead.
        """
        if self.input is None or self.connection.pc_mode:
            return  # the pad only drives in Xbox mode
        if self._routine_running():
            return  # a scripted routine owns the robot until it finishes

        direction = self.input.get_movement_direction()
        if direction:
            self.bot.move(direction, self.input.get_movement_magnitude())
            return

        if self.input.is_trigger_pressed('trigger_left'):
            self.bot.trot_in_place()
            return

        self.bot.idle()
        self.bot.stance_trim(*self.input.get_stance_offset(
            STANCE_TRIM_MAX_DX, STANCE_TRIM_MAX_DY))
        self._poll_gamepad_actions()

    def _poll_gamepad_actions(self):
        if self._pressed('reset'):
            Q8Logger.info("Reset to stance.")
            self.bot.stand(500)
        elif self._pressed('toggle_torque'):
            on = self.bot.set_torque(not self.bot.torque)
            Q8Logger.info(f"Torque {'ON' if on else 'OFF'}.")
        elif self._pressed('switch_gait'):
            new_gait = self.bot.next_gait()
            Q8Logger.info(f"Switched to {new_gait}." if new_gait
                          else "Failed to switch gait.")
        elif self._pressed('battery'):
            self.bot.request_battery()
        elif self._pressed('jump'):
            self._run_routine("Jump", self.bot.jump)
        elif self._pressed('greet'):
            self._run_routine("Greet", self.bot.greet)
        elif self._trigger_pressed('trigger_right'):
            self._run_routine("Shoot", lambda: self.bot.shoot(on_kick=self._rumble))

    def _pressed(self, action):
        """True on the press edge only, so a held button fires once."""
        down = self.input.is_action_pressed(action)
        was_down = self._buttons.get(action, False)
        self._buttons[action] = down
        return down and not was_down

    def _trigger_pressed(self, axis):
        down = self.input.is_trigger_pressed(axis)
        was_down = self._buttons.get(axis, False)
        self._buttons[axis] = down
        return down and not was_down

    def _rumble(self):
        if self.joystick is None:
            return
        try:
            self.joystick.rumble(0.5, 0.5, 100)
        except Exception:
            pass  # not every pad supports it - never worth failing a kick over

    def _routine_running(self):
        return self._routine is not None and self._routine.is_alive()

    def _run_routine(self, label, call):
        """
        Jump, greet and shoot block for seconds and hold Q8Bot's lock, so they
        run on a worker rather than freezing the window. Input is ignored until
        the routine finishes (see _poll_gamepad).
        """
        Q8Logger.info(f"{label}...")
        self.bot.idle()
        self._routine = threading.Thread(target=call, name=f'q8bot-{label.lower()}',
                                         daemon=True)
        self._routine.start()

    def _poll_robot_liveness(self):
        now = time.monotonic()

        battery = self.bot.take_battery_reading()
        if battery is not None:
            first_reply = self._last_reply_at is None
            self._last_reply_at = now
            self.info.set_battery(battery, self.bot.battery_volts)
            # No percentage here - Robot Information already shows it. This
            # label's job is whether the robot is answering, nothing more.
            self.connection.set_robot_status(
                StatusDot.LIVE, "Robot online",
                f"{self._port}: robot answering, battery {battery}%")
            self._set_status(f"Connected on {self._port} - {self.bot.gait} - battery {battery}%")
            if first_reply:
                Q8Logger.info(f"Robot is answering. Battery {battery}%.")

        if now - self._last_request_at >= BATTERY_POLL_SECONDS:
            self._last_request_at = now
            self.bot.request_battery()

        silent_for = now - (self._last_reply_at or self._connected_at)
        if self._last_reply_at is not None and silent_for >= ROBOT_SILENT_SECONDS:
            self.connection.set_robot_status(
                StatusDot.PENDING, "Robot not answering",
                f"{self._port} is still open, but the robot stopped replying "
                f"{silent_for:.0f}s ago.")
            self.info.set_battery(None)
            self._set_status(f"{self._port} open - robot not answering")

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
