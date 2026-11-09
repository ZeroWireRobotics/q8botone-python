'''
Written by yufeng.wu0902@gmail.com

The Q8bot desktop control window.

It owns a Q8Bot (see robot.py) and drives it from the Manual Control pads or
a gamepad (InputHandler); the panels themselves are in panels.py.

One QTimer does everything: drains the log queue, polls bot.connected and
the battery, and reads the pad. Nothing calls back from Q8Bot's tick thread,
which is why Q8Bot exposes state as properties rather than signals.

Nothing here may block: buttons fire on the press edge rather than being
debounced with time.sleep(), and the scripted routines run on a worker so
the window keeps painting.
'''

import json
import logging
import math
import threading
import time
from pathlib import Path

from PySide6.QtCore import QEvent, QSettings, QTimer, QUrl
from PySide6.QtGui import QAction, QActionGroup, QDesktopServices, QFont, QFontInfo, QIcon, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QMessageBox,
    QStatusBar,
)

from .. import __version__, PROTOCOL_VERSION, imu, routine_store
from ..control_config import get_joystick_direction, get_joystick_magnitude
from ..espnow import q8_espnow
from ..helpers import Q8Logger
from ..protocol import describe_servo_error, sensor_kind
from ..robot import JUMP_BATTERY_THRESHOLD, JointMode, Q8Bot
from . import theme
from .log_bridge import QueuedLogHandler
from .panels import (
    ConnectionPanel,
    DebugConsole,
    ManualControlPanel,
    RobotInfoPanel,
)
from .layout import AUTOMATIC, COLUMNS, STACK, PanelLayout
from .sensor_window import SensorWindow
from .teach_dialog import TeachRoutineDialog
from .viewer import FOLLOW_IMU_HZ, FOLLOW_TELEMETRY_HZ, SIMULATED_ROBOT, ViewerPanel
from .widgets import StatusDot, set_base_point_size

# Fast enough that a stick flick reaches the robot promptly. The gait itself
# is timed by Q8Bot's own thread, so this only has to carry changes.
POLL_INTERVAL_MS = 20

# The ZeroWire Robotics mark, for every window's title bar and the taskbar.
APP_ICON = Path(__file__).resolve().parent.parent / 'assets' / 'zwr-icon.png'

# Right-stick stance trim while idle, in mm at full deflection. Values
# settled on by hand on hardware.
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
# so it only uses slack) and ESP-NOW about 2 KB/s. Raised to the viewer's
# FOLLOW_TELEMETRY_HZ while its model follows the robot.
TELEMETRY_HZ = 10

# Qwiic sensors: a slow request whenever a robot is connected keeps the
# Robot Information status ("No sensor connected" / which sensor) current;
# the Sensors window raises it while open, and the 3D view's model further
# (FOLLOW_IMU_HZ) while it follows the robot. A robot whose firmware predates
# sensor support (protocol < 2) ignores the request, so silence past
# SENSOR_SILENT_SECONDS is reported rather than waited on forever.
SENSOR_PRESENCE_HZ = 1
SENSOR_LIVE_HZ = 10
SENSOR_SILENT_SECONDS = 5.0

# The move pad springs back through small values on release; ignore those
# rather than firing a last twitch of movement on the way to centre.
MOVE_PAD_DEADZONE = 0.15

# View > Text Size: the window's text, and everything sized from it, scaled
# from the font Qt picks at startup. Remembered between runs (QSettings).
TEXT_SIZES = (0.85, 1.0, 1.15, 1.3, 1.5, 1.75, 2.0)

# Help menu. The docs site's pages, by MkDocs's URL for each.
DOCS_URL = 'https://docs.zerowirerobotics.com/'
TROUBLESHOOTING_URL = DOCS_URL + 'using/troubleshooting/'
RELEASES_URL = 'https://github.com/ZeroWireRobotics/q8botone-python/releases'
ISSUES_URL = 'https://github.com/ZeroWireRobotics/q8botone-python/issues'
WEBSITE_URL = 'https://zerowirerobotics.com/'

SETTINGS_ORG, SETTINGS_APP = 'ZeroWireRobotics', 'Q8botOne'


class MainWindow(QMainWindow):
    def __init__(self, log_handler, settings=None):
        """settings: a QSettings to remember the window's layout and text
        size in - the user's own (per user, per machine) if left out."""
        super().__init__()
        self.setWindowTitle("Q8bot Control")
        self.setMinimumSize(480, 360)   # smaller than the panels: the window scrolls
        self._settings = settings if settings is not None else QSettings(SETTINGS_ORG, SETTINGS_APP)

        # The text size before any panel is built, so each starts out sized for it.
        app = QApplication.instance()
        self._base_font = QFont(app.font())
        set_base_point_size(self._base_font.pointSizeF())
        self._text_scale = 1.0
        self._apply_text_scale(self._saved_float('view/text_scale', 1.0), save=False)
        # Light or dark (View > Theme), also before the panels
        theme.apply(self._settings.value('view/theme', theme.SYSTEM))

        self._log_handler = log_handler
        self.bot = None
        self.joystick = None
        self.input = None          # InputHandler once a pad is connected
        self._buttons = {}         # last-seen state, for press-edge detection
        self._routine = None       # worker running a blocking routine
        self._port = None
        self._simulated = False    # connected to the 3D view's simulated robot, not a real one
        self._connected_at = 0.0
        self._last_request_at = 0.0
        self._last_reply_at = None   # None until the robot has ever answered
        self._shown_gait = None      # what the gait dropdown is currently showing
        self._shown_joint_mode = None  # what the joint mode slider is currently showing
        self._saved_routines = []    # [(name, path), ...] mirroring the reserved rows
        self._teach_dialog = None    # the open TeachRoutineDialog, or None
        self._sensor_window = None   # the open SensorWindow, or None
        self._plotted_snapshot = None  # the newest reading its plot has drawn
        self._servo_errors = None    # each servo's Hardware Error Status as last logged
        self._servo_restored = None  # the robot's torque-restore counts as last logged
        self._motor_window = None    # the open MotorSetupWindow, or None
        self._camera_window = None   # the open CameraWindow, or None

        self.viewer = ViewerPanel()
        self.connection = ConnectionPanel(self._list_gamepads, offer_simulated=self.viewer.can_simulate)
        self.console = DebugConsole()
        self.manual = ManualControlPanel()
        self.info = RobotInfoPanel()

        # Two columns or one, splitters between them, scrolling when the
        # window is smaller than its panels - see layout.py.
        self.panels = PanelLayout(self.connection, self.console, self.viewer, self.manual, self.info)
        self.setCentralWidget(self.panels)

        # The camera's video, one stream for Robot View and the Camera Video
        # window alike, and whether there's a camera to show at all
        from .camera_view import CameraFinder, CameraStream
        self.camera_stream = CameraStream(self)
        self.camera_finder = CameraFinder(self._settings, self._real_robot, self.connection.camera_ports,
                                          self.camera_stream, self)
        self.camera_finder.changed.connect(self._camera_found)

        self._build_menus()
        self._restore_window()

        self.setStatusBar(QStatusBar())
        self._set_status("Not connected")

        self.connection.robot_connect_requested.connect(self._connect_robot)
        self.connection.robot_disconnect_requested.connect(self._disconnect_robot)
        self.connection.gamepad_connect_requested.connect(self._connect_gamepad)
        self.connection.gamepad_disconnect_requested.connect(self._disconnect_gamepad)
        self.connection.mode_changed.connect(self._on_mode_changed)
        self.info.sensors_requested.connect(self._open_sensor_window)
        self.info.servo_clear_requested.connect(self._clear_servo_error)
        self.manual.gait_selected.connect(self._select_gait)
        self.manual.keep_level_toggled.connect(self._keep_level_toggled)
        self.manual.calibrate_imu_requested.connect(self._calibrate_imu)
        self.manual.joint_mode_requested.connect(self._joint_mode_requested)
        self.manual.jump_requested.connect(self._manual_jump)
        self.manual.greet_requested.connect(self._manual_greet)
        self.manual.routine_play_requested.connect(self._play_routine)
        self.manual.routine_teach_requested.connect(self._teach_routine)
        self.manual.routine_edit_requested.connect(self._edit_routine)
        self.manual.routine_delete_requested.connect(self._delete_routine)
        self.manual.set_available(self.connection.pc_mode)

        # Routines are just files on disk - load them at startup rather than
        # only on robot connect, so the rows are populated (if disabled)
        # immediately instead of flashing empty-then-filled after Connect.
        self._reload_routines()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(POLL_INTERVAL_MS)

        Q8Logger.info("Q8bot control ready. Pick a port and hit Connect.")

    #-------------------#
    # Window            #
    #-------------------#

    def _style_menu_bar(self):
        '''Tighter than the style's own spacing between the titles, sized from
        the text - set again after View > Text Size, since a style sheet's
        lengths are worked out once, when it's set.'''
        em = QFontInfo(self.menuBar().font()).pixelSize()
        sheet = f"QMenuBar::item {{ padding: {round(0.3 * em)}px {round(0.55 * em)}px; margin: 0px; }}"
        if sheet != self.menuBar().styleSheet():
            self.menuBar().setStyleSheet(sheet)

    def eventFilter(self, watched, event):
        if watched is self.menuBar() and event.type() == QEvent.FontChange:
            QTimer.singleShot(0, self._style_menu_bar)   # its new font, once it has it
        return super().eventFilter(watched, event)

    def _build_menus(self):
        bar = self.menuBar()
        self._style_menu_bar()
        bar.installEventFilter(self)

        # No File menu just for Quit - closing the window quits; Ctrl+Q too.
        quit_action = QAction("Quit", self)
        quit_action.setShortcut(QKeySequence("Ctrl+Q"))   # QKeySequence.Quit has no key on Windows
        quit_action.triggered.connect(self.close)
        self.addAction(quit_action)

        view = bar.addMenu("&View")
        size_menu = view.addMenu("&Text Size")
        larger = size_menu.addAction("&Larger", lambda: self._step_text_size(+1))
        larger.setShortcuts([QKeySequence("Ctrl+="), QKeySequence.ZoomIn])
        smaller = size_menu.addAction("&Smaller", lambda: self._step_text_size(-1))
        smaller.setShortcuts([QKeySequence.ZoomOut])
        default = size_menu.addAction("&Default", lambda: self._set_text_scale(1.0))
        default.setShortcut(QKeySequence("Ctrl+0"))
        size_menu.addSeparator()
        self._size_actions = QActionGroup(self)
        for scale in TEXT_SIZES:
            action = size_menu.addAction(f"{round(scale * 100)}%")
            action.setCheckable(True)
            action.setData(scale)
            action.triggered.connect(lambda _checked=False, s=scale: self._set_text_scale(s))
            self._size_actions.addAction(action)
        self._check_text_size()

        layout_menu = view.addMenu("&Layout")
        self._layout_actions = QActionGroup(self)
        for mode, label, tip in (
                (AUTOMATIC, "&Automatic", "Two columns when the window is wide enough, else one"),
                (COLUMNS, "&Two Columns", ""),
                (STACK, "&One Column", "")):
            action = layout_menu.addAction(label)
            action.setCheckable(True)
            action.setData(mode)
            action.setStatusTip(tip)
            action.triggered.connect(lambda _checked=False, m=mode: self._set_layout_mode(m))
            self._layout_actions.addAction(action)
        layout_menu.addSeparator()
        layout_menu.addAction("&Reset Panel Sizes", self._reset_panel_sizes)

        theme_menu = view.addMenu("T&heme")
        self._theme_actions = QActionGroup(self)
        for which, label, tip in (
                (theme.SYSTEM, "&System", "Light or dark, as Windows is set"),
                (theme.LIGHT, "&Light", ""),
                (theme.DARK, "&Dark", "")):
            action = theme_menu.addAction(label)
            action.setCheckable(True)
            action.setChecked(which == theme.choice())
            action.setStatusTip(tip)
            action.triggered.connect(lambda _checked=False, w=which: self._set_theme(w))
            self._theme_actions.addAction(action)

        # What the Robot View panel shows. The camera only once there's one
        # (CameraFinder) - greyed out until then.
        robot_view = view.addMenu("&Robot View")
        self._robot_view_actions = QActionGroup(self)
        self._model_view_action = robot_view.addAction("&3D Model")
        self._model_view_action.triggered.connect(self._show_model_view)
        self._camera_view_action = robot_view.addAction("&Camera")
        self._camera_view_action.triggered.connect(self._show_camera_view)
        for action in (self._model_view_action, self._camera_view_action):
            action.setCheckable(True)
            self._robot_view_actions.addAction(action)
        self._model_view_action.setChecked(True)
        self._camera_found(self.camera_finder.host)

        tools = bar.addMenu("&Tools")
        tools.addAction("&Flash Firmware...", self._open_firmware_dialog)
        tools.addAction("&Motor Setup...", self._open_motor_setup)
        tools.addAction("Camera &Video...", self._open_camera_window)
        tools.addAction("&Camera Wi-Fi...", self._open_camera_dialog)
        tools.addSeparator()
        self._sensors_action = tools.addAction("&Sensors...", self._open_sensor_window)
        # Only with readings to show, like the Sensors... button it stands for.
        tools.aboutToShow.connect(
            lambda: self._sensors_action.setEnabled(self.info.sensor_button.isEnabled()))

        help_menu = bar.addMenu("&Help")
        docs = help_menu.addAction("&Documentation", lambda: self._open_url(DOCS_URL))
        docs.setShortcut(QKeySequence.HelpContents)
        help_menu.addAction("&Troubleshooting", lambda: self._open_url(TROUBLESHOOTING_URL))
        help_menu.addAction("&Releases and Downloads", lambda: self._open_url(RELEASES_URL))
        help_menu.addAction("Report a &Problem...", lambda: self._open_url(ISSUES_URL))
        help_menu.addSeparator()
        help_menu.addAction("&About Q8botOne", self._show_about)

    def _saved_float(self, key, default):
        try:
            return float(self._settings.value(key, default))
        except (TypeError, ValueError):
            return default

    # ---- text size ----

    def _apply_text_scale(self, scale, save=True):
        '''The app's font, and so every window's text, at `scale` x the
        startup size. Drawn widgets and fixed sizes follow (widgets.text_scale).'''
        base = self._base_font.pointSizeF()
        if base <= 0:
            return   # a pixel-sized system font: nothing to scale from
        scale = min(TEXT_SIZES, key=lambda s: abs(s - scale))
        font = QFont(self._base_font)
        font.setPointSizeF(base * scale)
        QApplication.instance().setFont(font)
        self._text_scale = scale
        if save:
            self._settings.setValue('view/text_scale', scale)

    def _set_text_scale(self, scale):
        self._apply_text_scale(scale)
        self._check_text_size()
        # Panels re-measure once their fonts have changed; then which layout fits may too.
        QTimer.singleShot(50, self.panels.fit)
        self._set_status(f"Text size {round(self._text_scale * 100)}%")

    def _step_text_size(self, step):
        index = TEXT_SIZES.index(self._text_scale) + step
        self._set_text_scale(TEXT_SIZES[max(0, min(len(TEXT_SIZES) - 1, index))])

    def _check_text_size(self):
        for action in self._size_actions.actions():
            action.setChecked(action.data() == self._text_scale)

    # ---- layout ----

    def _set_layout_mode(self, mode, save=True):
        self.panels.set_mode(mode)
        for action in self._layout_actions.actions():
            action.setChecked(action.data() == mode)
        if save:
            self._settings.setValue('view/layout', mode)

    def _reset_panel_sizes(self):
        self.panels.set_sizes({})
        self._settings.remove('view/panel_sizes')

    def _restore_window(self):
        geometry = self._settings.value('window/geometry')
        if geometry is None or not self.restoreGeometry(geometry):
            self.resize(1180, 860)
        mode = self._settings.value('view/layout', AUTOMATIC)
        self._set_layout_mode(mode if mode in (AUTOMATIC, COLUMNS, STACK) else AUTOMATIC, save=False)
        try:
            self.panels.set_sizes(json.loads(self._settings.value('view/panel_sizes', '{}')))
        except (TypeError, ValueError):
            pass

    def _save_window(self):
        self._settings.setValue('window/geometry', self.saveGeometry())
        self._settings.setValue('view/panel_sizes', json.dumps(self.panels.sizes()))

    # ---- tools and help ----

    def _open_firmware_dialog(self):
        """Tools > Flash Firmware: the Firmware Flasher's own window. Picking
        the controller this window is connected through lets go of it first."""
        from .firmware_dialog import FirmwareDialog
        busy = self._port if self.bot is not None and not self._simulated else None
        FirmwareDialog(busy_port=busy, release_port=self._disconnect_robot, parent=self).exec()

    def _open_motor_setup(self):
        """Tools > Motor Setup: the robot's log over its own USB cable, and its servo commands."""
        from .motor_setup_window import MotorSetupWindow
        if self._motor_window is None:
            busy = self._port if self.bot is not None and not self._simulated else None
            self._motor_window = MotorSetupWindow(busy_port=busy, parent=self)
            self._motor_window.destroyed.connect(lambda *_: setattr(self, '_motor_window', None))
        self._motor_window.show()
        self._motor_window.raise_()
        self._motor_window.activateWindow()

    def _open_camera_dialog(self):
        """Tools > Camera Wi-Fi: the camera's network, over the camera's own USB cable."""
        from .camera_dialog import CameraDialog
        CameraDialog(settings=self._settings, parent=self).exec()

    def _set_theme(self, which):
        """View > Theme: System, Light or Dark - the whole app at once, and kept for next time."""
        theme.apply(which)
        self._settings.setValue('view/theme', which)

    def _real_robot(self):
        """The connected robot, or None - never the simulated one, which has no camera."""
        return self.bot if self.bot is not None and not self._simulated else None

    def _camera_found(self, host):
        """CameraFinder's news: a camera at `host`, or none. With none, Robot
        View's Camera is greyed out - and if it was showing, it's 3D again."""
        self._camera_view_action.setEnabled(host is not None)
        self._camera_view_action.setText("&Camera" if host else "Camera (none found)")
        self._camera_view_action.setStatusTip(
            f"The camera's video, from {host}" if host else
            "No camera answering - set its Wi-Fi in Tools > Camera Wi-Fi, or connect the robot it's on")
        if host is None and self.viewer.showing_camera:
            Q8Logger.warning("Robot View: the camera stopped answering - back to the 3D model")
            self._model_view_action.setChecked(True)
            self.viewer.show_model()
        elif host is not None and self.viewer.showing_camera:
            self.viewer.show_camera(self.camera_stream, host)   # its address changed

    def _show_model_view(self):
        """View > Robot View > 3D Model."""
        self.viewer.show_model()

    def _show_camera_view(self):
        """View > Robot View > Camera."""
        host = self.camera_finder.host
        if host is None:                    # gone between the menu opening and the click
            self._model_view_action.setChecked(True)
            return
        self.viewer.show_camera(self.camera_stream, host)

    def _open_camera_window(self):
        """Tools > Camera Video: the camera's live picture in a window of its own."""
        from .camera_view import CameraWindow
        if self._camera_window is None:
            self._camera_window = CameraWindow(self._settings, self._real_robot, self.camera_stream, parent=self)
            self._camera_window.destroyed.connect(lambda *_: setattr(self, '_camera_window', None))
        self._camera_window.show()
        self._camera_window.raise_()
        self._camera_window.activateWindow()

    def _open_url(self, url):
        if not QDesktopServices.openUrl(QUrl(url)):
            Q8Logger.warning(f"Couldn't open a browser for {url}")

    def _show_about(self):
        from .. import firmware_flasher as flasher
        firmware = []
        for target in flasher.TARGETS:
            try:
                identity = flasher.read_bin_identity(flasher.bin_path(target))
                firmware.append(f"{target} {identity.version}" if identity and identity.version else None)
            except OSError:
                pass
        firmware = ', '.join(f for f in firmware if f) or 'not bundled'
        QMessageBox.about(
            self, "About Q8botOne",
            f"<h3>Q8botOne {__version__}</h3>"
            f"<p>The desktop app for the Q8botOne robot, by ZeroWire Robotics.</p>"
            f"<p>Firmware it installs: {firmware}<br>Wire protocol: {PROTOCOL_VERSION}</p>"
            f'<p><a href="{WEBSITE_URL}">zerowirerobotics.com</a> &nbsp; '
            f'<a href="{DOCS_URL}">Documentation</a> &nbsp; '
            f'<a href="{RELEASES_URL}">Releases</a></p>')

    #-------------------#
    # Robot             #
    #-------------------#

    def _connect_robot(self, port):
        simulated = port == SIMULATED_ROBOT
        try:
            # The simulated robot is the 3D view's model
            link = self.viewer.simulated_link() if simulated else q8_espnow(port)
            self.bot = Q8Bot(link)
        except Exception as exc:
            Q8Logger.error(f"Could not connect on {port}: {exc}")
            self.connection.set_robot_status(
                StatusDot.ERROR, "Port failed", f"Could not open {port}: {exc}")
            return
        # The IMU's mounting, as last calibrated - this robot's, as far as
        # anything can tell (see imu.MOUNT_FILE)
        self.bot.imu_mount = imu.Mount.load()
        self.viewer.set_robot(self.bot)

        self._port = port
        self._simulated = simulated
        self._last_reply_at = None
        self._servo_errors = None
        self._servo_restored = None
        self._connected_at = time.monotonic()
        self._last_request_at = 0.0  # so the first request goes out immediately
        self.connection.set_robot_connected(True)
        self.connection.set_robot_status(
            StatusDot.PENDING, "Waiting for robot...",
            f"{port} is open. Waiting for the robot to answer - is it powered on and paired?")
        self._set_status(f"{port} open - waiting for the robot")
        Q8Logger.info(f"{port} open. Gaits: {', '.join(self.bot.selectable)}")

        # Torque on and into the stance as soon as we connect - the
        # robot has to be holding itself up before any of this can drive it.
        self.bot.set_joint_mode(JointMode.ON)
        self.bot.stand(1000)
        self._shown_joint_mode = 'on'
        self.manual.set_joint_mode('on')
        Q8Logger.info("Torque on, moving to stance.")

        # Telemetry runs whenever this window holds a robot. It's
        # request/response, so closing the window simply stops asking - there
        # is no streaming state left running on the robot.
        self.bot.telemetry_hz = self._telemetry_hz()
        self.bot.sensor_hz = self._sensor_hz()
        self.info.set_sensor_status("Sensors: checking...")

        self._shown_gait = self.bot.gait
        self.manual.set_gaits(self.bot.catalog, self._shown_gait)
        self.manual.set_can_turn(self.bot.can_turn)
        self.manual.set_scripted_routines_enabled(True)
        self.manual.set_saved_routines_enabled(True)

    def _telemetry_hz(self):
        """Faster while the 3D view follows the robot's pose, so it moves smoothly there."""
        return FOLLOW_TELEMETRY_HZ if self.viewer.follows_robot else TELEMETRY_HZ

    def _calibrate_imu(self):
        """Calibrate IMU: the robot tilts itself a few ways while its IMU is
        read (Q8Bot.calibrate_imu()) - seconds, on the routine worker like a
        jump. Saved for next time, and handed to the 3D view."""
        if self.bot is None or not self.bot.connected or self._routine_running():
            return
        bot = self.bot

        def calibrate():
            try:
                mount = bot.calibrate_imu()
            except ValueError as exc:
                Q8Logger.error(f"IMU calibration didn't work: {exc}. The last calibration stays.")
                return
            self.viewer.set_mount(mount)
        self._run_routine("IMU calibration", calibrate)

    def _keep_level_toggled(self, on):
        if self.bot is None or not self.bot.connected:
            return
        if self.bot.set_keep_level(on):
            Q8Logger.info("Keeping the body level with the IMU." if on else "Keep Level off.")
        self.manual.show_keep_level(self.bot.keep_level)

    def _disconnect_robot(self):
        self._close_teach_dialog()   # teaching needs a live robot
        if self.bot is not None:
            self.bot.close()
            self.bot = None
        self.viewer.set_robot(None)
        self._simulated = False
        self._last_reply_at = None
        self.connection.set_robot_connected(False)
        self.connection.set_robot_status(StatusDot.OFF, "Not connected")
        self.info.set_battery(None)
        self.info.set_telemetry(None)
        self.info.set_sensor_status("")
        self._close_sensor_window()  # its readings come from this robot
        self.info.setEnabled(True)   # in case a routine had it greyed out
        self.manual.set_gaits([])
        self._shown_gait = None
        self._shown_joint_mode = None
        self.manual.set_joint_mode(None)
        self.manual.set_scripted_routines_enabled(False)
        self.manual.set_saved_routines_enabled(False)
        self.manual.set_imu(None)
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
            self.viewer.set_robot(None)
            self._simulated = False
            self._last_reply_at = None
            self.connection.set_robot_connected(False)
            self.connection.set_robot_status(
                StatusDot.ERROR, "Connection lost", "The link dropped - check the cable.")
            self.info.set_battery(None)
            self.info.set_telemetry(None)
            self.info.set_sensor_status("")
            self._close_sensor_window()  # its readings come from this robot
            self.info.setEnabled(True)   # in case a routine had it greyed out
            self.manual.set_gaits([])
            self._shown_gait = None
            self._shown_joint_mode = None
            self.manual.set_joint_mode(None)
            self.manual.set_scripted_routines_enabled(False)
            self.manual.set_saved_routines_enabled(False)
            self.manual.set_imu(None)
            self._close_teach_dialog()
            self._set_status("Connection lost")
            return

        self._poll_robot_liveness()
        self.info.set_telemetry(self.bot.telemetry, simulated=self._simulated)
        self._log_servo_errors(self.bot.telemetry)
        self._poll_sensors()
        self._poll_imu()
        self._poll_joint_mode()
        # Telemetry itself stalls during a routine anyway - Q8Bot's tick
        # thread can't get the lock a routine holds for its whole duration,
        # so the panel would just be showing stale numbers. Greying it out
        # says so instead of quietly going stale looking live.
        self.info.setEnabled(not self._routine_running())
        self.manual.set_scripted_routines_enabled(not self._routine_running())
        self.manual.set_saved_routines_enabled(self._routine_slots_available())
        self._poll_gamepad()
        self._poll_manual()

    def _log_servo_errors(self, sample):
        """Say in the console when a servo shuts itself down, and when it's
        clear again - and when the robot had to turn one's torque back on."""
        errors = None if self._simulated or sample is None else sample.get('hw_error')
        if errors is None:
            return
        before = self._servo_errors or (0,) * len(errors)
        for joint, (was, now) in enumerate(zip(before, errors), 1):
            if now and now != was:
                Q8Logger.error(f"Joint {joint}'s servo shut itself down: {describe_servo_error(now)}. "
                               "Its torque is off - click its Error, under Robot Information, to clear it.")
            elif was and not now:
                Q8Logger.info(f"Joint {joint}'s servo is clear again.")
        self._servo_errors = tuple(errors)

        # Counts since the robot started; the first snapshot is the baseline.
        restored = sample.get('restored')
        if restored is not None:
            if self._servo_restored is not None:
                for joint, (was, now) in enumerate(zip(self._servo_restored, restored), 1):
                    if now != was:
                        Q8Logger.warning(f"Joint {joint}'s servo lost its torque without an error - it "
                                         "probably restarted - and the robot turned it back on.")
            self._servo_restored = tuple(restored)

    def _clear_servo_error(self, slot):
        """An Error field's Clear: restart that servo (Q8Bot.reboot_servo) - short of
        switching the robot off and on, the only way to clear its error."""
        if self.bot is None or not self.bot.connected or self._routine_running():
            return
        errors = self._servo_errors
        kind = describe_servo_error(errors[slot - 1]) if errors and errors[slot - 1] else 'its error'
        Q8Logger.info(f"Restarting Joint {slot}'s servo to clear {kind}.")
        self.bot.reboot_servo(slot)
        self.info.mark_clearing(slot)

    def _routine_slots_available(self):
        """
        Playing a saved routine or opening the teach dialog both need sole
        control of the robot - blocked while any routine (Jump/Greet/a
        preview) is already running, and while a teach session is already
        open (one at a time is plenty).
        """
        return self._teach_dialog is None and not self._routine_running()

    #-------------------#
    # Manual control    #
    #-------------------#

    def _poll_manual(self):
        """
        Drive both pads from the poll loop rather than their moved signals -
        a drag emits far faster than the robot needs, and this keeps manual
        and gamepad control on the same cadence.

        Both go into one set_stance() call so offset and tilt compose instead
        of overwriting each other. It's idle-only and no-ops at dead centre,
        so nothing is sent while the pads are parked or a gait is running.
        """
        if not self.connection.pc_mode:
            return  # the pads only drive in PC mode
        if self._routine_running():
            # Jump/Greet/a routine preview hold Q8Bot's lock for seconds;
            # idle()/set_stance() below would block this thread waiting for
            # it - i.e. freeze the whole window - if called while one runs.
            return

        # Keep the dropdown honest if the gait changed elsewhere - the
        # gamepad's switch_gait button cycles it too.
        if self.bot.gait != self._shown_gait:
            self._shown_gait = self.bot.gait
            self.manual.show_gait(self._shown_gait)
            self.manual.set_can_turn(self.bot.can_turn)

        # Walking wins over posing: a stance
        # offset is meaningless once a gait owns the leg positions.
        mx, my = self.manual.move_pad.value()
        if math.hypot(mx, my) >= MOVE_PAD_DEADZONE:
            # Reuse the gamepad's own mapping so a given deflection means the
            # same thing on both. Its vertical axis is negative-is-forward.
            direction = get_joystick_direction(mx, -my, analog_mode=True)
            if direction:
                self.bot.move(direction, get_joystick_magnitude(mx, -my))
                return

        self.bot.idle()  # no-op once already idle
        x, y = self.manual.stance_pad.value()
        roll, pitch = self.manual.tilt_pad.value()
        self.bot.set_stance(
            dx=x * ManualControlPanel.STANCE_MAX_X,
            dy=y * ManualControlPanel.STANCE_MAX_Y,
            roll=roll * ManualControlPanel.TILT_MAX,
            pitch=pitch * ManualControlPanel.TILT_MAX,
        )

    def _joint_mode_requested(self, mode_str):
        """
        The slider was dragged. If the robot rejects it (DAMPING while a
        gait is running, say - see Q8Bot.set_joint_mode()), re-sync to
        whatever it's actually in rather than leaving the slider showing a
        position that never took effect.
        """
        if self.bot is None or not self.bot.connected:
            return
        if not self.bot.set_joint_mode(JointMode(mode_str)):
            Q8Logger.warning(f"Could not switch to {mode_str} (a gait may be running).")
        self._shown_joint_mode = self.bot.joint_mode.value
        self.manual.set_joint_mode(self._shown_joint_mode)
        Q8Logger.info(f"Joints: {self._shown_joint_mode}.")

    def _poll_joint_mode(self):
        """
        Keep the slider honest against everything that can change joint
        mode out from under it: the gamepad's RB (still plain on/off), and
        Q8Bot's own tick thread dropping DAMPING back to full torque the
        moment a gait starts or a scripted routine needs it (see
        Q8Bot._tick()/_exit_damping()).
        """
        mode = self.bot.joint_mode.value
        if mode != self._shown_joint_mode:
            self._shown_joint_mode = mode
            self.manual.set_joint_mode(mode)

    def _manual_jump(self):
        if self.bot is None or not self.bot.connected or self._routine_running():
            return
        self._try_jump()

    def _try_jump(self):
        """
        Jump if the battery can support it, else skip it outright rather
        than spinning up a worker thread just to sit through jump()'s
        multi-second settle wait for what the firmware would silently drop
        anyway (see Q8Bot.jump_allowed). Shared by the manual button and the
        gamepad's jump button.
        """
        if not self.bot.jump_allowed:
            Q8Logger.warning(
                f"Jump skipped - battery at {self.bot.battery_percent}%, "
                f"needs {JUMP_BATTERY_THRESHOLD}% or more.")
            return
        self._run_routine("Jump", self.bot.jump)

    def _manual_greet(self):
        if self.bot is None or not self.bot.connected or self._routine_running():
            return
        self._run_routine("Greet", self.bot.greet)

    def _select_gait(self, name):
        """Dropdown -> robot. Ignored if the robot won't load it."""
        if self.bot is None or not self.bot.connected:
            return
        if self.bot.set_gait(name):
            self._shown_gait = self.bot.gait
            self.manual.set_can_turn(self.bot.can_turn)
            Q8Logger.info(f"Switched to {self._shown_gait}.")
        else:
            Q8Logger.error(f"Could not load {name}.")
            self.manual.show_gait(self.bot.gait)

    #-------------------#
    # Taught routines   #
    #-------------------#

    def _reload_routines(self):
        """Re-scan disk and refresh the reserved rows - after save/delete too."""
        routines = routine_store.list_routines()
        self._saved_routines = routines
        overflow = len(routines) - ManualControlPanel.ROUTINE_SLOTS
        if overflow > 0:
            Q8Logger.warning(
                f"{overflow} saved routine(s) not shown - only the first "
                f"{ManualControlPanel.ROUTINE_SLOTS} fit the reserved rows.")
        self.manual.set_routines(routines)

    def _play_routine(self, path):
        if self.bot is None or not self.bot.connected or not self._routine_slots_available():
            return
        name = next((n for n, p in self._saved_routines if p == path), path.stem)
        try:
            waypoints = routine_store.load(path)
        except Exception as exc:
            Q8Logger.error(f"Could not load routine '{name}': {exc}")
            return
        self._run_routine(f"Routine: {name}", lambda: self.bot.play_routine(waypoints))

    def _teach_routine(self):
        if self.bot is None or not self.bot.connected or not self._routine_slots_available():
            return
        self._open_teach_dialog()

    def _edit_routine(self, path):
        if self.bot is None or not self.bot.connected or not self._routine_slots_available():
            return
        name = next((n for n, p in self._saved_routines if p == path), path.stem)
        try:
            waypoints = routine_store.load(path)
        except Exception as exc:
            Q8Logger.error(f"Could not load routine '{name}' to edit: {exc}")
            return
        self._open_teach_dialog(existing=(name, waypoints))

    def _open_teach_dialog(self, existing=None):
        """
        Shared setup for both teaching a new routine and editing a saved
        one - existing=(name, waypoints) pre-loads the dialog for editing;
        None starts empty. See TeachRoutineDialog's own `existing` note for
        what saving under the same vs. a different name does.
        """
        # Teaching means posing with the Stance/Body Tilt pads or the joint
        # mode slider, both of which live in Manual Control and only work
        # in PC mode - switch to it automatically rather than opening a
        # dialog whose controls do nothing yet.
        if not self.connection.pc_mode:
            self.connection.mode_toggle.setChecked(True)

        self.bot.idle()  # start from a known state, not mid-gait

        # Damping holds a joint in place without fighting a hand trying to
        # re-pose it - see Q8Bot.set_joint_mode(). It's also the only
        # capture method that works with one mouse: sculpting a pose with
        # the Stance/Body Tilt pads means holding the mouse button down on
        # a pad, which leaves nothing free to click Capture with. The
        # slider still works normally for anyone who'd rather do that.
        self.bot.set_joint_mode(JointMode.DAMPING)
        self._shown_joint_mode = 'damping'
        self.manual.set_joint_mode('damping')

        self.manual.set_scripted_routines_enabled(False)
        self.manual.set_saved_routines_enabled(False)

        self._teach_dialog = TeachRoutineDialog(
            capture_pose=self.bot.capture_pose, run_preview=self._preview_routine,
            existing=existing, parent=self)
        self._teach_dialog.saved.connect(self._on_routine_saved)
        self._teach_dialog.closed.connect(self._on_teach_closed)
        self._teach_dialog.position_beside(self)
        self._teach_dialog.show()
        if existing:
            Q8Logger.info(f"Editing '{existing[0]}' - joints are damped for hand-posing.")
        else:
            Q8Logger.info("Teaching a new routine - joints are damped for "
                         "hand-posing, then Capture.")

    def _preview_routine(self, waypoints):
        """
        Passed into TeachRoutineDialog as its run_preview callback. Reuses
        the same worker-thread routine mechanism as Jump/Greet, and returns
        the thread so the dialog can poll it the way MainWindow polls its
        own routines.

        Teaching leaves the joints in OFF or DAMPING (see _teach_routine) -
        neither has the stiffness to actually play a routine back, so this
        switches to full ON just long enough to run it, then restores
        whichever mode was active before. Already at ON (nothing to
        restore, e.g. torque was never touched)? Left alone either way.
        """
        previous_mode = self.bot.joint_mode

        def play():
            if previous_mode is not JointMode.ON:
                self.bot.set_joint_mode(JointMode.ON)
            self.bot.play_routine(waypoints)
            if previous_mode is not JointMode.ON:
                self.bot.set_joint_mode(previous_mode)

        return self._run_routine("Preview", play)

    def _on_routine_saved(self, name):
        Q8Logger.info(f"Saved routine '{name}'.")
        self._reload_routines()

    def _on_teach_closed(self):
        self._teach_dialog = None
        available = self.bot is not None and self.bot.connected
        if available:
            self.bot.set_joint_mode(JointMode.ON)   # stand back up rather than stay soft/limp
        self.manual.set_scripted_routines_enabled(available and not self._routine_running())
        self.manual.set_saved_routines_enabled(available and self._routine_slots_available())

    def _close_teach_dialog(self):
        if self._teach_dialog is not None:
            self._teach_dialog.close()   # fires closed -> _on_teach_closed

    def _delete_routine(self, path):
        name = next((n for n, p in self._saved_routines if p == path), path.stem)
        if QMessageBox.question(
                self, "Delete Routine", f"Delete '{name}'? This can't be undone."
        ) != QMessageBox.Yes:
            return
        routine_store.delete(path)
        Q8Logger.info(f"Deleted routine '{name}'.")
        self._reload_routines()

    #-------------------#
    # Gamepad control   #
    #-------------------#

    def _poll_gamepad(self):
        """
        Turn gamepad state into Q8Bot commands. Buttons act on the press edge
        instead of time.sleep() debounces, which would freeze the window.
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
            self._try_jump()
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
        Jump, greet, shoot, and now taught-routine playback all block for
        seconds and hold Q8Bot's lock, so they run on a worker rather than
        freezing the window. Input is ignored until the routine finishes
        (see _poll_gamepad/_poll_manual and _routine_slots_available).

        Returns the Thread, so a caller that needs to know when it's done
        (the teach dialog, polling its own preview) can watch is_alive()
        the same way this class polls Q8Bot's state elsewhere.
        """
        Q8Logger.info(f"{label}...")
        self.bot.idle()
        self._routine = threading.Thread(target=call, name=f'q8bot-{label.lower()}',
                                         daemon=True)
        self._routine.start()
        return self._routine

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

    #-------------------#
    # Qwiic sensors     #
    #-------------------#

    def _poll_sensors(self):
        snapshot = self.bot.sensors
        age = self.bot.sensor_age

        if snapshot is None or age >= SENSOR_SILENT_SECONDS:
            waited = time.monotonic() - self._connected_at
            if snapshot is None and waited < SENSOR_SILENT_SECONDS:
                self.info.set_sensor_status("Sensors: checking...")
            else:
                self.info.set_sensor_status(
                    "Sensors: no reply",
                    "The robot isn't answering sensor requests - it may be off "
                    "or out of range, or its firmware may predate sensor support.")
        elif not snapshot.sensors:
            self.info.set_sensor_status(
                "No sensor connected",
                "Nothing supported on the robot's Qwiic port. Sensors are found "
                "when the robot boots - plug one in, then power-cycle it.")
        else:
            names = ", ".join(sensor_kind(s.kind, len(s.values)).name for s in snapshot.sensors)
            label = "Sensor" if len(snapshot.sensors) == 1 else "Sensors"
            self.info.set_sensor_status(f"{label}: {names}", "Open the live readings.",
                                        available=True)

        if self._sensor_window is not None:
            self._sensor_window.show_snapshot(snapshot, age)
            # Strips redraw once per new reading, not on every 20 ms poll.
            if snapshot is not None and snapshot is not self._plotted_snapshot:
                self._plotted_snapshot = snapshot
                self._sensor_window.show_history(self.bot.sensor_history)

    def _poll_imu(self):
        """Keep Level and Calibrate IMU, for as long as an MPU6050 answers -
        and Keep Level off if it stops."""
        age = self.bot.sensor_age
        present = (imu.reading(self.bot.sensors) is not None
                   and age is not None and age < SENSOR_SILENT_SECONDS)
        if not present:
            if self.bot.keep_level and not self.bot.calibrating_imu:
                self.bot.set_keep_level(False)
                Q8Logger.warning("The IMU stopped answering - Keep Level is off.")
            self.manual.set_imu(None)
            return
        self.manual.set_imu('calibrated' if self.bot.imu_mount is not None else 'uncalibrated')
        self.manual.show_keep_level(self.bot.keep_level)

    def _open_sensor_window(self):
        if self._sensor_window is not None:
            self._sensor_window.raise_()
            self._sensor_window.activateWindow()
            return
        self._sensor_window = SensorWindow(parent=self)
        self._plotted_snapshot = None   # draw the history straight away
        self._sensor_window.closed.connect(self._on_sensor_window_closed)
        self._sensor_window.position_beside(self)
        self._sensor_window.show()
        if self.bot is not None:
            self.bot.sensor_hz = self._sensor_hz()

    def _on_sensor_window_closed(self):
        self._sensor_window = None
        if self.bot is not None:
            self.bot.sensor_hz = self._sensor_hz()

    def _sensor_hz(self):
        """Fastest while the 3D view follows the robot (for its IMU), then while
        the Sensors window is open; otherwise just enough to see what's attached."""
        if self.viewer.follows_robot:
            return FOLLOW_IMU_HZ
        return SENSOR_LIVE_HZ if self._sensor_window is not None else SENSOR_PRESENCE_HZ

    def _close_sensor_window(self):
        if self._sensor_window is not None:
            self._sensor_window.close()   # fires closed -> _on_sensor_window_closed

    def _set_status(self, text):
        self.statusBar().showMessage(text)

    def closeEvent(self, event):
        self._save_window()
        self._timer.stop()
        self._close_teach_dialog()
        if self._motor_window is not None:
            self._motor_window.close()
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

    if sys.platform == 'win32':
        # A taskbar identity of its own, so Windows shows the app's icon
        # there rather than python.exe's when it's run from Python.
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID('ZeroWireRobotics.Q8botOne')

    app = QApplication(sys.argv[:1])
    app.setWindowIcon(QIcon(str(APP_ICON)))
    window = MainWindow(handler)
    window.show()
    return app.exec()
