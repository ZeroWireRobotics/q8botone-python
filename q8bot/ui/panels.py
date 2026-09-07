'''
Written by yufeng.wu0902@gmail.com

The individual panels of the control window.

ConnectionPanel and DebugConsole are fully live. ManualControlPanel's
sliders move but aren't wired to the robot yet, and RobotInfoPanel only has
real data for Battery % - each says so in its own docstring. The 3D viewer
lives in viewer.py.
'''

import html

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
)

from ..helpers import XiaoPortFinder
from .widgets import ClickableLabel, StatusDot, ToggleSwitch

# Manual Control and Robot Information sit side by side and their Joint 1-8
# rows are meant to read across the gap, so both grids are pinned to the same
# row pitch - a slider and a read-only field are naturally different heights.
PANEL_ROW_HEIGHT = 34

STATE_TEXT_COLORS = {
    StatusDot.PENDING: '#a5620a',
    StatusDot.LIVE: '#1a7f37',
    StatusDot.ERROR: '#b3262b',
}

LEVEL_COLORS = {
    'WARNING': '#b58900',
    'ERROR': '#dc322f',
    'CRITICAL': '#dc322f',
}


class RefreshingCombo(QComboBox):
    """A combo box that re-runs its source the moment it's opened."""

    def __init__(self, source, parent=None):
        super().__init__(parent)
        self._source = source

    def showPopup(self):
        self.repopulate()
        super().showPopup()

    def repopulate(self):
        previous = self.currentData()
        self.clear()
        for label, value in self._source():
            self.addItem(label, value)
        if previous is not None:
            index = self.findData(previous)
            if index >= 0:
                self.setCurrentIndex(index)


class ConnectionPanel(QGroupBox):
    """
    Robot and gamepad selection. Emits requests rather than owning the
    connection itself - MainWindow holds the Q8Bot and calls back in with
    set_robot_connected() / set_gamepad_connected().
    """

    robot_connect_requested = Signal(str)   # port
    robot_disconnect_requested = Signal()
    gamepad_connect_requested = Signal(object)  # joystick index
    gamepad_disconnect_requested = Signal()
    mode_changed = Signal(bool)             # True = PC mode

    def __init__(self, list_gamepads, parent=None):
        super().__init__("Connection", parent)
        self._list_gamepads = list_gamepads
        self._robot_connected = False
        self._gamepad_connected = False

        self.port_combo = RefreshingCombo(self._robot_ports)
        self.port_combo.setMinimumWidth(200)
        self.robot_button = QPushButton("Connect")
        self.robot_button.clicked.connect(self._toggle_robot)

        self.gamepad_combo = RefreshingCombo(self._gamepads)
        self.gamepad_combo.setMinimumWidth(200)
        self.gamepad_button = QPushButton("Connect")
        self.gamepad_button.clicked.connect(self._toggle_gamepad)

        self.mode_toggle = ToggleSwitch()
        self.mode_toggle.setChecked(True)  # PC mode until a gamepad drives things
        self.mode_toggle.toggled.connect(self._mode_changed)
        self.xbox_label = ClickableLabel(
            "Xbox Mode", lambda: self.mode_toggle.setChecked(False))
        self.pc_label = ClickableLabel(
            "PC Mode", lambda: self.mode_toggle.setChecked(True))

        self.robot_dot = StatusDot()
        self.robot_state = QLabel()
        self.gamepad_dot = StatusDot()
        self.gamepad_state = QLabel()
        for label in (self.robot_state, self.gamepad_state):
            label.setMinimumWidth(140)

        grid = QGridLayout(self)
        grid.addWidget(self.robot_dot, 0, 0)
        grid.addWidget(QLabel("Robot Controller"), 0, 1)
        grid.addWidget(self.port_combo, 0, 2)
        grid.addWidget(self.robot_button, 0, 3)
        grid.addWidget(self.robot_state, 0, 4)
        grid.addWidget(self.gamepad_dot, 1, 0)
        grid.addWidget(QLabel("Xbox Controller"), 1, 1)
        grid.addWidget(self.gamepad_combo, 1, 2)
        grid.addWidget(self.gamepad_button, 1, 3)
        grid.addWidget(self.gamepad_state, 1, 4)
        grid.setColumnStretch(4, 1)

        mode_row = QHBoxLayout()
        mode_row.addStretch()
        mode_row.addWidget(self.xbox_label)
        mode_row.addWidget(self.mode_toggle)
        mode_row.addWidget(self.pc_label)
        mode_row.addStretch()
        grid.addLayout(mode_row, 2, 0, 1, 5)

        self.set_robot_status(StatusDot.OFF, "Not connected")
        self.set_gamepad_status(StatusDot.OFF, "Not connected")

        self.port_combo.currentIndexChanged.connect(self._update_buttons)
        self.gamepad_combo.currentIndexChanged.connect(self._update_buttons)

        self.port_combo.repopulate()
        self.gamepad_combo.repopulate()
        self._update_buttons()
        self._paint_mode()

    def _robot_ports(self):
        ports = XiaoPortFinder.list_all()
        if not ports:
            return [("No controller found", None)]
        return [(p, p) for p in ports]

    def _gamepads(self):
        # Compatible devices only. SDL happily reports Bluetooth mice and
        # other oddities as joysticks, and offering one just leads to a
        # Connect that can't work.
        usable = [(d['name'], d['index']) for d in self._list_gamepads()
                  if d['compatible']]
        return usable or [("No gamepad found", None)]

    def _update_buttons(self):
        """Nothing valid selected means nothing to connect to."""
        if not self._robot_connected:
            self.robot_button.setEnabled(self.port_combo.currentData() is not None)
        if not self._gamepad_connected:
            self.gamepad_button.setEnabled(self.gamepad_combo.currentData() is not None)

    def _toggle_robot(self):
        if self._robot_connected:
            self.robot_disconnect_requested.emit()
        else:
            port = self.port_combo.currentData()
            if port:
                self.robot_connect_requested.emit(port)

    def _toggle_gamepad(self):
        if self._gamepad_connected:
            self.gamepad_disconnect_requested.emit()
        else:
            self.gamepad_connect_requested.emit(self.gamepad_combo.currentData())

    def set_robot_status(self, state, text, tooltip=None):
        self._paint_status(self.robot_dot, self.robot_state, state, text, tooltip)

    def set_gamepad_status(self, state, text, tooltip=None):
        self._paint_status(self.gamepad_dot, self.gamepad_state, state, text, tooltip)

    @staticmethod
    def _paint_status(dot, label, state, text, tooltip):
        dot.set_state(state, tooltip or text)
        color = STATE_TEXT_COLORS.get(state)
        label.setText(text)
        label.setStyleSheet(f"color: {color};" if color else "")
        label.setToolTip(tooltip or "")

    def set_robot_connected(self, connected):
        self._robot_connected = connected
        self.robot_button.setText("Disconnect" if connected else "Connect")
        self.robot_button.setEnabled(True)
        self.port_combo.setEnabled(not connected)
        self._update_buttons()

    def set_gamepad_connected(self, connected):
        self._gamepad_connected = connected
        self.gamepad_button.setText("Disconnect" if connected else "Connect")
        self.gamepad_button.setEnabled(True)
        self.gamepad_combo.setEnabled(not connected)
        self._update_buttons()

    @property
    def pc_mode(self):
        return self.mode_toggle.isChecked()

    def _mode_changed(self):
        self._paint_mode()
        self.mode_changed.emit(self.pc_mode)

    def _paint_mode(self):
        active = QFont()
        active.setBold(True)
        idle = QFont()
        self.pc_label.setFont(active if self.pc_mode else idle)
        self.xbox_label.setFont(idle if self.pc_mode else active)


class DebugConsole(QGroupBox):
    """Mirrors Q8Logger output, the way operate.py's pygame overlay does."""

    def __init__(self, parent=None):
        super().__init__("Debug Console", parent)
        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(2000)
        self.view.document().setDefaultFont(QFont("Consolas", 9))
        layout = QVBoxLayout(self)
        layout.addWidget(self.view)

    def append(self, timestamp, level, message):
        color = LEVEL_COLORS.get(level)
        text = f"[{timestamp}] {level}: {message}"
        if color:
            self.view.appendHtml(f'<span style="color:{color}">{html.escape(text)}</span>')
        else:
            self.view.appendPlainText(text)


class ManualControlPanel(QGroupBox):
    """
    Body pose and per-joint sliders. Live and draggable in PC mode, greyed
    out in Xbox mode - but nothing is wired to the robot yet, so moving one
    changes nothing (see the panel tooltip).

    Ranges are the ones these will eventually mean, so wiring them up later
    doesn't move every knob: degrees for roll/pitch and the joints, mm for
    the X/Y stance offsets (matching operate.py's stance trim limits).
    """

    JOINT_COUNT = 8
    POSE_RANGES = {           # name: (minimum, maximum, initial)
        'Roll': (-30, 30, 0),
        'Pitch': (-30, 30, 0),
        'X': (-25, 25, 0),
        'Y': (-15, 15, 0),
    }
    JOINT_RANGE = (-90, 270, 90)

    def __init__(self, parent=None):
        super().__init__("Manual Control", parent)
        self.setToolTip("Not wired to the robot yet - the sliders move but "
                        "nothing is sent.")

        grid = QGridLayout(self)
        self.pose_sliders = {}
        self.joint_sliders = []

        for row, (left, right) in enumerate((("Roll", "X"), ("Pitch", "Y"))):
            for col, name in ((0, left), (2, right)):
                grid.addWidget(QLabel(name), row, col)
                grid.addWidget(self._make_slider(name), row, col + 1)

        for index in range(self.JOINT_COUNT):
            row = 2 + index
            grid.addWidget(QLabel(f"Joint {index + 1}"), row, 0)
            slider = QSlider(Qt.Horizontal)
            low, high, start = self.JOINT_RANGE
            slider.setRange(low, high)
            slider.setValue(start)
            self.joint_sliders.append(slider)
            grid.addWidget(slider, row, 1, 1, 3)

        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)
        for row in range(2 + self.JOINT_COUNT):
            grid.setRowMinimumHeight(row, PANEL_ROW_HEIGHT)
        grid.setRowStretch(2 + self.JOINT_COUNT, 1)  # keep rows packed to the top

    def _make_slider(self, name):
        low, high, start = self.POSE_RANGES[name]
        slider = QSlider(Qt.Horizontal)
        slider.setRange(low, high)
        slider.setValue(start)
        self.pose_sliders[name] = slider
        return slider

    def set_available(self, available):
        """Draggable in PC mode, greyed out in Xbox mode, per the mockup."""
        self.setEnabled(available)
        self.setTitle("Manual Control" if available
                      else "Manual Control (disabled in Xbox Mode)")


class RobotInfoPanel(QGroupBox):
    """
    Battery and per-joint telemetry.

    Only Battery % is live - it comes back in BATTERY_REPLY. Voltage isn't in
    that reply at all, and while the firmware already reads joint position and
    current (q8Dynamixel::syncRead) it only feeds the recording path, with no
    command to query it live. Temperature isn't read at all.
    """

    JOINT_COUNT = 8
    COLUMNS = ("Position", "Current", "Temperature")

    def __init__(self, parent=None):
        super().__init__("Robot Information", parent)
        grid = QGridLayout(self)

        # The battery row lives in its own layout so its wide fields don't
        # stretch the per-joint columns underneath it.
        self.battery_voltage = QLineEdit()
        self.battery_percent = QLineEdit()
        battery_row = QHBoxLayout()
        battery_row.addWidget(QLabel("Battery Voltage"))
        battery_row.addWidget(self.battery_voltage, 1)
        battery_row.addSpacing(12)
        battery_row.addWidget(QLabel("Battery %"))
        battery_row.addWidget(self.battery_percent, 1)
        grid.addLayout(battery_row, 0, 0, 1, len(self.COLUMNS) + 1)

        for column, name in enumerate(self.COLUMNS):
            header = QLabel(name)
            header.setAlignment(Qt.AlignCenter)
            grid.addWidget(header, 1, column + 1)

        self.joint_fields = {}
        for index in range(self.JOINT_COUNT):
            row = 2 + index
            grid.addWidget(QLabel(f"Joint {index + 1}"), row, 0)
            for column, name in enumerate(self.COLUMNS):
                field = QLineEdit()
                field.setAlignment(Qt.AlignCenter)
                self.joint_fields[(name, index)] = field
                grid.addWidget(field, row, column + 1)

        for column in range(len(self.COLUMNS)):
            grid.setColumnStretch(column + 1, 1)

        for field in (self.battery_voltage, self.battery_percent,
                      *self.joint_fields.values()):
            field.setReadOnly(True)
            field.setFocusPolicy(Qt.NoFocus)

        # The firmware only reports a percentage - there's no voltage in
        # BATTERY_REPLY, so that box stays empty for now.
        self.battery_voltage.setPlaceholderText("not reported by firmware")
        for row in range(2 + self.JOINT_COUNT):
            grid.setRowMinimumHeight(row, PANEL_ROW_HEIGHT)
        grid.setRowStretch(2 + self.JOINT_COUNT, 1)

    def set_battery(self, percent):
        self.battery_percent.setText("" if percent is None else f"{percent}%")
