'''
Written by yufeng.wu0902@gmail.com

The individual panels of the control window.

ConnectionPanel and DebugConsole are live. ManualControlPanel,
RobotInfoPanel and ViewerPanel are deliberate placeholders - laid out with
the controls they will eventually have, all disabled, so the shape of the
window is real while the wiring catches up.
'''

import html

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..helpers import XiaoPortFinder
from .widgets import ClickableLabel, ToggleSwitch

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

        grid = QGridLayout(self)
        grid.addWidget(QLabel("Robot Controller"), 0, 0)
        grid.addWidget(self.port_combo, 0, 1)
        grid.addWidget(self.robot_button, 0, 2)
        grid.addWidget(QLabel("Xbox Controller"), 1, 0)
        grid.addWidget(self.gamepad_combo, 1, 1)
        grid.addWidget(self.gamepad_button, 1, 2)

        mode_row = QHBoxLayout()
        mode_row.addStretch()
        mode_row.addWidget(self.xbox_label)
        mode_row.addWidget(self.mode_toggle)
        mode_row.addWidget(self.pc_label)
        mode_row.addStretch()
        grid.addLayout(mode_row, 2, 0, 1, 3)

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
    """PLACEHOLDER. Roll/pitch need a body-frame model that doesn't exist yet."""

    def __init__(self, parent=None):
        super().__init__("Manual Control", parent)
        grid = QGridLayout(self)

        self._sliders = {}
        for row, (left, right) in enumerate((("Roll", "X"), ("Pitch", "Y"))):
            for col, name in ((0, left), (2, right)):
                grid.addWidget(QLabel(name), row, col)
                slider = QSlider(Qt.Horizontal)
                slider.setRange(-100, 100)
                grid.addWidget(slider, row, col + 1)
                self._sliders[name] = slider

        grid.addWidget(QLabel("Joint Select"), 2, 0)
        self.joint_combo = QComboBox()
        self.joint_combo.addItems([f"Joint {i + 1}" for i in range(8)])
        grid.addWidget(self.joint_combo, 2, 1)
        self.joint_slider = QSlider(Qt.Horizontal)
        self.joint_slider.setRange(-100, 100)
        grid.addWidget(self.joint_slider, 2, 2, 1, 2)
        grid.setRowStretch(3, 1)  # keep the rows packed to the top

        # Stays disabled whatever the mode - none of these do anything yet,
        # and controls that look live but aren't are worse than obvious ones.
        self.setEnabled(False)
        self.set_available(True)

    def set_available(self, available):
        """Greyed out in Xbox mode, per the mockup."""
        self.setTitle("Manual Control (placeholder)" if available
                      else "Manual Control (disabled in Xbox Mode)")


class RobotInfoPanel(QGroupBox):
    """PLACEHOLDER. Battery % is live-able today; the rest needs firmware."""

    JOINT_ROWS = ("Joint Position", "Joint Current", "Joint Temperature")

    def __init__(self, parent=None):
        super().__init__("Robot Information", parent)
        form = QFormLayout(self)

        self.battery_voltage = QLineEdit()
        self.battery_percent = QLineEdit()
        battery_row = QHBoxLayout()
        battery_row.addWidget(QLabel("Battery Voltage"))
        battery_row.addWidget(self.battery_voltage)
        battery_row.addWidget(QLabel("Battery %"))
        battery_row.addWidget(self.battery_percent)
        form.addRow(battery_row)

        self.joint_fields = {}
        for name in self.JOINT_ROWS:
            field = QLineEdit()
            field.setPlaceholderText("8 values")
            self.joint_fields[name] = field
            form.addRow(name, field)

        for field in (self.battery_voltage, self.battery_percent, *self.joint_fields.values()):
            field.setReadOnly(True)


class ViewerPanel(QGroupBox):
    """PLACEHOLDER. Needs forward kinematics for all four legs plus a body model."""

    def __init__(self, parent=None):
        super().__init__("", parent)
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        label = QLabel("Interactive 3D Viewer (Placeholder)")
        label.setAlignment(Qt.AlignCenter)
        inner = QVBoxLayout(frame)
        inner.addWidget(label)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(frame)
