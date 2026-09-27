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
from ..protocol import ticks_to_degrees
from .widgets import (
    BatteryGauge,
    ClickableLabel,
    JoystickPad,
    StatusDot,
    ToggleSwitch,
)

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
    flash_firmware_requested = Signal()

    def __init__(self, list_gamepads, parent=None):
        super().__init__("Connection", parent)
        self._list_gamepads = list_gamepads
        self._robot_connected = False
        self._gamepad_connected = False

        self.port_combo = RefreshingCombo(self._robot_ports)
        self.port_combo.setMinimumWidth(200)
        self.robot_button = QPushButton("Connect")
        self.robot_button.clicked.connect(self._toggle_robot)

        # Always enabled - flashing works with or without a robot connected,
        # and is how a device on broken firmware gets fixed.
        self.flash_button = QPushButton("Flash Firmware...")
        self.flash_button.setToolTip("Update the robot or controller firmware over USB.")
        self.flash_button.clicked.connect(self.flash_firmware_requested.emit)

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
        grid.addWidget(self.flash_button, 0, 5)
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
        grid.addLayout(mode_row, 2, 0, 1, 6)

        self.set_robot_status(StatusDot.OFF, "Not connected")
        self.set_gamepad_status(StatusDot.OFF, "Not connected")

        self.port_combo.currentIndexChanged.connect(self._update_buttons)
        self.gamepad_combo.currentIndexChanged.connect(self._update_buttons)

        self.port_combo.repopulate()
        self.gamepad_combo.repopulate()
        self._update_buttons()
        self._paint_mode()
        self._update_mode_lock()

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
        self._update_mode_lock()

    def _update_mode_lock(self):
        """
        Xbox mode is meaningless with nothing to drive it - lock the toggle
        to PC mode until a gamepad is actually connected. PC mode itself
        stays reachable either way (there's nothing to lock it against).
        """
        self.mode_toggle.setEnabled(self._gamepad_connected)
        self.xbox_label.setEnabled(self._gamepad_connected)
        tooltip = "" if self._gamepad_connected else "Connect a gamepad first."
        self.mode_toggle.setToolTip(tooltip)
        self.xbox_label.setToolTip(tooltip)

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
    Three thumbstick-style pads - walk, stance offset, body tilt - plus the
    gait selector, since which gait is loaded changes what the move pad can
    do. Draggable in PC mode, greyed out in Xbox mode.

    All three drive the robot. Tilt works by extending the legs on one side
    and retracting the other; see Q8Bot.set_stance() and the leg-spacing
    constants it scales by.

    There are no per-joint sliders. Each leg is a five-bar linkage whose two
    motors are mechanically coupled, so driving one joint while the other sits
    still bypasses the IK that keeps every commanded pose reachable, and can
    push the linkage into a hard stop with the servo still applying torque.
    Everything here goes through the solver instead.
    """

    gait_selected = Signal(str)
    joint_mode_requested = Signal(str)   # 'off' | 'damping' | 'on'
    jump_requested = Signal()
    greet_requested = Signal()
    routine_play_requested = Signal(object)    # path
    routine_teach_requested = Signal()          # an empty slot was clicked
    routine_edit_requested = Signal(object)    # path
    routine_delete_requested = Signal(object)  # path

    # Full deflection, in the units each pad commands.
    STANCE_MAX_X = 25.0   # mm, matching operate.py's right-stick trim limits
    STANCE_MAX_Y = 15.0   # mm
    TILT_MAX = 20.0       # degrees of roll/pitch

    # Slider position 0/1/2 -> mode. Off/on keep the old Torque button's
    # colours; damping reuses HOLD_COLOR - both are "this joint is soft",
    # just for different reasons (hand-posed vs. teaching-in-progress).
    JOINT_MODES = ('off', 'damping', 'on')
    JOINT_MODE_LABELS = {'off': 'Off', 'damping': 'Damping', 'on': 'On'}
    JOINT_MODE_COLORS = {'on': '#1a7f37', 'damping': '#8250df', 'off': '#b3262b'}

    # How many saved-routine rows to reserve. A fixed number rather than a
    # scrolling list, so the layout doesn't resize itself under the user as
    # routines are taught - see the readout-width comment above for why that
    # matters here. Raise this if it's ever too cramped.
    ROUTINE_SLOTS = 4

    def __init__(self, parent=None):
        super().__init__("Manual Control", parent)

        self.move_pad = JoystickPad()
        self.move_pad.setToolTip("Drag to walk; release to stop. Push further "
                                 "to go faster.")
        self.stance_pad = JoystickPad()
        self.stance_pad.setToolTip("Drag to shift the standing pose; release "
                                   "to return to the baseline. Idle only.")
        self.tilt_pad = JoystickPad()
        self.tilt_pad.setToolTip("Drag to tilt the body; release to level "
                                 "off. Idle only.")

        self.stance_readout = QLabel()
        self.tilt_readout = QLabel()
        # Fixed width, sized for the widest string each can ever show (both
        # values at max magnitude). Otherwise the readout's natural width
        # tracks its text - crossing from one digit to two (9.9 -> 10.0)
        # widens the label, which widens the grid column, which re-centres
        # the pad above it: the pad visibly jumps sideways while dragging.
        for label, worst_case in (
            (self.stance_readout,
             f"X {-self.STANCE_MAX_X:+.1f} mm    Y {-self.STANCE_MAX_Y:+.1f} mm"),
            (self.tilt_readout,
             f"Roll {-self.TILT_MAX:+.1f}°    Pitch {-self.TILT_MAX:+.1f}°"),
        ):
            label.setFixedWidth(label.fontMetrics().horizontalAdvance(worst_case) + 4)

        self.gait_combo = QComboBox()
        self.gait_combo.setEnabled(False)   # nothing to choose until connected
        self.gait_combo.activated.connect(self._gait_chosen)

        # Off / Damping / On, one slider rather than a button - damping is a
        # real third state (see Q8Bot.set_joint_mode()), not a toggle. Labels
        # never change their text (only which one is bold/coloured), so
        # unlike a button whose text changes, nothing here can resize the
        # column around it - see the Stance/Tilt readout fix for what that
        # bug looks like elsewhere in this panel.
        self.joint_mode_slider = QSlider(Qt.Horizontal)
        self.joint_mode_slider.setRange(0, len(self.JOINT_MODES) - 1)
        self.joint_mode_slider.setSingleStep(1)
        self.joint_mode_slider.setPageStep(1)
        self.joint_mode_slider.setTickPosition(QSlider.TicksBelow)
        self.joint_mode_slider.setTickInterval(1)
        self.joint_mode_slider.setEnabled(False)   # nothing to set until connected
        self.joint_mode_slider.setToolTip(
            "Off: fully backdrivable, no holding force.\n"
            "Damping: torqued but soft - holds a hand-posed joint without "
            "fighting it.\n"
            "On: normal full-stiffness control.")
        self.joint_mode_slider.valueChanged.connect(self._joint_mode_slider_changed)

        self.joint_mode_row_labels = {}
        joint_mode_labels_row = QHBoxLayout()
        for mode in self.JOINT_MODES:
            label = QLabel(self.JOINT_MODE_LABELS[mode])
            label.setAlignment(Qt.AlignCenter)
            self.joint_mode_row_labels[mode] = label
            joint_mode_labels_row.addWidget(label)

        self.hold_pose_button = QPushButton("Hold Pose: OFF")
        self.hold_pose_button.setCheckable(True)
        self.hold_pose_button.setToolTip(
            "Keep a pad wherever you drag it instead of springing back to "
            "centre on release - for holding a pose, or teaching a routine "
            "without needing to keep the mouse button down.")
        self.hold_pose_button.toggled.connect(self._hold_pose_toggled)

        # Hold Pose changes its own text - letting its natural width track
        # whichever text it currently shows would resize the column it's in
        # and shove the pads beside it sideways (the same bug the Stance/
        # Tilt readouts had; same fix: measure the widest state up front).
        self._lock_width_to_widest(self.hold_pose_button, [
            (False, "Hold Pose: OFF"), (True, "Hold Pose: ON")])

        # One-shot scripted routines - the same jump/greet the gamepad's
        # left/top buttons trigger, just without a pad to hold one.
        self.jump_button = QPushButton("Jump")
        self.jump_button.setToolTip("Run the jump routine. Takes a few seconds "
                                    "and holds the robot until it finishes.")
        self.jump_button.clicked.connect(self.jump_requested.emit)

        self.greet_button = QPushButton("Greet")
        self.greet_button.setToolTip("Run the greet routine. Takes a few seconds "
                                     "and holds the robot until it finishes.")
        self.greet_button.clicked.connect(self.greet_requested.emit)

        for pad, readout in ((self.stance_pad, self.stance_readout),
                             (self.tilt_pad, self.tilt_readout)):
            readout.setAlignment(Qt.AlignCenter)
            pad.moved.connect(self._update_readouts)

        grid = QGridLayout(self)
        for column, (title, pad, readout, hint) in enumerate((
            ("Move", self.move_pad, None, "Direction / speed"),
            ("Stance", self.stance_pad, self.stance_readout, "X / Y offset"),
            ("Body Tilt", self.tilt_pad, self.tilt_readout, "Roll / Pitch"),
        )):
            heading = QLabel(title)
            heading.setAlignment(Qt.AlignCenter)
            font = heading.font()
            font.setBold(True)
            heading.setFont(font)

            subtitle = QLabel(hint)
            subtitle.setAlignment(Qt.AlignCenter)

            grid.addWidget(heading, 0, column)
            grid.addWidget(subtitle, 1, column)
            grid.addWidget(pad, 2, column, Qt.AlignCenter)
            if readout is not None:
                grid.addWidget(readout, 3, column)

        # Row 3, flush with the Stance/Tilt readouts beside it - under the
        # move pad, because the gait is what decides where that pad can go
        # (a gait that can't turn locks its left/right axis).
        gait_row = QHBoxLayout()
        gait_row.addWidget(QLabel("Gait"))
        gait_row.addWidget(self.gait_combo, 1)
        grid.addLayout(gait_row, 3, 0)

        # Column 3 is the stretch space to the right of the three pads (see
        # setColumnStretch below) - the only real "empty space" in this
        # layout, and not tied to any one pad's readout. Stacked rather than
        # spread across the row so they read as one group of one-off actions,
        # separate from the continuous pads to their left.
        actions_col = QVBoxLayout()
        actions_col.addStretch()
        actions_col.addWidget(self.joint_mode_slider)
        actions_col.addLayout(joint_mode_labels_row)
        actions_col.addSpacing(8)
        actions_col.addWidget(self.hold_pose_button)
        actions_col.addSpacing(12)
        actions_col.addWidget(self.jump_button)
        actions_col.addWidget(self.greet_button)
        actions_col.addStretch()
        grid.addLayout(actions_col, 0, 3, 4, 1)

        # Reserved routine rows, below everything else and spanning the full
        # width. Populated by set_routines() - an empty slot opens the teach
        # dialog, a filled one plays that routine on click and has its own
        # Delete button (an explicit button rather than a right-click menu,
        # which nothing on screen hints exists - deleting a routine has to be
        # discoverable, especially since it's the only way to free a slot
        # once all ROUTINE_SLOTS are taken). Built once; set_routines() only
        # ever relabels these, so the layout itself never resizes as
        # routines are taught/deleted.
        routines_heading = QLabel("Routines")
        routines_heading.setAlignment(Qt.AlignCenter)
        font = routines_heading.font()
        font.setBold(True)
        routines_heading.setFont(font)
        grid.addWidget(routines_heading, 5, 0, 1, 4)

        self.routine_buttons = []
        self.routine_edit_buttons = []
        self.routine_delete_buttons = []
        for i in range(self.ROUTINE_SLOTS):
            button = QPushButton()
            button.setEnabled(False)
            button.clicked.connect(
                lambda _checked=False, index=i: self._routine_slot_clicked(index))

            edit_button = QPushButton("✎")
            edit_button.setFixedWidth(28)
            edit_button.setToolTip("Edit this routine")
            edit_button.setEnabled(False)
            edit_button.clicked.connect(
                lambda _checked=False, index=i: self._routine_edit_clicked(index))

            delete_button = QPushButton("✕")
            delete_button.setFixedWidth(28)
            delete_button.setToolTip("Delete this routine")
            delete_button.setEnabled(False)
            delete_button.clicked.connect(
                lambda _checked=False, index=i: self._routine_delete_clicked(index))

            row = QHBoxLayout()
            row.addWidget(button, 1)
            row.addWidget(edit_button)
            row.addWidget(delete_button)
            grid.addLayout(row, 6 + i, 0, 1, 4)
            self.routine_buttons.append(button)
            self.routine_edit_buttons.append(edit_button)
            self.routine_delete_buttons.append(delete_button)
        self._routines = [None] * self.ROUTINE_SLOTS  # None = empty/teach slot

        grid.setHorizontalSpacing(28)
        grid.setColumnStretch(3, 1)   # pads sit left; space to the right
        grid.setRowStretch(6 + self.ROUTINE_SLOTS, 1)   # above the bottom edge
        self._update_readouts()
        self.set_joint_mode(None)
        self.set_scripted_routines_enabled(False)
        self.set_routines([])
        self.set_saved_routines_enabled(False)

    @staticmethod
    def _lock_width_to_widest(button, states):
        """
        Fix `button`'s minimum width to whichever (checked, text) pair in
        `states` - the states it will actually be shown in - needs the most
        room, so switching between them later can't resize the column it
        sits in. Signals are blocked while probing - toggling a checkable
        button's checked state here would otherwise fire its real handler
        (see hold_pose_button's toggled -> _hold_pose_toggled) as a side
        effect of just measuring it.
        """
        blocked = button.blockSignals(True)
        original_checked, original_text = button.isChecked(), button.text()
        widest = 0
        for checked, text in states:
            button.setChecked(checked)
            button.setText(text)
            widest = max(widest, button.sizeHint().width())
        button.setChecked(original_checked)
        button.setText(original_text)
        button.blockSignals(blocked)
        button.setMinimumWidth(widest)

    #-------------------#
    # Gait selection    #
    #-------------------#

    def set_gaits(self, catalog, current=None):
        """
        Fill the dropdown from Q8Bot.catalog. Safe to call repeatedly - it
        rebuilds without emitting, so following the robot's own gait changes
        (the gamepad cycles them too) can't feed back into a command.
        """
        self.gait_combo.blockSignals(True)
        self.gait_combo.clear()
        for info in catalog:
            self.gait_combo.addItem(info.label, info.name)
            self.gait_combo.setItemData(self.gait_combo.count() - 1,
                                        info.description, Qt.ToolTipRole)
        self.gait_combo.setEnabled(bool(catalog))
        self.show_gait(current)
        self.gait_combo.blockSignals(False)

    def show_gait(self, name):
        """Point the dropdown at `name` without emitting a selection."""
        blocked = self.gait_combo.signalsBlocked()
        self.gait_combo.blockSignals(True)
        index = self.gait_combo.findData(name)
        if index >= 0:
            self.gait_combo.setCurrentIndex(index)
            self.gait_combo.setToolTip(
                self.gait_combo.itemData(index, Qt.ToolTipRole) or "")
        self.gait_combo.blockSignals(blocked)

    def _gait_chosen(self, index):
        name = self.gait_combo.itemData(index)
        if name:
            self.gait_selected.emit(name)

    def set_joint_mode(self, mode):
        """
        Reflect the robot's actual joint mode - it can change out from
        under this slider (the gamepad's RB toggles plain on/off, and a
        gait or scripted routine drops damping automatically), so
        MainWindow calls this on every poll rather than only after a drag
        here. `mode=None` means unknown (no robot connected).
        """
        blocked = self.joint_mode_slider.blockSignals(True)
        if mode is not None:
            self.joint_mode_slider.setValue(self.JOINT_MODES.index(mode))
        self.joint_mode_slider.setEnabled(mode is not None)
        self.joint_mode_slider.blockSignals(blocked)

        for name, label in self.joint_mode_row_labels.items():
            current = (name == mode)
            font = label.font()
            font.setBold(current)
            label.setFont(font)
            color = self.JOINT_MODE_COLORS.get(name) if current else None
            label.setStyleSheet(f"color: {color};" if color else "")

    def _joint_mode_slider_changed(self, value):
        self.joint_mode_requested.emit(self.JOINT_MODES[value])

    def _hold_pose_toggled(self, checked):
        """
        Purely local UI state - doesn't touch Q8Bot, so it needs no
        connection and MainWindow doesn't have to know about it. Applies to
        all three pads uniformly: released-but-held applies just as well to
        "keep walking without holding the mouse down" as it does to posing.
        """
        for pad in (self.move_pad, self.stance_pad, self.tilt_pad):
            pad.set_hold(checked)
        self.hold_pose_button.setText(f"Hold Pose: {'ON' if checked else 'OFF'}")
        # Solid fill, not just text colour: a checked QPushButton already
        # gets its own background from the OS style (blue on Windows), which
        # fights a text-only colour instead of blending with it. Setting the
        # background explicitly replaces that with the same colour a pinned
        # pad turns (JoystickPad.HOLD_COLOR), so the two read as one state.
        self.hold_pose_button.setStyleSheet(
            f"QPushButton {{ background-color: {JoystickPad.HOLD_COLOR.name()}; "
            "color: white; }" if checked else "")

    def set_scripted_routines_enabled(self, enabled):
        """
        Jump/Greet take the robot over for a few seconds - disabled while
        disconnected or while one is already running, same as the gamepad's
        equivalent buttons are simply ignored during that window.
        """
        self.jump_button.setEnabled(enabled)
        self.greet_button.setEnabled(enabled)

    #-------------------#
    # Saved routines    #
    #-------------------#

    def set_routines(self, routines):
        """
        Fill the reserved rows from routine_store.list_routines() -
        [(name, path), ...]. A row beyond what's saved shows as an empty
        "+ Teach New Routine" slot. Safe to call repeatedly (MainWindow
        calls this again after every save/delete); rebuilds the labels only,
        never the row layout itself.

        Extra saved routines beyond ROUTINE_SLOTS are silently not shown -
        MainWindow logs that rather than this panel, since deciding whether
        that's worth telling the user about isn't this widget's job.
        """
        self._routines = list(routines[:self.ROUTINE_SLOTS])
        self._routines += [None] * (self.ROUTINE_SLOTS - len(self._routines))
        for button, edit_button, delete_button, entry in zip(
                self.routine_buttons, self.routine_edit_buttons,
                self.routine_delete_buttons, self._routines):
            if entry is None:
                button.setText("+ Teach New Routine")
                button.setToolTip(
                    "Pose the robot by hand or with the pads, then save it "
                    "here as a new routine.")
                edit_button.setVisible(False)
                delete_button.setVisible(False)
            else:
                name, _path = entry
                button.setText(name)
                button.setToolTip(f"Play '{name}'.")
                edit_button.setVisible(True)
                delete_button.setVisible(True)

    def set_saved_routines_enabled(self, enabled):
        """
        Playing, editing, deleting or teaching a routine all need a
        connected robot - the empty "+ Teach" slots are gated the same as
        the filled ones rather than always being clickable, since a click
        on any of them eventually needs to talk to Q8Bot.
        """
        for button in self.routine_buttons:
            button.setEnabled(enabled)
        for i, (edit_button, delete_button) in enumerate(
                zip(self.routine_edit_buttons, self.routine_delete_buttons)):
            filled = self._routines[i] is not None
            edit_button.setEnabled(enabled and filled)
            delete_button.setEnabled(enabled and filled)

    def _routine_slot_clicked(self, index):
        entry = self._routines[index]
        if entry is None:
            self.routine_teach_requested.emit()
        else:
            _name, path = entry
            self.routine_play_requested.emit(path)

    def _routine_edit_clicked(self, index):
        entry = self._routines[index]
        if entry is not None:
            _name, path = entry
            self.routine_edit_requested.emit(path)

    def _routine_delete_clicked(self, index):
        entry = self._routines[index]
        if entry is not None:
            _name, path = entry
            self.routine_delete_requested.emit(path)

    def set_can_turn(self, can_turn):
        """Lock the move pad to forward/back for a gait that cannot steer."""
        self.move_pad.set_horizontal_enabled(can_turn)
        self.move_pad.setToolTip(
            "Drag to walk; release to stop. Push further to go faster."
            if can_turn else
            "This gait only walks forward and back, so the pad is locked to "
            "the vertical axis.")

    #-------------------#
    # Readouts          #
    #-------------------#

    def _update_readouts(self, *_args):
        x, y = self.stance_pad.value()
        self.stance_readout.setText(
            f"X {x * self.STANCE_MAX_X:+.1f} mm    Y {y * self.STANCE_MAX_Y:+.1f} mm")

        roll, pitch = self.tilt_pad.value()
        self.tilt_readout.setText(
            f"Roll {roll * self.TILT_MAX:+.1f}°    Pitch {pitch * self.TILT_MAX:+.1f}°")

    def set_available(self, available):
        """Draggable in PC mode, greyed out in Xbox mode, per the mockup."""
        self.setEnabled(available)
        self.setTitle("Manual Control" if available
                      else "Manual Control (disabled in Xbox Mode)")


class RobotInfoPanel(QGroupBox):
    """
    Battery and per-joint telemetry.

    Everything here is live: battery percent and raw pack voltage both come
    from one BATTERY_REPLY (and one ADC sample, so they can't disagree), and
    the three joint columns come from TELEMETRY_REPLY.

    Position and current are read fresh on every snapshot; temperature is
    whatever the robot last sampled on its own ~1s timer, since it sits
    outside the register block the main read covers and moves over seconds.
    """

    JOINT_COUNT = 8
    COLUMNS = ("Position", "Current", "Temperature")

    def __init__(self, parent=None):
        super().__init__("Robot Information", parent)
        grid = QGridLayout(self)

        # One gauge plus its two readouts, rather than a labelled box each -
        # the level is the thing you glance at, and the numbers are the
        # detail. Its own layout so it can't stretch the joint columns below.
        self.battery_gauge = BatteryGauge()
        self.battery_percent = QLabel()
        self.battery_voltage = QLabel()
        self.battery_percent.setMinimumWidth(42)
        self.battery_voltage.setMinimumWidth(58)

        battery_row = QHBoxLayout()
        battery_row.addWidget(QLabel("Battery"))
        battery_row.addWidget(self.battery_gauge)
        battery_row.addWidget(self.battery_percent)
        battery_row.addWidget(self.battery_voltage)
        battery_row.addStretch()
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

        # Joint readouts only - the battery row is plain labels now.
        for field in self.joint_fields.values():
            field.setReadOnly(True)
            field.setFocusPolicy(Qt.NoFocus)

        for row in range(2 + self.JOINT_COUNT):
            grid.setRowMinimumHeight(row, PANEL_ROW_HEIGHT)
        grid.setRowStretch(2 + self.JOINT_COUNT, 1)

    def set_battery(self, percent, volts=None):
        self.battery_gauge.set_percent(percent)
        self.battery_percent.setText("" if percent is None else f"{percent}%")
        # volts stays None against firmware old enough to send only a
        # percentage, so that readout empties rather than showing a stale one.
        self.battery_voltage.setText("" if volts is None else f"{volts:.2f} V")

    def set_telemetry(self, sample):
        """
        Fill the joint grid from one snapshot (see protocol.parse_telemetry),
        or clear it when passed None.
        """
        if sample is None:
            for field in self.joint_fields.values():
                field.setText("")
            return

        for i in range(self.JOINT_COUNT):
            degrees = ticks_to_degrees(sample['position'][i])
            self.joint_fields[("Position", i)].setText(f"{degrees:.1f}°")
            self.joint_fields[("Current", i)].setText(f"{sample['current'][i]} mA")
            self.joint_fields[("Temperature", i)].setText(f"{sample['temperature'][i]}°C")
