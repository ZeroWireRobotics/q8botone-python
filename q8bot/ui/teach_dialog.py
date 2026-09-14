'''
Written by yufeng.wu0902@gmail.com

The routine-teaching dialog: pose the robot by hand (joints damped - see
Q8Bot.set_joint_mode()) or with the Stance / Body Tilt pads (full torque),
Capture each pose, then Save.

Non-modal on purpose - teaching a routine *is* driving the robot through the
Manual Control panel, so this window only holds the capture/preview/save
controls around that; it must not block the pads or joint mode slider
behind it the way a normal (modal) dialog would.
'''

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .. import routine_store

# Spinbox ranges/steps for the per-waypoint timing fields below.
DWELL_RANGE_S = (0.0, 30.0)
MOVE_RANGE_S = (0.0, 10.0)
TIME_STEP_S = 0.1


class TeachRoutineDialog(QDialog):
    """
    Owns only the in-progress waypoint list and the capture/preview/save
    controls - it never talks to Q8Bot directly. MainWindow supplies
    capture_pose() and a routine-preview launcher as plain callables, the
    same arm's-length pattern the rest of the UI uses to keep the threading
    and locking discipline in one place (see MainWindow._run_routine).

    Each waypoint gets a row with its own Dwell field (routine_store's
    hold_s - how long to sit at that pose before moving on), and a Move
    Time row between it and the next waypoint (duration_ms - how long that
    next move takes). There's no Move Time row before the first waypoint;
    that move is from whatever pose preceded teaching, not from another
    waypoint in this list.
    """

    saved = Signal(str)   # routine name, once Save As... succeeds
    closed = Signal()

    def __init__(self, capture_pose, run_preview, parent=None):
        """
        capture_pose: () -> list[float] | None - Q8Bot.capture_pose().
        run_preview: (waypoints) -> threading.Thread - starts a background
            playback and returns the thread, so this dialog can poll
            is_alive() to know when to re-enable its own buttons (the same
            polling style MainWindow itself uses for Q8Bot's state).
        """
        super().__init__(parent)
        self.setWindowTitle("Teach a Routine")
        self.setWindowModality(Qt.NonModal)
        # Floats above the main window without capturing its input - unlike
        # a modal dialog, which would make the pads behind it undraggable.
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)

        self._capture_pose = capture_pose
        self._run_preview = run_preview
        self._preview_thread = None
        self.waypoints = []
        self._waypoint_rows = []   # one per waypoint
        self._move_rows = []       # one per waypoint after the first

        instructions = QLabel("Pose the robot, then Capture. Repeat, then Save.")
        instructions.setWordWrap(True)

        # A plain QVBoxLayout inside a scroll area rather than a QListWidget -
        # each row here holds its own spinbox, and QListWidget doesn't mix
        # well with per-item interactive widgets the way a normal layout does.
        self._rows_container = QWidget()
        self._rows_layout = QVBoxLayout(self._rows_container)
        self._rows_layout.setContentsMargins(0, 0, 0, 0)
        self._rows_layout.addStretch()   # rows insert above this
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._rows_container)

        self.capture_button = QPushButton("Capture Pose")
        self.capture_button.setToolTip("Record the robot's current joint angles as the next waypoint.")
        self.capture_button.clicked.connect(self._capture)

        self.undo_button = QPushButton("Undo Last")
        self.undo_button.clicked.connect(self._undo)

        self.preview_button = QPushButton("Preview")
        self.preview_button.setToolTip("Play the waypoints captured so far, without saving.")
        self.preview_button.clicked.connect(self._start_preview)

        self.save_button = QPushButton("Save As...")
        self.save_button.clicked.connect(self._save)

        self.cancel_button = QPushButton("Cancel")
        self.cancel_button.clicked.connect(self.close)

        actions = QHBoxLayout()
        for button in (self.capture_button, self.undo_button, self.preview_button):
            actions.addWidget(button)

        bottom = QHBoxLayout()
        bottom.addWidget(self.cancel_button)
        bottom.addStretch()
        bottom.addWidget(self.save_button)

        layout = QVBoxLayout(self)
        layout.addWidget(instructions)
        layout.addWidget(scroll, 1)
        layout.addLayout(actions)
        layout.addLayout(bottom)
        self.resize(380, 440)

        self._preview_timer = QTimer(self)
        self._preview_timer.setInterval(100)
        self._preview_timer.timeout.connect(self._check_preview)

        self._update_buttons()

    def position_beside(self, widget):
        """
        Place this dialog just outside `widget`'s frame - to its right if
        there's room, else to its left, else pinned against whichever screen
        edge fits - rather than wherever Qt's default placement happens to
        land, which was opening it right on top of the Manual Control panel
        it's meant to be used alongside (forcing a manual drag every time).
        """
        anchor = widget.frameGeometry()
        x, y = anchor.right() + 12, anchor.top()
        screen = widget.screen()
        if screen is not None:
            available = screen.availableGeometry()
            if x + self.width() > available.right():
                x = anchor.left() - self.width() - 12
            x = max(available.left(), min(x, available.right() - self.width()))
            y = max(available.top(), min(y, available.bottom() - self.height()))
        self.move(x, y)

    #-------------------#
    # Waypoint capture  #
    #-------------------#

    def _capture(self):
        angles = self._capture_pose()
        if angles is None:
            QMessageBox.warning(self, "No pose yet",
                "Haven't heard from the robot yet - wait a moment and try again.")
            return
        index = len(self.waypoints)
        self.waypoints.append(routine_store.new_waypoint(angles))

        if index > 0:
            move_row, move_spin = self._build_move_row(index)
            self._rows_layout.insertWidget(self._rows_layout.count() - 1, move_row)
            self._move_rows.append((move_row, move_spin))

        waypoint_row, dwell_spin = self._build_waypoint_row(index)
        self._rows_layout.insertWidget(self._rows_layout.count() - 1, waypoint_row)
        self._waypoint_rows.append((waypoint_row, dwell_spin))

        self._update_buttons()

    def _undo(self):
        if not self.waypoints:
            return
        self.waypoints.pop()
        row, _spin = self._waypoint_rows.pop()
        row.setParent(None)
        row.deleteLater()
        if self._move_rows:
            move_row, _move_spin = self._move_rows.pop()
            move_row.setParent(None)
            move_row.deleteLater()
        self._update_buttons()

    def _build_waypoint_row(self, index):
        """One row per waypoint: its label, and a Dwell field on the right."""
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(4, 2, 4, 2)
        row_layout.addWidget(QLabel(f"Waypoint {index + 1}"))
        row_layout.addStretch()
        row_layout.addWidget(QLabel("Dwell (s)"))

        spin = QDoubleSpinBox()
        spin.setRange(*DWELL_RANGE_S)
        spin.setSingleStep(TIME_STEP_S)
        spin.setDecimals(1)
        spin.setValue(self.waypoints[index]['hold_s'])
        spin.setToolTip("How long to hold this pose before moving to the next one.")
        spin.valueChanged.connect(lambda value, i=index: self._set_hold(i, value))
        row_layout.addWidget(spin)
        return row, spin

    def _build_move_row(self, index):
        """
        The row between waypoint (index-1) and waypoint (index): how long
        the move into waypoint `index` takes.
        """
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(4, 2, 4, 2)
        row_layout.addStretch()
        row_layout.addWidget(QLabel("↓ Move Time (s)"))

        spin = QDoubleSpinBox()
        spin.setRange(*MOVE_RANGE_S)
        spin.setSingleStep(TIME_STEP_S)
        spin.setDecimals(1)
        spin.setValue(self.waypoints[index]['duration_ms'] / 1000.0)
        spin.setToolTip("How long the move into the next waypoint takes.")
        spin.valueChanged.connect(lambda value, i=index: self._set_duration(i, value))
        row_layout.addWidget(spin)
        return row, spin

    def _set_hold(self, index, value_s):
        self.waypoints[index]['hold_s'] = value_s

    def _set_duration(self, index, value_s):
        self.waypoints[index]['duration_ms'] = round(value_s * 1000)

    #-------------------#
    # Preview           #
    #-------------------#

    def _start_preview(self):
        if not self.waypoints:
            return
        self._preview_thread = self._run_preview(list(self.waypoints))
        self._update_buttons()
        self._preview_timer.start()

    def _check_preview(self):
        if self._preview_thread is not None and self._preview_thread.is_alive():
            return
        self._preview_timer.stop()
        self._preview_thread = None
        self._update_buttons()

    def _previewing(self):
        return self._preview_thread is not None

    #-------------------#
    # Save              #
    #-------------------#

    def _save(self):
        if not self.waypoints:
            return
        name, ok = QInputDialog.getText(self, "Save Routine", "Routine name:")
        name = name.strip()
        if not ok or not name:
            return
        routine_store.save(name, self.waypoints)
        self.saved.emit(name)
        self.close()

    #-------------------#
    # Buttons / lifecycle
    #-------------------#

    def _update_buttons(self):
        busy = self._previewing()
        has_waypoints = bool(self.waypoints)
        self.capture_button.setEnabled(not busy)
        self.undo_button.setEnabled(has_waypoints and not busy)
        self.preview_button.setEnabled(has_waypoints and not busy)
        self.save_button.setEnabled(has_waypoints and not busy)
        # Editing timing mid-preview would change waypoints out from under
        # the routine already playing them back.
        for _row, spin in self._waypoint_rows:
            spin.setEnabled(not busy)
        for _row, spin in self._move_rows:
            spin.setEnabled(not busy)

    def closeEvent(self, event):
        self._preview_timer.stop()
        self.closed.emit()
        super().closeEvent(event)
