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
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from .. import routine_store
from .widgets import place_beside

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

    Every routine is bracketed by a Start and an End row - both mean "move
    to the robot's normal standing pose" (see routine_store.stance_waypoint())
    rather than a captured one, always present and never deletable, so
    there's never a question of whether the user needs to manually pose and
    capture "the robot standing normally" themselves before or after the
    real waypoints. Capture always inserts a new waypoint just before End;
    Undo always removes the one just before End, never Start or End
    themselves.

    Between any two rows (Start, a real waypoint, or End) sits a Move Time
    field for the transition into whichever comes next; each real waypoint
    also has its own editable name (cosmetic only) and a Dwell field (how
    long to hold that pose before moving on).
    """

    saved = Signal(str)   # routine name, once Save As... succeeds
    closed = Signal()

    def __init__(self, capture_pose, run_preview, existing=None, parent=None):
        """
        capture_pose: () -> list[float] | None - Q8Bot.capture_pose().
        run_preview: (waypoints) -> threading.Thread - starts a background
            playback and returns the thread, so this dialog can poll
            is_alive() to know when to re-enable its own buttons (the same
            polling style MainWindow itself uses for Q8Bot's state).
        existing: (name, waypoints) to edit an already-saved routine, or
            None to start teaching a new one. A routine saved before Start/
            End existed is upgraded with them on load. Saving offers `name`
            back as the default so re-saving under it overwrites rather
            than forking a second copy - typing a different name still
            creates a new routine alongside it rather than renaming/
            deleting the original, since that's the safer default.
        """
        super().__init__(parent)
        self._existing_name = existing[0] if existing else None
        self.setWindowTitle(f"Edit Routine: {self._existing_name}" if existing
                            else "Teach a Routine")
        self.setWindowModality(Qt.NonModal)
        # Floats above the main window without capturing its input - unlike
        # a modal dialog, which would make the pads behind it undraggable.
        self.setWindowFlag(Qt.WindowStaysOnTopHint, True)

        self._capture_pose = capture_pose
        self._run_preview = run_preview
        self._preview_thread = None
        self.waypoints = []
        self._waypoint_rows = []   # one per waypoint, same order as self.waypoints
        self._move_rows = []       # one per waypoint after the first (index i -> into waypoints[i+1])

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
        self.resize(420, 440)   # position_beside() sets the real height once shown

        # Seed Start/End - either fresh, or loaded from an existing routine
        # (upgrading it with whichever of the two it's missing, so an older
        # routine gains them the moment it's next edited).
        if existing:
            _name, loaded = existing
            loaded = [dict(wp) for wp in loaded]   # copy - don't mutate the caller's list
            if not loaded or not loaded[0].get('stance'):
                loaded.insert(0, routine_store.stance_waypoint('start'))
            if not loaded[-1].get('stance'):
                loaded.append(routine_store.stance_waypoint('end'))
            for waypoint in loaded:
                self._append_waypoint(waypoint)
        else:
            self._append_waypoint(routine_store.stance_waypoint('start'))
            self._append_waypoint(routine_store.stance_waypoint('end'))

        self._preview_timer = QTimer(self)
        self._preview_timer.setInterval(100)
        self._preview_timer.timeout.connect(self._check_preview)

        self._update_buttons()

    def position_beside(self, widget):
        place_beside(self, widget)

    #-------------------#
    # Waypoint capture  #
    #-------------------#

    def _capture(self):
        angles = self._capture_pose()
        if angles is None:
            QMessageBox.warning(self, "No pose yet",
                "Haven't heard from the robot yet - wait a moment and try again.")
            return
        self._insert_before_end(routine_store.new_waypoint(angles))
        self._update_buttons()

    def _append_waypoint(self, waypoint):
        """
        Adds `waypoint` at the very end of the list, building its row (plus
        a preceding move-time row, unless it's the first thing in the
        list). Only used while building the dialog top-to-bottom in order
        (fresh Start/End, or loading an existing routine) - see
        _insert_before_end() for adding a new real waypoint later.
        """
        index = len(self.waypoints)
        self.waypoints.append(waypoint)

        if index > 0:
            move_row, move_spin = self._build_move_row(waypoint)
            self._rows_layout.insertWidget(self._rows_layout.count() - 1, move_row)
            self._move_rows.append((move_row, move_spin))

        waypoint_row, name_edit, dwell_spin = self._build_waypoint_row(waypoint, index)
        self._rows_layout.insertWidget(self._rows_layout.count() - 1, waypoint_row)
        self._waypoint_rows.append((waypoint_row, name_edit, dwell_spin))

    def _insert_before_end(self, waypoint):
        """
        Inserts a new real waypoint just before End (always list position
        len - 1) - Capture's only entry point into the list. The move-time
        row already sitting between the previous last item and End doesn't
        need touching: its spinbox is bound to End's own dict by reference
        (see _build_move_row), so it keeps controlling End's duration no
        matter how many waypoints end up inserted before it.
        """
        index = len(self.waypoints) - 1   # End currently sits here
        self.waypoints.insert(index, waypoint)

        # Row layout is [row, move, row, move, ..., row, stretch] - waypoint
        # k's own row sits at child index 2k, preceded by its move-time row
        # at 2k-1 (k >= 1 always holds here, since index 0 is always Start).
        move_row, move_spin = self._build_move_row(waypoint)
        self._rows_layout.insertWidget(2 * index - 1, move_row)
        self._move_rows.insert(index - 1, (move_row, move_spin))

        waypoint_row, name_edit, dwell_spin = self._build_waypoint_row(waypoint, index)
        self._rows_layout.insertWidget(2 * index, waypoint_row)
        self._waypoint_rows.insert(index, (waypoint_row, name_edit, dwell_spin))

    def _undo(self):
        """Removes the last REAL waypoint (never Start or End)."""
        if len(self.waypoints) <= 2:
            return
        index = len(self.waypoints) - 2   # last real waypoint; -1 is always End
        self.waypoints.pop(index)

        row, _name_edit, _spin = self._waypoint_rows.pop(index)
        row.setParent(None)
        row.deleteLater()

        # The move-time row that led into the removed waypoint goes with it;
        # the row leading into whatever follows (End) is untouched - it's
        # bound to End's dict, not a position, so it's already correct.
        move_row, _move_spin = self._move_rows.pop(index - 1)
        move_row.setParent(None)
        move_row.deleteLater()

        self._update_buttons()

    def _build_waypoint_row(self, waypoint, display_index):
        """
        One row per waypoint. Start/End render as a fixed, italic label -
        there's nothing to name or rename, and no ambiguity about whether
        the user needs to fill them in themselves. A real waypoint gets an
        editable name field; `display_index` is its position in the full
        list (Start occupies 0, so this already equals the 1-based waypoint
        number without adjustment).
        """
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(4, 2, 4, 2)

        role = waypoint.get('stance')
        name_edit = None
        if role:
            label = QLabel(f"{role.capitalize()} (Default Stance)")
            font = label.font()
            font.setItalic(True)
            label.setFont(font)
            label.setToolTip("Always the robot's normal standing pose - nothing to capture here.")
            row_layout.addWidget(label, 1)
        else:
            name_edit = QLineEdit(waypoint.get('name', ''))
            name_edit.setPlaceholderText(f"Waypoint {display_index}")
            name_edit.setToolTip("Optional label - purely for your own reference, not used when playing the routine back.")
            name_edit.textChanged.connect(lambda text, wp=waypoint: self._set_name(wp, text))
            row_layout.addWidget(name_edit, 1)

        row_layout.addWidget(QLabel("Dwell (s)"))
        spin = QDoubleSpinBox()
        spin.setRange(*DWELL_RANGE_S)
        spin.setSingleStep(TIME_STEP_S)
        spin.setDecimals(1)
        spin.setValue(waypoint['hold_s'])
        spin.setToolTip("How long to hold this pose before moving to the next one.")
        spin.valueChanged.connect(lambda value, wp=waypoint: self._set_hold(wp, value))
        row_layout.addWidget(spin)
        return row, name_edit, spin

    def _build_move_row(self, waypoint):
        """
        The row leading into `waypoint` (Start has none - see
        _append_waypoint/_insert_before_end). Bound to `waypoint`'s own
        dict by reference rather than a list position, so it stays correct
        no matter how many waypoints get inserted or removed around it.
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
        spin.setValue(waypoint['duration_ms'] / 1000.0)
        spin.setToolTip("How long the move into the next row takes.")
        spin.valueChanged.connect(lambda value, wp=waypoint: self._set_duration(wp, value))
        row_layout.addWidget(spin)
        return row, spin

    def _set_name(self, waypoint, text):
        waypoint['name'] = text

    def _set_hold(self, waypoint, value_s):
        waypoint['hold_s'] = value_s

    def _set_duration(self, waypoint, value_s):
        waypoint['duration_ms'] = round(value_s * 1000)

    #-------------------#
    # Preview           #
    #-------------------#

    def _start_preview(self):
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
        # Pre-filled with the routine's existing name when editing, so
        # hitting OK re-saves over it - typing something else instead
        # forks a new routine (see __init__'s `existing` note).
        name, ok = QInputDialog.getText(
            self, "Save Routine", "Routine name:", text=self._existing_name or "")
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
        has_real_waypoints = len(self.waypoints) > 2   # more than just Start/End
        self.capture_button.setEnabled(not busy)
        self.undo_button.setEnabled(has_real_waypoints and not busy)
        # Preview/Save stay enabled even with zero real waypoints - Start
        # straight to End is a trivial but valid "return to stance" routine.
        self.preview_button.setEnabled(not busy)
        self.save_button.setEnabled(not busy)
        # Editing timing mid-preview would change waypoints out from under
        # the routine already playing them back.
        for _row, name_edit, spin in self._waypoint_rows:
            if name_edit is not None:
                name_edit.setEnabled(not busy)
            spin.setEnabled(not busy)
        for _row, spin in self._move_rows:
            spin.setEnabled(not busy)

    def closeEvent(self, event):
        self._preview_timer.stop()
        self.closed.emit()
        super().closeEvent(event)
