'''
Pose and drive the simulated Q8botOne, to check the model:

    venv\\Scripts\\python -m q8botone_sim.poser

Two windows: MuJoCo's 3D view, and this panel. The panel's open tab drives
the robot -

  Joints   each servo's target angle, degrees below forward
  Feet     each foot's target in its leg's plane: x forward and y down from
           the leg's rear servo shaft, mm - the library's leg frame
  Body     the chassis moving over the feet, which stay planted: forward,
           up, roll and pitch from the pose the tab was opened on
  Walk     the desktop app's Move, Stance and Body Tilt pads, and its gaits
  Moves    the jump, the greets and the other routines, taught ones too

Walk and Moves go through the q8bot library's Q8Bot, unchanged, to the
robot's firmware redone in the sim (firmware.py) - the same path they take
to the robot - and the pads are the desktop app's own (q8bot.ui, the
library's 'gui' extra). Above the tabs: on the floor or on a test stand,
servos on or limp, the contact shapes shown, pause, reset. Below them, what
the robot is actually doing.
'''

import collections
import math
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
                               QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QRadioButton, QSizePolicy,
                               QSlider, QTabWidget, QVBoxLayout, QWidget)
from q8bot import Q8Bot, routine_store

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')     # the desktop app's gamepad code imports pygame
from q8bot.control_config import get_joystick_direction, get_joystick_magnitude  # noqa: E402
from q8bot.ui.main_window import MOVE_PAD_DEADZONE, POLL_INTERVAL_MS  # noqa: E402
from q8bot.ui.panels import ManualControlPanel  # noqa: E402
from q8bot.ui.widgets import JoystickPad  # noqa: E402

from .firmware import Firmware, SimLink  # noqa: E402
from .kinematics import move_body  # noqa: E402
from .sim import LEGS, SERVOS, Q8Sim  # noqa: E402

JOINTS, FEET, BODY, WALK, MOVES = range(5)                          # the tabs
PLACES = {'FL': (0, 0), 'FR': (0, 1), 'RL': (1, 0), 'RR': (1, 1)}     # the legs seen from above, nose up
NAMES = {'FL': 'Front left', 'FR': 'Front right', 'RL': 'Rear left', 'RR': 'Rear right'}
JOINT_RANGE = (-90.0, 270.0)            # deg - the sliders' span; the boxes take any angle
FOOT_RANGE = {'x': (-45.0, 65.0), 'y': (5.0, 65.0)}                   # mm, in the leg's frame
CHASSIS = (                             # key, label, +- range, unit, what + means
    ('forward', 'Forward', 25, ' mm', ''),
    ('up', 'Up', 25, ' mm', ''),
    ('roll', 'Roll', 30, '°', 'right side down'),
    ('pitch', 'Pitch', 30, '°', 'nose up'),
)
PADS = ManualControlPanel               # the desktop app's pads' full deflections: STANCE_MAX_X/Y, TILT_MAX
STANCE_MS = 500                         # Walk opens standing on the gait's stance, as choosing a gait does
# The gamepad's greet: q8botone-firmware/controller's ROBOT_GREET_MS, GREET_RETURN_MS
ROBOT_GREET_S, GREET_RETURN_MS = 6.45, 1000
SPEED_WINDOW_S = 1.0                    # the speed readout's span, simulated time
SATURATED = 0.95                        # a servo's torque shown as maxed out from here (share of stall)
LOOP_OPEN_MM = 0.5                      # a leg's loop shown as forced open from here
WARN = '#b3262b'                        # the desktop app's error red
TICK_MS = 10                            # physics and the 3D view
READOUT_MS = 100                        # the panel's numbers
CAMERA = {'distance': 0.33, 'azimuth': 135, 'elevation': -22}


def caption(text, wrap=False):
    label = QLabel(text)
    label.setEnabled(False)             # greyed: a note, not a control
    label.setWordWrap(wrap)
    return label


def readout():
    label = QLabel('-')
    label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    label.setMinimumWidth(52)
    return label


def status_line():
    '''A line of changing text that never widens the window: a longer line
    would otherwise push the panel's edge off the screen.'''
    label = QLabel()
    label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    return label


def show(label, text, warn=False):
    label.setText(text)
    label.setStyleSheet(f'color: {WARN};' if warn else '')


class Number(QObject):
    '''A slider and a spin box for one value, kept in step.'''
    changed = Signal(float)

    def __init__(self, lo, hi, suffix, box=None, decimals=1):
        super().__init__()
        self._k = 10 ** decimals
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(round(lo * self._k), round(hi * self._k))
        self.slider.setMinimumWidth(120)
        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(decimals)
        self.spin.setRange(*(box or (lo, hi)))
        self.spin.setSuffix(suffix)
        self.spin.setAlignment(Qt.AlignRight)
        self.spin.setKeyboardTracking(False)            # act on Enter, not on every digit typed
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def _from_slider(self, v):
        self.spin.blockSignals(True)
        self.spin.setValue(v / self._k)
        self.spin.blockSignals(False)
        self.changed.emit(self.spin.value())

    def _from_spin(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(round(value * self._k))
        self.slider.blockSignals(False)
        self.changed.emit(value)

    def value(self):
        return self.spin.value()

    def set(self, value):
        '''Show a value without acting on it.'''
        for w in (self.slider, self.spin):
            w.blockSignals(True)
        self.spin.setValue(value)
        self.slider.setValue(round(value * self._k))
        for w in (self.slider, self.spin):
            w.blockSignals(False)


class Poser(QWidget):
    def __init__(self, sim, viewer=None):
        super().__init__()
        self.sim, self.viewer = sim, viewer
        self.robot = Firmware(sim)                  # the robot's firmware, in the sim
        self.bot = Q8Bot(SimLink(self.robot))       # the library, driving it as it drives the robot
        self.setWindowTitle('Q8botOne poser')
        self._paused = False
        self._tab = JOINTS
        self._body_ref = None           # the planted feet, chassis frame, as the Body tab opened
        self._move = None               # a move playing: (name, its thread, wall-clock start)
        self._track = collections.deque()           # (sim time, x, y, heading) for the speed readout

        layout = QVBoxLayout(self)
        layout.addLayout(self._controls())
        self.tabs = QTabWidget()
        self.tabs.addTab(self._joints_tab(), 'Joints')
        self.tabs.addTab(self._feet_tab(), 'Feet')
        self.tabs.addTab(self._body_tab(), 'Body')
        self.tabs.addTab(self._walk_tab(), 'Walk')
        self.tabs.addTab(self._moves_tab(), 'Moves')
        self.tabs.currentChanged.connect(self._open_tab)
        layout.addWidget(self.tabs)
        layout.addLayout(self._readouts())
        layout.addWidget(caption('3D view: drag to turn it, scroll to zoom. Double-click a part, then '
                                 'Ctrl+right-drag to push it. Tab and Shift+Tab show MuJoCo\'s own options.',
                                 wrap=True))
        self._open_tab(JOINTS)

        self._wall = time.perf_counter()
        self._owed = 0.0                # simulated time behind the wall clock, s
        self._speed_from = (self._wall, sim.time)
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start(TICK_MS)
        self._readout_timer = QTimer(self)
        self._readout_timer.timeout.connect(self._update_readouts)
        self._readout_timer.start(READOUT_MS)
        self._pads_timer = QTimer(self)
        self._pads_timer.timeout.connect(self._poll_pads)
        self._pads_timer.start(POLL_INTERVAL_MS)

    # ---- building ---------------------------------------------------------

    def _controls(self):
        self.on_floor = QRadioButton('On the floor')
        self.on_stand = QRadioButton('On a test stand')
        self.on_floor.setChecked(True)
        self.on_floor.setToolTip('Free, standing on its feet')
        self.on_stand.setToolTip(f'Held still in the air, the servo shafts {self.sim.stand_height * 1000:.0f} mm up')
        places = QButtonGroup(self)
        places.addButton(self.on_floor)
        places.addButton(self.on_stand)
        self.on_stand.toggled.connect(self._set_stand)
        self.servos = QCheckBox('Servos on')
        self.servos.setChecked(True)
        self.servos.setToolTip('Off: the legs go limp. Back on: the servos hold the legs where they are.')
        self.servos.toggled.connect(self._set_servos)
        self.shapes = QCheckBox('Contact shapes')
        self.shapes.setToolTip('Show the simple shapes used for contact instead of the meshes: '
                               'the feet, the arms and the chassis box')
        self.shapes.toggled.connect(self._show_shapes)
        self.pause = QPushButton('Pause')
        self.pause.setCheckable(True)
        self.pause.toggled.connect(self._set_paused)
        self.reset_button = QPushButton('Reset')
        idle = self.sim.idle['FL']
        self.reset_button.setToolTip(f'Back to the idle pose ({idle[0]:g}°, {idle[1]:g}°), standing still')
        self.reset_button.clicked.connect(self._reset)
        row = QHBoxLayout()
        for w in (self.on_floor, self.on_stand):
            row.addWidget(w)
        row.addSpacing(24)
        for w in (self.servos, self.shapes):
            row.addWidget(w)
        row.addStretch()
        row.addWidget(self.pause)
        row.addWidget(self.reset_button)
        return row

    def _leg_grid(self, fill):
        '''The four legs' boxes, laid out as on the robot seen from above.'''
        page = QWidget()
        grid = QGridLayout(page)
        for leg, (r, c) in PLACES.items():
            box = QGroupBox(f'{NAMES[leg]} ({leg})')
            fill(leg, QGridLayout(box))
            grid.addWidget(box, r, c)
        return page, grid

    def _joints_tab(self):
        self.joint, self.joint_actual, self.joint_effort = {}, {}, {}
        stall = self.sim.model.actuator_forcerange[0, 1]

        def fill(leg, g):
            g.addWidget(caption('target'), 0, 1, 1, 2)
            g.addWidget(caption('actual'), 0, 3, Qt.AlignRight)
            effort = caption('torque')
            effort.setToolTip(f'Share of the servo\'s {stall:g} N·m stall torque')
            g.addWidget(effort, 0, 4, Qt.AlignRight)
            for i, s in enumerate(SERVOS, 1):
                n = self.joint[leg, s] = Number(*JOINT_RANGE, '°', box=(-360.0, 360.0))
                n.changed.connect(lambda v, leg=leg, s=s: self._joint_changed(leg, s, v))
                g.addWidget(QLabel(s.capitalize()), i, 0)
                g.addWidget(n.slider, i, 1)
                g.addWidget(n.spin, i, 2)
                g.addWidget(actual := readout(), i, 3)
                g.addWidget(effort := readout(), i, 4)
                self.joint_actual[leg, s], self.joint_effort[leg, s] = actual, effort
            g.setColumnStretch(1, 1)

        page, grid = self._leg_grid(fill)
        grid.addWidget(caption('Servo angles in degrees below forward, as the library has them.'), 2, 0, 1, 2)
        return page

    def _feet_tab(self):
        self.foot, self.foot_actual, self.foot_status = {}, {}, {}

        def fill(leg, g):
            g.addWidget(caption('target'), 0, 1, 1, 2)
            g.addWidget(caption('actual'), 0, 3, Qt.AlignRight)
            for i, k in enumerate('xy', 1):
                n = self.foot[leg, k] = Number(*FOOT_RANGE[k], ' mm')
                n.changed.connect(lambda v, leg=leg: self._foot_changed(leg))
                g.addWidget(QLabel(k), i, 0)
                g.addWidget(n.slider, i, 1)
                g.addWidget(n.spin, i, 2)
                g.addWidget(actual := readout(), i, 3)
                self.foot_actual[leg, k] = actual
            g.addWidget(status := status_line(), 3, 0, 1, 4)
            self.foot_status[leg] = status
            g.setColumnStretch(1, 1)

        page, grid = self._leg_grid(fill)
        grid.addWidget(caption('x forward and y down from the leg\'s rear servo shaft, mm - the library\'s leg '
                               'frame. A foot out of reach leaves its servos where they were.', wrap=True),
                       2, 0, 1, 2)
        return page

    def _body_tab(self):
        page = QWidget()
        v = QVBoxLayout(page)
        box = QGroupBox('Chassis, over the planted feet')
        g = QGridLayout(box)
        self.body = {}
        for i, (key, label, span, unit, plus) in enumerate(CHASSIS):
            n = self.body[key] = Number(-span, span, unit)
            n.changed.connect(self._body_changed)
            g.addWidget(QLabel(label), i, 0)
            g.addWidget(n.slider, i, 1)
            g.addWidget(n.spin, i, 2)
            if plus:
                g.addWidget(caption(f'+ is {plus}'), i, 3)
        g.setColumnStretch(1, 1)
        zero = QPushButton('Zero')
        zero.setToolTip('Back to the pose this tab was opened on')
        zero.clicked.connect(self._zero_body)
        g.addWidget(zero, len(CHASSIS), 0)
        g.addWidget(status := status_line(), len(CHASSIS), 1, 1, 3)
        self.body_status = status
        v.addWidget(box)
        v.addWidget(caption('The feet stay where they are and the chassis moves over them; zero is the pose '
                            'the tab was opened on. Roll and pitch turn about the middle of the servo shafts. '
                            'A foot only moves in its leg\'s plane, so the sideways part of a roll is left out.',
                            wrap=True))
        v.addStretch()
        return page

    def _walk_tab(self):
        '''The desktop app's Manual Control pads (q8bot.ui), laid out and
        driving the robot as they do there.'''
        page = QWidget()
        grid = QGridLayout(page)
        self.move_pad, self.stance_pad, self.tilt_pad = JoystickPad(), JoystickPad(), JoystickPad()
        self.move_pad.setToolTip('Drag to walk; release to stop. Push further to go faster.')
        self.stance_pad.setToolTip('Drag to shift the standing pose; release to return to the baseline. Idle only.')
        self.tilt_pad.setToolTip('Drag to tilt the body; release to level off. Idle only.')
        self.stance_readout, self.tilt_readout = QLabel(), QLabel()
        # Sized for the widest each can show, so a pad never shifts sideways as its numbers grow
        for label, widest in ((self.stance_readout, self._stance_text(-1, -1)),
                              (self.tilt_readout, self._tilt_text(-1, -1))):
            label.setFixedWidth(label.fontMetrics().horizontalAdvance(widest) + 4)
            label.setAlignment(Qt.AlignCenter)
        for pad in (self.stance_pad, self.tilt_pad):
            pad.moved.connect(self._show_pads)
        for column, (title, pad, readout, hint) in enumerate((
                ('Move', self.move_pad, None, 'Direction / speed'),
                ('Stance', self.stance_pad, self.stance_readout, 'X / Y offset'),
                ('Body Tilt', self.tilt_pad, self.tilt_readout, 'Roll / Pitch'))):
            heading = QLabel(title)
            font = heading.font()
            font.setBold(True)
            heading.setFont(font)
            grid.addWidget(heading, 0, column, Qt.AlignCenter)
            grid.addWidget(QLabel(hint), 1, column, Qt.AlignCenter)
            grid.addWidget(pad, 2, column, Qt.AlignCenter)
            if readout is not None:
                grid.addWidget(readout, 3, column, Qt.AlignCenter)

        self.gait = QComboBox()
        for info in self.bot.catalog:
            self.gait.addItem(info.label, info.name)
            self.gait.setItemData(self.gait.count() - 1, info.description, Qt.ToolTipRole)
        self.gait.activated.connect(self._gait_chosen)
        gait_row = QHBoxLayout()
        gait_row.addWidget(QLabel('Gait'))
        gait_row.addWidget(self.gait, 1)
        grid.addLayout(gait_row, 3, 0)

        self.hold_pose = QPushButton('Hold Pose: OFF')
        self.hold_pose.setCheckable(True)
        self.hold_pose.setToolTip('Keep a pad wherever you drag it instead of springing back to centre on release.')
        self.hold_pose.setMinimumWidth(self.hold_pose.fontMetrics().horizontalAdvance('Hold Pose: OFF') + 24)
        self.hold_pose.toggled.connect(self._hold_pads)
        self.in_place = QPushButton('Trot in Place')
        self.in_place.setCheckable(True)
        self.in_place.setToolTip('Trot with no stride (TROT_INPLACE) while the move pad is let go - '
                                 'what the gamepad\'s left trigger does')
        actions = QVBoxLayout()
        actions.addStretch()
        actions.addWidget(self.hold_pose)
        actions.addWidget(self.in_place)
        actions.addStretch()
        grid.addLayout(actions, 0, 3, 4, 1)
        grid.addWidget(caption('The desktop app\'s own pads, driving the library the same way. The robot walks '
                               'its flashed gait tables - the Robot line below names the one playing. Stance and '
                               'Body Tilt work while it stands.', wrap=True), 4, 0, 1, 4)
        grid.setHorizontalSpacing(28)
        grid.setColumnStretch(3, 1)
        grid.setRowStretch(5, 1)
        self._show_pads()
        self._show_gait()
        return page

    def _moves_tab(self):
        page = QWidget()
        v = QVBoxLayout(page)
        bot = self.bot
        taught = [(name, lambda path=path: bot.play_routine(routine_store.load(path)))
                  for name, path in routine_store.list_routines()]
        groups = (
            ('Played by the robot', 'Sent as one command, played by the firmware. This Jump is the desktop '
             'app\'s and the gamepad\'s; this Greet is the gamepad\'s.',
             (('Jump', bot.jump), ('Greet', self._robot_greet))),
            ('Played by the computer', 'The library\'s routines, sent a pose at a time. This Greet is the '
             'desktop app\'s - the same poses as the robot\'s.',
             (('Greet', bot.greet), ('Show range', bot.show_range), ('Shoot', bot.shoot))),
            ('Taught', 'Saved by the desktop app\'s Teach dialog, in the library\'s routines folder.' if taught
             else 'None saved yet - the desktop app\'s Teach dialog saves them in the library\'s routines folder.',
             taught),
        )
        self.move_buttons = []
        for title, about, moves in groups:
            box = QGroupBox(title)
            g = QVBoxLayout(box)
            row = QHBoxLayout()
            for name, play in moves:
                b = QPushButton(name)
                b.clicked.connect(lambda _=False, name=name, play=play: self._play(name, play))
                row.addWidget(b)
                self.move_buttons.append(b)
            row.addStretch()
            g.addLayout(row)
            g.addWidget(note := caption(about, wrap=True))
            if title == 'Taught':
                note.setToolTip(str(routine_store.DEFAULT_DIR))
            v.addWidget(box)
        self.move_status = status_line()
        v.addWidget(self.move_status)
        v.addStretch()
        return page

    def _readouts(self):
        form = QFormLayout()
        self.robot_label, self.chassis_label, self.speed_label, self.feet_label, self.loops_label, self.time_label = (
            status_line() for _ in range(6))
        form.addRow('Robot', self.robot_label)
        form.addRow('Chassis', self.chassis_label)
        form.addRow('Speed', self.speed_label)
        form.addRow('Feet down', self.feet_label)
        form.addRow('Leg loops', self.loops_label)
        form.addRow('Time', self.time_label)
        form.labelForField(self.robot_label).setToolTip('What the robot\'s firmware is doing')
        form.labelForField(self.chassis_label).setToolTip('z is the servo shafts\' height above the floor')
        form.labelForField(self.speed_label).setToolTip('Over the last second of simulated time; '
                                                        'left is sideways, to the robot\'s left')
        return form

    # ---- posing -----------------------------------------------------------

    def _open_tab(self, i):
        '''A tab takes the robot over from wherever the last one left it.'''
        if self._tab == WALK:
            self._stop_walking()
        self._tab = i
        if i == WALK:
            self.bot.stand(STANCE_MS)
            self.robot.drain()
            self._show_gait()
        if i in (WALK, MOVES):
            return
        targets = self.robot.targets()
        if i == JOINTS:
            for (leg, s), n in self.joint.items():
                n.set(targets[leg][SERVOS.index(s)])
        elif i == FEET:
            for leg in LEGS:
                foot = self.sim.leg.fk(*targets[leg])
                if foot is None:
                    show(self.foot_status[leg], 'Its servo targets put the foot nowhere', warn=True)
                    continue
                self.foot[leg, 'x'].set(foot[0])
                self.foot[leg, 'y'].set(foot[1])
                show(self.foot_status[leg], self._servo_text(targets[leg]))
        else:
            for n in self.body.values():
                n.set(0.0)
            feet = {leg: self.sim.leg.fk(*targets[leg]) for leg in LEGS}
            lost = [leg for leg, f in feet.items() if f is None]
            if lost:
                self._body_ref = None
                show(self.body_status, f'{", ".join(lost)}: servo targets that put the foot nowhere', warn=True)
            else:
                self._body_ref = {leg: self.sim.leg_to_chassis(leg, *f) for leg, f in feet.items()}
                show(self.body_status, '')

    @staticmethod
    def _servo_text(q, held=False):
        return f'{"Out of reach - servos held at" if held else "Servos"} {q[0]:.1f}°, {q[1]:.1f}°'

    def _solve(self, leg, x, y):
        '''Servo angles for a foot target, each the turn nearest its current
        target (the model's joints count whole turns), or None.'''
        q = self.sim.leg.ik(x, y)
        if q is None:
            return None
        now = self.robot.targets()[leg]
        return tuple(a + 360 * round((c - a) / 360) for a, c in zip(q, now))

    def _joint_changed(self, leg, servo, value):
        front, rear = self.robot.targets()[leg]
        self.robot.pose({leg: (value, rear) if servo == 'front' else (front, value)})

    def _foot_changed(self, leg):
        q = self._solve(leg, self.foot[leg, 'x'].value(), self.foot[leg, 'y'].value())
        if q is None:
            show(self.foot_status[leg], self._servo_text(self.robot.targets()[leg], held=True), warn=True)
            return
        self.robot.pose({leg: q})
        show(self.foot_status[leg], self._servo_text(q))

    def _body_changed(self):
        if self._body_ref is None:
            return
        feet = move_body(self._body_ref, **{k: n.value() for k, n in self.body.items()})
        solved = {leg: self._solve(leg, *self.sim.chassis_to_leg(leg, p)) for leg, p in feet.items()}
        lost = [leg for leg, q in solved.items() if q is None]
        if lost:
            show(self.body_status, f'Out of reach for {", ".join(lost)} - the servos stay where they were', warn=True)
            return
        self.robot.pose(solved)
        show(self.body_status, '')

    def _zero_body(self):
        for n in self.body.values():
            n.set(0.0)
        self._body_changed()

    # ---- walking ----------------------------------------------------------

    def _poll_pads(self):
        '''The desktop app's MainWindow._poll_manual(): walking wins over
        posing, and the stance and tilt go into one set_stance() so they
        compose. Only while the Walk tab has the robot.'''
        if self._tab != WALK or self._move is not None or not self.robot.torque_on:
            return
        mx, my = self.move_pad.value()
        if math.hypot(mx, my) >= MOVE_PAD_DEADZONE:
            # The gamepad's own mapping, whose vertical axis is negative-is-forward
            direction = get_joystick_direction(mx, -my, analog_mode=True)
            if direction:
                self.bot.move(direction, get_joystick_magnitude(mx, -my))
                return
        if self.in_place.isChecked():
            self.bot.trot_in_place()
            return
        self.bot.idle()                             # a no-op once idle
        x, y = self.stance_pad.value()
        roll, pitch = self.tilt_pad.value()
        self.bot.set_stance(dx=x * PADS.STANCE_MAX_X, dy=y * PADS.STANCE_MAX_Y,
                            roll=roll * PADS.TILT_MAX, pitch=pitch * PADS.TILT_MAX)

    @staticmethod
    def _stance_text(x, y):
        return f'X {x * PADS.STANCE_MAX_X:+.1f} mm    Y {y * PADS.STANCE_MAX_Y:+.1f} mm'

    @staticmethod
    def _tilt_text(roll, pitch):
        return f'Roll {roll * PADS.TILT_MAX:+.1f}°    Pitch {pitch * PADS.TILT_MAX:+.1f}°'

    def _show_pads(self, *_):
        self.stance_readout.setText(self._stance_text(*self.stance_pad.value()))
        self.tilt_readout.setText(self._tilt_text(*self.tilt_pad.value()))

    def _show_gait(self):
        '''The move pad as the gait has it: a gait that can't turn locks it to
        forward and back.'''
        self.gait.setCurrentIndex(self.gait.findData(self.bot.gait))
        self.gait.setToolTip(self.gait.currentData(Qt.ToolTipRole) or '')
        self.move_pad.set_horizontal_enabled(self.bot.can_turn)

    def _gait_chosen(self, i):
        self._stop_walking()
        self.bot.set_gait(self.gait.itemData(i))    # loads it and stands on its stance
        self.robot.drain()
        self._show_gait()

    def _hold_pads(self, held):
        for pad in (self.move_pad, self.stance_pad, self.tilt_pad):
            pad.set_hold(held)
        self.hold_pose.setText(f'Hold Pose: {"ON" if held else "OFF"}')
        self.hold_pose.setStyleSheet(f'QPushButton {{ background-color: {JoystickPad.HOLD_COLOR.name()}; '
                                     'color: white; }' if held else '')

    def _stop_walking(self):
        '''Let go of the pads: the gait stops, standing on its stance
        (Q8Bot.idle()), and a stance or tilt eases back to the baseline.'''
        for pad in (self.move_pad, self.stance_pad, self.tilt_pad):
            pad.centre()
        self.in_place.setChecked(False)
        self.bot.idle()
        self.bot.set_stance()
        self.robot.drain()

    # ---- moves ------------------------------------------------------------

    def _play(self, name, play):
        '''A move, on a thread of its own - Q8Bot's moves take as long as they
        play - with everything that would take the robot from it held off.'''
        thread = threading.Thread(target=play, daemon=True)
        self._move = (name, thread, time.perf_counter())
        self._refresh_enabled()
        show(self.move_status, f'Playing {name}...')
        thread.start()

    def _robot_greet(self):
        '''The gamepad's greet (the controller on its own): the robot's greet,
        with the move back to the stance queued on the robot behind it.'''
        self.bot.idle()
        self.bot.link.send_greet()
        self.bot.stand(GREET_RETURN_MS)
        time.sleep(ROBOT_GREET_S + GREET_RETURN_MS / 1000)

    def _refresh_enabled(self):
        '''While a move plays, nothing that would take the robot from it. While
        paused, no moves: the computer times them on the wall clock.'''
        playing = self._move is not None
        for w in (self.tabs.tabBar(), self.servos, self.pause, self.reset_button):
            w.setEnabled(not playing)
        for b in self.move_buttons:
            b.setEnabled(not playing and not self._paused)

    # ---- the controls above the tabs ---------------------------------------

    def _set_stand(self, on):
        self.sim.set_on_stand(on)
        self._track.clear()

    def _set_servos(self, on):
        if not on:
            self._stop_walking()
        # Back on, the firmware holds the legs where they are: snapping them
        # back to the old targets at full torque can throw the robot onto its back.
        self.robot.set_torque(on)
        if on:
            self._open_tab(self.tabs.currentIndex())
        self.tabs.setEnabled(on)

    def _show_shapes(self, on):
        if self.viewer is not None:
            with self.viewer.lock():
                self.viewer.opt.geomgroup[3] = on

    def _set_paused(self, paused):
        self._paused = paused
        self.pause.setText('Resume' if paused else 'Pause')
        self._refresh_enabled()

    def _reset(self):
        self._stop_walking()
        self.sim.reset()
        self.robot.reset()
        self._track.clear()
        self._open_tab(self.tabs.currentIndex())

    # ---- running ----------------------------------------------------------

    def _tick(self):
        '''Physics in step with the wall clock, then the 3D view.'''
        now = time.perf_counter()
        elapsed, self._wall = min(now - self._wall, 0.05), now     # after a stall, slow down rather than leap
        if not self._paused:
            self._owed += elapsed
            n = int(self._owed / self.sim.model.opt.timestep)
            if n:
                self.robot.step(n)
                self._owed -= n * self.sim.model.opt.timestep
        else:
            self.robot.drain()
        if self.viewer is not None:
            if not self.viewer.is_running():
                self.close()
                return
            self.viewer.sync()

    def _update_readouts(self):
        sim, tab = self.sim, self.tabs.currentIndex()
        if self._move is not None and not self._move[1].is_alive():
            name, _, start = self._move
            self._move = None
            show(self.move_status, f'{name}: done, {time.perf_counter() - start:.1f} s')
            self._refresh_enabled()
        if tab == JOINTS:
            angles, effort = sim.angles(), sim.effort()
            for leg in LEGS:
                for i, s in enumerate(SERVOS):
                    self.joint_actual[leg, s].setText(f'{angles[leg][i]:.1f}°')
                    e = effort[leg][i]
                    show(self.joint_effort[leg, s], f'{e:.0%}', warn=e >= SATURATED)
        elif tab == FEET:
            for leg in LEGS:
                x, y = sim.foot(leg)
                self.foot_actual[leg, 'x'].setText(f'{x:.1f}')
                self.foot_actual[leg, 'y'].setText(f'{y:.1f}')
        self.robot_label.setText(self.robot.doing())
        x, y, z, roll, pitch = sim.chassis()
        heading = sim.heading()
        self.chassis_label.setText(f'x {x:+.1f}   y {y:+.1f}   z {z:.1f} mm      heading {heading:+.0f}°   '
                                   f'roll {roll:+.1f}°   pitch {pitch:+.1f}°')
        self.speed_label.setText(self._speed_text(x, y, heading))
        down = sim.feet_down()
        self.feet_label.setText('  '.join(down) if down else 'none')
        gap = sim.loop_gap()
        if gap < LOOP_OPEN_MM:
            show(self.loops_label, f'closed, to {gap:.3f} mm')
        else:
            show(self.loops_label, f'pulled {gap:.1f} mm open - a leg is being forced', warn=True)
        wall, t = time.perf_counter(), sim.time
        speed = (t - self._speed_from[1]) / max(wall - self._speed_from[0], 1e-6)
        self._speed_from = (wall, t)
        self.time_label.setText(f'{t:.1f} s simulated, ' + ('paused' if self._paused else f'{speed:.0%} of real time'))

    def _speed_text(self, x, y, heading):
        '''Forward, sideways and turning speed over the last SPEED_WINDOW_S.'''
        t, track = self.sim.time, self._track
        if track and t < track[-1][0]:
            track.clear()                           # reset: time went back
        if not track or t > track[-1][0]:
            if track:                               # heading counts on through whole turns
                heading = track[-1][3] + (heading - track[-1][3] + 180) % 360 - 180
            track.append((t, x, y, heading))
        while t - track[0][0] > SPEED_WINDOW_S:
            track.popleft()
        t0, x0, y0, h0 = track[0]
        _, x1, y1, h1 = track[-1]
        span = t - t0
        if span < SPEED_WINDOW_S / 2:
            return '-'
        h = math.radians((h0 + h1) / 2)
        dx, dy = x1 - x0, y1 - y0
        return (f'forward {(dx * math.cos(h) + dy * math.sin(h)) / span:+.0f}   '
                f'left {(dy * math.cos(h) - dx * math.sin(h)) / span:+.0f} mm/s      '
                f'turning {(h1 - h0) / span:+.0f}°/s')

    def closeEvent(self, event):
        for timer in (self._tick_timer, self._readout_timer, self._pads_timer):
            timer.stop()
        if self._move is None:          # mid-move, the library closes it at exit, once the move ends
            self.bot.close()
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.close()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    sim = Q8Sim()
    viewer = mujoco.viewer.launch_passive(sim.model, sim.data, show_left_ui=False, show_right_ui=False)
    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING      # follows the chassis
        viewer.cam.trackbodyid = sim.model.body('chassis').id
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = (
            CAMERA['distance'], CAMERA['azimuth'], CAMERA['elevation'])
    poser = Poser(sim, viewer)
    poser.show()

    def place():
        '''Against the screen's right edge - once shown, when the window frame's size is known.'''
        screen = poser.screen().availableGeometry()
        frame = poser.frameGeometry()
        poser.move(max(screen.left(), screen.right() - frame.width() - 16), screen.top() + 40)

    QTimer.singleShot(0, place)
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
