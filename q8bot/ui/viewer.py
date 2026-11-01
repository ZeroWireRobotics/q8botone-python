'''
Written by yufeng.wu0902@gmail.com

The 3D viewer panel. With the simulator installed (q8botone-sim, which
brings MuJoCo) it shows the robot's simulated model, live, and a switch picks
what the model follows:

  Controls  what the controls send the robot: every command also goes to the
            simulated robot, which plays it the way the robot's firmware does
  Robot     what the robot measures: its eight joint angles from telemetry,
            shown on a test stand - the joints alone don't say how the robot
            is tilted. With an MPU6050 on the robot's Qwiic port, once it's
            calibrated (Calibrate IMU: the robot tilts itself a few ways), the
            model comes off the stand: the IMU tilts and turns it, its lowest
            point rests on the floor, and its feet on the floor walk it along.

The simulated robot can also be connected in place of a real one
(SIMULATED_ROBOT in the Connection panel's list), to drive the app with no
hardware at all: the model is then the robot, following the controls.

Without the simulator it shows the robot as a single STL you can orbit.

STL rather than STEP on purpose. STEP is BREP/NURBS - a CAD exchange format
that no real-time viewer can draw directly; it has to be tessellated into
triangles first, and doing that at runtime means dragging in OpenCASCADE.
Exporting a mesh once from CAD is the cheaper trade by a wide margin.

The STL is one rigid body. Articulating it is the simulator's job: its model
is built from the CAD's separate parts, the five-bar legs solved and all.

Drop a .stl into q8bot/assets/ and it gets picked up; without one the panel
falls back to a message saying so.
'''

import struct
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget

from ..helpers import Q8Logger
from ..protocol import ticks_to_degrees
from .widgets import ClickableLabel, ToggleSwitch


def _shaded_from_above():
    """
    Replaces pyqtgraph's stock 'shaded' shader, which lights from below and
    behind - from any sensible camera angle almost every visible face lands
    on its 0.2 ambient floor, so the model reads as a flat silhouette no
    matter what colour it is. This lights mainly from above with a much
    higher ambient, so faces separate instead of crushing together.
    """
    from pyqtgraph.opengl.shaders import FragmentShader, ShaderProgram, VertexShader

    return ShaderProgram('q8shaded', [
        VertexShader("""
            uniform mat4 u_mvp;
            uniform mat3 u_normal;
            attribute vec4 a_position;
            attribute vec3 a_normal;
            attribute vec4 a_color;
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                v_normal = normalize(u_normal * a_normal);
                v_color = a_color;
                gl_Position = u_mvp * a_position;
            }
        """),
        FragmentShader("""
            #ifdef GL_ES
            precision mediump float;
            #endif
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                float key = max(dot(v_normal, normalize(vec3(0.35, 0.35, 0.87))), 0.0);
                float fill = max(dot(v_normal, normalize(vec3(-0.6, -0.3, 0.2))), 0.0);
                float light = 0.40 + 0.50 * key + 0.14 * fill;
                gl_FragColor = vec4(v_color.rgb * light, v_color.a);
            }
        """),
    ])


def _asset_dir():
    # PyInstaller unpacks datas under _MEIPASS rather than beside the module.
    bundled = getattr(sys, '_MEIPASS', None)
    if bundled:
        return Path(bundled) / 'q8bot' / 'assets'
    return Path(__file__).resolve().parent.parent / 'assets'


ASSET_DIR = _asset_dir()
PREFERRED_NAME = 'q8bot.stl'

# Two different colour conventions in the same library, so mind which is
# which: GLMeshItem unpacks its colour straight into glVertexAttrib4f and
# needs 0-1 floats, while setBackgroundColor and GLGridItem go through
# mkColor(), which reads floats as 0-255 ints and turns them all black.
MODEL_COLOR = (0.60, 0.66, 0.75, 1.0)
BACKGROUND = '#eef1f5'
GRID_COLOR = (120, 132, 148, 90)

# The live model: how often it's drawn (and the simulation caught up to the
# clock), and how often to ask for the robot's joint angles while it follows
# the robot - about 5% of the servo bus. Its feet walk it along only as well
# as they're seen: a trot's stance lasts 90 ms, and at 30 Hz the model
# covered 60% of the robot's distance in the sim, at 50 Hz 85%.
FRAME_MS = 33
FOLLOW_TELEMETRY_HZ = 50
# And its IMU: the gyro's turns are summed reading by reading, so it needs
# as many - a quick turn between two readings is lost (after a fast spin the
# tilt was ~10 deg out at 30 Hz, ~4 at 50, ~1.5 at 100, in the sim). Neither
# rate has been tried on the robot's radio link yet.
FOLLOW_IMU_HZ = 50
IMU_STALE_S = 1.0                   # an IMU silent this long is gone: back onto the stand

# The IMU's calibrated mounting - this robot's, so like the taught routines
# it's kept out of git (see .gitignore)
IMU_MOUNT_FILE = Path(__file__).resolve().parent.parent / 'imu_mount.json'

# The Connection panel's name for the simulated robot - never a serial port's
SIMULATED_ROBOT = "Simulated robot"


def _simulator():
    """
    The simulator's pieces - Q8Sim, Firmware, SimLink, MirrorLink, ModelView
    and the imu module, by name - or None if q8botone-sim isn't installed.
    It's optional: a separate repository, and MuJoCo is a large download.
    """
    try:
        from q8botone_sim import imu
        from q8botone_sim.firmware import Firmware, MirrorLink, SimLink
        from q8botone_sim.qtview import ModelView
        from q8botone_sim.sim import Q8Sim
    except ImportError:
        return None
    return SimpleNamespace(Q8Sim=Q8Sim, Firmware=Firmware, SimLink=SimLink, MirrorLink=MirrorLink,
                           ModelView=ModelView, imu=imu)


def find_model(asset_dir=None):
    """The STL to display: q8bot.stl if present, else any other in the folder."""
    folder = Path(asset_dir) if asset_dir else ASSET_DIR
    if not folder.is_dir():
        return None
    preferred = folder / PREFERRED_NAME
    if preferred.is_file():
        return preferred
    return next(iter(sorted(folder.glob('*.stl'))), None)


def load_stl(path):
    """
    Read an STL into (vertices, faces) arrays.

    Handles both binary and ASCII. Binary is the common case and is a fixed
    50-byte record per triangle, so it needs no parser to speak of; ASCII is
    slower but rare enough not to matter.

    Returns:
        (vertices Nx3 float32, faces Mx3 int32)
    """
    data = Path(path).read_bytes()

    if _is_ascii_stl(data):
        points = [
            [float(n) for n in line.split()[1:4]]
            for line in data.decode('utf-8', 'replace').splitlines()
            if line.strip().startswith('vertex')
        ]
        vertices = np.array(points, dtype=np.float32)
    else:
        triangle_count = struct.unpack('<I', data[80:84])[0]
        records = np.frombuffer(data, dtype=np.uint8, count=triangle_count * 50, offset=84)
        records = records.reshape(triangle_count, 50)
        # Each record is normal(3f) + 3 vertices(3f each) + attribute(uint16);
        # skip the normal and the trailing attribute, keep the 9 floats.
        vertices = records[:, 12:48].copy().view(np.float32).reshape(-1, 3)

    faces = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
    return vertices, faces


def _is_ascii_stl(data):
    # A binary STL's size is exactly the header plus its declared triangles;
    # checking that beats trusting the "solid" prefix, which binary files
    # written by some CAD tools also carry.
    if len(data) < 84:
        return True
    triangle_count = struct.unpack('<I', data[80:84])[0]
    return len(data) != 84 + triangle_count * 50


class LiveModel(QWidget):
    """
    The simulated robot, following either the controls or the robot (see
    the module docstring). MainWindow builds its Q8Bot on mirror()'s link,
    so every command the robot gets reaches the simulation too - or, with
    no robot, on simulated_link(), the simulation alone - and hands the bot
    over with set_robot() for its telemetry.
    """

    follow_robot_changed = Signal(bool)
    calibrate_requested = Signal()          # Calibrate IMU - MainWindow runs calibrate_imu()

    def __init__(self, simulator, parent=None):
        super().__init__(parent)
        self._mirror_link = simulator.MirrorLink
        self._imu = simulator.imu
        self.sim = simulator.Q8Sim()
        self.firmware = simulator.Firmware(self.sim)    # the robot's firmware, run against the model
        self._sim_link = simulator.SimLink(self.firmware)
        self.bot = None
        self.simulated = False                  # the bot is on simulated_link(): the model is the robot
        self._shown = None                      # the telemetry sample the model was last posed from
        self._wall = time.perf_counter()
        self._owed = 0.0                        # simulated time behind the clock, s
        # The robot's IMU, while following it: its calibrated mounting (None
        # until calibrated), the orientation tracked from it, and the robot
        # clock time of the last reading taken in
        self.mount = self._imu.Mount.load(IMU_MOUNT_FILE)
        self._attitude = None
        self._imu_t = None
        self._new_mount = None                  # handed over by calibrate_imu() on the worker
        self.calibrating = False

        self.view = simulator.ModelView(self.sim)
        self.view.setToolTip("Drag to turn, scroll to zoom.")
        self.toggle = ToggleSwitch()
        self.toggle.toggled.connect(self._follow_changed)
        self.controls_label = ClickableLabel("Controls", lambda: self.toggle.setChecked(False))
        self.robot_label = ClickableLabel("Robot", lambda: self.toggle.setChecked(True))
        self._tips = {
            self.controls_label: "The model does what the controls send the robot, simulated",
            self.toggle: "",
            self.robot_label: "The model takes the robot's measured joint angles",
        }
        for widget, tip in self._tips.items():
            widget.setToolTip(tip)
        self.status = QLabel()
        # Its text changes all the time; a longer line mustn't widen the window
        self.status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.status.setEnabled(False)
        self.reset_button = QPushButton("Reset")
        self.reset_button.setToolTip("Put the model back where it started, in the middle of the floor")
        self.reset_button.clicked.connect(self.reset)
        self.calibrate_button = QPushButton("Calibrate IMU")
        self.calibrate_button.setToolTip(
            "Find which way the robot's MPU6050 is mounted, so the model can show the robot's tilt.\n"
            "The robot stands and tilts its body four ways on its own feet, for about 15 s:\n"
            "have it on a flat floor, motors on, and leave it be until it's done.")
        self.calibrate_button.clicked.connect(self.calibrate_requested)
        self.calibrate_button.hide()

        row = QHBoxLayout()
        row.setContentsMargins(8, 6, 8, 0)
        row.addWidget(QLabel("Model follows"))
        row.addSpacing(6)
        for widget in (self.controls_label, self.toggle, self.robot_label):
            row.addWidget(widget)
        row.addSpacing(12)
        row.addWidget(self.status, 1)
        row.addWidget(self.calibrate_button)
        row.addWidget(self.reset_button)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(row)
        layout.addWidget(self.view, 1)
        self._paint_follow()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(FRAME_MS)

    @property
    def follows_robot(self):
        return self.toggle.isChecked()

    def mirror(self, link):
        """The robot's link, with every command copied to the simulation."""
        return self._mirror_link(link, self._sim_link)

    def simulated_link(self):
        """A link to the simulation itself, standing in for a robot's."""
        return self._sim_link

    def set_robot(self, bot):
        """
        The connected Q8Bot (built on mirror()'s link or simulated_link()),
        or None. Before the bot sends anything: following the controls again
        restarts the simulated firmware, dropping what's queued for it.
        """
        self.bot = bot
        self._shown = None
        self._attitude = None
        self.simulated = bot is not None and bot.link is self._sim_link
        if self.simulated:
            self.toggle.setChecked(False)       # there's no other robot to follow
        tip = "Connected to the simulated robot - the model is the robot" if self.simulated else ""
        for widget in (self.controls_label, self.toggle, self.robot_label):
            widget.setEnabled(not self.simulated)
            widget.setToolTip(tip or self._tips[widget])

    def reset(self):
        """The model back where it started, in the middle of the floor facing
        forward - upright, on its feet (or on the stand, while it follows the
        robot) - its legs as they are."""
        self.sim.recentre()
        if not self.follows_robot:
            self._simulate_on()
        elif self._attitude is not None:
            self._attitude.set_heading(0.0)     # (the IMU still says how it's tilted)
        self._shown = None                      # the robot's pose again, from its next joint angles
        self.view.refresh()

    def _simulate_on(self):
        """Following the controls from here: the simulated firmware restarted on
        the model's pose, motors as the robot has them."""
        if self.bot is not None:
            self.sim.set_torque(self.bot.torque)
        self.firmware.reset()
        self._owed = 0.0

    def _follow_changed(self, robot):
        # The robot's joint angles alone don't say how it's tilted or where
        # it stands, so the model shows them held still on the test stand -
        # until a calibrated IMU answers (see _follow_robot)
        self.sim.set_on_stand(robot)
        self.sim.show_imu(None)
        self._attitude = None
        if not robot:
            self._simulate_on()                 # set down on the floor in the robot's last pose
        self._shown = None
        self._paint_follow()
        self.follow_robot_changed.emit(robot)

    def _paint_follow(self):
        active, idle = QFont(), QFont()
        active.setBold(True)
        self.robot_label.setFont(active if self.follows_robot else idle)
        self.controls_label.setFont(idle if self.follows_robot else active)

    def _tick(self):
        now = time.perf_counter()
        elapsed, self._wall = min(now - self._wall, 0.05), now     # after a stall, slow down rather than leap
        if self.follows_robot:
            self.firmware.inbox.clear()         # the robot plays its commands; the model shows the outcome
            text = self._follow_robot()
        else:
            self._owed += elapsed
            step = self.sim.model.opt.timestep
            n = int(self._owed / step)
            if n:
                self.firmware.step(n)
                self._owed -= n * step
            text = f"Simulated: {self.firmware.doing()}"
            if self.bot is None:
                text += " - connect a robot, or the simulated one, to drive it"
        self.status.setText(text)
        self.status.setToolTip(text)            # (in case it's cut short)
        self.view.refresh()

    def _follow_robot(self):
        """
        One frame of following the robot: its newest joint angles, on the
        stand - or, with a calibrated IMU answering, turned as the IMU has
        it and stood on the floor. Returns the status line.
        """
        imu = self._imu
        if self._new_mount is not None:         # a calibration just finished
            self.mount, self._new_mount = self._new_mount, None
            self._attitude = None
        bot = self.bot
        if bot is None:
            self.calibrate_button.hide()
            return "Connect a robot to see its pose"
        sample = bot.telemetry
        latest = imu.reading(bot.sensors)
        age = bot.sensor_age
        has_imu = latest is not None and age is not None and age < IMU_STALE_S
        self.calibrate_button.setVisible(has_imu and not self.simulated)
        self.calibrate_button.setEnabled(not self.calibrating)
        self.calibrate_button.setText("Recalibrate IMU" if self.mount is not None else "Calibrate IMU")
        use_imu = has_imu and self.mount is not None and imu.reading(bot.sensors, self.mount.address) is not None
        if use_imu == self.sim.on_stand:        # onto the stand, or off it onto the floor
            self.sim.set_on_stand(not use_imu)
            self.sim.show_imu(self.mount.rotation if use_imu else None)
            self._attitude = None
            self._shown = None
        moved = use_imu and self._take_imu(bot)
        if sample is not None and (sample is not self._shown or moved):
            self._shown = sample
            angles = [ticks_to_degrees(t) for t in sample['position']]
            ready = self._attitude is not None and self._attitude.ready
            self.sim.show_pose(angles, self._attitude.orientation() if use_imu and ready else None)
        if self.calibrating:
            return "Calibrating - leave the robot be"
        if sample is None:
            return "Waiting for the robot..."
        if use_imu:
            return "Joints + IMU"
        if has_imu:
            return "On a stand - calibrate the IMU to see the tilt"
        return "Joints, on a test stand"

    def _take_imu(self, bot):
        """The IMU readings that came in since the last frame, into the tracked
        orientation. True if there were any."""
        imu, address = self._imu, self.mount.address
        if self._attitude is None:
            self._attitude = imu.Attitude(self.mount, heading=self.sim.heading())
            self._imu_t = None
        new = []
        for snapshot in reversed(bot.sensor_history):       # newest first, back to the last one taken
            reading = imu.reading(snapshot, address)
            if reading is None:
                continue
            if self._imu_t is not None and reading[0] <= self._imu_t:
                if not new and reading[0] < self._imu_t - 1.0:
                    self._imu_t = None          # the robot's clock started over: it rebooted
                    new.append(reading)
                break
            new.append(reading)
        for reading in reversed(new):
            self._attitude.update(*reading)
        if new:
            self._imu_t = new[0][0]
        return bool(new)

    def calibrate_imu(self, bot):
        """
        Calibrate IMU, on the routine worker (MainWindow._run_routine): the
        robot stands, then eases its body into four tilts on its own feet,
        holding each still while the IMU is read. Which way up the chassis is
        in each comes from the joint angles (Q8Sim.floor_up), which way up the
        chip is from its accelerometer; the rotation between them is the
        mounting, saved to IMU_MOUNT_FILE.
        """
        imu = self._imu
        self.calibrating = True
        try:
            latest = imu.reading(bot.sensors)
            if latest is None:
                raise ValueError("no MPU6050 is answering")
            address = next(s.address for s in bot.sensors.sensors if s.kind == imu.KIND)
            if not bot.torque:
                raise ValueError("the motors are off - turn them on, with the robot on the floor")
            Q8Logger.info("Calibrating the IMU: the robot tilts four ways - leave it be.")
            bot.stand(500)
            time.sleep(1.0)
            poses, tilt = [], (0, 0)
            for target in imu.CALIBRATION_TILTS:
                for roll, pitch in imu.ease(tilt, target):
                    bot.set_stance(roll=roll, pitch=pitch)
                    time.sleep(imu.EASE_STEP_S)
                tilt = target
                time.sleep(imu.SETTLE_S)
                poses.append(self._hold_still(bot, address))
                for roll, pitch in imu.ease(tilt, (0, 0)):
                    bot.set_stance(roll=roll, pitch=pitch)
                    time.sleep(imu.EASE_STEP_S)
                tilt = (0, 0)
            mount = imu.fit_mount(poses, address)
            mount.save(IMU_MOUNT_FILE)
            self._new_mount = mount
            Q8Logger.info(f"IMU calibrated: {mount.describe()} - readings within {mount.fit:.1f} deg "
                          f"of that; gyro offset {np.degrees(np.linalg.norm(mount.gyro_bias)):.1f} deg/s.")
        except ValueError as exc:
            Q8Logger.error(f"IMU calibration didn't work: {exc}. The model stays as it was.")
        finally:
            self.calibrating = False
            if bot.connected:
                bot.set_stance()

    def _hold_still(self, bot, address):
        """One calibration pose, the robot holding it: the IMU's mean readings
        and the joint angles' up, over GATHER_S. (Worker thread.)"""
        imu = self._imu
        history = bot.sensor_history
        since = history[-1].t_ms / 1000 if history else -1.0
        angles = []
        end = time.monotonic() + imu.GATHER_S
        while time.monotonic() < end:
            sample = bot.telemetry
            if sample is not None:
                angles.append([ticks_to_degrees(t) for t in sample['position']])
            time.sleep(0.03)
        readings = [r for r in (imu.reading(s, address) for s in bot.sensor_history) if r and r[0] > since]
        if len(readings) < 5 or not angles:
            raise ValueError("the robot stopped answering")
        accel = np.array([r[1] for r in readings])
        gyro = np.array([r[2] for r in readings])
        if np.linalg.norm(gyro.std(0)) > 0.05:
            raise ValueError("the robot wasn't still")
        up = self.sim.floor_up(np.mean(angles, 0))
        if up is None:
            raise ValueError("its legs were in a pose they can't close")
        return accel.mean(0), gyro.mean(0), up


class ViewerPanel(QGroupBox):
    """
    The robot in 3D: the simulator's live model if it's installed (see
    LiveModel), else the STL to orbit. Click-drag to rotate, scroll to zoom.
    """

    follow_robot_changed = Signal(bool)
    calibrate_requested = Signal()

    def __init__(self, parent=None):
        super().__init__("", parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = None
        self.live = None
        simulator = _simulator()
        if simulator is not None:
            try:
                self.live = LiveModel(simulator)
            except Exception as exc:
                Q8Logger.error(f"Could not start the simulated model, showing the STL instead: {exc}")
        if self.live is not None:
            self.live.follow_robot_changed.connect(self.follow_robot_changed)
            self.live.calibrate_requested.connect(self.calibrate_requested)
            layout.addWidget(self.live)
            Q8Logger.info("3D view: the simulated model, following the controls.")
            return

        model_path = find_model()

        if model_path is None:
            layout.addWidget(self._placeholder(
                "Interactive 3D Viewer\n\n"
                f"Drop a .stl into {ASSET_DIR} to show the robot here."))
            return

        try:
            widget = self._build_view(model_path)
        except Exception as exc:
            Q8Logger.error(f"Could not show {model_path.name}: {exc}")
            layout.addWidget(self._placeholder(
                f"Could not load {model_path.name}\n\n{exc}"))
            return

        layout.addWidget(widget)

    @property
    def follows_robot(self):
        """True while the live model shows the robot's measured pose."""
        return self.live is not None and self.live.follows_robot

    @property
    def can_simulate(self):
        """The simulated robot is there to connect (simulated_link())."""
        return self.live is not None

    def mirror(self, link):
        """The link to build the Q8Bot on: copied to the live model, if there is one."""
        return self.live.mirror(link) if self.live is not None else link

    def simulated_link(self):
        """The live model's own link, for a Q8Bot with no robot. Needs can_simulate."""
        return self.live.simulated_link()

    def calibrate_imu(self, bot):
        """LiveModel.calibrate_imu() - blocks for seconds, so on a worker."""
        if self.live is not None:
            self.live.calibrate_imu(bot)

    def set_robot(self, bot):
        if self.live is not None:
            self.live.set_robot(bot)

    @staticmethod
    def _placeholder(text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setWordWrap(True)
        return label

    def _build_view(self, model_path):
        # Imported here so a missing/broken OpenGL stack degrades to the
        # message above instead of stopping the whole window from opening.
        import pyqtgraph.opengl as gl

        vertices, faces = load_stl(model_path)

        # Centre on the model's bounding box and frame it, so any export
        # lands in view whatever its origin or units.
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        centre = (low + high) / 2
        size = float(np.linalg.norm(high - low))

        view = gl.GLViewWidget()
        view.setBackgroundColor(BACKGROUND)
        view.setCameraPosition(distance=size * 1.6, elevation=22, azimuth=45)

        mesh = gl.GLMeshItem(
            meshdata=gl.MeshData(vertexes=vertices - centre, faces=faces),
            smooth=False,
            shader=_shaded_from_above(),
            color=MODEL_COLOR,
        )
        view.addItem(mesh)

        grid = gl.GLGridItem(color=GRID_COLOR)
        grid.setSize(size * 1.4, size * 1.4)
        grid.setSpacing(size / 12, size / 12)
        grid.translate(0, 0, -(high[2] - low[2]) / 2)
        view.addItem(grid)

        self.view = view
        Q8Logger.info(f"Loaded {model_path.name} ({len(faces):,} triangles).")
        return view
