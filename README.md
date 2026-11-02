# q8botone-sim

A physics simulation of Q8botOne in [MuJoCo](https://mujoco.org): one robot
model, used two ways -

- **on your computer**, from Python: drive it with the same `q8bot` library
  as the real robot, or train a controller against it
- **in a browser**, on [zerowirerobotics.com](https://zerowirerobotics.com)

The robot model is built from a simplified CAD of Q8botOne with each part's
measured mass, centre of mass and moments of inertia, checked, and walks
every gait and plays every routine the library has, through the library
itself. It's a plain MuJoCo model (`model/q8botone.xml` and its meshes), so
it loads as it is into MuJoCo's own viewer, your own visualiser, or a
training setup. Still to come: tuning its servos against the real robot.

## Layout

| Path | |
| --- | --- |
| `model/q8botone.xml` | The MuJoCo model - generated, don't edit it by hand |
| `model/meshes/` | Its display meshes - generated, simplified from the CAD |
| `model/inertials.yaml` | Measured mass properties, from Onshape |
| `model/build.py` | Builds the two above from the CAD exports and `inertials.yaml` |
| `model/check_model.py` | Checks the built model in MuJoCo (placement, leg loops, standing, limp) |
| `model/render.py` | Pictures of it, into `renders/` |
| `q8botone_sim/kinematics.py` | A leg's servo angles <-> foot position, and the chassis moving over planted feet, in the library's conventions |
| `q8botone_sim/sim.py` | The model loaded: servo targets in degrees, feet, the chassis's pose, the test stand, servos on/off |
| `q8botone_sim/firmware.py` | The robot's firmware redone against the model - gait tables, servo motion profiles, jump, greet, its replies - and stand-ins for the radio link, so the library drives the sim unchanged |
| `q8botone_sim/imu.py` | A simulated MPU6050, with a real chip's faults - and, under the same names, the q8bot library's calibration and orientation tracking (`q8bot.imu`) |
| `q8botone_sim/qtview.py` | The model drawn into a Qt widget, for a window of its own - the desktop app's 3D view uses it |
| `q8botone_sim/poser.py` | A window for posing and driving the robot |
| `web/export.py` | Writes the model and gait tables the browser version on zerowirerobotics.com loads |
| `pyproject.toml` | So other environments can install the sim (editable: `pip install -e`) |

## Setting up

The sim runs the [q8bot library](https://github.com/ZeroWireRobotics/q8botone-python),
so check out q8botone-python next to this repository, then:

```
python -m venv venv
venv\Scripts\pip install mujoco numpy pyyaml fast-simplification pillow
$env:Q8BOT_SKIP_FIRMWARE_CHECK = 1
venv\Scripts\pip install -e "..\q8botone-python[gui]"
```

`-e` keeps the sim on the library's working copy, so a change there shows in
the sim at once, and `[gui]` brings the desktop app's own widgets (with Qt),
whose pads the poser uses. The variable skips the library's check that its
bundled firmware is the latest release - the sim doesn't use that firmware.

## Posing and driving it

```
venv\Scripts\python -m q8botone_sim.poser
```

Opens MuJoCo's 3D view and a panel beside it. The panel's open tab drives
the robot:

- **Joints** - each servo's target angle (degrees below forward)
- **Feet** - each foot's target in its leg's plane: x forward and y down from
  the leg's rear servo shaft, in mm (the library's leg frame)
- **Body** - the chassis moving over the planted feet: forward, up, roll and
  pitch, from the pose the tab was opened on
- **Walk** - the desktop app's Move, Stance and Body Tilt pads and its gait
  list, doing what they do there: push the move pad to walk any way, further
  to go faster; the other two shift and tilt the robot while it stands. Hold
  Pose keeps a pad where you let go of it; Trot in Place does what the
  gamepad's left trigger does
- **Moves** - the jump and the greet the robot plays itself; the greet, range
  show and shoot the library plays a pose at a time; and the routines taught
  in the desktop app

Walk and Moves go through the library's own `Q8Bot`, unchanged, to the
robot's firmware redone in `firmware.py`: the gait tables the robot has
flashed, the servos easing into each pose over its move time, the jump's
crouch, push and tuck. The firmware's timing runs in simulated time, the
library's on the wall clock, so these keep to real time: pause stops a gait
where it is, but a move can't be paused, nor started while paused.

Above the tabs: on the floor or on a test stand, servos on or limp, the
contact shapes shown, pause and reset. Below them: what the robot actually
does - what its firmware is playing, the chassis's pose and speed, which feet
are down, how tightly the leg loops are closed. In the 3D view, double-click a
part and Ctrl+right-drag to push it; Tab and Shift+Tab show MuJoCo's own
options (contact points, inertia boxes and more).

## In the desktop app

Install the sim into the desktop app's environment, from q8botone-python:

```
venv\Scripts\pip install -e ..\q8botone-sim
```

The app's 3D view then shows this model, live. What moves it depends on
what's connected:

- **a robot** - what the robot measures: its eight joint angles from
  telemetry (asked for 50 times a second meanwhile), each leg's loop closed,
  on a test stand - joint angles alone don't say how the robot is tilted
  - with an **MPU6050** on the robot's Qwiic port, **Calibrate IMU** (in
    Manual Control; the q8bot library's `Q8Bot.calibrate_imu()`, so it works
    without the sim too) finds which way it's mounted: the robot eases its
    body into four tilts on its own feet and holds each still, about 15 s.
    The joint angles say which way up the chassis is in each, the
    accelerometer which way up the chip is, and the turn between them is the
    mounting - any mounting, nothing assumed. It's saved
    (`q8bot/imu_mount.json`, per robot). From then on the model comes off the stand: the IMU
    (read 50 times a second) tilts and turns it, its lowest point rests on
    the floor, its feet on the floor walk it along - and the board is drawn
    on top with its axes as calibrated (X red, Y green, Z blue; where on the
    robot it sits isn't known). The gyro's offset is measured during
    calibration and kept up to date whenever the robot stands still.
- **the simulated robot** - with no robot, connect **Simulated robot** (the
  last entry in the app's Connection list) instead: the model is then the
  robot, playing what the app sends it the way the firmware does.

**Reset** puts the model back in the middle of the floor, facing forward,
upright - its legs as they are.

The simulated robot answers the app as the robot does - its joint
angles, a battery that reads full, no Qwiic sensors. Servo current and
temperature aren't modelled, so they read n/a.

Without the sim installed, the app shows its STL as before. The app's
released builds ([q8botone-python's Releases](https://github.com/ZeroWireRobotics/q8botone-python/releases))
have the sim built in.

How well the twin follows, in the sim - a second simulated robot as the
"real" one, its MPU6050 mounted at odd angles with a real chip's
measured faults (11% scale error, a 31 deg/s gyro offset) and datasheet
noise, readings arriving with radio-like jitter: mounting found within
about 1 deg; standing tilt within 0.2 deg, holding a body tilt within 1 deg,
trotting within 2-4 deg; heading within a few degrees over a 500 deg spin;
the robot flipped over by BOUND, the model too; it walks 80-90% as far as
the robot.

## Rebuilding the model after a CAD change

Export the sim CAD's parts as STL (millimetres, assembly coordinates) into
`q8botone-hardware/cad/sim`, update `model/inertials.yaml` if any mass
properties changed, then:

```
venv\Scripts\python model\build.py
venv\Scripts\python model\check_model.py
```

`build.py` finds the servo shafts and every link's pivots in the meshes
themselves, so new link lengths or shaft positions need no code changes.

To look at the bare model in MuJoCo's own viewer: `venv\Scripts\python -m mujoco.viewer --mjcf=model\q8botone.xml`.

## Where the model comes from

The simplified sim CAD lives with the rest of the CAD, in
[q8botone-hardware](https://github.com/ZeroWireRobotics/q8botone-hardware)
under `cad/sim`. Its STL exports are copied (lightened for the web) into
`model/meshes/`, and its mass properties - masses measured on the real parts -
are in `model/inertials.yaml`.

The MPU6050 board drawn on the twin is Adafruit's MPU-6050 6-DoF breakout
(product 3886), laid out from its open-source PCB design - designed by Bryan
Siepert for Adafruit Industries, CC BY-SA,
[github.com/adafruit/Adafruit-MPU6050-PCB](https://github.com/adafruit/Adafruit-MPU6050-PCB).
Its outline, holes, chip and sockets are in `model/build.py` (`BOARD` and
the rest), its outline mesh in `model/meshes/mpu6050_board.stl`.

## License

MIT - see [LICENSE](LICENSE). The MPU6050 board's outline
(`model/meshes/mpu6050_board.stl`, and its layout in `model/build.py`) follows
Adafruit's design and stays under its CC BY-SA licence, credited above.
