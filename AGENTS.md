# AGENTS.md

A brief for AI coding assistants - Claude Code, Codex and others read this
file when they start in this repository. People: see [README.md](README.md).

## What this is

The Python side of Q8botOne, a palm-sized quadruped with eight DYNAMIXEL
XL330 servos and an ESP32-C3. The computer talks over USB serial to the
handheld controller, which relays everything to the robot by radio
(ESP-NOW). The firmware for both boards is in `q8botone-firmware`, usually
cloned next to this repository (`../q8botone-firmware`).

- `q8bot/robot.py` - `Q8Bot`, the public API; every command is documented there
- `q8bot/protocol.py` - the binary wire protocol (framed, CRC-checked)
- `q8bot/espnow.py` - the serial link to the controller
- `q8bot/kinematics_solver.py`, `gait_generator.py`, `gait_manager.py` - leg kinematics and gaits
- `q8bot/export_gait_tables.py` - writes the firmware's `robot/include/gaitTables.h`
- `q8bot/imu.py` - the MPU6050: mount calibration, body tilt and heading, Keep Level
- `q8bot/firmware/` - the firmware `.bin` files the app flashes
- `q8bot/ui/` - the desktop app (PySide6); `viewer.py` is the 3D view
- `q8botone_sim/` - the simulator (MuJoCo): `sim.py` (`Q8Sim`), `firmware.py`
  (the robot's firmware redone against the model), `poser.py`
- `model/` - the MuJoCo model (`q8botone.xml`, meshes) and the tools that build it
- `appbuild/` - the app's release builds (PyInstaller) and release notes
- `example.py` - a guided tour of the API, on a real robot

## Running things

```
pip install -e ".[gui,sim]"          # Python 3.13; editable, or the simulator can't find model/
q8bot-gui                            # the desktop app
python appbuild/launcher.py --selftest   # the app headless, with the simulated robot - no hardware
```

There's no test suite yet. To try a change without a robot, drive the
simulator with the library's own `Q8Bot`:

```python
import time
from q8bot import Q8Bot
from q8botone_sim.sim import Q8Sim
from q8botone_sim.firmware import Firmware, SimLink

sim = Q8Sim()
robot = Firmware(sim)
bot = Q8Bot(SimLink(robot))
bot.move('f')
for _ in range(100):                 # 1 s - the library keeps wall-clock time, so keep pace
    robot.step(10)
    time.sleep(0.01)
print(sim.chassis())                 # where the body got to, mm
```

## Rules

- **Hardware moves.** Anything that connects to a robot (`Q8Bot.connect()`,
  `example.py`, the app's Connect) drives real motors. Don't run it unless
  the user says the robot is ready; use the simulator instead.
- **The protocol has two ends.** `q8bot/protocol.py` mirrors `protocol.h` in
  both `robot/include/` and `controller/include/` of the firmware. Keep
  command IDs, payload layouts and `DIRECTION_ORDER` in step, and bump
  `PROTOCOL_VERSION` (both sides) for any change the other side has to know about.
- **Generated files.** `gaitTables.h` comes from `python -m q8bot.export_gait_tables`;
  `requirements.txt` from `python update_requirements.py` (edit `pyproject.toml`);
  `q8bot/firmware/*.bin` from `python update_firmware_bins.py`, after building
  a tagged firmware release.
- **The simulator copies the firmware.** `q8botone_sim/firmware.py` redoes the
  robot's gait player and servo motion; a change to how the robot behaves
  belongs there too.
- **The controller copies the library.** The firmware's
  `controller/src/independentControl.cpp` (driving without a computer) is
  ported from `control_config.py`, `GaitManager`'s speed tiers and
  `kinematics_solver.py`; change those and it may need to follow.
- **Commits.** `git config core.hooksPath .githooks` turns on hooks that stamp
  the Python, bundled-firmware and protocol versions into each commit message
  and check the generated files above.
