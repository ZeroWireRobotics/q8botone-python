"""
Written by yufeng.wu0902@gmail.com

The packaged desktop app's entry point (see build.py). What differs from
running from source:

  - `-m esptool ...`: the Flash Firmware window runs `sys.executable -m
    esptool`. Packaged, sys.executable is the app itself, so that would open
    a second window - instead those arguments run esptool here, and exit.
  - Saved routines and the IMU's calibration default to files inside the
    package, which packaged is a temporary folder wiped on exit (or read
    only) - so they go to the app's folder in the user's own data instead:
    %APPDATA%\\Q8botOne, ~/Library/Application Support/Q8botOne, or
    ~/.local/share/Q8botOne.
  - `--selftest`: checks the build without a window or a robot - what the
    release workflow runs on each one (see selftest()).
"""

import os
import runpy
import sys
import time
from pathlib import Path

if sys.argv[1:3] == ['-m', 'esptool']:
    sys.argv = ['esptool'] + sys.argv[3:]
    runpy.run_module('esptool', run_name='__main__', alter_sys=True)
    sys.exit(0)


def user_data_dir():
    if sys.platform == 'win32':
        base = Path(os.environ.get('APPDATA', Path.home()))
    elif sys.platform == 'darwin':
        base = Path.home() / 'Library' / 'Application Support'
    else:
        base = Path(os.environ.get('XDG_DATA_HOME', Path.home() / '.local' / 'share'))
    return base / 'Q8botOne'


def use_user_data():
    from q8bot import imu, routine_store
    data = user_data_dir()
    routine_store.DEFAULT_DIR = data / 'routines'
    routine_store.DEFAULT_DIR.mkdir(parents=True, exist_ok=True)
    imu.MOUNT_FILE = data / 'imu_mount.json'


def selftest():
    """
    The build, checked without a robot: what it bundles, then the app itself
    started headless, the simulated robot connected through the Connection
    panel's own path and walked. Prints a line per check; exits 1 if any
    failed. The 3D view needs OpenGL - where there's none (a build server
    without a GPU), its drawing is reported and skipped rather than failed.
    """
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    results = []

    def check(name, ok, detail=''):
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}", flush=True)

    from q8bot import __version__
    from q8bot.firmware_flasher import FIRMWARE_DIR, bin_path, read_bin_identity
    print(f'Q8botOne {__version__}, {sys.platform}', flush=True)
    for target in ('robot', 'controller'):
        identity = read_bin_identity(bin_path(target))
        check(f'firmware: {target}.bin bundled', identity is not None and identity.target == target,
              identity.version if identity else 'missing')
    check('firmware: blank-board images bundled',
          all((FIRMWARE_DIR / n).exists() for n in ('bootloader.bin', 'partitions.bin', 'boot_app0.bin')))
    import esptool
    check('esptool bundled', bool(esptool.__version__), esptool.__version__)

    from q8bot.ui import viewer
    simulator = viewer._simulator()
    check('simulator bundled', simulator is not None)
    if simulator is not None:
        from q8botone_sim import sim
        import mujoco
        check('simulator: the robot model bundled', sim.MODEL.exists(), f'MuJoCo {mujoco.__version__}')

    import logging
    from PySide6.QtWidgets import QApplication
    from q8bot.helpers import Q8Logger
    from q8bot.ui import main_window as mw
    from q8bot.ui.log_bridge import QueuedLogHandler

    class Errors(logging.Handler):          # (the window drains its own log queue as it goes)
        def __init__(self):
            super().__init__(logging.ERROR)
            self.seen = []

        def emit(self, record):
            self.seen.append(record.getMessage())

    Q8Logger()
    log, errors = QueuedLogHandler(), Errors()
    Q8Logger.add_handler(log)
    Q8Logger.add_handler(errors)
    app = QApplication(sys.argv[:1])
    # Settings of its own, not the user's: a check mustn't move their window
    # or change their text size.
    import tempfile
    from PySide6.QtCore import QSettings
    scratch = QSettings(str(Path(tempfile.mkdtemp()) / 'selftest.ini'), QSettings.IniFormat)
    win = mw.MainWindow(log, settings=scratch)
    win.show()

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.005)

    pump(0.5)
    live = win.viewer.live
    no_gl = [e for e in errors.seen if 'simulated model' in e or e.startswith(viewer.NO_DRAWING)]
    if live is None and no_gl:
        print(f"SKIP  3D view: no OpenGL here - {no_gl[0]}", flush=True)
    else:
        check('app: the 3D view is the live model', live is not None)
    if simulator is not None and live is not None:
        win._connect_robot(viewer.SIMULATED_ROBOT)
        pump(2.0)
        check('simulated robot: connects and answers', win._last_reply_at is not None)
        x0 = live.sim.chassis()[0]
        win.manual.move_pad._set_value(0.0, 1.0)
        pump(2.0)
        win.manual.move_pad.centre()
        walked = live.sim.chassis()[0] - x0
        check('simulated robot: walks', walked > 100, f'{walked:.0f} mm in 2 s')
        if live.view.unavailable is not None:
            print(f"SKIP  3D view: no OpenGL here - {live.view.unavailable}", flush=True)
        else:
            picture = live.view.grab().toImage()
            check('3D view: draws the robot', picture.width() > 0 and len({picture.pixel(x, picture.height() // 2)
                  for x in range(0, picture.width(), 7)}) > 3, f'{picture.width()}x{picture.height()}')
        win._disconnect_robot()
    win.close()
    other = [e for e in errors.seen if e not in no_gl]
    check('no errors logged', not other, ' | '.join(other))
    print('ALL PASS' if all(results) else 'FAILED', flush=True)
    return 0 if all(results) else 1


use_user_data()
if '--selftest' in sys.argv:
    sys.exit(selftest())

from q8bot.ui import main  # noqa: E402

sys.exit(main(sys.argv[1:]))
