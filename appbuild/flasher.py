"""
Written by yufeng.wu0902@gmail.com

The standalone Q8botOne Firmware Flasher's entry point (see build.py
--flasher): the desktop app's own Flash Firmware window, on its own. It
checks which board is plugged in (robot or controller) before writing
anything, and refuses a firmware meant for the other one. The firmware
files are NOT inside the program - they're read from the `firmware` folder
next to it, so a newer robot.bin / controller.bin can be dropped in without
rebuilding.

    Q8botOne-Firmware-Flasher.exe              the window
    Q8botOne-Firmware-Flasher.exe --check      print what's in the firmware folder, and exit
    Q8botOne-Firmware-Flasher.exe --selftest   open the window, report what it sees, and exit
"""
import os
import runpy
import sys
import types
from pathlib import Path

# The Flash Firmware window runs `sys.executable -m esptool ...`. Frozen,
# sys.executable is this exe - so those arguments run esptool here and exit.
if sys.argv[1:3] == ['-m', 'esptool']:
    sys.argv = ['esptool'] + sys.argv[3:]
    runpy.run_module('esptool', run_name='__main__', alter_sys=True)
    sys.exit(0)

FROZEN = getattr(sys, 'frozen', False)
HERE = Path(sys.executable).resolve().parent if FROZEN else Path(__file__).resolve().parent

import q8bot

# q8bot.ui's own __init__ loads the whole desktop app (3D viewer, gamepads,
# plots). Only the Flash Firmware window is wanted, so stand in a bare
# package for q8bot.ui - its modules then load without that start-up file.
_ui = types.ModuleType('q8bot.ui')
_ui.__path__ = [os.path.join(sys._MEIPASS, 'q8bot', 'ui') if FROZEN
                else str(Path(q8bot.__file__).parent / 'ui')]
sys.modules['q8bot.ui'] = _ui
q8bot.ui = _ui

from q8bot import firmware_flasher

firmware_flasher.FIRMWARE_DIR = HERE / 'firmware'

if '--check' in sys.argv[1:]:
    print(f'firmware folder: {firmware_flasher.FIRMWARE_DIR}')
    for target in firmware_flasher.TARGETS:
        path = firmware_flasher.bin_path(target)
        identity = firmware_flasher.read_bin_identity(path) if path.exists() else None
        print(f'  {path.name}: ' + (f'{identity.target} {identity.version}' if identity else
                                    'missing' if not path.exists() else 'not Q8botOne firmware'))
    sys.exit(0)

from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication

from q8bot.ui.firmware_dialog import FirmwareDialog

if sys.platform == 'win32':
    # Its own taskbar identity, so the taskbar shows its icon, not python's.
    import ctypes
    ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID('ZeroWireRobotics.Q8botOne.Flasher')

app = QApplication(sys.argv[:1])
icon = (Path(sys._MEIPASS) / 'icon.png') if FROZEN else (Path(q8bot.__file__).parent / 'assets' / 'zwr-icon.png')
if icon.exists():
    app.setWindowIcon(QIcon(str(icon)))
dialog = FirmwareDialog()
dialog.setWindowTitle('Q8botOne Firmware Flasher')
dialog.show()

if '--selftest' in sys.argv[1:]:
    # Build check: the window opens, sees the firmware folder, and lists ports.
    # `--selftest out.png` also saves a picture of it.
    from PySide6.QtCore import QTimer

    def report():
        rest = sys.argv[sys.argv.index('--selftest') + 1:]
        if rest:
            dialog.grab().save(rest[0])
        for i in range(dialog.target_combo.count()):
            dialog.target_combo.setCurrentIndex(i)
            print(f'{dialog.target_combo.currentText()}: {dialog.file_label.text()}')
        print('ports:', [dialog.port_combo.itemText(i) for i in range(dialog.port_combo.count())])
        app.quit()
    QTimer.singleShot(1500, report)

sys.exit(app.exec())
