'''
Written by yufeng.wu0902@gmail.com

The Flash Firmware window: pick a device and a port, and it checks both the
file and the device before writing anything (see q8bot.firmware_flasher for
how, and why both).
'''

import threading
import time

from PySide6.QtCore import Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
)

from .. import firmware_flasher as flasher
from ..helpers import XiaoPortFinder
from .panels import RefreshingCombo

TARGET_LABELS = {'robot': 'Robot', 'controller': 'Controller'}

# A just-flashed board reboots and re-enumerates over USB before it can
# answer again - long enough to cover that, short enough not to feel hung.
VERIFY_TIMEOUT_S = 6.0

OK_COLOR = '#1a7f37'
ERROR_COLOR = '#b3262b'


def _version_text(version):
    # None is firmware from before versioning - it only reports the device.
    return f"v{version}" if version else "unversioned"


class FirmwareDialog(QDialog):
    """
    Modal: nothing else in the window should be driving a serial port while
    this one is flashing.

    The checks and the flash run on a worker thread, reporting back through
    the signals below (queued across threads by Qt), so the window keeps
    painting through a ~10s write.
    """

    _identified = Signal(object, object)   # FirmwareIdentity or None, error or None
    _line = Signal(str)
    _progress = Signal(float)
    _flashed = Signal(bool)
    _verified = Signal(object)             # FirmwareIdentity or None

    def __init__(self, busy_port=None, release_port=None, parent=None):
        """
        busy_port: the port the main window currently holds open (the
            controller it's connected through), if any - flashing it means
            letting go of it first.
        release_port: () -> None, called to do that.
        """
        super().__init__(parent)
        self.setWindowTitle("Flash Firmware")
        self.setModal(True)
        self._busy_port = busy_port
        self._release_port = release_port
        self._busy = False
        self._port = None
        self._target = None
        self._file_ok = False   # set by _update_file_info(); read by _update_buttons(), which runs first
        self._file_version = None

        self.target_combo = QComboBox()
        for target in flasher.TARGETS:
            self.target_combo.addItem(TARGET_LABELS[target], target)
        self.target_combo.currentIndexChanged.connect(self._update_file_info)

        self.port_combo = RefreshingCombo(self._ports)
        self.port_combo.setMinimumWidth(160)
        self.port_combo.currentIndexChanged.connect(self._update_buttons)

        self.file_label = QLabel()
        self.file_label.setWordWrap(True)

        form = QFormLayout()
        form.addRow("Device:", self.target_combo)
        form.addRow("Port:", self.port_combo)
        form.addRow("Firmware:", self.file_label)

        self.status_label = QLabel("Plug the device in over USB, pick it above, then Flash.")
        self.status_label.setWordWrap(True)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.document().setDefaultFont(QFont("Consolas", 8))
        self.log.setPlaceholderText("esptool output appears here.")

        self.flash_button = QPushButton("Flash")
        self.flash_button.setDefault(True)
        self.flash_button.clicked.connect(self._start)
        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.reject)

        buttons = QHBoxLayout()
        buttons.addStretch()
        buttons.addWidget(self.flash_button)
        buttons.addWidget(self.close_button)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(self.status_label)
        layout.addWidget(self.progress)
        layout.addWidget(self.log, 1)
        layout.addLayout(buttons)
        self.resize(460, 420)

        self._identified.connect(self._on_identified)
        self._line.connect(self.log.appendPlainText)
        self._progress.connect(lambda pct: self.progress.setValue(round(pct)))
        self._flashed.connect(self._on_flashed)
        self._verified.connect(self._on_verified)

        self.port_combo.repopulate()
        self._update_file_info()

    #-------------------#
    # Setup             #
    #-------------------#

    def _ports(self):
        ports = XiaoPortFinder.list_all()
        if not ports:
            return [("No device found", None)]
        return [(f"{p} (connected)" if p == self._busy_port else p, p) for p in ports]

    def _update_file_info(self):
        """Show what's on disk for the chosen device, and whether it's usable."""
        target = self.target_combo.currentData()
        path = flasher.bin_path(target)
        self._file_ok = False
        if not path.exists():
            self._set_label(self.file_label, f"Missing: {path.name} - run update_firmware_bins.py", ERROR_COLOR)
        else:
            identity = flasher.read_bin_identity(path)
            found = identity.target if identity else None
            built = time.strftime('%Y-%m-%d %H:%M', time.localtime(path.stat().st_mtime))
            if found == target:
                self._file_ok = True
                self._file_version = identity.version
                self._set_label(self.file_label,
                                f"{path.name} - {_version_text(identity.version)}, "
                                f"{path.stat().st_size // 1024} KB, built {built}", None)
            elif found is None:
                self._set_label(self.file_label,
                                f"{path.name} has no device marker - rebuild it from current firmware",
                                ERROR_COLOR)
            else:
                self._set_label(self.file_label,
                                f"{path.name} is {TARGET_LABELS[found]} firmware, not "
                                f"{TARGET_LABELS[target]} - refusing to use it", ERROR_COLOR)
        self._update_buttons()

    @staticmethod
    def _set_label(label, text, color):
        label.setText(text)
        label.setStyleSheet(f"color: {color};" if color else "")

    def _update_buttons(self):
        idle = not self._busy
        self.flash_button.setEnabled(idle and self._file_ok
                                     and self.port_combo.currentData() is not None)
        self.target_combo.setEnabled(idle)
        self.port_combo.setEnabled(idle)
        self.close_button.setEnabled(idle)

    def _set_busy(self, busy):
        self._busy = busy
        self._update_buttons()

    def _status(self, text, color=None):
        self._set_label(self.status_label, text, color)

    #-------------------#
    # Check, then flash #
    #-------------------#

    def _start(self):
        self._target = self.target_combo.currentData()
        self._port = self.port_combo.currentData()

        if self._port == self._busy_port:
            if QMessageBox.question(
                    self, "Disconnect First?",
                    f"The app is connected through {self._port}. Flashing it will "
                    f"disconnect - reconnect afterwards from the Connection panel.\n\nContinue?"
            ) != QMessageBox.Yes:
                return
            if self._release_port:
                self._release_port()
            self._busy_port = None

        self.log.clear()
        self.progress.setValue(0)
        self._status(f"Checking what's on {self._port}...")
        self._set_busy(True)
        threading.Thread(target=self._identify_worker, name='q8bot-fw-identify',
                         daemon=True).start()

    def _identify_worker(self):
        try:
            self._identified.emit(flasher.identify_device(self._port), None)
        except Exception as exc:
            self._identified.emit(None, exc)

    def _on_identified(self, identity, error):
        name = TARGET_LABELS[self._target]
        if error is not None:
            self._set_busy(False)
            self._status(f"Couldn't open {self._port}: {error}", ERROR_COLOR)
            return

        # The board is an ESP32-C3 either way (see XiaoPortFinder) - what
        # differs is what its firmware says, or whether it says anything.
        device = identity.target if identity else None
        if device == self._target:
            self._status(f"Confirmed {name} on {_version_text(identity.version)}. "
                         f"Flashing {_version_text(self._file_version)}...")

        elif device in TARGET_LABELS:
            self._refuse(
                "Wrong Device",
                f"Refused: that's the {TARGET_LABELS[device]}, not the {name}.",
                f"The device on {self._port} is the {TARGET_LABELS[device]}, but you "
                f"picked {name} firmware.\n\nNothing was flashed. Pick "
                f"\"{TARGET_LABELS[device]}\" to update it instead.")
            return

        elif device is not None:
            # Current firmware that explicitly identified as something this
            # app doesn't update - not a guess to override, so no "continue
            # anyway" here, unlike the legacy case below.
            self._refuse(
                "Unrecognized Firmware",
                f"Refused: the device identifies as \"{device}\", which is neither "
                f"the robot nor the controller.",
                f"The device on {self._port} identifies itself as \"{device}\" - "
                f"neither the robot nor the controller.\n\nNothing was flashed.")
            return

        else:
            # Legacy firmware from before the identify command (or firmware
            # that isn't running) - the board can still be flashed, the user
            # just has to vouch for which one it is.
            if QMessageBox.warning(
                    self, "Legacy Firmware",
                    f"The device on {self._port} is the right kind of board, but its "
                    f"firmware can't report whether it's the robot or the controller - "
                    f"expected for firmware from before this check existed.\n\n"
                    f"Only continue if you're sure it's the {name}. After this update "
                    f"it'll be identified automatically.",
                    QMessageBox.Yes | QMessageBox.No, QMessageBox.No) != QMessageBox.Yes:
                self._set_busy(False)
                self._status("Cancelled - nothing was flashed.")
                return
            self._status(f"Flashing {name} firmware over legacy firmware (unconfirmed)...")

        threading.Thread(target=self._flash_worker, name='q8bot-fw-flash',
                         daemon=True).start()

    def _refuse(self, title, status, detail):
        self._set_busy(False)
        self._status(f"{status} Nothing was flashed.", ERROR_COLOR)
        QMessageBox.warning(self, title, detail)

    def _flash_worker(self):
        ok = flasher.flash(self._port, flasher.bin_path(self._target),
                           on_line=self._line.emit, on_progress=self._progress.emit)
        self._flashed.emit(ok)

    def _on_flashed(self, ok):
        if not ok:
            self._set_busy(False)
            self._status("Flashing failed - see the log below. The device may still be "
                         "on its old firmware; it's safe to try again.", ERROR_COLOR)
            return
        self.progress.setValue(100)
        self._status("Written. Waiting for the device to restart...")
        threading.Thread(target=self._verify_worker, name='q8bot-fw-verify',
                         daemon=True).start()

    def _verify_worker(self):
        try:
            self._verified.emit(flasher.identify_device(self._port, timeout=VERIFY_TIMEOUT_S))
        except Exception:
            self._verified.emit(None)

    def _on_verified(self, identity):
        self._set_busy(False)
        name = TARGET_LABELS[self._target]
        if identity is not None and identity.target == self._target:
            self._status(f"Done - the {name} restarted on {_version_text(identity.version)}.",
                         OK_COLOR)
        else:
            self._status(f"Flashed, but the {name} didn't report back yet. Unplug and "
                         f"replug it if it doesn't start normally.")

    #-------------------#
    # Lifecycle         #
    #-------------------#

    def reject(self):
        # No closing mid-write - an interrupted flash leaves the board
        # needing another attempt, and the worker would outlive the window.
        if not self._busy:
            super().reject()

    def closeEvent(self, event):
        if self._busy:
            event.ignore()
        else:
            super().closeEvent(event)
