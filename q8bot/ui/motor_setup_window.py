'''
Written by yufeng.wu0902@gmail.com

The Motor Setup window (Tools > Motor Setup): the robot's own log over its
USB cable, and the servo commands its serial monitor takes - for setting up
servos, and finding out why one isn't answering, without a serial monitor.

The robot sets its servos up itself (q8botone-firmware's motorSetup.cpp): at
power-up it finds any empty slot and waits for a new servo there, one at a
time. This window shows what it's doing and sends the keys it understands:

    m      scan the servo bus - how reliably each servo answers
    q      in motor setup: start without the missing servos
    i      which firmware it runs
    F, Y   reset every servo to factory settings (asked first)

None of them moves the robot. The port is opened without touching the
board's reset line (helpers.open_serial), so connecting doesn't restart it.
'''

import threading
from concurrent.futures import ThreadPoolExecutor

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..helpers import XiaoPortFinder, open_serial
from .panels import RefreshingCombo
from .widgets import is_font_change

SETUP_DOCS_URL = 'https://docs.zerowirerobotics.com/hardware/assembly/#servo-setup'
RESET_CONFIRM_DELAY_MS = 300    # F, then Y: the robot wants Y within 5 s


class MotorSetupWindow(QWidget):
    _line = Signal(str)
    _lost = Signal(str)
    _identified = Signal(str, str, str)   # port, kind, version

    def __init__(self, busy_port=None, parent=None):
        """busy_port: the port the main window holds open (its controller)."""
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Motor Setup")
        self.setAttribute(Qt.WA_DeleteOnClose)   # each Tools > Motor Setup starts afresh
        self._busy_port = busy_port
        self._serial = None
        self._reader = None
        self._stop = threading.Event()
        self._identity = {}            # port -> (kind, version)
        self._asking = set()
        self._pool = ThreadPoolExecutor(4)

        intro = QLabel(
            "Plug the robot into this computer with a USB cable, pick it, and Connect. Its log "
            "shows here: when it powers up with a servo slot empty, it waits in motor setup "
            "for a new servo in that slot, one at a time. Connecting doesn't move or restart "
            f'it. <a href="{SETUP_DOCS_URL}">How servo setup works</a>')
        intro.setWordWrap(True)
        intro.setOpenExternalLinks(True)

        self.port_combo = RefreshingCombo(self._ports)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle)
        port_row = QHBoxLayout()
        port_row.addWidget(QLabel("Robot:"))
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.connect_button)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(3000)
        self.log.setPlaceholderText("The robot's log appears here once connected.")

        self.scan_button = QPushButton("Scan Servos")
        self.scan_button.setToolTip("20 pings and position reads per servo: which answer, and how "
                                    "reliably - for a slot that keeps coming and going.")
        self.scan_button.clicked.connect(lambda: self._send('m'))
        self.skip_button = QPushButton("Start Without Missing Servos")
        self.skip_button.setToolTip("In motor setup only: stop waiting for the empty slots and "
                                    "start with the servos it has.")
        self.skip_button.clicked.connect(lambda: self._send('q'))
        self.version_button = QPushButton("Firmware Version")
        self.version_button.clicked.connect(lambda: self._send('i'))
        self.reset_button = QPushButton("Reset All Servos...")
        self.reset_button.setToolTip("Every servo back to its factory settings - each then needs "
                                     "setting up again, one at a time.")
        self.reset_button.clicked.connect(self._reset_all)
        self.command_buttons = (self.scan_button, self.skip_button, self.version_button,
                                self.reset_button)
        commands = QHBoxLayout()
        for button in self.command_buttons:
            commands.addWidget(button)
        commands.addStretch()

        self.status = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(intro)
        layout.addLayout(port_row)
        layout.addWidget(self.log, 1)
        layout.addLayout(commands)
        layout.addWidget(self.status)
        self.resize(720, 520)

        self._line.connect(self.log.appendPlainText)
        self._lost.connect(self._on_lost)
        self._identified.connect(self._on_identified)
        self._apply_metrics()
        self._set_connected(False)
        self.port_combo.repopulate()

    # ---- ports ---------------------------------------------------------------

    def _ports(self):
        '''(label, port, port) per XIAO on USB, robots first - each asked
        what it is in the background the first time it's seen.'''
        ports = XiaoPortFinder.list_all()
        for gone in set(self._identity) - set(ports):
            del self._identity[gone]
        for port in ports:
            if port not in self._identity and port not in self._asking and port != self._open_port():
                self._asking.add(port)
                self._pool.submit(self._ask, port)
        order = {'robot': 0, None: 1}
        ports.sort(key=lambda p: order.get(self._identity.get(p, (None,))[0], 2))
        return [(self._label(p), p, p) for p in ports]

    def _ask(self, port):
        kind, version = ('busy', None) if port == self._busy_port else XiaoPortFinder.identify(port)
        try:
            self._identified.emit(port, kind, version or '')
        except RuntimeError:
            pass   # the window closed while it asked

    def _on_identified(self, port, kind, version):
        self._asking.discard(port)
        self._identity[port] = (kind, version or None)
        if self.port_combo.view().isVisible():
            self.port_combo.update_entry(port, self._label(port), port)   # open: relabel in place
        else:
            self.port_combo.repopulate()   # closed: re-sorted, the robot first
        if self._serial is None and kind == 'robot' and self.port_combo.currentData() != port:
            index = self.port_combo.findData(port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)   # the robot, once found

    def _label(self, port):
        if port == self._busy_port:
            return f"{port} - the app's controller"
        kind, version = self._identity.get(port, (None, None))
        if kind is None:
            return f"{port} - asking..."
        return f"{port} - {XiaoPortFinder.describe(kind, version)}"

    def _open_port(self):
        return self._serial.port if self._serial is not None else None

    # ---- the connection --------------------------------------------------------

    def _toggle(self):
        if self._serial is not None:
            self._disconnect()
            return
        port = self.port_combo.currentData()
        if not port:
            self.status.setText("No board on USB - plug the robot in.")
            return
        if port == self._busy_port:
            self.status.setText("That's the controller the app is connected through - pick the robot.")
            return
        try:
            self._serial = open_serial(port, 115200, 0.1)
        except Exception as exc:
            self.status.setText(f"Couldn't open {port}: {exc}")
            return
        self._stop.clear()
        self._reader = threading.Thread(target=self._read, args=(self._serial,), daemon=True)
        self._reader.start()
        self.log.appendPlainText(f"--- connected to {port} ---")
        self._set_connected(True)

    def _read(self, link):
        buffer = b''
        while not self._stop.is_set():
            try:
                buffer += link.read(256)
            except Exception as exc:
                if not self._stop.is_set():
                    self._lost.emit(str(exc))
                return
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                self._line.emit(line.decode(errors='replace').rstrip('\r'))

    def _on_lost(self, reason):
        self.log.appendPlainText(f"--- the connection dropped: {reason} ---")
        self._disconnect()
        self.status.setText("The robot's USB connection dropped - is the cable still in?")

    def _disconnect(self):
        self._stop.set()
        link, self._serial = self._serial, None
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
        self._set_connected(False)

    def _set_connected(self, connected):
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.port_combo.setEnabled(not connected)
        for button in self.command_buttons:
            button.setEnabled(connected)
        self.status.setText(f"Connected to {self._open_port()}." if connected else
                            "Not connected.")

    def _send(self, key):
        if self._serial is None:
            return
        try:
            self._serial.write(key.encode())
        except Exception as exc:
            self._on_lost(str(exc))

    def _reset_all(self):
        answer = QMessageBox.warning(
            self, "Reset All Servos",
            "Every servo goes back to its factory settings - ID 1, 57600 baud.\n\n"
            "The robot then stops - servo power off, its light solid red - until it's switched "
            "fully off, its USB unplugged too. When it's switched on again, motor setup asks "
            "for the servos one at a time, and each has to be plugged in on its own.\n\n"
            "Reset all eight?",
            QMessageBox.Reset | QMessageBox.Cancel, QMessageBox.Cancel)
        if answer != QMessageBox.Reset:
            return
        self._send('F')
        QTimer.singleShot(RESET_CONFIRM_DELAY_MS, lambda: self._send('Y'))

    # ---- the window ------------------------------------------------------------

    def _apply_metrics(self):
        self.log.document().setDefaultFont(QFont("Consolas", round(self.font().pointSizeF())))

    def changeEvent(self, event):
        if is_font_change(event):
            QTimer.singleShot(0, self._apply_metrics)
        super().changeEvent(event)

    def closeEvent(self, event):
        self._disconnect()
        self._pool.shutdown(wait=False)
        super().closeEvent(event)
