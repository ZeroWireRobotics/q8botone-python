'''
Written by yufeng.wu0902@gmail.com

The Camera Wi-Fi window: which network the camera joins, set over the
camera's own USB cable (see q8bot.camera for why only that way).
'''

import html
import threading
import time

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
)

from ..camera import JOIN_TIMEOUT_S, Camera
from ..protocol import CameraWifi, build_camera_wifi_payload, describe_wifi_failure
from .camera_view import ADDRESS_KEY
from . import theme
from .firmware_dialog import ERROR_COLOR, OK_COLOR

POLL_MS = 2000   # how often the window asks the camera how it is


class CameraDialog(QDialog):
    """
    All serial work runs on worker threads and reports back through the
    signals below. One port lock keeps the regular status check and a save
    from opening the camera's port at the same time.
    """

    _found = Signal(object)              # [(port, version), ...]
    _answered = Signal(object, object)   # (CameraStatus or None, error text or None)
    _progress = Signal(object)           # CameraStatus, while a save waits for the camera to join
    _saved = Signal(object, object)      # (final CameraStatus or None, error text or None)

    def __init__(self, settings=None, parent=None):
        """settings: the app's QSettings - the camera's address goes in it
        once it's on Wi-Fi, for Tools > Camera Video."""
        super().__init__(parent)
        self.setWindowTitle("Camera Wi-Fi")
        self.setModal(True)
        self._settings = settings
        self._port_lock = threading.Lock()
        self._saving = False
        self._closed = False
        self._ssid_edited = False

        self.camera_combo = QComboBox()
        self.camera_combo.setMinimumWidth(200)
        self.camera_combo.currentIndexChanged.connect(self._camera_changed)
        self.search_button = QPushButton("Search Again")
        self.search_button.clicked.connect(self._search)
        camera_row = QHBoxLayout()
        camera_row.addWidget(self.camera_combo, 1)
        camera_row.addWidget(self.search_button)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.status_label.setOpenExternalLinks(True)

        self.ssid_edit = QLineEdit()
        self.ssid_edit.setPlaceholderText("Network name")
        self.ssid_edit.textEdited.connect(self._on_ssid_edited)
        self.password_edit = QLineEdit()
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.password_edit.setPlaceholderText("Empty for an open network")
        self.show_password = QCheckBox("Show")
        self.show_password.toggled.connect(
            lambda shown: self.password_edit.setEchoMode(QLineEdit.Normal if shown else QLineEdit.Password))
        password_row = QHBoxLayout()
        password_row.addWidget(self.password_edit, 1)
        password_row.addWidget(self.show_password)

        form = QFormLayout()
        form.addRow("Camera:", camera_row)
        form.addRow("Status:", self.status_label)
        form.addRow("Network:", self.ssid_edit)
        form.addRow("Password:", password_row)

        note = QLabel("The camera keeps this network through restarts. It's sent over the "
                      "camera's USB cable only - never over the robot's radio.")
        note.setWordWrap(True)
        note.setStyleSheet("color: palette(placeholder-text);")

        self.message_label = QLabel()
        self.message_label.setWordWrap(True)

        self.save_button = QPushButton("Save to Camera")
        self.save_button.setDefault(True)
        self.save_button.clicked.connect(self._save)
        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.reject)
        buttons = QHBoxLayout()
        buttons.addStretch()
        buttons.addWidget(self.save_button)
        buttons.addWidget(self.close_button)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(note)
        layout.addWidget(self.message_label)
        layout.addStretch()
        layout.addLayout(buttons)
        self.resize(480, 300)

        self._found.connect(self._on_found)
        self._answered.connect(self._on_answered)
        self._progress.connect(self._on_progress)
        self._saved.connect(self._on_saved)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(POLL_MS)
        self._poll_timer.timeout.connect(self._poll)

        self._search()

    #-------------------#
    # Finding it        #
    #-------------------#

    def _camera(self):
        port = self.camera_combo.currentData()
        return Camera(port) if port else None

    def _search(self):
        self._poll_timer.stop()
        self.camera_combo.clear()
        self.camera_combo.addItem("Searching...", None)
        self._show_status("Looking for the camera on USB...")
        self._update_buttons()
        self.search_button.setEnabled(False)
        threading.Thread(target=lambda: self._found.emit(Camera.find_all()),
                         name='q8bot-camera-find', daemon=True).start()

    def _on_found(self, cameras):
        if self._closed:
            return
        self.search_button.setEnabled(True)
        self.camera_combo.clear()
        for port, version in cameras:
            self.camera_combo.addItem(f"{port} - camera {version}" if version else port, port)
        if not cameras:
            self.camera_combo.addItem("No camera found", None)
            self._show_status("Plug in the camera's own USB cable, then Search Again.", ERROR_COLOR)
        self._update_buttons()

    def _camera_changed(self):
        if self.camera_combo.currentData():
            self._show_status("Asking the camera...")
            self._poll()
            self._poll_timer.start()

    #-------------------#
    # Its status        #
    #-------------------#

    def _poll(self):
        camera = self._camera()
        if camera is None or self._saving:
            return
        threading.Thread(target=self._poll_worker, args=(camera,), name='q8bot-camera-status',
                         daemon=True).start()

    def _poll_worker(self, camera):
        if not self._port_lock.acquire(blocking=False):
            return   # a save has the port - it reports the status itself
        try:
            self._answered.emit(camera.status(), None)
        except Exception as exc:
            self._answered.emit(None, str(exc) or type(exc).__name__)
        finally:
            self._port_lock.release()

    def _on_answered(self, status, error):
        if self._closed or self._saving:
            return
        if status is None:
            self._show_status(f"No answer: {error}", ERROR_COLOR)
            return
        self._show_camera_status(status)
        if not self._ssid_edited and status.ssid and not self.ssid_edit.text():
            self.ssid_edit.setText(status.ssid)

    def _show_camera_status(self, status):
        ssid = html.escape(status.ssid)
        if status.wifi == CameraWifi.CONNECTED:
            if self._settings is not None:
                self._settings.setValue(ADDRESS_KEY, status.ip)
            url = html.escape(status.url)
            self._show_status(f"Connected to <b>{ssid}</b> ({status.rssi} dBm) - "
                              f"<a href=\"{url}\">{url}</a>", OK_COLOR, rich=True)
        elif status.wifi == CameraWifi.FAILED:
            self._show_status(f"Can't join <b>{ssid}</b> yet: {describe_wifi_failure(status.reason)} "
                              f"It keeps trying.", ERROR_COLOR, rich=True)
        elif status.wifi == CameraWifi.CONNECTING:
            self._show_status(f"Joining <b>{ssid}</b>...", rich=True)
        else:
            self._show_status("No network set yet - enter one below.")

    #-------------------#
    # Saving a network  #
    #-------------------#

    def _on_ssid_edited(self, _text):
        self._ssid_edited = True

    def _save(self):
        camera = self._camera()
        ssid, password = self.ssid_edit.text(), self.password_edit.text()
        try:
            build_camera_wifi_payload(ssid, password)   # check first - nothing is sent if it's wrong
        except ValueError as exc:
            self._message(str(exc), ERROR_COLOR)
            return
        self._saving = True
        self._update_buttons()
        self._message(f"Saving {ssid} on the camera...")
        threading.Thread(target=self._save_worker, args=(camera, ssid, password),
                         name='q8bot-camera-save', daemon=True).start()

    def _save_worker(self, camera, ssid, password):
        with self._port_lock:
            try:
                status = camera.set_wifi(ssid, password)
                self._progress.emit(status)
                deadline = time.monotonic() + JOIN_TIMEOUT_S
                while status.wifi != CameraWifi.CONNECTED and time.monotonic() < deadline and not self._closed:
                    time.sleep(1.0)
                    status = camera.status()
                    self._progress.emit(status)
                self._saved.emit(status, None)
            except Exception as exc:
                self._saved.emit(None, str(exc) or type(exc).__name__)

    def _on_progress(self, status):
        if not self._closed:
            self._show_camera_status(status)

    def _on_saved(self, status, error):
        self._saving = False
        self._update_buttons()
        if self._closed:
            return
        if status is None:
            self._message(f"Not saved: {error}", ERROR_COLOR)
        elif status.wifi == CameraWifi.CONNECTED:
            self.password_edit.clear()
            self._ssid_edited = False
            self._message("Saved - the camera is on the network.", OK_COLOR)
        else:
            reason = describe_wifi_failure(status.reason)
            self._message(f"Saved, but the camera hasn't joined {status.ssid} yet"
                          f"{f' ({reason})' if reason else ''}. Check the name and password - "
                          f"it keeps trying meanwhile.", ERROR_COLOR)
        if status is not None:
            self._show_camera_status(status)

    #-------------------#
    # Helpers           #
    #-------------------#

    def _show_status(self, text, color=None, rich=False):
        self.status_label.setTextFormat(Qt.RichText if rich else Qt.PlainText)
        self.status_label.setText(text)
        theme.set_tone(self.status_label, color)

    def _message(self, text, color=None):
        self.message_label.setText(text)
        theme.set_tone(self.message_label, color)

    def _update_buttons(self):
        has_camera = self.camera_combo.currentData() is not None
        self.save_button.setEnabled(has_camera and not self._saving)
        self.camera_combo.setEnabled(not self._saving)
        self.search_button.setEnabled(not self._saving)
        for edit in (self.ssid_edit, self.password_edit):
            edit.setEnabled(not self._saving)

    def done(self, result):
        self._closed = True
        self._poll_timer.stop()
        super().done(result)
