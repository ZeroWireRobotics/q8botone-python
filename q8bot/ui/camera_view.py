'''
Written by yufeng.wu0902@gmail.com

The Camera window: the camera's live video, as its web page shows it, in a
window of the app's own. The camera streams MJPEG over Wi-Fi - port 81,
/stream - one viewer at a time; its other controls (frame size, a still
photo) are on port 80.

Its address comes from the camera itself (Find Camera): through the robot
when the app is connected to it, or over the camera's own USB cable. The
last one used is remembered.
'''

import http.client
import json
import re
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path
from urllib.parse import urlsplit

from PySide6.QtCore import QObject, QPoint, QRect, QStandardPaths, Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPainter, QPalette
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMenu,
    QPushButton,
    QSizePolicy,
    QSlider,
    QToolButton,
    QVBoxLayout,
    QWidget,
    QWidgetAction,
)

from ..camera import Camera
from ..protocol import CameraWifi
from . import theme
from .firmware_dialog import ERROR_COLOR

ADDRESS_KEY = 'camera/address'   # QSettings: the camera's last address - set here and by Tools > Camera Wi-Fi

STREAM_PORT = 81
STALL_S = 3.0          # no video for this long: reconnect, as the camera's own page does
RETRY_S = 1.0          # between attempts to reach it
CONTROL_TIMEOUT_S = 3.0

# Whether there's a camera (CameraFinder): checked this often, gone after
# this many checks in a row find nothing, and a camera on USB asked at most
# this often - asking opens its port, which Tools > Camera Wi-Fi may want.
CHECK_S = 3.0
LOST_AFTER = 3
USB_EVERY_S = 30.0

# The camera's frame sizes (esp32-camera's framesize_t), smallest first
FRAME_SIZES = (
    (1, "160 × 120"),
    (5, "320 × 240"),
    (8, "640 × 480"),
    (9, "800 × 600"),
    (10, "1024 × 768"),
    (11, "1280 × 720"),
)
PHOTO_SIZE = 13        # UXGA, 1600 × 1200 - what Save Photo takes, as the page's Save High-Res Image
PHOTO_WIDTH = 1600

# Adjust's picture settings: the camera's own names (its /control and
# /status), and the range each takes
QUALITIES = ((10, "High"), (20, "Medium"), (35, "Low"))   # JPEG quality: lower is sharper, and bigger
SLIDERS = (
    ('brightness', "Brightness", -2, 2),
    ('contrast', "Contrast", -2, 2),
    ('saturation', "Saturation", -2, 2),
)
SWITCHES = (
    ('vflip', "Flip upside down"),
    ('hmirror', "Mirror"),
)

_CONTENT_LENGTH = re.compile(rb'Content-Length: *(\d+)', re.IGNORECASE)


def host_of(text):
    """'10.0.0.101' from what someone typed: an address, or the page's URL."""
    text = text.strip()
    if not text:
        return ''
    return urlsplit(text if '//' in text else f'//{text}').hostname or ''


def split_frames(buf, on_frame):
    """
    Hands every whole JPEG in buf to on_frame and returns what's left. The
    stream is multipart: a boundary, headers with a Content-Length, then
    that many bytes of JPEG.
    """
    while True:
        end = buf.find(b'\r\n\r\n')
        if end < 0:
            return buf
        found = _CONTENT_LENGTH.search(buf, 0, end)
        if found is None:
            buf = buf[end + 4:]              # a part without a picture - skip its headers
            continue
        start, length = end + 4, int(found.group(1))
        if len(buf) < start + length:
            return buf
        on_frame(buf[start:start + length])
        buf = buf[start + length:]


class StreamReader(threading.Thread):
    """
    Reads the camera's stream frame by frame, and connects again whenever it
    stalls for STALL_S or ends - until stop(). on_frame gets each JPEG's
    bytes, on_state a line saying what's happening; both on this thread.
    """

    def __init__(self, host, on_frame, on_state):
        super().__init__(name='q8bot-camera-stream', daemon=True)
        self.host = host
        self._on_frame = on_frame
        self._on_state = on_state
        self._stop = threading.Event()
        self._conn = None

    def stop(self):
        self._stop.set()
        conn = self._conn
        if conn is not None and conn.sock is not None:
            try:
                conn.sock.close()            # wakes a read that's waiting
            except OSError:
                pass

    def run(self):
        while not self._stop.is_set():
            frames = [0]

            def counted(jpeg):
                frames[0] += 1
                self._on_frame(jpeg)

            self._on_state(f"Connecting to {self.host}...")
            try:
                self._conn = http.client.HTTPConnection(self.host, STREAM_PORT, timeout=STALL_S)
                self._conn.request('GET', '/stream')
                response = self._conn.getresponse()
                if response.status != 200:
                    raise ConnectionError(f"the camera answered HTTP {response.status}")
                buf = b''
                while not self._stop.is_set():
                    chunk = response.read1(65536)
                    if not chunk:
                        raise ConnectionError("the camera ended the stream")
                    buf = split_frames(buf + chunk, counted)
            except TimeoutError:
                if not self._stop.is_set():
                    self._on_state("The picture stopped - reconnecting..." if frames[0] else
                                   f"No picture from {self.host} - is the camera's page open in a "
                                   f"browser? It streams to one viewer at a time.")
            except (OSError, http.client.HTTPException, ConnectionError) as exc:
                if not self._stop.is_set():
                    self._on_state(f"Can't reach {self.host} ({exc}) - trying again...")
            finally:
                if self._conn is not None:
                    self._conn.close()
                self._conn = None
            self._stop.wait(RETRY_S)


class VideoView(QWidget):
    """The latest frame, as large as fits, keeping its shape."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.image = None
        self.message = ""
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def show_image(self, image):
        self.image = image
        self.update()

    def show_message(self, text):
        self.image = None
        self.message = text
        self.update()

    def paintEvent(self, _event):
        # Around the picture, the panel's own colour - it blends in, whatever
        # the theme, where black bands would frame it.
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.palette().color(QPalette.Window))
        if self.image is None:
            painter.setPen(self.palette().color(QPalette.PlaceholderText))
            painter.drawText(self.rect().adjusted(16, 16, -16, -16), Qt.AlignCenter | Qt.TextWordWrap, self.message)
            return
        size = self.image.size().scaled(self.size(), Qt.KeepAspectRatio)
        corner = QPoint((self.width() - size.width()) // 2, (self.height() - size.height()) // 2)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        painter.drawImage(QRect(corner, size), self.image)


class CameraStream(QObject):
    """
    The camera's video, for every view in the app at once: the camera streams
    to one viewer at a time, so the Robot View panel and the Camera Video
    window take their frames from this one connection. It runs while
    anything is watching (watch / unwatch).
    """

    frame = Signal(QImage)     # the newest frame - ones the window can't keep up with are skipped
    state = Signal(str)        # what's happening, while there's no picture
    settings = Signal(dict)    # camera settings read or changed: {'framesize': 8, 'brightness': 0, ...}
    _ready = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.host = None
        self.last_frame_at = None           # monotonic time of the newest frame, None without one
        self._watchers = set()
        self._reader = None
        self._latest = None                 # handed over from the stream thread
        self._latest_lock = threading.Lock()
        self._pending = False
        self._ready.connect(self._deliver)

    @property
    def live(self):
        """Frames are arriving."""
        return self.last_frame_at is not None and time.monotonic() - self.last_frame_at < STALL_S

    def watch(self, watcher, host):
        """Start watching `host` - switching the stream to it, for every watcher, if it's another."""
        self._watchers.add(watcher)
        if self._reader is None or host != self.host:
            self._stop()
            self.host = host
            self._reader = StreamReader(host, self._on_jpeg, lambda text: _emit(self.state, text))
            self._reader.start()
            self.read_settings()

    def read_settings(self):
        """Ask the camera for its settings - `settings` brings them."""
        if self.host:
            _worker(self._read_settings_worker, self.host)

    def change(self, variable, value):
        """Change one of the camera's settings; `settings` brings it once done."""
        if self.host:
            _worker(self._change_worker, self.host, variable, value)

    def _read_settings_worker(self, host):
        try:
            with urllib.request.urlopen(f'http://{host}/status', timeout=CONTROL_TIMEOUT_S) as reply:
                _emit(self.settings, json.loads(reply.read()))
        except Exception:
            pass                            # the controls just stay as they are

    def _change_worker(self, host, variable, value):
        try:
            _control(host, variable, value)
            _emit(self.settings, {variable: value})
        except Exception:
            self.read_settings()            # whatever it has now

    def unwatch(self, watcher):
        self._watchers.discard(watcher)
        if not self._watchers:
            self._stop()

    def _stop(self):
        if self._reader is not None:
            self._reader.stop()
            self._reader = None
        self.last_frame_at = None

    def _on_jpeg(self, jpeg):
        # On the stream thread: decode here, and hand over only the newest -
        # frames the window can't keep up with are skipped, not queued.
        image = QImage.fromData(jpeg, 'JPG')
        if image.isNull():
            return
        with self._latest_lock:
            self._latest = image
            if self._pending:
                return
            self._pending = True
        _emit(self._ready)

    def _deliver(self):
        with self._latest_lock:
            image, self._latest = self._latest, None
            self._pending = False
        if image is not None and self._reader is not None:
            self.last_frame_at = time.monotonic()
            self.frame.emit(image)


def answers(host, timeout=1.0):
    """Whether the camera at `host` answers on its web port."""
    try:
        with urllib.request.urlopen(f'http://{host}/status', timeout=timeout) as reply:
            return 'framesize' in json.loads(reply.read())
    except Exception:
        return False


def ask_through_robot(bot, tries=3, wait=0.7):
    """
    The camera's CameraStatus, asked through the connected robot - None if
    nothing answered. Asks again when a request goes missing on the radio.
    """
    for _ in range(tries):
        asked = time.monotonic()
        bot.request_camera_status()
        while time.monotonic() - asked < wait:
            age = bot.camera_status_age
            if age is not None and age < time.monotonic() - asked:
                return bot.camera_status
            time.sleep(0.02)
    return None


class CameraControls(QWidget):
    """
    The camera's settings, in a row: its frame size, Adjust (picture
    quality, brightness, contrast, saturation, flip, mirror) and Save Photo.
    They act on the camera `stream` is showing, and every CameraControls on
    one stream shows the same values - a change made in the Robot View
    panel shows in the Camera Video window too.
    """

    message = Signal(str, object)       # (text, tone or None) - for the owner's status line
    _photo_saved = Signal(object, str)  # path or None, what happened

    def __init__(self, stream, compact=False, parent=None):
        """compact: the panel's shorter labels, for a narrow column."""
        super().__init__(parent)
        self._stream = stream

        self.size_combo = QComboBox()
        for value, label in FRAME_SIZES:
            self.size_combo.addItem(label, value)
        self.size_combo.setToolTip("The camera's frame size - bigger is sharper, and slower over Wi-Fi")
        self.size_combo.activated.connect(lambda: self._stream.change('framesize', self.size_combo.currentData()))

        self.adjust_button = QToolButton()
        self.adjust_button.setText("Adjust ▾")
        self.adjust_button.setStyleSheet("QToolButton::menu-indicator { image: none; width: 0px; }")   # the ▾ says it
        self.adjust_button.setToolTip("The camera's picture: quality, brightness, contrast, saturation, flip")
        self.adjust_button.setPopupMode(QToolButton.InstantPopup)
        menu = QMenu(self.adjust_button)
        panel = QWidget()
        form = QFormLayout(panel)
        form.setContentsMargins(12, 10, 12, 10)

        self.quality_combo = QComboBox()
        for value, label in QUALITIES:
            self.quality_combo.addItem(label, value)
        self.quality_combo.setToolTip("Higher is sharper, with bigger frames - fewer of them a second on a slow network")
        self.quality_combo.activated.connect(lambda: self._stream.change('quality', self.quality_combo.currentData()))
        form.addRow("Quality", self.quality_combo)

        self.sliders = {}
        for name, label, low, high in SLIDERS:
            slider = QSlider(Qt.Horizontal)
            slider.setRange(low, high)
            slider.setPageStep(1)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setMinimumWidth(140)
            slider.valueChanged.connect(lambda value, n=name: self._stream.change(n, value))
            form.addRow(label, slider)
            self.sliders[name] = slider

        self.switches = {}
        for name, label in SWITCHES:
            box = QCheckBox(label)
            box.toggled.connect(lambda on, n=name: self._stream.change(n, int(on)))
            form.addRow(box)
            self.switches[name] = box

        action = QWidgetAction(menu)
        action.setDefaultWidget(panel)
        menu.addAction(action)
        menu.aboutToShow.connect(self._stream.read_settings)   # what the camera has now
        self.adjust_button.setMenu(menu)

        self.photo_button = QPushButton("Photo..." if compact else "Save Photo...")
        self.photo_button.setToolTip(f"Save one {PHOTO_WIDTH} × 1200 photo from the camera")
        self.photo_button.clicked.connect(self._save_photo)

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        if not compact:
            row.addWidget(QLabel("Size:"))
        row.addWidget(self.size_combo)
        row.addWidget(self.adjust_button)
        row.addWidget(self.photo_button)

        stream.settings.connect(self._show_settings)
        self._photo_saved.connect(self._on_photo_saved)

    def _show_settings(self, values):
        """The camera's settings onto the controls - quietly, so they don't send them back."""
        widgets = [self.size_combo, self.quality_combo, *self.sliders.values(), *self.switches.values()]
        for widget in widgets:
            widget.blockSignals(True)
        try:
            if 'framesize' in values:
                _select(self.size_combo, int(values['framesize']))
            if 'quality' in values:
                quality = int(values['quality'])   # the nearest of ours, for any other the camera has
                nearest = min(QUALITIES, key=lambda q: abs(q[0] - quality))[0]
                _select(self.quality_combo, nearest)
            for name, slider in self.sliders.items():
                if name in values:
                    slider.setValue(int(values[name]))
            for name, box in self.switches.items():
                if name in values:
                    box.setChecked(bool(int(values[name])))
        finally:
            for widget in widgets:
                widget.blockSignals(False)

    def _save_photo(self):
        host = self._stream.host
        if not host:
            return
        folder = QStandardPaths.writableLocation(QStandardPaths.PicturesLocation)
        default = str(Path(folder) / time.strftime('q8bot-camera-%Y%m%d-%H%M%S.jpg'))
        path, _ = QFileDialog.getSaveFileName(self.window(), "Save Photo", default, "JPEG images (*.jpg)")
        if not path:
            return
        self.photo_button.setEnabled(False)
        self.message.emit("Taking a photo...", None)
        _worker(self._photo_worker, host, self.size_combo.currentData(), path)

    def _photo_worker(self, host, size_before, path):
        # As the camera's page does it: the largest size for one picture,
        # then back - the stream shows the switch for a moment.
        try:
            _control(host, 'framesize', PHOTO_SIZE)
            time.sleep(0.5)
            for _ in range(4):              # the first may still be the old size
                with urllib.request.urlopen(f'http://{host}/capture', timeout=CONTROL_TIMEOUT_S * 2) as reply:
                    jpeg = reply.read()
                image = QImage.fromData(jpeg, 'JPG')
                if image.width() >= PHOTO_WIDTH:
                    break
                time.sleep(0.3)
            Path(path).write_bytes(jpeg)
            _emit(self._photo_saved, path, f"Saved a {image.width()} × {image.height()} photo: {path}")
        except Exception as exc:
            _emit(self._photo_saved, None, f"No photo: {exc}")
        finally:
            try:
                _control(host, 'framesize', size_before)
            except Exception:
                pass

    def _on_photo_saved(self, path, message):
        self.photo_button.setEnabled(True)
        self.message.emit(message, None if path else 'error')


class CameraWindow(QWidget):
    """
    Everything that waits on the network - the stream, finding the camera,
    its settings, a photo - runs on worker threads and reports back through
    signals.
    """

    _found = Signal(object, str)        # host or None, what happened

    def __init__(self, settings, robot=lambda: None, stream=None, parent=None):
        """
        settings: the app's QSettings - the camera's last address lives there.
        robot: () -> the connected Q8Bot, or None - Find Camera asks through it.
        stream: the app's CameraStream, shared with the Robot View panel -
            a window of its own makes one.
        """
        super().__init__(parent, Qt.Window)
        self.setWindowTitle("Camera")
        self.setAttribute(Qt.WA_DeleteOnClose)
        self._settings = settings
        self._robot = robot
        self._stream = stream if stream is not None else CameraStream(self)
        self._watching = False
        self._frame_times = deque(maxlen=30)
        self._live = False
        self._last_status_at = 0.0

        self.address_edit = QLineEdit()
        self.address_edit.setPlaceholderText("The camera's address, e.g. 10.0.0.101")
        self.address_edit.returnPressed.connect(self._start)
        self.find_button = QPushButton("Find Camera")
        self.find_button.setToolTip("Ask the camera for its address - through the robot if the app is "
                                    "connected to it, or over the camera's own USB cable")
        self.find_button.clicked.connect(self._find)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self._toggle)
        top = QHBoxLayout()
        top.addWidget(QLabel("Address:"))
        top.addWidget(self.address_edit, 1)
        top.addWidget(self.find_button)
        top.addWidget(self.connect_button)

        self.video = VideoView()

        self.controls = CameraControls(self._stream)
        self.controls.message.connect(self._status)
        self.size_combo, self.photo_button = self.controls.size_combo, self.controls.photo_button
        self.status_label = QLabel()
        self.status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        bottom = QHBoxLayout()
        bottom.addWidget(self.status_label, 1)
        bottom.addWidget(self.controls)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.video, 1)
        layout.addLayout(bottom)
        self.resize(760, 640)

        self._stream.frame.connect(self._show_frame)
        self._stream.state.connect(self._show_state)
        self._found.connect(self._on_found)

        saved = self._settings.value(ADDRESS_KEY, '') or ''
        self.address_edit.setText(saved)
        self._update_buttons()
        if saved:
            self._start()
        else:
            self._find()

    #-------------------#
    # The stream        #
    #-------------------#

    def _toggle(self):
        if not self._watching:
            self._start()
        else:
            self._stop()
            self._status("Stopped.")
            self.video.show_message("Stopped - Connect to watch again.")

    def _start(self):
        host = host_of(self.address_edit.text())
        if not host:
            self._status("Enter the camera's address, or Find Camera.", ERROR_COLOR)
            return
        self.address_edit.setText(host)
        self._settings.setValue(ADDRESS_KEY, host)
        self._live = False
        self._frame_times.clear()
        if not self._stream.live or self._stream.host != host:
            self.video.show_message(f"Connecting to {host}...")
        self._watching = True
        self._stream.watch(self, host)
        self._stream.read_settings()        # the panel may have the stream already
        self._update_buttons()

    def _stop(self):
        if self._watching:
            self._stream.unwatch(self)
            self._watching = False
        self._live = False
        self._update_buttons()

    def _show_frame(self, image):
        if not self._watching:
            return
        self.video.show_image(image)
        now = time.monotonic()
        self._frame_times.append(now)
        self._live = True
        if now - self._last_status_at >= 0.5:
            self._last_status_at = now
            rate = ""
            if len(self._frame_times) > 1:
                rate = f", {(len(self._frame_times) - 1) / (now - self._frame_times[0]):.0f} fps"
            self._status(f"Live from {self._stream.host} - {image.width()} × {image.height()}{rate}")

    def _show_state(self, text):
        if not self._watching:
            return
        self._live = False
        self._status(text)
        if self.video.image is None:
            self.video.show_message(text)

    #-------------------#
    # Finding it        #
    #-------------------#

    def _find(self):
        self.find_button.setEnabled(False)
        self._status("Asking the camera for its address...")
        _worker(self._find_worker, self._robot())

    def _find_worker(self, bot):
        status, where = None, None
        if bot is not None:
            status, where = ask_through_robot(bot), "through the robot"
        if status is None:
            try:
                cameras = Camera.find_all()
                if cameras:
                    status, where = Camera(cameras[0][0]).status(), f"on {cameras[0][0]}"
            except Exception:
                pass
        if status is None:
            _emit(self._found, None, "No camera answered - connect the app to the robot, or plug in the "
                                     "camera's own USB cable. Or type its address.")
        elif status.wifi != CameraWifi.CONNECTED:
            _emit(self._found, None, f"Found the camera {where}, but it isn't on Wi-Fi: "
                                     f"{status.describe()}. Set its network in Tools > Camera Wi-Fi.")
        else:
            _emit(self._found, status.ip, f"Found the camera {where}, on {status.ssid}.")

    def _on_found(self, host, message):
        self.find_button.setEnabled(True)
        if host is None:
            self._status(message, ERROR_COLOR)
            if self.video.image is None:
                self.video.show_message(message)
            return
        self.address_edit.setText(host)
        self._status(message)
        self._start()

    #-------------------#
    # Helpers           #
    #-------------------#

    def _status(self, text, color=None):
        self.status_label.setText(text)
        self.status_label.setToolTip(text)
        theme.set_tone(self.status_label, color)

    def _update_buttons(self):
        self.connect_button.setText("Stop" if self._watching else "Connect")
        self.controls.setEnabled(self._watching)

    def closeEvent(self, event):
        self._stop()
        super().closeEvent(event)


class CameraPage(QWidget):
    """The camera's video in the Robot View panel, from the app's CameraStream."""

    def __init__(self, stream, parent=None):
        super().__init__(parent)
        self._stream = stream
        self._watching = False
        self._last_status_at = 0.0

        self.status = QLabel()
        self.status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.status.setEnabled(False)       # greyed, like the 3D model's status line
        self.controls = CameraControls(stream, compact=True)
        self.controls.message.connect(lambda text, _tone: self.status.setText(text))
        self.video = VideoView()
        self.video.setMinimumSize(160, 120)
        row = QHBoxLayout()
        row.setContentsMargins(8, 6, 8, 0)
        row.addWidget(self.status, 1)
        row.addWidget(self.controls)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(row)
        layout.addWidget(self.video, 1)

        stream.frame.connect(self._show_frame)
        stream.state.connect(self._show_state)

    @property
    def watching(self):
        return self._watching

    def start(self, host):
        if not self._stream.live or self._stream.host != host:
            self.video.show_message(f"Connecting to the camera at {host}...")
        self.status.setText(f"Camera at {host}")
        self._watching = True
        self._stream.watch(self, host)

    def stop(self):
        if self._watching:
            self._stream.unwatch(self)
            self._watching = False

    def _show_frame(self, image):
        if not self._watching:
            return
        self.video.show_image(image)
        now = time.monotonic()
        if now - self._last_status_at >= 0.5:
            self._last_status_at = now
            self.status.setText(f"Camera at {self._stream.host} - {image.width()} × {image.height()}")

    def _show_state(self, text):
        if self._watching:
            self.status.setText(text)
            if self.video.image is None:
                self.video.show_message(text)


class CameraFinder(QObject):
    """
    Whether there's a camera to watch, and where, checked every CHECK_S in
    the background. The camera answering at its last address (the app's
    settings) is enough; failing that, it asks for the camera's address
    through the robot when the app is connected to one, then over USB from
    a camera the Connection panel found there. `changed` carries the
    camera's host, or None once LOST_AFTER checks in a row have found
    nothing - one slow answer on a busy network doesn't count.
    """

    changed = Signal(object)            # host, or None
    _checked = Signal(object)           # from the worker: host or None

    def __init__(self, settings, robot, camera_ports, stream, parent=None):
        """
        robot: () -> the connected Q8Bot, or None.
        camera_ports: () -> the USB ports a camera was found on.
        stream: the app's CameraStream - while it has frames, there's a camera.
        """
        super().__init__(parent)
        self.host = None
        self._settings = settings
        self._robot = robot
        self._camera_ports = camera_ports
        self._stream = stream
        self._checking = False
        self._misses = 0
        self._last_usb = -USB_EVERY_S
        self._checked.connect(self._on_checked)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.check)
        self._timer.start(round(CHECK_S * 1000))
        QTimer.singleShot(0, self.check)

    def check(self):
        if self._checking:
            return
        if self._stream.live:
            self._on_checked(self._stream.host)
            return
        self._checking = True
        ports = []
        if time.monotonic() - self._last_usb >= USB_EVERY_S:
            ports = list(self._camera_ports())
            if ports:
                self._last_usb = time.monotonic()
        _worker(self._check_worker, self._settings.value(ADDRESS_KEY, '') or '', self._robot(), ports)

    def _check_worker(self, saved, bot, usb_ports):
        host = saved if saved and answers(saved) else None
        if host is None and bot is not None:
            try:
                status = ask_through_robot(bot, tries=1)
            except Exception:               # the robot disconnected meanwhile
                status = None
            if status is not None and status.wifi == CameraWifi.CONNECTED and answers(status.ip):
                host = status.ip
        for port in usb_ports if host is None else ():
            try:
                status = Camera(port).status()
            except Exception:
                continue
            if status.wifi == CameraWifi.CONNECTED and answers(status.ip):
                host = status.ip
                break
        _emit(self._checked, host)

    def _on_checked(self, host):
        self._checking = False
        if host:
            self._misses = 0
            if host != (self._settings.value(ADDRESS_KEY, '') or ''):
                self._settings.setValue(ADDRESS_KEY, host)
            if host != self.host:
                self.host = host
                self.changed.emit(host)
        else:
            self._misses += 1
            if self.host is not None and self._misses >= LOST_AFTER:
                self.host = None
                self.changed.emit(None)


def _select(combo, value):
    index = combo.findData(value)
    if index >= 0:
        combo.setCurrentIndex(index)


def _control(host, variable, value):
    """One of the camera's settings, as its page sets them."""
    with urllib.request.urlopen(f'http://{host}/control?var={variable}&val={value}',
                                timeout=CONTROL_TIMEOUT_S) as reply:
        reply.read()


def _worker(fn, *args):
    threading.Thread(target=fn, args=args, name='q8bot-camera-work', daemon=True).start()


def _emit(signal, *args):
    # A worker can finish after its window has closed - there's no one left to tell.
    try:
        signal.emit(*args)
    except RuntimeError:
        pass
