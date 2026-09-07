'''
Written by yufeng.wu0902@gmail.com

Small custom widgets for the control window.

Qt ships no switch widget, so ToggleSwitch is drawn by hand - a checkbox
reads as "on/off", which is the wrong idea for a control that picks between
two equally-valid named modes.
'''

from PySide6.QtCore import (
    Property,
    QEasingCurve,
    QPropertyAnimation,
    QRectF,
    QSize,
    Qt,
)
from PySide6.QtGui import QColor, QPainter, QPalette
from PySide6.QtWidgets import QAbstractButton, QLabel, QSizePolicy, QWidget


def _blend(start, end, amount):
    """Linear mix of two QColors; amount 0.0 gives start, 1.0 gives end."""
    return QColor(
        round(start.red() + (end.red() - start.red()) * amount),
        round(start.green() + (end.green() - start.green()) * amount),
        round(start.blue() + (end.blue() - start.blue()) * amount),
    )


class ToggleSwitch(QAbstractButton):
    """
    A sliding on/off switch. Checkable like a QCheckBox and emits the same
    toggled(bool), so it drops straight in where one would go.
    """

    TRACK_HEIGHT_RATIO = 0.72
    KNOB_MARGIN = 3

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.TabFocus)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self._position = 0.0  # 0.0 at the left stop, 1.0 at the right
        self._animation = QPropertyAnimation(self, b"position", self)
        self._animation.setDuration(140)
        self._animation.setEasingCurve(QEasingCurve.InOutCubic)
        self.toggled.connect(self._animate_to)

    def sizeHint(self):
        return QSize(52, 26)

    def _get_position(self):
        return self._position

    def _set_position(self, value):
        self._position = value
        self.update()

    position = Property(float, _get_position, _set_position)

    def _animate_to(self, checked):
        self._animation.stop()
        self._animation.setStartValue(self._position)
        self._animation.setEndValue(1.0 if checked else 0.0)
        self._animation.start()

    def setChecked(self, checked):
        super().setChecked(checked)
        # Land on the right stop even when set before the widget is shown,
        # where the animation has nothing to drive.
        if not self.isVisible():
            self._position = 1.0 if checked else 0.0

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        palette = self.palette()
        enabled = self.isEnabled()
        group = QPalette.Normal if enabled else QPalette.Disabled

        # Base, not BrightText, for the knob: on the default Windows palette
        # BrightText is the same blue as Highlight, which paints an invisible
        # knob on a switched-on track.
        knob_color = palette.color(group, QPalette.Base)
        track_color = _blend(palette.color(group, QPalette.Mid),
                             palette.color(group, QPalette.Highlight),
                             self._position)

        width, height = self.width(), self.height()
        track_height = height * self.TRACK_HEIGHT_RATIO
        track_top = (height - track_height) / 2

        painter.setPen(Qt.NoPen)
        painter.setBrush(track_color)
        painter.drawRoundedRect(0, track_top, width, track_height,
                                track_height / 2, track_height / 2)

        knob_diameter = track_height - self.KNOB_MARGIN * 2
        travel = width - knob_diameter - self.KNOB_MARGIN * 2
        knob_x = self.KNOB_MARGIN + travel * self._position
        knob_y = track_top + self.KNOB_MARGIN

        painter.setPen(_blend(knob_color, palette.color(group, QPalette.Shadow), 0.25))
        painter.setBrush(knob_color)
        painter.drawEllipse(QRectF(knob_x, knob_y, knob_diameter, knob_diameter))

        if self.hasFocus():
            painter.setPen(palette.color(QPalette.Highlight))
            painter.setBrush(Qt.NoBrush)
            painter.drawRoundedRect(0, track_top, width - 1, track_height - 1,
                                    track_height / 2, track_height / 2)


class StatusDot(QWidget):
    """
    A small indicator lamp. Pulses while pending, so "waiting on the robot"
    is distinguishable at a glance from "sitting there doing nothing".

    Always pair it with text - colour alone is not a signal everyone can
    read (see ConnectionPanel, which puts a word beside every dot).
    """

    OFF = 'off'
    PENDING = 'pending'
    LIVE = 'live'
    ERROR = 'error'

    COLORS = {
        OFF: QColor('#9098a0'),
        PENDING: QColor('#d9820a'),
        LIVE: QColor('#2ea043'),
        ERROR: QColor('#d13438'),
    }
    DIAMETER = 12

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.DIAMETER + 4, self.DIAMETER + 4)
        self._state = self.OFF
        self._glow = 1.0

        self._pulse = QPropertyAnimation(self, b"glow", self)
        self._pulse.setDuration(900)
        self._pulse.setStartValue(1.0)
        self._pulse.setKeyValueAt(0.5, 0.35)
        self._pulse.setEndValue(1.0)
        self._pulse.setLoopCount(-1)

    def _get_glow(self):
        return self._glow

    def _set_glow(self, value):
        self._glow = value
        self.update()

    glow = Property(float, _get_glow, _set_glow)

    @property
    def state(self):
        return self._state

    def set_state(self, state, tooltip=""):
        self._state = state
        self.setToolTip(tooltip)
        if state == self.PENDING:
            if self._pulse.state() != QPropertyAnimation.Running:
                self._pulse.start()
        else:
            self._pulse.stop()
            self._glow = 1.0
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        color = QColor(self.COLORS[self._state])
        if self._state == self.PENDING:
            color = _blend(self.palette().color(QPalette.Window), color, self._glow)

        centre = (self.width() - self.DIAMETER) / 2
        painter.setPen(_blend(color, QColor('#000000'), 0.35))
        painter.setBrush(color)
        painter.drawEllipse(QRectF(centre, centre, self.DIAMETER, self.DIAMETER))


class ClickableLabel(QLabel):
    """A label that acts as a bigger hit target for the control beside it."""

    def __init__(self, text, on_click, parent=None):
        super().__init__(text, parent)
        self._on_click = on_click
        self.setCursor(Qt.PointingHandCursor)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.isEnabled() and self.rect().contains(event.position().toPoint()):
            self._on_click()
        super().mouseReleaseEvent(event)
