'''
Written by yufeng.wu0902@gmail.com

Small custom widgets for the control window.

Qt ships no switch widget, so ToggleSwitch is drawn by hand - a checkbox
reads as "on/off", which is the wrong idea for a control that picks between
two equally-valid named modes.
'''

import math

from PySide6.QtCore import (
    Property,
    QEasingCurve,
    QPropertyAnimation,
    QPointF,
    QRectF,
    QSize,
    Qt,
    Signal,
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


class JoystickPad(QWidget):
    """
    A two-axis pad you drag with the mouse, shaped like a thumbstick.

    Springs back to centre when released, like the stick it's modelled on -
    so letting go returns the robot to its baseline rather than parking it at
    whatever offset you last dragged to. See set_hold() to turn that off and
    hold a pose instead, without keeping the mouse button down.

    Reports both axes as -1.0..1.0 with up and right positive, clamped to the
    unit circle so a diagonal can't exceed full deflection on both at once.
    """

    moved = Signal(float, float)   # x, y in -1.0..1.0

    SIZE = 132
    KNOB_R = 13
    RETURN_MS = 130

    # Distinct from the OS accent colour (used for a normal, spring-back
    # knob) and from ManualControlPanel's green/red torque colours, so a
    # held pad reads as its own third state at a glance.
    HOLD_COLOR = QColor('#8250df')

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.SIZE, self.SIZE)
        self.setCursor(Qt.PointingHandCursor)
        self._x = 0.0
        self._y = 0.0
        self._dragging = False

        # Animated rather than snapped, so the robot eases back to baseline
        # instead of being handed a step change. Interpolates from wherever
        # the knob was released down to zero.
        self._release_from = (0.0, 0.0)
        self._return = 0.0
        self._horizontal = True
        self._hold = False   # see set_hold()
        self._animation = QPropertyAnimation(self, b"returnFraction", self)
        self._animation.setDuration(self.RETURN_MS)
        self._animation.setEasingCurve(QEasingCurve.OutCubic)

    def _get_return(self):
        return self._return

    def _set_return(self, fraction):
        self._return = fraction
        x0, y0 = self._release_from
        self._set_value(x0 * fraction, y0 * fraction)

    returnFraction = Property(float, _get_return, _set_return)

    def value(self):
        return self._x, self._y

    def centre(self):
        self._set_value(0.0, 0.0)

    def set_hold(self, hold):
        """
        When held, letting go leaves the knob wherever it was dragged
        instead of springing back to centre - for holding a pose (teaching a
        routine, or just parking the robot in a stance) without keeping the
        mouse button down. Turning hold back off immediately springs back to
        centre if the pad isn't at rest already, the same as a normal
        release would.
        """
        if hold == self._hold:
            return
        self._hold = hold
        if not hold and not self._dragging and (self._x, self._y) != (0.0, 0.0):
            self._start_return()
        else:
            self.update()   # _start_return()'s animation repaints on its own

    def set_horizontal_enabled(self, enabled):
        """
        Turn the left/right axis off for a control that has no meaning there -
        a gait that can't turn, say. The pad then draws as a vertical slot
        rather than a circle, so it looks as constrained as it behaves; a
        knob that simply refuses to go sideways reads as broken.
        """
        if enabled == self._horizontal:
            return
        self._horizontal = enabled
        if not enabled:
            self._set_value(0.0, self._y)
        self.update()

    def _set_value(self, x, y):
        if not self._horizontal:
            x = 0.0
        # Clamp into the unit circle rather than the square, so the reachable
        # area matches the ring drawn on screen.
        magnitude = math.hypot(x, y)
        if magnitude > 1.0:
            x, y = x / magnitude, y / magnitude
        if (x, y) == (self._x, self._y):
            return
        self._x, self._y = x, y
        self.update()
        self.moved.emit(x, y)

    def _from_pos(self, pos):
        radius = self.SIZE / 2 - self.KNOB_R
        x = (pos.x() - self.SIZE / 2) / radius
        y = -(pos.y() - self.SIZE / 2) / radius   # screen y grows downward
        return x, y

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.isEnabled():
            self._animation.stop()
            self._dragging = True
            self._set_value(*self._from_pos(event.position()))

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._set_value(*self._from_pos(event.position()))

    def mouseReleaseEvent(self, _event):
        if not self._dragging:
            return
        self._dragging = False
        if self._hold:
            return   # stay right where it was let go
        self._start_return()

    def _start_return(self):
        self._release_from = (self._x, self._y)
        self._animation.stop()
        self._animation.setStartValue(1.0)
        self._animation.setEndValue(0.0)
        self._animation.start()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        palette = self.palette()
        group = QPalette.Normal if self.isEnabled() else QPalette.Disabled
        outline = palette.color(group, QPalette.Mid)
        # Blend the accent toward grey by hand when disabled. The palette's
        # own Disabled Highlight is no help here - on Windows it comes back
        # brighter than the Normal one, so the knob would stay vividly blue
        # on a panel that's meant to look switched off. Coloured HOLD_COLOR
        # for as long as hold is on, not only once dragged away from centre -
        # the whole point is telling the user "this pad won't spring back"
        # before they find out by letting go of it.
        accent = self.HOLD_COLOR if self._hold else palette.color(QPalette.Normal, QPalette.Highlight)
        if not self.isEnabled():
            accent = _blend(accent, palette.color(QPalette.Window), 0.62)

        travel = self.SIZE / 2 - self.KNOB_R
        centre = self.SIZE / 2
        well = QRectF(self.KNOB_R, self.KNOB_R, travel * 2, travel * 2)

        painter.setBrush(palette.color(group, QPalette.Base))
        painter.setPen(outline)
        if self._horizontal:
            painter.drawEllipse(well)
        else:
            # A capsule the width of the knob: the shape itself says "up and
            # down only", before anyone tries to drag it sideways.
            slot = QRectF(centre - self.KNOB_R, self.KNOB_R,
                          self.KNOB_R * 2, travel * 2)
            painter.drawRoundedRect(slot, self.KNOB_R, self.KNOB_R)

        # Crosshair, so zero is findable without dragging to look for it
        hair = _blend(palette.color(group, QPalette.Base), outline, 0.45)
        painter.setPen(hair)
        painter.drawLine(QPointF(centre, self.KNOB_R + 4), QPointF(centre, self.SIZE - self.KNOB_R - 4))
        if self._horizontal:
            painter.drawLine(QPointF(self.KNOB_R + 4, centre),
                             QPointF(self.SIZE - self.KNOB_R - 4, centre))

        knob_x = centre + self._x * travel
        knob_y = centre - self._y * travel

        # A line from centre to knob reads the deflection at a glance
        if (self._x, self._y) != (0.0, 0.0):
            painter.setPen(_blend(accent, outline, 0.35))
            painter.drawLine(QPointF(centre, centre), QPointF(knob_x, knob_y))

        painter.setPen(_blend(accent, QColor('#000000'), 0.35))
        painter.setBrush(accent)
        painter.drawEllipse(QPointF(knob_x, knob_y), self.KNOB_R, self.KNOB_R)


class BatteryGauge(QWidget):
    """
    A battery-shaped level bar. Shows charge at a glance; the exact percent
    and voltage live in labels beside it (see RobotInfoPanel).

    Bands match the colours the status dots use, so "amber means pay
    attention" reads the same way everywhere in the window.
    """

    GOOD, LOW, CRITICAL = 30, 10, 0
    COLORS = {
        'good': QColor('#2ea043'),
        'low': QColor('#d9820a'),
        'critical': QColor('#d13438'),
    }
    BODY_W, BODY_H = 78, 18
    CAP_W, CAP_H = 3, 8
    INSET = 2.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(self.BODY_W + self.CAP_W + 2, self.BODY_H + 2)
        self._percent = None

    def set_percent(self, percent):
        self._percent = None if percent is None else max(0, min(100, int(percent)))
        self.setToolTip("No reading yet" if self._percent is None
                        else f"Battery {self._percent}%")
        self.update()

    def _fill_color(self):
        if self._percent > self.GOOD:
            return self.COLORS['good']
        if self._percent > self.LOW:
            return self.COLORS['low']
        return self.COLORS['critical']

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        palette = self.palette()
        outline = _blend(palette.color(QPalette.Mid), palette.color(QPalette.Shadow), 0.3)

        body = QRectF(0.5, 0.5, self.BODY_W, self.BODY_H)
        painter.setPen(outline)
        painter.setBrush(palette.color(QPalette.Base))
        painter.drawRoundedRect(body, 3, 3)

        # The terminal nub, so it reads as a battery rather than a progress bar
        cap = QRectF(self.BODY_W + 0.5, (self.BODY_H - self.CAP_H) / 2 + 0.5,
                     self.CAP_W, self.CAP_H)
        painter.setBrush(outline)
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(cap, 1, 1)

        if not self._percent:
            return  # nothing to fill: no reading, or a flat pack

        track = body.adjusted(self.INSET, self.INSET, -self.INSET, -self.INSET)
        fill = QRectF(track)
        fill.setWidth(track.width() * self._percent / 100.0)
        painter.setBrush(self._fill_color())
        painter.drawRoundedRect(fill, 1.5, 1.5)


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


def place_beside(window, widget):
    """
    Place `window` just outside `widget`'s frame - to its right if
    there's room, else to its left, else pinned against whichever screen
    edge fits - rather than wherever Qt's default placement happens to
    land, which was opening it right on top of the Manual Control panel
    it's meant to be used alongside (forcing a manual drag every time).

    Also matches `widget`'s height by default, so it reads as a panel
    alongside the main window rather than an oddly small popup next to
    a much taller one - width is left as whatever __init__ set.
    """
    anchor = widget.frameGeometry()
    # resize() sets content size, not frame size - matching it against
    # frameGeometry() (which includes the title bar) would overshoot by
    # that title bar's height. widget.height() is the same content
    # measurement resize() itself uses, so this compares like with like.
    window.resize(window.width(), widget.height())
    x, y = anchor.right() + 12, anchor.top()
    screen = widget.screen()
    if screen is not None:
        available = screen.availableGeometry()
        if x + window.width() > available.right():
            x = anchor.left() - window.width() - 12
        x = max(available.left(), min(x, available.right() - window.width()))
        y = max(available.top(), min(y, available.bottom() - window.height()))
    window.move(x, y)
