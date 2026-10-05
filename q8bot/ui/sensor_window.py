'''
Written by yufeng.wu0902@gmail.com

The Sensors window: live readings from whatever is plugged into the robot's
Qwiic port - a tab per sensor, holding its values (named and unit-labelled
from protocol.SENSOR_KINDS) above plots of the last PLOT_SECONDS. MainWindow
feeds it from its own poll (show_snapshot, show_history) rather than it
running a timer of its own, like the rest of the UI.

Each kind's SensorKind.plots decides how its values are charted - an IMU's
x/y/z share one plot, an INA237's values each get their own.

The plots draw from Q8Bot.sensor_history, which fills at the slow presence
rate even while this window is closed - so it opens already showing the
last half minute, then fills in at the live rate.
'''

import math

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPalette
from PySide6.QtWidgets import (
    QDialog,
    QFormLayout,
    QLabel,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..protocol import sensor_kind
from .widgets import place_beside

# A reading older than this gets called out rather than shown as if live.
STALE_SECONDS = 2.0

PLOT_SECONDS = 30.0
# One color per line in a plot - enough for x/y/z, and distinguishable for
# the common forms of color blindness.
PLOT_COLORS = ('#1f77b4', '#ff7f0e', '#2ca02c', '#d62728')


def format_value(value):
    """Enough decimals to see change at the value's own scale; NaN as a dash."""
    if math.isnan(value):
        return "—"
    magnitude = abs(value)
    decimals = 3 if magnitude < 10 else 2 if magnitude < 100 else 1 if magnitude < 1000 else 0
    return f"{value:.{decimals}f}"


def plot_groups(kind):
    """kind.plots, or one plot per value when the kind doesn't group them."""
    if kind.plots is not None:
        return kind.plots
    return tuple((f"{name} ({unit})" if unit else name, (index,))
                 for index, (name, unit) in enumerate(kind.values))


class SensorWindow(QDialog):
    """Non-modal: meant to stay open beside the main window while driving."""

    closed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sensors")
        self.resize(420, 260)

        self._tabs = QTabWidget()
        self._empty = QLabel(
            "No sensors attached to the robot.\n"
            "Sensors are found when the robot boots - plug one into the "
            "Qwiic port, then power-cycle the robot.")
        self._empty.setWordWrap(True)
        self._empty.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._empty.hide()
        self._status = QLabel("Waiting for a reading...")
        self._status.setWordWrap(True)

        layout = QVBoxLayout(self)
        layout.addWidget(self._tabs, 1)
        layout.addWidget(self._empty, 1)
        layout.addWidget(self._status)

        self._shown_sensors = None   # the (kind, address, value count) set the tabs were built for
        self._value_labels = {}      # (kind, address) -> one QLabel per value
        self._curves = {}            # (kind, address, value index) -> its plotted line

        # finished fires however the window goes away - its close button,
        # Escape, or close() - where closeEvent misses Escape. Deleted then
        # too: it's a child of the main window, so it would otherwise linger
        # hidden, one per open.
        self.finished.connect(self.closed)
        self.finished.connect(self.deleteLater)

    def position_beside(self, widget):
        place_beside(self, widget)

    def show_snapshot(self, snapshot, age):
        """
        Render the latest protocol.SensorSnapshot, or None if none has
        arrived yet. age is seconds since it arrived.
        """
        if snapshot is None:
            self._status.setText("Waiting for a reading...")
            return

        layout_key = tuple((s.kind, s.address, len(s.values)) for s in snapshot.sensors)
        if layout_key != self._shown_sensors:
            self._rebuild(snapshot)
            self._shown_sensors = layout_key

        for reading in snapshot.sensors:
            kind = sensor_kind(reading.kind, len(reading.values))
            labels = self._value_labels[(reading.kind, reading.address)]
            for label, (_, unit), value in zip(labels, kind.values, reading.values):
                text = format_value(value)
                label.setText(text if math.isnan(value) or not unit else f"{text} {unit}")

        if age is not None and age >= STALE_SECONDS:
            self._status.setText(f"No new reading for {age:.0f} s - is the robot still answering?")
        elif any(math.isnan(v) for reading in snapshot.sensors for v in reading.values):
            self._status.setText("A sensor stopped answering - check its cable. "
                                 "(It was detected at boot, so it stays listed.)")
        else:
            self._status.setText(f"Live - robot clock {snapshot.t_ms / 1000:.1f} s")

    def show_history(self, history):
        """
        Redraw the plots from Q8Bot.sensor_history (oldest first), placed by
        the robot's own sample times relative to the newest one.
        """
        if not history or not self._curves:
            return
        newest = history[-1].t_ms
        series = {key: ([], []) for key in self._curves}
        for snapshot in history:
            seconds = (snapshot.t_ms - newest) / 1000.0
            # Later than the newest means the robot rebooted and its clock
            # restarted - those belong to a run that's over.
            if seconds > 0 or seconds < -PLOT_SECONDS:
                continue
            for reading in snapshot.sensors:
                for index, value in enumerate(reading.values):
                    points = series.get((reading.kind, reading.address, index))
                    if points is not None:
                        points[0].append(seconds)
                        points[1].append(value)
        for key, curve in self._curves.items():
            curve.setData(*series[key])

    #-------------------#
    # Building          #
    #-------------------#

    def _rebuild(self, snapshot):
        """A tab per sensor, rebuilt only when the set of sensors changes."""
        while self._tabs.count():
            page = self._tabs.widget(0)
            self._tabs.removeTab(0)
            page.deleteLater()
        self._value_labels = {}
        self._curves = {}

        self._tabs.setVisible(bool(snapshot.sensors))
        self._empty.setVisible(not snapshot.sensors)
        for reading in snapshot.sensors:
            kind = sensor_kind(reading.kind, len(reading.values))
            self._tabs.addTab(self._sensor_page(reading, kind),
                              f"{kind.name} 0x{reading.address:02X}")

    def _sensor_page(self, reading, kind):
        """One sensor's tab: its values, then its plots on a shared time axis."""
        page = QWidget()
        layout = QVBoxLayout(page)

        form = QFormLayout()
        labels = []
        for name, _ in kind.values:
            value = QLabel()
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            form.addRow(name, value)
            labels.append(value)
        self._value_labels[(reading.kind, reading.address)] = labels
        layout.addLayout(form)

        text = self.palette().color(QPalette.WindowText)
        canvas = pg.GraphicsLayoutWidget()
        canvas.setBackground(self.palette().color(QPalette.Window))
        plots = []
        for title, indices in plot_groups(kind):
            plot = canvas.addPlot(row=len(plots), col=0)
            plot.setTitle(title, size='9pt', color=text)
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setMouseEnabled(x=False, y=False)   # it scrolls itself
            plot.setMenuEnabled(False)
            plot.hideButtons()
            plot.setXRange(-PLOT_SECONDS, 0, padding=0)
            plot.enableAutoRange(axis='y')
            for axis in ('left', 'bottom'):
                plot.getAxis(axis).setPen(text)
                plot.getAxis(axis).setTextPen(text)
            if plots:
                plot.setXLink(plots[0])
            if len(indices) > 1:
                plot.addLegend(offset=(6, 6), labelTextColor=text, labelTextSize='8pt')
            for color, index in zip(PLOT_COLORS * len(indices), indices):
                # connect='finite' leaves a gap where the sensor wasn't answering (NaN).
                curve = plot.plot(pen=pg.mkPen(color, width=1.5), connect='finite',
                                  name=kind.values[index][0] if len(indices) > 1 else None)
                self._curves[(reading.kind, reading.address, index)] = curve
            plots.append(plot)
        for plot in plots[:-1]:
            plot.getAxis('bottom').setStyle(showValues=False)
        if plots:
            plots[-1].setLabel('bottom', 'seconds (robot clock)', color=text)
        layout.addWidget(canvas, 1)
        return page
