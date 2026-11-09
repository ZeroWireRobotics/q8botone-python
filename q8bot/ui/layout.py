'''
Written by yufeng.wu0902@gmail.com

Where the control window's five panels go, and how that follows the
window's shape.

Wide enough for two columns: Connection over the Debug Console beside the
Robot View, and Manual Control beside Robot Information below. Every boundary
is a splitter you can drag; the top and bottom halves share one column split,
so the panels' edges line up across the window. Narrower than that: one
column - Connection, Robot View, Manual Control, Robot Information, the
Debug Console - scrolling. Either way the window scrolls rather than refusing
to get smaller than its panels.

View > Layout can pin either arrangement instead (see MainWindow).
'''

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import QFrame, QScrollArea, QSplitter, QStyle, QVBoxLayout, QWidget

from .widgets import text_scale

AUTOMATIC, COLUMNS, STACK = 'auto', 'columns', 'stack'

# One column only below this much more than two columns need, once in it -
# so a window dragged to right about the boundary doesn't flip back and forth.
HYSTERESIS_PX = 40

# The stack's Robot View and Debug Console: these tall (at 100% text), as
# neither has a natural height of its own.
STACK_VIEWER_HEIGHT = 380
STACK_CONSOLE_HEIGHT = 200


class PanelLayout(QScrollArea):
    def __init__(self, connection, console, viewer, manual, info, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setFrameShape(QFrame.NoFrame)
        self.connection, self.console, self.viewer, self.manual, self.info = (
            connection, console, viewer, manual, info)
        self.mode = AUTOMATIC
        self.arrangement = None
        self.rows = self.top = self.bottom = None   # the splitters, in COLUMNS
        self._sizes = {}       # COLUMNS splitter sizes to put back: 'rows', 'columns'
        self._syncing = False
        self._arrange(COLUMNS)

    @property
    def panels(self):
        return (self.connection, self.console, self.viewer, self.manual, self.info)

    # ---- the arrangement ----------------------------------------------------

    def set_mode(self, mode):
        """AUTOMATIC (follows the window), COLUMNS or STACK."""
        self.mode = mode
        self.fit()

    def fit(self):
        """Into whichever arrangement suits the window now."""
        if self.mode != AUTOMATIC:
            wanted = self.mode
        else:
            need = self.columns_min_width()
            slack = HYSTERESIS_PX if self.arrangement == STACK else 0
            wanted = COLUMNS if self.width() >= need + slack else STACK
        if wanted != self.arrangement:
            self._arrange(wanted)

    def columns_min_width(self):
        """How wide the window's centre has to be for two columns."""
        left = max(p.minimumSizeHint().width() for p in (self.connection, self.console, self.manual))
        right = max(p.minimumSizeHint().width() for p in (self.viewer, self.info))
        style = self.style()
        margins = style.pixelMetric(QStyle.PM_LayoutLeftMargin) + style.pixelMetric(QStyle.PM_LayoutRightMargin)
        return left + right + style.pixelMetric(QStyle.PM_SplitterWidth) + margins

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # After the resize settles: rearranging from inside it fights Qt's own layout pass.
        QTimer.singleShot(0, self._after_resize)

    def _after_resize(self):
        self.fit()
        if self.arrangement != COLUMNS:
            return
        if self._sizes:
            self._sync(self.top, self.bottom, remember=False)   # each half resized on its own
        else:
            self._apply_sizes()   # until a splitter is dragged, the defaults follow the window

    def _arrange(self, kind):
        # Out of the old arrangement first, so deleting it can't take them along.
        for panel in self.panels:
            panel.setParent(None)
        old = self.takeWidget()
        content = QWidget()
        layout = QVBoxLayout(content)
        s = text_scale(self)
        if kind == COLUMNS:
            left = QWidget()
            column = QVBoxLayout(left)
            column.setContentsMargins(0, 0, 0, 0)
            column.addWidget(self.connection)
            column.addWidget(self.console, 1)
            self.top = QSplitter(Qt.Horizontal)
            self.top.addWidget(left)
            self.top.addWidget(self.viewer)
            self.bottom = QSplitter(Qt.Horizontal)
            self.bottom.addWidget(self.manual)
            self.bottom.addWidget(self.info)
            self.rows = QSplitter(Qt.Vertical)
            self.rows.addWidget(self.top)
            self.rows.addWidget(self.bottom)
            self.rows.setStretchFactor(0, 1)    # a taller window: the console and Robot View get it
            self.rows.setStretchFactor(1, 0)
            for half in (self.top, self.bottom):   # a wider one: both columns
                half.setStretchFactor(0, 1)
                half.setStretchFactor(1, 1)
            for splitter in (self.top, self.bottom, self.rows):
                splitter.setChildrenCollapsible(False)
            # One column split for both halves, so the edges line up.
            self.top.splitterMoved.connect(lambda *_: self._sync(self.top, self.bottom))
            self.bottom.splitterMoved.connect(lambda *_: self._sync(self.bottom, self.top))
            self.rows.splitterMoved.connect(lambda *_: self._remember())
            layout.addWidget(self.rows)
            self.viewer.setMinimumHeight(0)
            self.console.setMinimumHeight(0)
        else:
            self.rows = self.top = self.bottom = None
            for panel in (self.connection, self.viewer, self.manual, self.info, self.console):
                layout.addWidget(panel)
            layout.setStretchFactor(self.viewer, 1)
            self.viewer.setMinimumHeight(round(STACK_VIEWER_HEIGHT * s))
            self.console.setMinimumHeight(round(STACK_CONSOLE_HEIGHT * s))
        self.setWidget(content)
        for panel in self.panels:
            panel.show()
        if old is not None:
            old.deleteLater()
        self.arrangement = kind
        QTimer.singleShot(0, self._apply_sizes)

    # ---- the splitters --------------------------------------------------------

    def _apply_sizes(self):
        if self.arrangement != COLUMNS:
            return
        rows, columns = self._sizes.get('rows'), self._sizes.get('columns')
        if not rows:
            # The panels below as tall as they ask to be; the console and Robot View the rest.
            below = max(self.manual.sizeHint().height(), self.info.sizeHint().height())
            rows = [max(1, self.rows.height() - below), below]
        if not columns:
            half = self.top.width() // 2
            columns = [half, self.top.width() - half]
        self.rows.setSizes(rows)
        self.top.setSizes(columns)
        self._sync(self.top, self.bottom, remember=False)

    def _sync(self, source, target, remember=True):
        '''target's column split to source's. remember: it was a drag - keep
        the sizes from now on, rather than the defaults.'''
        if self._syncing:
            return
        self._syncing = True
        target.setSizes(source.sizes())
        # The other half's minimums can win - then both follow it.
        if target.sizes() != source.sizes():
            source.setSizes(target.sizes())
        self._syncing = False
        if remember:
            self._remember()

    def _remember(self):
        if self.arrangement == COLUMNS:
            self._sizes = {'rows': self.rows.sizes(), 'columns': self.top.sizes()}

    def sizes(self):
        """The splitters' sizes, to save - {} until there are any."""
        return dict(self._sizes)

    def set_sizes(self, sizes):
        """Put saved splitter sizes back (see sizes()); {} for the defaults."""
        self._sizes = {k: [int(v) for v in sizes[k]] for k in ('rows', 'columns')
                       if isinstance(sizes.get(k), list) and len(sizes[k]) == 2}
        QTimer.singleShot(0, self._apply_sizes)
