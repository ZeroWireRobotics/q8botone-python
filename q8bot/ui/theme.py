'''
Written by yufeng.wu0902@gmail.com

The app's light and dark looks (View > Theme). Both are Qt's Fusion style
with a palette of our own - a neutral grey and one blue accent, after VS
Code's Modern themes, with ChatGPT's charcoal for dark - so every widget
follows the theme, the hand-drawn ones included (they paint from the
palette). System follows Windows' own light or dark setting.

Status colours come in two shades each: an error red that reads on white is
too dark to read on charcoal. Text that wears one says so with set_tone(),
and follows the theme from then on:

    theme.apply('dark')                 # the whole app, at once
    theme.set_tone(label, 'error')      # red text, in the theme's red
    theme.css('ok')                     # '#3fb950' - for a style sheet of its own
'''

from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

SYSTEM, LIGHT, DARK = 'system', 'light', 'dark'
CHOICES = (SYSTEM, LIGHT, DARK)

THEMES = {
    LIGHT: {
        'window': '#f3f3f3',        # panels, and the window behind them
        'base': '#ffffff',          # fields, lists, the console
        'alt_base': '#f7f7f7',
        'text': '#1f1f1f',
        'muted': '#6e6e6e',         # placeholders, disabled text, quiet status lines
        'button': '#fbfbfb',
        'border': '#d0d0d0',
        'accent': '#005fb8',
        'accent_text': '#ffffff',
        'link': '#005fb8',
        'tooltip': '#ffffff',
        'tooltip_text': '#1f1f1f',
        'ok': '#1a7f37',
        'warning': '#9a6700',
        'error': '#b3262b',
        'hold': '#8250df',
    },
    DARK: {
        'window': '#1f1f1f',
        'base': '#181818',
        'alt_base': '#222222',
        'text': '#e3e3e3',
        'muted': '#9d9d9d',
        'button': '#2b2b2b',
        'border': '#3c3c3c',
        'accent': '#0078d4',
        'accent_text': '#ffffff',
        'link': '#4daafc',
        'tooltip': '#252526',
        'tooltip_text': '#e3e3e3',
        'ok': '#3fb950',
        'warning': '#d29922',
        'error': '#f85149',
        'hold': '#a371f7',
    },
}

# The status colours set_tone() offers
TONES = ('ok', 'warning', 'error', 'hold', 'muted')

# Dark check and radio boxes' border is this, darkened (see apply)
BOX_OUTLINE_FROM = '#8a8a8a'


class _Notifier(QObject):
    changed = Signal(str)               # the theme now showing: LIGHT or DARK


notifier = None                         # made with the app (a QObject needs one)
_choice = SYSTEM
_shown = LIGHT


def choice():
    """What was picked: SYSTEM, LIGHT or DARK."""
    return _choice


def shown():
    """The theme showing now: LIGHT or DARK (SYSTEM resolved)."""
    return _shown


def color(role):
    """A role of the theme showing, as a QColor - THEMES' keys."""
    return QColor(THEMES[_shown][role])


def css(role_or_color):
    """A role of the theme showing as '#rrggbb' - anything else (a colour) as it is."""
    return THEMES[_shown].get(role_or_color, role_or_color)


def set_tone(widget, tone):
    """Text in a status colour (TONES), or back to plain with None."""
    if widget.property('tone') == tone:
        return
    widget.setProperty('tone', tone)
    widget.style().unpolish(widget)     # a changed property only shows once restyled
    widget.style().polish(widget)


def apply(which=SYSTEM):
    """Show `which` - SYSTEM, LIGHT or DARK - across the whole app."""
    global notifier, _choice, _shown
    app = QApplication.instance()
    if notifier is None:
        notifier = _Notifier(app)
        app.styleHints().colorSchemeChanged.connect(_system_changed)
    _choice = which if which in CHOICES else SYSTEM
    _shown = _resolve(_choice)
    if not app.property('q8bot_fusion'):   # once - with a style sheet on, the style reports no name
        app.setStyle('Fusion')
        app.setProperty('q8bot_fusion', True)
    app.setPalette(palette(_shown))
    # Fusion draws a check box's border a shade darker than its Window
    # colour - lost on charcoal. In dark, check and radio boxes take a
    # mid-grey Window to draw it from (they don't fill with it).
    boxes = palette(_shown)
    if _shown == DARK:
        boxes.setColor(QPalette.Window, QColor(BOX_OUTLINE_FROM))
    for kind in ('QCheckBox', 'QRadioButton'):
        app.setPalette(boxes, kind)
    app.setStyleSheet(_tone_sheet(_shown))
    notifier.changed.emit(_shown)


def palette(name):
    """The QPalette of LIGHT or DARK."""
    c = {role: QColor(value) for role, value in THEMES[name].items()}
    p = QPalette()
    for group in (QPalette.Active, QPalette.Inactive, QPalette.Disabled):
        p.setColor(group, QPalette.Window, c['window'])
        p.setColor(group, QPalette.WindowText, c['text'])
        p.setColor(group, QPalette.Base, c['base'])
        p.setColor(group, QPalette.AlternateBase, c['alt_base'])
        p.setColor(group, QPalette.Text, c['text'])
        p.setColor(group, QPalette.Button, c['button'])
        p.setColor(group, QPalette.ButtonText, c['text'])
        p.setColor(group, QPalette.BrightText, c['error'])
        p.setColor(group, QPalette.Highlight, c['accent'])
        p.setColor(group, QPalette.HighlightedText, c['accent_text'])
        p.setColor(group, QPalette.Accent, c['accent'])
        p.setColor(group, QPalette.Link, c['link'])
        p.setColor(group, QPalette.LinkVisited, c['link'])
        p.setColor(group, QPalette.ToolTipBase, c['tooltip'])
        p.setColor(group, QPalette.ToolTipText, c['tooltip_text'])
        p.setColor(group, QPalette.PlaceholderText, c['muted'])
        # Fusion draws frames and bevels from these
        p.setColor(group, QPalette.Light, c['button'].lighter(110 if name == LIGHT else 130))
        p.setColor(group, QPalette.Midlight, c['button'])
        p.setColor(group, QPalette.Mid, c['border'])
        p.setColor(group, QPalette.Dark, c['border'].darker(120))
        p.setColor(group, QPalette.Shadow, QColor(0, 0, 0, 60 if name == LIGHT else 140))
    for role in (QPalette.WindowText, QPalette.Text, QPalette.ButtonText):
        p.setColor(QPalette.Disabled, role, c['muted'])
    p.setColor(QPalette.Disabled, QPalette.Highlight, c['border'])
    p.setColor(QPalette.Disabled, QPalette.Base, c['window'])
    return p


def _resolve(which):
    if which != SYSTEM:
        return which
    scheme = QApplication.instance().styleHints().colorScheme()
    return DARK if scheme == Qt.ColorScheme.Dark else LIGHT


def _system_changed(_scheme):
    if _choice == SYSTEM:               # Windows switched between light and dark
        apply(SYSTEM)


def _tone_sheet(name):
    colors = THEMES[name]
    return '\n'.join(f'*[tone="{tone}"] {{ color: {colors[tone]}; }}' for tone in TONES)
