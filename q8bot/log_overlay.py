'''
Written by yufeng.wu0902@gmail.com

On-screen log display for the pygame control window.

This is a logging.Handler, not something the logger knows about: it buffers
the most recent records and draws them over a pygame surface. Q8Logger stays
pure logging (see helpers.py), and a different front end - a desktop GUI, a
headless script - attaches its own handler, or none at all.
'''

import logging
import time
from collections import deque

import pygame

LEVEL_COLORS = {
    'WARNING': (255, 255, 0),
    'ERROR': (255, 0, 0),
}
DEFAULT_COLOR = (255, 255, 255)


class PygameLogOverlay(logging.Handler):
    """
    Buffers recent log records and renders them top-left over a pygame
    surface.

    Attach it once with Q8Logger.add_handler(overlay), then call
    render(surface) each frame. INFO and above by default, matching what the
    window used to show.
    """

    def __init__(self, max_messages=5, level=logging.INFO):
        super().__init__(level=level)
        self._messages = deque(maxlen=max_messages)
        self._font = None

    def emit(self, record):
        self._messages.append(
            (time.strftime('%H:%M:%S'), record.levelname, record.getMessage())
        )

    def render(self, surface, x=10, y=10, line_height=28):
        """Draw the buffered messages onto surface. Call once per frame."""
        # Deferred so the overlay can be attached before pygame.init() runs.
        if self._font is None:
            self._font = pygame.font.Font(None, 24)

        # Snapshot first: Q8Bot's tick thread logs too, and a deque that grows
        # past maxlen mid-iteration raises "deque mutated during iteration".
        for i, (timestamp, level, msg) in enumerate(list(self._messages)):
            color = LEVEL_COLORS.get(level, DEFAULT_COLOR)
            text_surface = self._font.render(f"[{timestamp}] {level}: {msg}", True, color)
            surface.blit(text_surface, (x, y + i * line_height))
