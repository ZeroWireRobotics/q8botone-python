'''
Written by yufeng.wu0902@gmail.com

Bridges Q8Logger into the GUI's debug console.

A logging.Handler that buffers records for the UI to collect on its own
timer, rather than touching widgets when it's called. Q8Bot logs from its
background tick thread, and Qt widgets may only be touched from the GUI
thread - draining a queue keeps that boundary in one obvious place.
'''

import logging
import time
from collections import deque


class QueuedLogHandler(logging.Handler):
    """Buffers log records; call drain() from the GUI thread to collect them."""

    def __init__(self, capacity=2000, level=logging.INFO):
        super().__init__(level=level)
        self._records = deque(maxlen=capacity)

    def emit(self, record):
        self._records.append(
            (time.strftime('%H:%M:%S'), record.levelname, record.getMessage())
        )

    def drain(self):
        """Pop everything buffered since the last call. Safe from any thread."""
        out = []
        while True:
            try:
                out.append(self._records.popleft())
            except IndexError:
                return out
