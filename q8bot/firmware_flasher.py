'''
Written by yufeng.wu0902@gmail.com

Flashing the robot and controller firmware from the desktop app.

Both are the same XIAO ESP32-C3 board with the same USB VID/PID, so nothing
about the port says which one is plugged in - and robot firmware on the
controller (or the reverse) is the mistake worth preventing. Two checks:

- The file: each firmware embeds its FIRMWARE_ID ("Q8BOT_FW|robot" or
  "Q8BOT_FW|controller") as a string literal, so read_bin_target() can read
  a .bin's intended device straight out of it.
- The device: both firmwares answer an 'i' over serial with that same line,
  so identify_device() can ask what's actually plugged in.

What the device says decides what happens (the board being an ESP32-C3 at
all is already established by it showing up in XiaoPortFinder's VID/PID-
filtered list, and esptool's --chip check enforces it again at flash time):

- Answers with the device you picked -> flash.
- Answers with the other device -> refuse.
- Answers with anything else -> refuse. It identified itself explicitly,
  just not as something this app knows how to update.
- Doesn't answer -> legacy firmware from before the identify command (or
  firmware that isn't running). The caller has the user confirm, since
  flashing goes through the ESP32-C3's ROM bootloader and so works whatever
  the current firmware is doing.

A wrong-device flash is recoverable, not a brick - only the application
image at APP_OFFSET is ever written, never the bootloader, so the board can
always be flashed again with the right file.
'''

import re
import subprocess
import sys
import time
from pathlib import Path

import serial

# Where the app looks for firmware - refreshed from the PlatformIO build
# output by python-tools/update_firmware_bins.py.
FIRMWARE_DIR = Path(__file__).resolve().parent / 'firmware'
TARGETS = ('robot', 'controller')

# Must match FIRMWARE_ID in firmware/q8bot_robot/src/main.cpp and
# firmware/q8bot_controller/src/main.cpp.
_MARKER = re.compile(rb'Q8BOT_FW\|([A-Za-z0-9_-]+)')
# The serial reply is matched only once its newline has arrived - reads come
# in chunks, and matching mid-line could catch "Q8BOT_FW|rob" before the
# rest shows up, then refuse it as an unknown device.
_REPLY = re.compile(rb'Q8BOT_FW\|([A-Za-z0-9_-]+)\r?\n')
_IDENTIFY_COMMAND = b'i'

CHIP = 'esp32c3'
APP_OFFSET = '0x10000'   # application image only - bootloader and partition table never change between builds
FLASH_BAUD = 921600      # ignored by the C3's native USB, but esptool wants one

_PERCENT = re.compile(r'(\d+(?:\.\d+)?)\s*%')
_ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


def bin_path(target):
    return FIRMWARE_DIR / f'{target}.bin'


def read_bin_target(path):
    """
    Which device a firmware file is for, read from its embedded marker.

    Returns:
        'robot', 'controller', or None - no marker at all (a build from
        before FIRMWARE_ID existed, or not a q8bot firmware), or markers for
        more than one device, which no genuine build contains.
    """
    found = {m.group(1).decode() for m in _MARKER.finditer(Path(path).read_bytes())}
    if len(found) != 1:
        return None
    target = found.pop()
    return target if target in TARGETS else None


def identify_device(port, timeout=2.0):
    """
    Ask the device on `port` what it is.

    Re-sends the probe every few hundred ms rather than once: the first
    bytes after opening a native-USB port can be lost while it settles, and
    a board that was just flashed is still re-enumerating for a moment
    (which is also why failing to open the port is retried, not fatal,
    until the timeout).

    Returns:
        The name the device identified as - normally 'robot' or
        'controller', but returned as-is even when it's neither, so a
        device that explicitly says it's something else can be told apart
        from one that said nothing. None if nothing answered in time
        (legacy firmware).
    Raises:
        serial.SerialException if the port never opened at all - usually
        another program (or this one) still holds it.
    """
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            with serial.Serial(port, 115200, timeout=0.05) as link:
                link.reset_input_buffer()
                received = b''
                next_probe = 0.0
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_probe:
                        link.write(_IDENTIFY_COMMAND)
                        next_probe = time.monotonic() + 0.4
                    received += link.read(256)
                    match = _REPLY.search(received)
                    if match:
                        return match.group(1).decode()
                return None
        except serial.SerialException as exc:
            last_error = exc
            time.sleep(0.2)
    if last_error is not None:
        raise last_error
    return None


def flash(port, path, on_line=None, on_progress=None):
    """
    Write a firmware image to the application partition of the board on
    `port`, then reset it so the new firmware runs. Blocks until done -
    run it off the UI thread.

    Runs esptool as a subprocess rather than calling it in-process: it
    parses its own argv, keeps module-level state, and can sys.exit() on
    errors, none of which should be able to take the app down with it.

    on_line(str): each line of esptool's output, for a log view.
    on_progress(float): write progress, 0-100.

    Returns:
        bool: True if esptool reported success.
    """
    cmd = [sys.executable, '-m', 'esptool', '--chip', CHIP, '--port', port,
           '--baud', str(FLASH_BAUD), 'write-flash', APP_OFFSET, str(path)]
    creationflags = subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            creationflags=creationflags)

    # Split on \r as well as \n - esptool redraws its progress line in
    # place, so a whole write can otherwise arrive as one "line" at the end.
    pending = b''
    while True:
        chunk = proc.stdout.read1(256) if hasattr(proc.stdout, 'read1') else proc.stdout.read(256)
        if not chunk:
            break
        pending += chunk
        *lines, pending = re.split(rb'[\r\n]', pending)
        for raw in lines:
            _emit(raw, on_line, on_progress)
    _emit(pending, on_line, on_progress)
    return proc.wait() == 0


def _emit(raw, on_line, on_progress):
    text = _ANSI.sub('', raw.decode(errors='replace')).strip()
    if not text:
        return
    if on_line:
        on_line(text)
    match = _PERCENT.search(text)
    if match and on_progress:
        on_progress(min(100.0, float(match.group(1))))
