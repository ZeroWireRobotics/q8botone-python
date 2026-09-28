'''
Written by yufeng.wu0902@gmail.com

This package's build backend (pyproject.toml's build-backend): plain
setuptools, except that every build - wheel, sdist, or editable install -
first checks that the firmware bundled in q8bot/firmware/ is the latest
firmware release, and fails if it isn't (see check_firmware_bins.py).
Failing rather than warning is deliberate: pip hides a backend's output
unless the build fails.

Set Q8BOT_SKIP_FIRMWARE_CHECK=1 to build anyway.
'''

import os
import sys

from setuptools import build_meta as _setuptools
from setuptools.build_meta import *  # noqa: F401,F403 - every hook not overridden below

import check_firmware_bins


def _check_firmware():
    if os.environ.get('Q8BOT_SKIP_FIRMWARE_CHECK') == '1':
        print('firmware check skipped: Q8BOT_SKIP_FIRMWARE_CHECK=1', file=sys.stderr)
        return
    ok, lines = check_firmware_bins.check()
    print('\n'.join(lines), file=sys.stderr)
    if not ok:
        raise RuntimeError('The bundled firmware is not the latest release (see above). '
                           'Set Q8BOT_SKIP_FIRMWARE_CHECK=1 to build anyway.')


def build_wheel(wheel_directory, config_settings=None, metadata_directory=None):
    _check_firmware()
    return _setuptools.build_wheel(wheel_directory, config_settings, metadata_directory)


def build_sdist(sdist_directory, config_settings=None):
    _check_firmware()
    return _setuptools.build_sdist(sdist_directory, config_settings)


def build_editable(wheel_directory, config_settings=None, metadata_directory=None):
    _check_firmware()
    return _setuptools.build_editable(wheel_directory, config_settings, metadata_directory)
