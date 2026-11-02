'''
Written by yufeng.wu0902@gmail.com

Builds the desktop app as one download (PyInstaller), into dist/:

    Windows   Q8botOne-<version>-windows-x64.exe       one file, double-click
    macOS     Q8botOne-<version>-macos-arm64.zip       Q8botOne.app inside
    Linux     Q8botOne-<version>-linux-x64.tar.gz      one executable inside

Run it from this repository, in an environment with the app installed and
PyInstaller - and, so the app has its live 3D view and the simulated robot,
q8botone-sim (it looks for a checkout next to this one; SIM=path elsewhere):

    pip install ".[gui]" pyinstaller pillow
    pip install -e ../q8botone-sim
    python appbuild/build.py
    python appbuild/build.py --test      # then run the build's self-test

Without q8botone-sim it builds the app without them (it then shows the
robot as a still model). GitHub builds all three on every version tag (see
.github/workflows/release.yml).

    python appbuild/build.py --flasher

builds the standalone Firmware Flasher instead: a folder with the flasher
and the firmware bundled in q8bot/firmware/ beside it, zipped as
Q8botOne-Firmware-Flasher-<firmware version>-<system>.zip.
'''

import os
import platform
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DIST = ROOT / 'dist'
NAME = 'Q8botOne'


def version():
    sys.path.insert(0, str(ROOT))
    from q8bot import __version__
    return __version__


def sim_checkout():
    '''q8botone-sim's checkout (it carries the model beside the package), or None.'''
    try:
        import q8botone_sim
    except ImportError:
        return None
    path = Path(os.environ.get('SIM', Path(q8botone_sim.__file__).resolve().parent.parent))
    return path if (path / 'model' / 'q8botone.xml').exists() else None


def target():
    arch = {'amd64': 'x64', 'x86_64': 'x64', 'arm64': 'arm64', 'aarch64': 'arm64'}.get(platform.machine().lower(),
                                                                                          platform.machine())
    system = {'win32': 'windows', 'darwin': 'macos'}.get(sys.platform, 'linux')
    return system, arch


def pyinstaller_args(sim):
    sep = os.pathsep                         # --add-data's source/destination separator: ; on Windows, : elsewhere
    args = [
        '--noconfirm', '--clean', '--name', NAME,
        '--windowed',
        '--icon', str(ROOT / 'q8bot' / 'assets' / 'zwr-icon.png'),
        '--collect-data', 'q8bot', '--collect-submodules', 'q8bot',
        '--collect-all', 'esptool',
        '--exclude-module', 'tkinter', '--exclude-module', 'matplotlib', '--exclude-module', 'scipy',
        '--exclude-module', 'PIL',
        '--distpath', str(DIST), '--workpath', str(ROOT / 'build' / 'pyinstaller'), '--specpath', str(ROOT / 'build'),
    ]
    # A macOS app is a folder (Q8botOne.app); elsewhere, one self-contained file
    args += ['--osx-bundle-identifier', 'com.zerowirerobotics.q8botone'] if sys.platform == 'darwin' else ['--onefile']
    if sim is not None:
        # The simulator, MuJoCo with its native library and window/GL helper,
        # and the model - unpacked to q8botone-model/, where sim.py looks
        args += ['--collect-submodules', 'q8botone_sim', '--collect-all', 'mujoco', '--collect-all', 'glfw',
                 '--add-data', f"{sim / 'model' / 'q8botone.xml'}{sep}q8botone-model",
                 '--add-data', f"{sim / 'model' / 'meshes'}{sep}q8botone-model/meshes"]
    return args + [str(ROOT / 'appbuild' / 'launcher.py')]


def package(ver):
    '''The build, renamed and packed as the one file people download.'''
    system, arch = target()
    stem = f'{NAME}-{ver}-{system}-{arch}'
    if system == 'windows':
        out = DIST / f'{stem}.exe'
        shutil.move(DIST / f'{NAME}.exe', out)
    elif system == 'macos':
        out = DIST / f'{stem}.zip'
        # ditto keeps the app's symlinks, permissions and code signature as macOS needs them
        subprocess.run(['ditto', '-c', '-k', '--keepParent', str(DIST / f'{NAME}.app'), str(out)], check=True)
    else:
        out = DIST / f'{stem}.tar.gz'
        with tarfile.open(out, 'w:gz') as tar:
            tar.add(DIST / NAME, arcname=f'{stem}/{NAME}')
    return out


def built_executable():
    system, _ = target()
    if system == 'windows':
        return next(DIST.glob(f'{NAME}-*-windows-*.exe'))
    if system == 'macos':
        return DIST / f'{NAME}.app' / 'Contents' / 'MacOS' / NAME
    return DIST / NAME


def selftest():
    '''The build's own self-test (launcher.selftest), its output shown here.'''
    exe = built_executable()
    command = [str(exe), '--selftest']
    if sys.platform.startswith('linux') and not os.environ.get('DISPLAY') and shutil.which('xvfb-run'):
        command = ['xvfb-run', '-a'] + command          # MuJoCo's view wants a display, even an invisible one
    result = subprocess.run(command, capture_output=True, text=True, timeout=300)
    print(result.stdout + result.stderr)
    return result.returncode


FLASHER = 'Q8botOne-Firmware-Flasher'
FLASH_SET = ('robot.bin', 'controller.bin', 'bootloader.bin', 'partitions.bin', 'boot_app0.bin')


def build_flasher():
    '''The Firmware Flasher: a lean single file - the app's Flash Firmware
    window without the rest of the app - in a folder with the firmware.'''
    import zipfile

    import PyInstaller.__main__
    from q8bot.firmware_flasher import FIRMWARE_DIR, read_bin_identity
    sep = os.pathsep
    icon = ROOT / 'q8bot' / 'assets' / 'zwr-icon.png'
    args = ['--noconfirm', '--clean', '--onefile', '--windowed', '--name', FLASHER, '--icon', str(icon),
            '--add-data', f'{icon}{sep}.', '--collect-all', 'esptool',
            '--hidden-import', 'q8bot.ui.firmware_dialog', '--hidden-import', 'q8bot.ui.panels',
            '--hidden-import', 'q8bot.ui.widgets',
            '--distpath', str(DIST / 'flasher-build'), '--workpath', str(ROOT / 'build' / 'flasher'),
            '--specpath', str(ROOT / 'build')]
    # The flasher never loads the rest of the app (see flasher.py) - leave it out
    for module in ('q8bot.ui.main_window', 'q8bot.ui.viewer', 'q8bot.ui.sensor_window', 'q8bot.ui.teach_dialog',
                   'q8bot.ui.log_bridge', 'pygame', 'pyqtgraph', 'OpenGL', 'numpy', 'mujoco', 'q8botone_sim',
                   'tkinter', 'matplotlib', 'scipy', 'PIL'):
        args += ['--exclude-module', module]
    PyInstaller.__main__.run(args + [str(ROOT / 'appbuild' / 'flasher.py')])

    identity = read_bin_identity(FIRMWARE_DIR / 'robot.bin')
    fw = identity.version if identity else 'unknown'
    system, arch = target()
    folder = DIST / FLASHER
    shutil.rmtree(folder, ignore_errors=True)
    (folder / 'firmware').mkdir(parents=True)
    exe = next((DIST / 'flasher-build').glob(f'{FLASHER}*'))
    shutil.copy2(exe, folder / exe.name)
    for name in FLASH_SET:
        shutil.copy2(FIRMWARE_DIR / name, folder / 'firmware' / name)
    readme = (ROOT / 'appbuild' / 'flasher-README.txt').read_text(encoding='utf-8').format(firmware_version=fw)
    (folder / 'README.txt').write_bytes(readme.replace('\n', '\r\n').encode('utf-8'))      # for Notepad
    out = DIST / f'{FLASHER}-{fw}-{system}-{arch}.zip'
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED) as z:
        for path in sorted(folder.rglob('*')):
            z.write(path, path.relative_to(DIST))
    print(f'{out} ({out.stat().st_size / 1e6:.0f} MB), firmware {fw}')
    return out


def main():
    if '--test' in sys.argv:
        return selftest()
    if '--flasher' in sys.argv:
        build_flasher()
        return 0
    import PyInstaller.__main__
    ver = version()
    sim = sim_checkout()
    print(f'Building {NAME} {ver} for {"-".join(target())}, '
          f'{"with the simulator from " + str(sim) if sim else "WITHOUT the simulator (q8botone-sim not installed)"}')
    PyInstaller.__main__.run(pyinstaller_args(sim))
    out = package(ver)
    print(f'{out} ({out.stat().st_size / 1e6:.0f} MB)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
