'''
Written by yufeng.wu0902@gmail.com

Regenerates requirements.txt from pyproject.toml. pyproject.toml is the one
place dependencies are declared; requirements.txt mirrors the core
dependencies plus the extras in INCLUDED_EXTRAS (the same set as
`pip install -e ".[gui]"`) for anyone who prefers `pip install -r`. Other
extras are listed commented out. Comments written above each dependency list
in pyproject.toml are carried over.

    python update_requirements.py          # rewrite requirements.txt
    python update_requirements.py --check  # exit 1 if it's out of date

Needs Python 3.11+ (tomllib).
'''

import re
import sys
import tomllib
from pathlib import Path

REPO = Path(__file__).resolve().parent
PYPROJECT = REPO / 'pyproject.toml'
REQUIREMENTS = REPO / 'requirements.txt'
INCLUDED_EXTRAS = ('gui',)


def _key_comments(text):
    '''Map each `key = [` in pyproject.toml to the # comment block right above it.'''
    comments, pending = {}, []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('#'):
            pending.append(stripped.lstrip('#').strip())
            continue
        match = re.match(r'([A-Za-z0-9_-]+)\s*=\s*\[', stripped)
        if match:
            comments[match.group(1)] = pending
        pending = []
    return comments


def _block(title, notes, deps, commented=False):
    lines = [f'# {title}'] + [f'# {note}' for note in notes if note]
    lines += [f'# {dep}' if commented else dep for dep in deps]
    return lines


def render():
    text = PYPROJECT.read_text(encoding='utf-8')
    project = tomllib.loads(text)['project']
    notes = _key_comments(text)
    extras = project.get('optional-dependencies', {})
    missing = [name for name in INCLUDED_EXTRAS if name not in extras]
    if missing:
        raise SystemExit(f"pyproject.toml has no extra named {', '.join(missing)}")

    install = '.[' + ','.join(INCLUDED_EXTRAS) + ']' if INCLUDED_EXTRAS else '.'
    lines = [
        '# GENERATED from pyproject.toml by update_requirements.py - do not edit by hand.',
        '# Change dependencies in pyproject.toml, then run: python update_requirements.py',
        f'# Same dependencies as: pip install -e "{install}"',
        '',
    ]
    lines += _block('Core library', notes.get('dependencies', []), project.get('dependencies', []))
    for name in INCLUDED_EXTRAS:
        lines.append('')
        lines += _block(f'[{name}] extra', notes.get(name, []), extras[name])

    others = [name for name in extras if name not in INCLUDED_EXTRAS]
    if others:
        lines += ['', '# ---- Optional extras, not installed by this file ----']
        for name in others:
            lines.append('')
            lines += _block(f'[{name}] extra', notes.get(name, []), extras[name], commented=True)
    return '\n'.join(lines) + '\n'


def main(argv):
    wanted = render()
    current = REQUIREMENTS.read_text(encoding='utf-8') if REQUIREMENTS.exists() else ''
    if current == wanted:
        print('requirements.txt is up to date')
        return 0
    if '--check' in argv:
        print('requirements.txt is OUT OF DATE - run: python update_requirements.py')
        return 1
    REQUIREMENTS.write_text(wanted, encoding='utf-8')
    print('requirements.txt regenerated from pyproject.toml')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
