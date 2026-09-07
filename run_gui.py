'''
Entry point for the Q8bot desktop control window.

    python run_gui.py [--debug]

Equivalent to `python -m q8bot.ui`, or the `q8bot-gui` command if you
pip-installed the package with the 'gui' extra.
'''

import sys

from q8bot.ui import main

if __name__ == '__main__':
    sys.exit(main())
