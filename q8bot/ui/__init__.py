'''
Desktop control UI for Q8bot (PySide6).

Needs the 'gui' extra: pip install -e .[gui]

    python run_gui.py          # or: python -m q8bot.ui

Kept out of q8bot/__init__.py so importing the library never pulls Qt in.
'''

from .main_window import MainWindow, main

__all__ = ['MainWindow', 'main']
