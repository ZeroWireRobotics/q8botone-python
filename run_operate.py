'''
Entry point for the pygame control app, and the script PyInstaller builds
from (see q8bot_build.spec).

q8bot/operate.py can't be run directly any more now that q8bot is a real
package - its imports are relative. Run this instead:

    python run_operate.py [COM_PORT] [--debug]

Equivalent to `python -m q8bot.operate`, or the `q8bot-operate` command if
you pip-installed the package.
'''

from q8bot.operate import main

if __name__ == '__main__':
    main()
