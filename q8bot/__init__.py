'''
Q8bot - Python control library for the Q8bot quadruped.

    from q8bot import Q8Bot

    with Q8Bot.connect() as bot:
        bot.set_torque(True)
        bot.stand(1000)
        bot.move('f')
        time.sleep(3)
        bot.idle()

Q8Bot is the whole API for driving a robot; see robot.py for what it owns and
q8bot.operate for the pygame app built on it.

Importing this package needs only pyserial. The input and UI modules
(control_config, input_handler, log_overlay, startup_dialogs, operate) need
pygame and sometimes tkinter, so they are deliberately NOT imported here -
import them directly if you want them. Likewise kinematics_solver.fk_solve()
is the one thing that needs scipy, and imports it on demand.
'''

__version__ = '0.1.0'

from .espnow import q8_espnow
from .gait_manager import GAITS, GaitInfo, GaitManager, gait_info
from .helpers import Q8Logger, XiaoPortFinder
from .kinematics_solver import k_solver
from .protocol import (
    DIRECTION_ORDER,
    DIRECTION_TO_ID,
    GAIT_IDS,
    GAIT_NAME_TO_ID,
    PROTOCOL_VERSION,
    CmdType,
    ReplyType,
)
from .robot import Mode, Q8Bot, selectable_gaits

__all__ = [
    '__version__',
    # the main entry point
    'Q8Bot',
    'Mode',
    'selectable_gaits',
    # transport
    'q8_espnow',
    'XiaoPortFinder',
    # gaits and kinematics
    'GAITS',
    'GaitInfo',
    'GaitManager',
    'gait_info',
    'k_solver',
    # wire protocol
    'PROTOCOL_VERSION',
    'CmdType',
    'ReplyType',
    'GAIT_IDS',
    'GAIT_NAME_TO_ID',
    'DIRECTION_ORDER',
    'DIRECTION_TO_ID',
    # logging
    'Q8Logger',
]
