'''
Written by yufeng.wu0902@gmail.com

Control mappings configuration for Q8bot.
Defines keyboard and joystick control schemes using a two-layer mapping system.
'''

import math
import pygame
from .helpers import Q8Logger

# =============================================================================
# KEYBOARD CONFIGURATION
# =============================================================================

KEYBOARD_MAPPING = {
    'movement': {
        'forward': pygame.K_w,
        'backward': pygame.K_s,
        'left': pygame.K_a,
        'right': pygame.K_d,
        'forward_left': pygame.K_q,
        'forward_right': pygame.K_e,
    },
    'actions': {
        'greet': pygame.K_h,
        'battery': pygame.K_b,
        'switch_gait': pygame.K_g,
        'jump': pygame.K_j,
        'reset': pygame.K_r,
        'exit': pygame.K_ESCAPE,
        'show_range': pygame.K_c,
        'brightness_cycle': pygame.K_l,
    }
}

# =============================================================================
# JOYSTICK CONFIGURATION
# =============================================================================

# Joystick configuration embedded as Python dictionary
# This configuration defines controller mappings and action bindings

# Shared by both Joy-Con pairing modes: SDL reports "(R)" for a lone right
# Joy-Con, but "(L/R)" when both halves are paired together over Bluetooth.
_JOYCON_MAPPING = {
    "axes": {
        "horizontal": 0,
        "vertical": 1,
        "deadzone": 0.2
    },
    "buttons": {
        "top": 2,
        "bottom": 1,
        "left": 3,
        "right": 0,
        "side1": [9, 10],  # L/R (either Joy-Con) -> reset
        "side2": 5         # Home -> exit
    }
}

JOYSTICK_CONFIG = {
    "controllers": {
        "Nintendo Switch Joy-Con (R)": _JOYCON_MAPPING,
        "Nintendo Switch Joy-Con (L/R)": _JOYCON_MAPPING,
        "Controller for PC": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.1
            },
            "buttons": {
                "top": 0,
                "bottom": 2,
                "left": 3,
                "right": 1,
                "side1": 4,
                "side2": 9
            }
        },
        "Xbox Series X Controller": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.1,
                # Right stick, for static stance trim (see InputHandler.
                # get_stance_offset() / operate.py). Verified live on
                # hardware (axis_probe.py): this controller's axis order is
                # 0=LX, 1=LY, 2=RX, 3=RY, 4=LT, 5=RT - note RX/RY come
                # before the triggers, unlike some other SDL2 mappings.
                # Triggers rest at -1.0 (not 0.0), so mapping either of
                # them here would masquerade as a permanently-deflected
                # stick and constantly displace the stance.
                "stance_horizontal": 2,
                "stance_vertical": 3,
                # RT/LT - analog trigger axes (idle -1.0, +1.0 fully
                # pressed), verified live (rt_probe.py). Used via
                # InputHandler.is_trigger_pressed(), not is_action_pressed()
                # (which only handles discrete buttons). RT drives 'shoot';
                # LT drives trot-in-place (see operate.py).
                "trigger_right": 5,
                "trigger_left": 4
            },
            "buttons": {
                "top": 3,
                "bottom": 0,
                "left": 2,
                "right": 1,
                "side1": 4,
                "side2": 7,
                "side3": 5  # RB - right bumper
            }
        },
        "Generic Controller": {
            "axes": {
                "horizontal": 0,
                "vertical": 1,
                "deadzone": 0.2
            },
            "buttons": {
                "top": 0,
                "bottom": 1,
                "left": 2,
                "right": 3,
                "side1": 4,
                "side2": 5
            }
        }
    },
    "requirements": {
        "min_axes": 2,
        "min_buttons": 6
    },
    "action_mapping": {
        "greet": "top",
        "battery": "right",
        "switch_gait": "bottom",
        "jump": "left",
        "reset": "side1",
        "exit": "side2",
        "toggle_torque": "side3"
    }
}

# Movement control settings (universal for all joysticks)
JOYSTICK_MOVEMENT = {
    'forward_backward_axis': 1,      # axis[1]: -1 = forward, +1 = backward
    'left_right_axis': 0,             # axis[0]: -1 = left, +1 = right
    'analog_mode': True,              # Analog stride control enabled
}

# =============================================================================
# JOYSTICK HELPER FUNCTIONS
# =============================================================================

def check_joystick_compatible(joystick):
    """
    Check if the connected joystick is compatible.
    Returns True if joystick is recognized OR meets minimum requirements.

    Args:
        joystick: pygame.joystick.Joystick object

    Returns:
        bool: True if joystick is compatible, False otherwise
    """
    if joystick is None or JOYSTICK_CONFIG is None:
        return False

    joystick_name = joystick.get_name()
    num_axes = joystick.get_numaxes()
    num_buttons = joystick.get_numbuttons()

    # Check if joystick is in our recognized list
    if joystick_name in JOYSTICK_CONFIG['controllers']:
        Q8Logger.debug(f"Recognized joystick: '{joystick_name}'")
        return True

    # Fall back to minimum requirements check
    required_axes = JOYSTICK_CONFIG['requirements']['min_axes']
    required_buttons = JOYSTICK_CONFIG['requirements']['min_buttons']

    if num_axes < required_axes or num_buttons < required_buttons:
        # Debug, not a warning: SDL enumerates plenty of things as joysticks
        # that were never meant to be one (Bluetooth mice, for instance), and
        # rejecting those is the system working, not a problem to report.
        Q8Logger.debug(f"Joystick '{joystick_name}' does not meet requirements "
                       f"(needs {required_axes} axes / {required_buttons} buttons, "
                       f"has {num_axes} / {num_buttons}) - not offering it.")
        return False

    Q8Logger.debug(f"Joystick '{joystick_name}' meets minimum requirements (generic mode)")
    return True


def get_joystick_mapping(joystick):
    """
    Get the button/axis mapping for a specific joystick using two-layer lookup.

    Layer 1: Action → Physical Position (e.g., 'greet' → 'top')
    Layer 2: Physical Position → Button Number (e.g., 'top' → 2 for Joycon, 0 for GPDWIN)

    If joystick is not recognized but meets minimum requirements, falls back to GPDWIN mapping.

    Args:
        joystick: pygame.joystick.Joystick object

    Returns:
        dict: Joystick mapping with structure:
            {
                'name': str,
                'axes': dict,
                'buttons': dict,      # Physical position → button number
                'actions': dict,      # Action → button number (resolved)
                'recognized': bool
            }
        None if joystick not supported or config not loaded
    """
    if joystick is None or JOYSTICK_CONFIG is None:
        return None

    joystick_name = joystick.get_name()
    controller_config = None
    recognized = False

    # Check if we have a config for this specific controller
    if joystick_name in JOYSTICK_CONFIG['controllers']:
        controller_config = JOYSTICK_CONFIG['controllers'][joystick_name]
        recognized = True
    else:
        # Fall back to Generic Controller mapping for unrecognized controllers
        if 'Generic Controller' in JOYSTICK_CONFIG['controllers']:
            controller_config = JOYSTICK_CONFIG['controllers']['Generic Controller']
            recognized = False  # Using generic fallback
        else:
            return None

    # Build action-to-button mapping using two-layer lookup
    action_buttons = {}
    for action, position in JOYSTICK_CONFIG['action_mapping'].items():
        if position in controller_config['buttons']:
            action_buttons[action] = controller_config['buttons'][position]

    return {
        'name': joystick_name,
        'axes': controller_config['axes'],
        'buttons': controller_config['buttons'],  # Physical position mapping
        'actions': action_buttons,                # Logical action mapping
        'recognized': recognized
    }


def apply_deadzone(value, deadzone):
    """
    Apply deadzone to joystick axis value.

    Args:
        value: float, axis value
        deadzone: float, deadzone threshold

    Returns:
        float: 0.0 if within deadzone, otherwise original value
    """
    return 0.0 if abs(value) < deadzone else value


def get_joystick_magnitude(axis0, axis1):
    """
    Compute overall joystick push distance from center (0.0-1.0), independent
    of direction. Used for speed-tier selection (e.g. fast vs slow trot) -
    a separate concern from the angle-based direction/turn-radius logic in
    get_joystick_direction().

    Args:
        axis0: float, left/right axis value (post-deadzone)
        axis1: float, forward/backward axis value (post-deadzone)

    Returns:
        float: 0.0 (centered) to 1.0 (fully pushed), clamped
    """
    return min(1.0, math.hypot(axis0, axis1))


def get_joystick_direction(axis0, axis1, analog_mode=False):
    """
    Determine movement direction from joystick axes.
    Returns recognized command strings matching trajectory names.

    NOTE: axis0 is left/right, axis1 is forward/backward

    Mapping scheme (analog mode):
    - Straight: abs(a[0]) < 0.25 → 'f' or 'b'
    - Moderate turn: 0.25 <= abs(a[0]) < 0.6 → 'fl_0.75', 'fr_0.75', 'bl_0.75', 'br_0.75'
    - Strong turn: 0.6 <= abs(a[0]) < 0.85 → 'fl_0.5', 'fr_0.5', 'bl_0.5', 'br_0.5'
    - Full left/right: abs(a[0]) >= 0.85 and abs(a[1]) <= 0.4 → 'l', 'r'

    Args:
        axis0: float, left/right axis value (negative = left, positive = right)
        axis1: float, forward/backward axis value (negative = forward, positive = backward)
        analog_mode: bool, if True returns command strings, else simple directions

    Returns:
        str: Command string ('f', 'b', 'l', 'r', 'fl_0.75', 'fl_0.5', etc.) or None
    """
    if not analog_mode:
        # Binary mode: simple 4-direction control
        if axis1 < 0:
            return 'f'  # Forward
        elif axis1 > 0:
            return 'b'  # Backward
        elif axis0 < 0:
            return 'l'  # Left turn
        elif axis0 > 0:
            return 'r'  # Right turn
        else:
            return None
    else:
        # Analog mode: returns command strings matching trajectory names
        # Check for no input
        if axis0 == 0 and axis1 == 0:
            return None

        abs_a0 = abs(axis0)
        abs_a1 = abs(axis1)

        # Full turn mode: high horizontal, low vertical (tightened window)
        if abs_a0 >= 0.85 and abs_a1 <= 0.4:
            # Only very extreme positions are pure left/right
            return 'l' if axis0 < 0 else 'r'

        # Forward/backward with turning
        if abs_a1 > abs_a0:  # Forward/backward dominant
            base_dir = 'f' if axis1 < 0 else 'b'
            turn_dir = 'l' if axis0 < 0 else 'r'

            # Determine turn intensity (expanded 0.5 window)
            if abs_a0 < 0.25:  # Straight movement
                return base_dir
            elif abs_a0 < 0.6:  # Moderate turn (75% stride)
                return f"{base_dir}{turn_dir}_0.75"
            else:  # Strong turn (50% stride) - now abs_a0 >= 0.6
                return f"{base_dir}{turn_dir}_0.5"
        else:
            # Turning is dominant - use full turn mode
            return 'l' if axis0 < 0 else 'r'
