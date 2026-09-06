'''
Written by yufeng.wu0902@gmail.com

This is the latest control script for Q8bot (using ESPNow). Run this script 
in your laptop with an ESP32C3 connected and control the robot via keyboard.
'''

import os
# Disable SDL's enhanced HIDAPI Joy-Con driver: it exposes extra virtual
# sensor/gyro devices that trigger a pygame bug (KeyError inside
# pygame.event.get()) when the Joy-Con connects/disconnects. Must be set
# before pygame.init().
os.environ.setdefault("SDL_JOYSTICK_HIDAPI_SWITCH", "0")

# Keep reading joystick input even when this window isn't focused, so
# multiple operate.py instances (one per bot/controller) can run and be
# controlled at the same time instead of only whichever window is active.
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")

import time
import pygame
import sys
import argparse
from kinematics_solver import k_solver
from espnow import q8_espnow
from protocol import ReplyType, DIRECTION_ORDER, GAIT_IDS
from helpers import XiaoPortFinder, Q8Logger
from gait_manager import GaitManager, GAITS
from routine_generator import show_range, greet, shoot
from input_handler import (
    InputHandler,
    list_available_joysticks,
    prompt_controller_selection,
    prompt_bot_selection,
    init_selected_joystick,
)

# Q8bot leg configuration
CENTER_DIST = 19.5  # Distance between two actuators
L1 = 25             # Upper leg length
L2 = 40             # Lower leg length

# Pygame config
SPEED = 200
res = 0.2

# Native on-robot gait support (see protocol.py, GaitRunner in firmware).
# Whether the "Local gaits"/"Remotely generated gaits" startup dropdown
# selected local (native) execution for every gait, or the original
# per-tick streaming for every gait. Set once below from the dropdown.
use_local_gaits = True
# Direction string -> canonical index the robot's gaitTables.h uses.
DIRECTION_TO_ID = {name: i for i, name in enumerate(DIRECTION_ORDER)}
# Name -> gait_id, reverse of protocol.GAIT_IDS.
GAIT_NAME_TO_ID = {name: gait_id for gait_id, name in GAIT_IDS.items()}
# Low-rate keep-alive: bounds how long a single dropped GAIT_RUN frame can
# leave the host and robot silently out of sync (no ACK/retry otherwise -
# a resend at this rate is far cheaper than going back to per-tick streaming).
NATIVE_GAIT_RESEND_INTERVAL = 0.25  # seconds

# Static stance trim via the right stick, while idle only (see
# InputHandler.get_stance_offset()): max mm offset from the current gait's
# baseline (pos_x, pos_y) at full stick deflection. Tuned by hand on
# hardware.
STANCE_TRIM_MAX_DX = 25.0
STANCE_TRIM_MAX_DY = 15.0

# Keyboard brightness cycle ('l' key, see control_config.py's
# 'brightness_cycle'). Not wired up for joystick/gamepad yet. Steps are
# perceptual percentages of LED_BRIGHTNESS_CAP ("full"), gamma-corrected
# (2.8, matching Adafruit_NeoPixel's own gamma32() table already used for
# hue colors in RGBManager::setHue) so each step reads as an even brightness
# drop to the eye instead of a raw linear one - low raw PWM values look
# disproportionately bright, so a "half as bright" step needs a much smaller
# raw value than half.
LED_BRIGHTNESS_GAMMA = 2.8
LED_BRIGHTNESS_CAP = 128  # "full" for this cycle - matches the robot's own power-on default
LED_BRIGHTNESS_PERCENTS = (100, 75, 50, 25)
LED_BRIGHTNESS_LEVELS = [
    round(LED_BRIGHTNESS_CAP * (pct / 100) ** LED_BRIGHTNESS_GAMMA)
    for pct in LED_BRIGHTNESS_PERCENTS
]

# Helper Functions
def move_xy(x, y, dur = 0, deg = True):
    """Move robot legs to specific x,y position."""
    q1, q2, success = leg.ik_solve(x, y, deg, 1)
    q8.move_mirror([q1, q2], dur)
    return success

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Q8bot control script')
parser.add_argument('com_port', nargs='?', help='COM port for ESP32C3 (optional, auto-detect if not provided)')
parser.add_argument('--debug', action='store_true', help='Enable debug logging')
args = parser.parse_args()

# Initialize logger
log = Q8Logger(debug=args.debug)

# Flags for main loop
movement = False
exit = False
record = False
request = "none"
native_gait_active = False  # Is the robot currently running a gait on its own?
native_direction_id = None  # Last direction_id sent while native_gait_active
native_gait_id = None       # Last gait_id (tier) sent while native_gait_active
native_tier_index = 0  # Corrected below once the default gait is loaded
native_last_sent_time = 0.0  # For the low-rate keep-alive resend
torqued_on = True  # Toggled by the 'toggle_torque' action; movement/gait keep running either way
stance_trimmed = False  # Is a static stance trim (right stick) currently applied?
# Index into LED_BRIGHTNESS_LEVELS, advanced by the 'brightness_cycle'
# action ('l' key) - always starts at 100% (index 0) regardless of the
# robot's actual persisted brightness, since we never read that back.
led_brightness_index = 0


def send_native_gait_if_needed(gait_id, direction_id):
    """
    Send GAIT_RUN for (gait_id, direction_id) if it differs from what the
    robot is currently running, or the low-rate keep-alive resend is due.
    Updates the native_* globals. Shared by stick-driven native gaits and
    the LT trot-in-place trigger - the robot only ever runs one GAIT_RUN at
    a time, so both paths drive the same state.
    """
    global native_gait_active, native_direction_id, native_gait_id, native_last_sent_time
    now = time.time()
    changed = (
        not native_gait_active
        or direction_id != native_direction_id
        or gait_id != native_gait_id
    )
    resend_due = now - native_last_sent_time >= NATIVE_GAIT_RESEND_INTERVAL
    if changed or resend_due:
        q8.run_gait(gait_id, direction_id)
        native_gait_active = True
        native_direction_id = direction_id
        native_gait_id = gait_id
        native_last_sent_time = now


# Start pygame instance
pygame.init()
window = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
clock = pygame.time.Clock()

# Set up pygame surface for logger
Q8Logger.set_pygame_surface(window)

# Resolve COM port and input device
available_joysticks = list_available_joysticks()

if args.com_port:
    # User provided a COM port on the command line - only prompt for input device
    com_port = args.com_port
    if not XiaoPortFinder.validate(com_port):
        log.error(f"{com_port} is not an ESP32C3 or does not exist.")
        sys.exit(1)
    selected_joystick_index, use_local_gaits = prompt_controller_selection(available_joysticks)
else:
    # No COM port given - let the user pick both port and input device together
    available_ports = XiaoPortFinder.list_all()
    if not available_ports:
        log.error("No ESP32C3 controller device found.")
        sys.exit(1)
    com_port, selected_joystick_index, use_local_gaits = prompt_bot_selection(available_joysticks, available_ports)

use_joystick, joystick, joystick_mapping = init_selected_joystick(selected_joystick_index)
input_handler = InputHandler(use_joystick, joystick, joystick_mapping)

# Load appropriate instruction image based on input device
def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

if use_joystick:
    instruction_image_path = get_resource_path(os.path.join("docs", "Instruction_Joystick.jpg"))
else:
    instruction_image_path = get_resource_path(os.path.join("docs", "Instruction_Default.jpg"))

try:
    instruction_image = pygame.image.load(instruction_image_path)
    log.info(f"Loaded instruction image: {os.path.basename(instruction_image_path)}")
except Exception as e:
    log.warning(f"Failed to load instruction image: {e}")
    instruction_image = None

# Vertical space reserved above the image for logger messages
IMAGE_TOP_OFFSET = 150

# Cached scaled copy of instruction_image, recomputed only when window size changes
scaled_instruction_image = None
scaled_for_window_size = None


def get_scaled_instruction_image(window_size):
    """Rescale instruction_image to fit within window_size, preserving aspect ratio."""
    global scaled_instruction_image, scaled_for_window_size

    if instruction_image is None:
        return None

    if scaled_for_window_size == window_size:
        return scaled_instruction_image

    window_width, window_height = window_size
    available_width = max(window_width, 1)
    available_height = max(window_height - IMAGE_TOP_OFFSET, 1)

    img_width, img_height = instruction_image.get_size()
    scale = min(available_width / img_width, available_height / img_height)
    scaled_size = (max(int(img_width * scale), 1), max(int(img_height * scale), 1))

    scaled_instruction_image = pygame.transform.smoothscale(instruction_image, scaled_size)
    scaled_for_window_size = window_size
    return scaled_instruction_image

# Initialize kinamatics solver and Q8bot ESPNow instance
leg = k_solver(CENTER_DIST, L1, L2, L1, L2)
q8 = q8_espnow(com_port)
q8.enable_torque()

# Initialize GaitManager
# Non-fastest tier companions (e.g. TROT_SLOW, TROT_MED) are excluded from
# the switch_gait cycle - they're internal companions GaitManager blends
# into automatically (see SPEED_TIER_CHAINS), not user-selectable presets
# in their own right. Each chain's last entry is the gait itself, which
# stays selectable as normal.
_tier_companions = {name for chain in GaitManager.SPEED_TIER_CHAINS.values() for name in chain[:-1]}
# Driven directly by the LT trigger (see below), not switch_gait-selectable.
_internal_only_gaits = {'TROT_INPLACE'}
gait_names = [name for name in GAITS.keys() if name not in _tier_companions and name not in _internal_only_gaits]
gait_manager = GaitManager(leg, GAITS)

# Trot-in-place: in local mode, driven via GAIT_RUN like any other native
# gait (see protocol.py's GAIT_IDS and firmware/q8bot_robot/include/
# gaitTables.h - re-exported below). In remote mode there's no firmware
# table to run, so it's streamed directly instead, same as the pre-native-
# gait path - a dedicated GaitManager instance keeps its phase/state fully
# separate from whichever gait is currently selected for stick movement.
inplace_gait_manager = GaitManager(leg, GAITS)
if not inplace_gait_manager.load_gait('TROT_INPLACE'):
    log.error("Failed to load TROT_INPLACE gait")
trotting_in_place = False  # Is LT currently held and driving trot-in-place?

# Starting location of leg end effector in x and y
first_gait_params = GAITS[gait_names[0]]
pos_x = first_gait_params[1]
pos_y = first_gait_params[2]
move_xy(pos_x, pos_y, 1000)

# Pre-calculate trajectories for default gait
if not gait_manager.load_gait(gait_names[0]):
    log.error(f"Failed to load default gait: {gait_names[0]}")
    sys.exit(1)

if gait_names[0] in GaitManager.SPEED_TIER_CHAINS:
    # Start fresh at the fastest tier, matching the streaming path's own
    # tiering convention on a fresh load_gait().
    native_tier_index = len(GaitManager.SPEED_TIER_CHAINS[gait_names[0]]) - 1

time.sleep(2)

while True:
    clock.tick(SPEED)
    try:
        frame_events = pygame.event.get()
    except SystemError:
        # Known pygame/SDL bug: sensor/device events from Joy-Con style
        # controllers can raise a KeyError inside pygame's event pump.
        # Safe to ignore and continue polling next frame.
        frame_events = []

    for event in frame_events:
        if event.type == pygame.VIDEORESIZE:
            window = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
            Q8Logger.set_pygame_surface(window)

    # Clear screen and render logger messages
    window.fill((0, 0, 0))  # Black background

    # Draw instruction image below logger, scaled to fit the current window
    current_image = get_scaled_instruction_image(window.get_size())
    if current_image is not None:
        image_x = (window.get_width() - current_image.get_width()) // 2
        window.blit(current_image, (image_x, IMAGE_TOP_OFFSET))

    Q8Logger.render_pygame_messages()  # Draw logger on top
    pygame.display.flip()

    if movement:
        # Get requested direction from input handler
        requested_direction = input_handler.get_movement_direction()

        if requested_direction:
            if use_local_gaits:
                # Local mode: drive every gait natively via GAIT_RUN instead
                # of streaming raw positions. A gait with a tier chain
                # (currently just TROT_FAST) blends slow/med/fast by
                # magnitude exactly like the streaming path's own tiering;
                # everything else maps 1:1 to its own gait_id.
                resolved_direction = gait_manager.resolve_direction(requested_direction)
                direction_id = DIRECTION_TO_ID.get(resolved_direction) if resolved_direction else None
                if direction_id is not None:
                    tier_chain = GaitManager.SPEED_TIER_CHAINS.get(gait_manager.current_gait)
                    if tier_chain:
                        magnitude = input_handler.get_movement_magnitude()
                        native_tier_index = GaitManager.resolve_tier_index(
                            tier_chain, GaitManager.TIER_BOUNDARIES, native_tier_index, magnitude
                        )
                        gait_id = GAIT_NAME_TO_ID[tier_chain[native_tier_index]]
                    else:
                        gait_id = GAIT_NAME_TO_ID[gait_manager.current_gait]

                    send_native_gait_if_needed(gait_id, direction_id)
                else:
                    # Matches the streaming path's failure case exactly
                    # (e.g. CRAWL/BOUND have no true 'l'/'r').
                    movement = False
            else:
                # Normal streaming path, unchanged - includes TROT_FAST's
                # fast/med/slow tiering (still raw positions, no GAIT_RUN).
                gait_manager.update_speed_tier(input_handler.get_movement_magnitude())

                if gait_manager.start_movement(requested_direction):
                    pos = gait_manager.tick()
                    if pos:
                        q8.move_all(pos, 0, record)
                else:
                    # Failed to start movement
                    movement = False
        else:
            # No movement input - transition to idle
            if native_gait_active:
                q8.stop_gait()
                native_gait_active = False
                native_direction_id = None
                native_gait_id = None
            move_xy(pos_x, pos_y, 0)
            q8.finish_recording()
            record = False
            gait_manager.stop()
            movement = False
    elif input_handler.is_trigger_pressed('trigger_left'):
        # Trot in place while LT is held.
        if use_local_gaits:
            # Native GAIT_RUN, same as any other local gait - direction is
            # arbitrary since TROT_INPLACE's xrange=0 makes every direction
            # identical (see gait_manager.py).
            send_native_gait_if_needed(GAIT_NAME_TO_ID['TROT_INPLACE'], DIRECTION_TO_ID['f'])
        else:
            # Remote mode: streamed every tick via the dedicated
            # inplace_gait_manager (see setup above).
            if not trotting_in_place:
                inplace_gait_manager.start_movement('f')
            pos = inplace_gait_manager.tick()
            if pos:
                q8.move_all(pos, 0, False)
        trotting_in_place = True
    else:
        if trotting_in_place:
            if use_local_gaits:
                q8.stop_gait()
                native_gait_active = False
                native_direction_id = None
                native_gait_id = None
            else:
                inplace_gait_manager.stop()
                move_xy(pos_x, pos_y, 0)
            trotting_in_place = False

        # Static stance trim via the right stick - idle only (walking
        # gaits own the leg positions once movement starts). Snaps back to
        # the current gait's baseline (pos_x, pos_y) once the stick
        # recenters, rather than adopting the trim as a new baseline.
        stance_dx, stance_dy = input_handler.get_stance_offset(STANCE_TRIM_MAX_DX, STANCE_TRIM_MAX_DY)
        if stance_dx or stance_dy:
            move_xy(pos_x + stance_dx, pos_y + stance_dy, 0)
            stance_trimmed = True
        elif stance_trimmed:
            move_xy(pos_x, pos_y, 300)
            stance_trimmed = False

        # Check for movement input
        if input_handler.is_movement_input():
            movement = True
        # Check action inputs using generalized interface
        elif input_handler.is_action_pressed('reset'):
            log.info("Gait Reset")
            move_xy(pos_x, pos_y, 500)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('jump'):
            log.info("Jump")
            q8.send_jump()
            time.sleep(5)
            # Jump can brown out the robot MCU mid-sequence - if it reset,
            # torque comes back up disabled, so re-send an explicit enable
            # rather than assuming the pre-jump state still holds.
            q8.enable_torque()
            torqued_on = True
            move_xy(pos_x, pos_y, 500)
        elif input_handler.is_action_pressed('toggle_torque'):
            # Deliberately does not touch movement/gait_manager state - the
            # ESPNow connection and pygame loop stay up, and movement/gait
            # keep running exactly as before regardless of torque state,
            # to check for interference (firmware ignores position writes
            # while untorqued; TORQUE_SET is already idempotent/edge-detected
            # on the robot side, see q8Dynamixel.cpp's _prevTorqueFlag check).
            torqued_on = not torqued_on
            if torqued_on:
                log.info("Torque ON")
                q8.enable_torque()
                move_xy(pos_x, pos_y, 500)  # Return to current gait's idle stance
            else:
                log.info("Torque OFF")
                q8.disable_torque()
            time.sleep(0.2)
        elif input_handler.is_action_pressed('switch_gait'):
            # Cycle to next gait
            gait_names.append(gait_names.pop(0))
            new_gait = gait_names[0]

            # Load new gait
            if gait_manager.load_gait(new_gait):
                # Update position to match new gait
                pos_x, pos_y = GAITS[new_gait][1], GAITS[new_gait][2]
                move_xy(pos_x, pos_y, 500)
                if new_gait in GaitManager.SPEED_TIER_CHAINS:
                    # Start fresh at the fastest tier, matching the
                    # streaming path's own tiering convention on a fresh
                    # load_gait().
                    native_tier_index = len(GaitManager.SPEED_TIER_CHAINS[new_gait]) - 1
                log.info(f"Switched to {new_gait}")
            else:
                log.error(f"Failed to load gait: {new_gait}")
                gait_names.insert(0, gait_names.pop())  # Revert gait change
            time.sleep(0.2)
        elif input_handler.is_action_pressed('battery'):
            q8.check_battery()
            request = "battery"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('record'):
            log.debug("Record next movement")
            record = True
            request = "data"
            time.sleep(0.2)
        elif input_handler.is_action_pressed('show_range'):
            log.info("Show Range")
            show_range(q8)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('greet'):
            log.info("Greet")
            greet(q8)
            move_xy(pos_x, pos_y, 1000)  # Return to rest position
            time.sleep(0.2)
        elif input_handler.is_trigger_pressed('trigger_right'):
            log.info("Shoot")
            shoot(q8, joystick)
            move_xy(pos_x, pos_y, 500)  # Return to rest position
            time.sleep(0.2)
        elif input_handler.is_action_pressed('brightness_cycle'):
            led_brightness_index = (led_brightness_index + 1) % len(LED_BRIGHTNESS_LEVELS)
            brightness = LED_BRIGHTNESS_LEVELS[led_brightness_index]
            percent = LED_BRIGHTNESS_PERCENTS[led_brightness_index]
            log.info(f"LED brightness: {percent}% (raw {brightness})")
            q8.set_led_brightness(brightness)
            time.sleep(0.2)
        elif input_handler.is_action_pressed('exit'):
            break
        else:
            try:
                while q8.serialHandler.in_waiting > 0:  # Check if there is any data waiting to be read
                    reply = q8.read_reply()
                    if reply is None:
                        break
                    reply_type, payload = reply
                    if request == "battery" and reply_type == ReplyType.BATTERY_REPLY:
                        log.info(f"Battery: {payload[0]}%")
                request = "none"
            except Exception:
                log.debug("Data reading failed. Continuing...")

q8.disable_torque()
if joystick:
    joystick.quit()
pygame.quit()
