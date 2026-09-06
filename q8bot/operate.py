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

if __package__ in (None, ''):
    # Running as a loose script (`python operate.py`, or an editor's "run
    # this file") rather than `python -m q8bot.operate`. The relative
    # imports below need a package to resolve against, so put python-tools
    # on the path and adopt the package name.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = 'q8bot'

from .espnow import q8_espnow
from .helpers import XiaoPortFinder, Q8Logger
from .robot import Q8Bot
from .input_handler import (
    InputHandler,
    list_available_joysticks,
    init_selected_joystick,
)
from .startup_dialogs import prompt_controller_selection, prompt_bot_selection
from .log_overlay import PygameLogOverlay

# UI loop rate. Only input polling and rendering happen at this rate now -
# gait timing belongs to Q8Bot's own thread (see robot.TICK_HZ), so this
# could drop to 60 without affecting how the robot moves.
SPEED = 200

# Static stance trim via the right stick, while idle only (see
# InputHandler.get_stance_offset()): max mm offset from the current gait's
# baseline at full stick deflection. Tuned by hand on hardware.
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

def get_resource_path(relative_path):
    """Get absolute path to resource, works for dev and for PyInstaller"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        # Running in development mode
        base_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)



def main():
    """Run the pygame control app."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Q8bot control script')
    parser.add_argument('com_port', nargs='?', help='COM port for ESP32C3 (optional, auto-detect if not provided)')
    parser.add_argument('--debug', action='store_true', help='Enable debug logging')
    args = parser.parse_args()

    # Initialize logger
    log = Q8Logger(debug=args.debug)

    # Which gait execution mode the startup dropdown picked - local (the robot
    # runs gaits from its own flashed tables) or remote (streamed from here).
    use_local_gaits = True

    # Index into LED_BRIGHTNESS_LEVELS, advanced by the 'brightness_cycle'
    # action ('l' key) - always starts at 100% (index 0) regardless of the
    # robot's actual persisted brightness, since we never read that back.
    led_brightness_index = 0

    # Start pygame instance
    pygame.init()
    window = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    # Mirror log messages onto the control window (see log_overlay.py). The
    # surface is passed in at render time, so a window resize needs no fixup.
    log_overlay = PygameLogOverlay()
    Q8Logger.add_handler(log_overlay)

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
        nonlocal scaled_instruction_image, scaled_for_window_size

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

    # Connect and hand everything robot-shaped to Q8Bot: it owns the link, the
    # gait catalog, and the movement state machine, and runs its own thread to
    # keep gaits going. This loop's only job from here is turning input into
    # commands (see robot.py).
    bot = Q8Bot(q8_espnow(com_port), use_local_gaits=use_local_gaits)
    log.info(f"Gaits: {', '.join(bot.selectable)} (starting on {bot.gait})")

    bot.set_torque(True)
    bot.stand(1000)


    def rumble_gamepad():
        """Fired mid-shoot, timed to the kick rather than the trigger press."""
        if not joystick:
            return
        try:
            joystick.rumble(0.5, 0.5, 100)
        except Exception:
            pass  # Rumble unsupported on this device - not fatal


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

        # Clear screen and render logger messages
        window.fill((0, 0, 0))  # Black background

        # Draw instruction image below logger, scaled to fit the current window
        current_image = get_scaled_instruction_image(window.get_size())
        if current_image is not None:
            image_x = (window.get_width() - current_image.get_width()) // 2
            window.blit(current_image, (image_x, IMAGE_TOP_OFFSET))

        log_overlay.render(window)  # Draw logger on top
        pygame.display.flip()

        # Battery replies are drained by Q8Bot's thread; this just picks up a
        # value the first time it's seen.
        battery = bot.take_battery_reading()
        if battery is not None:
            log.info(f"Battery: {battery}%")

        direction = input_handler.get_movement_direction()
        if direction:
            # Keeps going on its own from here - Q8Bot re-sends or streams as
            # its execution mode requires. Re-issuing the same direction each
            # frame is a no-op; False means this gait has no such direction
            # (e.g. CRAWL has no true 'l'/'r'), leaving the robot as it was.
            bot.move(direction, input_handler.get_movement_magnitude())
        elif input_handler.is_trigger_pressed('trigger_left'):
            bot.trot_in_place()
        else:
            bot.idle()  # no-op once already idle

            # Static stance trim via the right stick - idle only (walking gaits
            # own the leg positions once movement starts). Recentring the stick
            # eases back to the gait's baseline rather than adopting the trim.
            bot.stance_trim(*input_handler.get_stance_offset(STANCE_TRIM_MAX_DX,
                                                             STANCE_TRIM_MAX_DY))

            if input_handler.is_action_pressed('reset'):
                log.info("Gait Reset")
                bot.stand(500)
                time.sleep(0.2)
            elif input_handler.is_action_pressed('jump'):
                log.info("Jump")
                bot.jump()
            elif input_handler.is_action_pressed('toggle_torque'):
                # Deliberately does not touch movement state - the ESPNow
                # connection and the gait keep running exactly as before either
                # way, to check for interference (firmware ignores position
                # writes while untorqued; TORQUE_SET is already idempotent/
                # edge-detected on the robot side, see q8Dynamixel.cpp's
                # _prevTorqueFlag check).
                if bot.set_torque(not bot.torque):
                    log.info("Torque ON")
                else:
                    log.info("Torque OFF")
                time.sleep(0.2)
            elif input_handler.is_action_pressed('switch_gait'):
                new_gait = bot.next_gait()
                if new_gait:
                    log.info(f"Switched to {new_gait}")
                else:
                    log.error("Failed to switch gait")
                time.sleep(0.2)
            elif input_handler.is_action_pressed('battery'):
                bot.request_battery()
                time.sleep(0.2)
            elif input_handler.is_action_pressed('record'):
                log.debug("Record next movement")
                bot.start_recording()
                time.sleep(0.2)
            elif input_handler.is_action_pressed('show_range'):
                log.info("Show Range")
                bot.show_range()
                time.sleep(0.2)
            elif input_handler.is_action_pressed('greet'):
                log.info("Greet")
                bot.greet()
                time.sleep(0.2)
            elif input_handler.is_trigger_pressed('trigger_right'):
                log.info("Shoot")
                bot.shoot(on_kick=rumble_gamepad)
                time.sleep(0.2)
            elif input_handler.is_action_pressed('brightness_cycle'):
                led_brightness_index = (led_brightness_index + 1) % len(LED_BRIGHTNESS_LEVELS)
                brightness = LED_BRIGHTNESS_LEVELS[led_brightness_index]
                percent = LED_BRIGHTNESS_PERCENTS[led_brightness_index]
                log.info(f"LED brightness: {percent}% (raw {brightness})")
                bot.set_led_brightness(brightness)
                time.sleep(0.2)
            elif input_handler.is_action_pressed('exit'):
                break

    bot.close()
    if joystick:
        joystick.quit()
    pygame.quit()



if __name__ == '__main__':
    main()
