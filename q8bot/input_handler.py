'''
Written by yufeng.wu0902@gmail.com

Unified input handler for Q8bot control.
Abstracts keyboard and joystick input into a common interface.
'''

import pygame
from control_config import (
    KEYBOARD_MAPPING,
    check_joystick_compatible,
    get_joystick_mapping,
    apply_deadzone,
    get_joystick_direction
)
from helpers import Q8Logger


class InputHandler:
    """Unified input handler for both keyboard and joystick."""

    def __init__(self, use_joystick=False, joystick=None, joystick_mapping=None):
        """
        Initialize input handler.

        Args:
            use_joystick: bool, True to use joystick mode
            joystick: pygame.joystick.Joystick object (if using joystick)
            joystick_mapping: dict from get_joystick_mapping() (if using joystick)
        """
        self.use_joystick = use_joystick
        self.joystick = joystick
        self.joystick_mapping = joystick_mapping

    def get_movement_direction(self):
        """
        Get current movement direction from keyboard or joystick.

        Returns:
            str: Direction command string or None if no input
                For joystick: Can return 12 different commands (f, b, l, r, fl_0.75, fl_0.5, etc.)
                For keyboard: Returns 6 basic commands (f, b, l, r, fl, fr)
        """
        if self.use_joystick:
            pygame.event.pump()
            axes = self.joystick_mapping['axes'] if self.joystick_mapping else {
                'horizontal': 0, 'vertical': 1, 'deadzone': 0.1
            }
            deadzone = axes.get('deadzone', 0.1)

            axis0 = apply_deadzone(self.joystick.get_axis(axes['horizontal']), deadzone)
            axis1 = apply_deadzone(self.joystick.get_axis(axes['vertical']), deadzone)

            # Use analog mode for joystick to get full range of commands
            return get_joystick_direction(axis0, axis1, analog_mode=True)
        else:
            # Keyboard mode - map to basic commands
            keys = pygame.key.get_pressed()
            # Map keyboard to closest trajectory equivalents
            if keys[KEYBOARD_MAPPING['movement']['forward']]:
                return 'f'
            elif keys[KEYBOARD_MAPPING['movement']['backward']]:
                return 'b'
            elif keys[KEYBOARD_MAPPING['movement']['left']]:
                return 'l'
            elif keys[KEYBOARD_MAPPING['movement']['right']]:
                return 'r'
            elif keys[KEYBOARD_MAPPING['movement']['forward_left']]:
                return 'fl_0.75'  # Map to moderate forward-left turn
            elif keys[KEYBOARD_MAPPING['movement']['forward_right']]:
                return 'fr_0.75'  # Map to moderate forward-right turn
            return None

    def is_movement_input(self):
        """
        Check if any movement input is active.

        Returns:
            bool: True if movement input detected
        """
        return self.get_movement_direction() is not None

    def is_action_pressed(self, action_name):
        """
        Check if a specific action button/key is pressed.

        Args:
            action_name: str, action name ('greet', 'battery', 'switch_gait', etc.)

        Returns:
            bool: True if action is pressed
        """
        if self.use_joystick:
            pygame.event.pump()
            if self.joystick_mapping and action_name in self.joystick_mapping['actions']:
                button_num = self.joystick_mapping['actions'][action_name]
                if isinstance(button_num, (list, tuple)):
                    return any(self.joystick.get_button(b) for b in button_num)
                return self.joystick.get_button(button_num)
            return False
        else:
            # Keyboard mode
            keys = pygame.key.get_pressed()
            if action_name in KEYBOARD_MAPPING['actions']:
                return keys[KEYBOARD_MAPPING['actions'][action_name]]
            return False


KEYBOARD_LABEL = "Keyboard (Default)"


def list_available_joysticks():
    """
    Enumerate all connected joysticks and check their compatibility.
    Does not select or commit to any input device.

    Returns:
        list of dict: [{'index': int, 'name': str, 'compatible': bool}, ...]
    """
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()

    Q8Logger.debug("=" * 60)
    Q8Logger.debug(f"Detected {joystick_count} joystick(s)")

    devices = []
    for i in range(joystick_count):
        js = pygame.joystick.Joystick(i)
        js.init()
        compatible = check_joystick_compatible(js)
        Q8Logger.debug(f"Joystick {i}: {js.get_name()} "
                       f"(Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}, "
                       f"Compatible: {compatible})")
        devices.append({'index': i, 'name': js.get_name(), 'compatible': compatible})
    Q8Logger.debug("=" * 60)

    return devices


def prompt_controller_selection(devices):
    """
    Show a dropdown dialog letting the user choose which input device to use.
    Always offers "Keyboard (Default)" alongside any compatible joysticks found.

    Args:
        devices: list of dicts from list_available_joysticks()

    Returns:
        int or None: index of the selected joystick, or None to use keyboard
    """
    import tkinter as tk
    from tkinter import ttk

    compatible_devices = [d for d in devices if d['compatible']]
    labels = [KEYBOARD_LABEL] + [f"{d['name']} (Joystick {d['index']})" for d in compatible_devices]

    root = tk.Tk()
    root.title("Q8bot - Select Input Device")
    root.resizable(False, False)

    window_width, window_height = 340, 130
    x = (root.winfo_screenwidth() - window_width) // 2
    y = (root.winfo_screenheight() - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.attributes("-topmost", True)

    tk.Label(root, text="Select control input:").pack(pady=(15, 5))

    selected = tk.StringVar(value=labels[0])
    combo = ttk.Combobox(root, textvariable=selected, values=labels, state="readonly", width=35)
    combo.current(0)
    combo.pack(pady=5)

    result = {'choice': labels[0]}

    def on_ok():
        result['choice'] = selected.get()
        root.destroy()

    tk.Button(root, text="OK", command=on_ok, width=10).pack(pady=(10, 5))
    root.protocol("WM_DELETE_WINDOW", on_ok)  # closing window keeps the default (keyboard)
    root.mainloop()

    chosen_label = result['choice']
    for d in compatible_devices:
        if chosen_label == f"{d['name']} (Joystick {d['index']})":
            return d['index']
    return None


def prompt_bot_selection(devices, com_ports):
    """
    Show a single dialog with two dropdowns: COM port and input device.

    Args:
        devices: list of dicts from list_available_joysticks()
        com_ports: list of str, available COM port device names (non-empty)

    Returns:
        tuple: (com_port, joystick_index)
            - com_port: str, the selected COM port
            - joystick_index: int or None (None = keyboard)
    """
    import tkinter as tk
    from tkinter import ttk

    compatible_devices = [d for d in devices if d['compatible']]
    controller_labels = [KEYBOARD_LABEL] + [f"{d['name']} (Joystick {d['index']})" for d in compatible_devices]

    root = tk.Tk()
    root.title("Q8bot - Select Port and Input Device")
    root.resizable(False, False)

    window_width, window_height = 360, 200
    x = (root.winfo_screenwidth() - window_width) // 2
    y = (root.winfo_screenheight() - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.attributes("-topmost", True)

    tk.Label(root, text="Select COM port:").pack(pady=(15, 5))
    port_var = tk.StringVar(value=com_ports[0])
    port_combo = ttk.Combobox(root, textvariable=port_var, values=com_ports, state="readonly", width=35)
    port_combo.current(0)
    port_combo.pack(pady=5)

    tk.Label(root, text="Select control input:").pack(pady=(15, 5))
    controller_var = tk.StringVar(value=controller_labels[0])
    controller_combo = ttk.Combobox(root, textvariable=controller_var, values=controller_labels,
                                     state="readonly", width=35)
    controller_combo.current(0)
    controller_combo.pack(pady=5)

    result = {'port': port_var.get(), 'choice': controller_labels[0]}

    def on_ok():
        result['port'] = port_var.get()
        result['choice'] = controller_var.get()
        root.destroy()

    tk.Button(root, text="OK", command=on_ok, width=10).pack(pady=(15, 5))
    root.protocol("WM_DELETE_WINDOW", on_ok)  # closing window keeps current selections
    root.mainloop()

    chosen_port = result['port']
    chosen_label = result['choice']
    joystick_index = None
    for d in compatible_devices:
        if chosen_label == f"{d['name']} (Joystick {d['index']})":
            joystick_index = d['index']
            break

    return chosen_port, joystick_index


def init_selected_joystick(index):
    """
    Initialize the joystick chosen by the user and build its control mapping.

    Args:
        index: int or None, joystick index from prompt_controller_selection(),
            or None to use keyboard control.

    Returns:
        tuple: (use_joystick, joystick, joystick_mapping)
            - use_joystick: bool, True if joystick should be used
            - joystick: pygame.joystick.Joystick object or None
            - joystick_mapping: dict from get_joystick_mapping() or None
    """
    if index is None:
        Q8Logger.debug(">>> Using KEYBOARD control <<<")
        return False, None, None

    js = pygame.joystick.Joystick(index)
    js.init()
    joystick_mapping = get_joystick_mapping(js)

    if joystick_mapping is None:
        Q8Logger.warning(f"Error loading mapping for '{js.get_name()}' - falling back to keyboard")
        return False, None, None

    Q8Logger.debug(f">>> Using joystick: '{js.get_name()}' <<<")
    if joystick_mapping['recognized']:
        Q8Logger.debug(">>> Controller is RECOGNIZED (custom mapping loaded) <<<")
    else:
        Q8Logger.warning("Controller not recognized - using Generic Controller fallback mapping")

    return True, js, joystick_mapping


def detect_and_init_joystick():
    """
    Show a device-selection dropdown and initialize the chosen input device.
    Falls back to keyboard if no compatible joystick is found or selected.

    Returns:
        tuple: (use_joystick, joystick, joystick_mapping)
    """
    devices = list_available_joysticks()
    selected_index = prompt_controller_selection(devices)
    return init_selected_joystick(selected_index)
