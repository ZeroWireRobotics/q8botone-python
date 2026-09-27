'''
Written by yufeng.wu0902@gmail.com

Tkinter startup dialogs for operate.py: pick a COM port, an input device, and
a gait execution mode before the control window opens.

These are front-end code, kept out of input_handler.py so that reading a
gamepad doesn't drag tkinter in with it - a desktop GUI brings its own
pickers, and a script driving the robot needs neither. tkinter is imported
inside each function for the same reason.
'''

import sys

from .helpers import Q8Logger

KEYBOARD_LABEL = "Keyboard (Default)"

# Gait execution mode dropdown - see operate.py's use_local_gaits.
LOCAL_GAITS_LABEL = "Local gaits (Default)"
REMOTE_GAITS_LABEL = "Remotely generated gaits"
GAIT_MODE_LABELS = [LOCAL_GAITS_LABEL, REMOTE_GAITS_LABEL]


def prompt_controller_selection(devices):
    """
    Show a dropdown dialog letting the user choose which input device to use,
    and whether gaits should run locally on the robot or be streamed from the
    host. Always offers "Keyboard (Default)" alongside any compatible
    joysticks found.

    Args:
        devices: list of dicts from list_available_joysticks()

    Returns:
        tuple: (joystick_index, use_local_gaits)
            - joystick_index: int or None (None = keyboard)
            - use_local_gaits: bool, True = local gaits, False = remote
    """
    import tkinter as tk
    from tkinter import ttk

    compatible_devices = [d for d in devices if d['compatible']]
    labels = [f"{d['name']} (Joystick {d['index']})" for d in compatible_devices] + [KEYBOARD_LABEL]

    root = tk.Tk()
    root.title("Q8bot - Select Input Device")
    root.resizable(False, False)

    window_width, window_height = 340, 190
    x = (root.winfo_screenwidth() - window_width) // 2
    y = (root.winfo_screenheight() - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.attributes("-topmost", True)

    tk.Label(root, text="Select control input:").pack(pady=(15, 5))

    selected = tk.StringVar(value=labels[0])
    combo = ttk.Combobox(root, textvariable=selected, values=labels, state="readonly", width=35)
    combo.current(0)
    combo.pack(pady=5)

    tk.Label(root, text="Gait execution:").pack(pady=(10, 5))

    gait_mode = tk.StringVar(value=GAIT_MODE_LABELS[0])
    gait_mode_combo = ttk.Combobox(root, textvariable=gait_mode, values=GAIT_MODE_LABELS,
                                    state="readonly", width=35)
    gait_mode_combo.current(0)
    gait_mode_combo.pack(pady=5)

    result = {'choice': labels[0], 'gait_mode': GAIT_MODE_LABELS[0], 'cancelled': False}

    def on_ok():
        result['choice'] = selected.get()
        result['gait_mode'] = gait_mode.get()
        root.destroy()

    def on_cancel():
        result['cancelled'] = True
        root.destroy()

    tk.Button(root, text="OK", command=on_ok, width=10).pack(pady=(10, 5))
    root.protocol("WM_DELETE_WINDOW", on_cancel)  # closing the window aborts, does not proceed
    root.mainloop()

    if result['cancelled']:
        Q8Logger.info("Input device selection cancelled - exiting.")
        sys.exit(0)

    chosen_label = result['choice']
    joystick_index = None
    for d in compatible_devices:
        if chosen_label == f"{d['name']} (Joystick {d['index']})":
            joystick_index = d['index']
            break

    use_local_gaits = result['gait_mode'] == LOCAL_GAITS_LABEL
    return joystick_index, use_local_gaits


def prompt_bot_selection(devices, com_ports):
    """
    Show a single dialog with three dropdowns: COM port, input device, and
    gait execution mode (local vs remote).

    Args:
        devices: list of dicts from list_available_joysticks()
        com_ports: list of str, available COM port device names (non-empty)

    Returns:
        tuple: (com_port, joystick_index, use_local_gaits)
            - com_port: str, the selected COM port
            - joystick_index: int or None (None = keyboard)
            - use_local_gaits: bool, True = local gaits, False = remote
    """
    import tkinter as tk
    from tkinter import ttk

    compatible_devices = [d for d in devices if d['compatible']]
    controller_labels = [f"{d['name']} (Joystick {d['index']})" for d in compatible_devices] + [KEYBOARD_LABEL]

    root = tk.Tk()
    root.title("Q8bot - Select Port and Input Device")
    root.resizable(False, False)

    window_width, window_height = 360, 260
    x = (root.winfo_screenwidth() - window_width) // 2
    y = (root.winfo_screenheight() - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    root.attributes("-topmost", True)

    tk.Label(root, text="Select COM port:").pack(pady=(15, 5))
    port_var = tk.StringVar(value=com_ports[0])
    port_combo = ttk.Combobox(root, textvariable=port_var, values=com_ports, state="readonly", width=35)
    port_combo.current(0)
    port_combo.pack(pady=5)

    tk.Label(root, text="Select control input:").pack(pady=(10, 5))
    controller_var = tk.StringVar(value=controller_labels[0])
    controller_combo = ttk.Combobox(root, textvariable=controller_var, values=controller_labels,
                                     state="readonly", width=35)
    controller_combo.current(0)
    controller_combo.pack(pady=5)

    tk.Label(root, text="Gait execution:").pack(pady=(10, 5))
    gait_mode = tk.StringVar(value=GAIT_MODE_LABELS[0])
    gait_mode_combo = ttk.Combobox(root, textvariable=gait_mode, values=GAIT_MODE_LABELS,
                                    state="readonly", width=35)
    gait_mode_combo.current(0)
    gait_mode_combo.pack(pady=5)

    result = {
        'port': port_var.get(),
        'choice': controller_labels[0],
        'gait_mode': GAIT_MODE_LABELS[0],
        'cancelled': False,
    }

    def on_ok():
        result['port'] = port_var.get()
        result['choice'] = controller_var.get()
        result['gait_mode'] = gait_mode.get()
        root.destroy()

    def on_cancel():
        result['cancelled'] = True
        root.destroy()

    tk.Button(root, text="OK", command=on_ok, width=10).pack(pady=(15, 5))
    root.protocol("WM_DELETE_WINDOW", on_cancel)  # closing the window aborts, does not proceed
    root.mainloop()

    if result['cancelled']:
        Q8Logger.info("Port/input device selection cancelled - exiting.")
        sys.exit(0)

    chosen_port = result['port']
    chosen_label = result['choice']
    joystick_index = None
    for d in compatible_devices:
        if chosen_label == f"{d['name']} (Joystick {d['index']})":
            joystick_index = d['index']
            break

    use_local_gaits = result['gait_mode'] == LOCAL_GAITS_LABEL
    return chosen_port, joystick_index, use_local_gaits
