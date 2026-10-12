'''
A tour of the Q8botOne Python API.

Before running:
  - plug the handheld controller into this computer over USB
  - switch the robot on and wait for its light to turn solid blue (paired)
  - put the robot on the floor with clear space around it - it walks, turns
    and jumps during the tour

Run the whole tour, or only the parts you name:

    python example.py
    python example.py walk pose

Parts, in tour order: readings, walk, gaits, pose, routines, moves.

To poke at it by hand instead, start Python in this folder:

    >>> from q8bot import Q8Bot
    >>> bot = Q8Bot.connect()
    INFO: Connected to the controller on COM10 - the robot is answering (battery 87%)
    >>> bot.set_torque(True)
    True
    >>> bot.move(0.5)
    True
    >>> bot
    <Q8Bot on COM10: connected, motors on, walking, TROT_FAST, battery 87%>
    >>> bot.close()

Ctrl+C stops the robot at any point: leaving the `with` block turns the
motors off and the robot sits down. Every call is documented in
q8bot/robot.py.
'''

import sys
import time

from q8bot import Q8Bot, routine_store
from q8bot.protocol import sensor_kind, ticks_to_degrees


def wait_for(read, timeout=2.0):
    """Poll read() until it returns something other than None (or give up)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = read()
        if value is not None:
            return value
        time.sleep(0.05)
    return None


def readings(bot):
    """Battery, joint telemetry and Qwiic sensors - nothing moves."""
    # bot.ping() is the quick "are you there?"; it also refreshes
    # battery_percent and battery_volts. To ask and collect yourself:
    bot.request_battery()                         # the reply arrives in the background
    percent = wait_for(bot.take_battery_reading)  # each reply is handed out once
    print(f'Battery: {percent}% ({bot.battery_volts} V) | robot answering: {bot.ping()}')

    # Joint readings are requested in the background at telemetry_hz (0 = off).
    bot.telemetry_hz = 10
    sample = wait_for(lambda: bot.telemetry)
    bot.telemetry_hz = 0
    if sample is None:
        print('No joint readings arrived')
    else:
        print('Joint angles (deg): ', [round(ticks_to_degrees(t), 1) for t in sample['position']])
        print('Currents (mA):      ', sample['current'])
        print('Temperatures (C):   ', sample['temperature'])

    # Sensors on the robot's Qwiic port work the same way, at sensor_hz.
    bot.sensor_hz = 5
    snapshot = wait_for(lambda: bot.sensors)
    bot.sensor_hz = 0
    if snapshot is None:
        print('No sensor reply - robot firmware may be too old')
        return
    if not snapshot.sensors:
        print('No Qwiic sensors attached')
    for reading in snapshot.sensors:
        kind = sensor_kind(reading.kind, len(reading.values))
        print(f'{kind.name} at 0x{reading.address:02X}:')
        for (label, unit), value in zip(kind.values, reading.values):
            print(f'    {label:<14} {value:9.2f} {unit}')


def walk(bot):
    """move(forward, turn): walks until the next command, returns at once."""
    # forward and turn both run -1..1. Their ratio picks the path (straight,
    # gentle curve, tight curve, spin) and their size the speed - Trot has
    # three speeds: under 0.52 slow, under 0.82 medium, then fast.
    # Positive turn always turns the nose left, also when backing up.
    steps = [
        (0.3, 0, 'forward, slow'),
        (-0.3, 0, 'back, slow'),
        (1.0, 0, 'forward, fast'),
        (-1.0, 0, 'back, fast'),
        (0.8, 0.3, 'forward, curving left'),
        (-0.8, -0.3, 'backing up, nose turning right - roughly back the way it came'),
        (0, 0.6, 'spinning left on the spot'),
        (0, -0.6, 'spinning right on the spot'),
    ]
    for forward, turn, what in steps:
        print(f'move({forward}, {turn}): {what}')
        bot.move(forward, turn)
        time.sleep(2)

    print('trot_in_place()')
    bot.trot_in_place()
    time.sleep(2)

    print('move(0, 0): stop and stand')
    bot.move(0, 0)                # same as bot.idle()
    time.sleep(0.5)


def gaits(bot):
    """Switch between the robot's gaits."""
    for gait in bot.catalog:      # GaitInfo: name, label, description, stance
        print(f'    {gait.name:<10} {gait.label}')
    start = bot.gait

    print("set_gait('TROT_HIGH'): forward and back")
    bot.set_gait('TROT_HIGH')     # stops, then settles onto the new gait's stance
    for forward in (0.6, -0.6):
        bot.move(forward)
        time.sleep(2)

    print("set_gait('CRAWL'): forward and back")
    bot.set_gait('CRAWL')         # straight from walking - it stops first
    # Crawl and Bound only walk straight: curves come out straight, and a
    # spin can't be done at all - the robot stands still and move() says so.
    print('    can turn:', bot.can_turn, '| move(0, 0.5) ->', bot.move(0, 0.5))
    for forward in (0.5, -0.5):
        bot.move(forward)
        time.sleep(3)

    bot.set_gait(start)
    time.sleep(0.5)


def pose(bot):
    """set_stance(dx, dy, roll, pitch) - shift and tilt the body, standing still."""
    bot.move(0, 0)
    poses = [
        (dict(dy=10), 'taller (dy is mm, up or down)'),
        (dict(dy=-10), 'lower'),
        (dict(dx=15), 'body shifted one way (dx is mm, front or back)'),
        (dict(dx=-15), '...and the other'),
        (dict(roll=10), 'leaning right (roll, degrees)'),
        (dict(roll=-10), 'leaning left'),
        (dict(pitch=10), 'nose up (pitch, degrees)'),
        (dict(pitch=-10), 'nose down'),
        (dict(dy=5, roll=8, pitch=-6), 'all at once'),
    ]
    for kwargs, what in poses:
        args = ', '.join(f'{k}={v}' for k, v in kwargs.items())
        reachable = bot.set_stance(**kwargs)
        print(f'set_stance({args}): {what}' + ('' if reachable else ' - out of reach'))
        time.sleep(1)

    print('set_stance(): back to the normal stance')
    bot.set_stance()
    time.sleep(0.5)


def routines(bot):
    """Play saved routines, and build a new one from captured poses."""
    # The routines the desktop app saves (Teach). play_routine() blocks
    # until the routine has finished.
    saved = routine_store.list_routines()        # [(name, path), ...]
    print('Saved routines:', [name for name, _ in saved])
    if saved:
        name, path = saved[0]
        print(f'Playing "{name}"')
        bot.play_routine(routine_store.load(path))

    # capture_pose() reads the joints' actual angles, so it needs joint
    # readings coming in. Here the poses come from set_stance(); with the
    # motors off you could pose the legs by hand instead.
    print('Capturing two poses, then playing them back as a routine')
    bot.telemetry_hz = 10
    captured = []
    for roll in (12, -12):
        bot.set_stance(roll=roll)
        time.sleep(1)                            # get there, and a fresh reading
        captured.append(bot.capture_pose())
    bot.set_stance()
    bot.telemetry_hz = 0
    if None in captured:
        print('No joint readings arrived - skipping playback')
        return

    sway = [routine_store.new_waypoint(angles, duration_ms=600) for angles in captured]
    routine = [routine_store.stance_waypoint('start'), *sway, *sway,
               routine_store.stance_waypoint('end')]
    bot.play_routine(routine)
    # routine_store.save('Sway', routine) would add it to the app's list.


def moves(bot):
    """The robot's built-in moves. Each blocks until it's done, then stands."""
    print('greet()')
    bot.greet()

    if bot.jump_allowed:          # the robot only jumps with 50% battery or more
        print('jump()')
        bot.jump()
    else:
        print('Battery under 50% - skipping jump()')
    # Also: bot.shoot() (a kick) and bot.show_range() (the legs' full range).


PARTS = {
    'readings': readings,
    'walk': walk,
    'gaits': gaits,
    'pose': pose,
    'routines': routines,
    'moves': moves,
}


def main(names):
    unknown = [name for name in names if name not in PARTS]
    if unknown:
        sys.exit(f"Unknown part: {', '.join(unknown)} - choose from {', '.join(PARTS)}")

    # connect() finds the controller among the boards on USB (to skip the
    # search, pass its port: Q8Bot.connect('COM10')), checks that the robot
    # answers and prints what it found. It moves nothing.
    with Q8Bot.connect() as bot:
        print(bot)                # the state at a glance
        if bot.battery_percent is None:   # connect() asked; no answer came
            sys.exit('The robot is not answering - is it switched on and paired?')

        print('set_torque(True): motors on, standing up')
        bot.set_torque(True)
        time.sleep(1)
        # bot.set_led_brightness(64)   # 0-255, saved on the robot

        for name in names or PARTS:
            print(f'\n--- {name} ---')
            PARTS[name](bot)
            if not bot.connected:  # cable pulled, controller reset...
                sys.exit(f'Lost the robot: {bot.last_error}')

        print('\nDone - motors off')
    # Leaving the with block stopped any gait, turned the motors off and
    # closed the port. Outside a with block, call bot.close().


if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print('\nStopped - motors off')
