'''
Idle power test: what the robot draws as each part is switched off in
turn - standing, motors off, the status light off, the 6 V servo buck off,
the 5 V peripheral buck off too - logging the INA237 on its Qwiic port.
Each step's drop is what that part costs. The report goes into
q8botone-docs (docs/tests/battery-runtime, next to this repository).

    python tools/power_idle.py --setup "on its feet" --robot-fw 0.2.2-...+powertest

Needs matplotlib: pip install -e ".[tools]"

Before starting:
  - protocol 4 robot firmware (POWER_RAIL, and LED_BRIGHTNESS's don't-save
    flag) - older firmware ignores both, and the script says so when the
    servos keep answering with their buck off
  - the INA237 on the robot's Qwiic port, in the battery path
  - the robot's USB unplugged - otherwise the charger feeds it
  - the controller plugged into this computer
  - the robot somewhere it can stand up, and sit down again

At the end both bucks go back on, but the robot has to be switched off and
on before its motors go on again: servos that lost power come back with
their settings reset, so the firmware refuses torque until it restarts. The
restart also brings the light back (it was only turned off for now).

Ctrl+C ends the run early, and still turns both bucks back on and writes
the report. `--from-log BAT-ID-001` rebuilds a report from its log.
'''

import argparse
import csv
import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from power_run import Log, ask, find_ina237, save_meta, RESTART_JUMP_MS, POLL_S, DEFAULT_OUT   # noqa: E402
from power_report import COLORS, _band, _rolling_mean   # noqa: E402
from power_stand import reference, without_sensor, SENSOR_NOTE, _charger, _duration   # noqa: E402
from q8bot import Q8Bot                                     # noqa: E402
from q8bot.firmware_flasher import identify_device          # noqa: E402
from q8bot.helpers import XiaoPortFinder                    # noqa: E402

import matplotlib.pyplot as plt                             # noqa: E402  (power_report picked Agg)

RUN_PREFIX = 'BAT-ID-'
README_MARKER = '<!-- power_idle.py adds each run above this line -->'

# The steps, in order: (phase, what it is, short label for the plot).
# Each one switches one more thing off; 'restored' turns both bucks back
# on, to check the draw comes back to where it was before they went off.
STEPS = [
    ('standing', 'Standing, motors on', 'standing'),
    ('motors_off', 'Motors off, status light on', 'motors off'),
    ('light_off', 'Status light off', 'light off'),
    ('servos_off', '6 V buck off - the servos unpowered', '6 V off'),
    ('bucks_off', '5 V buck off too - both bucks off', '5 V off too'),
    ('restored', 'Both bucks back on (light and motors still off)', 'bucks back on'),
]
LABELS = {name: label for name, label, _ in STEPS}

# What each step's drop is - "Where the standing power goes", base first.
# (name, the step it's measured from, the step after); colors are the
# dataviz reference palette's categorical slots 1-5, checked on white.
SHARES = [
    ('Left with both bucks off: the ESP32 and the rest of the board', None, 'bucks_off'),
    ('The 5 V buck and its peripherals, light off', 'bucks_off', 'servos_off'),
    ('The servos idling, and their 6 V buck', 'servos_off', 'light_off'),
    ('The status light', 'light_off', 'motors_off'),
    ('Holding the stance', 'motors_off', 'standing'),
]
SHARE_COLORS = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4')

REPEATS = 3              # each switch goes out this many times - a radio frame can be lost
REPEAT_GAP_S = 0.05
SMOOTH_S = 1.0


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', help="the controller's port (found by asking each board if left out)")
    p.add_argument('--setup', default='not recorded',
                   help='where it stands, e.g. "on its feet" or "on a test stand"')
    p.add_argument('--pcb', default='not recorded', help='PCB revision / serial')
    p.add_argument('--pack', default='not recorded', help='battery pack')
    p.add_argument('--robot-fw', default='not recorded',
                   help="the robot's firmware (it can't report it over the radio)")
    p.add_argument('--note', default='', help='anything else worth keeping with the run')
    p.add_argument('--out', type=Path, default=DEFAULT_OUT, help='test folder (default: %(default)s)')
    p.add_argument('--measure-s', type=float, default=15.0, help='each step measured, s (default: %(default)s)')
    p.add_argument('--settle-s', type=float, default=4.0, help='each step settling first, s (default: %(default)s)')
    p.add_argument('--stand-settle-s', type=float, default=8.0,
                   help='settling after standing up, s (default: %(default)s)')
    p.add_argument('--reference', default='BAT-RT-001',
                   help='the walking run whose energy the runtime column uses (default: %(default)s)')
    p.add_argument('--sensor-hz', type=float, default=10.0)
    p.add_argument('--servo-hz', type=float, default=1.0, help='servo readings per second, 0 for none')
    p.add_argument('--charger-every', type=float, default=5.0, help='seconds between charger readings')
    p.add_argument('--ina-address', type=lambda s: int(s, 0), help='which INA237, if there are several')
    p.add_argument('--yes', action='store_true', help="don't ask before starting")
    p.add_argument('--from-log', metavar='RUN_ID', help="rebuild that run's report from its log")
    return p.parse_args()


def next_run_id(folder):
    used = [int(m.group(1)) for f in (folder / 'data').glob(f'{RUN_PREFIX}*')
            if (m := re.match(re.escape(RUN_PREFIX) + r'(\d+)', f.name))]
    return f'{RUN_PREFIX}{max(used, default=0) + 1:03d}'


# ---------- the run ----------

def _repeat(fn, *args):
    for i in range(REPEATS):
        fn(*args)
        if i < REPEATS - 1:
            time.sleep(REPEAT_GAP_S)


def _switch(bot, name):
    '''Put the robot into step `name`.'''
    if name == 'standing':
        bot.set_torque(True)          # once - it also stands the robot up
    elif name == 'motors_off':
        _repeat(bot.set_torque, False)
    elif name == 'light_off':
        _repeat(bot.set_led_brightness, 0, False)
    elif name == 'servos_off':
        _repeat(bot.set_power_rail, 6, False)
    elif name == 'bucks_off':
        _repeat(bot.set_power_rail, 5, False)
    elif name == 'restored':
        _restore(bot)


def _restore(bot):
    _repeat(bot.set_power_rail, 5, True)
    _repeat(bot.set_power_rail, 6, True)


def record(args, folder, run_id):
    port = args.port or XiaoPortFinder.find_controller()
    ident = identify_device(port)    # before Q8Bot opens the port
    bot = Q8Bot.connect(port)
    try:
        if not bot.ping():
            sys.exit("The robot isn't answering - is it switched on and paired?")
        bot.sensor_hz = args.sensor_hz
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not (bot.sensors and find_ina237(bot.sensors, args.ina_address)):
            time.sleep(0.05)
        if not (bot.sensors and find_ina237(bot.sensors, args.ina_address)):
            found = ', '.join(f'kind {s.kind} at {s.address:#04x}' for s in bot.sensors.sensors) \
                if bot.sensors else 'no sensor reply'
            sys.exit(f'No INA237 on the robot ({found}) - is it on the Qwiic port?')
        ina_address = find_ina237(bot.sensors, args.ina_address).address
        bot.telemetry_hz = args.servo_hz

        steps = [{'name': name, 'label': label,
                  'settle_s': args.stand_settle_s if name == 'standing' else args.settle_s,
                  'measure_s': args.measure_s if name != 'restored' else min(args.measure_s, 8.0)}
                 for name, label, _ in STEPS]
        total = sum(s['settle_s'] + s['measure_s'] for s in steps)
        print(f'{run_id}: controller on {port}, robot answering at {bot.battery_volts:.2f} V '
              f'({bot.battery_percent}%), INA237 at {ina_address:#04x}.')
        print(f'This takes about {total:.0f} s: the robot stands up, then sits down as its motors go '
              'off, and its light and both buck converters go off in turn. Check:\n'
              '  - it has room to stand up and sit down\n'
              '  - its USB is unplugged (or the charger feeds it)\n'
              '  - it runs protocol 4 firmware (POWER_RAIL)\n'
              'Afterwards it needs switching off and on before its motors go on again.')
        if not args.yes:
            try:
                ask('Enter to start, Ctrl+C to cancel: ')
            except KeyboardInterrupt:
                print('\nCancelled - nothing was recorded.')
                return None

        folder.joinpath('data').mkdir(parents=True, exist_ok=True)
        log = Log(folder / 'data' / f'{run_id}-log.csv')
        meta = {
            'run_id': run_id,
            'started': datetime.now().strftime('%Y-%m-%d %H:%M'),
            'setup': args.setup, 'pcb': args.pcb, 'pack': args.pack, 'note': args.note,
            'robot_fw': args.robot_fw,
            'controller': f"{port}, firmware {ident.version if ident and ident.version else 'unknown'}",
            'sensor': f'INA237 at {ina_address:#04x}, asked {args.sensor_hz:g} times a second',
            'steps': steps,
            'reference': args.reference,
            'command': 'python ' + ' '.join(Path(a).name if i == 0 else (f'"{a}"' if ' ' in a else a)
                                            for i, a in enumerate(sys.argv)),
            'clock': {'monotonic': time.monotonic(), 'unix': time.time()},
            'ended_by': None,
        }
        meta_path = folder / 'data' / f'{run_id}-meta.json'
        save_meta(meta_path, meta)
        try:
            meta['ended_by'] = _steps(bot, args, log, ina_address, steps)
        finally:
            # Whatever happened, both bucks back on before letting go.
            if bot.connected:
                _restore(bot)
            bot.close()      # motors off
            log.close()
    finally:
        if bot.connected:
            bot.close()
    save_meta(meta_path, meta)
    print('Done. Switch the robot off and on before turning its motors on - the firmware '
          'refuses until it restarts, since its servos lost power.')
    return meta_path


def _steps(bot, args, log, ina_address, steps):
    '''The run itself. Returns how it ended: 'done', 'ctrl_c' or 'link'.'''
    start = time.monotonic()
    ends = np.cumsum([s['settle_s'] + s['measure_s'] for s in steps]) + start
    current = -1
    last_robot_ms = None
    seen = seen_servos = None
    servo_seen_at = None
    next_charger = start
    measured = []
    try:
        while True:
            now = time.monotonic()
            if current < 0 or now >= ends[current]:
                if current >= 0 and measured:
                    i, p = np.mean(measured, axis=0)
                    print(f'  {steps[current]["label"]}: {i * 1000:.0f} mA, {p:.2f} W')
                if current >= 0 and steps[current]['name'] == 'servos_off' and servo_seen_at \
                        and servo_seen_at > ends[current] - steps[current]['measure_s']:
                    text = ("Servos still answering with the 6 V buck switched off - the firmware "
                            "doesn't know POWER_RAIL (needs protocol 4)")
                    log.write(now, 'event', text=text)
                    print(text)
                current += 1
                if current == len(steps):
                    log.write(now, 'event', text='Done')
                    return 'done'
                step = steps[current]
                log.write(now, 'phase', text=step['name'])
                log.write(now, 'event', text=step['label'])
                print(step['label'] + '...')
                _switch(bot, step['name'])
                measured = []
                now = time.monotonic()   # readings from here on came after the switch

            history = bot.sensor_history
            new = history
            if seen is not None:
                idx = next((i for i in range(len(history) - 1, -1, -1) if history[i] is seen), None)
                new = history[idx + 1:] if idx is not None else history
            for snap in new:
                seen = snap
                if last_robot_ms is not None and snap.t_ms < last_robot_ms - RESTART_JUMP_MS:
                    log.write(now, 'event', text='Robot restarted')
                    print('The robot restarted - the steps after this are not what they say.')
                last_robot_ms = snap.t_ms
                ina = find_ina237(snap, ina_address)
                if ina is None or not all(math.isfinite(v) for v in ina.values):
                    continue
                bus_v, _shunt_mv, current_ma, power_mw, die_c = ina.values
                log.write(now, 'ina', robot_ms=snap.t_ms, bus_v=f'{bus_v:.4f}',
                          current_a=f'{current_ma / 1000:.5f}', power_w=f'{power_mw / 1000:.4f}',
                          die_c=f'{die_c:.2f}')
                if now >= ends[current] - steps[current]['measure_s']:
                    measured.append((current_ma / 1000, power_mw / 1000))

            servos = bot.telemetry
            if servos is not None and servos is not seen_servos:
                seen_servos = servos
                servo_seen_at = now
                volts, errors = servos.get('voltage'), servos.get('hw_error')
                log.write(now, 'servo', servo_c=' '.join(str(c) for c in servos['temperature']),
                          servo_ma=' '.join(str(c) for c in servos['current']),
                          servo_v=' '.join(f'{v:.1f}' for v in volts) if volts else '',
                          servo_err=' '.join(str(e) for e in errors) if errors else '')

            if now >= next_charger:
                bot.request_battery()
                next_charger = now + args.charger_every
            pct = bot.take_battery_reading()
            if pct is not None and bot.battery_volts is not None:
                log.write(now, 'charger', percent=pct, volts=f'{bot.battery_volts:.3f}')

            if not bot.connected:
                log.write(now, 'event', text='Lost the link to the controller')
                return 'link'
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        log.write(time.monotonic(), 'event', text='Stopped with Ctrl+C - both bucks back on')
        print('Stopped with Ctrl+C.')
        return 'ctrl_c'


# ---------- log -> numbers ----------

def load(folder, run_id):
    '''The run from its log: readings on one clock, 0 = the first step's start.'''
    meta = json.loads((folder / 'data' / f'{run_id}-meta.json').read_text(encoding='utf-8'))
    rows = list(csv.DictReader(open(folder / 'data' / f'{run_id}-log.csv', encoding='utf-8')))
    phase_at = {r['text']: float(r['host_s']) for r in rows if r['kind'] == 'phase'}
    ina = [r for r in rows if r['kind'] == 'ina']
    if not phase_at or len(ina) < 10:
        return None
    # When each reading arrived, on the host's clock - not the robot's clock
    # pinned to it, as the long runs do: a reading that arrived inside a
    # step's window was certainly sampled after its settling began and
    # before the next switch went out, which is what the windows need.
    zero = min(phase_at.values())
    col = lambda name: np.array([float(r[name]) for r in ina])
    run = {'meta': meta, 't': col('host_s') - zero, 'bus_v': col('bus_v'), 'current_a': col('current_a'),
           'power_w': col('power_w')}

    # Each step: (start, end) and its measured window, in seconds from zero.
    end_all = max(float(run['t'][-1]), max(float(r['host_s']) for r in rows) - zero)
    steps = [s for s in meta['steps'] if s['name'] in phase_at]
    run['steps'] = {}
    for k, s in enumerate(steps):
        a = phase_at[s['name']] - zero
        b = phase_at[steps[k + 1]['name']] - zero if k + 1 < len(steps) else min(end_all, a + s['settle_s'] + s['measure_s'])
        run['steps'][s['name']] = {'label': s['label'], 'span': (a, b), 'window': (min(a + s['settle_s'], b), b)}
    run['charger'] = [(float(r['host_s']) - zero, int(r['percent']), float(r['volts']))
                      for r in rows if r['kind'] == 'charger']
    run['events'] = sorted(((float(r['host_s']) - zero, r['text']) for r in rows if r['kind'] == 'event'),
                           key=lambda e: e[0])
    servo = [r for r in rows if r['kind'] == 'servo']
    run['servo_t'] = np.array([float(r['host_s']) - zero for r in servo]) if servo else np.array([])
    return run


def summarize(run, ref):
    t = run['t']
    s = {'steps': {}, 'ref': ref}
    for name, st in run['steps'].items():
        a, b = st['window']
        m = (t >= a) & (t < b)
        if np.count_nonzero(m) < 5:
            s['steps'][name] = None
            continue
        tm = t[m]
        span = float(tm[-1] - tm[0])
        i, p = run['current_a'][m], run['power_w'][m]
        s['steps'][name] = {
            'ma': np.trapezoid(i, tm) / span * 1000, 'w': np.trapezoid(p, tm) / span,
            'v': float(np.mean(run['bus_v'][m])), 'n': int(np.count_nonzero(m)), 'measured_s': span,
            'p10_ma': float(np.percentile(i, 10)) * 1000, 'p90_ma': float(np.percentile(i, 90)) * 1000,
            'servo_readings': int(np.count_nonzero((run['servo_t'] >= a) & (run['servo_t'] < b))),
        }
    got = s['steps']
    s['shares'] = []
    for label, lower, upper in SHARES:
        if got.get(upper) is None or (lower and got.get(lower) is None):
            continue
        lo_w = got[lower]['w'] if lower else 0.0
        lo_ma = got[lower]['ma'] if lower else 0.0
        s['shares'].append({'label': label, 'w': got[upper]['w'] - lo_w, 'ma': got[upper]['ma'] - lo_ma})
    s['readings'] = len(t)
    s['rate_hz'] = (len(t) - 1) / (t[-1] - t[0])
    s['longest_gap_s'] = float(np.max(np.diff(t)))
    return s


# ---------- the report ----------

def _title(meta):
    where = '' if meta['setup'] == 'not recorded' else f", {meta['setup']}"
    return f"{meta['run_id']} - idle power{where}"


def write_markdown(run, s, path, png, files):
    meta, ref, got = run['meta'], s['ref'], s['steps']
    lines = []
    add = lines.append
    add(f'# {_title(meta)}')
    add('')
    off, bare = got.get('motors_off'), got.get('bucks_off')
    head = []
    if off:
        head.append(f"**With its motors off the robot draws {off['w']:.2f} W - {off['ma']:.0f} mA at "
                    f"{off['v']:.2f} V.**")
    if bare:
        head.append(f"With the status light and both buck converters off too, {bare['w']:.2f} W "
                    f"({bare['ma']:.0f} mA) is left.")
    if ref and off:
        head.append(f"On the {ref['wh']:.2f} Wh that {ref['run_id']}'s pack gave walking, that's about "
                    f"{_duration(ref['wh'] / off['w'])} idling with the motors off"
                    + (f", {_duration(ref['wh'] / bare['w'])} with everything switchable off." if bare else '.'))
    add(' '.join(head))
    add('')

    add('## Each step')
    add('')
    add('| | Current | Power | Less than the step before |' + (' On one charge |' if ref else ''))
    add('|---|---|---|---|' + ('---|' if ref else ''))
    before = None
    for name, label, _ in STEPS:
        g = got.get(name)
        if g is None:
            if name in run['steps']:
                add(f'| {label} | no readings | | |' + (' |' if ref else ''))
            continue
        less = ''
        if before and name != 'restored':
            less = f"{before['ma'] - g['ma']:.0f} mA, {before['w'] - g['w']:.2f} W"
        row = f"| {label} | {g['ma']:.0f} mA | {g['w']:.2f} W | {less} |"
        if ref:
            row += f" {_duration(ref['wh'] / g['w'])} |" if name != 'restored' else ' |'
        add(row)
        if name != 'restored':
            before = g
    add('')
    m = meta['steps'][1] if len(meta['steps']) > 1 else meta['steps'][0]
    add(f"Each step is averaged over its last {m['measure_s']:g} s, after {m['settle_s']:g} s settling "
        f"({meta['steps'][0]['settle_s']:g} s after standing up)."
        + (f" On one charge: {ref['run_id']}'s {ref['wh']:.2f} Wh at that power - see "
           f"[BAT-ST-001](BAT-ST-001.md) for why energy, and why it's likely on the short side."
           if ref else ''))
    add('')

    add('## Without the sensor board')
    add('')
    add(SENSOR_NOTE + ' So every step is at most that much lower without it:')
    add('')
    add('| | Measured | Without the INA237 board |' + (' On one charge, without it |' if ref else ''))
    add('|---|---|---|' + ('---|' if ref else ''))
    for name, label, _ in STEPS:
        g = got.get(name)
        if g is None or name == 'restored':
            continue
        ma, w = without_sensor(g['ma'], g['w'], g['v'])
        row = f"| {label} | {g['ma']:.1f} mA, {g['w']:.3f} W | {ma:.1f} mA, {w:.3f} W |"
        if ref:
            row += f" {_duration(ref['wh'] / w)} |"
        add(row)
    add('')
    add("All of it comes off what's left with both bucks off - the share it changes most.")
    add('')

    if s['shares']:
        total = sum(sh['w'] for sh in s['shares'])
        add('## Where the standing power goes')
        add('')
        add('| | Power | Current | Share |')
        add('|---|---|---|---|')
        for sh in s['shares']:
            add(f"| {sh['label']} | {sh['w']:.2f} W | {sh['ma']:.0f} mA | {sh['w'] / total * 100:.0f}% |")
        add(f"| **Standing, motors on** | **{total:.2f} W** | | |")
        add('')
        add('Each share is the drop from one step to the next, in the order the parts were switched off.')
        add('')

    if meta.get('observations'):
        add('## Observations')
        add('')
        for note in meta['observations']:
            add(f'- {note}')
        add('')

    add(f"![{meta['run_id']} plots]({png})")
    add('')
    add('## Timeline')
    add('')
    add('| Time | |')
    add('|---|---|')
    for t, text in run['events']:
        add(f'| {t:.0f} s | {text} |')
    add('')
    add('## Run details')
    add('')
    add('| | |')
    add('|---|---|')
    rows = {
        'Started': meta['started'], 'Setup': meta['setup'], 'PCB': meta['pcb'], 'Pack': meta['pack'],
        'Robot firmware': meta['robot_fw'], 'Controller': meta['controller'], 'Sensor': meta['sensor'],
        'Battery': (f"{_charger(run['charger'][0])} at the start, {_charger(run['charger'][-1])} at the end "
                    '(charger)' if run['charger'] else 'no charger reading'),
        'Reference run': f"[{ref['run_id']}]({ref['run_id']}.md)" if ref else 'none',
        'Command': f"`{meta['command']}`",
    }
    if meta.get('note'):
        rows['Note'] = meta['note']
    for k, v in rows.items():
        add(f'| {k} | {v} |')
    add(f"| INA237 readings | {s['readings']} ({s['rate_hz']:.1f} per second), longest pause "
        f"{s['longest_gap_s']:.1f} s |")
    short = {name: lab for name, _, lab in STEPS}
    servo_counts = ', '.join(f"{short[n]} {g['servo_readings']}" for n, g in got.items() if g)
    add(f'| Servo readings in each measured stretch | {servo_counts} |')
    add('')
    add('Times are seconds from the start. Averages are the INA237 readings integrated over time. '
        'Readings: ' + ', '.join(f'[{f}]({f})' for f in files) + '.')
    add('')
    path.write_text('\n'.join(lines), encoding='utf-8')


def write_csvs(run, ina_path, charger_path):
    def step_of(t):
        return next((n for n, st in run['steps'].items() if st['span'][0] <= t <= st['span'][1]), '')

    def measured(t):
        return any(st['window'][0] <= t < st['window'][1] for st in run['steps'].values())
    with open(ina_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'step', 'measured', 'bus_v', 'current_ma', 'power_mw'])
        for k, t in enumerate(run['t']):
            w.writerow([f'{t:.3f}', step_of(t), int(measured(t)), f"{run['bus_v'][k]:.3f}",
                        f"{run['current_a'][k] * 1000:.1f}", f"{run['power_w'][k] * 1000:.0f}"])
    with open(charger_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'step', 'percent', 'volts'])
        for t, pct, v in run['charger']:
            w.writerow([f'{t:.3f}', step_of(t), pct, f'{v:.3f}'])


def plot(run, s, path):
    t = run['t']
    got = s['steps']
    height = 1.4 + 2.2 * 3 + 2.0
    fig = plt.figure(figsize=(11, height))
    gs = fig.add_gridspec(5, 1, height_ratios=[2.2, 2.2, 2.2, 0.45, 1.6], hspace=0.14)
    axes = [fig.add_subplot(gs[0])]
    axes += [fig.add_subplot(gs[k], sharex=axes[0]) for k in (1, 2)]
    bar = fig.add_subplot(gs[4])          # gs[3]: room for the voltage panel's x label
    fig.subplots_adjust(top=1 - 1.45 / height, bottom=0.25 / height, left=0.08, right=0.95)
    gs.update(hspace=0.14)

    meta = run['meta']
    off, bare = got.get('motors_off'), got.get('bucks_off')
    sub = []
    if got.get('standing'):
        sub.append(f"standing {got['standing']['w']:.2f} W")
    if off:
        sub.append(f"motors off {off['w']:.2f} W")
    if bare:
        sub.append(f"both bucks off {bare['w']:.2f} W")
    fig.text(0.08, 1 - 0.52 / height, _title(meta), fontsize=15, weight='bold')
    fig.text(0.08, 1 - 0.8 / height, '  ·  '.join(sub), fontsize=10.5, color='#3b434c')

    short = {name: lab for name, _, lab in STEPS}
    for ax in axes:
        for name, st in run['steps'].items():
            ax.axvspan(st['span'][0], st['window'][0], color=COLORS['shade'], zorder=0, lw=0)
            ax.axvline(st['span'][0], color=COLORS['event'], lw=0.6, ls=':', zorder=1)
        ax.grid(True, color='#dde2e7', lw=0.6)
        ax.spines[['top', 'right']].set_visible(False)
    for ax in axes[:2]:
        plt.setp(ax.get_xticklabels(), visible=False)
    for name, st in run['steps'].items():
        axes[0].text(sum(st['span']) / 2, 1.03, short[name], transform=axes[0].get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=8.5, color='#1d2329')

    smooth = lambda y: _rolling_mean(t, y, SMOOTH_S)

    def panel(ax, y, color, unit, fmt):
        for name, st in run['steps'].items():
            m = (t >= st['window'][0]) & (t < st['window'][1])
            if np.count_nonzero(m) >= 3:
                c, lo, hi = _band(t[m], y[m], 2.0)
                if len(c):
                    ax.fill_between(c, lo, hi, color=color, alpha=0.16, lw=0)
        ax.plot(t, smooth(y), color=color, lw=1.6)
        for name, g in got.items():
            if g is None:
                continue
            a, b = run['steps'][name]['window']
            value = g[unit]
            ax.hlines(value if unit != 'ma' else value, a, b, color=color, lw=0.9, ls='--')
            ax.annotate(fmt(value), ((a + b) / 2, value), xytext=(0, 6), textcoords='offset points',
                        ha='center', va='bottom', fontsize=8.5, color='#1d2329')

    panel(axes[0], run['current_a'] * 1000, COLORS['current'], 'ma', lambda v: f'{v:.0f} mA')
    axes[0].set_ylabel('Current (mA)')
    axes[0].set_ylim(0, float(np.max(smooth(run['current_a'] * 1000))) * 1.25)
    panel(axes[1], run['power_w'], COLORS['power'], 'w', lambda v: f'{v:.2f} W')
    axes[1].set_ylabel('Power (W)')
    axes[1].set_ylim(0, float(np.max(smooth(run['power_w']))) * 1.25)

    ax = axes[2]
    ax.plot(t, smooth(run['bus_v']), color=COLORS['voltage'], lw=1.6, label='INA237')
    if run['charger']:
        ax.plot([r[0] for r in run['charger']], [r[2] for r in run['charger']], 'o', ms=4,
                color=COLORS['charger'], label='charger')
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo - 0.3 * (hi - lo), hi + 0.3 * (hi - lo))
    ax.set_ylabel('Voltage (V)')
    ax.legend(loc='upper right', fontsize=8.5, frameon=False, ncol=2)
    ax.set_xlabel('Seconds from the start  ·  shaded: settling, not measured', fontsize=9)

    # Where the standing power goes: one stacked bar, base first, a 2 px
    # gap between segments, each labeled - its colors alone don't name it.
    shares = [sh for sh in s['shares'] if sh['w'] > 0]
    left = 0.0
    total = sum(sh['w'] for sh in shares) or 1.0
    for k, sh in enumerate(shares):
        color = SHARE_COLORS[[x[0] for x in SHARES].index(sh['label'])]
        bar.barh(0, sh['w'], left=left, height=0.5, color=color, edgecolor='white', linewidth=2,
                 label=f"{sh['label']} - {sh['w']:.2f} W")
        if sh['w'] / total > 0.07:
            bar.text(left + sh['w'] / 2, 0, f"{sh['w']:.2f} W", ha='center', va='center',
                     fontsize=8.5, color='#1d2329')
        left += sh['w']
    bar.set_xlim(0, total * 1.02)
    bar.set_ylim(-0.45, 0.45)
    bar.set_yticks([])
    bar.set_xlabel('Where the standing power goes (W)', fontsize=9)
    bar.spines[['top', 'right', 'left']].set_visible(False)
    bar.legend(loc='upper center', bbox_to_anchor=(0.5, -0.5), fontsize=8, frameon=False, ncol=2)

    fig.savefig(path, dpi=130, bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)


def add_to_readme(folder, run, s):
    readme = folder / 'README.md'
    if not readme.exists():
        return False
    text = readme.read_text(encoding='utf-8')
    if README_MARKER not in text:
        return False
    meta, got = run['meta'], s['steps']
    fmt = lambda n: f"{got[n]['w']:.2f} W" if got.get(n) else '-'
    row = (f"| [{meta['run_id']}]({meta['run_id']}.md) | {meta['started'][:10]} | {meta['setup']} | "
           f"{fmt('standing')} | {fmt('motors_off')} | {fmt('light_off')} | {fmt('servos_off')} | "
           f"{fmt('bucks_off')} |")
    lines = text.split('\n')
    mine = [k for k, line in enumerate(lines) if line.startswith(f"| [{meta['run_id']}]")]
    if mine:
        lines[mine[0]] = row
    else:
        lines.insert(lines.index(next(l for l in lines if README_MARKER in l)), row)
    readme.write_text('\n'.join(lines), encoding='utf-8')
    return True


def report_from(folder, run_id):
    run = load(folder, run_id)
    if run is None:
        print(f'{run_id} has too few readings for a report - its log is in {folder / "data"}')
        return None
    s = summarize(run, reference(folder, run['meta'].get('reference')))
    (folder / 'images').mkdir(parents=True, exist_ok=True)
    png = f'images/{run_id}-plots.png'
    files = [f'data/{run_id}-ina237.csv', f'data/{run_id}-charger.csv']
    plot(run, s, folder / png)
    write_csvs(run, folder / files[0], folder / files[1])
    report = folder / f'{run_id}.md'
    write_markdown(run, s, report, png, files)
    add_to_readme(folder, run, s)
    return report


def main():
    args = parse_args()
    folder = args.out
    if not folder.parent.exists():
        sys.exit(f"{folder.parent} doesn't exist - is q8botone-docs next to this repository? "
                 'Or pass --out.')
    if args.from_log:
        path = report_from(folder, args.from_log)
    else:
        folder.mkdir(exist_ok=True)
        run_id = next_run_id(folder)
        if record(args, folder, run_id) is None:
            return
        path = report_from(folder, run_id)
    if path:
        print(f'Report: {path}')


if __name__ == '__main__':
    main()
