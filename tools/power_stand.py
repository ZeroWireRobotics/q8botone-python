'''
Standing power test: a minute of the robot standing with its motors on,
logging the INA237 on its Qwiic port - and from that, how long it could
stand on one charge. The quick version of the battery runtime test
(power_run.py); the report goes into q8botone-docs
(docs/tests/battery-runtime, next to this repository).

    python tools/power_stand.py --pcb PROTO-002 --pack "2S 1000 mAh"

Needs matplotlib: pip install -e ".[tools]"

Before starting:
  - the INA237 on the robot's Qwiic port, in the battery path
  - the robot's USB unplugged - otherwise the charger feeds it, and the
    INA237 sees less than the robot uses
  - the controller plugged into this computer
  - the robot somewhere it can stand up
Any firmware will do - the robot only stands.

A run: motors off for a few seconds (the rested voltage and the idle draw),
motors on and standing until the draw settles, a minute of standing - the
measurement - then motors off again for a few seconds.

The runtime estimate sets the measured minute's average power against the
energy a walking run got out of its pack (--reference, BAT-RT-001 by
default), and its average current against that run's charge.

Ctrl+C ends the run early (motors off) and still writes the report. Every
reading is logged as it arrives (data/<run id>-log.csv, with
data/<run id>-meta.json); `--from-log BAT-ST-001` rebuilds a report from
those alone.
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

import power_run                                            # noqa: E402  (puts the repo on the path)
from power_run import Log, ask, find_ina237, save_meta, RESTART_JUMP_MS, POLL_S, DEFAULT_OUT   # noqa: E402
import power_report                                         # noqa: E402
from power_report import COLORS, LEGS, LEG_COLORS, _band, _clock, _cumulative, _rolling_mean   # noqa: E402
from q8bot import Q8Bot                                     # noqa: E402
from q8bot.firmware_flasher import identify_device          # noqa: E402
from q8bot.helpers import XiaoPortFinder                    # noqa: E402
from q8bot.protocol import describe_servo_error             # noqa: E402

import matplotlib.pyplot as plt                             # noqa: E402  (power_report picked Agg)

RUN_PREFIX = 'BAT-ST-'
README_MARKER = '<!-- power_stand.py adds each run above this line -->'

# Settled: the last SETTLE_WINDOW_S of current within SETTLE_SHARE (or
# SETTLE_A, whichever is more) of the SETTLE_WINDOW_S before it.
SETTLE_WINDOW_S = 5.0
SETTLE_SHARE = 0.02
SETTLE_A = 0.005
PROGRESS_EVERY_S = 15

SMOOTH_S = 2.0         # rolling average for the plotted lines - the run is only a minute or two
PEAK_WINDOW_S = 1.0
EDGE_S = 10.0          # "first / last 10 s" of the measured minute, and of the reference walk
GAP_S = 0.5

# The Adafruit INA237 board's own draw. It runs from the robot's Qwiic
# 3.3 V, downstream of its own shunt, so every reading includes it: the
# INA237 640 uA typical (TI datasheet), the board's green "on" LED about
# 0.1 mA (10k from 3.3 V), its 10k I2C pull-ups next to nothing (only while a
# line is low); there's no regulator (Adafruit's schematic, INA237 Rev A).
# From the battery that's at most this much - less if the robot makes its
# 3.3 V with a buck rather than linearly.
SENSOR_SELF_MA = 0.75
# For scale: the INA237's offset is up to +-50 uV (datasheet maximum) - on
# the board's 15 mOhm shunt, +-3.3 mA.
SENSOR_OFFSET_MA = 3.3
SENSOR_NOTE = (f"The INA237 board measures its own supply too: it runs from the robot's Qwiic 3.3 V, "
               f"downstream of its shunt, and draws about {SENSOR_SELF_MA:g} mA - the INA237 0.64 mA "
               f"(typical, TI's datasheet) and the board's \"on\" LED about 0.1 mA (Adafruit's "
               f"schematic: 10 kΩ from 3.3 V; no regulator). From the battery that's at most "
               f"{SENSOR_SELF_MA:g} mA, less if the robot makes its 3.3 V with a buck. It's smaller "
               f"than the INA237's own worst-case offset, ±50 µV on the board's 15 mΩ shunt: "
               f"±{SENSOR_OFFSET_MA:g} mA.")


def without_sensor(ma, w, v):
    '''A reading less the INA237 board's own draw: (mA, W), at most.'''
    return ma - SENSOR_SELF_MA, w - SENSOR_SELF_MA / 1000 * v

PHASE_NAMES = {
    'rest_start': 'Motors off, before',
    'settling': 'Settling, motors on',
    'standing': 'Standing, motors on (measured)',
    'rest_end': 'Motors off, after',
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', help="the controller's port (found by asking each board if left out)")
    p.add_argument('--setup', default='not recorded',
                   help='where it stands, e.g. "on the floor" or "on a test stand"')
    p.add_argument('--pcb', default='not recorded', help='PCB revision / serial')
    p.add_argument('--pack', default='not recorded', help='battery pack')
    p.add_argument('--robot-fw', default='not recorded',
                   help="the robot's firmware (it can't report it over the radio)")
    p.add_argument('--note', default='', help='anything else worth keeping with the run')
    p.add_argument('--out', type=Path, default=DEFAULT_OUT, help='test folder (default: %(default)s)')
    p.add_argument('--measure-s', type=float, default=60.0, help='standing measured, s (default: %(default)s)')
    p.add_argument('--rest-s', type=float, default=8.0, help='motors off at the start, s')
    p.add_argument('--settle-min', type=float, default=10.0, help='standing before checking it has settled, s')
    p.add_argument('--settle-max', type=float, default=60.0,
                   help='measure anyway after this long settling, s')
    p.add_argument('--rest-end-s', type=float, default=5.0, help='motors off at the end, s')
    p.add_argument('--reference', default='BAT-RT-001',
                   help='the walking run whose energy and charge the estimate uses (default: %(default)s)')
    p.add_argument('--sensor-hz', type=float, default=10.0)
    p.add_argument('--servo-hz', type=float, default=1.0,
                   help='servo readings per second, 0 for none')
    p.add_argument('--charger-every', type=float, default=5.0, help='seconds between charger readings')
    p.add_argument('--silent-after', type=float, default=5.0, help='no reply for this long ends the run')
    p.add_argument('--ina-address', type=lambda s: int(s, 0), help='which INA237, if there are several')
    p.add_argument('--yes', action='store_true', help="don't ask before starting")
    p.add_argument('--from-log', metavar='RUN_ID', help="rebuild that run's report from its log")
    return p.parse_args()


def next_run_id(folder):
    used = [int(m.group(1)) for f in (folder / 'data').glob(f'{RUN_PREFIX}*')
            if (m := re.match(re.escape(RUN_PREFIX) + r'(\d+)', f.name))]
    return f'{RUN_PREFIX}{max(used, default=0) + 1:03d}'


# ---------- the run ----------

def settled(samples, now):
    '''Whether the current (samples: (host s, A)) has stopped moving.'''
    last = [a for t, a in samples if t > now - SETTLE_WINDOW_S]
    before = [a for t, a in samples if now - 2 * SETTLE_WINDOW_S < t <= now - SETTLE_WINDOW_S]
    if len(last) < 10 or len(before) < 10:
        return False
    a, b = np.mean(last), np.mean(before)
    return abs(a - b) <= max(SETTLE_SHARE * b, SETTLE_A)


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

        print(f'{run_id}: controller on {port}, robot answering at {bot.battery_volts:.2f} V '
              f'({bot.battery_percent}%), INA237 at {ina_address:#04x}.')
        print(f'This turns the motors on and stands the robot for about '
              f'{args.rest_s + args.settle_min + args.measure_s + args.rest_end_s:.0f} s or more. Check:\n'
              '  - it has room to stand up\n'
              '  - its USB is unplugged (or the charger feeds it)')
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
            'servos': ('temperature and current of all eight, asked '
                       + ('once a second' if args.servo_hz == 1 else f'{args.servo_hz:g} times a second')
                       if args.servo_hz else 'not read'),
            'settle': (f'at least {args.settle_min:g} s, then until the last {SETTLE_WINDOW_S:g} s average '
                       f'current is within {SETTLE_SHARE * 100:g}% (or {SETTLE_A * 1000:g} mA) of the '
                       f'{SETTLE_WINDOW_S:g} s before it; at most {args.settle_max:g} s'),
            'measure_s': args.measure_s,
            'reference': args.reference,
            'command': 'python ' + ' '.join(Path(a).name if i == 0 else (f'"{a}"' if ' ' in a else a)
                                            for i, a in enumerate(sys.argv)),
            'clock': {'monotonic': time.monotonic(), 'unix': time.time()},
            'ended_by': None,
        }
        meta_path = folder / 'data' / f'{run_id}-meta.json'
        save_meta(meta_path, meta)
        try:
            meta['ended_by'] = _stand(bot, args, log, ina_address)
        finally:
            bot.close()      # motors off
            log.close()
    finally:
        if bot.connected:
            bot.close()
    save_meta(meta_path, meta)
    return meta_path


def _stand(bot, args, log, ina_address):
    '''The run itself. Returns how it ended: 'done', 'ctrl_c', 'silent' or 'link'.'''
    start = time.monotonic()
    phase, phase_at = 'rest_start', start
    last_reply = start
    last_robot_ms = None
    seen = seen_servos = None
    next_charger = start
    next_progress = None
    since_on = []            # (host s, A) from the motors going on - for settled()
    measured = []            # likewise, the measured minute - for the progress line

    def enter(name, now, event):
        nonlocal phase, phase_at
        phase, phase_at = name, now
        log.write(now, 'phase', text=name)
        log.write(now, 'event', text=event)
        print(event + '.')

    log.write(start, 'phase', text=phase)
    log.write(start, 'event', text='Connected, motors off')
    bot.set_torque(False)
    try:
        while True:
            now = time.monotonic()

            if phase == 'rest_start' and now - phase_at >= args.rest_s:
                enter('settling', now, 'Motors on, standing - waiting for the draw to settle')
                bot.set_torque(True)
            elif phase == 'settling':
                waited = now - phase_at
                if waited >= args.settle_min and settled(since_on, now):
                    enter('standing', now, f'Settled after {waited:.0f} s - measuring '
                                           f'{args.measure_s:g} s of standing')
                    next_progress = now + PROGRESS_EVERY_S
                elif waited >= args.settle_max:
                    enter('standing', now, f'Not settled after {args.settle_max:g} s - measuring anyway')
                    next_progress = now + PROGRESS_EVERY_S
            elif phase == 'standing' and now - phase_at >= args.measure_s:
                enter('rest_end', now, 'Measured - motors off')
                bot.set_torque(False)
            elif phase == 'rest_end' and now - phase_at >= args.rest_end_s:
                log.write(now, 'event', text='Done')
                return 'done'

            # Sensor replies - every one in the history since the last poll.
            history = bot.sensor_history
            new = history
            if seen is not None:
                idx = next((i for i in range(len(history) - 1, -1, -1) if history[i] is seen), None)
                new = history[idx + 1:] if idx is not None else history
            for snap in new:
                seen = snap
                last_reply = now
                if last_robot_ms is not None and snap.t_ms < last_robot_ms - RESTART_JUMP_MS:
                    log.write(now, 'event', text='Robot restarted - motors back on')
                    print('Robot restarted - motors back on.')
                    if phase in ('settling', 'standing'):
                        bot.set_torque(True)
                last_robot_ms = snap.t_ms
                ina = find_ina237(snap, ina_address)
                if ina is None or not all(math.isfinite(v) for v in ina.values):
                    continue
                bus_v, _shunt_mv, current_ma, power_mw, die_c = ina.values
                log.write(now, 'ina', robot_ms=snap.t_ms, bus_v=f'{bus_v:.4f}',
                          current_a=f'{current_ma / 1000:.5f}', power_w=f'{power_mw / 1000:.4f}',
                          die_c=f'{die_c:.2f}')
                if phase in ('settling', 'standing'):
                    since_on.append((now, current_ma / 1000))
                if phase == 'standing':
                    measured.append((current_ma / 1000, power_mw / 1000, bus_v))

            servos = bot.telemetry
            if servos is not None and servos is not seen_servos:
                seen_servos = servos
                last_reply = now
                volts, errors = servos.get('voltage'), servos.get('hw_error')
                log.write(now, 'servo', servo_c=' '.join(str(c) for c in servos['temperature']),
                          servo_ma=' '.join(str(c) for c in servos['current']),
                          servo_v=' '.join(f'{v:.1f}' for v in volts) if volts else '',
                          servo_err=' '.join(str(e) for e in errors) if errors else '')

            if now >= next_charger:
                bot.request_battery()
                next_charger = now + args.charger_every
            pct = bot.take_battery_reading()
            if pct is not None:
                last_reply = now
                if bot.battery_volts is not None:
                    log.write(now, 'charger', percent=pct, volts=f'{bot.battery_volts:.3f}')

            if phase == 'standing' and next_progress and now >= next_progress and measured:
                next_progress = now + PROGRESS_EVERY_S
                i, p, v = np.mean(measured, axis=0)
                print(f'  {now - phase_at:3.0f} s standing   average so far {i * 1000:.0f} mA   '
                      f'{p:.2f} W   {v:.2f} V')

            if not bot.connected:
                log.write(now, 'event', text='Lost the link to the controller')
                return 'link'
            if now - last_reply > args.silent_after:
                log.write(last_reply, 'event', text='Robot stopped answering')
                print('The robot stopped answering.')
                return 'silent'
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        log.write(time.monotonic(), 'event', text='Stopped with Ctrl+C')
        print('Stopped with Ctrl+C.')
        return 'ctrl_c'


# ---------- log -> numbers ----------

def load(folder, run_id):
    '''The run from its log: readings on one clock, 0 = the measured minute's start.'''
    meta = json.loads((folder / 'data' / f'{run_id}-meta.json').read_text(encoding='utf-8'))
    rows = list(csv.DictReader(open(folder / 'data' / f'{run_id}-log.csv', encoding='utf-8')))
    phase_at = {r['text']: float(r['host_s']) for r in rows if r['kind'] == 'phase'}
    if 'standing' not in phase_at:
        return None
    ina = [r for r in rows if r['kind'] == 'ina']
    host = np.array([float(r['host_s']) for r in ina])
    robot = np.array([float(r['robot_ms']) / 1000 for r in ina])
    # Spaced by the robot's clock, pinned to the host's - as power_run.build_run.
    t = np.empty_like(host)
    starts = [0] + [i for i in range(1, len(robot)) if robot[i] < robot[i - 1] - RESTART_JUMP_MS / 1000]
    for a, b in zip(starts, starts[1:] + [len(robot)]):
        t[a:b] = robot[a:b] + np.min(host[a:b] - robot[a:b])
    zero = phase_at['standing']
    t -= zero
    order = np.argsort(t, kind='stable')
    col = lambda name: np.array([float(r[name]) for r in ina])[order]

    run = {'meta': meta, 't': t[order], 'bus_v': col('bus_v'), 'current_a': col('current_a'),
           'power_w': col('power_w'), 'die_c': col('die_c')}
    names = [n for n in PHASE_NAMES if n in phase_at]
    bounds = [phase_at[n] - zero for n in names] + [float(run['t'][-1])]
    run['phases'] = {n: (bounds[i], bounds[i + 1]) for i, n in enumerate(names)}
    run['charger'] = [(float(r['host_s']) - zero, int(r['percent']), float(r['volts']))
                      for r in rows if r['kind'] == 'charger']
    run['events'] = sorted(((float(r['host_s']) - zero, r['text']) for r in rows if r['kind'] == 'event'),
                           key=lambda e: e[0])
    servo = [r for r in rows if r['kind'] == 'servo']
    run['servo_t'] = np.array([float(r['host_s']) - zero for r in servo]) if servo else None
    run['servo_c'] = np.array([[int(c) for c in r['servo_c'].split()] for r in servo]) if servo else None
    run['servo_ma'] = np.array([[int(c) for c in r['servo_ma'].split()] for r in servo]) if servo else None
    health = servo and all(r.get('servo_v') for r in servo)
    run['servo_v'] = np.array([[float(v) for v in r['servo_v'].split()] for r in servo]) if health else None
    run['servo_err'] = np.array([[int(e) for e in r['servo_err'].split()] for r in servo]) if health else None
    return run


def _mask(run, phase):
    a, b = run['phases'][phase]
    return (run['t'] >= a) & (run['t'] <= b)


def _phase_means(run):
    out = {}
    for name in run['phases']:
        m = _mask(run, name)
        if np.count_nonzero(m) >= 3:
            out[name] = {'ma': float(np.mean(run['current_a'][m])) * 1000,
                         'w': float(np.mean(run['power_w'][m])),
                         'v': float(np.mean(run['bus_v'][m]))}
    return out


def reference(folder, run_id):
    '''What a walking run (power_run.py) got out of its pack, until its motors stopped.'''
    meta_path = folder / 'data' / f'{run_id}-meta.json'
    if not run_id or not meta_path.exists():
        return None
    run = power_run.build_run(folder / 'data' / f'{run_id}-log.csv',
                              json.loads(meta_path.read_text(encoding='utf-8')))
    if run is None:
        return None
    s = power_report.summarize(run)
    # Early and late in the walk, clear of its start and end: 10-20% and 80-90% in.
    w0, w1 = run.phases['walking']
    span = w1 - w0
    first = (run.t >= w0 + 0.1 * span) & (run.t <= w0 + 0.2 * span)
    last = (run.t >= w0 + 0.8 * span) & (run.t <= w0 + 0.9 * span)
    return {
        'early': (0.1 * span, 0.2 * span), 'late': (0.8 * span, 0.9 * span),
        'run_id': run_id, 'wh': s['energy_wh'], 'mah': s['charge_mah'], 'walk_s': s['walk_s'],
        'avg_ma': s['avg_ma'], 'avg_w': s['avg_w'], 'setup': run.setup,
        'standing_s': run.phases['standing'][1] - run.phases['standing'][0],
        'standing_ma': s['standing_ma'],
        'standing_w': float(np.mean(run.power_w[power_report._mask(run, 'standing')])),
        'start': s['charger_start'],                    # (t, %, V) rested, or None
        'end_v': s['motor_stop']['v'] if s['motor_stop'] else s['v_last'],
        'motors_stopped': s['motor_stop'] is not None,
        'i_first': float(np.mean(run.current_a[first])), 'i_last': float(np.mean(run.current_a[last])),
        'w_first': float(np.mean(run.power_w[first])), 'w_last': float(np.mean(run.power_w[last])),
    }


def _first_error(errors):
    faulted = np.nonzero(errors)[0]
    return describe_servo_error(int(errors[faulted[0]])) if len(faulted) else None


def summarize(run, ref):
    st = _mask(run, 'standing')
    t, i, p, v = run['t'][st], run['current_a'][st], run['power_w'][st], run['bus_v'][st]
    a, b = run['phases']['standing']
    span = float(t[-1] - t[0])
    s = {
        'measured_s': b - a,
        'settle_s': run['phases']['settling'][1] - run['phases']['settling'][0],
        'avg_ma': np.trapezoid(i, t) / span * 1000,
        'avg_w': np.trapezoid(p, t) / span,
        'avg_v': float(np.mean(v)),
        'median_ma': float(np.median(i)) * 1000,
        'p10_ma': float(np.percentile(i, 10)) * 1000,
        'p90_ma': float(np.percentile(i, 90)) * 1000,
        'peak_ma': float(np.max(i)) * 1000,
        'peak_1s_ma': float(np.max(_rolling_mean(t, i, PEAK_WINDOW_S))) * 1000,
        'first_ma': float(np.mean(i[t <= t[0] + EDGE_S])) * 1000,
        'last_ma': float(np.mean(i[t >= t[-1] - EDGE_S])) * 1000,
        'mah': np.trapezoid(i, t) / 3.6,
        'wh': np.trapezoid(p, t) / 3600,
        'phases': _phase_means(run),
        'charger_first': run['charger'][0] if run['charger'] else None,
        'charger_last': run['charger'][-1] if run['charger'] else None,
        'die_start': float(run['die_c'][0]), 'die_max': float(np.max(run['die_c'])),
        'readings': len(run['t']),
        'rate_hz': (len(run['t']) - 1) / (run['t'][-1] - run['t'][0]),
    }
    gaps = np.diff(run['t'])
    s['gaps'] = int(np.sum(gaps > GAP_S))
    s['longest_gap_s'] = float(np.max(gaps))
    s['ref'] = ref
    if ref:
        s['hours_energy'] = ref['wh'] / s['avg_w']
        s['hours_charge'] = ref['mah'] / s['avg_ma']
    s['hours_per_wh'] = 1 / s['avg_w']

    s['servos'] = None
    if run['servo_t'] is not None:
        on = (run['servo_t'] >= a) & (run['servo_t'] <= b)
        if np.count_nonzero(on):
            s['servos'] = [{'ma': float(np.mean(np.abs(run['servo_ma'][on, k]))),
                            'c_first': int(run['servo_c'][on, k][0]), 'c_last': int(run['servo_c'][on, k][-1]),
                            'v_min': float(run['servo_v'][on, k].min()) if run['servo_v'] is not None else None,
                            'error': _first_error(run['servo_err'][on, k]) if run['servo_err'] is not None else None}
                           for k in range(run['servo_c'].shape[1])]
            s['servo_readings'] = len(run['servo_t'])
    return s


# ---------- formatting ----------

def _duration(hours):
    minutes = int(round(hours * 60))
    return f'{minutes // 60} h {minutes % 60:02d} min' if minutes >= 60 else f'{minutes} min'


def _title(meta):
    where = '' if meta['setup'] == 'not recorded' else f", {meta['setup']}"
    return f"{meta['run_id']} - standing power{where}"


def _charger(reading):
    return f'{reading[2]:.2f} V, {reading[1]}%' if reading else 'no reading'


def write_markdown(run, s, path, png, files):
    meta, ref = run['meta'], s['ref']
    ph = s['phases']
    lines = []
    add = lines.append
    add(f"# {_title(meta)}")
    add('')
    head = (f"**Standing with the motors on: {s['avg_w']:.2f} W - {s['avg_ma']:.0f} mA at "
            f"{s['avg_v']:.2f} V, averaged over {s['measured_s']:.0f} s.**")
    if ref:
        head += (f" At that draw the robot would stand for about {_duration(s['hours_energy'])} on the "
                 f"{ref['wh']:.2f} Wh that {ref['run_id']}'s pack gave walking.")
    add(head)
    add('')

    add('## Runtime estimate')
    add('')
    add('| From | Battery | Standing draw | Standing time |')
    add('|---|---|---|---|')
    if ref:
        until = 'until its motors stopped' if ref['motors_stopped'] else 'to its end'
        add(f"| Energy - what {ref['run_id']}'s pack gave walking, {until} | {ref['wh']:.2f} Wh | "
            f"{s['avg_w']:.2f} W | **{_duration(s['hours_energy'])}** |")
        add(f"| Charge - the same run | {ref['mah']:.0f} mAh | {s['avg_ma']:.0f} mA | "
            f"{_duration(s['hours_charge'])} |")
    add(f"| Each Wh of usable battery | 1 Wh | {s['avg_w']:.2f} W | {_duration(s['hours_per_wh'])} |")
    add('')
    if ref:
        early = f"{_clock(ref['early'][0])}-{_clock(ref['early'][1])}"
        late = f"{_clock(ref['late'][0])}-{_clock(ref['late'][1])}"
        add(f"Energy is the better guide. The servos run from a 6 V buck converter, which takes "
            f"about the same power whatever the pack's voltage, so the current rises as the pack "
            f"runs down. Walking in {ref['run_id']}, the current went from "
            f"{ref['i_first']:.2f} A ({early} in) to {ref['i_last']:.2f} A ({late} in) while the power "
            f"went from {ref['w_first']:.1f} W to {ref['w_last']:.1f} W. The {s['avg_ma']:.0f} mA here "
            f"was measured at {s['avg_v']:.2f} V and would rise the same way, so the charge figure "
            f"overstates the time.")
        add('')
        start = f"at {ref['start'][1]}% (charger, rested)" if ref['start'] else 'part-charged'
        stopped = (f"its motors stopped at {ref['end_v']:.2f} V, before the "
                   f"{power_report.SOFT_CUTOFF_V:.2f} V software cutoff" if ref['motors_stopped']
                   else f"it ended at {ref['end_v']:.2f} V")
        add(f"Both are likely on the short side: {ref['run_id']}'s pack started {start} and "
            f"{stopped}, and a pack usually gives a little more energy at a lower current - "
            f"{s['avg_ma']:.0f} mA here, {ref['avg_ma']:.0f} mA walking. Both take this run's pack "
            f"to be {ref['run_id']}'s, or one like it.")
    else:
        add(f"No walking run to compare with ({meta.get('reference') or 'none given'}) - multiply "
            f"a pack's usable energy in Wh by {_duration(s['hours_per_wh'])}.")
    add('')
    ma, w = without_sensor(s['avg_ma'], s['avg_w'], s['avg_v'])
    add(SENSOR_NOTE + f" Without the board the robot would draw {ma:.1f} mA, {w:.3f} W standing"
        + (f" - {_duration(ref['wh'] / w)} on the same energy." if ref else '.'))
    add('')

    add('## Standing, motors on')
    add('')
    add('| | |')
    add('|---|---|')
    add(f"| Measured | {s['measured_s']:.0f} s, after {s['settle_s']:.0f} s settling |")
    add(f"| Average current | {s['avg_ma']:.0f} mA |")
    add(f"| Median current | {s['median_ma']:.0f} mA |")
    add(f"| Middle 80% of readings | {s['p10_ma']:.0f} - {s['p90_ma']:.0f} mA |")
    add(f"| Highest current, 1 s average | {s['peak_1s_ma']:.0f} mA |")
    add(f"| Highest current, single reading | {s['peak_ma']:.0f} mA |")
    add(f"| First / last {EDGE_S:g} s | {s['first_ma']:.0f} mA / {s['last_ma']:.0f} mA |")
    add(f"| Average power | {s['avg_w']:.2f} W |")
    add(f"| Average voltage (INA237) | {s['avg_v']:.2f} V |")
    add(f"| Charge / energy used in the {s['measured_s']:.0f} s | {s['mah']:.1f} mAh / {s['wh'] * 1000:.1f} mWh |")
    add('')

    add('## For comparison')
    add('')
    add('| | Current | Power |')
    add('|---|---|---|')
    for name, label in PHASE_NAMES.items():
        if name in ph:
            add(f"| {label} | {ph[name]['ma']:.0f} mA | {ph[name]['w']:.2f} W |")
    if ref:
        add(f"| Standing, {ref['run_id']} ({ref['standing_s']:.0f} s, {ref['setup']}) | "
            f"{ref['standing_ma']:.0f} mA | {ref['standing_w']:.2f} W |")
        add(f"| Walking, {ref['run_id']} (Trot, middle speed) | {ref['avg_ma']:.0f} mA | {ref['avg_w']:.2f} W |")
    add('')
    if 'rest_start' in ph:
        off = [ph[n] for n in ('rest_start', 'rest_end') if n in ph]
        off_ma, off_w = np.mean([o['ma'] for o in off]), np.mean([o['w'] for o in off])
        add(f"Holding the stance adds {s['avg_ma'] - off_ma:.0f} mA ({s['avg_w'] - off_w:.2f} W) to "
            f"the motors-off draw; the other {off_ma:.0f} mA ({off_w:.2f} W) the robot draws with its "
            f"motors off too.")
        add('')

    # Added to the meta afterwards, from what the operator saw or said.
    if meta.get('observations'):
        add('## Observations')
        add('')
        for note in meta['observations']:
            add(f'- {note}')
        add('')

    add('## Battery')
    add('')
    add('| | Start | End |')
    add('|---|---|---|')
    rest = [ph[n]['v'] if n in ph else None for n in ('rest_start', 'rest_end')]
    add('| Voltage, motors off (INA237) | ' + ' | '.join(f'{v:.2f} V' if v else 'not measured' for v in rest) + ' |')
    add(f"| Charger reading | {_charger(s['charger_first'])} | {_charger(s['charger_last'])} |")
    add('')

    if s['servos']:
        add('## Servos while standing')
        add('')
        health = s['servos'][0]['v_min'] is not None
        add('| Slot | Leg | Current (own reading) | Temperature, start / end |'
            + (' Lowest supply | Hardware error |' if health else ''))
        add('|---|---|---|---|' + ('---|---|' if health else ''))
        for k, sv in enumerate(s['servos']):
            row = f"| {k + 1} | {LEGS[k // 2]} | {sv['ma']:.0f} mA | {sv['c_first']} / {sv['c_last']} °C |"
            if health:
                row += f" {sv['v_min']:.1f} V | {'none' if not sv['error'] else sv['error']} |"
            add(row)
        add('')
        add("Each servo's own reading of its motor current, on the 6 V servo rail - not the battery "
            "side, and rougher than the INA237: a guide to which servos hold the most.")
        add('')

    add(f"![{meta['run_id']} plots]({png})")
    add('')

    add('## Timeline')
    add('')
    add('| Time | |')
    add('|---|---|')
    for t, text in run['events']:
        add(f'| {_clock(t)} | {text} |')
    add('')

    add('## Run details')
    add('')
    add('| | |')
    add('|---|---|')
    rows = {
        'Started': meta['started'], 'Setup': meta['setup'], 'PCB': meta['pcb'], 'Pack': meta['pack'],
        'Robot firmware': meta['robot_fw'], 'Controller': meta['controller'], 'Sensor': meta['sensor'],
        'Servos': meta['servos'], 'Settled when': meta['settle'],
        'Reference run': f"[{ref['run_id']}]({ref['run_id']}.md)" if ref else 'none',
        'Command': f"`{meta['command']}`",
    }
    if meta.get('note'):
        rows['Note'] = meta['note']
    for k, v in rows.items():
        add(f'| {k} | {v} |')
    gaps = 'no gaps' if not s['gaps'] else f"{s['gaps']} gap{'s' if s['gaps'] != 1 else ''}"
    add(f"| INA237 readings | {s['readings']} ({s['rate_hz']:.1f} per second); {gaps} over "
        f"{GAP_S:g} s, longest pause {s['longest_gap_s']:.1f} s |")
    add(f"| Charger readings | {len(run['charger'])} |")
    if s['servos']:
        add(f"| Servo readings | {s['servo_readings']} |")
    add(f"| INA237 die temperature | {s['die_start']:.1f} °C at the start, {s['die_max']:.1f} °C highest |")
    add('')
    add('Times are minutes:seconds from when the measured stretch started. Averages, charge and '
        'energy are the INA237 readings integrated over time. Readings: '
        + ', '.join(f'[{f}]({f})' for f in files) + '.')
    add('')
    path.write_text('\n'.join(lines), encoding='utf-8')


def write_csvs(run, ina_path, charger_path, servo_path):
    def phase_of(t):
        return next((n for n, (a, b) in run['phases'].items() if a <= t <= b), '')
    with open(ina_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'phase', 'bus_v', 'current_ma', 'power_mw', 'die_c'])
        for k, t in enumerate(run['t']):
            w.writerow([f'{t:.3f}', phase_of(t), f"{run['bus_v'][k]:.3f}", f"{run['current_a'][k] * 1000:.1f}",
                        f"{run['power_w'][k] * 1000:.0f}", f"{run['die_c'][k]:.1f}"])
    with open(charger_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'phase', 'percent', 'volts'])
        for t, pct, v in run['charger']:
            w.writerow([f'{t:.3f}', phase_of(t), pct, f'{v:.3f}'])
    if run['servo_t'] is not None:
        n = run['servo_c'].shape[1]
        with open(servo_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'phase'] + [f'slot{k + 1}_c' for k in range(n)]
                       + [f'slot{k + 1}_ma' for k in range(n)])
            for t, c, ma in zip(run['servo_t'], run['servo_c'], run['servo_ma']):
                w.writerow([f'{t:.3f}', phase_of(t)] + [int(x) for x in c] + [int(x) for x in ma])


# ---------- plot ----------

PHASE_LABELS = {'rest_start': 'motors off', 'settling': 'settling', 'standing': 'measured', 'rest_end': 'motors off'}


def plot(run, s, path):
    t = run['t']
    servos = run['servo_t'] is not None
    panels = 5 if servos else 4
    height = 1.4 + 2.3 * panels
    fig, axes = plt.subplots(panels, 1, figsize=(11, height), sharex=True, gridspec_kw={'hspace': 0.12})
    fig.subplots_adjust(top=1 - 1.45 / height, bottom=0.7 / height, left=0.08, right=0.95)

    meta, ref = run['meta'], s['ref']
    title = _title(meta)
    sub = (f"standing, motors on: {s['avg_ma']:.0f} mA  ·  {s['avg_w']:.2f} W  ·  {s['avg_v']:.2f} V, "
           f"averaged over {s['measured_s']:.0f} s")
    if ref:
        sub += f"  ·  about {_duration(s['hours_energy'])} on {ref['run_id']}'s {ref['wh']:.2f} Wh"
    fig.text(0.08, 1 - 0.52 / height, title, fontsize=15, weight='bold')
    fig.text(0.08, 1 - 0.8 / height, sub, fontsize=10.5, color='#3b434c')

    a, b = run['phases']['standing']
    for ax in axes:
        for name, (p0, p1) in run['phases'].items():
            if name != 'standing':
                ax.axvspan(p0, p1, color=COLORS['shade'], zorder=0, lw=0)
        ax.grid(True, color='#dde2e7', lw=0.6)
        ax.spines[['top', 'right']].set_visible(False)
    for name, (p0, p1) in run['phases'].items():
        axes[0].text((p0 + p1) / 2, 1.03, PHASE_LABELS[name], transform=axes[0].get_xaxis_transform(),
                     ha='center', va='bottom', fontsize=8.5,
                     color='#1d2329' if name == 'standing' else COLORS['event'])

    smooth = lambda y: _rolling_mean(t, y, SMOOTH_S)

    def spread(ax, y, color, label=None):
        # Only over the measured stretch - a bin across the motors going on
        # or off would just draw a wedge.
        m = _mask(run, 'standing')
        c, lo, hi = _band(t[m], y[m], SMOOTH_S)
        if len(c):
            ax.fill_between(c, lo, hi, color=color, alpha=0.16, lw=0, label=label)

    def average(ax, value, color, text):
        ax.hlines(value, a, b, color=color, lw=0.9, ls='--')
        ax.annotate(text, (a + 1, value), xytext=(0, 7), textcoords='offset points',
                    va='bottom', ha='left', fontsize=8.5, color=color)

    ax = axes[0]
    spread(ax, run['current_a'] * 1000, COLORS['current'], 'middle 80% of readings')
    ax.plot(t, smooth(run['current_a'] * 1000), color=COLORS['current'], lw=1.6, label=f'{SMOOTH_S:g} s average')
    average(ax, s['avg_ma'], COLORS['current'], f"standing average {s['avg_ma']:.0f} mA")
    ax.set_ylabel('Current (mA)')
    ax.set_ylim(0, max(s['peak_1s_ma'], float(np.max(smooth(run['current_a'] * 1000)))) * 1.25)
    ax.legend(loc='lower right', fontsize=8.5, frameon=False, ncol=2)

    ax = axes[1]
    spread(ax, run['bus_v'], COLORS['voltage'])
    ax.plot(t, smooth(run['bus_v']), color=COLORS['voltage'], lw=1.6, label='INA237')
    if run['charger']:
        ax.plot([r[0] for r in run['charger']], [r[2] for r in run['charger']], 'o', ms=4,
                color=COLORS['charger'], label='charger')
    if run['servo_v'] is not None:
        ax.plot(run['servo_t'], run['servo_v'].min(axis=1), color=COLORS['event'], lw=1.2,
                label="servos' supply (lowest)")
    ax.set_ylabel('Voltage (V)')
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo - 0.3 * (hi - lo), hi + 0.3 * (hi - lo))
    ax.legend(loc='upper right', fontsize=8.5, frameon=False, ncol=3)

    ax = axes[2]
    spread(ax, run['power_w'], COLORS['power'])
    ax.plot(t, smooth(run['power_w']), color=COLORS['power'], lw=1.6)
    average(ax, s['avg_w'], COLORS['power'], f"standing average {s['avg_w']:.2f} W")
    ax.set_ylabel('Power (W)')
    ax.set_ylim(0, float(np.max(smooth(run['power_w']))) * 1.25)

    if servos:
        ax = axes[3]
        for k in range(run['servo_ma'].shape[1]):
            ax.plot(run['servo_t'], np.abs(run['servo_ma'][:, k]), color=LEG_COLORS[k // 2],
                    ls='-' if k % 2 == 0 else '--', lw=1.6,
                    label=f'slot {k + 1} ({LEGS[k // 2]})')
        ax.set_ylabel("Servo current (mA)\neach servo's own reading")
        ax.set_ylim(0, max(10, np.abs(run['servo_ma']).max()) * 1.45)
        ax.legend(loc='upper left', fontsize=7.5, frameon=False, ncol=4)

    ax = axes[-1]
    die = smooth(run['die_c'])
    chip = ax.plot(t, die, color=COLORS['die'], lw=1.6, label='INA237 chip')
    lows, highs = [die.min()], [die.max()]
    if servos:
        lines = []
        for k in range(run['servo_c'].shape[1]):
            lines += ax.plot(run['servo_t'], run['servo_c'][:, k], color=LEG_COLORS[k // 2],
                             ls='-' if k % 2 == 0 else '--', lw=1.6, label=f'slot {k + 1} ({LEGS[k // 2]})')
        lows.append(run['servo_c'].min())
        highs.append(run['servo_c'].max())
        ax.add_artist(ax.legend(handles=lines, loc='upper left', fontsize=7.5, frameon=False, ncol=4))
    ax.legend(handles=chip, loc='upper right', fontsize=7.5, frameon=False)
    ax.set_ylabel('Temperature (°C)')
    lo, hi = min(lows), max(highs)
    ax.set_ylim(lo - 2, hi + 0.35 * (hi - lo + 4))
    ax.set_xlabel('Seconds since the measured stretch started')

    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------- the report ----------

def add_to_readme(folder, run, s):
    readme = folder / 'README.md'
    if not readme.exists():
        return False
    text = readme.read_text(encoding='utf-8')
    if README_MARKER not in text:
        return False
    meta, ref = run['meta'], s['ref']
    estimate = f"about {_duration(s['hours_energy'])} (on {ref['run_id']}'s {ref['wh']:.2f} Wh)" if ref \
        else f"{_duration(s['hours_per_wh'])} per Wh"
    row = (f"| [{meta['run_id']}]({meta['run_id']}.md) | {meta['started'][:10]} | {meta['setup']} | "
           f"{s['avg_ma']:.0f} mA, {s['avg_w']:.2f} W at {s['avg_v']:.2f} V | {estimate} |")
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
        print(f'{run_id} never got to the measured stretch - its log is in {folder / "data"}')
        return None
    s = summarize(run, reference(folder, run['meta'].get('reference')))
    (folder / 'images').mkdir(parents=True, exist_ok=True)
    png = f'images/{run_id}-plots.png'
    files = [f'data/{run_id}-ina237.csv', f'data/{run_id}-charger.csv']
    if run['servo_t'] is not None:
        files.append(f'data/{run_id}-servos.csv')
    plot(run, s, folder / png)
    write_csvs(run, folder / files[0], folder / files[1], folder / f'data/{run_id}-servos.csv')
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
