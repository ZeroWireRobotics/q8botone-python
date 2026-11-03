'''
Battery runtime test: walk the robot on its own battery until the battery
gives out, logging the INA237 on its Qwiic port, then write a report into
q8botone-docs (docs/tests/battery-runtime, next to this repository).

    python tools/power_run.py --pcb PROTO-002 --pack "2S 1000 mAh"

Needs matplotlib: pip install -e ".[tools]"

Before starting:
  - flash the robot with the power-test firmware (q8botone-firmware/robot,
    `pio run -e robot_power_test -t upload`). Normal firmware refuses to
    start below 5% if the robot restarts late in the run, which would end
    the run before the PCB's undervoltage protection does.
  - the INA237 on the robot's Qwiic port, in the battery path
  - the robot's USB unplugged - otherwise it charges during the run
  - the controller plugged into this computer

A run: motors off for a few seconds (the rested starting voltage),
standing (the idle draw), then walking forward until the robot stops
answering - nothing here stops it earlier. If the robot restarts on the
way, its motors go back on and it walks on. At the end it asks how the
robot's light looks and for the pack voltage on a multimeter, both
optional, then writes the report.

Ctrl+C ends the run early (the robot stops, motors off) and still writes
the report. Every reading is logged as it arrives (data/<run id>-log.csv,
with data/<run id>-meta.json); `--from-log BAT-RT-001` rebuilds a report
from those alone.
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

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from q8bot import Q8Bot                                    # noqa: E402
from q8bot.firmware_flasher import identify_device          # noqa: E402
from q8bot.gait_manager import GaitManager                  # noqa: E402
from q8bot.helpers import XiaoPortFinder                    # noqa: E402
from q8bot.protocol import describe_servo_error             # noqa: E402
from q8bot.robot import _nearest_tier                       # noqa: E402
import power_report                                         # noqa: E402
from power_report import Run, UVP_V                         # noqa: E402

DEFAULT_OUT = REPO.parent / 'q8botone-docs' / 'docs' / 'tests' / 'battery-runtime'
RUN_PREFIX = 'BAT-RT-'
README_MARKER = '<!-- power_run.py adds each run above this line -->'

INA237_KIND = 1          # protocol.SENSOR_KINDS
POLL_S = 0.02
RESTART_JUMP_MS = 1000   # the robot's clock going back more than this = it restarted
SERVO_SILENT_S = 3.0     # no servo reading this long while the robot still answers = see servo_gap_events
PROGRESS_EVERY_S = 30


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', help="the controller's port (found by asking each board if left out)")
    p.add_argument('--speed', type=float, default=0.64,
                   help='bot.move() forward share, 0-1 (default 0.64 = Trot\'s middle speed)')
    p.add_argument('--setup', default='on a test stand', help='where it walks (default: %(default)s)')
    p.add_argument('--pcb', default='not recorded', help='PCB revision / serial')
    p.add_argument('--pack', default='not recorded', help='battery pack')
    p.add_argument('--robot-fw', default='not recorded',
                   help="the robot's firmware (it can't report it over the radio)")
    p.add_argument('--note', default='', help='anything else worth keeping with the run')
    p.add_argument('--out', type=Path, default=DEFAULT_OUT, help='test folder (default: %(default)s)')
    p.add_argument('--sensor-hz', type=float, default=10.0)
    p.add_argument('--servo-hz', type=float, default=1.0,
                   help='servo temperature readings per second, 0 for none (the robot '
                        'refreshes them once a second)')
    p.add_argument('--charger-every', type=float, default=5.0, help='seconds between charger readings')
    p.add_argument('--rest-s', type=float, default=8.0, help='motors off at the start')
    p.add_argument('--stand-s', type=float, default=15.0, help='standing before walking')
    p.add_argument('--silent-after', type=float, default=20.0,
                   help='no reply for this long = the robot has stopped answering '
                        '(long enough for a restart to come back)')
    p.add_argument('--ina-address', type=lambda s: int(s, 0), help='which INA237, if there are several')
    p.add_argument('--yes', action='store_true', help="don't ask before starting")
    p.add_argument('--no-prompts', action='store_true', help="don't ask anything at the end either")
    p.add_argument('--from-log', metavar='RUN_ID', help='rebuild that run\'s report from its log')
    return p.parse_args()


# ---------- the log: written as the run goes, the only input to the report ----------

class Log:
    '''data/<run id>-log.csv: every reading, phase change and event, on the host's clock.'''
    # servo_c / servo_ma / servo_v / servo_err: all eight servos'
    # temperatures, currents, supply voltages and Hardware Error Status, space
    # separated. Logs from before they were added just have no 'servo' rows,
    # or no servo_v / servo_err (protocol 3 firmware).
    FIELDS = ['host_s', 'kind', 'robot_ms', 'bus_v', 'current_a', 'power_w', 'die_c',
              'percent', 'volts', 'servo_c', 'servo_ma', 'servo_v', 'servo_err', 'text']

    def __init__(self, path):
        self._f = open(path, 'w', newline='', encoding='utf-8')
        self._w = csv.DictWriter(self._f, self.FIELDS)
        self._w.writeheader()
        self._flushed = time.monotonic()

    def write(self, host_s, kind, **values):
        self._w.writerow({'host_s': f'{host_s:.4f}', 'kind': kind, **values})
        if time.monotonic() - self._flushed > 1.0:
            self._f.flush()
            self._flushed = time.monotonic()

    def close(self):
        self._f.close()


def save_meta(path, meta):
    path.write_text(json.dumps(meta, indent=2), encoding='utf-8')


def next_run_id(folder):
    used = [int(m.group(1)) for f in (folder / 'data').glob(f'{RUN_PREFIX}*')
            if (m := re.match(re.escape(RUN_PREFIX) + r'(\d+)', f.name))]
    return f'{RUN_PREFIX}{max(used, default=0) + 1:03d}'


# ---------- the run ----------

def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ''


def find_ina237(snapshot, address):
    for s in snapshot.sensors:
        if s.kind == INA237_KIND and (address is None or s.address == address):
            return s
    return None


def tier_name(bot, speed):
    chain = GaitManager.SPEED_TIER_CHAINS.get(bot.gait)
    return chain[_nearest_tier(chain, bot.gaits, speed)] if chain else bot.gait


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
        print('This walks the robot until its battery gives out - the PCB\'s undervoltage '
              f'protection ({UVP_V:.2f} V) is the only thing that stops it. Check:\n'
              '  - it is ' + args.setup + '\n'
              '  - its USB is unplugged (or it charges during the run)\n'
              '  - it runs the power-test firmware (env robot_power_test)\n'
              '  - the pack is one you are OK to over-discharge, and someone stays with it')
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
            'speed': args.speed, 'gait': tier_name(bot, args.speed),
            'sensor': f'INA237 at {ina_address:#04x}, asked {args.sensor_hz:g} times a second',
            'servos': (f'temperature and current of all eight, asked {args.servo_hz:g} times a second'
                       if args.servo_hz else 'not read'),
            'command': 'python ' + ' '.join(Path(a).name if i == 0 else a for i, a in enumerate(sys.argv)),
            # The log's host_s is time.monotonic(); this pins it to the wall
            # clock, to line up another logger's file (e.g. a thermocouple's).
            'clock': {'monotonic': time.monotonic(), 'unix': time.time()},
            'ended_by': None, 'light': None, 'rested_end_v': None,
        }
        meta_path = folder / 'data' / f'{run_id}-meta.json'
        save_meta(meta_path, meta)
        try:
            meta['ended_by'] = _walk(bot, args, log, ina_address, meta)
        finally:
            bot.close()
            log.close()
    finally:
        if bot.connected:
            bot.close()

    if not args.no_prompts and sys.stdin.isatty():
        try:
            light = ask("How does the robot's light look now? [d]ark, [r]ed blink every 10 s, "
                        'or describe it (Enter to skip): ')
            meta['light'] = {'d': 'dark', 'r': 'red blink'}.get(light.lower(), light) or None
            volts = ask('Pack voltage on a multimeter once it has rested (Enter to skip): ')
            try:
                meta['rested_end_v'] = float(volts.rstrip('Vv ')) if volts else None
            except ValueError:
                print(f"Couldn't read {volts!r} as a voltage - leaving it out")
        except KeyboardInterrupt:
            print()
    save_meta(meta_path, meta)
    return meta_path


def _walk(bot, args, log, ina_address, meta):
    '''The run itself. Returns how it ended: 'silent', 'ctrl_c' or 'link'.'''
    start = time.monotonic()
    last_reply = start
    last_robot_ms = None
    last_v = float('nan')
    seen = None                 # the newest sensor snapshot already logged
    seen_servos = None          # likewise the newest joint snapshot
    last_errors = (0,) * 8      # each servo's Hardware Error Status as last seen
    last_servo_at = None        # when the last servo reading came
    servos_quiet = False        # said so already (see servo_gap_events)
    next_charger = start
    used_mah, last_i, last_t = 0.0, None, None
    next_progress = start + PROGRESS_EVERY_S
    phase = 'rest_start'

    log.write(start, 'phase', text='rest_start')
    log.write(start, 'event', text='Connected, motors off')
    bot.set_torque(False)
    try:
        while True:
            now = time.monotonic()

            if phase == 'rest_start' and now - start >= args.rest_s:
                phase = 'standing'
                log.write(now, 'phase', text=phase)
                log.write(now, 'event', text='Motors on, standing')
                bot.set_torque(True)
            elif phase == 'standing' and now - start >= args.rest_s + args.stand_s:
                phase = 'walking'
                log.write(now, 'phase', text=phase)
                log.write(now, 'event', text=f"Walking started ({meta['gait']})")
                bot.move(args.speed)
                print('Walking. Ctrl+C stops the run early.')

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
                    log.write(now, 'event', text=f'Robot restarted (last reading {last_v:.2f} V) - '
                                                 'motors back on')
                    print('Robot restarted - motors back on.')
                    if phase != 'rest_start':
                        bot.set_torque(True)
                    if phase == 'walking':
                        bot.move(args.speed)
                last_robot_ms = snap.t_ms
                ina = find_ina237(snap, ina_address)
                if ina is None or not all(math.isfinite(v) for v in ina.values):
                    continue
                bus_v, _shunt_mv, current_ma, power_mw, die_c = ina.values
                last_v = bus_v
                log.write(now, 'ina', robot_ms=snap.t_ms, bus_v=f'{bus_v:.4f}',
                          current_a=f'{current_ma / 1000:.5f}', power_w=f'{power_mw / 1000:.4f}',
                          die_c=f'{die_c:.2f}')
                if phase == 'walking':
                    if last_i is not None:
                        used_mah += (current_ma / 1000 + last_i) / 2 * (now - last_t) / 3.6
                    last_i, last_t = current_ma / 1000, now

            # Servo temperatures (and currents) - a new dict for every reply.
            servos = bot.telemetry
            if servos is not None and servos is not seen_servos:
                seen_servos = servos
                last_reply = last_servo_at = now
                if servos_quiet:
                    servos_quiet = False
                    print('Servo readings are back.')
                volts, errors = servos.get('voltage'), servos.get('hw_error')
                log.write(now, 'servo', servo_c=' '.join(str(c) for c in servos['temperature']),
                          servo_ma=' '.join(str(c) for c in servos['current']),
                          servo_v=' '.join(f'{v:.1f}' for v in volts) if volts else '',
                          servo_err=' '.join(str(e) for e in errors) if errors else '')
                # Say so the moment a servo reports a new hardware error.
                if errors:
                    new = {}
                    for slot, (was, err) in enumerate(zip(last_errors, errors), 1):
                        if err and err != was:
                            new.setdefault(describe_servo_error(err), []).append(slot)
                    for kind, slots in new.items():
                        lowest = min(volts[s - 1] for s in slots)
                        print(f'Servo hardware error - {kind}, {_slots(slots)} (supply down to '
                              f'{lowest:.1f} V): shut down, torque off until restarted.')
                    last_errors = errors

            # Charger readings.
            if now >= next_charger:
                bot.request_battery()
                next_charger = now + args.charger_every
            pct = bot.take_battery_reading()
            if pct is not None:
                last_reply = now
                if bot.battery_volts is not None:   # None = firmware that only sends the %
                    log.write(now, 'charger', percent=pct, volts=f'{bot.battery_volts:.3f}')

            if now >= next_progress and phase == 'walking' and last_i is not None:
                next_progress = now + PROGRESS_EVERY_S
                walked = now - start - args.rest_s - args.stand_s
                hottest = f'   hottest servo {max(seen_servos["temperature"])} C' if seen_servos else ''
                print(f'{int(walked // 60):3d}:{int(walked % 60):02d} walking   {last_v:.2f} V   '
                      f'{last_i:.2f} A   {used_mah:.0f} mAh   charger {bot.battery_percent}%{hottest}')

            if (last_servo_at is not None and not servos_quiet and now - last_servo_at > SERVO_SILENT_S
                    and now - last_reply < 1.0):
                servos_quiet = True
                print("Servo readings stopped - the robot couldn't read all eight servos, though it "
                      'keeps answering. One may have dropped off the bus.')

            if not bot.connected:
                log.write(now, 'event', text='Lost the link to the controller')
                return 'link'
            if now - last_reply > args.silent_after:
                log.write(last_reply, 'event', text=f'Robot stopped answering - last reading {last_v:.2f} V')
                print(f'The robot stopped answering (last reading {last_v:.2f} V).')
                return 'silent'
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        log.write(time.monotonic(), 'event', text='Stopped with Ctrl+C')
        print('Stopped with Ctrl+C.')
        return 'ctrl_c'


# ---------- log -> report ----------

def _slots(slots):
    '''[1, 2, 3, 5] -> "slots 1-3, 5".'''
    runs, start = [], slots[0]
    for a, b in zip(slots, slots[1:] + [None]):
        if b != a + 1:
            runs.append(f'{start}' if start == a else f'{start}-{a}')
            start = b
    return ('slot ' if len(slots) == 1 else 'slots ') + ', '.join(runs)


def servo_error_events(t, volts, errors):
    '''
    Timeline events for each new hardware error, servos that fault together
    with the same error in one event - and (time, description) of the first
    reading with at least half the servos in error, or None.
    '''
    if t is None:
        return [], None
    events, stop = [], None
    previous = np.zeros(errors.shape[1], dtype=int)
    for when, v, err in zip(t, volts, errors):
        by_kind = {}
        for i, (was, e) in enumerate(zip(previous, err)):
            if e and e != was:
                by_kind.setdefault(describe_servo_error(int(e)), []).append(i)
        for kind, idx in by_kind.items():
            lo, hi = min(v[idx]), max(v[idx])
            supply = f'{lo:.1f} V' if lo == hi else f'{lo:.1f}-{hi:.1f} V'
            events.append((float(when), f'Servo hardware error - {kind}, '
                                        f'{_slots([i + 1 for i in idx])} (supply {supply})'))
        if stop is None and np.count_nonzero(err) * 2 >= len(err):
            kinds = sorted({describe_servo_error(int(e)) for e in err if e})
            stop = (float(when), ', '.join(kinds))
        previous = err
    return events, stop


def servo_gap_events(servo_t, ina_t):
    '''
    Events for stretches with no servo readings while the INA237's kept
    coming - the robot reads all eight servos in one go and only replies if
    every one answered, so one servo gone quiet on the bus stops them all.
    '''
    if servo_t is None or len(servo_t) < 2:
        return []
    events = []
    for a, b in list(zip(servo_t[:-1], servo_t[1:])) + [(servo_t[-1], ina_t[-1])]:
        if b - a > SERVO_SILENT_S and np.count_nonzero((ina_t > a) & (ina_t < b)) > 10:
            events.append((float(a), "Servo readings stopped - the robot couldn't read all eight "
                                     'servos, though it kept answering'))
            if b < ina_t[-1]:
                events.append((float(b), 'Servo readings back'))
    return events


ENDINGS = {
    'silent': (f"Robot stopped answering (the PCB's undervoltage protection is set to {UVP_V:.2f} V)",
               'stopped answering'),
    'ctrl_c': ('Run stopped with Ctrl+C', 'stopped (Ctrl+C)'),
    'link': ('Link to the controller lost', 'link lost'),
}


def build_run(log_path, meta):
    rows = list(csv.DictReader(open(log_path, encoding='utf-8')))
    phase_at = {r['text']: float(r['host_s']) for r in rows if r['kind'] == 'phase'}
    if 'walking' not in phase_at:
        return None

    ina = [r for r in rows if r['kind'] == 'ina']
    host = np.array([float(r['host_s']) for r in ina])
    robot = np.array([float(r['robot_ms']) / 1000 for r in ina])
    # The robot's clock spaces the readings; a restart starts it again, so
    # each stretch of it is pinned to the host's clock separately - by its
    # smallest host-minus-robot gap, the reply that arrived quickest.
    t = np.empty_like(host)
    starts = [0] + [i for i in range(1, len(robot)) if robot[i] < robot[i - 1] - RESTART_JUMP_MS / 1000]
    for a, b in zip(starts, starts[1:] + [len(robot)]):
        t[a:b] = robot[a:b] + np.min(host[a:b] - robot[a:b])
    zero = phase_at['walking']
    t -= zero
    order = np.argsort(t, kind='stable')
    col = lambda name: np.array([float(r[name]) for r in ina])[order]
    t = t[order]
    if np.sum(t >= 0) < 10:
        return None

    charger = [(float(r['host_s']) - zero, int(r['percent']), float(r['volts']))
               for r in rows if r['kind'] == 'charger']
    events = sorted(((float(r['host_s']) - zero, r['text']) for r in rows if r['kind'] == 'event'),
                    key=lambda e: e[0])
    servo_rows = [r for r in rows if r['kind'] == 'servo']
    servo_t = np.array([float(r['host_s']) - zero for r in servo_rows]) if servo_rows else None
    servo_c = np.array([[int(c) for c in r['servo_c'].split()] for r in servo_rows]) if servo_rows else None
    # Supply voltage and hardware errors - protocol 4 firmware on.
    health = [r for r in servo_rows if r.get('servo_v') and r.get('servo_err')]
    health_t = np.array([float(r['host_s']) - zero for r in health]) if health else None
    health_v = np.array([[float(v) for v in r['servo_v'].split()] for r in health]) if health else None
    health_err = np.array([[int(e) for e in r['servo_err'].split()] for r in health]) if health else None
    error_events, errors_stop = servo_error_events(health_t, health_v, health_err)
    events = sorted(events + error_events + servo_gap_events(servo_t, t), key=lambda e: e[0])
    phases = {
        'rest_start': (phase_at['rest_start'] - zero, phase_at['standing'] - zero),
        'standing': (phase_at['standing'] - zero, 0.0),
        'walking': (0.0, float(t[-1])),
    }
    # When the motors gave out before the robot went quiet (seconds since
    # walking started): when at least half the servos reported a hardware
    # error, or added to the meta afterwards from the readings (BAT-RT-001,
    # before the servos' errors were recorded).
    stopped = meta.get('motors_stopped_s')
    if stopped is None and errors_stop is not None and errors_stop[0] > 0:
        stopped = errors_stop[0]
        meta = dict(meta, motors_stopped_label=f'Motors stopped - servo hardware error ({errors_stop[1]})')
    elif stopped is not None:
        events = sorted(events + [(float(stopped), meta.get('motors_stopped_note', 'Motors stopped'))],
                        key=lambda e: e[0])
    if stopped is not None:
        phases['walking'] = (0.0, float(stopped))
        phases['stopped'] = (float(stopped), float(t[-1]))

    ended_by = meta.get('ended_by') or 'ctrl_c'
    end_label, end_mark = ENDINGS[ended_by]
    last_v = float(col('bus_v')[-1])
    if ended_by == 'silent' and last_v > UVP_V + 0.2:
        end_label = (f"Robot stopped answering - at {last_v:.2f} V, above the PCB's "
                     f"{UVP_V:.2f} V undervoltage protection")
    end_clock = power_report._clock(float(t[-1]))
    stop_reason = {
        'silent': f'Walked until the robot stopped answering, last reading {last_v:.2f} V',
        'ctrl_c': f'Stopped early with Ctrl+C, at {last_v:.2f} V',
        'link': f'Ended when the link to the controller was lost, at {last_v:.2f} V',
    }[ended_by]
    if stopped is not None and ended_by == 'silent':
        stop_reason = (f'The motors stopped at {power_report._clock(float(stopped))}; the robot '
                       f'stopped answering at {end_clock}, last reading {last_v:.2f} V')
    # Added to the meta afterwards, from what the operator saw - e.g. that a
    # robot which went quiet was still powered.
    end_label = meta.get('end_label', end_label)
    if meta.get('stop_note'):
        stop_reason += f" - {meta['stop_note']}"
    light = meta.get('light')
    if light == 'dark':
        stop_reason += ' - its light was dark afterwards: the power was cut'
    elif light == 'red blink':
        stop_reason += (" - its light blinked red afterwards: the firmware's boot cutoff stopped it, "
                        'not the PCB')

    details = {
        'Started': meta['started'],
        'Setup': meta['setup'],
        'PCB': meta['pcb'],
        'Pack': meta['pack'],
        'Gait': f"Trot, {meta['gait']} - `bot.move({meta['speed']:g})`",
        'Stop rule': 'none - walks until the robot stops answering (or Ctrl+C)',
        'Robot firmware': meta['robot_fw'],
        'Controller': meta['controller'],
        'Sensor': meta['sensor'],
        'Servos': meta.get('servos', 'not read'),
        'Light after the run': light or 'not recorded',
        'Command': f"`{meta['command']}`",
    }
    if meta.get('note'):
        details['Note'] = meta['note']
    return Run(run_id=meta['run_id'], meta=details, setup=meta['setup'], t=t,
               bus_v=col('bus_v'), current_a=col('current_a'), power_w=col('power_w'),
               die_c=col('die_c'), phases=phases, battery=charger, events=events,
               stop_reason=stop_reason, end_label=end_label, end_mark=end_mark,
               rested_end_v=meta.get('rested_end_v'), servo_t=servo_t, servo_c=servo_c,
               health_t=health_t, servo_v=health_v, servo_err=health_err,
               motor_stop_label=meta.get('motors_stopped_label', 'Motors stopped'),
               notes=meta.get('observations'))


def add_to_readme(folder, run):
    '''The run's row in the test page's Runs table, if it has one - added, or
    brought up to date when the report is rebuilt.'''
    readme = folder / 'README.md'
    if not readme.exists():
        return False
    text = readme.read_text(encoding='utf-8')
    if README_MARKER not in text:
        return False
    s = run.stats
    ended = f"{run.end_mark}, last reading {s['v_last']:.2f} V"
    if s['motor_stop']:
        ended = f"motors stopped at {power_report._clock(s['motor_stop']['t'])}; " + ended
    row = (f"| [{run.run_id}]({run.run_id}.md) | {run.meta['Started'][:10]} | {run.setup} | "
           f"{power_report._hms(s['walk_s'])} | {s['charge_mah']:.0f} mAh | {ended} |")
    lines = text.split('\n')
    mine = [i for i, line in enumerate(lines) if line.startswith(f'| [{run.run_id}]')]
    if mine:
        lines[mine[0]] = row
    else:
        lines.insert(lines.index(next(l for l in lines if README_MARKER in l)), row)
    readme.write_text('\n'.join(lines), encoding='utf-8')
    return True


def report_from(folder, run_id):
    meta = json.loads((folder / 'data' / f'{run_id}-meta.json').read_text(encoding='utf-8'))
    run = build_run(folder / 'data' / f'{run_id}-log.csv', meta)
    if run is None:
        print(f'{run_id} has too little walking for a report - its log is in {folder / "data"}')
        return None
    path = power_report.write_report(run, folder)
    add_to_readme(folder, run)
    return path


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
