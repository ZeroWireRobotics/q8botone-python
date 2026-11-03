'''
The report for a battery runtime run (see power_run.py): the summary
numbers, a report page, its plot and the processed CSVs.

A run walks until the robot stops answering - nothing on the robot or in
power_run.py stops it earlier - so it shows both layers of battery
protection: where the firmware's software cutoff would have powered the
robot off, and where the PCB's undervoltage protection actually did.
Times are seconds on one clock, 0 = walking started:

    motors off  ->  standing  ->  walking ... until the robot goes quiet
    (rested V)      (idle draw)

Files, in the docs test folder (q8botone-docs/docs/tests/battery-runtime):

    <run id>.md                  the report
    images/<run id>-plots.png    its plot
    data/<run id>-ina237.csv     every INA237 reading, with phase and charge used
    data/<run id>-charger.csv    every charger reading
'''

import csv
import re
from dataclasses import dataclass, field

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from q8bot.protocol import describe_servo_error

# q8botone-firmware/robot/src/battery.cpp: 0% = 5.40 V is the software
# cutoff, the PCB's UVP trips at 4.8 V. Keep in step with it.
SOFT_CUTOFF_V = 5.40
UVP_V = 4.80

SMOOTH_S = 10.0        # rolling-average window for the plotted lines
PEAK_WINDOW_S = 1.0    # "highest 1 s average" window
GAP_S = 0.5            # a pause between readings longer than this counts as a gap
RESTED_TAIL_S = 5.0    # rested voltage = the average over the end of the motors-off phase
EDGE_S = 10.0          # "first / last 10 s" of the walk

COLORS = {
    'current': '#1f6fb2',
    'voltage': '#2f8f5b',
    'charger': '#b04a2f',
    'power': '#7a4fb0',
    'charge': '#1f6fb2',
    'percent': '#b04a2f',
    'soft': '#c27c0e',
    'end': '#b3261e',
    'shade': '#eef1f4',
    'event': '#5b6470',
    'die': '#3b434c',
}


@dataclass
class Run:
    run_id: str
    meta: dict             # Run details rows, in order
    setup: str             # e.g. "on a test stand" - goes in the headline too
    t: np.ndarray          # INA237 reading times, s (0 = walking started)
    bus_v: np.ndarray
    current_a: np.ndarray
    power_w: np.ndarray
    die_c: np.ndarray
    phases: dict           # 'rest_start', 'standing', 'walking' -> (start s, end s)
    battery: list          # charger readings: (t s, percent, volts)
    events: list           # (t s, text) - summarize() adds the software cutoff
    stop_reason: str       # a sentence for the headline
    end_label: str         # the last Battery protection row, e.g. "Robot stopped answering (...)"
    end_mark: str          # the same, short, for the plot
    rested_end_v: float = None   # measured after the run, if it was
    servo_t: np.ndarray = None   # servo readings' times, s - None if they weren't read
    servo_c: np.ndarray = None   # their temperatures, °C, one column per slot
    health_t: np.ndarray = None  # times of the readings below - protocol 4 firmware on
    servo_v: np.ndarray = None   # each servo's supply, V, one column per slot
    servo_err: np.ndarray = None # each servo's Hardware Error Status (0 = fine)
    motor_stop_label: str = None # the Battery protection row when phases has 'stopped'
    notes: list = None           # what the operator saw - an Observations list
    sample_data: bool = False
    stats: dict = field(default_factory=dict)


# The robot's servo slots (motor setup's numbering, internal IDs 11-18): two
# per leg. On the plot a leg is a color, its second servo dashed - the first
# four categorical slots of the dataviz reference palette, checked on white.
LEGS = ('front-left', 'front-right', 'back-left', 'back-right')
LEG_COLORS = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100')


# ---------- numbers ----------

def _cumulative(t, y):
    '''Running trapezoid integral of y over t, same length as t.'''
    out = np.zeros_like(t, dtype=float)
    out[1:] = np.cumsum((y[1:] + y[:-1]) / 2 * np.diff(t))
    return out


def _rolling_mean(t, y, window):
    '''Time-based rolling average (the integral over the last `window` s / window).'''
    area = _cumulative(t, y)
    start = np.maximum(t - window, t[0])
    span = t - start
    back = np.interp(start, t, area)
    with np.errstate(invalid='ignore', divide='ignore'):
        return np.where(span > 0, (area - back) / span, y)


def _band(t, y, window, lo=10, hi=90):
    '''Per `window`-second bin: (bin centers, lo percentile, hi percentile).'''
    edges = np.arange(t[0], t[-1] + window, window)
    idx = np.digitize(t, edges)
    centers, lows, highs = [], [], []
    for i in np.unique(idx):
        v = y[idx == i]
        if len(v) >= 3:
            centers.append(t[idx == i].mean())
            lows.append(np.percentile(v, lo))
            highs.append(np.percentile(v, hi))
    return np.array(centers), np.array(lows), np.array(highs)


def _mask(run, phase):
    a, b = run.phases[phase]
    return (run.t >= a) & (run.t <= b)


def _mean(values):
    return float(np.mean(values)) if len(values) else float('nan')


def _used_mah(run):
    '''Charge used since walking started, at each reading (negative before).'''
    used = _cumulative(run.t, run.current_a) / 3.6
    return used - np.interp(run.phases['walking'][0], run.t, used)


def summarize(run):
    w = _mask(run, 'walking')
    tw, iw, pw = run.t[w], run.current_a[w], run.power_w[w]
    w0, w1 = run.phases['walking']
    walk_s = w1 - w0
    used = _used_mah(run)
    charge_mah = np.trapezoid(iw, tw) / 3.6
    energy_wh = np.trapezoid(pw, tw) / 3600
    a, b = run.phases['rest_start']
    rested = (run.t >= max(a, b - RESTED_TAIL_S)) & (run.t <= b)
    s = {
        'walk_s': walk_s,
        'charge_mah': charge_mah,
        'energy_wh': energy_wh,
        'avg_ma': charge_mah * 3.6 / walk_s * 1000,
        'median_ma': float(np.median(iw)) * 1000,
        'peak_ma': float(np.max(iw)) * 1000,
        'peak_1s_ma': float(np.max(_rolling_mean(tw, iw, PEAK_WINDOW_S))) * 1000,
        'avg_w': energy_wh * 3600 / walk_s,
        'peak_w': float(np.max(pw)),
        'standing_ma': _mean(run.current_a[_mask(run, 'standing')]) * 1000,
        'off_ma': _mean(run.current_a[_mask(run, 'rest_start')]) * 1000,
        'v_start_rest': _mean(run.bus_v[rested]),
        'v_walk_first': _mean(run.bus_v[w & (run.t <= w0 + EDGE_S)]),
        'v_walk_last': _mean(run.bus_v[w & (run.t >= w1 - EDGE_S)]),
        'v_last': float(run.bus_v[-1]),
        'v_min': float(np.min(run.bus_v[w])),
        'die_start': float(run.die_c[0]),
        'die_max': float(np.max(run.die_c)),
        'charger_start': next((r for r in reversed(run.battery) if a <= r[0] <= b), None),
        'charger_last': run.battery[-1] if run.battery else None,
    }

    # Where the software cutoff would have tripped: the first charger reading
    # at or under it from walking on - the same chip and reading the firmware uses.
    end_t = float(run.t[-1])
    soft = next((r for r in run.battery if w0 <= r[0] <= end_t and r[2] <= SOFT_CUTOFF_V), None)
    s['soft'] = None if soft is None else {
        't': soft[0], 'v': soft[2], 'mah': float(np.interp(soft[0], run.t, used))}
    s['end'] = {'t': end_t, 'v': s['v_last'], 'mah': float(used[-1])}
    # A 'stopped' phase: the motors gave out before the robot went quiet. Its
    # voltage is the last second under load, before it bounced back.
    s['motor_stop'] = None
    if 'stopped' in run.phases:
        a = run.phases['stopped'][0]
        s['motor_stop'] = {'t': a, 'v': _mean(run.bus_v[(run.t >= a - 1.0) & (run.t < a)]),
                           'mah': float(np.interp(a, run.t, used))}

    s['servos'] = None
    if run.servo_t is not None and len(run.servo_t):
        c = run.servo_c
        s['servos'] = [{'start': int(c[0, i]), 'max': int(c[:, i].max()),
                        't_max': float(run.servo_t[int(np.argmax(c[:, i]))]), 'end': int(c[-1, i])}
                       for i in range(c.shape[1])]
        s['servo_readings'] = len(run.servo_t)
        if run.health_t is not None and len(run.health_t):
            for i, sv in enumerate(s['servos']):
                sv['v_min'] = float(run.servo_v[:, i].min())
                faulted = np.nonzero(run.servo_err[:, i])[0]
                sv['error'] = None if not len(faulted) else (
                    float(run.health_t[faulted[0]]), describe_servo_error(int(run.servo_err[faulted[0], i])))

    gaps = np.diff(run.t)
    s['readings'] = len(run.t)
    s['rate_hz'] = (len(run.t) - 1) / (run.t[-1] - run.t[0])
    s['gaps'] = int(np.sum(gaps > GAP_S))
    s['longest_gap_s'] = float(np.max(gaps))
    s['charger_readings'] = len(run.battery)
    run.stats = s

    if s['soft']:
        run.events = sorted(run.events + [(s['soft']['t'], 'Software cutoff would power off here '
                                                           f"({s['soft']['v']:.2f} V)")],
                            key=lambda e: e[0])
    return s


# ---------- formatting ----------

def _hms(seconds):
    seconds = int(round(seconds))
    return f'{seconds // 3600}:{seconds // 60 % 60:02d}:{seconds % 60:02d}'


def _clock(t):
    sign = '-' if t < 0 else ''
    t = abs(int(round(t)))
    return f'{sign}{t // 60}:{t % 60:02d}'


def _charger_text(reading):
    if not reading:
        return 'no reading'
    return f'{reading[2]:.2f} V, {reading[1]}%'


def write_markdown(run, path, png, ina_csv, charger_csv, servo_csv):
    s = run.stats
    lines = []
    add = lines.append
    add(f"# {run.run_id} - battery runtime, {run.setup}")
    add('')
    if run.sample_data:
        add('> **SAMPLE DATA** - made up to preview the layout. Nothing here was measured.')
        add('')
    add(f"**{_hms(s['walk_s'])} walking {run.setup}, {s['charge_mah']:.0f} mAh used, "
        f"{s['avg_ma']:.0f} mA average.** {run.stop_reason}.")
    add('')

    add('## Battery protection')
    add('')
    add('The robot was left walking until it stopped answering - nothing in the script or the '
        'firmware stopped it. The software cutoff is where the firmware would have powered it '
        'off; it isn\'t enforced on this run.')
    add('')
    add('| | Time | Voltage | Charge used by then |')
    add('|---|---|---|---|')
    soft, end, stop = s['soft'], s['end'], s['motor_stop']
    rows = []
    if stop:
        rows.append((stop['t'], f"| {run.motor_stop_label} | {_clock(stop['t'])} | "
                                f"{stop['v']:.2f} V (last second under load) | {stop['mah']:.0f} mAh |"))
    if soft:
        rows.append((soft['t'], f"| Software cutoff would have powered off (charger reads "
                                f"{SOFT_CUTOFF_V:.2f} V or less) | {_clock(soft['t'])} | "
                                f"{soft['v']:.2f} V | {soft['mah']:.0f} mAh |"))
    rows.append((end['t'], f"| {run.end_label} | {_clock(end['t'])} | {end['v']:.2f} V (last reading) | "
                           f"{end['mah']:.0f} mAh |"))
    for _, row in sorted(rows, key=lambda r: r[0]):
        add(row)
    if not soft:
        add(f"| Software cutoff ({SOFT_CUTOFF_V:.2f} V) | never reached | | |")
    elif soft['t'] < end['t']:
        add(f"| Margin from the software cutoff to the end | {_clock(end['t'] - soft['t'])} | | "
            f"{end['mah'] - soft['mah']:.0f} mAh |")
    add('')

    if run.notes:
        add('## Observations')
        add('')
        for note in run.notes:
            add(f'- {note}')
        add('')

    add('## Battery')
    add('')
    add('| | Start | End |')
    add('|---|---|---|')
    end_rest = (f"{run.rested_end_v:.2f} V (measured after the run)" if run.rested_end_v
                else 'not measured')
    add(f"| Rested voltage | {s['v_start_rest']:.2f} V (INA237, motors off) | {end_rest} |")
    add(f"| Walking voltage, first / last {EDGE_S:g} s (INA237) | {s['v_walk_first']:.2f} V | "
        f"{s['v_walk_last']:.2f} V |")
    add(f"| Charger reading | {_charger_text(s['charger_start'])} (rested) | "
        f"{_charger_text(s['charger_last'])} (last) |")
    add(f"| Lowest voltage while walking | | {s['v_min']:.2f} V |")
    add('')

    add('## Consumption while walking')
    add('')
    add('| | |')
    add('|---|---|')
    add(f"| Runtime | {_hms(s['walk_s'])} |")
    add(f"| Charge used | {s['charge_mah']:.0f} mAh |")
    if stop:
        add(f"| Charge used to the end, with the motors stopped | {end['mah']:.0f} mAh |")
    if soft:
        add(f"| Charge used down to the software cutoff | {soft['mah']:.0f} mAh, "
            f"in {_hms(soft['t'] - run.phases['walking'][0])} |")
    add(f"| Energy used | {s['energy_wh']:.2f} Wh |")
    add(f"| Average current | {s['avg_ma']:.0f} mA |")
    add(f"| Median current | {s['median_ma']:.0f} mA |")
    add(f"| Highest current, 1 s average | {s['peak_1s_ma']:.0f} mA |")
    add(f"| Highest current, single reading | {s['peak_ma']:.0f} mA |")
    add(f"| Average power | {s['avg_w']:.2f} W |")
    add(f"| Highest power, single reading | {s['peak_w']:.2f} W |")
    add('')

    add('## For comparison')
    add('')
    add('| | Current |')
    add('|---|---|')
    add(f"| Motors off | {s['off_ma']:.0f} mA |")
    add(f"| Standing, motors on | {s['standing_ma']:.0f} mA |")
    add(f"| Walking | {s['avg_ma']:.0f} mA |")
    add('')

    if s['servos']:
        add('## Servo temperatures')
        add('')
        health = 'v_min' in s['servos'][0]
        add('| Slot | Leg | Start | Highest | When | End |' + (' Lowest supply | Hardware error |' if health else ''))
        add('|---|---|---|---|---|---|' + ('---|---|' if health else ''))
        for i, sv in enumerate(s['servos']):
            row = (f"| {i + 1} | {LEGS[i // 2]} | {sv['start']} °C | {sv['max']} °C | "
                   f"{_clock(sv['t_max'])} | {sv['end']} °C |")
            if health:
                err = f"{sv['error'][1]}, from {_clock(sv['error'][0])}" if sv['error'] else 'none'
                row += f" {sv['v_min']:.1f} V | {err} |"
            add(row)
        add('')
        if health:
            add("A servo with a hardware error switches its own torque off and keeps it off until "
                "it restarts. The supply is each servo's own reading of the voltage it gets.")
            add('')

    add(f'![{run.run_id} plots]({png})')
    add('')

    add('## Timeline')
    add('')
    add('| Time | |')
    add('|---|---|')
    for t, text in run.events:
        add(f'| {_clock(t)} | {text} |')
    add('')

    add('## Run details')
    add('')
    add('| | |')
    add('|---|---|')
    for k, v in run.meta.items():
        add(f'| {k} | {v} |')
    gaps = 'no gaps' if not s['gaps'] else f"{s['gaps']} gap{'s' if s['gaps'] != 1 else ''}"
    add(f"| INA237 readings | {s['readings']} ({s['rate_hz']:.1f} per second); "
        f"{gaps} over {GAP_S:g} s, longest pause {s['longest_gap_s']:.1f} s |")
    add(f"| Charger readings | {s['charger_readings']} |")
    if s['servos']:
        add(f"| Servo readings | {s['servo_readings']} |")
    add(f"| INA237 die temperature | {s['die_start']:.1f} °C at the start, {s['die_max']:.1f} °C highest |")
    add('')
    files = [ina_csv, charger_csv] + ([servo_csv] if s['servos'] else [])
    add('Times are minutes:seconds from when walking started. Charge and energy are the '
        'INA237 readings integrated over time; across a gap the reading is taken to change '
        'in a straight line. Readings: ' + ', '.join(f'[{f}]({f})' for f in files) + '.')
    add('')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def _phase_lookup(run):
    def phase_of(t):
        for name, (a, b) in run.phases.items():
            if a <= t <= b:
                return name
        return ''
    return phase_of


def write_csvs(run, ina_path, charger_path, servo_path):
    phase_of = _phase_lookup(run)
    used = _used_mah(run)
    start = run.phases['walking'][0]
    with open(ina_path, 'w', newline='') as f:
        w = csv.writer(f)
        # charge_used_mah counts from when walking started - blank before then
        w.writerow(['t_s', 'phase', 'bus_v', 'current_ma', 'power_mw', 'die_c', 'charge_used_mah'])
        for i, t in enumerate(run.t):
            w.writerow([f'{t:.3f}', phase_of(t), f'{run.bus_v[i]:.3f}', f'{run.current_a[i] * 1000:.1f}',
                        f'{run.power_w[i] * 1000:.0f}', f'{run.die_c[i]:.1f}',
                        f'{used[i]:.2f}' if t >= start else ''])
    with open(charger_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'phase', 'percent', 'volts'])
        for t, pct, v in run.battery:
            w.writerow([f'{t:.3f}', phase_of(t), pct, f'{v:.3f}'])
    if run.servo_t is not None and len(run.servo_t):
        with open(servo_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'phase'] + [f'slot{i + 1}_c' for i in range(run.servo_c.shape[1])])
            for t, row in zip(run.servo_t, run.servo_c):
                w.writerow([f'{t:.3f}', phase_of(t)] + [int(c) for c in row])


# ---------- plot ----------

def plot(run, path):
    s = run.stats
    m = run.t / 60
    panels = 5
    height = 1.4 + 2.5 * panels
    fig, axes = plt.subplots(panels, 1, figsize=(11, height), sharex=True,
                             gridspec_kw={'hspace': 0.12})
    fig.subplots_adjust(top=1 - 1.38 / height, bottom=0.7 / height, left=0.08, right=0.9)

    title = f"{run.run_id} - battery runtime, {run.setup}"
    sub = (f"{_hms(s['walk_s'])} walking  ·  {s['charge_mah']:.0f} mAh  ·  "
           f"{s['avg_ma']:.0f} mA average  ·  {s['avg_w']:.1f} W average  ·  "
           f"{s['v_start_rest']:.2f} V rested → {s['v_last']:.2f} V last reading")
    if s['servos']:
        sub += f"  ·  hottest servo {max(sv['max'] for sv in s['servos'])} °C"
    fig.text(0.08, 1 - 0.52 / height, title, fontsize=15, weight='bold')
    fig.text(0.08, 1 - 0.8 / height, sub, fontsize=10.5, color='#3b434c')
    if run.sample_data:
        fig.text(0.9, 1 - 0.52 / height, 'SAMPLE DATA - not a measurement', fontsize=10,
                 weight='bold', color='#b04a2f', ha='right')

    soft, end = s['soft'], s['end']
    for ax in axes:
        for name, (a, b) in run.phases.items():
            if name != 'walking':
                ax.axvspan(a / 60, b / 60, color=COLORS['shade'], zorder=0, lw=0)
        for t, _ in run.events:
            ax.axvline(t / 60, color=COLORS['event'], lw=0.6, ls=':', zorder=1)
        if soft:
            ax.axvline(soft['t'] / 60, color=COLORS['soft'], lw=1.2, ls='--', zorder=2)
        ax.axvline(end['t'] / 60, color=COLORS['end'], lw=1.2, zorder=2)
        ax.grid(True, color='#dde2e7', lw=0.6)
        ax.spines[['top', 'right']].set_visible(False)

    smooth = lambda y: _rolling_mean(run.t, y, SMOOTH_S)

    def spread(ax, y, color, label=None):
        c, lo, hi = _band(run.t, y, SMOOTH_S)
        if len(c):
            ax.fill_between(c / 60, lo, hi, color=color, alpha=0.16, lw=0, label=label)

    ax = axes[0]
    spread(ax, run.current_a, COLORS['current'], 'middle 80% of readings')
    ax.plot(m, smooth(run.current_a), color=COLORS['current'], lw=1.6, label=f'{SMOOTH_S:g} s average')
    ax.axhline(s['avg_ma'] / 1000, color=COLORS['current'], lw=0.9, ls='--')
    ax.text(0.03, s['avg_ma'] / 1000, f"walking average {s['avg_ma'] / 1000:.2f} A",
            transform=ax.get_yaxis_transform(),   # x across the plot, y in amps
            va='center', ha='left', fontsize=8.5, color=COLORS['current'],
            bbox=dict(facecolor='white', edgecolor='none', pad=1.5))
    ax.set_ylabel('Current (A)')
    ax.set_ylim(bottom=0)
    ax.legend(loc='upper left', fontsize=8.5, frameon=False, ncol=2)

    ax = axes[1]
    spread(ax, run.bus_v, COLORS['voltage'])
    ax.plot(m, smooth(run.bus_v), color=COLORS['voltage'], lw=1.6, label='INA237')
    if run.battery:
        ax.plot([r[0] / 60 for r in run.battery], [r[2] for r in run.battery], 'o', ms=1.8,
                color=COLORS['charger'], alpha=0.7, label='charger')
    if run.health_t is not None and len(run.health_t):
        ax.plot(run.health_t / 60, run.servo_v.min(axis=1), color=COLORS['event'], lw=1.2,
                label="servos' supply (lowest)")
    for v, color, text in ((SOFT_CUTOFF_V, COLORS['soft'], 'software cutoff'),
                           (UVP_V, COLORS['end'], 'PCB UVP')):
        ax.axhline(v, color=color, lw=1.0, ls='--' if color == COLORS['soft'] else '-')
        ax.text(1.005, v, f'{text}\n{v:.2f} V', transform=ax.get_yaxis_transform(),
                va='center', ha='left', fontsize=7.5, color=color)
    ax.set_ylim(min(UVP_V, s['v_min']) - 0.15, None)
    ax.set_ylabel('Voltage (V)')
    ax.legend(loc='upper right', fontsize=8.5, frameon=False, ncol=2)

    ax = axes[2]
    spread(ax, run.power_w, COLORS['power'])
    ax.plot(m, smooth(run.power_w), color=COLORS['power'], lw=1.6)
    ax.set_ylabel('Power (W)')
    ax.set_ylim(bottom=0)

    ax = axes[3]
    used = _used_mah(run)
    after = run.t >= run.phases['walking'][0]
    ax.plot(m[after], used[after], color=COLORS['charge'], lw=1.8)
    if soft:
        ax.plot(soft['t'] / 60, soft['mah'], 'o', color=COLORS['soft'], ms=5)
        ax.annotate(f"{soft['mah']:.0f} mAh", (soft['t'] / 60, soft['mah']), xytext=(-8, 6),
                    textcoords='offset points', ha='right', fontsize=8, color=COLORS['soft'])
    if s['motor_stop']:
        ms = s['motor_stop']
        ax.plot(ms['t'] / 60, ms['mah'], 'o', color=COLORS['event'], ms=5)
        ax.annotate(f"{ms['mah']:.0f} mAh", (ms['t'] / 60, ms['mah']), xytext=(-8, -28),
                    textcoords='offset points', ha='right', fontsize=8, color=COLORS['event'])
    ax.plot(end['t'] / 60, end['mah'], 'o', color=COLORS['end'], ms=5)
    ax.annotate(f"{end['mah']:.0f} mAh", (end['t'] / 60, end['mah']), xytext=(-8, 6),
                textcoords='offset points', ha='right', fontsize=8, color=COLORS['end'])
    ax.set_ylabel('Charge used (mAh)')
    ax.set_ylim(bottom=0)

    # Every temperature on one scale: the INA237's chip always, the servos
    # when they were read.
    ax = axes[4]
    die = smooth(run.die_c)
    chip = ax.plot(m, die, color=COLORS['die'], lw=1.6, label='INA237 chip')
    lows, highs = [die.min()], [die.max()]
    if s['servos']:
        servo_lines = []
        for i in range(run.servo_c.shape[1]):
            servo_lines += ax.plot(run.servo_t / 60, run.servo_c[:, i], color=LEG_COLORS[i // 2],
                                   ls='-' if i % 2 == 0 else '--', lw=1.6,
                                   label=f'slot {i + 1} ({LEGS[i // 2]})')
        lows.append(run.servo_c.min())
        highs.append(run.servo_c.max())
        ax.add_artist(ax.legend(handles=servo_lines, loc='upper left', fontsize=7.5,
                                frameon=False, ncol=4))
    ax.legend(handles=chip, loc='upper right', fontsize=7.5, frameon=False)
    ax.set_ylabel('Temperature (°C)')
    lo, hi = min(lows), max(highs)
    ax.set_ylim(lo - 2, hi + 0.35 * (hi - lo + 4))
    axes[-1].set_xlabel('Minutes since walking started')

    # Labels along the top: the software cutoff, the end, and any run event
    # during the walk - short, the timeline has the rest (and the events
    # before the walk, too close together at this scale). A label that
    # would overlap one already placed goes up a row.
    top = axes[0]
    w0 = run.phases['walking'][0]
    marks = [(t, re.split(r' \(| - ', text)[0], COLORS['event']) for t, text in run.events
             if w0 < t < end['t'] and not (soft and t == soft['t'])]
    if soft:
        marks.append((soft['t'], 'software cutoff would power off', COLORS['soft']))
    marks.append((end['t'], run.end_mark, COLORS['end']))
    marks.sort(key=lambda mk: mk[0])
    renderer = fig.canvas.get_renderer()
    placed = []
    for t, text, color in marks:
        for row in range(4):
            label = top.text(t / 60, 1.02 + 0.07 * row, text, transform=top.get_xaxis_transform(),
                             ha='right' if t > w0 + 0.8 * (end['t'] - w0) else 'center', va='bottom',
                             fontsize=7.5, color=color)
            box = label.get_window_extent(renderer).expanded(1.15, 1.0)
            if not any(box.overlaps(p) for p in placed):
                placed.append(box)
                break
            label.remove()

    fig.savefig(path, dpi=130)
    plt.close(fig)


def write_report(run, folder):
    '''Write the report, plot and CSVs into the test folder. Returns the report's path.'''
    (folder / 'images').mkdir(parents=True, exist_ok=True)
    (folder / 'data').mkdir(parents=True, exist_ok=True)
    png = f'images/{run.run_id}-plots.png'
    ina_csv = f'data/{run.run_id}-ina237.csv'
    charger_csv = f'data/{run.run_id}-charger.csv'
    servo_csv = f'data/{run.run_id}-servos.csv'
    summarize(run)
    plot(run, folder / png)
    write_csvs(run, folder / ina_csv, folder / charger_csv, folder / servo_csv)
    report = folder / f'{run.run_id}.md'
    write_markdown(run, report, png, ina_csv, charger_csv, servo_csv)
    return report
