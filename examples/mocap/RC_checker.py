"""
RC_checker — figure out which stick/switch on an RC transmitter maps to which
HID axis/button.

Plug the transmitter (e.g. a RadioMaster running EdgeTX/OpenTX) into USB and
pick "USB Joystick (HID)" on the radio. It then shows up as a Linux joystick
device, usually /dev/input/js0.

Usage:
    python3 RC_checker.py                # live dashboard on /dev/input/js0
    python3 RC_checker.py --device /dev/input/js1
    python3 RC_checker.py --raw          # one line per event, easy to copy/paste
    python3 RC_checker.py --list         # list joystick devices and exit

In the live view: wiggle one control at a time. The axis or button that just
moved is highlighted and appended to the "recent changes" log, so you can write
down e.g. "SD switch -> axis 6".

Pure standard library: reads the Linux joystick API (/dev/input/jsN) directly,
8 bytes per event (time, value, type, number). No pygame / evdev required.
Ctrl-C to quit.
"""
import argparse
import array
import fcntl
import glob
import os
import select
import struct
import sys
import time

# /dev/input/jsN event: __u32 time, __s16 value, __u8 type, __u8 number
JS_EVENT_FMT = '=IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80  # OR'ed in for the synthetic events sent right after open

# ioctls from linux/joystick.h
JSIOCGAXES = 0x80016A11      # _IOR('j', 0x11, __u8)
JSIOCGBUTTONS = 0x80016A12   # _IOR('j', 0x12, __u8)
JSIOCGNAME_LEN = 128
JSIOCGNAME = 0x80006A13 | (JSIOCGNAME_LEN << 16)

AXIS_MAX = 32767.0

# ANSI helpers
CLEAR = '\033[2J'
HOME = '\033[H'
CLR_EOL = '\033[K'
HIDE_CUR = '\033[?25l'
SHOW_CUR = '\033[?25h'
BOLD = '\033[1m'
DIM = '\033[2m'
HILITE = '\033[7m'
RESET = '\033[0m'

# Typical EdgeTX/OpenTX joystick channel order (mode 2, default mixer).
# Only a hint -- the real mapping depends on the radio's channel setup, which is
# exactly what this script is for.
AXIS_HINTS = [
    'ch1  aileron / right stick X',
    'ch2  elevator / right stick Y',
    'ch3  throttle / left stick Y',
    'ch4  rudder / left stick X',
    'ch5  aux (switch?)',
    'ch6  aux (switch?)',
    'ch7  aux (switch?)',
    'ch8  aux (switch?)',
]


def list_devices():
    devs = sorted(glob.glob('/dev/input/js*'))
    if not devs:
        print('No /dev/input/js* devices found.')
        print('Is the radio plugged in and set to USB Joystick (HID) mode?')
        return
    for dev in devs:
        try:
            with open(dev, 'rb') as fd:
                name, axes, buttons = device_info(fd)
            print(f'{dev}: "{name}"  {axes} axes, {buttons} buttons')
        except OSError as err:
            print(f'{dev}: <cannot open: {err}>')


def device_info(fd):
    buf = array.array('B', [0] * JSIOCGNAME_LEN)
    try:
        fcntl.ioctl(fd, JSIOCGNAME, buf)
        name = buf.tobytes().rstrip(b'\x00').decode('utf-8', 'replace')
    except OSError:
        name = 'unknown'

    n = array.array('B', [0])
    fcntl.ioctl(fd, JSIOCGAXES, n)
    axes = n[0]
    fcntl.ioctl(fd, JSIOCGBUTTONS, n)
    buttons = n[0]
    return name, axes, buttons


def bar(value, width=21):
    """Centre-zero bar for a normalised (-1..1) axis value."""
    mid = width // 2
    pos = int(round((value + 1.0) / 2.0 * (width - 1)))
    pos = max(0, min(width - 1, pos))
    cells = ['-'] * width
    cells[mid] = '|'
    cells[pos] = '#'
    return '[' + ''.join(cells) + ']'


def switch_guess(seen):
    """Guess whether an axis behaves like a 2/3-position switch."""
    if len(seen) == 0:
        return ''
    if len(seen) == 1:
        return 'idle'
    if len(seen) == 2:
        return '2-pos switch?'
    if len(seen) == 3:
        return '3-pos switch?'
    if len(seen) <= 6:
        return f'{len(seen)}-pos switch?'
    return 'analog (stick/pot)'


def run_raw(fd):
    print('Raw event stream. Move one control at a time. Ctrl-C to quit.\n')
    print(f'{"time":>9}  {"kind":<6} {"idx":>3}  {"raw":>7}  {"norm":>7}')
    t0 = time.time()
    while True:
        data = fd.read(JS_EVENT_SIZE)
        if not data or len(data) < JS_EVENT_SIZE:
            return
        _, value, ev_type, number = struct.unpack(JS_EVENT_FMT, data)
        init = ev_type & JS_EVENT_INIT
        kind = 'axis' if ev_type & JS_EVENT_AXIS else 'button'
        norm = value / AXIS_MAX if kind == 'axis' else value
        tag = '  (initial state)' if init else ''
        print(f'{time.time() - t0:9.3f}  {kind:<6} {number:>3}  {value:>7}  {norm:>7.3f}{tag}')


def run_dashboard(fd, path, name, n_axes, n_buttons, deadzone):
    axes = [0] * n_axes
    buttons = [0] * n_buttons
    axis_seen = [set() for _ in range(n_axes)]
    axis_min = [None] * n_axes
    axis_max = [None] * n_axes
    last_axis_change = [0.0] * n_axes
    last_button_change = [0.0] * n_buttons
    history = []  # (timestamp, text)
    ready = False  # becomes True once the initial state burst is done

    sys.stdout.write(CLEAR + HIDE_CUR)
    next_draw = 0.0

    while True:
        readable, _, _ = select.select([fd], [], [], 0.02)
        now = time.time()

        while readable:
            data = fd.read(JS_EVENT_SIZE)
            if not data or len(data) < JS_EVENT_SIZE:
                return
            _, value, ev_type, number = struct.unpack(JS_EVENT_FMT, data)
            init = bool(ev_type & JS_EVENT_INIT)
            if not init:
                ready = True

            if ev_type & JS_EVENT_AXIS and number < n_axes:
                norm = value / AXIS_MAX
                prev = axes[number]
                axes[number] = norm
                lo, hi = axis_min[number], axis_max[number]
                axis_min[number] = norm if lo is None else min(lo, norm)
                axis_max[number] = norm if hi is None else max(hi, norm)
                # Quantise so a noisy stick does not look like 500 switch positions
                axis_seen[number].add(round(norm, 1))
                if not init and abs(norm - prev) > deadzone:
                    last_axis_change[number] = now
                    if ready:
                        history.append((now, f'axis {number:>2}  {prev:+.3f} -> {norm:+.3f}'))
            elif ev_type & JS_EVENT_BUTTON and number < n_buttons:
                prev = buttons[number]
                buttons[number] = value
                if not init and value != prev:
                    last_button_change[number] = now
                    if ready:
                        state = 'pressed' if value else 'released'
                        history.append((now, f'btn  {number:>2}  {state}'))

            readable, _, _ = select.select([fd], [], [], 0)

        history = history[-12:]

        if now < next_draw:
            continue
        next_draw = now + 0.05

        out = [HOME]
        out.append(f'{BOLD}RC_checker{RESET}  "{name}"  {path}   '
                   f'{n_axes} axes, {n_buttons} buttons{CLR_EOL}')
        out.append(f'{DIM}Move one control at a time; the highlighted row is the one that just '
                   f'changed. Ctrl-C to quit.{RESET}{CLR_EOL}')
        out.append(CLR_EOL)
        out.append(f'{BOLD}AXES{RESET}{CLR_EOL}')
        out.append(f'{DIM}  idx  value    bar                     range seen        behaviour'
                   f'        hint{RESET}{CLR_EOL}')

        for i in range(n_axes):
            hot = (now - last_axis_change[i]) < 1.0
            hint = AXIS_HINTS[i] if i < len(AXIS_HINTS) else ''
            rng = ('  --  ' if axis_min[i] is None
                   else f'{axis_min[i]:+.2f}..{axis_max[i]:+.2f}')
            row = (f'  {i:>3}  {axes[i]:+.3f}  {bar(axes[i])}  {rng:<17} '
                   f'{switch_guess(axis_seen[i]):<16} {hint}')
            out.append((HILITE + row + RESET if hot else row) + CLR_EOL)

        out.append(CLR_EOL)
        out.append(f'{BOLD}BUTTONS{RESET}{CLR_EOL}')
        if n_buttons:
            line = '  '
            for i in range(n_buttons):
                hot = (now - last_button_change[i]) < 1.0
                cell = f'{i:>2}:{"X" if buttons[i] else "."}'
                line += (HILITE + cell + RESET if hot else cell) + '  '
                if (i + 1) % 12 == 0:
                    out.append(line + CLR_EOL)
                    line = '  '
            if line.strip():
                out.append(line + CLR_EOL)
        else:
            out.append(f'{DIM}  (none reported by this device){RESET}{CLR_EOL}')

        out.append(CLR_EOL)
        out.append(f'{BOLD}RECENT CHANGES{RESET}{CLR_EOL}')
        for ts, text in history:
            out.append(f'  {time.strftime("%H:%M:%S", time.localtime(ts))}  {text}{CLR_EOL}')
        for _ in range(12 - len(history)):
            out.append(CLR_EOL)

        sys.stdout.write('\n'.join(out))
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(
        description='Show which RC stick/switch maps to which HID axis/button.')
    parser.add_argument('--device', '-d', default='/dev/input/js0',
                        help='joystick device (default: /dev/input/js0)')
    parser.add_argument('--raw', action='store_true',
                        help='print one line per event instead of the live dashboard')
    parser.add_argument('--list', action='store_true',
                        help='list joystick devices and exit')
    parser.add_argument('--deadzone', type=float, default=0.02,
                        help='ignore axis moves smaller than this (default: 0.02)')
    args = parser.parse_args()

    if args.list:
        list_devices()
        return 0

    if not os.path.exists(args.device):
        print(f'{args.device} does not exist.', file=sys.stderr)
        print('Plug the radio in, set it to USB Joystick (HID) mode, then try --list.',
              file=sys.stderr)
        return 1

    try:
        # Unbuffered: the dashboard uses select() on this fd, which only sees
        # data that has not already been slurped into a Python-side buffer.
        fd = open(args.device, 'rb', buffering=0)
    except PermissionError:
        print(f'No permission to read {args.device}.', file=sys.stderr)
        print('Fix with: sudo usermod -aG input $USER   (then log out/in)', file=sys.stderr)
        return 1

    with fd:
        name, n_axes, n_buttons = device_info(fd)
        try:
            if args.raw:
                run_raw(fd)
            else:
                run_dashboard(fd, args.device, name, n_axes, n_buttons, args.deadzone)
        except (KeyboardInterrupt, BrokenPipeError):
            pass
        finally:
            if not args.raw:
                try:
                    sys.stdout.write(SHOW_CUR + '\n')
                    sys.stdout.flush()
                except BrokenPipeError:
                    pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
