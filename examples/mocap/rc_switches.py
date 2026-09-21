"""
rc_switches — read the arm / launch / kill switches off an RC transmitter in
USB HID (joystick) mode and expose them as plain boolean flags to other scripts.

The transmitter is polled in a background thread, so a flight loop never blocks
on it: it just reads properties.

Default switch mapping (found with RC_checker.py on a RadioMaster Zorro):
    axis 4 positive -> ARM
    axis 5 positive -> LAUNCH
    axis 6 positive -> KILL

Typical use:

    from rc_switches import RCSwitches

    with RCSwitches() as rc:
        rc.wait_for_arm()                     # flick the arm switch up
        cf.platform.send_arming_request(True)

        rc.wait_for_launch()                  # flick the launch switch up
        commander.takeoff(1.0, 2.0)

        while not rc.kill:                    # fly
            ...
            rc.raise_if_kill()                # or let it raise out of the loop

        cf.commander.send_stop_setpoint()     # kill switch flicked -> motors off

Safety behaviour:
  * KILL latches. Once the switch goes positive the flag stays True even if the
    switch is flicked back, until reset_kill() is called explicitly.
  * If the transmitter is unplugged or the device stops reading, KILL is
    asserted (fail-safe). Pass fail_safe_kill=False to opt out.
  * KILL is evaluated before anything else: should_arm / should_launch are
    always False once killed.

Run it directly to watch the flags live:
    python3 rc_switches.py
"""
import argparse
import os
import select
import struct
import sys
import threading
import time

# /dev/input/jsN event: __u32 time, __s16 value, __u8 type, __u8 number
JS_EVENT_FMT = '=IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_MAX = 32767.0

DEFAULT_DEVICE = '/dev/input/js0'
DEFAULT_ARM_AXIS = 4
DEFAULT_LAUNCH_AXIS = 5
DEFAULT_KILL_AXIS = 6
DEFAULT_THRESHOLD = 0.5


class KillSwitchError(Exception):
    """Raised by raise_if_kill() when the kill switch has been flicked."""


class RCSwitches:
    """Background reader turning three transmitter switches into flags.

    Args:
        device: joystick device path.
        arm_axis, launch_axis, kill_axis: axis indices (see RC_checker.py).
            Set any of them to None to disable that flag.
        threshold: an axis counts as "positive" above this (axes run -1..+1).
        invert: iterable of axis indices whose sense is flipped, for a switch
            that reads negative in the position you want.
        latch_kill: keep kill True once triggered (default True).
        fail_safe_kill: assert kill if the device cannot be read (default True).
        on_arm / on_launch / on_kill: optional callables invoked on the rising
            edge of that flag, from the reader thread. Keep them short.
    """

    def __init__(self,
                 device=DEFAULT_DEVICE,
                 arm_axis=DEFAULT_ARM_AXIS,
                 launch_axis=DEFAULT_LAUNCH_AXIS,
                 kill_axis=DEFAULT_KILL_AXIS,
                 threshold=DEFAULT_THRESHOLD,
                 invert=(),
                 latch_kill=True,
                 fail_safe_kill=True,
                 on_arm=None,
                 on_launch=None,
                 on_kill=None):
        self.device = device
        self.arm_axis = arm_axis
        self.launch_axis = launch_axis
        self.kill_axis = kill_axis
        self.threshold = threshold
        self.invert = set(invert)
        self.latch_kill = latch_kill
        self.fail_safe_kill = fail_safe_kill
        self._callbacks = {'arm': on_arm, 'launch': on_launch, 'kill': on_kill}

        self._lock = threading.Lock()
        self._axes = {}
        self._flags = {'arm': False, 'launch': False, 'kill': False}
        self._edges = {'arm': False, 'launch': False, 'kill': False}
        self._events = {name: threading.Event() for name in self._flags}
        self._connected = False
        self._last_event_time = 0.0

        self._stop = threading.Event()
        self._thread = None

    # ---------------------------------------------------------------- flags

    @property
    def arm(self):
        """Arm switch is positive (and kill has not been triggered)."""
        with self._lock:
            return self._flags['arm'] and not self._flags['kill']

    @property
    def launch(self):
        """Launch switch is positive (and kill has not been triggered)."""
        with self._lock:
            return self._flags['launch'] and not self._flags['kill']

    @property
    def kill(self):
        """Kill switch has been flicked (latched by default)."""
        with self._lock:
            return self._flags['kill']

    @property
    def should_launch(self):
        """Armed AND launch requested AND not killed -- the safe gate to fly on."""
        with self._lock:
            f = self._flags
            return f['arm'] and f['launch'] and not f['kill']

    @property
    def connected(self):
        """The joystick device is open and readable."""
        with self._lock:
            return self._connected

    @property
    def axes(self):
        """Snapshot of every axis seen so far, {index: value in -1..1}."""
        with self._lock:
            return dict(self._axes)

    def axis(self, index, default=0.0):
        with self._lock:
            return self._axes.get(index, default)

    def state(self):
        """One-word summary: 'killed', 'flying', 'armed' or 'disarmed'."""
        with self._lock:
            f = dict(self._flags)
        if f['kill']:
            return 'killed'
        if f['arm'] and f['launch']:
            return 'flying'
        if f['arm']:
            return 'armed'
        return 'disarmed'

    # ---------------------------------------------------------------- edges

    def consume_edge(self, name):
        """True once per rising edge of 'arm' / 'launch' / 'kill', then False.

        Use it in a polling loop to act on a flick rather than on a level.
        """
        with self._lock:
            hit = self._edges[name]
            self._edges[name] = False
            return hit

    # ---------------------------------------------------------------- waits

    def wait_for(self, name, timeout=None):
        """Block until flag `name` goes True. Returns True, or False on timeout.

        Raises KillSwitchError if the kill switch fires while waiting for
        something else.
        """
        deadline = None if timeout is None else time.time() + timeout
        while True:
            if getattr(self, name):
                return True
            if name != 'kill' and self.kill:
                raise KillSwitchError('kill switch active while waiting for ' + name)
            remaining = None if deadline is None else deadline - time.time()
            if remaining is not None and remaining <= 0:
                return False
            self._events[name].wait(0.05 if remaining is None else min(0.05, remaining))

    def wait_for_arm(self, timeout=None):
        return self.wait_for('arm', timeout)

    def wait_for_launch(self, timeout=None):
        return self.wait_for('launch', timeout)

    def wait_for_kill(self, timeout=None):
        return self.wait_for('kill', timeout)

    def raise_if_kill(self):
        """Raise KillSwitchError if the kill switch has fired. Call it in loops."""
        if self.kill:
            raise KillSwitchError('kill switch active')

    def reset_kill(self):
        """Clear a latched kill. The switch must be back in its safe position."""
        with self._lock:
            live = self._axis_active(self.kill_axis)
            if live:
                return False
            self._flags['kill'] = False
            self._edges['kill'] = False
            self._events['kill'].clear()
            return True

    # -------------------------------------------------------------- control

    def start(self):
        if self._thread is not None:
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='RCSwitches', daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    # -------------------------------------------------------------- internals

    def _axis_active(self, index):
        """Caller must hold the lock."""
        if index is None:
            return False
        value = self._axes.get(index)
        if value is None:
            return False
        if index in self.invert:
            value = -value
        return value > self.threshold

    def _set_flag(self, name, value):
        """Caller must hold the lock. Returns a callback to fire, or None."""
        if name == 'kill' and self.latch_kill and self._flags['kill']:
            return None
        if value == self._flags[name]:
            return None
        self._flags[name] = value
        if value:
            self._edges[name] = True
            self._events[name].set()
            return self._callbacks[name]
        self._events[name].clear()
        return None

    def _recompute(self):
        """Caller must hold the lock. Returns callbacks to fire outside it."""
        fire = []
        for name, index in (('kill', self.kill_axis),
                            ('arm', self.arm_axis),
                            ('launch', self.launch_axis)):
            cb = self._set_flag(name, self._axis_active(index))
            if cb is not None:
                fire.append(cb)
        return fire

    def _trigger_fail_safe(self):
        if not self.fail_safe_kill:
            return
        with self._lock:
            cb = self._set_flag('kill', True)
        if cb is not None:
            cb()

    def _run(self):
        fd = None
        while not self._stop.is_set():
            if fd is None:
                try:
                    # Unbuffered: select() must see the real device state.
                    fd = open(self.device, 'rb', buffering=0)
                except OSError:
                    self._trigger_fail_safe()
                    with self._lock:
                        self._connected = False
                    self._stop.wait(0.5)
                    continue
                with self._lock:
                    self._connected = True

            try:
                readable, _, _ = select.select([fd], [], [], 0.05)
                if not readable:
                    continue
                data = fd.read(JS_EVENT_SIZE)
                if not data or len(data) < JS_EVENT_SIZE:
                    raise OSError('short read from ' + self.device)
            except OSError:
                try:
                    fd.close()
                except OSError:
                    pass
                fd = None
                with self._lock:
                    self._connected = False
                self._trigger_fail_safe()
                continue

            _, value, ev_type, number = struct.unpack(JS_EVENT_FMT, data)
            if not ev_type & JS_EVENT_AXIS:
                continue

            with self._lock:
                self._axes[number] = value / AXIS_MAX
                self._last_event_time = time.time()
                fire = self._recompute()
            for cb in fire:
                try:
                    cb()
                except Exception as err:  # never let a callback kill the reader
                    print(f'rc_switches: callback error: {err}', file=sys.stderr)

        if fd is not None:
            fd.close()
        with self._lock:
            self._connected = False


def _demo(args):
    if not os.path.exists(args.device):
        print(f'{args.device} not found -- plug the radio in and select USB Joystick (HID).',
              file=sys.stderr)
        return 1

    print(f'Watching {args.device}: arm=axis {args.arm_axis}, launch=axis {args.launch_axis}, '
          f'kill=axis {args.kill_axis}.  Ctrl-C to quit.\n')
    with RCSwitches(device=args.device,
                    arm_axis=args.arm_axis,
                    launch_axis=args.launch_axis,
                    kill_axis=args.kill_axis,
                    latch_kill=not args.no_latch) as rc:
        try:
            while True:
                flags = (f'arm={"YES" if rc.arm else " no"}  '
                         f'launch={"YES" if rc.launch else " no"}  '
                         f'kill={"KILL" if rc.kill else "  ok"}')
                raw = '  '.join(f'ax{i}={rc.axis(i):+.2f}'
                                for i in (args.arm_axis, args.launch_axis, args.kill_axis)
                                if i is not None)
                link = 'linked' if rc.connected else 'NO LINK'
                sys.stdout.write(f'\r{flags}   state={rc.state():<9} [{raw}]  {link}   ')
                sys.stdout.flush()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print()
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--device', '-d', default=DEFAULT_DEVICE)
    parser.add_argument('--arm-axis', type=int, default=DEFAULT_ARM_AXIS)
    parser.add_argument('--launch-axis', type=int, default=DEFAULT_LAUNCH_AXIS)
    parser.add_argument('--kill-axis', type=int, default=DEFAULT_KILL_AXIS)
    parser.add_argument('--no-latch', action='store_true',
                        help='do not latch the kill flag (for testing only)')
    sys.exit(_demo(parser.parse_args()))
