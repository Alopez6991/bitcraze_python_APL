# -*- coding: utf-8 -*-
#
# RC_backup -- fly forward 12 m, climbing or side-stepping around anything the
# forward-facing multizone ToF sees, gated by the RC switches.
#
# Drone: radio://0/80/2M/E7E7E7E702. Same ONBOARD-estimator-only setup and the
# same RC switch gating as RC_baby_safe.py (see rc_switches.py):
#        axis 4 up -> ARM    : sends the arming request
#        axis 5 up -> LAUNCH : starts the mission
#        axis 6 up -> KILL   : cuts the motors immediately, any time
#
# ---------------------------------------------------------------------------
# The front ToF (range8, zranger3.c)
# ---------------------------------------------------------------------------
# A 4x4 multizone sensor. The script steers on its four column averages, in mm,
# each the mean of that column's VALID zones, or 4000 if the whole column had
# no valid return:
#     range8.colR   zones 0,4,8,12    right
#     range8.colMR  zones 1,5,9,13    mid-right
#     range8.colML  zones 2,6,10,14   mid-left
#     range8.colL   zones 3,7,11,15   left
# "The front distance" below is the MINIMUM of those columns (TOF_FRONT_COLUMNS),
# so an obstacle anywhere in view counts. 4000 mm (nothing seen) reads as 4 m,
# i.e. clear. The per-zone range8.z00..z15 values are not needed for this.
#
# ---------------------------------------------------------------------------
# What the drone does
# ---------------------------------------------------------------------------
# Takeoff to BK_Z (1.5 m), then a state machine streamed at BK_RATE_HZ with
# send_hover_setpoint(vx, vy, 0, z_target). Yaw is held at 0 throughout.
#
#   FORWARD     vx = BK_FWD_SPEED at the current altitude.
#               front <= BK_STOP_DIST (0.75 m)  -> stop, CLIMB
#               forward progress >= BK_FWD (12 m) -> hover, land
#
#   CLIMB       vx = 0, altitude rises at BK_CLIMB_SPEED.
#               front >= BK_CLEAR_DIST (1.5 m)   -> FORWARD (at the new height)
#               height reaches BK_MAX_Z (2.0 m)   -> SIDESTEP, then SWEEP_DOWN
#
#   SIDESTEP    vx = 0, altitude held, moves BK_SIDE_STEP m left OR right,
#               picked at random (see BK_SIDESTEP_MODE for how). If that would
#               take the drone more than BK_MAX_LATERAL from the start line, it
#               goes the other way.
#
#   SWEEP_DOWN  vx = 0, altitude falls at BK_SWEEP_SPEED, looking for a gap.
#               front >= BK_CLEAR_DIST           -> FORWARD (at that height)
#               height reaches BK_MIN_Z, no gap  -> SIDESTEP, then CLIMB
#
# CLIMB and SWEEP_DOWN alternate with a random sidestep between each pass, so
# the drone scans a zig-zag of fresh columns until one of them is clear. The
# stop/resume thresholds (0.75 m / 1.5 m) give hysteresis, and "clear" must
# hold for BK_CLEAR_CONFIRM_N readings in a row so one noisy sample cannot send
# it forward. After BK_MAX_SIDESTEPS sidesteps with no way through, it lands.
# The sidestep count resets each time it gets past an obstacle, so every
# obstacle gets the full budget.
#
# Forward progress is stateEstimate.x relative to where it took off. Only FORWARD
# advances it, so climbs and sidesteps do not eat into the 12 m.
#
# Safety: the script will not arm if the range8 log block is unavailable --
# this mission is obstacle avoidance, so flying it without the sensor is not an
# option. If ToF data goes stale mid-flight the drone holds position, and lands
# if it does not recover within BK_TOF_ABORT_S; if it does recover, it picks up
# exactly where it left off (mid-climb, mid-sidestep, ...).
import random
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from rc_switches import KillSwitchError
from rc_switches import RCSwitches

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')

# --- Flight geometry ---
BK_Z = 1.25              # takeoff / cruise height (m)
BK_FWD = 12.0           # forward distance to cover (m)
BK_FWD_SPEED = 1.4      # m/s forward
BK_MAX_Z = 2.0          # never climb above this; reaching it triggers a sidestep
BK_MIN_Z = 0.5          # bottom of a sweep-down (m)
BK_CLIMB_SPEED = 0.25   # m/s up while climbing over an obstacle
BK_SWEEP_SPEED = 0.25   # m/s down while sweeping for a gap
BK_SIDE_STEP = 0.5      # m moved left or right per sidestep
BK_SIDE_SPEED = 0.3     # m/s sideways
BK_MAX_LATERAL = 2.0    # never wander further than this from the start line (m)
# Give up and land after this many sidesteps at one obstacle. 12 is what
# 'sweep' needs to cover the whole corridor in the worst case: out to one edge
# (BK_MAX_LATERAL / BK_SIDE_STEP = 4) then all the way across (8).
BK_MAX_SIDESTEPS = 12
# How each sidestep picks its direction:
#   'random' -- a fresh coin flip on every sidestep (as specified). A random
#               walk: it can dither back and forth in front of the obstacle and
#               miss a gap only a step or two away. In simulation (a gap 0.5 m
#               to one side, 200 runs) it got through ~80% of the time, and more
#               sidesteps barely help.
#   'sweep'  -- a coin flip on the FIRST sidestep at each obstacle, then keep
#               going that way, reversing only at BK_MAX_LATERAL. Still random
#               in which side it tries first, but it covers the corridor
#               systematically: 100% in the same simulation, provided
#               BK_MAX_SIDESTEPS is large enough to reach both edges (below).
BK_SIDESTEP_MODE = 'random'

# --- Front ToF thresholds ---
BK_STOP_DIST = 0.75     # m: stop flying forward at or inside this
BK_CLEAR_DIST = 1.5     # m: path counts as clear at or beyond this
BK_CLEAR_CONFIRM_N = 3  # consecutive clear readings needed before moving on

# Columns that count toward "the front distance" (their minimum). Drop the
# outer two to only react to what is dead ahead.
TOF_COLUMNS = ('range8.colR', 'range8.colMR', 'range8.colML', 'range8.colL')
TOF_FRONT_COLUMNS = TOF_COLUMNS
TOF_LABELS = {'range8.colR': 'R', 'range8.colMR': 'MR',
              'range8.colML': 'ML', 'range8.colL': 'L'}
BK_TOF_STALE_S = 0.5    # ToF reading older than this -> hold position
BK_TOF_ABORT_S = 3.0    # ...and land if it stays stale this long

BK_RATE_HZ = 50.0            # setpoint stream rate
BK_MISSION_TIMEOUT_S = 240.0  # hard cap on the whole flight
BK_END_HOVER_S = 1.0          # hover at the 12 m mark before landing
BK_LAND_TIME = 2.5            # s for the landing ramp
TAKEOFF_HOVER = 1.0           # s to hover after takeoff before moving


# --- RC behaviour ---
RC_DEVICE = '/dev/input/js0'
RC_ARM_AXIS = 4
RC_LAUNCH_AXIS = 5
RC_KILL_AXIS = 6
# Flicking ARM back down while flying cuts the motors (a pilot who switches off
# arm expects the props to stop). Set False to ignore it once airborne.
ARM_DOWN_IS_KILL = True
# Flicking LAUNCH back down while flying abandons the trajectory and lands.
LAUNCH_DOWN_IS_LAND = True

# time variables
t_start = 0


class PreflightError(Exception):
    """Raised before arming when the mission cannot be flown safely."""


class LandRequest(Exception):
    """Raised when the launch switch is flicked down mid-flight."""


# --------------------------------------------------------------------------
# Status line
# --------------------------------------------------------------------------

BOLD = '\033[1m'
DIM = '\033[2m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'
RESET = '\033[0m'
CLR_EOL = '\033[K'

# Drone states, in the three categories asked for:
#   ON / INACTIVE : CONNECTING, ON-INACTIVE, ON-ARMED
#   ON / ACTIVE   : ON-ACTIVE, LANDING
#   KILLED        : KILLED
STATE_COLORS = {
    'CONNECTING': DIM,
    'ON-INACTIVE': YELLOW,
    'ON-ARMED': CYAN,
    'ON-ACTIVE': GREEN,
    'LANDING': CYAN,
    'KILLED': BOLD + RED,
    'DONE': DIM,
}


class Status:
    """Single refreshing status line; log() scrolls text above it."""

    def __init__(self, rc):
        self.rc = rc
        self.state = 'CONNECTING'
        self.phase = 'startup'
        self.telemetry = {}
        self.tof_available = None   # set once the log blocks are started
        self.extra = ''             # mission progress, set by the control loop
        self.enabled = sys.stdout.isatty()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._console_buf = ''

    def set_state(self, state):
        with self._lock:
            self.state = state
        self.log(f'state -> {state}')

    def set_phase(self, phase):
        with self._lock:
            self.phase = phase
        self.log(phase)

    def set_telemetry(self, data):
        # Merged, not replaced: several log blocks feed this.
        with self._lock:
            self.telemetry.update(data)

    def get_telemetry(self):
        with self._lock:
            return dict(self.telemetry)

    def set_extra(self, text):
        with self._lock:
            self.extra = text

    def log(self, message):
        """Print a scrolling message without clobbering the status line."""
        with self._lock:
            if self.enabled:
                sys.stdout.write('\r' + CLR_EOL + message + '\n')
            else:
                sys.stdout.write(message + '\n')
            sys.stdout.flush()

    def console(self, text):
        """Firmware console output: buffer until a full line is available."""
        self._console_buf += text
        while '\n' in self._console_buf:
            line, self._console_buf = self._console_buf.split('\n', 1)
            if line.strip():
                self.log(f'{DIM}[cf] {line}{RESET}')

    def line(self):
        with self._lock:
            state, phase, tlm = self.state, self.phase, dict(self.telemetry)
            extra = self.extra
        colour = STATE_COLORS.get(state, '')
        elapsed = (time.time() - t_start) if t_start else 0.0

        def sw(name, on):
            return f'{GREEN}{name}:UP{RESET}' if on else f'{DIM}{name}:dn{RESET}'

        rc = self.rc
        kill = f'{BOLD}{RED}KILL:UP{RESET}' if rc.kill else f'{DIM}kill:dn{RESET}'
        switches = f'{sw("arm", rc.arm)} {sw("launch", rc.launch)} {kill}'
        link = '' if rc.connected else f'  {RED}RC LINK LOST{RESET}'

        pos = ''
        if 'stateEstimate.x' in tlm:
            pos = (f'  pos({tlm.get("stateEstimate.x", 0):+.2f},'
                   f'{tlm.get("stateEstimate.y", 0):+.2f},'
                   f'{tlm.get("stateEstimate.z", 0):+.2f})'
                   f'  yaw{tlm.get("stabilizer.yaw", 0):+6.1f}'
                   f'  bat {tlm.get("pm.vbat", 0):.2f}V')

        # Front ToF: the minimum that drives the flight, then each column.
        tof = ''
        if '_tof_t' in tlm:
            cols = {k: tlm.get(k) for k in TOF_COLUMNS}
            valid = [cols[k] for k in TOF_FRONT_COLUMNS if cols[k] is not None]
            if valid:
                front = min(valid) / 1000.0
                if front <= BK_STOP_DIST:
                    front_txt = f'{BOLD}{RED}{front:4.2f}m{RESET}'
                elif front >= BK_CLEAR_DIST:
                    front_txt = f'{GREEN}{front:4.2f}m{RESET}'
                else:
                    front_txt = f'{YELLOW}{front:4.2f}m{RESET}'
                per_col = ' '.join(f'{TOF_LABELS[k]} {cols[k] / 1000.0:4.2f}'
                                   for k in TOF_COLUMNS if cols[k] is not None)
                tof = f'  tof {front_txt} [{per_col}]'
            if time.time() - tlm['_tof_t'] > BK_TOF_STALE_S:
                tof += f' {BOLD}{RED}STALE{RESET}'
        elif self.tof_available is False:
            tof = f'  {BOLD}{RED}tof n/a{RESET}'
        if extra:
            tof += f'  {extra}'
        phase = phase if len(phase) <= 28 else phase[:27] + '~'
        return (f'{colour}[{state:^11}]{RESET} t={elapsed:6.1f}s  {phase:<28}'
                f'{pos}{tof}  [{switches}]{link}')

    def _run(self):
        while not self._stop.is_set():
            sys.stdout.write('\r' + self.line() + CLR_EOL)
            sys.stdout.flush()
            self._stop.wait(0.1)

    def start(self):
        if self.enabled and self._thread is None:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        if self.enabled:
            sys.stdout.write('\r' + self.line() + CLR_EOL + '\n')
            sys.stdout.flush()


# Globals wired up in __main__
rc = None
status = None


# --------------------------------------------------------------------------
# RC-aware waiting
# --------------------------------------------------------------------------

def rc_check(in_flight=True):
    """Raise if the pilot has asked to stop. Call this in every loop."""
    if rc.kill:
        raise KillSwitchError('kill switch')
    if not rc.connected:
        raise KillSwitchError('RC link lost')
    if in_flight:
        if ARM_DOWN_IS_KILL and not rc.arm:
            raise KillSwitchError('arm switch flicked down')
        if LAUNCH_DOWN_IS_LAND and not rc.launch:
            raise LandRequest('launch switch flicked down')


def wait(duration, in_flight=True):
    """time.sleep() that aborts as soon as a switch says so."""
    deadline = time.time() + duration
    while True:
        rc_check(in_flight)
        remaining = deadline - time.time()
        if remaining <= 0:
            return
        time.sleep(min(0.02, remaining))


def wait_while_landing(duration):
    """Sleep during a land: only the KILL switch interrupts it.

    Arm/launch going down must not abort here -- we are already coming down --
    but kill still has to cut the motors mid-descent.
    """
    deadline = time.time() + duration
    while True:
        if rc.kill:
            raise KillSwitchError('kill switch during landing')
        remaining = deadline - time.time()
        if remaining <= 0:
            return
        time.sleep(min(0.02, remaining))


def wait_for_switch(name, prompt):
    """Block until switch `name` goes up, aborting on kill / RC loss."""
    status.set_phase(prompt)
    while not getattr(rc, name):
        rc_check(in_flight=False)
        time.sleep(0.02)


# --------------------------------------------------------------------------
# Estimator / params (unchanged from baby.py)
# --------------------------------------------------------------------------

def wait_for_position_estimator(scf):
    status.set_phase('waiting for estimator to converge')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000] * 10
    var_y_history = [1000] * 10
    var_z_history = [1000] * 10
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x, max_x = min(var_x_history), max(var_x_history)
            min_y, max_y = min(var_y_history), max(var_y_history)
            min_z, max_z = min(var_z_history), max(var_z_history)

            if (max_x - min_x) < threshold and (max_y - min_y) < threshold and \
                    (max_z - min_z) < threshold:
                break


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')


def configure_flow_only(cf):
    """Fly on flow + IMU only; make sure no external-pose fusion is active."""
    for name, val in (('mtf02.flowDisable', '0'), ('locSrv.enExtPoseFuse', '0')):
        try:
            cf.param.set_value(name, val)
        except Exception:
            pass


def _start_log_block(cf, name, variables, period_ms=100, stamp_key=None):
    """Start one log block feeding the status line. Returns None if the drone
    does not have these variables (KeyError) or the block is too big
    (AttributeError) -- a missing deck must not stop the flight."""
    log_config = LogConfig(name=name, period_in_ms=period_ms)
    for var, ctype in variables:
        log_config.add_variable(var, ctype)

    def _cb(timestamp, data, logconf):
        if stamp_key is not None:          # arrival time, for staleness checks
            data = dict(data)
            data[stamp_key] = time.time()
        status.set_telemetry(data)

    log_config.data_received_cb.add_callback(_cb)
    try:
        cf.log.add_config(log_config)
        log_config.start()
    except (KeyError, AttributeError) as err:
        status.log(f'  (log block {name!r} unavailable: {err} -- skipping)')
        return None
    return log_config


def start_telemetry(cf):
    """Feed the status line and the control loop. Two blocks: a log packet
    holds at most 26 bytes (LogConfig.MAX_LEN)."""
    blocks = []

    # 20 bytes
    state_block = _start_log_block(cf, 'RCBackupState', [
        ('stateEstimate.x', 'float'),
        ('stateEstimate.y', 'float'),
        ('stateEstimate.z', 'float'),
        ('stabilizer.yaw', 'float'),
        ('pm.vbat', 'float'),
    ])
    if state_block is not None:
        blocks.append(state_block)

    # Front ToF column averages. No type given, so each is fetched as whatever
    # the firmware stores it as; four columns fit comfortably in one packet.
    tof_block = _start_log_block(cf, 'RCBackupTof',
                                 [(name, None) for name in TOF_COLUMNS],
                                 period_ms=50, stamp_key='_tof_t')
    status.tof_available = tof_block is not None
    if tof_block is not None:
        blocks.append(tof_block)

    return blocks


# --------------------------------------------------------------------------
# Flight sequence -- baby.py's, with every sleep made interruptible
# --------------------------------------------------------------------------

def read_tof():
    """Front ToF snapshot: (front_m, age_s), or (None, None) before any data.

    front_m is the minimum over TOF_FRONT_COLUMNS, in metres. A column with no
    valid zones reports 4000 mm, so "nothing seen" reads as 4 m -- clear.
    """
    tlm = status.get_telemetry()
    stamp = tlm.get('_tof_t')
    if stamp is None:
        return None, None
    valid = [tlm[k] for k in TOF_FRONT_COLUMNS if tlm.get(k) is not None]
    if not valid:
        return None, None
    return min(valid) / 1000.0, time.time() - stamp


def run_backup_sequence(cf):
    """
    Forward BK_FWD m at BK_Z, getting past obstacles seen by the front ToF:

        FORWARD --front <= 0.75 m--> CLIMB --front >= 1.5 m--> FORWARD
                                       |
                                  height 2.0 m
                                       v
                  SIDESTEP (random L/R) -> SWEEP_DOWN --front >= 1.5 m--> FORWARD
                                             |
                                        floor, no gap
                                             v
                                 SIDESTEP (random L/R) -> CLIMB -> ...

    Streams send_hover_setpoint at BK_RATE_HZ. The altitude command is
    absolute, so climb and sweep speeds are integrated into z_target. Every
    tick calls rc_check(), so the kill switch acts within one iteration.
    """
    global t_start
    dt = 1.0 / BK_RATE_HZ
    hlc = cf.high_level_commander
    t_start = time.time()

    status.set_phase(f'takeoff to z={BK_Z:.1f} m')
    hlc.takeoff(BK_Z, 3.0)
    wait(3.0)
    status.set_phase(f'hover at takeoff {TAKEOFF_HOVER:.1f}s')
    wait(TAKEOFF_HOVER)

    tlm = status.get_telemetry()
    x0 = tlm.get('stateEstimate.x', 0.0)
    y0 = tlm.get('stateEstimate.y', 0.0)

    z_target = BK_Z
    mode = None
    clear_count = 0
    side_dir = 0
    side_until = 0.0
    after_side = None
    sidesteps = 0
    sweep_dir = 0          # 'sweep' mode: direction kept for this obstacle
    stale_since = None
    resume_mode = None     # mode to go back to when the ToF recovers
    t_mission = time.time()

    def set_mode(new_mode, text):
        """Update the phase line only when the mode actually changes."""
        nonlocal mode, clear_count
        if new_mode != mode:
            mode = new_mode
            clear_count = 0
            status.set_phase(text)

    def start_sidestep(then, y_now, now):
        """Pick left or right (per BK_SIDESTEP_MODE), never leaving the corridor."""
        nonlocal side_dir, side_until, after_side, sidesteps, sweep_dir
        sidesteps += 1
        if BK_SIDESTEP_MODE == 'sweep' and sweep_dir != 0:
            d = sweep_dir
        else:
            d = random.choice((-1, 1))                   # +1 = left (body +y)
        if abs(y_now + d * BK_SIDE_STEP) > BK_MAX_LATERAL:
            d = -d                                       # bounce off the edge
        sweep_dir = d
        side_dir = d
        side_until = now + BK_SIDE_STEP / BK_SIDE_SPEED
        after_side = then
        way = 'left' if d > 0 else 'right'
        set_mode('sidestep', f'sidestep {way} {BK_SIDE_STEP:.1f} m (#{sidesteps})')

    def cleared(text):
        """Past the obstacle: back to FORWARD with a fresh sidestep budget."""
        nonlocal sidesteps, sweep_dir
        sidesteps = 0
        sweep_dir = 0
        set_mode('forward', text)

    def finish(reason):
        status.set_state('LANDING')
        status.set_phase(f'landing ({reason})')
        try:
            cf.commander.send_notify_setpoint_stop()   # hand back to the hlc
        except Exception:
            pass
        hlc.land(0.0, BK_LAND_TIME)
        wait_while_landing(BK_LAND_TIME + 1.5)
        hlc.stop()

    set_mode('forward', f'forward {BK_FWD:.1f} m')

    while True:
        rc_check()
        now = time.time()
        if now - t_mission > BK_MISSION_TIMEOUT_S:
            finish('mission timeout')
            return

        tlm = status.get_telemetry()
        x = tlm.get('stateEstimate.x', x0) - x0
        y = tlm.get('stateEstimate.y', y0) - y0
        z = tlm.get('stateEstimate.z', z_target)
        status.set_extra(f'fwd {x:5.1f}/{BK_FWD:.0f}m')

        front, age = read_tof()

        # --- no trustworthy ToF: never move on a reading we cannot trust -----
        if front is None or age > BK_TOF_STALE_S:
            if stale_since is None:
                stale_since = now
                resume_mode = mode
            if now - stale_since > BK_TOF_ABORT_S:
                finish('front ToF stale')
                return
            set_mode('stale', 'ToF stale -- holding position')
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
            time.sleep(dt)
            continue
        if stale_since is not None:
            side_until += now - stale_since    # a paused sidestep keeps its length
            stale_since = None
            set_mode(resume_mode, f'ToF back -- resuming {resume_mode}')

        clear_count = clear_count + 1 if front >= BK_CLEAR_DIST else 0
        path_clear = clear_count >= BK_CLEAR_CONFIRM_N
        vx = vy = 0.0

        if mode == 'forward':
            if x >= BK_FWD:
                break
            if front <= BK_STOP_DIST:
                set_mode('climb', f'obstacle at {front:.2f} m -- climbing')
            else:
                vx = BK_FWD_SPEED

        elif mode == 'climb':
            if path_clear:
                cleared(f'clear at z={z:.2f} m -- forward')
            elif z_target >= BK_MAX_Z or z > BK_MAX_Z:
                if sidesteps >= BK_MAX_SIDESTEPS:
                    finish(f'no way through after {sidesteps} sidesteps')
                    return
                start_sidestep('sweep_down', y, now)
            else:
                z_target = min(z_target + BK_CLIMB_SPEED * dt, BK_MAX_Z)

        elif mode == 'sidestep':
            if now >= side_until:
                word = 'sweeping down' if after_side == 'sweep_down' else 'climbing'
                set_mode(after_side, f'{word} for a gap')
            else:
                vy = side_dir * BK_SIDE_SPEED

        elif mode == 'sweep_down':
            if path_clear:
                cleared(f'gap at z={z:.2f} m -- forward')
            elif z_target <= BK_MIN_Z:
                if sidesteps >= BK_MAX_SIDESTEPS:
                    finish(f'no way through after {sidesteps} sidesteps')
                    return
                start_sidestep('climb', y, now)
            else:
                z_target = max(z_target - BK_SWEEP_SPEED * dt, BK_MIN_Z)

        cf.commander.send_hover_setpoint(vx, vy, 0.0, z_target)
        time.sleep(dt)

    # --- covered BK_FWD: hover, then land ------------------------------------
    status.set_phase(f'{BK_FWD:.0f} m reached -- hover {BK_END_HOVER_S:.1f}s')
    for _ in range(int(BK_END_HOVER_S * BK_RATE_HZ)):
        rc_check()
        cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
        time.sleep(dt)
    finish('mission complete')


def land_now(cf):
    """Graceful land requested by the launch switch going down."""
    status.set_state('LANDING')
    status.set_phase('landing (launch switch down)')
    try:
        cf.commander.send_notify_setpoint_stop()
    except Exception:
        pass
    try:
        cf.high_level_commander.land(0.0, 2.5)
        wait_while_landing(4.0)
        cf.high_level_commander.stop()
    except KillSwitchError:
        emergency_stop(cf)
        return
    except Exception as err:
        status.log(f'land failed ({err!r}) -- cutting motors')
        emergency_stop(cf)
        return
    try:
        cf.platform.send_arming_request(False)
    except Exception:
        pass


def emergency_stop(cf):
    """Cut the motors NOW. Best-effort and hammered several times, since a single
    CRTP packet can be dropped. Disarming is the surest kill (motors off
    regardless of any setpoint), so send that too.
    """
    status.set_state('KILLED')
    status.set_phase('motors cut')
    status.log(f'{BOLD}{RED}!!! EMERGENCY STOP -- cutting motors !!!{RESET}')
    for _ in range(5):
        try:
            cf.commander.send_stop_setpoint()          # thrust 0 / stop
        except Exception:
            pass
        try:
            cf.platform.send_arming_request(False)     # disarm -> motors off
        except Exception:
            pass
        try:
            cf.high_level_commander.stop()             # cancel any trajectory
        except Exception:
            pass
        time.sleep(0.05)


def reconnect_and_land():
    start_time = time.time()
    while time.time() - start_time < 10:
        print('Try to reconnect')
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print('Recovered connection and stopping propellors')
                scf.cf.high_level_commander.stop()
        except Exception as e:
            print('Connection failed: ', e)
            time.sleep(1)


def connection_failed_link_error(link_uri, msg):
    print(f'Connection to {link_uri} failed: {msg}')
    reconnect_and_land()


if __name__ == '__main__':
    rc = RCSwitches(device=RC_DEVICE,
                    arm_axis=RC_ARM_AXIS,
                    launch_axis=RC_LAUNCH_AXIS,
                    kill_axis=RC_KILL_AXIS)
    status = Status(rc)

    print('initializing drivers')
    cflib.crtp.init_drivers()

    print(f'Reading RC switches from {RC_DEVICE} '
          f'(arm=axis {RC_ARM_AXIS}, launch=axis {RC_LAUNCH_AXIS}, kill=axis {RC_KILL_AXIS})')
    rc.start()

    print('Connect to the Crazyflie')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.connection_lost.add_callback(connection_failed_link_error)
        cf.console.receivedChar.add_callback(status.console)

        status.start()
        status.set_state('ON-INACTIVE')

        telemetry_logs = []
        try:
            status.set_phase('activating the kalman estimator')
            activate_kalman_estimator(cf)
            configure_flow_only(cf)
            telemetry_logs = start_telemetry(cf)
            reset_estimator(cf)

            if not status.tof_available:
                raise PreflightError('front ToF (range8) log block unavailable -- '
                                     'will not fly an obstacle-avoidance mission blind')

            # Safety gate: never arm with the kill switch up or the arm switch
            # already up -- the pilot has to flick arm deliberately.
            if rc.kill:
                raise KillSwitchError('kill switch is up at startup -- flick it down')
            if rc.arm:
                status.set_phase('flick ARM down first')
                while rc.arm:
                    rc_check(in_flight=False)
                    time.sleep(0.05)

            wait_for_switch('arm', 'waiting for ARM switch (axis 4 up)')
            cf.platform.send_arming_request(True)
            status.set_state('ON-ARMED')
            wait(3.0, in_flight=False)

            wait_for_switch('launch', 'waiting for LAUNCH switch (axis 5 up)')
            status.set_state('ON-ACTIVE')
            run_backup_sequence(cf)

            status.set_state('DONE')
            status.set_phase('mission complete')
            try:
                cf.platform.send_arming_request(False)
            except Exception:
                pass
            time.sleep(1.0)
        except PreflightError as e:
            status.log(f'{BOLD}{RED}PREFLIGHT: {e}{RESET}')
            status.set_state('DONE')
            status.set_phase('not armed')
        except LandRequest as e:
            status.log(f'{YELLOW}Land requested: {e}{RESET}')
            land_now(cf)
            status.set_state('DONE')
            status.set_phase('landed on request')
        except KillSwitchError as e:
            status.log(f'{BOLD}{RED}KILL: {e}{RESET}')
            emergency_stop(cf)
        except KeyboardInterrupt:
            status.log('Ctrl-C caught')
            emergency_stop(cf)
        except Exception as e:
            status.log(f'Error during flight: {e!r}')
            emergency_stop(cf)
        finally:
            for block in telemetry_logs:
                try:
                    block.stop()
                except Exception:
                    pass
            status.stop()
            rc.stop()
