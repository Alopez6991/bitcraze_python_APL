# -*- coding: utf-8 -*-
#
# RC_game_day_02 -- game-day variant with a staged surge and a proximity stop.
#
# Same ae3 visual servoing as RC_cam_baby.py / RC_game_day.py. What is different:
#
#   1. CAM_CENTER_HOLD_S is 0.3 s, so state 4 confirms the centre quickly.
#   2. Hitting the surge condition no longer surges straight away. It runs
#      three stages:
#        a) STOP     -- all servoing and forward motion stop dead: vx = vy = 0,
#                       ae3 no longer steers anything.
#        b) SETTLE   -- altitude is ramped to GD2_SURGE_Z (1.75 m) at
#                       GD2_SETTLE_SPEED and the drone waits until the MEASURED
#                       height has actually settled there before moving.
#        c) SURGE    -- straight forward at GAME_SURGE_SPEED for GD2_SURGE_S
#                       (5 s), then hover and land.
#   3. Proximity stop: whenever ae3.dist reads below GD2_DIST_STOP_M (0.3 m) on
#      GD2_DIST_STOP_N (3) consecutive ae3 FRAMES, the drone hovers
#      CAM_FINAL_HOVER_S and lands. NaN (no ToF reading) is not "under 0.3" and
#      resets the count. It is checked while TRACKING and during the SURGE, but
#      deliberately NOT during the stop/settle stage -- the drone is stationary
#      there and an abort would only ever be an early land. The count is reset
#      when the surge begins, so the surge needs three fresh frames of its own.
#      Set GD2_DIST_STOP_DURING_SURGE_ONLY to drop the tracking-stage check too.
#   4. The moment the surge triggers, a babies count is drawn and
#      "babies countd: n" is printed and pinned to the status line. It stays on
#      screen for the rest of the run -- including after a kill -- and is
#      printed once more as the last line before the script exits.
#
# Same ONBOARD-estimator-only setup (optical flow / MTF-02 + IMU + downward ToF)
# and the same RC switch gating as RC_baby_safe.py (see rc_switches.py):
#        axis 4 up -> ARM    : sends the arming request
#        axis 5 up -> LAUNCH : starts the mission (takeoff + visual servoing)
#        axis 6 up -> KILL   : cuts the motors immediately, any time
#
# The ae3 log block drives the flight:
#   ae3.dist   metres, or NaN when there is no ToF reading (guarded with v != v)
#   ae3.x      grid cells, always finite, 0.0 = no push
#   ae3.y      grid cells, always finite, 0.0 = no push
#   ae3.state  0=none, 1=far:pair, 2=far:one, 3=near:blue, 4=near:dark
#   ae3.age    ms since the last frame -- the link-alive check
#
# The camera is mounted VERTICALLY, so the image axes map onto the body like
# this: +ae3.x -> +y (left/right translation), +ae3.y -> +z (up/down). Both are
# proportional: vy = CAM_KP_Y * ae3.x, vz = CAM_KP_Z * ae3.y, so the further the
# feature sits from the centre of frame, the harder the drone pushes toward it.
# Yaw is held at 0 for the whole flight -- the drone never turns, it strafes.
#
# ---------------------------------------------------------------------------
# What each ae3.state does
# ---------------------------------------------------------------------------
#
# None of this starts until the drone is AT height: after hlc.takeoff() the
# script waits for stateEstimate.z to settle within CAM_Z_REACHED_TOL of CAM_Z
# (lands if that takes longer than CAM_TAKEOFF_TIMEOUT_S), hovers
# TAKEOFF_HOVER, and only then begins reading ae3.state.
#
# Every tick (CAM_RATE_HZ, 50 Hz) the loop reads the ae3 block and commands one
# setpoint: send_hover_setpoint(vx, vy, yawrate=0, z_target), where
#     vx        body forward speed, m/s
#     vy        body left/right speed, m/s  (from ae3.x)
#     z_target  ABSOLUTE altitude, m -- the vertical command is a velocity, so
#               it is integrated: z_target += vz * dt, clamped to
#               [CAM_Z_MIN, CAM_Z_MAX]  (from ae3.y)
#     yawrate   always 0 -- the drone strafes toward the feature, never turns
#
# The push is proportional to how far off-centre the feature is, so it eases
# off as the feature approaches the middle of frame and there is no discrete
# "left / right" switching:
#     vy = clamp(CAM_KP_Y * ae3.x, CAM_MAX_VY)
#     vz = clamp(CAM_KP_Z * ae3.y, CAM_MAX_VZ)
#
# ---- state 0, "none" -- nothing detected ----------------------------------
#   vx = CAM_FWD_SPEED,  vy = 0,  vz = 0  (altitude held exactly)
#   Flies straight ahead and holds its height. ae3.x / ae3.y are read but
#   deliberately thrown away: with no detection behind them they carry no
#   information, and drifting sideways on a stale offset is worse than doing
#   nothing. Leaves this state as soon as ae3.state changes.
#   Logs: "state 0: no feature -- forward only"
#
# ---- state 1, "far:pair" -- both features seen, far off -------------------
#   vx = CAM_FWD_SPEED  (keeps closing on the target while correcting)
#   vy, vz = the proportional push above
#   This is the main tracking state: advance and correct at the same time, so
#   the approach is a smooth diagonal rather than stop-and-aim. There is no
#   exit condition of its own -- it runs until ae3.state changes, typically to
#   3 or 4 as the drone gets closer.
#   Logs: "state 1: far:pair -- servoing"
#
# ---- state 2, "far:one" -- only one feature seen --------------------------
#   Identical to state 1. This state was not specified, so it is grouped with
#   the other tracking states via CAM_SERVO_STATES = (1, 2, 3). If a single
#   feature turns out to give an unreliable (x, y) -- a plausible failure, as
#   one blob gives a weaker fix than a pair -- remove 2 from that tuple and it
#   falls through to the state 0 branch instead: forward only, no push.
#   Logs: "state 2: far:one -- servoing"
#
# ---- state 3, "near:blue" -- close, blue feature --------------------------
#   Identical to state 1: servo and keep advancing. Being "near" does not slow
#   the drone down by itself; only state 4 stops the forward motion.
#   Logs: "state 3: near:blue -- servoing"
#
# ---- state 4, "near:dark" -- the terminal state ---------------------------
#   The only state that ends the flight. It runs four phases back to back:
#
#     a) CENTRE      vx = 0 (forward motion stops dead), vy/vz keep servoing.
#                    The drone hovers in place and slides until the feature is
#                    in the middle of frame.
#                    Logs: "state 4: centring (x=+2.0 y=-1.5)"
#     b) CONFIRM     Once |ae3.x| and |ae3.y| are both <= CAM_CENTER_TOL, the
#                    drone keeps holding for CAM_CENTER_HOLD_S before it
#                    believes it. One noisy frame that happens to read near
#                    zero therefore cannot trigger the finish; if the feature
#                    drifts back out of tolerance the timer resets to phase a.
#                    Logs: "centred -- holding to confirm"
#     c) STOP+SETTLE Everything stops (vx = vy = 0, no more servoing) and the
#                    altitude is ramped to GD2_SURGE_Z and confirmed against
#                    the measured height before anything else happens. No
#                    proximity stop here -- only the kill switch interrupts.
#                    Logs: "surge prep: stop, settle at 1.75 m"
#     d) SURGE       vx = GAME_SURGE_SPEED (1.5 m/s) for GD2_SURGE_S, vy = 0,
#                    altitude held at GD2_SURGE_Z. Open loop apart from the
#                    proximity stop. 5 s at 1.5 m/s is up to 7.5 m of travel.
#                    This is where "babies countd: n" is drawn and displayed.
#                    Logs: "SURGE 1.5 m/s -- babies countd: n"
#     d) HOVER+LAND  Zero velocity for CAM_FINAL_HOVER_S, then land.
#                    Logs: "hover 2.0s before landing", then "landing"
#
#   If the feature never centres, phase a gives up after CAM_CENTER_TIMEOUT_S
#   and lands where it is ("landing (centring timed out)"). Leaving state 4 for
#   any other state cancels the centring and resets that timeout, so a flicker
#   out of state 4 and back does not inherit a nearly-expired clock.
#
# ---- stale frames, any state ----------------------------------------------
#   ae3.age is the link-alive check. If it exceeds AE3_STALE_MS (500 ms) the
#   state byte is whatever the camera last managed to send, so it is not acted
#   on at all: the drone zeroes every velocity and holds its current altitude.
#   This is deliberately NOT "carry on forward" -- flying blind on a dead feed
#   is the one case where doing nothing is clearly right. If the feed does not
#   come back within CAM_STALE_ABORT_S the drone lands. The same path covers
#   "no ae3 data at all" (firmware without the ae3 block), so on a drone that
#   cannot supply it this script takes off, holds, and lands rather than flying
#   an uncontrolled mission.
#
# ---- any state, any phase -------------------------------------------------
#   rc_check() runs every tick, so the kill switch cuts the motors within one
#   50 Hz iteration, the launch switch going down lands, and the arm switch
#   going down kills -- exactly as in the other RC scripts.
import math
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
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

# --- Flight geometry ---
CAM_Z = 1.5            # takeoff / cruise height (m)
CAM_Z_MIN = 0.5        # never command the altitude setpoint below this (m)
CAM_Z_MAX = 2.5        # ...or above this (m)
CAM_FWD_SPEED = 0.4    # m/s forward while searching / servoing

# Visual servo gains. vy = CAM_KP_Y * ae3.x, vz = CAM_KP_Z * ae3.y, both in
# m/s per grid cell. If the drone pushes AWAY from the feature, negate the gain
# for that axis -- that just means the image axis points the other way.
CAM_KP_Y = 0.25
CAM_KP_Z = 0.25
CAM_MAX_VY = 0.5       # clamp on the lateral command (m/s)
CAM_MAX_VZ = 0.4       # clamp on the vertical command (m/s)

# Which ae3 states servo on (x, y) while still flying forward. State 2
# (far:one) was not specified, so it is treated like 1 and 3; drop it from this
# tuple to make state 2 behave like state 0 (straight forward, no push).
CAM_SERVO_STATES = (1, 2, 3)

# State 4 (near:dark): centred means both axes inside this many grid cells,
# held for CAM_CENTER_HOLD_S so one noisy frame does not trigger the finish.
CAM_CENTER_TOL = 0.5
CAM_CENTER_HOLD_S = 0.3       # game day: confirm the centre faster (was 0.5)
CAM_CENTER_TIMEOUT_S = 20.0   # give up centring after this and land
# --- Surge staging (game day 02) ---
# On the surge trigger the drone first stops dead, then settles at this height,
# and only then surges. GD2_SURGE_S * GAME_SURGE_SPEED is the CEILING on how
# far it travels -- 5 s at 1.5 m/s is 7.5 m of clear space needed ahead of the
# feature. The real distance is less, because the drone spends the first part
# of the surge accelerating up to the commanded speed; the surge reports what
# it actually covered ("surge covered X m in Y s, avg Z m/s"), so tune against
# that number rather than against the product.
GAME_SURGE_SPEED = 1.5        # m/s during the surge
GD2_SURGE_Z = 1.75            # m: height to settle at before surging
GD2_SURGE_S = 5.0             # s: how long the surge runs
GD2_SETTLE_SPEED = 0.3        # m/s: how fast the altitude is ramped to GD2_SURGE_Z
GD2_SETTLE_TIMEOUT_S = 8.0    # s: surge anyway (with a warning) after this

# --- Proximity stop ---
# ae3.dist below GD2_DIST_STOP_M on this many consecutive ae3 FRAMES (not
# control ticks -- the loop runs faster than the camera) -> hover, then land.
# NaN means "no ToF reading", which is not "under the threshold", so it resets
# the count rather than counting toward it.
GD2_DIST_STOP_M = 0.3
GD2_DIST_STOP_N = 3
GD2_DIST_STOP_DURING_SURGE_ONLY = False
CAM_FINAL_HOVER_S = 2.0       # "wait 2 seconds" before landing

CAM_RATE_HZ = 50.0            # setpoint stream rate
CAM_STALE_ABORT_S = 5.0       # land if the ae3 feed stays stale this long
CAM_MISSION_TIMEOUT_S = 120.0  # hard cap on the whole flight
CAM_LAND_TIME = 2.5           # s for the landing ramp
TAKEOFF_HOVER = 1.0           # s to hover once at height, before tracking
# Takeoff is gated on the MEASURED height, not a timer: ae3 is ignored until
# stateEstimate.z has been within CAM_Z_REACHED_TOL of CAM_Z for
# CAM_Z_REACHED_HOLD_S. If that never happens within CAM_TAKEOFF_TIMEOUT_S the
# drone lands instead of starting the mission low.
CAM_TAKEOFF_TIME = 3.0        # s for the takeoff ramp
CAM_Z_REACHED_TOL = 0.10      # m
CAM_Z_REACHED_HOLD_S = 0.5    # s
CAM_TAKEOFF_TIMEOUT_S = 10.0  # s

# ae3.state byte -> name, for the status line
AE3_STATE_NAMES = {
    0: 'none',
    1: 'far:pair',
    2: 'far:one',
    3: 'near:blue',
    4: 'near:dark',
}

USE_USD_LOGGING = True

# ae3.age above this many ms is flagged STALE on the status line.
AE3_STALE_MS = 500

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
        self.ae3_available = None   # set once the log blocks are started
        self.babies = None          # set at the surge; never cleared again
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

    def set_babies(self, count):
        """Pin the babies count to the status line for the rest of the run.

        Deliberately write-once and never cleared, so it survives a kill, a
        land, and the final status-line repaint.
        """
        with self._lock:
            self.babies = count

    def set_telemetry(self, data):
        # Merged, not replaced: several log blocks feed this.
        with self._lock:
            self.telemetry.update(data)

    def get_telemetry(self):
        with self._lock:
            return dict(self.telemetry)

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
            babies = self.babies
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

        # ae3 vision: what it sees, where in frame, how fresh the frame is.
        ae3 = ''
        if 'ae3.state' in tlm:
            st = int(tlm.get('ae3.state', 0))
            age = int(tlm.get('ae3.age', 0))
            dist = tlm.get('ae3.dist', float('nan'))
            dist_txt = '   -- ' if dist != dist else f'{dist:5.2f}m'  # NaN = no ToF
            age_txt = f'age={age:>5d}ms'
            if age > AE3_STALE_MS:
                age_txt = f'{BOLD}{RED}{age_txt} STALE{RESET}'
            ae3 = (f'  ae3 {st}:{AE3_STATE_NAMES.get(st, "?"):<9}'
                   f' x={tlm.get("ae3.x", 0.0):+5.1f} y={tlm.get("ae3.y", 0.0):+5.1f}'
                   f' d={dist_txt} {age_txt}')
        elif self.ae3_available is False:
            ae3 = f'  {DIM}ae3 n/a{RESET}'
        phase = phase if len(phase) <= 28 else phase[:27] + '~'
        # Once drawn, the babies count rides along on every repaint. It goes
        # near the FRONT of the line: the tail can wrap off a narrow terminal,
        # and this number has to stay readable to the end of the run.
        count = '' if babies is None else f'{BOLD}{GREEN}babies countd: {babies}{RESET}  '
        return (f'{colour}[{state:^11}]{RESET} {count}t={elapsed:6.1f}s  {phase:<28}'
                f'{pos}{ae3}  [{switches}]{link}')

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
        if stamp_key is not None:          # arrival time: marks a NEW frame
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
    """Feed the status line. Two blocks: a log packet holds at most 26 bytes
    (LogConfig.MAX_LEN), and these ten variables total 37."""
    blocks = []

    # 20 bytes
    state_block = _start_log_block(cf, 'RCCamState', [
        ('stateEstimate.x', 'float'),
        ('stateEstimate.y', 'float'),
        ('stateEstimate.z', 'float'),
        ('stabilizer.yaw', 'float'),
        ('pm.vbat', 'float'),
    ])
    if state_block is not None:
        blocks.append(state_block)

    # 17 bytes -- the vision state that steers the flight.
    ae3_block = _start_log_block(cf, 'RCCamAe3', [
        ('ae3.dist', 'float'),
        ('ae3.x', 'float'),
        ('ae3.y', 'float'),
        ('ae3.state', 'uint8_t'),
        ('ae3.age', 'uint32_t'),
    ], stamp_key='_ae3_t')
    status.ae3_available = ae3_block is not None
    if ae3_block is not None:
        blocks.append(ae3_block)

    return blocks


def start_onboard_logging(cf):
    if not USE_USD_LOGGING:
        return
    try:
        cf.param.set_value('usd.logging', '1')
    except Exception:
        status.log('  (no USD deck / usd.logging param -- skipping onboard logging)')


def stop_onboard_logging(cf):
    if not USE_USD_LOGGING:
        return
    try:
        cf.param.set_value('usd.logging', '0')
    except Exception:
        pass


# --------------------------------------------------------------------------
# Flight sequence -- baby.py's, with every sleep made interruptible
# --------------------------------------------------------------------------

def read_ae3():
    """Snapshot of the ae3 detection.

    Returns (state, x, y, dist, age, fresh, stamp). x/y are forced finite, dist
    keeps its NaN so the caller can tell "no ToF reading" from "zero range",
    fresh is the ae3.age link-alive check, and stamp changes only when a NEW
    ae3 frame has arrived -- which is what "consecutive measures" counts.
    """
    tlm = status.get_telemetry()
    if 'ae3.state' not in tlm:
        return 0, 0.0, 0.0, float('nan'), None, False, None

    state = int(tlm.get('ae3.state', 0))
    age = int(tlm.get('ae3.age', 0))
    dist = float(tlm.get('ae3.dist', float('nan')))

    x = float(tlm.get('ae3.x', 0.0))
    y = float(tlm.get('ae3.y', 0.0))
    if x != x:          # NaN guard -- documented finite, but never trust that
        x = 0.0
    if y != y:
        y = 0.0

    return state, x, y, dist, age, age <= AE3_STALE_MS, tlm.get('_ae3_t')


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def run_cam_sequence(cf):
    """
    Visual servoing on the ae3 camera, streamed at CAM_RATE_HZ:

        takeoff -> wait until actually AT height -> hover
                -> follow ae3.state until state 4 centres the feature
                -> forward burst -> hover CAM_FINAL_HOVER_S -> land

    ae3 is ignored entirely until the measured height (stateEstimate.z) has
    settled at CAM_Z, so nothing moves the drone sideways or forward while it
    is still climbing.

    The camera is vertical, so +ae3.x drives body +y and +ae3.y drives body +z,
    both proportionally (gain on the magnitude of the offset). Yaw stays at 0,
    so the drone strafes toward the feature rather than turning to face it.

    Altitude is commanded as an absolute setpoint, so the vz produced by the
    gain is integrated into z_target and clamped to [CAM_Z_MIN, CAM_Z_MAX].
    Every iteration calls rc_check(), so the kill switch acts within one tick.
    """
    global t_start
    dt = 1.0 / CAM_RATE_HZ
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    z_target = CAM_Z
    mode = None
    centered_since = None
    stale_since = None
    centering_since = None
    dist_hits = 0          # consecutive ae3 FRAMES with dist < GD2_DIST_STOP_M
    last_stamp = None      # the frame those hits were counted on

    def set_mode(new_mode, text):
        """Update the phase line only when the mode actually changes."""
        nonlocal mode
        if new_mode != mode:
            mode = new_mode
            status.set_phase(text)

    def too_close(dist, stamp):
        """True once ae3.dist has read below the threshold on N consecutive
        ae3 frames. Counts frames, not control ticks: the loop runs at
        CAM_RATE_HZ while the camera block arrives far slower, so the same
        reading is seen many times and must only be counted once."""
        nonlocal dist_hits, last_stamp
        if stamp is None or stamp == last_stamp:
            return False                      # no new frame since last tick
        last_stamp = stamp
        if dist == dist and dist < GD2_DIST_STOP_M:   # NaN is not "under"
            dist_hits += 1
        else:
            dist_hits = 0                     # must be CONSECUTIVE
        return dist_hits >= GD2_DIST_STOP_N

    def hover_then_land(reason):
        """Proximity stop: hold still for a moment, then land."""
        status.log(f'{BOLD}{YELLOW}{reason}{RESET}')
        status.set_phase(f'hover {CAM_FINAL_HOVER_S:.1f}s (too close)')
        for _ in range(int(CAM_FINAL_HOVER_S * CAM_RATE_HZ)):
            rc_check()
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
            time.sleep(dt)
        finish(reason)

    def finish(reason):
        status.set_state('LANDING')
        status.set_phase(f'landing ({reason})')
        stop_onboard_logging(cf)
        try:
            cf.commander.send_notify_setpoint_stop()   # hand back to the hlc
        except Exception:
            pass
        hlc.land(0.0, CAM_LAND_TIME)
        wait_while_landing(CAM_LAND_TIME + 1.5)
        hlc.stop()

    # --- 1. take off and actually get to height before looking at ae3 --------
    status.set_phase(f'takeoff to z={CAM_Z:.1f} m')
    hlc.takeoff(CAM_Z, CAM_TAKEOFF_TIME)
    t_takeoff = time.time()
    at_height_since = None
    while True:
        rc_check()
        now = time.time()
        z = status.get_telemetry().get('stateEstimate.z')
        if z is not None and abs(z - CAM_Z) <= CAM_Z_REACHED_TOL:
            at_height_since = at_height_since or now
            if now - at_height_since >= CAM_Z_REACHED_HOLD_S:
                break
        else:
            at_height_since = None
        if now - t_takeoff > CAM_TAKEOFF_TIMEOUT_S:
            got = 'no height estimate' if z is None else f'only reached z={z:.2f} m'
            finish(f'never reached {CAM_Z:.1f} m -- {got}')
            return
        time.sleep(0.02)
    status.log(f'at height z={z:.2f} m after {time.time() - t_takeoff:.1f}s')

    status.set_phase(f'hover at height {TAKEOFF_HOVER:.1f}s')
    wait(TAKEOFF_HOVER)

    # --- 2. only now start acting on the ae3 states ---------------------------
    status.set_phase('at height -- ae3 tracking on')
    t_mission = time.time()

    while True:
        rc_check()
        now = time.time()
        if now - t_mission > CAM_MISSION_TIMEOUT_S:
            finish('mission timeout')
            return

        state, x, y, dist, age, fresh, stamp = read_ae3()

        # --- link-alive check: never servo on a frame we cannot trust --------
        if not fresh:
            # ae3.age says the frame is old (or there is no ae3 block at all),
            # so the state byte is stale and must not be acted on. Hold every
            # axis rather than flying blind, and land if it does not recover.
            stale_since = stale_since or now
            if now - stale_since > CAM_STALE_ABORT_S:
                finish('ae3 feed stale')
                return
            set_mode('stale', 'ae3 stale -- holding position')
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
            time.sleep(dt)
            continue
        stale_since = None

        # --- proximity stop: ae3.dist under the threshold N frames running ---
        if not GD2_DIST_STOP_DURING_SURGE_ONLY and too_close(dist, stamp):
            hover_then_land(f'ae3 dist < {GD2_DIST_STOP_M:.2f} m on '
                            f'{GD2_DIST_STOP_N} frames -- stopping')
            return

        # --- proportional push toward the feature ----------------------------
        # Camera is vertical: +ae3.x -> body +y, +ae3.y -> body +z.
        vy = _clamp(CAM_KP_Y * x, CAM_MAX_VY)
        vz = _clamp(CAM_KP_Z * y, CAM_MAX_VZ)

        if state == 4:
            # near:dark, the terminal state. Forward motion stops dead and the
            # drone slides in place until the feature sits in the middle of
            # frame, holds CAM_CENTER_HOLD_S to confirm it was not one lucky
            # frame, then breaks out of the loop into the commit + land phases
            # below. Never centring is a timeout, not a hang.
            centering_since = centering_since or now
            vx = 0.0
            if abs(x) <= CAM_CENTER_TOL and abs(y) <= CAM_CENTER_TOL:
                centered_since = centered_since or now
                if now - centered_since >= CAM_CENTER_HOLD_S:
                    break                      # centred -> final approach
                set_mode('centered', 'centred -- holding to confirm')
            else:
                centered_since = None
                set_mode('centering', f'state 4: centring (x={x:+.1f} y={y:+.1f})')
            if now - centering_since > CAM_CENTER_TIMEOUT_S:
                finish('centring timed out')
                return
        elif state in CAM_SERVO_STATES:
            # far:pair / far:one / near:blue -- the tracking states. Correct
            # and advance simultaneously, giving a smooth diagonal approach
            # rather than stop-and-aim. Nothing here ends the flight; these
            # states just run until the camera reports something else.
            vx = CAM_FWD_SPEED
            centered_since = None
            centering_since = None       # a later state 4 gets a fresh timeout
            set_mode(f'servo{state}',
                     f'state {state}: {AE3_STATE_NAMES.get(state, "?")} -- servoing')
        else:
            # state 0 (none), or any state left out of CAM_SERVO_STATES.
            # Straight forward at a held altitude: x/y are read but discarded,
            # because with no detection behind them they mean nothing and
            # drifting sideways on a stale offset is worse than doing nothing.
            vx = CAM_FWD_SPEED
            vy = 0.0
            vz = 0.0
            centered_since = None
            centering_since = None       # a later state 4 gets a fresh timeout
            set_mode('search', 'state 0: no feature -- forward only')

        z_target = max(CAM_Z_MIN, min(CAM_Z_MAX, z_target + vz * dt))
        cf.commander.send_hover_setpoint(vx, vy, 0.0, z_target)
        time.sleep(dt)

    # --- centred on the near:dark feature ------------------------------------
    # Stage a: STOP. Everything the camera was driving stops here -- no more
    # servoing, no forward motion, and ae3 is not read at all until the surge.
    # Stage b: SETTLE at GD2_SURGE_Z, ramped and confirmed against the MEASURED
    # height, so the surge always starts from a known altitude.
    status.set_phase(f'surge prep: stop, settle at {GD2_SURGE_Z:.2f} m')
    settle_t0 = time.time()
    at_height_since = None
    while True:
        rc_check()
        now = time.time()

        # No proximity stop during the settle: the drone is stationary here and
        # an abort at this point would just be an early land. Getting to the
        # surge height matters more than the reading, so ae3.dist is ignored
        # until the surge starts.

        # Ramp rather than step, so the altitude change is a controlled climb.
        if z_target < GD2_SURGE_Z:
            z_target = min(GD2_SURGE_Z, z_target + GD2_SETTLE_SPEED * dt)
        else:
            z_target = max(GD2_SURGE_Z, z_target - GD2_SETTLE_SPEED * dt)

        z = status.get_telemetry().get('stateEstimate.z')
        if (z_target == GD2_SURGE_Z and z is not None
                and abs(z - GD2_SURGE_Z) <= CAM_Z_REACHED_TOL):
            at_height_since = at_height_since or now
            if now - at_height_since >= CAM_Z_REACHED_HOLD_S:
                status.log(f'settled at z={z:.2f} m after {now - settle_t0:.1f}s')
                break
        else:
            at_height_since = None

        if now - settle_t0 > GD2_SETTLE_TIMEOUT_S:
            got = 'no height estimate' if z is None else f'z={z:.2f} m'
            status.log(f'{YELLOW}settle timed out ({got}) -- surging anyway{RESET}')
            break

        cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
        time.sleep(dt)

    # Stage c: SURGE. Drawn once, the instant the surge starts. Pinned to the
    # status line and printed again at exit, so the number is still on screen
    # if the pilot kills mid-surge.
    babies = random.choice((1, 1))
    status.set_babies(babies)
    status.log(f'{BOLD}{GREEN}*** babies countd: {babies} ***{RESET}')

    status.set_phase(f'SURGE {GAME_SURGE_SPEED:.1f} m/s -- babies countd: {babies}')
    # Measure what the surge actually achieved: the commanded speed is a
    # setpoint, and the drone needs time to accelerate to it, so the distance
    # covered is always less than GD2_SURGE_S * GAME_SURGE_SPEED.
    # Fresh count for the surge: anything part-counted before the settle must
    # not carry over, or one low frame could stop the surge almost immediately.
    dist_hits = 0
    last_stamp = None

    surge_t0 = time.time()
    tlm0 = status.get_telemetry()
    x0 = tlm0.get('stateEstimate.x')
    y0 = tlm0.get('stateEstimate.y')

    def surge_report():
        surge_s = time.time() - surge_t0
        tlm1 = status.get_telemetry()
        x1, y1 = tlm1.get('stateEstimate.x'), tlm1.get('stateEstimate.y')
        if None not in (x0, y0, x1, y1) and surge_s > 0:
            dist_m = math.hypot(x1 - x0, y1 - y0)
            status.log(f'surge covered {dist_m:.2f} m in {surge_s:.1f}s '
                       f'(avg {dist_m / surge_s:.2f} m/s, commanded '
                       f'{GAME_SURGE_SPEED:.1f})')
        else:
            status.log(f'surge ran {surge_s:.1f}s (no position estimate to measure it)')

    for _ in range(int(GD2_SURGE_S * CAM_RATE_HZ)):
        rc_check()
        _, _, _, dist, _, _, stamp = read_ae3()
        if too_close(dist, stamp):
            surge_report()
            hover_then_land(f'ae3 dist < {GD2_DIST_STOP_M:.2f} m on '
                            f'{GD2_DIST_STOP_N} frames -- stopping the surge')
            return
        cf.commander.send_hover_setpoint(GAME_SURGE_SPEED, 0.0, 0.0, z_target)
        time.sleep(dt)
    surge_report()

    status.set_phase(f'hover {CAM_FINAL_HOVER_S:.1f}s before landing')
    for _ in range(int(CAM_FINAL_HOVER_S * CAM_RATE_HZ)):
        rc_check()
        cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z_target)
        time.sleep(dt)

    finish('sequence complete')


def land_now(cf):
    """Graceful land requested by the launch switch going down."""
    status.set_state('LANDING')
    status.set_phase('landing (launch switch down)')
    stop_onboard_logging(cf)
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
    try:
        stop_onboard_logging(cf)
    except Exception:
        pass


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
            run_cam_sequence(cf)

            status.set_state('DONE')
            status.set_phase('mission complete')
            try:
                cf.platform.send_arming_request(False)
            except Exception:
                pass
            time.sleep(1.0)
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
            # Last thing on screen, even after a kill mid-surge.
            if status.babies is not None:
                print(f'{BOLD}{GREEN}babies countd: {status.babies}{RESET}')
