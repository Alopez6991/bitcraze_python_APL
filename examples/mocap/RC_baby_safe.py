# -*- coding: utf-8 -*-
#
# RC_baby_safe -- a straight-line, no-yaw version of RC_baby.py.
#
# Same ONBOARD-estimator-only setup (optical flow / MTF-02 + IMU + downward ToF,
# NO motion capture) and the same RC switch gating (see rc_switches.py):
#        axis 4 up -> ARM    : sends the arming request
#        axis 5 up -> LAUNCH : starts the mission (takeoff + trajectory)
#        axis 6 up -> KILL   : cuts the motors immediately, any time
#      Flicking LAUNCH back down mid-flight lands gracefully; flicking ARM back
#      down mid-flight is treated as a kill (see the two flags below).
# A live status line shows the drone state, switch positions and telemetry.
#
# Trajectory -- no rotation at any point, the nose stays at yaw 0 throughout:
#   takeoff to SAFE_Z -> hover -> forward SAFE_FWD m -> hover
#           -> BACKWARDS SAFE_BACK m (flying in reverse, nose still forward)
#           -> land
#
# "Safe" here means no yaw: baby.py's 360/180 turns are the part that corrupts
# the optical flow, so they are gone. What is NOT safer is the distance -- at
# 11 m out on a flow-only estimate, position error accumulates the whole way
# and there is no absolute reference to correct it. Fly this only somewhere with
# well over 11 m of clear space ahead plus margin on both sides, and keep a
# thumb on the kill switch.
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
SAFE_Z = 2.0          # hover / cruise height (m)
SAFE_FWD = 11.0       # forward distance (m)
SAFE_BACK = 3.0       # distance (m) flown backwards after the hover
SAFE_SPEED = 1.25     # m/s cruise for each go_to leg
SAFE_SETTLE = 1.5     # s to settle at each waypoint
TAKEOFF_HOVER = 1.0   # s to hover after takeoff before moving
SAFE_HOVER_S = 3.0    # s to hover at the far end before flying backwards

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

        # ae3 ranging: rx climbing = link alive, small age = fresh frame.
        ae3 = ''
        if 'ae3.dist' in tlm:
            age = int(tlm.get('ae3.age', 0))
            age_txt = f'age={age:>5d}ms'
            if age > AE3_STALE_MS:
                age_txt = f'{BOLD}{RED}{age_txt} STALE{RESET}'
            ae3 = (f'  ae3 d={tlm.get("ae3.dist", 0):5.2f}m'
                   f' rx={int(tlm.get("ae3.rx", 0)):>7d} {age_txt}')
        elif self.ae3_available is False:
            ae3 = f'  {DIM}ae3 n/a{RESET}'
        phase = phase if len(phase) <= 28 else phase[:27] + '~'
        return (f'{colour}[{state:^11}]{RESET} t={elapsed:6.1f}s  {phase:<28}'
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


def _start_log_block(cf, name, variables, period_ms=100):
    """Start one log block feeding the status line. Returns None if the drone
    does not have these variables (KeyError) or the block is too big
    (AttributeError) -- a missing deck must not stop the flight."""
    log_config = LogConfig(name=name, period_in_ms=period_ms)
    for var, ctype in variables:
        log_config.add_variable(var, ctype)

    def _cb(timestamp, data, logconf):
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
    (LogConfig.MAX_LEN), and these eight variables total 32."""
    blocks = []

    # 20 bytes
    state_block = _start_log_block(cf, 'RCBabyState', [
        ('stateEstimate.x', 'float'),
        ('stateEstimate.y', 'float'),
        ('stateEstimate.z', 'float'),
        ('stabilizer.yaw', 'float'),
        ('pm.vbat', 'float'),
    ])
    if state_block is not None:
        blocks.append(state_block)

    # 12 bytes -- ae3 ranging: dist, rx climbs while the link is alive, age is
    # ms since the last frame (small = fresh).
    ae3_block = _start_log_block(cf, 'RCBabyAe3', [
        ('ae3.dist', 'float'),
        ('ae3.rx', 'uint32_t'),
        ('ae3.age', 'uint32_t'),
    ])
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

def run_safe_sequence(cf):
    """
    Straight-line demo via absolute position setpoints (high-level commander
    go_to), with no yaw anywhere:

        takeoff to SAFE_Z -> hover -> forward SAFE_FWD m -> hover SAFE_HOVER_S
                -> backwards SAFE_BACK m -> land

    Body convention: forward = +x, left = +y; takeoff point is the origin (0,0)
    and the nose stays pointed along +x for the whole flight. The last leg is a
    go_to back toward the origin WITHOUT turning around, so the drone flies in
    reverse and lands at x = SAFE_FWD - SAFE_BACK.

    Every wait is interruptible: the kill switch cuts the motors mid-leg.
    """
    global t_start
    z = SAFE_Z
    settle = SAFE_SETTLE
    yaw = 0.0                       # held at 0 for the entire flight
    fwd_dur = max(SAFE_FWD / SAFE_SPEED, 1.0)
    back_dur = max(SAFE_BACK / SAFE_SPEED, 1.0)
    x_end = SAFE_FWD - SAFE_BACK

    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    status.set_phase(f'takeoff to z={z:.1f} m')
    hlc.takeoff(z, 3.0)
    wait(3.0)                       # let the takeoff ramp complete

    status.set_phase(f'hover at takeoff {TAKEOFF_HOVER:.1f}s')
    hlc.go_to(0.0, 0.0, z, yaw, TAKEOFF_HOVER)
    wait(TAKEOFF_HOVER)

    # 1) forward SAFE_FWD m, nose along +x
    status.set_phase(f'forward {SAFE_FWD:.1f} m -> ({SAFE_FWD:.1f}, 0)')
    hlc.go_to(SAFE_FWD, 0.0, z, yaw, fwd_dur)
    wait(fwd_dur + settle)

    # 2) hover at the far end
    status.set_phase(f'hover {SAFE_HOVER_S:.1f}s at ({SAFE_FWD:.1f}, 0)')
    hlc.go_to(SAFE_FWD, 0.0, z, yaw, SAFE_HOVER_S)
    wait(SAFE_HOVER_S)

    # 3) backwards SAFE_BACK m -- no turn, the drone flies in reverse
    status.set_phase(f'backwards {SAFE_BACK:.1f} m -> ({x_end:.1f}, 0)')
    hlc.go_to(x_end, 0.0, z, yaw, back_dur)
    wait(back_dur + settle)

    status.set_state('LANDING')
    status.set_phase('landing')
    stop_onboard_logging(cf)
    hlc.land(0.0, 2.5)
    wait_while_landing(4.0)         # only kill interrupts a landing
    hlc.stop()


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
            run_safe_sequence(cf)

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
