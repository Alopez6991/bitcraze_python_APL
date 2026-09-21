"""
Keyboard-controlled Crazyflie flight with mocap-based state estimation.

Controls:
    Arrow Up    -> forward  (body +x)
    Arrow Down  -> backward (body -x)
    Arrow Left  -> left     (body +y)
    Arrow Right -> right    (body -y)
    w           -> up       (altitude +)
    s           -> down     (altitude -)
    a           -> yaw left
    d           -> yaw right
    Ctrl-C      -> land and exit

On launch the drone arms, takes off to 1 m, then enters a keyboard-driven
velocity-streaming loop. Ctrl-C triggers a smooth landing.

Requires: pip install motioncapture

Keyboard input is read directly from the terminal via termios cbreak mode,
so the script must be run in a real terminal (not piped) and the terminal
window must be the focused window when you're sending commands.
"""
import math
import select
import signal
import sys
import termios
import threading
import time
import tty

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# --- Connection / mocap config ---------------------------------------------
URI              = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E702')
MOCAP_HOST       = '192.168.209.81'
MOCAP_SYSTEM     = 'optitrack'
RIGID_BODY_NAME  = 'flapper_two'

# --- Flight tuning ----------------------------------------------------------
TAKEOFF_HEIGHT   = 1.0    # meters
MIN_ALTITUDE     = 0.2    # hard floor for streamed setpoints
MAX_ALTITUDE     = 2.0    # hard ceiling for streamed setpoints
CONTROL_HZ       = 20.0
CONTROL_PERIOD   = 1.0 / CONTROL_HZ

# Per-keystroke step sizes. Terminal auto-repeat (~30 Hz by default) will
# produce many keystrokes per second when you hold a key, so these steps
# are small.
STEP_XY          = 0.1    # meters per arrow-key nudge
STEP_Z           = 0.03    # meters per w/s nudge
STEP_YAW         = 3.0     # degrees per a/d nudge


# --- Terminal raw-mode keyboard reader -------------------------------------
class RawKeyboard:
    """Context manager that puts stdin into cbreak mode and yields keys
    as they arrive.

    read_key() returns one of the strings 'up', 'down', 'left', 'right',
    a single char ('w', 's', 'a', 'd', ...), or None if nothing is pending.
    Ctrl-C still raises KeyboardInterrupt in cbreak mode (ISIG stays on).
    """

    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old_attrs = None

    def __enter__(self):
        self._old_attrs = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)

    def _pending(self, timeout=0.0):
        return bool(select.select([self._fd], [], [], timeout)[0])

    def read_key(self):
        if not self._pending():
            return None
        ch = sys.stdin.read(1)
        if ch != '\x1b':
            return ch
        # Possible escape sequence for an arrow key: ESC [ A/B/C/D
        if not self._pending(timeout=0.001):
            return 'esc'
        if sys.stdin.read(1) != '[':
            return 'esc'
        if not self._pending(timeout=0.001):
            return 'esc'
        code = sys.stdin.read(1)
        return {'A': 'up', 'B': 'down', 'C': 'right', 'D': 'left'}.get(code)


# --- Mocap streaming --------------------------------------------------------
class MocapWrapper(threading.Thread):
    def __init__(self, body_name):
        super().__init__(daemon=True)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system...')
        mc = motioncapture.connect(MOCAP_SYSTEM, {'hostname': MOCAP_HOST})
        print('Mocap connected')
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name and self.on_pose:
                    p = obj.position
                    self.on_pose([p[0], p[1], p[2], obj.rotation])


def send_extpose(cf, pose):
    # Axis convention matches examples/mocap/flapper_ekf_data.py for this rig:
    # pass (z, x, y) for position and (qz, qx, qy, qw) for orientation.
    x, y, z, q = pose
    cf.extpos.send_extpose(z, x, y, q.z, q.x, q.y, q.w)


# --- Estimator setup --------------------------------------------------------
def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def wait_for_position_estimator(scf):
    print('Waiting for estimator to converge...')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    hist = {'x': [1000.0] * 10, 'y': [1000.0] * 10, 'z': [1000.0] * 10}
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for entry in logger:
            d = entry[1]
            for axis, key in (('x', 'kalman.varPX'),
                              ('y', 'kalman.varPY'),
                              ('z', 'kalman.varPZ')):
                hist[axis].append(d[key])
                hist[axis].pop(0)
            if all(max(hist[a]) - min(hist[a]) < threshold for a in 'xyz'):
                print('Estimator converged')
                return


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


# --- Keyboard -> position setpoint mapping ---------------------------------
def apply_key(key, step_xy, target_x, target_y, target_z, target_yaw):
    """Apply a single keystroke as a nudge to the world-frame target pose.

    Arrow keys are body-frame translations of size `step_xy` (rotated by
    target_yaw into world frame). w/s move altitude, a/d rotate yaw.
    Returns the new (x, y, z, yaw_deg) tuple.
    """
    vx_body = 0.0
    vy_body = 0.0
    dz = 0.0
    dyaw = 0.0

    if key == 'up':
        vx_body += step_xy
    elif key == 'down':
        vx_body -= step_xy
    elif key == 'left':
        vy_body += step_xy
    elif key == 'right':
        vy_body -= step_xy
    elif key == 'w':
        dz += STEP_Z
    elif key == 's':
        dz -= STEP_Z
    elif key == 'a':
        dyaw += STEP_YAW
    elif key == 'd':
        dyaw -= STEP_YAW
    else:
        return target_x, target_y, target_z, target_yaw

    yaw_rad = math.radians(target_yaw)
    dx_world = vx_body * math.cos(yaw_rad) - vy_body * math.sin(yaw_rad)
    dy_world = vx_body * math.sin(yaw_rad) + vy_body * math.cos(yaw_rad)

    target_x += dx_world
    target_y += dy_world
    target_z = max(MIN_ALTITUDE, min(MAX_ALTITUDE, target_z + dz))
    target_yaw += dyaw
    return target_x, target_y, target_z, target_yaw


def read_current_state(scf):
    """Snapshot stateEstimate.{x,y,z,yaw} once via a synchronous log."""
    log_config = LogConfig(name='InitPose', period_in_ms=100)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')
    with SyncLogger(scf, log_config) as logger:
        for entry in logger:
            d = entry[1]
            return (d['stateEstimate.x'],
                    d['stateEstimate.y'],
                    d['stateEstimate.z'],
                    d['stateEstimate.yaw'])


# --- Main -------------------------------------------------------------------
def main():
    cflib.crtp.init_drivers()

    mocap = MocapWrapper(RIGID_BODY_NAME)

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            mocap.on_pose = lambda pose: send_extpose(cf, pose)

            # Give the mocap stream a moment to start flowing before resetting
            # the Kalman filter, so it has a valid reference to lock onto.
            time.sleep(1.0)

            activate_kalman_estimator(cf)
            reset_estimator(scf)

            # Pre-arm sanity: dump the estimator position. With mocap feeding
            # the EKF, this should match where the rigid body physically sits.
            sx, sy, sz, syaw = read_current_state(scf)
            print(f'Pre-arm stateEstimate: x={sx:+.2f} y={sy:+.2f} '
                  f'z={sz:+.2f} yaw={syaw:+.1f} deg')

            cf.platform.send_arming_request(True)
            time.sleep(0.2)

            print(f'Taking off to {TAKEOFF_HEIGHT:.2f} m')
            cf.high_level_commander.takeoff(TAKEOFF_HEIGHT, 2.5)
            time.sleep(3.0)

            # Seed the teleop target from the current estimator pose so the
            # first position setpoint is not a jump.
            target_x, target_y, _, target_yaw = read_current_state(scf)
            target_z = TAKEOFF_HEIGHT
            print(f'Hover target seeded at x={target_x:+.2f} '
                  f'y={target_y:+.2f} z={target_z:+.2f} yaw={target_yaw:+.1f}')
            step_xy = STEP_XY
            print('Ready. Arrows translate, w/s up/down, a/d yaw, '
                  '0-9 set xy step size, Ctrl-C lands.')
            print(f'[step_xy = {step_xy:.2f} m]')

            try:
                with RawKeyboard() as kb:
                    next_tick = time.time()
                    while True:
                        # Drain every keystroke the terminal has buffered
                        # since the last tick and apply each as a nudge.
                        while True:
                            key = kb.read_key()
                            if key is None:
                                break
                            if key is not None and len(key) == 1 and key.isdigit():
                                step_xy = 0.03 * (int(key) + 1)
                                print(f'[step_xy = {step_xy:.2f} m]')
                                continue
                            target_x, target_y, target_z, target_yaw = \
                                apply_key(key, step_xy, target_x, target_y,
                                          target_z, target_yaw)

                        cf.commander.send_position_setpoint(
                            target_x, target_y, target_z, target_yaw)

                        next_tick += CONTROL_PERIOD
                        sleep_for = next_tick - time.time()
                        if sleep_for > 0:
                            time.sleep(sleep_for)
                        else:
                            next_tick = time.time()
            except KeyboardInterrupt:
                print('\nCtrl-C received — landing')

            # Ignore additional Ctrl-C during landing so a jumpy user can't
            # interrupt SyncCrazyflie teardown and leave the drone airborne.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                cf.commander.send_notify_setpoint_stop()
                cf.high_level_commander.land(0.0, 2.5)
                time.sleep(3.0)
                cf.high_level_commander.stop()
            finally:
                signal.signal(signal.SIGINT, signal.SIG_DFL)
    finally:
        mocap.close()


if __name__ == '__main__':
    main()
