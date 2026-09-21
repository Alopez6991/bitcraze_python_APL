"""Mocap-guided hover at the world origin for a Crazyflie.

This example arms the drone, takes off, moves to (0.0, 0.0, 1.5), hovers for
the configured duration, and lands.

Defaults are set for rigid body ``flapper_01`` and radio channel ``01``.

Requires: pip install motioncapture
"""

import argparse
import signal
import threading
import time

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


DEFAULT_URI = uri_helper.uri_from_env(default='radio://0/01/2M/E7E7E7E701')
DEFAULT_MOCAP_HOST = '192.168.209.81'
DEFAULT_MOCAP_SYSTEM = 'optitrack'
DEFAULT_RIGID_BODY_NAME = 'flapper_01'

CONTROL_HZ = 20.0
CONTROL_PERIOD = 1.0 / CONTROL_HZ
DEFAULT_TARGET_X = 0.0
DEFAULT_TARGET_Y = 0.0
DEFAULT_TARGET_Z = 1.5
DEFAULT_TAKEOFF_TIME = 2.5
DEFAULT_TRANSIT_TIME = 5.0
DEFAULT_HOVER_SECONDS = 30.0
DEFAULT_YAW_DEG = 0.0


class MocapWrapper(threading.Thread):
    def __init__(self, mocap_system, mocap_host, body_name):
        super().__init__(daemon=True)
        self._mocap_system = mocap_system
        self._mocap_host = mocap_host
        self._body_name = body_name
        self._stay_open = True
        self.on_pose = None
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system...')
        mocap_client = motioncapture.connect(
            self._mocap_system,
            {'hostname': self._mocap_host},
        )
        print('Mocap connected')

        while self._stay_open:
            mocap_client.waitForNextFrame()
            rigid_body = mocap_client.rigidBodies.get(self._body_name)
            if rigid_body is None or self.on_pose is None:
                continue

            position = rigid_body.position
            self.on_pose([position[0], position[1], position[2], rigid_body.rotation])


def parse_args():
    parser = argparse.ArgumentParser(
        description='Take off, fly to the origin, hover for a configurable duration, and land.',
    )
    parser.add_argument('--uri', default=DEFAULT_URI,
                        help='Crazyflie URI, default: %(default)s')
    parser.add_argument('--mocap-host', default=DEFAULT_MOCAP_HOST,
                        help='Motion capture host, default: %(default)s')
    parser.add_argument('--mocap-system', default=DEFAULT_MOCAP_SYSTEM,
                        help='Motion capture backend, default: %(default)s')
    parser.add_argument('--rigid-body', default=DEFAULT_RIGID_BODY_NAME,
                        help='Mocap rigid body name, default: %(default)s')
    parser.add_argument('--target-x', type=float, default=DEFAULT_TARGET_X,
                        help='Target X in meters, default: %(default)s')
    parser.add_argument('--target-y', type=float, default=DEFAULT_TARGET_Y,
                        help='Target Y in meters, default: %(default)s')
    parser.add_argument('--target-z', type=float, default=DEFAULT_TARGET_Z,
                        help='Target Z in meters, default: %(default)s')
    parser.add_argument('--hover-seconds', type=float, default=DEFAULT_HOVER_SECONDS,
                        help='Seconds to hover at the target, default: %(default)s')
    parser.add_argument('--transit-seconds', type=float, default=DEFAULT_TRANSIT_TIME,
                        help='Seconds to move from takeoff to target, default: %(default)s')
    parser.add_argument('--yaw-deg', type=float, default=DEFAULT_YAW_DEG,
                        help='Fixed yaw during the mission, default: %(default)s')

    args = parser.parse_args()
    if args.target_z <= 0.0:
        parser.error('--target-z must be positive')
    if args.hover_seconds < 0.0:
        parser.error('--hover-seconds cannot be negative')
    if args.transit_seconds <= 0.0:
        parser.error('--transit-seconds must be positive')

    return args


def send_extpose(cf, pose):
    # Axis convention matches examples/mocap/flapper_ekf_data.py for this rig.
    x, y, z, quat = pose
    cf.extpos.send_extpose(z, x, y, quat.z, quat.x, quat.y, quat.w)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', '0.06')


def wait_for_position_estimator(scf):
    print('Waiting for estimator to converge...')
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    history = {'x': [1000.0] * 10, 'y': [1000.0] * 10, 'z': [1000.0] * 10}
    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for entry in logger:
            data = entry[1]
            for axis, key in (('x', 'kalman.varPX'),
                              ('y', 'kalman.varPY'),
                              ('z', 'kalman.varPZ')):
                history[axis].append(data[key])
                history[axis].pop(0)

            if all(max(history[axis]) - min(history[axis]) < threshold for axis in 'xyz'):
                print('Estimator converged')
                return


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def read_current_state(scf):
    log_config = LogConfig(name='StateEstimate', period_in_ms=100)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')

    with SyncLogger(scf, log_config) as logger:
        for entry in logger:
            data = entry[1]
            return (
                data['stateEstimate.x'],
                data['stateEstimate.y'],
                data['stateEstimate.z'],
                data['stateEstimate.yaw'],
            )


def stream_position_hold(commander, x, y, z, yaw_deg, duration):
    end_time = time.time() + duration
    next_tick = time.time()
    while time.time() < end_time:
        commander.send_position_setpoint(x, y, z, yaw_deg)
        next_tick += CONTROL_PERIOD
        sleep_for = next_tick - time.time()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            next_tick = time.time()


def move_line(commander, start_pose, end_pose, duration):
    start_time = time.time()
    next_tick = start_time

    while True:
        elapsed = time.time() - start_time
        ratio = min(1.0, elapsed / duration)
        x = start_pose[0] + (end_pose[0] - start_pose[0]) * ratio
        y = start_pose[1] + (end_pose[1] - start_pose[1]) * ratio
        z = start_pose[2] + (end_pose[2] - start_pose[2]) * ratio
        yaw_deg = start_pose[3] + (end_pose[3] - start_pose[3]) * ratio
        commander.send_position_setpoint(x, y, z, yaw_deg)
        if ratio >= 1.0:
            return

        next_tick += CONTROL_PERIOD
        sleep_for = next_tick - time.time()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            next_tick = time.time()


def run_mission(scf, args):
    cf = scf.cf

    activate_kalman_estimator(cf)
    reset_estimator(scf)

    state_x, state_y, state_z, state_yaw = read_current_state(scf)
    print(
        f'Pre-arm stateEstimate: x={state_x:+.2f} y={state_y:+.2f} '
        f'z={state_z:+.2f} yaw={state_yaw:+.1f} deg'
    )

    cf.platform.send_arming_request(True)
    time.sleep(0.2)

    print(f'Taking off to {args.target_z:.2f} m')
    cf.high_level_commander.takeoff(args.target_z, DEFAULT_TAKEOFF_TIME)
    time.sleep(DEFAULT_TAKEOFF_TIME + 0.5)

    current_x, current_y, _, _ = read_current_state(scf)
    start_pose = (current_x, current_y, args.target_z, args.yaw_deg)
    target_pose = (args.target_x, args.target_y, args.target_z, args.yaw_deg)

    print(
        f'Moving to target x={args.target_x:+.2f} y={args.target_y:+.2f} '
        f'z={args.target_z:+.2f} over {args.transit_seconds:.1f} s'
    )
    move_line(cf.commander, start_pose, target_pose, args.transit_seconds)

    print(
        f'Hovering at target for {args.hover_seconds:.1f} s '
        f'with yaw {args.yaw_deg:+.1f} deg'
    )
    stream_position_hold(cf.commander, *target_pose, duration=args.hover_seconds)


def land_and_stop(cf):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        cf.commander.send_notify_setpoint_stop()
        cf.high_level_commander.land(0.0, DEFAULT_TAKEOFF_TIME)
        time.sleep(DEFAULT_TAKEOFF_TIME + 0.5)
        cf.high_level_commander.stop()
    finally:
        signal.signal(signal.SIGINT, signal.SIG_DFL)


def main():
    args = parse_args()
    cflib.crtp.init_drivers()

    mocap = MocapWrapper(args.mocap_system, args.mocap_host, args.rigid_body)

    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            mocap.on_pose = lambda pose: send_extpose(cf, pose)

            time.sleep(1.0)

            try:
                run_mission(scf, args)
            except KeyboardInterrupt:
                print('\nCtrl-C received - landing')
            finally:
                land_and_stop(cf)
    finally:
        mocap.close()


if __name__ == '__main__':
    main()