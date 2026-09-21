# -*- coding: utf-8 -*-
#
# Standalone "baby2" demo for the Flapper on the ONBOARD estimator only
# (optical flow / MTF-02 + IMU + downward ToF). NO motion capture: nothing is
# streamed in and nothing external is fused -- the drone flies purely on its own
# estimate. Run it with no mocap system connected.
#
# Trajectory (absolute position setpoints via the high-level commander):
#   takeoff -> hover -> forward BABY_FWD m -> hover -> fly back to takeoff -> land
#
# No turns: the nose stays facing +x the whole time, so the return leg is flown
# backward (body -x). Because this is flow-only, position drifts, so it will not
# land exactly on the takeoff point -- expected without mocap.
#
# Derived from baby.py (forward / hover / back, no rotations).
import math
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

# --- Flight geometry ---
BOX_Z = 1.0          # hover height (m)
BOX_SPEED = 0.5      # m/s cruise for each go_to leg
BOX_SETTLE = 1.5     # s to settle at each waypoint
TAKEOFF_HOVER = 2.0  # s to hover after takeoff before moving
BABY_FWD = 3.0       # forward distance (m) out and back
BABY_HOVER_S = 2.0   # s to hover at the far point

# Record to the SD card during the flight (skipped automatically if the drone
# has no USD deck).
USE_USD_LOGGING = True

# time variables
t_start = 0


def wait_for_position_estimator(scf):
    # Flow-only: these are the VELOCITY variances (position variance is unbounded
    # without an absolute reference, which is expected).
    print('Waiting for estimator (velocity variance) to converge...')
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
    """
    Fly on flow + IMU only: make sure optical flow is enabled and that NO
    external-pose (mocap) fusion is active, even if enExtPoseFuse was persisted
    on. Nothing is streamed in either way, so this is belt-and-suspenders.
    Params are wrapped in try/except so the script works on any deck set.
    """
    for name, val in (('mtf02.flowDisable', '0'), ('locSrv.enExtPoseFuse', '0')):
        try:
            cf.param.set_value(name, val)
        except Exception:
            pass


def start_onboard_logging(cf):
    if not USE_USD_LOGGING:
        return
    try:
        cf.param.set_value('usd.logging', '1')
    except Exception:
        print('  (no USD deck / usd.logging param — skipping onboard logging)')


def stop_onboard_logging(cf):
    if not USE_USD_LOGGING:
        return
    try:
        cf.param.set_value('usd.logging', '0')
    except Exception:
        pass


def run_baby2_sequence(cf):
    """
    'baby2' demo via absolute position setpoints (high-level commander go_to):

        takeoff -> hover -> forward BABY_FWD m -> hover -> back to takeoff -> land

    Body convention: forward = +x; takeoff point is the origin (0,0). No turns --
    the nose stays at yaw=0 the whole flight, so the return leg is flown backward.
    """
    global t_start
    z = BOX_Z
    fwd = BABY_FWD
    hover_s = BABY_HOVER_S
    settle = BOX_SETTLE
    move_dur = max(fwd / BOX_SPEED, 1.0)

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    print(f'Takeoff to z={z} m')
    hlc.takeoff(z, 3.0)
    time.sleep(3.0)          # let the takeoff ramp complete
    print(f'Hover at takeoff for {TAKEOFF_HOVER:.1f}s')
    hlc.go_to(0.0, 0.0, z, 0.0, TAKEOFF_HOVER)
    time.sleep(TAKEOFF_HOVER)

    # 1) forward BABY_FWD m, facing +x
    print(f'Forward {fwd:.1f} m -> ({fwd:.1f}, 0)')
    hlc.go_to(fwd, 0.0, z, 0.0, move_dur)
    time.sleep(move_dur + settle)

    # 2) hover at the far point
    print(f'Hover {hover_s:.1f}s')
    hlc.go_to(fwd, 0.0, z, 0.0, hover_s)
    time.sleep(hover_s)

    # 3) fly back to takeoff (still facing +x, so this is flown backward)
    print(f'Back {fwd:.1f} m -> (0, 0)')
    hlc.go_to(0.0, 0.0, z, 0.0, move_dur)
    time.sleep(move_dur + settle)

    print('Landing')
    stop_onboard_logging(cf)
    hlc.land(0.0, 2.5)
    time.sleep(4.0)
    hlc.stop()


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
    print(f"Connection to {link_uri} failed: {msg}")
    reconnect_and_land()


def console_incoming(console_text):
    print(console_text, end='')


def emergency_stop(cf):
    """Cut the motors NOW (Ctrl-C or any crash mid-flight). Best-effort and
    hammered several times, since a single CRTP packet can be dropped. Disarming
    is the surest kill (motors off regardless of any setpoint), so send that too.
    """
    print('\n!!! EMERGENCY STOP -- cutting motors !!!')
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


if __name__ == '__main__':
    print('initializing drivers')
    cflib.crtp.init_drivers()

    print('Connect to the Crazyflie')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.connection_lost.add_callback(connection_failed_link_error)
        cf.console.receivedChar.add_callback(console_incoming)

        print('Activating the kalman estimator')
        activate_kalman_estimator(cf)

        # Fly on flow + IMU only; make sure no mocap fusion is active.
        configure_flow_only(cf)

        reset_estimator(cf)

        # Ctrl-C (or any crash mid-flight) cuts the motors instead of leaving the
        # drone flying its last setpoint. The stop is sent while the link is still
        # open (we are still inside the SyncCrazyflie context).
        try:
            run_baby2_sequence(cf)
            time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nCtrl-C caught')
            emergency_stop(cf)
        except Exception as e:
            print(f'\nError during flight: {e!r}')
            emergency_stop(cf)
