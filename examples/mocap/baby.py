# -*- coding: utf-8 -*-
#
# Standalone "baby" demo for the Flapper on the ONBOARD estimator only
# (optical flow / MTF-02 + IMU + downward ToF). NO motion capture: nothing is
# streamed in and nothing external is fused -- the drone flies purely on its own
# estimate. Run it with no mocap system connected.
#
# Trajectory (absolute position setpoints via the high-level commander):
#   takeoff -> hover -> forward BABY_FWD m -> 360 deg turn -> hover
#           -> 180 deg turn -> forward BABY_FWD m back to takeoff -> land
#
# Because this is flow-only, position and yaw are unobservable and drift over
# time. Two full 360/180 turns accumulate heading error, so the drone will NOT
# land exactly on the takeoff point -- that is expected without mocap.
#
# Derived from nocap.py with all mocap code removed.
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
BOX_SPEED = 0.75      # m/s cruise for each go_to leg
BOX_SETTLE = 1.5     # s to settle at each waypoint
TAKEOFF_HOVER = 1.0  # s to hover after takeoff before moving
BABY_FWD = 3.0            # forward distance (m) out and back
BABY_HOVER_S = 1.0        # s hover before the turn and after the 180 turn
# Yaw is done in small absolute-angle steps (not one big sweep): flow-only, a
# fast/large rotation corrupts the optical flow and destabilises the hold.
# Smaller step / longer step time = gentler and more stable.
BABY_YAW_STEP_DEG = 30.0  # yaw increment per step (deg)
BABY_YAW_STEP_S = 1.5     # s to execute each yaw step
BABY_YAW_SETTLE = 0.5     # s to let the estimator settle between yaw steps

# Yaw command style for the turns -- flip this to A/B test the two approaches:
#   'setpoint' - stepped absolute yaw ANGLE via go_to (position held during turn)
#   'rate'     - yaw RATE via send_hover_setpoint (holds zero body velocity, so
#                there is NO absolute position hold during the turn -- the drone
#                drifts, then the next go_to recovers it)
YAW_MODE = 'rate'
BABY_YAW_RATE = 15.0      # deg/s, used only when YAW_MODE == 'rate'

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


def run_baby_sequence(cf):
    """
    'baby' demo via absolute position setpoints (high-level commander go_to):

        takeoff -> hover -> forward BABY_FWD m -> 360 deg turn in place -> hover
                -> 180 deg turn in place -> forward BABY_FWD m back to takeoff
                -> land

    Body convention: forward = +x, left = +y; takeoff point is the origin (0,0).
    After the 180 deg turn the nose faces back toward the origin, so the second
    "forward BABY_FWD m" returns the drone home.

    Rotations use ABSOLUTE yaw-angle setpoints (go_to yaw is an angle, not a
    rate) done in small BABY_YAW_STEP_DEG increments with a short settle between
    each. Flow-only, one big sweep corrupts the optical flow and wobbles the
    position hold; small steps keep the yaw rate low and let the estimator
    recover between them. Each step is < 180 deg so the planner never wraps.
    """
    global t_start
    z = BOX_Z
    fwd = BABY_FWD
    settle = BOX_SETTLE
    hover_s = BABY_HOVER_S
    move_dur = max(fwd / BOX_SPEED, 1.0)
    step = math.radians(BABY_YAW_STEP_DEG)
    setpoint_hz = 50.0
    dt = 1.0 / setpoint_hz

    cf.platform.send_arming_request(True)
    time.sleep(3.0)
    hlc = cf.high_level_commander
    start_onboard_logging(cf)
    t_start = time.time()

    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))   # -> [-pi, pi]

    def turn_setpoint(x, y, yaw, total):
        """YAW_MODE='setpoint': rotate `total` rad in small absolute-yaw ANGLE
        go_to steps (position held during the turn). Returns new wrapped yaw."""
        done = 0.0
        while done < total - 1e-3:
            d = min(step, total - done)
            yaw += d
            hlc.go_to(x, y, z, _wrap(yaw), BABY_YAW_STEP_S)
            time.sleep(BABY_YAW_STEP_S + BABY_YAW_SETTLE)
            done += d
        return _wrap(yaw)

    def turn_rate(x, y, yaw, total):
        """YAW_MODE='rate': rotate `total` rad by streaming a yaw RATE via
        send_hover_setpoint (vx=vy=0, hold altitude). No absolute position hold
        during the turn -- the drone drifts, then the next go_to recovers it.
        Hands control back to the high-level commander at the end."""
        yawrate = math.copysign(BABY_YAW_RATE, total)             # deg/s
        n = max(int(abs(total) / math.radians(BABY_YAW_RATE) * setpoint_hz), 1)
        for _ in range(n):
            cf.commander.send_hover_setpoint(0.0, 0.0, yawrate, z)
            time.sleep(dt)
        for _ in range(int(BABY_YAW_SETTLE * setpoint_hz)):       # settle at 0 rate
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
            time.sleep(dt)
        cf.commander.send_notify_setpoint_stop()                  # hand back to hlc
        return _wrap(yaw + total)

    def turn(x, y, yaw, total):
        fn = turn_rate if YAW_MODE == 'rate' else turn_setpoint
        return fn(x, y, yaw, total)

    yaw = 0.0   # track absolute heading
    print(f'YAW_MODE = {YAW_MODE!r}')

    print(f'Takeoff to z={z} m')
    hlc.takeoff(z, 3.0)
    time.sleep(3.0)          # let the takeoff ramp complete
    print(f'Hover at takeoff for {TAKEOFF_HOVER:.1f}s')
    hlc.go_to(0.0, 0.0, z, yaw, TAKEOFF_HOVER)
    time.sleep(TAKEOFF_HOVER)

    # 1) forward BABY_FWD m, facing +x
    print(f'Forward {fwd:.1f} m -> ({fwd:.1f}, 0)')
    hlc.go_to(fwd, 0.0, z, yaw, move_dur)
    time.sleep(move_dur + settle)

    # hover before the turn
    print(f'Hover {hover_s:.1f}s before the turn')
    hlc.go_to(fwd, 0.0, z, yaw, hover_s)
    time.sleep(hover_s)

    # 2) 360 deg turn in place (small absolute-yaw steps; ends back at yaw=0)
    print(f'360 deg turn in place ({BABY_YAW_STEP_DEG:.0f} deg steps)')
    yaw = turn(fwd, 0.0, yaw, 2 * math.pi)

    # 3) hover
    print(f'Hover for {settle:.1f}s')
    hlc.go_to(fwd, 0.0, z, yaw, settle)
    time.sleep(settle)

    # 4) 180 deg turn in place -> now facing back toward takeoff (yaw = pi)
    print(f'180 deg turn in place ({BABY_YAW_STEP_DEG:.0f} deg steps)')
    yaw = turn(fwd, 0.0, yaw, math.pi)

    # hover after the 180 turn (facing back toward takeoff)
    print(f'Hover {hover_s:.1f}s after the 180 turn')
    hlc.go_to(fwd, 0.0, z, yaw, hover_s)
    time.sleep(hover_s)

    # 5) forward BABY_FWD m back to takeoff (body-forward, holding yaw)
    print(f'Forward {fwd:.1f} m back to takeoff (0, 0)')
    hlc.go_to(0.0, 0.0, z, yaw, move_dur)
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
            run_baby_sequence(cf)
            time.sleep(1.0)
        except KeyboardInterrupt:
            print('\nCtrl-C caught')
            emergency_stop(cf)
        except Exception as e:
            print(f'\nError during flight: {e!r}')
            emergency_stop(cf)
