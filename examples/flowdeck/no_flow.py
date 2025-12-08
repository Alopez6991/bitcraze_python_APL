#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Takeoff with Flow → move forward 1 m → hover → disable Flow-based estimator →
try to fly straight without Flow for a time or distance → land.

Notes
- We disable Flow usage by switching the estimator to Complementary (1) mid-flight.
  This effectively removes optical-flow fusion. It may cause drift; fly with care.
- The "no-flow" leg defaults to time-based straight flight to avoid reliance on x/y.
- Hardware: Crazyflie 2.x + Flow deck + (optional) Multiranger + Crazyradio PA

"""
import logging
import math
import time
from threading import Lock
from typing import Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# ------------------- User params -------------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E709')

HEIGHT = 0.5     # takeoff height [m]
FIRST_LEG_M = 1.0     # first move with Flow [m]
HOVER_AFTER_FIRST_S = 2.0     # seconds
VEL = 0.3     # forward velocity [m/s]
NOFLOW_MODE = 'time'  # 'time' or 'distance'
NOFLOW_TIME_S = 3.0     # used when NOFLOW_MODE == 'time'
NOFLOW_DIST_M = 0.5     # used when NOFLOW_MODE == 'distance'
LOOP_HZ = 20      # command loop rate during no-flow leg
# Safety: cap total travel to at most this distance (first leg + no-flow)
TOTAL_MAX_M = 1.0
# Hold duration after turning off Flow/estimator switch (spam zero velocity)
HOLD_AFTER_SWITCH_S = 2.0
# ---------------------------------------------------

logging.basicConfig(level=logging.ERROR)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _get_param(scf, name):
    try:
        return scf.cf.param.get_value(name)
    except Exception:
        return None


def _truthy(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')


def print_flow_status(scf):
    candidates = ('deck.bcFlow2', 'deck.bcFlow', 'deck.flow', 'deck.flow2', 'deck.pmw3901')
    detected = False
    print('\n=== Sensor/estimator status ===')
    for name in candidates:
        val = _get_param(scf, name)
        if val is not None:
            print(f"{name} = {val}")
            if _truthy(val):
                detected = True
    est = _get_param(scf, 'stabilizer.estimator')
    print(f"stabilizer.estimator = {est}  (2 = Kalman, 1 = Complementary)")
    if detected and est == '2':
        print('Flow deck: detected and used by Kalman ✅\n')
    elif detected:
        print('Flow deck: detected but Kalman is not selected\n')
    else:
        print('Flow deck: NOT detected\n')


def send_zero_burst(cf, dur_s=0.3, hz=100, lock: Optional[Lock] = None):
    dt = 1.0 / max(hz, 1)
    steps = int(math.ceil(dur_s / dt))
    for _ in range(steps):
        if lock:
            lock.acquire()
        try:
            cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
        finally:
            if lock:
                lock.release()
        time.sleep(dt)


def main():
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Arm (CF 2.1 brushless)
        if hasattr(scf.cf.platform, 'send_arming_request'):
            print('Arming …')
            scf.cf.platform.send_arming_request(True)
            time.sleep(0.8)

        # Force Kalman initially (Flow)
        try:
            scf.cf.param.set_value('stabilizer.estimator', '2')
        except Exception:
            pass
        time.sleep(0.1)

        # Fresh estimator
        try:
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            scf.cf.param.set_value('kalman.resetEstimation', '0')
        except Exception:
            pass

        print_flow_status(scf)

        # Optional: log state to derive distance if NOFLOW_MODE == 'distance'
        state = {'x': None, 'y': None}
        lg_state = LogConfig('state', period_in_ms=100)
        for v in ('stateEstimate.x', 'stateEstimate.y'):
            lg_state.add_variable(v, 'float')

        def _cb_state(ts, data, lg):
            if 'stateEstimate.x' in data:
                state['x'] = float(data['stateEstimate.x'])
            if 'stateEstimate.y' in data:
                state['y'] = float(data['stateEstimate.y'])

        scf.cf.log.add_config(lg_state)
        lg_state.data_received_cb.add_callback(_cb_state)
        lg_state.start()

        control_lock = Lock()

        with MotionCommander(scf, default_height=HEIGHT) as mc:
            print(f"Taking off to {HEIGHT:.2f} m …")
            time.sleep(1.5)

            # First leg (with Flow), capped by TOTAL_MAX_M
            first_leg = min(max(0.0, FIRST_LEG_M), TOTAL_MAX_M)
            print(f"First leg: forward {first_leg:.2f} m with Flow …")
            if first_leg > 0:
                mc.forward(first_leg, velocity=max(0.1, VEL))
            send_zero_burst(scf.cf, dur_s=0.4, hz=100, lock=control_lock)

            print(f"Hovering {HOVER_AFTER_FIRST_S:.1f} s …")
            time.sleep(HOVER_AFTER_FIRST_S)

            # Try to switch OFF flow by changing estimator to complementary (1)
            print('Switching estimator to Complementary (no Flow fusion) …')
            try:
                # Zero commands briefly to reduce transients
                send_zero_burst(scf.cf, dur_s=0.2, hz=100, lock=control_lock)
                scf.cf.param.set_value('stabilizer.estimator', '1')
                time.sleep(0.2)
            except Exception as e:
                print(f"Could not switch estimator: {e}")
            print_flow_status(scf)
            # Actively hold position right after switch to avoid jumps
            print(f"Holding still for {HOLD_AFTER_SWITCH_S:.1f} s after switch …")
            send_zero_burst(scf.cf, dur_s=max(0.0, HOLD_AFTER_SWITCH_S), hz=100, lock=control_lock)

            # No-flow leg (cap by remaining distance)
            dt = 1.0 / max(LOOP_HZ, 1)
            print('No-flow leg: trying to fly straight …')
            remaining = max(0.0, TOTAL_MAX_M - first_leg)
            if remaining <= 1e-6:
                print('Total distance cap reached; skipping no-flow leg.')
            elif NOFLOW_MODE == 'distance':
                # Limit distance by remaining cap
                dist_goal = min(max(0.0, float(NOFLOW_DIST_M)), remaining)
                if dist_goal <= 0:
                    print('No remaining distance; skipping no-flow leg.')
                else:
                    # If state is not available (likely in complementary), fallback to time using cap
                    if state['x'] is None or state['y'] is None:
                        print('Position not available, falling back to time mode with distance cap …')
                        duration = dist_goal / max(0.05, float(VEL))
                        end = time.time() + duration
                        while time.time() < end:
                            control_lock.acquire()
                            try:
                                scf.cf.commander.send_velocity_world_setpoint(VEL, 0.0, 0.0, 0.0)
                            finally:
                                control_lock.release()
                            time.sleep(dt)
                    else:
                        x0, y0 = state['x'], state['y']
                        while True:
                            if state['x'] is not None and state['y'] is not None:
                                d = math.hypot(state['x'] - x0, state['y'] - y0)
                                if d >= dist_goal:
                                    break
                            control_lock.acquire()
                            try:
                                scf.cf.commander.send_velocity_world_setpoint(VEL, 0.0, 0.0, 0.0)
                            finally:
                                control_lock.release()
                            time.sleep(dt)
            else:
                # Time mode with distance cap: duration limited so V*dt <= remaining
                max_duration = remaining / max(0.05, float(VEL))
                duration = min(max(0.0, float(NOFLOW_TIME_S)), max_duration)
                if duration <= 0:
                    print('No remaining distance; skipping no-flow leg.')
                else:
                    end = time.time() + duration
                    while time.time() < end:
                        control_lock.acquire()
                        try:
                            scf.cf.commander.send_velocity_world_setpoint(VEL, 0.0, 0.0, 0.0)
                        finally:
                            control_lock.release()
                        time.sleep(dt)

            # Stop and land
            send_zero_burst(scf.cf, dur_s=0.5, hz=100, lock=control_lock)
            print('Landing …')
            mc.land(velocity=0.2)
            time.sleep(1.5)

        # Stop logs
        try:
            lg_state.stop()
        except Exception:
            pass

        # Disarm
        if hasattr(scf.cf.platform, 'send_arming_request'):
            print('Disarming …')
            scf.cf.platform.send_arming_request(False)


if __name__ == '__main__':
    main()
