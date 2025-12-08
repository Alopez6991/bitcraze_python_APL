#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal: takeoff → continuous velocity loop (Multiranger-style) → stop conditions
- Forces Kalman estimator (Flow)
- Flies forward with continuous velocity updates (like the Multiranger demo)
- Stops when:
    1) Flow-estimated traveled distance >= TARGET_DIST_M, OR
    2) Downward range detects a sudden drop (terrain rise)
- Then hovers briefly and lands.

UP sensor from Multi-ranger is NOT used.

No CSV logging. Prints only.

Hardware expected: Crazyflie 2.x + Flow deck + Multiranger deck (+ Crazyradio PA)
"""
import logging
import math
import time
from collections import deque
from threading import Lock

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

# ------------------- User params -------------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E709')

# Flight plan
HEIGHT = 1.0         # takeoff height [m]
TARGET_DIST_M = 5.0         # stop after traveling this far [m]
BASE_FWD_VEL = 0.5         # base forward speed [m/s] in world +X
LOOP_HZ = 20          # velocity loop rate (10–50 Hz is fine)
TAKEOFF_VEL = 0.3         # [m/s]
LAND_VEL = 0.3         # [m/s]
HOVER_AFTER_STOP = 2.0         # [s]

# Multiranger “push-away” behavior (like example) — NO 'up' usage
AVOID_MIN_DIST = 0.20        # [m] start pushing away if closer than this
AVOID_PUSH_VEL = 0.5         # [m/s] per-axis push

# Downward range spike (terrain rise) detection
RANGE_UNITS = 'mm'        # firmware logs often give 'mm'; if already meters use 'm'
MONITOR_MS = 20          # 20 ms ≈ 50 Hz; use 10 ms for 100 Hz if link is solid
STEP_THRESH_MM = 20.0        # spike if step <= -50 mm (range shrinks quickly)
RATE_THRESH_MM = 300.0       # or if |rate| >= 500 mm/s
MOVE_FORWARD_AFTER_EDGE_M = 0.55  # [m] forward move after spike-induced stop to avoid edge
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
    print(f"stabilizer.estimator = {est}  (2 = Kalman)")
    if detected and est == '2':
        print('Flow deck: detected and used by Kalman ✅\n')
    elif detected:
        print('Flow deck: detected but Kalman is not selected (set stabilizer.estimator=2)\n')
    else:
        print('Flow deck: NOT detected\n')


def send_zero_burst(cf, dur_s=0.3, hz=100, lock: Lock = None):
    """Spam zero-velocity setpoints for a short burst to ensure the stop 'sticks'."""
    dt = 1.0/max(hz, 1)
    steps = int(math.ceil(dur_s/dt))
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

        # Force Kalman
        scf.cf.param.set_value('stabilizer.estimator', '2')
        time.sleep(0.1)
        # Fresh estimator
        try:
            scf.cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            scf.cf.param.set_value('kalman.resetEstimation', '0')
        except Exception:
            pass

        print_flow_status(scf)

        # --- State estimate (x,y,z) for distance tracking ---
        state = {'x': None, 'y': None, 'z': None}
        lg_state = LogConfig('state', period_in_ms=100)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z'):
            lg_state.add_variable(v, 'float')

        def _cb_state(ts, data, lg):
            if 'stateEstimate.x' in data:
                state['x'] = float(data['stateEstimate.x'])
            if 'stateEstimate.y' in data:
                state['y'] = float(data['stateEstimate.y'])
            if 'stateEstimate.z' in data:
                state['z'] = float(data['stateEstimate.z'])

        scf.cf.log.add_config(lg_state)
        lg_state.data_received_cb.add_callback(_cb_state)
        lg_state.start()

        # --- Downward range monitor (spike detection) ---
        lg_down = LogConfig('down', period_in_ms=max(5, int(MONITOR_MS)))
        down_candidates = ['range.zrange', 'range.zDistance', 'range.down', 'range.altitude', 'range.distance']
        down_var = None
        for name in down_candidates:
            try:
                lg_down.add_variable(name, 'float')
                down_var = name
                break
            except Exception:
                pass
        if down_var:
            print(f"[monitor] Down-range via '{down_var}'")
        else:
            print('[monitor] No down-range var found; spike guard disabled.')

        hist_r = deque(maxlen=4)
        hist_t = deque(maxlen=4)
        spike_abort = {'now': False}
        takeoff_or_landing = {'on': False}  # ignore spikes during T/O and landing

        def _cb_down(ts, data, lg):
            if spike_abort['now'] or takeoff_or_landing['on'] or down_var not in data:
                return
            r = float(data[down_var])               # likely mm
            hist_r.append(r)
            hist_t.append(time.time())
            if len(hist_r) >= 2:
                dr = hist_r[-1] - hist_r[-2]      # mm
                dt = max(hist_t[-1] - hist_t[-2], 1e-3)
                rate = dr / dt                      # mm/s
                step_hit = (dr <= -STEP_THRESH_MM)
                rate_hit = (abs(rate) >= RATE_THRESH_MM)
                if step_hit and rate_hit and TARGET_DIST_M - state['x'] <= 1.0:
                    Test_var = TARGET_DIST_M - state['x']
                    print(f"Test_var={Test_var}")
                    spike_abort['now'] = True
                    print(f"[SPIKE] Δr={dr:.1f} {RANGE_UNITS}, rate={rate:.1f} {RANGE_UNITS}/s")

        if down_var:
            scf.cf.log.add_config(lg_down)
            lg_down.data_received_cb.add_callback(_cb_down)
            lg_down.start()

        # ---- Flight ----
        control_lock = Lock()
        # Mark that we're about to take off so the spike monitor ignores events
        takeoff_or_landing['on'] = True
        with MotionCommander(scf, default_height=HEIGHT) as mc, Multiranger(scf) as mr:
            # MotionCommander.__enter__ does the takeoff automatically; we've set
            # the takeoff flag before entering so the monitor won't trigger.
            print(f"Taking off to {HEIGHT:.2f} m …")
            # allow automatic takeoff to complete
            time.sleep(1.5)
            takeoff_or_landing['on'] = False

            # Wait a moment
            print('Stabilizing hover …')
            time.sleep(1.0)

            # Start distance tracking
            while state['x'] is None or state['y'] is None:
                time.sleep(0.05)
            x0, y0 = state['x'], state['y']
            print(f"Starting distance integration from ({x0:.2f}, {y0:.2f})")

            dt = 1.0 / max(LOOP_HZ, 1)
            # Smoothing params
            mr_hist = {
                'front': deque(maxlen=3),
                'back': deque(maxlen=3),
                'left': deque(maxlen=3),
                'right': deque(maxlen=3),
                'up': deque(maxlen=3)
            }

            def _mean(dq):
                return (sum(dq)/len(dq)) if dq and len(dq) > 0 else None

            # Velocity smoothing: exponential low-pass + rate limit
            VEL_SMOOTH_TAU = 0.12   # seconds (time constant)
            MAX_ACCEL = 2.0         # m/s^2 maximum commanded acceleration
            smoothed_vx = BASE_FWD_VEL
            smoothed_vy = 0.0
            prev_sent_vx = BASE_FWD_VEL
            prev_sent_vy = 0.0
            stop_now = False
            print('Velocity-control loop started … (Ctrl-C to abort)')
            while True:
                # Stop conditions
                if spike_abort['now']:
                    print('Spike detected → stopping …')
                    stop_now = True

                if None not in (state['x'], state['y']):
                    dx = state['x'] - x0
                    dy = state['y'] - y0
                    dist = math.hypot(dx, dy)
                    if dist >= TARGET_DIST_M:
                        print(f"Target distance reached: {dist:.2f} m")
                        stop_now = True

                if stop_now:
                    time.sleep((0.5))
                    send_zero_burst(scf.cf, dur_s=0.4, hz=100, lock=control_lock)
                    print(f"Hovering {HOVER_AFTER_STOP:.1f} s …")
                    time.sleep(HOVER_AFTER_STOP)
                    # move forward .25 to not clip edge of terrain
                    mc.start_linear_motion(0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    print('Moving forward slightly to avoid edge …')
                    mc.forward(MOVE_FORWARD_AFTER_EDGE_M, velocity=.1)
                    send_zero_burst(scf.cf, dur_s=10, hz=100, lock=control_lock)
                    time.sleep(0.5)
                    mc.start_linear_motion(0.0, 0.0, 0.0)
                    print('Landing …')
                    takeoff_or_landing['on'] = True
                    mc.land(velocity=0.2)
                    time.sleep(1.5)
                    takeoff_or_landing['on'] = False
                    break

                # --- Continuous velocity update (Multiranger-style, NO 'up') ---
                # Smooth multiranger readings via short moving average to reduce sensor jitter
                for side in ('front', 'back', 'left', 'right', 'up'):
                    val = getattr(mr, side)
                    if val is not None:
                        mr_hist[side].append(val)

                mf = _mean(mr_hist['front'])
                mb = _mean(mr_hist['back'])
                ml = _mean(mr_hist['left'])
                mrgt = _mean(mr_hist['right'])

                # Base command
                vx_des = BASE_FWD_VEL
                vy_des = 0.0

                def is_close_smooth(r):
                    return (r is not None) and (r < AVOID_MIN_DIST)

                if is_close_smooth(mf):
                    vx_des -= AVOID_PUSH_VEL
                if is_close_smooth(mb):
                    vx_des += AVOID_PUSH_VEL
                if is_close_smooth(ml):
                    vy_des -= AVOID_PUSH_VEL
                if is_close_smooth(mrgt):
                    vy_des += AVOID_PUSH_VEL

                vmax = max(BASE_FWD_VEL, AVOID_PUSH_VEL)
                vx_des = clamp(vx_des, -vmax, vmax)
                vy_des = clamp(vy_des, -vmax, vmax)

                # Exponential smoothing on desired velocity (time-constant based)
                alpha = dt / (VEL_SMOOTH_TAU + dt)
                smoothed_vx = smoothed_vx + alpha * (vx_des - smoothed_vx)
                smoothed_vy = smoothed_vy + alpha * (vy_des - smoothed_vy)

                # Rate-limit commanded velocity changes to avoid sharp jumps
                dv_max = MAX_ACCEL * dt
                dvx = clamp(smoothed_vx - prev_sent_vx, -dv_max, dv_max)
                dvy = clamp(smoothed_vy - prev_sent_vy, -dv_max, dv_max)

                vx_cmd = prev_sent_vx + dvx
                vy_cmd = prev_sent_vy + dvy

                prev_sent_vx = vx_cmd
                prev_sent_vy = vy_cmd

                # Send world-frame velocity (vz=0: no terrain following)
                control_lock.acquire()
                try:
                    scf.cf.commander.send_velocity_world_setpoint(vx_cmd, vy_cmd, 0.0, 0.0)
                finally:
                    control_lock.release()

                time.sleep(dt)

        # Stop logs cleanly
        try:
            lg_state.stop()
        except Exception:
            pass
        try:
            if down_var:
                lg_down.stop()
        except Exception:
            pass

        # Disarm
        if hasattr(scf.cf.platform, 'send_arming_request'):
            print('Disarming …')
            scf.cf.platform.send_arming_request(False)


if __name__ == '__main__':
    main()
