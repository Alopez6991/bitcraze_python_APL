#!/usr/bin/env python3
"""
Flow + Mocap + Multi-ranger + Terrain mapping (no terrain following)
Interrupt-on-terrain-rise with fast stop:
 - Terrain rise triggers: immediate zero-velocity "brake" burst, hover 2 s, land.
 - Otherwise: complete mission.

Key changes for snappier stop:
 - Terrain monitor at 20 ms (50 Hz). Set to 10 ms (100 Hz) if your link can handle it.
 - Velocity loop at 100 Hz.
 - Immediate brake burst from the monitor callback + a second burst after loop exit.

Outputs under logs/log_YY_MM_DD_02/:
  mocap_<epoch>.csv
  cf_<epoch>.csv
  terrain_<epoch>.csv
"""
import csv
import logging
import math
import time
from pathlib import Path
from threading import Lock
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

# -----------------------------
# User params
# -----------------------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E709')

# Mocap
MOCAP_HOST = '192.168.209.81'
MOCAP_TYPE = 'optitrack'
RIGID_BODY = 'buzz'

# Flight plan
TAKEOFF_H = 1.0
TAKEOFF_VEL = 0.25
FWD_METERS = 7.0
FWD_VEL = 0.3       # m/s
HOVER_S = 2.0

# Range unit hint
RANGE_UNITS = 'mm'  # 'mm' or 'm'

# Monitor / control timing
MONITOR_PERIOD_MS = 20   # 20 ms ≈ 50 Hz (try 10 ms if the link is solid)
VEL_LOOP_HZ = 100   # velocity loop at 100 Hz
BRAKE_BURST_S = 0.30  # send 0-vel for 0.30 s on abort

# -----------------------------
# Mocap thread (observe-only)
# -----------------------------


class MocapWrapper(Thread):
    def __init__(self, body_name, host, sys_type, on_pose):
        super().__init__(daemon=True)
        self.body_name = body_name
        self.host = host
        self.sys_type = sys_type
        self.on_pose = on_pose
        self._stay_open = True

    def close(self):
        self._stay_open = False

    def run(self):
        print('Connecting to mocap system...')
        mc = motioncapture.connect(self.sys_type, {'hostname': self.host})
        print(f"Connected to mocap ({self.sys_type}) at {self.host}")
        while self._stay_open:
            mc.waitForNextFrame()
            rb = mc.rigidBodies.get(self.body_name)
            if rb is not None and self.on_pose:
                pos = rb.position
                self.on_pose([pos[0], pos[1], pos[2], rb.rotation])

# -----------------------------
# Helper: flow status
# -----------------------------


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
        print('Flow deck: DETECTED and will be USED by the Kalman estimator ✅\n')
    elif detected:
        print('Flow deck: DETECTED but Kalman is not selected — set stabilizer.estimator=2 to use it.\n')
    else:
        print('Flow deck: NOT detected — hover will drift without position/vel feedback.\n')
    return detected

# -----------------------------
# Control helpers
# -----------------------------


def send_zero_vel_burst(cf, duration_s=BRAKE_BURST_S, hz=100, lock: Lock = None):
    """Spam zero velocity-world setpoints to 'stick' the stop quickly."""
    dt = 1.0 / max(hz, 1)
    steps = int(math.ceil(duration_s / dt))
    for _ in range(steps):
        if lock:
            lock.acquire()
        try:
            cf.commander.send_velocity_world_setpoint(0.0, 0.0, 0.0, 0.0)
        finally:
            if lock:
                lock.release()
        time.sleep(dt)


def vel_forward_ignore_terrain(cf, distance_m, speed_mps, abort_flag, ctrl_lock: Lock, yaw_rate_dps=0.0, hz=VEL_LOOP_HZ):
    """
    Fly +X world at constant velocity; vz = 0 (no terrain following).
    Checks abort_flag['now'] each iteration. Sends a final brake burst on exit.
    """
    assert speed_mps > 0 and distance_m > 0
    dur = distance_m / speed_mps
    steps = int(math.ceil(dur * max(hz, 1)))
    dt = 1.0 / max(hz, 1)
    yaw_rate = math.radians(yaw_rate_dps)

    try:
        for _ in range(steps):
            if abort_flag['now']:
                break
            if ctrl_lock:
                ctrl_lock.acquire()
            try:
                cf.commander.send_velocity_world_setpoint(speed_mps, 0.0, 0.0, yaw_rate)
            finally:
                if ctrl_lock:
                    ctrl_lock.release()
            time.sleep(dt)
    finally:
        # Redundant brake on the way out
        send_zero_vel_burst(cf, duration_s=BRAKE_BURST_S, hz=hz, lock=ctrl_lock)

# -----------------------------
# Main
# -----------------------------


def main():
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Prepare logging dirs/files
    t0 = time.time()
    date_str = time.strftime('%y_%m_%d')
    out_root = Path('logs') / f"log_{date_str}_02"
    out_root.mkdir(parents=True, exist_ok=True)

    mocap_path = out_root / f"mocap_{int(t0)}.csv"
    cf_path = out_root / f"cf_{int(t0)}.csv"
    terrain_path = out_root / f"terrain_{int(t0)}.csv"

    mocap_f = open(mocap_path, 'w', newline='')
    cf_f = open(cf_path, 'w', newline='')
    terr_f = open(terrain_path, 'w', newline='')

    mocap_w = csv.writer(mocap_f)
    cf_w = csv.writer(cf_f)
    terr_w = csv.writer(terr_f)

    mocap_w.writerow(['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
    cf_w.writerow([
        't',
        'pm.vbat', 'pm.state',
        'stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z',
        'stabilizer.yaw', 'posCtl.targetX', 'locSrv.x',
        'range.front', 'range.back', 'range.left', 'range.right', 'range.up'
    ])
    terr_w.writerow(['t', 'x', 'y', 'z_est', 'range_down_m', 'ground_z'])

    write_lock = Lock()
    control_lock = Lock()   # serialize setpoint sends between loop and callback

    # CF consolidated writer
    def _write_cf_row(data: dict):
        with write_lock:
            cf_w.writerow([
                time.time() - t0,
                data.get('pm.vbat', float('nan')),
                data.get('pm.state', float('nan')),
                data.get('stateEstimate.x', float('nan')),
                data.get('stateEstimate.y', float('nan')),
                data.get('stateEstimate.z', float('nan')),
                data.get('stabilizer.yaw', float('nan')),
                data.get('posCtl.targetX', float('nan')),
                data.get('locSrv.x', float('nan')),
                data.get('range.front', float('nan')),
                data.get('range.back', float('nan')),
                data.get('range.left', float('nan')),
                data.get('range.right', float('nan')),
                data.get('range.up', float('nan')),
            ])

    # Mocap logger (no influence)
    def on_mocap_pose(pose):
        x, y, z, q = pose
        with write_lock:
            mocap_w.writerow([time.time() - t0, x, y, z, q.w, q.x, q.y, q.z])

    # Start mocap thread
    mocap_thread = MocapWrapper(RIGID_BODY, MOCAP_HOST, MOCAP_TYPE, on_mocap_pose)
    mocap_thread.start()

    mc = None
    log1 = log2 = log3 = log4 = None

    # Gates/flags shared with callbacks
    mapping_enabled = {'on': False}
    monitor_enabled = {'on': False}
    abort = {'now': False}

    # Keep latest values for mapping
    last = {'x': float('nan'), 'y': float('nan'), 'z': float('nan'), 'down_raw': None}

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # Arm
            if hasattr(cf.platform, 'send_arming_request'):
                print('Arming (CF 2.1 brushless)...')
                cf.platform.send_arming_request(True)
                time.sleep(1.0)

            # Estimator
            cf.param.set_value('stabilizer.estimator', '2')
            time.sleep(0.1)
            try:
                cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                cf.param.set_value('kalman.resetEstimation', '0')
            except Exception:
                pass

            _ = print_flow_status(scf)

            # ---- CF log blocks ----
            def safe_add(logconf, var, vtype):
                try:
                    logconf.add_variable(var, vtype)
                    return True
                except Exception:
                    print(f"[log] Skipping {var}")
                    return False

            # State/yaw @100 ms
            log1 = LogConfig(name='hostlog1', period_in_ms=100)
            for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'stabilizer.yaw'):
                safe_add(log1, v, 'float')

            # Battery & extras @200 ms
            log2 = LogConfig(name='hostlog2', period_in_ms=200)
            for v, t in (('pm.vbat', 'float'), ('pm.state', 'int8_t'),
                         ('posCtl.targetX', 'float'), ('locSrv.x', 'float')):
                safe_add(log2, v, t)

            # Multiranger (sides+up) @100 ms
            log3 = LogConfig(name='multiranger', period_in_ms=100)
            for v in ['range.front', 'range.back', 'range.left', 'range.right', 'range.up']:
                safe_add(log3, v, 'float')

            # Downward range + accelZ for monitoring/mapping @ MONITOR_PERIOD_MS
            log4 = LogConfig(name='terrain_monitor', period_in_ms=MONITOR_PERIOD_MS)
            down_candidates = [
                'range.zrange', 'range.zDistance', 'range.distance',
                'range.down', 'range.altitude'
            ]
            accz_candidates = ['acc.z', 'stabilizer.accelZ', 'imu.accZ']
            down_var = None
            accz_var = None
            for v in down_candidates:
                try:
                    log4.add_variable(v, 'float')
                    down_var = v
                    break
                except Exception:
                    pass
            for v in accz_candidates:
                try:
                    log4.add_variable(v, 'float')
                    accz_var = v
                    break
                except Exception:
                    pass

            # thresholds (units-aware)
            if RANGE_UNITS == 'mm':
                STEP_THRESH = 100.0      # mm step
                RATE_THRESH = 1000.0     # mm/s rate
                to_m = 1.0/1000.0
            else:
                STEP_THRESH = 0.05      # m
                RATE_THRESH = 0.50      # m/s
                to_m = 1.0

            from collections import deque
            r_hist = deque(maxlen=4)   # smaller history → less smoothing → faster trigger
            t_hist = deque(maxlen=4)

            def _cf_cb(ts, data, lc):
                _write_cf_row(data)
                if 'stateEstimate.x' in data:
                    last['x'] = float(data['stateEstimate.x'])
                if 'stateEstimate.y' in data:
                    last['y'] = float(data['stateEstimate.y'])
                if 'stateEstimate.z' in data:
                    last['z'] = float(data['stateEstimate.z'])

            def _rng_cb(ts, data, lc):
                _write_cf_row(data)

            # immediate brake from callback to reduce latency
            def _monitor_cb(ts, data, lc):
                if down_var and down_var in data:
                    last['down_raw'] = float(data[down_var])
                    r_hist.append(last['down_raw'])
                    t_hist.append(time.time() - t0)

                    if monitor_enabled['on'] and len(r_hist) >= 2 and not abort['now']:
                        r_now, r_prev = r_hist[-1], r_hist[-2]
                        dt = max(t_hist[-1] - t_hist[-2], 1e-3)
                        delta = r_now - r_prev
                        rate = delta / dt
                        if (delta <= -STEP_THRESH) or (abs(rate) >= RATE_THRESH):
                            abort['now'] = True
                            # IMMEDIATE brake burst to minimize overshoot
                            send_zero_vel_burst(cf, duration_s=BRAKE_BURST_S, hz=VEL_LOOP_HZ, lock=control_lock)
                            print(f"[ABORT] Terrain rise: Δr={delta*to_m:.3f} m, rate={rate*to_m:.3f} m/s")

                # mapping
                if mapping_enabled['on']:
                    x, y, z = last['x'], last['y'], last['z']
                    dr = last['down_raw']
                    if (not math.isnan(x)) and (not math.isnan(y)) and (not math.isnan(z)) and (dr is not None):
                        r_m = dr * to_m
                        ground_z = z - r_m
                        with write_lock:
                            terr_w.writerow([time.time() - t0, x, y, z, r_m, ground_z])

            # Start logs
            for lc in (log1, log2, log3):
                try:
                    cf.log.add_config(lc)
                    if lc is log3:
                        lc.data_received_cb.add_callback(_rng_cb)
                    else:
                        lc.data_received_cb.add_callback(_cf_cb)
                    lc.start()
                except Exception as e:
                    print(f"[log] Could not start {lc.name}: {e}")
            if down_var or accz_var:
                try:
                    cf.log.add_config(log4)
                    log4.data_received_cb.add_callback(_monitor_cb)
                    log4.start()
                except Exception as e:
                    print(f"[monitor] Could not start: {e}")
                    log4 = None
            else:
                print('[monitor] No down-range/accZ variables found — skipping monitor')
                log4 = None

            # ---- Flight sequence ----
            time.sleep(2.0)  # preflight logging window
            mc = MotionCommander(scf, default_height=0.3)

            print(f"Taking off to {TAKEOFF_H} m...")
            mc.take_off(height=TAKEOFF_H, velocity=TAKEOFF_VEL)
            time.sleep(1.5)

            print(f"Hover {HOVER_S:.1f} s")
            time.sleep(HOVER_S)

            # Enable during forward segment only
            mapping_enabled['on'] = True
            monitor_enabled['on'] = True

            print(f"Forward {FWD_METERS} m @ {FWD_VEL} m/s (velocity-world, vz=0, interruptible)")
            vel_forward_ignore_terrain(cf, distance_m=FWD_METERS, speed_mps=FWD_VEL,
                                       abort_flag=abort, ctrl_lock=control_lock)

            # Disable
            monitor_enabled['on'] = False
            mapping_enabled['on'] = False

            if abort['now']:
                try:
                    mc.stop()
                except Exception:
                    pass
                print('Abort: hover 2.0 s …')
                time.sleep(2.0)
                mc.forward(.15, velocity=.1)
                time.sleep(1.0)
                print('Landing (abort)…')
                mc.land(velocity=0.3)
                time.sleep(1.5)
            else:
                print(f"Hover {HOVER_S:.1f} s")
                time.sleep(HOVER_S)
                print('Stopping (hover hold)...')
                try:
                    mc.stop()
                except Exception:
                    pass
                time.sleep(0.2)
                print('Landing...')
                mc.land(velocity=0.1)
                time.sleep(1.5)

    finally:
        # Cleanup
        for lc in (log1, log2, log3, log4):
            try:
                if lc is not None:
                    lc.stop()
            except Exception:
                pass

        try:
            if mc is not None:
                mc.stop()
        except Exception:
            pass

        # Disarm (best-effort)
        try:
            with SyncCrazyflie(URI) as scf2:
                if hasattr(scf2.cf.platform, 'send_arming_request'):
                    print('Disarming...')
                    scf2.cf.platform.send_arming_request(False)
        except KeyboardInterrupt:
            print('[cleanup] Interrupted during disarm reconnect — skipping.')
        except Exception:
            pass

        # Stop mocap thread
        try:
            mocap_thread.close()
        except Exception:
            pass
        try:
            mocap_thread.join(timeout=1.0)
        except Exception:
            pass

        # Close CSVs
        mocap_f.close()
        cf_f.close()
        terr_f.close()

        print('Logs written to:')
        print(f"  {mocap_path}")
        print(f"  {cf_path}")
        print(f"  {terrain_path}")
        print('Done.')


if __name__ == '__main__':
    main()
